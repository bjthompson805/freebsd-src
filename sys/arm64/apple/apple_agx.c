/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple M5 (T8142) GPU, bring-up stage 1: identify the hardware and the
 * firmware iBoot loaded, without starting either coprocessor.
 *
 * Follows Aurora's drm/asahi G17 probe (g17/mod.rs Gpu::new, regs.rs
 * get_g17_id, fwid.rs, hw/t8142.rs, g17/pmpdash.c): hold ps_gfx on, read the
 * ID and core-mask registers, and match the loaded TEXT (the m1n1 "fw-text"
 * region, both roles back to back) against the known builds.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/callout.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>

#include <machine/atomic.h>
#include <machine/cpufunc.h>

#include <machine/bus.h>

#include <crypto/sha2/sha256.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <arm64/apple/apple_agx.h>
#include <arm64/apple/apple_agx_graph.h>
#include <arm64/apple/apple_agx_queue.h>
#include <arm64/apple/apple_agx_queue2.h>
#include <arm64/apple/apple_agx_render_install.h>
#include <arm64/apple/apple_agx_job.h>
#include <arm64/apple/apple_agx_fw.h>
#include <arm64/apple/apple_agx_render.h>
#include <arm64/apple/apple_agx_api.h>
#include <arm64/apple/apple_pmgr.h>
#include <arm64/apple/apple_rtkit.h>

/* SGX ID registers (regs.rs). */
#define	AGX_ID_VERSION		0xd04000
#define	AGX_ID_UNK08		0xd04008
#define	AGX_ID_COUNTS_1		0xd04010
#define	AGX_ID_COUNTS_2		0xd04014
#define	AGX_ID_UNK18		0xd04018
#define	AGX_ID_CLUSTERS		0xd0401c
#define	AGX_G17_PERF_CONTROL	0xe0141c
#define	AGX_G17_PERF_MAP	0xe01480
#define	AGX_CORE_MASKS		0xe01500	/* three words */

/* J813 identity (hw/t8142.rs BASE). */
#define	AGX_T8142_FAMILY	0xb
#define	AGX_T8142_VARIANT	2

/* Stage 2: M5 start control and gating (regs.rs). */
#define	AGX_M5_START_CONTROL	0xe90000
#define	AGX_G17_GATING_CONTROL	0xd06030
#define	AGX_G17_GATING_DISABLE	0x6

/* G17 endpoints and messages (g17/mod.rs). */
#define	AGX_EP_CRASHLOG		0x01
#define	AGX_EP_FIRMWARE		0x20
#define	AGX_EP_DOORBELL		0x21
#define	AGX_MSG_LOG_BUFFER	(0x9ull << 52)
#define	AGX_MSG_INITDATA	(0x81ull << 48)
#define	AGX_MSG_INITDATA_ACK	0x09
#define	AGX_MSG_INITDATA_DONE	(0x89ull << 48)
#define	AGX_MSG_POWER_ON_DONE	(0x89ull << 48)
#define	AGX_MSG_ADDR_MASK	((1ull << 44) - 1)
#define	AGX_FW_LOG_SIZE		0x8000
#define	AGX_MSG_CONTROL_NOTIFY	((0x84ull << 48) | 0x11)
#define	AGX_MSG_WORK_NOTIFY	(0x83ull << 48)	/* | pipe | priority << 2 */

/*
 * The client VM's windows (Aurora Mesa agx_m5_va.h): the driver window and
 * data heap below the canonical arena, and the USC window. The canonical
 * (0x80_..) and client-private (0x90_..) arenas are the driver's.
 */
#define	AGX_VM_SLOTS		8
#define	AGX_VM_FIRST_CONTEXT	6
#define	AGX_LOWER_TOP_SPAN	(1ul << 36)	/* one lower root entry */
#define	AGX_CLIENT_START	0x4000ul
#define	AGX_CLIENT_LOW_END	0x8000000000ul	/* AGX_M5_DATA_END */
#define	AGX_CLIENT_SHADER	0x10000000000ul	/* AGX_M5_SHADER_BASE */
#define	AGX_CLIENT_SHADER_END	(AGX_CLIENT_SHADER + 0x100000000ul)
#define	AGX_COMPACT_BASE	0x1000000000ul	/* BlockGeometry::M5_COMPACT */

#define	AGX_MSG_EVENTS		(0x42ull << 48)
/* Event ring in each role's status block (fw/initdata.rs status, channels.rs). */
#define	AGX_STATUS_EVENT_STATE	0x40
#define	AGX_STATUS_EVENT_RING	0x2c0
#define	AGX_RX_PRODUCER_OFFSET	0x20
#define	AGX_EVENT_SLOTS		256
#define	AGX_EVENT_WORDS		18
#define	AGX_EVENT_STAMP		1
#define	AGX_EVENT_RESTART	4
/* Device control (g17/fw/channels.rs). */
#define	AGX_CONTROL_SLOTS	256
#define	AGX_CONTROL_RECORD_SIZE	0x40
#define	AGX_CTL_IDLE_POWER_OFF	0x0a
#define	AGX_IDLE_KEEP_POWERED	0
#define	AGX_IDLE_ALLOW_POWER_OFF 1
/* abi CONTROL_STATE_OFFSET (12 work channels x 0x40) and OPENING_PRODUCER. */
#define	AGX_CONTROL_STATE	0x300
#define	AGX_OPENING_PRODUCER	1

/* ASCWrap v6 (soc/apple/mailbox.c). */
#define	AGX_ASC_CPU_CONTROL	0x44
#define	AGX_ASC_CPU_RUN		0x10
#define	AGX_ASC_MBOX_OFFSET	0x8000

/* UAT handoff (mmu.rs Handoff, SlotTTBS). */
#define	AGX_PPL_MAGIC		0x4b1d000000000002ull
#define	AGX_UAT_NUM_CTX		64
#define	AGX_TTBR_VALID		0x1ull
#define	AGX_TTBR_ASID_SHIFT	48
#define	AGX_KERNEL_ALIAS_CTX	1

struct agx_flush_info {
	uint64_t	state;
	uint64_t	addr;
	uint64_t	size;
};

struct agx_handoff {
	uint64_t	magic_ap;
	uint64_t	magic_fw;
	uint8_t		lock_ap;
	uint8_t		lock_fw;
	uint8_t		pad0[2];
	uint32_t	turn;
	uint32_t	cur_slot;
	uint8_t		pad1[4];
	struct agx_flush_info flush[AGX_UAT_NUM_CTX + 1];
	uint8_t		unk2;
	uint8_t		pad2[7];
	uint64_t	unk3;
};
_Static_assert(sizeof(struct agx_handoff) == 1608, "handoff layout");

struct agx_slot_ttbs {
	uint64_t	ttb0;
	uint64_t	ttb1;
};

MALLOC_DEFINE(M_AGX, "apple_agx", "Apple GPU");

static SYSCTL_NODE(_hw, OID_AUTO, apple_agx, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Apple M5 GPU");
/* 1–4: identify, start, InitData, control; 5–6: compute; 7: render install. */
static int agx_stage = 1;
SYSCTL_INT(_hw_apple_agx, OID_AUTO, stage, CTLFLAG_RDTUN, &agx_stage, 0,
    "Bring-up stage to run at attach");
/*
 * initdata.rs publish_effort: the effort word of Region C, which the
 * firmware takes as a floor for its performance state (power.rs
 * FULL_EFFORT 0x10000). Published at each kick; -1 leaves it alone.
 */
#define	AGX_REGION_C_EFFORT		0xe8
#define	AGX_REGION_C_EFFORT_OVERRIDE	0xec
static int agx_effort = 0x10000;
SYSCTL_INT(_hw_apple_agx, OID_AUTO, effort, CTLFLAG_RWTUN, &agx_effort, 0,
    "Effort published to the firmware at each kick (-1: none)");
/*
 * Renders of two contexts in flight on one pair restart the firmware, so a
 * render for another context than the pair's last waits for the pair to
 * drain. 0: only on a pair that several live roots share (more clients than
 * AGX_RENDER_PAIRS); 1: on every pair.
 */
static int agx_ctx_drain = 0;
SYSCTL_INT(_hw_apple_agx, OID_AUTO, ctx_drain, CTLFLAG_RWTUN, &agx_ctx_drain,
    0, "Drain a render pair between contexts (0 shared pairs only, 1 always)");

struct agx_fw_build {
	const char	*name;
	uint64_t	text_size;
	uint64_t	data_size;
	/* Each role's GKTS and ECAP entropy record offsets in TEXT. */
	uint32_t	entropy[2][2];
	/* Other ranges iBoot fills, zeroed before hashing. */
	uint32_t	boot_filled[8][2];
	uint8_t		text_sha256[SHA256_DIGEST_LENGTH];
};

/* hw/t8142.rs BUILD_26A428 and BUILD_26B123. */
static const struct agx_fw_build agx_t8142_builds[] = {
	{
		.name = "27.0 (26A428), G17G",
		.text_size = 0x9c000,
		.data_size = 0x184000,
		.entropy = { { 0x49d10, 0x49df3 }, { 0x997d0, 0x998b3 } },
		.boot_filled = {
			{ 0x224, 4 }, { 0x23c, 8 }, { 0x495d0, 0x740 },
			{ 0x49d10, 0x231 }, { 0x50224, 4 }, { 0x5023c, 8 },
			{ 0x99090, 0x740 }, { 0x997d0, 0x231 },
		},
		.text_sha256 = {
			0xec, 0xff, 0x40, 0xe3, 0xad, 0xb8, 0x9c, 0x90, 0x62,
			0xe9, 0x1d, 0xb6, 0x3f, 0x31, 0xba, 0xaf, 0x75, 0xeb,
			0x5d, 0x3f, 0x97, 0x22, 0xe3, 0x06, 0x95, 0x45, 0x3f,
			0xad, 0x02, 0xfc, 0xf5, 0x0c,
		},
	},
	{
		.name = "27.2 (26B123), G17G",
		.text_size = 0x9c000,
		.data_size = 0x184000,
		.entropy = { { 0x49d54, 0x49e37 }, { 0x997e4, 0x998c7 } },
		.boot_filled = {
			{ 0x224, 4 }, { 0x23c, 8 }, { 0x49614, 0x740 },
			{ 0x49d54, 0x231 }, { 0x50224, 4 }, { 0x5023c, 8 },
			{ 0x990a4, 0x740 }, { 0x997e4, 0x231 },
		},
		.text_sha256 = {
			0xe3, 0xab, 0x32, 0x7b, 0x73, 0x1b, 0xf2, 0xf1, 0x74,
			0x9b, 0x04, 0x44, 0x1f, 0x5c, 0xb5, 0xab, 0xfc, 0x23,
			0xf3, 0x41, 0xc3, 0x99, 0x72, 0x8e, 0x2c, 0xb4, 0x6f,
			0x68, 0xb1, 0x7a, 0x39, 0xb9,
		},
	},
};

struct agx_id {
	uint32_t	version;
	uint32_t	family;
	uint32_t	variant;
	uint32_t	rev;
	uint32_t	num_dies;
	uint32_t	num_clusters;
	uint32_t	num_cores;
	uint32_t	core_masks[3];
	uint32_t	active_cores;
	uint32_t	perf_control;
	uint32_t	perf_map;
};

struct agx_role {
	struct apple_agx_softc	*sc;
	const char		*name;
	uint8_t			*wrapper;
	bus_size_t		wrapper_size;
	struct apple_rtkit	*rtk;
	u_int			msgs;
	bool			init_ack;
	int			irq_rid;
	struct resource		*irq;
	void			*irq_tag;
	u_long			irqs;
	u_long			events;
	u_long			reports[2];	/* trace, statistics records */
};

/* g17/qos.rs Accounting, for the one compute owner this driver has. */
#define	AGX_QOS_QUEUES		128
#define	AGX_QOS_DATA_MASTERS	4
#define	AGX_QOS_CLOCK_HZ	24000000ul	/* hw/t8140.rs qos::CLOCK_HZ */
struct agx_qos {
	uint32_t	in_flight[AGX_QOS_QUEUES];
	uint64_t	dm_in_flight[AGX_QOS_DATA_MASTERS];
	uint8_t		dm_by_qid[AGX_QOS_QUEUES];
	uint8_t		rows[AGX_QOS_QUEUES][8];
	uint32_t	queue_submitted[AGX_QOS_QUEUES];
	uint32_t	owner_submitted[AGX_QOS_QUEUES];
	uint64_t	intervals;
	uint64_t	times_and_counts[7];
};

struct agx_client_root {
	vm_paddr_t	pa;
	uint8_t		*mesh_storage;	/* its own task mesh rendering buffer */
	uint8_t		generation;
	bool		render_mapped;
	bool		live;
	bool		published;
};

#define	AGX_QUEUE_VA(queue, name)	(AGX_Q_ ## name ## _VA + (queue)->offset)

/* Compute kicks in flight on a queue, as the render pair's slots. */
#define	AGX_COMPUTE_IN_FLIGHT	4

struct agx_compute_queue {
	uint64_t		offset;
	u_int			qid;
	u_int			qos;
	u_int			pool_row;
	u_int			users;	/* DRM queues sharing it */
	uint8_t			*sched;
	uint8_t			*work;
	uint8_t			*graph;
	uint8_t			*kick;
	uint8_t			*pool;
	uint8_t			*pool_control;
	uint8_t			*pool_state;
	uint8_t			*preempt;
	uint8_t			*usage;
	uint8_t			*support;
	uint8_t			*timestamps;
	uint8_t			*records;
	uint8_t			*cached_records;
	uint8_t			*desc;
	uint8_t			*operand;
	uint32_t		reclaimed;	/* Graph::reclaimed */
	uint32_t		ordinal;	/* kicks published */
	uint64_t		parent;	/* the last kick's timestamp */
	uint32_t		next_word;	/* WorkStates::next */
	uint32_t		word;		/* the last kick's lease */
	uint64_t		node_va;
	uint32_t		stamp;	/* the stamp that retires it */
	uint32_t		items_end;
	u_int			context;
	uint8_t			context_generation;
	uint64_t		stamp_events;
	bool			busy;
	bool			failed;
	/*
	 * Kicks published and not yet retired (apple_agx_compute_kick),
	 * oldest first. Retiring an older one, agx_retire skips the consumer
	 * cursor check (later kicks' items follow its end).
	 */
	struct agx_compute_pending {
		uint32_t	stamp;
		uint32_t	items_end;
		uint64_t	node_va;
		uint64_t	events;		/* stamp_events before it */
	}			pend[AGX_COMPUTE_IN_FLIGHT];
	u_int			pend_head;
	u_int			npend;
	bool			retire_skip_cursor;
};

/*
 * The render pair (QIDs 0 and 1) stage 7 installs. Objects the render
 * registers address from the M5 compact window
 * (0x10_0000_0000) are client-private: every client root aliases them.
 */
#define	AGX_RENDER_SLOTS	4
/* Page list, metrics, scratch, discard, TVB blocks, and three per slot. */
#define	AGX_RENDER_COMPACT_MAX	(4 + AGX_RENDER_TVB_BLOCKS + 3 * AGX_RENDER_SLOTS)
/*
 * The task mesh rendering buffer at a 64 KiB aligned base (memory.rs
 * mesh_storage). Its register counts 1 KiB units here, not memory.rs's
 * 64 KiB chunks: the payload ring's highest word tracks count x 1 KiB.
 */
#define	AGX_RENDER_MESH_STORAGE_VA	0x1008000000ul
#define	AGX_RENDER_MESH_STORAGE_SIZE	(2ul << 20)	/* 0x8000 KiB hung */
#define	AGX_RENDER_MESH_STORAGE_CHUNKS	(AGX_RENDER_MESH_STORAGE_SIZE >> 10)
struct agx_render_object {
	uint8_t		*kva;
	vm_size_t	size;
	uint64_t	va;
	int		prot;
};

/*
 * Each in-flight render has its own target, TPC, state, commands, records
 * and scene (queue/render.rs keeps RECORDS generations the same way).
 */
struct agx_render_slot {
	uint8_t		*target;
	uint8_t		*tpc;
	uint8_t		*state;
	uint8_t		*desc;		/* TA and 3D commands */
	bool		used;
	uint32_t	ordinal;
	uint32_t	stamp;		/* both stages' completion stamp */
	u_int		context;
	uint32_t	width, height;
	/* Diagnostics: what the render was built from. */
	struct agx_render_pass pass;
	struct agx_render_program programs[4];
	u_int		nprograms;
	uint64_t	node_va;	/* the work-state node */
	sbintime_t	deadline;
	int		error;		/* failed by a firmware recovery */
	void		(*done)(void *, int);
	void		*arg;
};

/*
 * One render pair: a TA and a fragment queue. Each client root renders on
 * pair (root index % AGX_RENDER_PAIRS), as Aurora gives each owner its own
 * pair: renders of two VM contexts in flight on one pair restart the
 * firmware. Pair k's firmware objects sit k * AGX_RENDER_PAIR_STRIDE above
 * pair 0's (low ones k * AGX_RENDER_PAIR_LOW_STRIDE); its client-window
 * objects keep pair 0's VAs, mapped into the roots that render on it.
 */
struct agx_render {
	struct apple_agx_softc *sc;
	u_int		index;
	uint64_t	fw_off;		/* added to pair 0's firmware VAs */
	uint64_t	low_off;	/* added to pair 0's low VAs */
	const struct agx_render_pair_install *in;	/* QIDs, slots, records */
	uint8_t		*mesh_storage;	/* the last task client's, for the probe */
	u_int		mesh_chunks;	/* its register's chunk count */
	u_int		task_context;	/* the last task render's context */
	bool		installed;
	bool		failed;
	uint8_t		*graph;
	uint8_t		*work;
	uint8_t		*kicks;
	uint8_t		*parameter;
	uint8_t		*pool_state;
	uint8_t		*support;	/* stamps, usage, timestamps */
	uint8_t		*records;	/* configurations, announcements */
	struct agx_render_object compact[AGX_RENDER_COMPACT_MAX];
	u_int		ncompact;
	struct agx_render_slot slots[AGX_RENDER_SLOTS];
	u_int		oldest;		/* slot of the oldest render in flight */
	u_int		in_flight;
	u_int		peak_in_flight;
	uint32_t	retired;
	uint64_t	ts;		/* both queues' next kick timestamp */
	uint32_t	ordinal;	/* renders published */
	uint32_t	next_word;
	uint32_t	next_id;
	uint32_t	items_end[2];	/* item-ring producers */
	uint64_t	pb_token;	/* buffer.rs: a token names one PB binding */
	uint64_t	stamp_events;	/* stamp events naming the pair's QIDs */
	u_int		pb_context;
	u_int		context;
	uint8_t		generation;
	struct task	reap_task;
	struct callout	watchdog;
	struct agx_render_slot last_good;	/* the last render retired */
};

struct apple_agx_softc;
struct agx_client_root;
static int agx_render_map_client(struct apple_agx_softc *sc,
    struct agx_client_root *root);

struct apple_agx_softc {
	device_t		dev;
	struct mtx		mtx;	/* both RTKits and the rings */
	bool			running;	/* stage 3 done */
	uint64_t		data_base;
	uint64_t		data_size;
	struct agx_role		roles[2];	/* GFX, GFX1 */
	void			*lower_root;
	struct agx_uat		uat;
	uint8_t			*graph_va[nitems(agx_graph_objects)];
	struct agx_compute_queue	queues[2];
	u_int			compute_in_flight;
	u_int			compute_peak_in_flight;
	uint8_t			*job_cdm;
	uint8_t			*job_usc;
	uint8_t			*job_out;
	struct agx_client_root	client_roots[AGX_VM_SLOTS];
	struct agx_render	renders[AGX_RENDER_PAIRS];
	/* g17/recovery.rs State: the firmware restart handshake. */
	struct task		recovery_task;
	bool			recovering;	/* a restart is being serviced */
	uint64_t		recovery_accepted;	/* next generation */
	uint64_t		recovery_request;
	int			recovery_blamed;	/* QID, or -1 */
	u_int			recoveries;
	struct agx_qos		qos;
	uint8_t			*asc;
	uint8_t			*sgx;
	bus_size_t		asc_size;
	bus_size_t		sgx_size;
	struct agx_id		id;
	const struct agx_fw_build *build;
};

static struct ofw_compat_data compat_data[] = {
	{ "apple,agx-t8142",	1 },
	{ NULL,			0 }
};

/* A memory-region of node by its memory-region-names entry. */
static int
agx_region(phandle_t node, const char *name, uint64_t *base, uint64_t *size)
{
	pcell_t *phs, reg[4];
	phandle_t rn;
	int idx, n;

	if (ofw_bus_find_string_index(node, "memory-region-names", name,
	    &idx) != 0)
		return (ENOENT);
	n = OF_getencprop_alloc_multi(node, "memory-region", sizeof(*phs),
	    (void **)&phs);
	if (n <= idx) {
		OF_prop_free(phs);
		return (ENOENT);
	}
	rn = OF_node_from_xref(phs[idx]);
	OF_prop_free(phs);
	if (OF_getencprop(rn, "reg", reg, sizeof(reg)) != sizeof(reg))
		return (EINVAL);
	*base = ((uint64_t)reg[0] << 32) | reg[1];
	*size = ((uint64_t)reg[2] << 32) | reg[3];
	return (*size != 0 ? 0 : EINVAL);
}

static int
agx_map(phandle_t node, int idx, uint8_t **va, bus_size_t *size)
{
	bus_addr_t pa;
	bus_size_t sz;
	int error;

	if ((error = ofw_reg_to_paddr(node, idx, &pa, &sz, NULL)) != 0)
		return (error);
	*va = pmap_mapdev(pa, sz);
	*size = sz;
	return (0);
}

static inline uint32_t
agx_sgx_read(struct apple_agx_softc *sc, bus_size_t off)
{

	return (*(volatile uint32_t *)(sc->sgx + off));
}

static int
agx_read_id(struct apple_agx_softc *sc)
{
	struct agx_id *id = &sc->id;
	uint32_t c1, max_mask, regs[3], mask;
	int i, j;

	id->version = agx_sgx_read(sc, AGX_ID_VERSION);
	c1 = agx_sgx_read(sc, AGX_ID_COUNTS_1);
	device_printf(sc->dev, "ID registers: %#x %#x %#x %#x %#x %#x\n",
	    id->version, agx_sgx_read(sc, AGX_ID_UNK08), c1,
	    agx_sgx_read(sc, AGX_ID_COUNTS_2),
	    agx_sgx_read(sc, AGX_ID_UNK18),
	    agx_sgx_read(sc, AGX_ID_CLUSTERS));

	id->perf_control = agx_sgx_read(sc, AGX_G17_PERF_CONTROL) & 1;
	id->perf_map = agx_sgx_read(sc, AGX_G17_PERF_MAP);
	for (i = 0; i < 3; i++)
		regs[i] = agx_sgx_read(sc, AGX_CORE_MASKS + 4 * i);
	device_printf(sc->dev, "core masks: %#x %#x %#x\n", regs[0], regs[1],
	    regs[2]);

	id->family = (id->version >> 24) & 0xff;
	id->variant = (id->version >> 16) & 0xff;
	id->rev = (id->version >> 8) & 0xff;
	id->num_dies = (c1 >> 16) & 0xf;
	id->num_clusters = ((c1 >> 8) & 0xff) * id->num_dies;
	id->num_cores = c1 & 0xff;
	if (id->num_cores == 0 || id->num_cores > 32 ||
	    id->num_cores * id->num_clusters > 3 * 32 ||
	    id->num_clusters > nitems(id->core_masks)) {
		device_printf(sc->dev, "invalid core counts (%u x %u)\n",
		    id->num_clusters, id->num_cores);
		return (ENXIO);
	}

	/* split_core_masks */
	max_mask = id->num_cores == 32 ? ~0u : (1u << id->num_cores) - 1;
	id->active_cores = 0;
	for (i = 0; i < (int)id->num_clusters; i++) {
		mask = regs[0] & max_mask;
		id->core_masks[i] = mask;
		id->active_cores += __builtin_popcount(mask);
		for (j = 0; j < 3; j++) {
			regs[j] = id->num_cores == 32 ? 0 : regs[j] >> id->num_cores;
			if (j < 2 && id->num_cores != 0)
				regs[j] |= regs[j + 1] << (32 - id->num_cores);
		}
	}
	if (regs[0] != 0 || regs[1] != 0 || regs[2] != 0) {
		device_printf(sc->dev, "leftover core mask %#x %#x %#x\n",
		    regs[0], regs[1], regs[2]);
		return (EIO);
	}

	device_printf(sc->dev,
	    "G17 family %#x variant %#x rev %#x: %u die(s), %u cluster(s) x "
	    "%u core slots, %u active; perf control %u map %#x\n",
	    id->family, id->variant, id->rev, id->num_dies, id->num_clusters,
	    id->num_cores, id->active_cores, id->perf_control, id->perf_map);
	if (id->family != AGX_T8142_FAMILY || id->variant != AGX_T8142_VARIANT ||
	    id->num_dies != 1) {
		device_printf(sc->dev, "unexpected GPU identity\n");
		return (ENXIO);
	}
	return (0);
}

static bool
agx_record_at(const uint8_t *text, size_t len, uint32_t off, const char *tag)
{
	uint32_t l;

	if (off > len || len - off < 16 || memcmp(text + off, tag, 4) != 0)
		return (false);
	memcpy(&l, text + off + 4, 4);
	return (le32toh(l) == 8);
}

/* fwid.rs digest_at with boot-filled ranges, on a private copy. */
static bool
agx_digest(const struct agx_fw_build *b, const uint8_t *text, uint8_t *copy,
    size_t len, uint8_t out[SHA256_DIGEST_LENGTH])
{
	static const char *const tags[2] = { "GKTS", "ECAP" };
	SHA256_CTX ctx;
	int r, t;

	for (r = 0; r < 2; r++)
		for (t = 0; t < 2; t++)
			if (!agx_record_at(text, len, b->entropy[r][t], tags[t]))
				return (false);
	memcpy(copy, text, len);
	for (r = 0; r < 2; r++)
		for (t = 0; t < 2; t++)
			memset(copy + b->entropy[r][t] + 8, 0, 8);
	for (r = 0; r < (int)nitems(b->boot_filled); r++) {
		uint32_t off = b->boot_filled[r][0], n = b->boot_filled[r][1];

		if (n == 0)
			continue;
		if (off > len || len - off < n)
			return (false);
		memset(copy + off, 0, n);
	}
	SHA256_Init(&ctx);
	SHA256_Update(&ctx, copy, len);
	SHA256_Final(out, &ctx);
	return (true);
}

static int
agx_identify_fw(struct apple_agx_softc *sc, phandle_t node)
{
	uint64_t tbase, tsize, dbase, dsize;
	uint8_t digest[SHA256_DIGEST_LENGTH];
	uint8_t *text, *copy;
	void *map;
	int error, i;

	if ((error = agx_region(node, "fw-text", &tbase, &tsize)) != 0 ||
	    (error = agx_region(node, "fw-data", &dbase, &dsize)) != 0) {
		device_printf(sc->dev,
		    "no fw-text/fw-data regions (m1n1 too old?)\n");
		return (error);
	}
	device_printf(sc->dev, "fw-text %#lx+%#lx, fw-data %#lx+%#lx\n",
	    tbase, tsize, dbase, dsize);
	sc->data_base = dbase;
	sc->data_size = dsize;
	if (tsize > 16 * 1024 * 1024)
		return (EINVAL);

	map = pmap_mapdev_attr(tbase, tsize, VM_MEMATTR_WRITE_COMBINING);
	text = malloc(tsize, M_AGX, M_WAITOK);
	copy = malloc(tsize, M_AGX, M_WAITOK);
	memcpy(text, map, tsize);
	pmap_unmapdev(map, tsize);

	error = ENXIO;
	for (i = 0; i < (int)nitems(agx_t8142_builds); i++) {
		const struct agx_fw_build *b = &agx_t8142_builds[i];

		if (b->text_size != tsize || b->data_size != dsize)
			continue;
		if (!agx_digest(b, text, copy, tsize, digest))
			continue;
		if (memcmp(digest, b->text_sha256, sizeof(digest)) == 0) {
			sc->build = b;
			error = 0;
			break;
		}
	}
	if (error == 0) {
		device_printf(sc->dev, "firmware %s admitted\n",
		    sc->build->name);
	} else {
		device_printf(sc->dev, "unknown GPU firmware; last digest ");
		for (i = 0; i < SHA256_DIGEST_LENGTH; i++)
			printf("%02x", digest[i]);
		printf("\n");
	}
	free(copy, M_AGX);
	free(text, M_AGX);
	return (error);
}

static inline void
agx_sgx_write(struct apple_agx_softc *sc, bus_size_t off, uint32_t v)
{

	*(volatile uint32_t *)(sc->sgx + off) = v;
}

/* Handoff::lock: Peterson lock against the firmware's half. */
static void
agx_handoff_lock(volatile struct agx_handoff *h)
{

	h->lock_ap = 1;
	dsb(sy);
	while (h->lock_fw != 0) {
		if (h->turn != 0) {
			h->lock_ap = 0;
			while (h->turn != 0)
				cpu_spinwait();
			h->lock_ap = 1;
			dsb(sy);
		}
	}
	dsb(sy);
}

static void
agx_handoff_unlock(volatile struct agx_handoff *h)
{

	h->turn = 1;
	dsb(sy);
	h->lock_ap = 0;
	dsb(sy);
}

/*
 * Uat::new for UatFirmware::PublishedRoots with UatQuirks::DEFAULT: the AP
 * half of the handoff, then context 0 (a new kernel lower root and the
 * iBoot "pagetables" upper root) and its context-1 alias.
 */
static int
agx_uat_init(struct apple_agx_softc *sc, phandle_t node)
{
	volatile struct agx_handoff *h;
	volatile struct agx_slot_ttbs *ttbs;
	uint64_t hbase, hsize, tbase, tsize, pbase, psize, ttb0, ttb1;
	int error, i;

	if ((error = agx_region(node, "handoff", &hbase, &hsize)) != 0 ||
	    (error = agx_region(node, "ttbs", &tbase, &tsize)) != 0 ||
	    (error = agx_region(node, "pagetables", &pbase, &psize)) != 0) {
		device_printf(sc->dev, "missing UAT regions\n");
		return (error);
	}
	/* secondary_root_offset 0x40000: both published roots must fit. */
	if (hsize < sizeof(*h) ||
	    tsize < AGX_UAT_NUM_CTX * sizeof(struct agx_slot_ttbs) ||
	    psize < 0x40000 + AGX_UAT_PGSZ) {
		device_printf(sc->dev, "UAT regions too small\n");
		return (EINVAL);
	}

	sc->lower_root = contigmalloc(AGX_UAT_PGSZ, M_AGX, M_WAITOK | M_ZERO,
	    0, ~(vm_paddr_t)0, AGX_UAT_PGSZ, 0);
	if (sc->lower_root == NULL)
		return (ENOMEM);
	cpu_dcache_wb_range(sc->lower_root, AGX_UAT_PGSZ);
	ttb0 = pmap_kextract((vm_offset_t)sc->lower_root);
	ttb1 = pbase;

	h = pmap_mapdev_attr(hbase, hsize, VM_MEMATTR_WRITE_BACK);
	ttbs = pmap_mapdev_attr(tbase, tsize, VM_MEMATTR_WRITE_BACK);
	device_printf(sc->dev, "handoff before: magic ap %#lx fw %#lx lock "
	    "%u/%u turn %u slot %u; ttb[0] %#lx/%#lx ttb[1] %#lx/%#lx\n",
	    h->magic_ap, h->magic_fw, h->lock_ap, h->lock_fw, h->turn,
	    h->cur_slot, ttbs[0].ttb0, ttbs[0].ttb1, ttbs[1].ttb0,
	    ttbs[1].ttb1);

	/* Handoff::init */
	h->magic_ap = AGX_PPL_MAGIC;
	h->cur_slot = 0;
	h->unk2 = 0;
	h->unk3 = 0;
	dsb(sy);
	for (i = 0; i <= AGX_UAT_NUM_CTX; i++) {
		h->flush[i].state = 0;
		h->flush[i].addr = 0;
		h->flush[i].size = 0;
	}
	dsb(sy);

	agx_handoff_lock(h);
	ttbs[0].ttb0 = ttb0 | AGX_TTBR_VALID;
	ttbs[0].ttb1 = ttb1 | AGX_TTBR_VALID;
	for (i = 1; i < AGX_UAT_NUM_CTX; i++) {
		ttbs[i].ttb0 = 0;
		ttbs[i].ttb1 = 0;
	}
	ttbs[AGX_KERNEL_ALIAS_CTX].ttb0 = ttb0 | AGX_TTBR_VALID |
	    (uint64_t)AGX_KERNEL_ALIAS_CTX << AGX_TTBR_ASID_SHIFT;
	ttbs[AGX_KERNEL_ALIAS_CTX].ttb1 = ttb1 | AGX_TTBR_VALID |
	    (uint64_t)AGX_KERNEL_ALIAS_CTX << AGX_TTBR_ASID_SHIFT;
	dsb(sy);
	agx_handoff_unlock(h);

	device_printf(sc->dev, "UAT: ttb0 %#lx (new lower root), ttb1 %#lx\n",
	    ttb0, ttb1);
	agx_uat_attach(&sc->uat, sc->dev, ttb0, ttb1, ttb1 + 0x40000);
	if ((error = agx_uat_add_region(&sc->uat, pbase, psize)) != 0 ||
	    (error = agx_region(node, "shared-l2", &pbase, &psize)) != 0 ||
	    (error = agx_uat_add_region(&sc->uat, pbase, psize)) != 0) {
		device_printf(sc->dev, "UAT: table regions: %d\n", error);
		return (error);
	}
	pmap_unmapdev(__DEVOLATILE(void *, ttbs), tsize);
	pmap_unmapdev(__DEVOLATILE(void *, h), hsize);
	return (0);
}

/* The firmware places its RTKit buffers in its data segment, by PA. */
static int
agx_buf_lookup(void *arg, uint64_t iova, size_t size, vm_paddr_t *pa)
{
	struct agx_role *r = arg;
	struct apple_agx_softc *sc = r->sc;

	if (iova < sc->data_base || size == 0 ||
	    iova - sc->data_base > sc->data_size ||
	    sc->data_size - (iova - sc->data_base) < size) {
		device_printf(sc->dev, "%s: buffer %#lx+%#zx outside fw-data\n",
		    r->name, iova, size);
		return (EINVAL);
	}
	*pa = iova;
	return (0);
}

static int
agx_buf_map(void *arg, vm_paddr_t pa, size_t size, uint64_t *iova)
{
	struct agx_role *r = arg;

	device_printf(r->sc->dev, "%s: refusing host buffer of %#zx\n",
	    r->name, size);
	return (EOPNOTSUPP);
}

static void agx_drain_events(struct agx_role *r);
static void agx_drain_reports(struct agx_role *r);

static void
agx_recv(void *arg, uint32_t ep, uint64_t msg)
{
	struct agx_role *r = arg;

	/* recv_message_early: the InitData acknowledgment, then events. */
	if (ep == AGX_EP_FIRMWARE && msg >> 48 == AGX_MSG_INITDATA_ACK) {
		r->init_ack = true;
	} else if (ep == AGX_EP_FIRMWARE && msg == AGX_MSG_EVENTS) {
		agx_drain_events(r);
		agx_drain_reports(r);
		return;
	}
	if (r->msgs++ < 16)
		device_printf(r->sc->dev, "%s: ep %#x msg %#018lx\n", r->name,
		    ep, msg);
}

static const struct apple_rtkit_ops agx_rtkit_ops = {
	.buf_map = agx_buf_map,
	.buf_lookup = agx_buf_lookup,
	.recv = agx_recv,
};

static int
agx_roles_init(struct apple_agx_softc *sc, phandle_t node)
{
	static const char *const names[2] = { "GFX", "GFX1" };
	pcell_t mboxes[2];
	phandle_t mb;
	char role[8];
	int error, i;

	if (OF_getencprop(node, "mboxes", mboxes, sizeof(mboxes)) !=
	    sizeof(mboxes))
		return (ENXIO);
	for (i = 0; i < 2; i++) {
		struct agx_role *r = &sc->roles[i];

		mb = OF_node_from_xref(mboxes[i]);
		if (OF_getprop(mb, "apple,firmware-role", role,
		    sizeof(role)) <= 0 || strcmp(role, names[i]) != 0) {
			device_printf(sc->dev, "mailbox %d is not %s\n", i,
			    names[i]);
			return (ENXIO);
		}
		if ((error = agx_map(mb, 0, &r->wrapper,
		    &r->wrapper_size)) != 0)
			return (error);
		r->sc = sc;
		r->name = names[i];
		r->rtk = apple_rtkit_create(sc->dev, (vm_offset_t)r->wrapper,
		    (vm_offset_t)r->wrapper + AGX_ASC_MBOX_OFFSET,
		    &agx_rtkit_ops, r, 0);
		if (r->rtk == NULL)
			return (ENOMEM);
	}
	return (0);
}

/* Coprocessor::boot: start the CPU, then wait for RTKit. */
static int
agx_role_boot(struct apple_agx_softc *sc, struct agx_role *r)
{
	volatile uint32_t *cpu;
	uint32_t v;
	int error;

	cpu = (volatile uint32_t *)(r->wrapper + AGX_ASC_CPU_CONTROL);
	v = *cpu;
	device_printf(sc->dev, "%s: CPU control %#x, starting\n", r->name, v);
	*cpu = v | AGX_ASC_CPU_RUN;
	error = apple_rtkit_boot(r->rtk);
	device_printf(sc->dev, "%s: RTKit boot %s (%d)\n", r->name,
	    error == 0 ? "done" : "failed", error);
	return (error);
}

static int
agx_graph_find(const char *name)
{
	u_int i;

	for (i = 0; i < nitems(agx_graph_objects); i++)
		if (strcmp(agx_graph_objects[i].name, name) == 0)
			return (i);
	return (-1);
}

/* build_m5: the 26B123 graph (apple_agx_graph.h) in the kernel window. */
static int
agx_build_graph(struct apple_agx_softc *sc)
{
	const struct agx_graph_object *o;
	vm_size_t size;
	vm_paddr_t pa;
	u_int i;
	int error;

	for (i = 0; i < nitems(agx_graph_objects); i++) {
		o = &agx_graph_objects[i];
		size = roundup2(o->size, AGX_UAT_PGSZ);
		sc->graph_va[i] = (uint8_t *)kmem_alloc_contig(size,
		    M_WAITOK | M_ZERO, 0, ~(vm_paddr_t)0, AGX_UAT_PGSZ, 0,
		    o->wb ? VM_MEMATTR_DEFAULT : VM_MEMATTR_WRITE_COMBINING);
		if (sc->graph_va[i] == NULL)
			return (ENOMEM);
	}
	for (i = 0; i < nitems(agx_graph_words); i++) {
		const struct agx_graph_word *w = &agx_graph_words[i];

		*(volatile uint32_t *)(sc->graph_va[w->obj] + w->off) = w->val;
	}
	dsb(sy);
	for (i = 0; i < nitems(agx_graph_objects); i++) {
		o = &agx_graph_objects[i];
		pa = pmap_kextract((vm_offset_t)sc->graph_va[i]);
		if ((error = agx_uat_map(&sc->uat, sc->uat.upper_root, o->va,
		    pa, roundup2(o->size, AGX_UAT_PGSZ), o->prot)) != 0) {
			device_printf(sc->dev, "graph: map %s: %d\n", o->name,
			    error);
			return (error);
		}
	}
	for (i = 0; i < nitems(agx_graph_aliases); i++) {
		const struct agx_graph_alias *a = &agx_graph_aliases[i];

		o = &agx_graph_objects[a->obj];
		pa = pmap_kextract((vm_offset_t)sc->graph_va[a->obj]);
		if ((error = agx_uat_map(&sc->uat, sc->uat.lower_root, a->va,
		    pa, roundup2(o->size, AGX_UAT_PGSZ), a->prot)) != 0) {
			device_printf(sc->dev, "graph: alias %s: %d\n", o->name,
			    error);
			return (error);
		}
	}
	for (i = 0; i < nitems(agx_graph_windows); i++) {
		const struct agx_graph_window *w = &agx_graph_windows[i];

		if ((error = agx_uat_map(&sc->uat, sc->uat.upper_root, w->va,
		    w->pa, w->size, AGX_PROT_FW_MMIO_RW)) != 0) {
			device_printf(sc->dev, "graph: window %u: %d\n",
			    w->slot, error);
			return (error);
		}
	}
	device_printf(sc->dev, "M5 firmware graph ready: %zu objects, %zu "
	    "aliases, %zu register windows, %d tables\n",
	    nitems(agx_graph_objects), nitems(agx_graph_aliases),
	    nitems(agx_graph_windows), sc->uat.ntables);
	return (0);
}

static void
agx_poll_both(struct apple_agx_softc *sc)
{

	apple_rtkit_poll(sc->roles[1].rtk);
	apple_rtkit_poll(sc->roles[0].rtk);
}

/* Coprocessor::init with InitAck::Message, then the INITDATA_DONE doorbell. */
static int
agx_role_init(struct apple_agx_softc *sc, int idx)
{
	struct agx_role *r = &sc->roles[idx];
	uint64_t root = agx_graph_roots[idx];
	int error, i;

	if ((error = apple_rtkit_start_ep(r->rtk, AGX_EP_FIRMWARE)) != 0 ||
	    (error = apple_rtkit_start_ep(r->rtk, AGX_EP_DOORBELL)) != 0)
		return (error);
	r->init_ack = false;
	if ((error = apple_rtkit_send(r->rtk, AGX_EP_FIRMWARE,
	    AGX_MSG_INITDATA | (root & AGX_MSG_ADDR_MASK))) != 0)
		return (error);
	for (i = 0; i < 2000 && !r->init_ack; i++) {	/* 2 s */
		agx_poll_both(sc);
		if (apple_rtkit_crashed(r->rtk))
			return (EIO);
		DELAY(1000);
	}
	if (!r->init_ack) {
		device_printf(sc->dev, "%s: no InitData acknowledgment\n",
		    r->name);
		return (ETIMEDOUT);
	}
	device_printf(sc->dev, "%s: InitData %#lx acknowledged after %d ms\n",
	    r->name, root, i);
	return (apple_rtkit_send(r->rtk, AGX_EP_DOORBELL,
	    AGX_MSG_INITDATA_DONE));
}

static bool
agx_opening_retired(struct apple_agx_softc *sc, int idx)
{
	volatile uint32_t *st = (volatile uint32_t *)(sc->graph_va[0] +
	    agx_graph_state_grids[idx] + AGX_CONTROL_STATE);

	return (st[0] == AGX_OPENING_PRODUCER &&
	    st[0x10 / 4] == AGX_OPENING_PRODUCER &&
	    st[0x20 / 4] == AGX_OPENING_PRODUCER);
}

static volatile uint32_t *
agx_control_state(struct apple_agx_softc *sc)
{

	return ((volatile uint32_t *)(sc->graph_va[0] +
	    agx_graph_state_grids[0] + AGX_CONTROL_STATE));
}

/* initdata.rs publish_effort: the override, then the effort, then a barrier. */
static void
agx_publish_effort(struct apple_agx_softc *sc)
{
	volatile uint32_t *effort;
	int rc;

	if (agx_effort < 0 || (rc = agx_graph_find("region_c")) < 0)
		return;
	effort = (volatile uint32_t *)(sc->graph_va[rc] + AGX_REGION_C_EFFORT);
	if (*effort == (uint32_t)agx_effort)
		return;
	effort[(AGX_REGION_C_EFFORT_OVERRIDE - AGX_REGION_C_EFFORT) / 4] = 0;
	*effort = agx_effort;
	dsb(sy);
}

/*
 * Rings::publish_control and InitData::publish_control: the record, the
 * power-assert tally, barrier, producer, barrier; then the primary's
 * doorbell. Returns the consumer value that retires the record.
 */
static int
agx_publish_control(struct apple_agx_softc *sc, const uint32_t *record,
    bool asserts_power, uint32_t *target)
{
	volatile uint32_t *st = agx_control_state(sc);
	uint8_t *bundle, *tally;
	uint32_t consumer, producer, next, v;
	int b, rc;

	if ((b = agx_graph_find("bundle")) < 0 ||
	    (rc = agx_graph_find("region_c")) < 0)
		return (EINVAL);
	dsb(sy);
	consumer = st[0];
	producer = st[0x20 / 4];
	if (consumer >= AGX_CONTROL_SLOTS || producer >= AGX_CONTROL_SLOTS)
		return (EIO);
	next = (producer + 1) & (AGX_CONTROL_SLOTS - 1);
	if (next == consumer)
		return (EAGAIN);
	bundle = sc->graph_va[b] + AGX_GRAPH_CONTROL_RING +
	    producer * AGX_CONTROL_RECORD_SIZE;
	memcpy(bundle, record, AGX_CONTROL_RECORD_SIZE);
	if (asserts_power) {
		/* An unaligned word: copy it as four bytes. */
		tally = sc->graph_va[rc] + AGX_GRAPH_POWER_TALLY;
		memcpy(&v, tally, 4);
		v = htole32(le32toh(v) + 1);
		memcpy(tally, &v, 4);
	}
	dsb(sy);
	st[0x20 / 4] = next;
	dsb(sy);
	*target = next;
	return (apple_rtkit_send(sc->roles[0].rtk, AGX_EP_DOORBELL,
	    AGX_MSG_CONTROL_NOTIFY));
}

/* Stage 4: one device-control record, IdlePowerOff(KeepPowered). */
static int
agx_stage4(struct apple_agx_softc *sc)
{
	volatile uint32_t *st = agx_control_state(sc);
	uint32_t record[AGX_CONTROL_RECORD_SIZE / 4] = {
		AGX_CTL_IDLE_POWER_OFF, AGX_IDLE_KEEP_POWERED,
	};
	uint32_t target;
	int error, i;

	device_printf(sc->dev, "control counters before: %u %u %u\n", st[0],
	    st[0x10 / 4], st[0x20 / 4]);
	if ((error = agx_publish_control(sc, record, true, &target)) != 0)
		return (error);
	for (i = 0; i <= 1000; i++) {
		agx_poll_both(sc);
		if (apple_rtkit_crashed(sc->roles[0].rtk))
			return (EIO);
		if (st[0] == target)
			break;
		DELAY(1000);
	}
	device_printf(sc->dev, "control counters after %d ms: %u %u %u "
	    "(target %u)\n", i, st[0], st[0x10 / 4], st[0x20 / 4], target);
	return (st[0] == target ? 0 : ETIMEDOUT);
}

/* A write-combined, zeroed, page-aligned object mapped at va in root. */
static int
agx_queue_object(struct apple_agx_softc *sc, const char *name, uint8_t **kva,
    vm_size_t size, vm_paddr_t root, uint64_t va, int prot)
{
	int error;

	*kva = (uint8_t *)kmem_alloc_contig(size, M_WAITOK | M_ZERO, 0,
	    ~(vm_paddr_t)0, AGX_UAT_PGSZ, 0, VM_MEMATTR_WRITE_COMBINING);
	if (*kva == NULL)
		return (ENOMEM);
	if ((error = agx_uat_map(&sc->uat, root, va,
	    pmap_kextract((vm_offset_t)*kva), size, prot)) != 0)
		device_printf(sc->dev, "queue: map %s: %d\n", name, error);
	return (error);
}

/*
 * Uat::new_execution_context: publish context id's TTBAT pair, the client
 * VM's root below and the kernel root above.
 */
static int
agx_publish_context(struct apple_agx_softc *sc, phandle_t node, u_int id,
    vm_paddr_t lower)
{
	volatile struct agx_handoff *h;
	volatile struct agx_slot_ttbs *ttbs;
	uint64_t hbase, hsize, tbase, tsize;
	int error;

	if ((error = agx_region(node, "handoff", &hbase, &hsize)) != 0 ||
	    (error = agx_region(node, "ttbs", &tbase, &tsize)) != 0)
		return (error);
	if (id >= tsize / sizeof(*ttbs) || hsize < sizeof(*h))
		return (EINVAL);
	h = pmap_mapdev_attr(hbase, hsize, VM_MEMATTR_WRITE_BACK);
	ttbs = pmap_mapdev_attr(tbase, tsize, VM_MEMATTR_WRITE_BACK);
	agx_handoff_lock(h);
	if (ttbs[id].ttb0 != 0 || ttbs[id].ttb1 != 0)
		error = EBUSY;
	else {
		ttbs[id].ttb0 = lower | AGX_TTBR_VALID |
		    (uint64_t)id << AGX_TTBR_ASID_SHIFT;
		ttbs[id].ttb1 = sc->uat.upper_root | AGX_TTBR_VALID |
		    (uint64_t)id << AGX_TTBR_ASID_SHIFT;
	}
	dsb(sy);
	agx_handoff_unlock(h);
	pmap_unmapdev(__DEVOLATILE(void *, ttbs), tsize);
	pmap_unmapdev(__DEVOLATILE(void *, h), hsize);
	agx_uat_tlbi_all();
	return (error);
}

/*
 * InitData::publish_work_at on the M5 compute ring of the queue's priority:
 * the work slot, barrier, producer, barrier.
 */
static int
agx_publish_work_slots(struct apple_agx_softc *sc, const uint8_t *first,
    const uint8_t *second)
{
	volatile uint32_t *st;
	uint8_t *ring;
	uint32_t consumer, consumer_b, producer, next, end;
	int b;

	if ((b = agx_graph_find("bundle")) < 0)
		return (EINVAL);
	st = (volatile uint32_t *)(sc->graph_va[0] + agx_graph_state_grids[0] +
	    AGX_Q_WORK_STATE);
	dsb(sy);
	consumer = st[0];
	consumer_b = st[0x10 / 4];
	producer = st[0x20 / 4];
	if (consumer >= AGX_Q_WORK_RING_SLOTS ||
	    consumer_b >= AGX_Q_WORK_RING_SLOTS ||
	    producer >= AGX_Q_WORK_RING_SLOTS)
		return (EIO);
	next = (producer + 1) % AGX_Q_WORK_RING_SLOTS;
	end = second == NULL ? next : (next + 1) % AGX_Q_WORK_RING_SLOTS;
	if (next == consumer || next == consumer_b ||
	    end == consumer || end == consumer_b)
		return (EAGAIN);
	ring = sc->graph_va[b] + AGX_Q_WORK_RING;
	memcpy(ring + producer * AGX_WORK_SLOT_SIZE, first, AGX_WORK_SLOT_SIZE);
	if (second != NULL)
		memcpy(ring + next * AGX_WORK_SLOT_SIZE, second, AGX_WORK_SLOT_SIZE);
	dsb(sy);
	st[0x20 / 4] = end;
	dsb(sy);
	return (0);
}

static int
agx_publish_work(struct apple_agx_softc *sc, const uint8_t *slot)
{
	return (agx_publish_work_slots(sc, slot, NULL));
}

/* An alias of an object at va in root (KernelObject::alias_in, map_alias). */
static int
agx_queue_alias(struct apple_agx_softc *sc, const char *name, uint8_t *kva,
    vm_size_t size, vm_paddr_t root, uint64_t va, int prot)
{
	int error;

	if ((error = agx_uat_map(&sc->uat, root, va,
	    pmap_kextract((vm_offset_t)kva), size, prot)) != 0)
		device_printf(sc->dev, "queue: alias %s: %d\n", name, error);
	return (error);
}

static void
agx_put_words(uint8_t *base, const struct agx_queue_word *w, size_t n)
{
	size_t i;

	for (i = 0; i < n; i++)
		*(volatile uint32_t *)(base + w[i].off) = w[i].val;
}

/* qos.rs time_from_host: the 24 MHz firmware domain. */
static uint64_t
agx_qos_time(void)
{
	uint64_t counter = READ_SPECIALREG(cntpct_el0);
	uint64_t frequency = READ_SPECIALREG(cntfrq_el0);

	return ((counter / frequency) * AGX_QOS_CLOCK_HZ +
	    (counter % frequency) * AGX_QOS_CLOCK_HZ / frequency);
}

/*
 * qos.rs Accounting::publish_owners for one owner: the row, scheduler and
 * share, a barrier on the first, then the submission counts and, when the
 * data master starts busy, its start time and the interval count.
 */
static int
agx_qos_publish(struct apple_agx_softc *sc, u_int qid, u_int qos, u_int dm,
    uint64_t scheduler, uint8_t class, uint32_t share)
{
	struct agx_qos *a = &sc->qos;
	uint8_t *t;
	uint64_t time;
	bool busy, first;
	int q;

	if ((q = agx_graph_find("qos")) < 0 || qid >= AGX_QOS_QUEUES ||
	    qos >= AGX_QOS_QUEUES || dm >= AGX_QOS_DATA_MASTERS ||
	    scheduler == 0)
		return (EINVAL);
	t = sc->graph_va[q];
	if (a->in_flight[qid] != 0 &&
	    (a->rows[qid][0] != qos || a->dm_by_qid[qid] != dm))
		return (EBUSY);
	busy = a->dm_in_flight[dm] == 0;
	time = busy ? agx_qos_time() : 0;
	*(volatile uint8_t *)(t + qid * 8) = qos;
	*(volatile uint8_t *)(t + qid * 8 + 1) = class;
	a->rows[qid][0] = qos;
	a->rows[qid][1] = class;
	*(volatile uint64_t *)(t + 0x800 + qos * 8) = scheduler;
	*(volatile uint32_t *)(t + 0xc00 + qos * 4) = share;
	first = a->in_flight[qid] == 0;
	if (first)
		dsb(sy);
	a->queue_submitted[qid]++;
	*(volatile uint32_t *)(t + 0x400 + qid * 4) = a->queue_submitted[qid];
	a->owner_submitted[qos]++;
	*(volatile uint32_t *)(t + 0x600 + qos * 4) = a->owner_submitted[qos];
	if (busy) {
		*(volatile uint64_t *)(t + 0xe08 + dm * 8) = time;
		a->times_and_counts[dm] = time;
	}
	a->times_and_counts[3 + dm]++;
	*(volatile uint64_t *)(t + 0xe08 + (3 + dm) * 8) =
	    a->times_and_counts[3 + dm];
	if (busy) {
		a->intervals++;
		*(volatile uint64_t *)(t + 0xe00) = a->intervals;
	}
	a->in_flight[qid]++;
	a->dm_in_flight[dm]++;
	a->dm_by_qid[qid] = dm;
	dsb(sy);
	return (0);
}

/* Graph::append_to: the item, barrier, producer. */
static int
agx_queue_append(struct agx_compute_queue *queue, uint32_t *producer,
    uint32_t consumer, uint64_t va)
{
	volatile uint32_t *cursors = (volatile uint32_t *)(queue->graph +
	    AGX_Q_CURSORS);
	volatile uint64_t *items = (volatile uint64_t *)(queue->graph +
	    AGX_Q_ITEMS);
	uint32_t next;

	if (*producer >= AGX_Q_ITEM_ENTRIES)
		return (EBUSY);
	next = (*producer + 1) % AGX_Q_ITEM_ENTRIES;
	if (next == consumer || items[*producer] != 0)
		return (EBUSY);
	items[*producer] = va;
	dsb(sy);
	cursors[(AGX_Q_PRODUCER - AGX_Q_CURSORS) / 4] = next;
	*producer = next;
	return (0);
}

/* Graph::record: clear the record's window, then write the record. */
static uint64_t
agx_queue_record(uint8_t *kva, uint64_t va, size_t off, size_t window,
    const uint8_t *rec, size_t len)
{
	memset(kva + off, 0, window);
	memcpy(kva + off, rec, len);
	dsb(sy);
	return (va + off);
}

/* qos.rs Accounting::complete for one owner. */
static int
agx_qos_complete(struct apple_agx_softc *sc, u_int qid, u_int qos, u_int dm)
{
	struct agx_qos *a = &sc->qos;
	uint8_t *t;
	int q;

	if ((q = agx_graph_find("qos")) < 0 || qid >= AGX_QOS_QUEUES ||
	    dm >= AGX_QOS_DATA_MASTERS)
		return (EINVAL);
	if (a->in_flight[qid] == 0 || a->rows[qid][0] != qos ||
	    a->dm_by_qid[qid] != dm || a->dm_in_flight[dm] == 0)
		return (EIO);
	t = sc->graph_va[q];
	if (a->in_flight[qid] == 1) {
		*(volatile uint8_t *)(t + qid * 8) = 0xff;
		*(volatile uint8_t *)(t + qid * 8 + 1) = 0;
		a->rows[qid][0] = 0xff;
		a->rows[qid][1] = 0;
	}
	a->in_flight[qid]--;
	a->dm_in_flight[dm]--;
	dsb(sy);
	return (0);
}

/* The J813 board facts the compute encoders read (agx_compute exports them). */
static const struct agx_board agx_j813_board = {
	.clusters = AGX_Q_BOARD_CLUSTERS,
	.j813_forms = AGX_Q_BOARD_J813,
	.completion_inputs = AGX_Q_BOARD_COMPLETION_INPUTS,
	.g17d_forms = AGX_Q_BOARD_G17D,
	.common26 = AGX_Q_BOARD_COMMON26,
};

#define	AGX_RECORD_SLOTS	32
#define	AGX_RECORD_STRIDE	0x1000
#define	AGX_CACHED_STRIDE	0x4000
#define	AGX_DESCRIPTORS		32
#define	AGX_DESCRIPTOR_STRIDE	0x4000	/* COMPUTE_DESCRIPTOR_SIZE */
#define	AGX_WORK_FIRST_WORD	0x40	/* context/identity.rs WorkStates */
#define	AGX_WORK_STATES		63
#define	AGX_NODE_STRIDE		0x100

/*
 * One compute kick of the queue, as publish_with_native_waits publishes it
 * on the J813, its records built by apple_agx_fw.c: the work-state node,
 * the M5 descriptor, the pool submission, QoS, the configuration and
 * descriptor items, the kick entry, the announcement, the work slot and
 * the doorbell. The previous kick has retired. cdm_va..cdm_end is the
 * job's CDM stream, terminator included. policy and control are its
 * agx_m5_compute_plan, or 0 for the fixed profile. The kick also waits for
 * the ndeps kicks in deps (agx_kick_dependency words, completion: type 1,
 * as Aurora's G18 kick entries).
 */
static int
agx_kick(struct apple_agx_softc *sc, struct agx_compute_queue *queue,
    u_int context, uint8_t generation,
    uint64_t cdm_va, uint64_t cdm_end, uint64_t policy, uint64_t control,
    const uint64_t *deps, u_int ndeps)
{
	const struct agx_board *board = &agx_j813_board;
	volatile uint32_t *cursors = (volatile uint32_t *)(queue->graph +
	    AGX_Q_CURSORS);
	struct agx_m5_compute_args d;
	struct agx_kick_args ka;
	struct agx_queue_config_args ca;
	struct agx_kick_tuple tuple;
	uint8_t config[AGX_QUEUE_CONFIG_SIZE], announce[AGX_KICK_ANNOUNCE_SIZE];
	uint8_t slot[AGX_WORK_SLOT_SIZE], parent_class;
	uint8_t node[0xc8], types[APPLE_AGX_MAX_DEPS + 1];
	uint64_t va, ts, desc_off, desc_high, desc_low, node_va, counter_off;
	uint32_t consumer, producer, ordinal, record, word, end;
	bool first;
	u_int i;
	int error;

	if (ndeps > APPLE_AGX_MAX_DEPS)
		return (EINVAL);
	ordinal = queue->ordinal;
	first = ordinal == 0;
	ts = ordinal + 1;			/* KickTimestamp::FIRST.. */
	record = ordinal % AGX_RECORD_SLOTS;
	desc_off = (ts & 0xff) % AGX_DESCRIPTORS * AGX_DESCRIPTOR_STRIDE;
	desc_high = AGX_QUEUE_VA(queue, DESC_HIGH) + desc_off;
	desc_low = AGX_QUEUE_VA(queue, DESC_LOW) + desc_off;
	/*
	 * Queues sharing a context must use distinct work keys. QIDs 2
	 * and 3 take the even and odd indices, with index 63 reserved.
	 */
	do {
		word = queue->next_word;
		queue->next_word += nitems(sc->queues);
	} while ((word & 63) == AGX_WORK_STATES);
	node_va = AGX_QUEUE_VA(queue, WORK) + (word & 63) * AGX_NODE_STRIDE;
	counter_off = AGX_UAT_PGSZ + (word & 63) * 4;

	/* WorkStorage::initialize (WorkNode::new) and add_submitted_kicks(1). */
	memset(node, 0, sizeof(node));
	va = AGX_QUEUE_VA(queue, WORK) + counter_off;
	memcpy(node, &va, 8);
	memcpy(node + 8, &word, 4);
	node[0x10] = 0x50;			/* threshold */
	memcpy(queue->work + (node_va - AGX_QUEUE_VA(queue, WORK)), node, sizeof(node));
	*(volatile uint32_t *)(queue->work + counter_off) = 0;
	dsb(sy);
	*(volatile uint32_t *)(queue->work + counter_off) = 1;
	dsb(sy);
	queue->word = word;
	queue->node_va = node_va;

	/* The M5 descriptor (fw/m5/compute.rs DescriptorArgs). */
	parent_class = agx_m5_parent_dependency(board, true);
	agx_m5_kick_tuple(board, parent_class, &tuple);
	memset(&d, 0, sizeof(d));
	d.cdm_va = cdm_va;
	d.preempt_va = AGX_QUEUE_VA(queue, CLIENT_PREEMPT);
	d.operand_va = AGX_QUEUE_VA(queue, CLIENT_OPERAND);
	d.usage_va = AGX_QUEUE_VA(queue, CLIENT_USAGE);
	d.free_list_slot = queue->qid + 1;
	d.context_id = context;
	d.work_state = word;
	d.object_ids = 0x0000000100000001ul;
	/*
	 * queue/compute.rs: a dispatch plan's pressure and flags; without one,
	 * macOS 27.0's J813 profile (one secondary and one imageblock unit
	 * beside the primary one).
	 */
	d.resource_policy = policy != 0 ? policy : 0xf500400020ul;
	d.preempt_state_offset = tuple.preempt_state_offset;
	d.operand_region_bytes = AGX_Q_OPERAND_BANK;
	d.operand_aux_offset = AGX_Q_OPERAND_TAIL;
	d.operand_bytes = AGX_Q_OPERAND_SIZE;
	d.resource_control = control != 0 ? control :
	    board->j813_forms ? 0x0104054b : 0x54b;
	d.dispatch_control = agx_m5_dispatch_control(board, tuple.dispatch_base);
	d.auxiliary_control = tuple.auxiliary_control;
	d.common26_tail = board->common26;
	d.descriptor_va = desc_low;
	d.submit_sequence = board->completion_inputs ? ordinal : 0;
	d.work_state_va = node_va;
	d.cdm_end_va = cdm_end - 4;		/* the terminator */
	d.stamp_va[0] = AGX_QUEUE_VA(queue, STAMP);
	d.stamp_va[1] = AGX_QUEUE_VA(queue, STAMP) + 8;
	d.stamp_value = agx_stamp_value(ts);
	d.qid = queue->qid;
	d.kick_timestamp = ts;
	d.completion_ordinal = agx_m5_completion_ordinal(board, ordinal);
	d.status_va[0] = AGX_QUEUE_VA(queue, SUPPORT) + 0x310;
	d.status_va[1] = AGX_QUEUE_VA(queue, SUPPORT) + 0x318;
	d.timestamp_va[0] = AGX_QUEUE_VA(queue, M5_TIMESTAMPS) + record * 16;
	d.timestamp_va[1] = d.timestamp_va[0] + 8;
	d.shared_control_va = AGX_QUEUE_VA(queue, POOL_CONTROL);
	d.usage_fw_va = AGX_QUEUE_VA(queue, USAGE);
	agx_m5_completion_forms(board, &d.completion_identity,
	    d.event_metadata);
	memset(queue->desc + desc_off, 0, AGX_DESCRIPTOR_STRIDE);
	if ((error = agx_m5_compute_descriptor(queue->desc + desc_off,
	    &d)) != 0)
		return (error);
	dsb(sy);

	/*
	 * One read of the cursors serves the publication. With kicks in
	 * flight the firmware may still be consuming their items, which end
	 * at our last producer.
	 */
	consumer = cursors[0];
	producer = cursors[(AGX_Q_PRODUCER - AGX_Q_CURSORS) / 4];
	if (cursors[0x60 / 4] != AGX_Q_ITEM_ENTRIES ||
	    producer >= AGX_Q_ITEM_ENTRIES ||
	    (queue->npend == 0 ? consumer != producer :
	    producer != queue->items_end)) {
		device_printf(sc->dev, "kick %u: cursors %u/%u\n", ordinal,
		    consumer, producer);
		return (EIO);
	}
	/* Graph::recycle: clear the consumed items. */
	while (queue->reclaimed != consumer) {
		((volatile uint64_t *)(queue->graph + AGX_Q_ITEMS))
		    [queue->reclaimed] = 0;
		queue->reclaimed = (queue->reclaimed + 1) % AGX_Q_ITEM_ENTRIES;
	}
	dsb(sy);
	/* queue_config: a seed on the first kick; updates while unpublished. */
	memset(&ca, 0, sizeof(ca));
	ca.kick_ring_va = AGX_QUEUE_VA(queue, KICK_LOW);
	ca.kick_ring_fw_va = AGX_QUEUE_VA(queue, KICK_HIGH);
	ca.qid = queue->qid;
	ca.has_completion_seed = first;
	ca.context_id = context;
	ca.free_list_va = AGX_QUEUE_VA(queue, POOL_CONTROL);
	ca.free_list_generation = generation;
	ca.free_list_slot = queue->qid + 1;
	ca.scheduler_va = AGX_QUEUE_VA(queue, SCHED);
	ca.priority = AGX_Q_PRIORITY;
	ca.policy = AGX_Q_POLICY;
	ca.qos_slot = queue->qos;
	ca.qos_update = first;
	ca.context_update = first || context != queue->context ||
	    generation != queue->context_generation;
	ca.owner_pid = 1;
	if ((error = agx_queue_config_compute(config, &ca)) != 0)
		return (error);
	va = agx_queue_record(queue->cached_records, AGX_QUEUE_VA(queue, CACHED_RECORDS),
	    record * AGX_CACHED_STRIDE, 0xc0, config, sizeof(config));

	/* ComputePool::reserve_submission, then QoS. */
	(*(volatile uint64_t *)queue->pool_state)++;
	dsb(sy);
	if ((error = agx_qos_publish(sc, queue->qid, queue->qos, AGX_DM_COMPUTE,
	    AGX_QUEUE_VA(queue, SCHED), AGX_Q_QOS_CLASS, AGX_Q_QOS_SHARE)) != 0)
		return (error);
	/* The configuration, then (J813) the descriptor. */
	if ((error = agx_queue_append(queue, &producer, consumer, va)) != 0 ||
	    (error = agx_queue_append(queue, &producer, consumer,
	    desc_high)) != 0)
		return (error);

	/* The kick entry; prepare_compute_install on the first. */
	memset(&ka, 0, sizeof(ka));
	ka.qid = queue->qid;
	ka.timestamp = ts;
	ka.parent = queue->parent;
	for (i = 0; i < ndeps; i++)
		types[i] = 1;
	types[ndeps] = parent_class;
	ka.barriers = ndeps != 0 ? deps : NULL;
	ka.nbarriers = ndeps;
	ka.dependency_types = types;
	ka.descriptor_va = desc_high;
	ka.queue_va = AGX_QUEUE_VA(queue, QGRAPH);
	ka.qos_slot = queue->qos;
	ka.qos_class = AGX_COMPUTE_QOS_CLASS;
	ka.priority = AGX_Q_PRIORITY;
	if ((error = agx_m5_kick_event_mask(board, context,
	    ka.event_mask)) != 0 ||
	    (error = agx_m5_bindings(desc_low, &d.scratch, false,
	    ka.register_arrays)) != 0)
		return (error);
	if (first)
		agx_uat_tlbi_all();
	if ((error = agx_kick_entry(queue->kick + (ts & 0xff) *
	    AGX_KICK_ENTRY_SIZE, &ka)) != 0)
		return (error);
	dsb(sy);
	queue->parent = ts;

	if ((error = agx_kick_announce(announce, queue->qid, AGX_DM_COMPUTE, ts,
	    AGX_Q_PRIORITY)) != 0)
		return (error);
	va = agx_queue_record(queue->records, AGX_QUEUE_VA(queue, RECORDS),
	    record * AGX_RECORD_STRIDE + 0x800, 0x40, announce,
	    sizeof(announce));
	if ((error = agx_queue_append(queue, &producer, consumer, va)) != 0)
		return (error);
	dsb(sy);
	end = producer;
	if ((error = agx_work_slot(slot, AGX_DM_COMPUTE, AGX_QUEUE_VA(queue, QGRAPH),
	    queue->qid, end, false)) != 0 ||
	    (error = agx_publish_work(sc, slot)) != 0) {
		device_printf(sc->dev, "kick %u: work slot: %d\n", ordinal,
		    error);
		return (error);
	}
	dsb(sy);
	queue->context = context;
	queue->context_generation = generation;
	queue->stamp = d.stamp_value;
	queue->items_end = end;
	queue->ordinal = ordinal + 1;
	agx_publish_effort(sc);
	return (apple_rtkit_send(sc->roles[0].rtk, AGX_EP_DOORBELL,
	    AGX_MSG_WORK_NOTIFY | 0x02 | AGX_Q_PRIORITY << 2));
}

/* retirement.rs stamp_covers: the queue's stamp has reached the kick's. */
static bool
agx_stamp_covers(uint32_t word, uint32_t stamp)
{
	uint32_t delta = word - stamp;

	return ((delta & 0xff) == 0 && delta < 0x80000000u);
}

/*
 * Queue::retire for the queue's last kick: wait until the stamp covers it
 * and the firmware consumed its items, then complete its QoS and release
 * its work-state node (the lease is host state only). Returns the wait in
 * ms.
 */
static int
agx_retire(struct apple_agx_softc *sc, struct agx_compute_queue *queue,
    u_int *ms, bool interrupt,
    uint64_t prior_stamp_events)
{
	volatile uint32_t *cursors = (volatile uint32_t *)(queue->graph +
	    AGX_Q_CURSORS);
	volatile uint32_t *stamp = (volatile uint32_t *)(queue->support +
	    (AGX_Q_STAMP_VA - AGX_Q_SUPPORT_VA));
	volatile uint32_t *node = (volatile uint32_t *)(queue->work +
	    (queue->node_va - AGX_QUEUE_VA(queue, WORK)));
	sbintime_t started, deadline;
	u_int i = 0;

	started = sbinuptime();
	deadline = started + 2 * SBT_1S;
	for (;;) {
		if (!interrupt)
			agx_poll_both(sc);
		if (apple_rtkit_crashed(sc->roles[0].rtk) ||
		    apple_rtkit_crashed(sc->roles[1].rtk))
			return (EIO);
		dsb(sy);
		if ((!interrupt || queue->stamp_events != prior_stamp_events) &&
		    agx_stamp_covers(*stamp, queue->stamp) &&
		    (queue->retire_skip_cursor ||
		    cursors[0] == queue->items_end) &&
		    node[AGX_Q_NODE_COMPLETED / 4] >= 1)
			break;

		if (interrupt) {
			if (sbinuptime() >= deadline)
				goto timed_out;
			msleep_sbt(&queue->stamp_events, &sc->mtx, 0, "agxstp",
			    deadline, 0, C_ABSOLUTE);
		} else {
			if (i++ == 2000)
				goto timed_out;
			DELAY(1000);
		}
	}
	*ms = interrupt ? (sbinuptime() - started) / SBT_1MS : i;
	return (agx_qos_complete(sc, queue->qid, queue->qos, AGX_DM_COMPUTE));

timed_out:
	device_printf(sc->dev, "queue %u kick %u: not retired: stamp %#x "
	    "(want %#x), consumer %u (end %u), node completed %u, stamp events "
	    "%ju -> %ju\n", queue->qid, queue->ordinal - 1, *stamp, queue->stamp,
	    cursors[0], queue->items_end, node[AGX_Q_NODE_COMPLETED / 4],
	    (uintmax_t)prior_stamp_events, (uintmax_t)queue->stamp_events);
	return (ETIMEDOUT);
}

/*
 * Stage 6: AGX_STAGE6_KICKS kicks of the job, each published after the
 * previous one retired. The job (apple_agx_job.h, Aurora Mesa) stores
 * AGX_JOB_STORE_BASE + i to out[i] for AGX_JOB_THREADS invocations; the
 * output is cleared before every kick and checked after it retires.
 */
static struct apple_agx_softc *apple_agx_sc;	/* queue installed */

device_t
apple_agx_device(void)
{

	return (apple_agx_sc != NULL ? apple_agx_sc->dev : NULL);
}

void
apple_agx_get_params(device_t dev, struct apple_agx_params *p)
{
	struct apple_agx_softc *sc = device_get_softc(dev);

	memset(p, 0, sizeof(*p));
	p->chip_id = 0x8142;
	p->generation = 17;
	p->variant = 'G';
	p->revision = sc->id.rev;
	p->num_dies = sc->id.num_dies;
	p->num_clusters = sc->id.num_clusters;
	p->num_cores_per_cluster = sc->id.num_cores;
	for (int i = 0; i < 3 && i < (int)nitems(sc->id.core_masks); i++)
		p->core_masks[i] = sc->id.core_masks[i];
	p->vm_start = AGX_CLIENT_START;
	p->vm_end = AGX_CLIENT_SHADER_END;
	p->usc_base = AGX_CLIENT_SHADER;
}

bool
apple_agx_vm_valid(device_t dev, uint64_t va, vm_size_t size)
{
	uint64_t end = va + size;

	if (end < va || ((va | size) & AGX_UAT_PGMSK) != 0 || size == 0)
		return (false);
	/* The M5 compact window holds the render pair's objects. */
	if (va < AGX_COMPACT_BASE + 0x100000000ul && end > AGX_COMPACT_BASE)
		return (false);
	return ((va >= AGX_CLIENT_START && end <= AGX_CLIENT_LOW_END) ||
	    (va >= AGX_CLIENT_SHADER && end <= AGX_CLIENT_SHADER_END));
}

static struct agx_client_root *
agx_client_root(struct apple_agx_softc *sc, u_int context)
{
	if (context < AGX_VM_FIRST_CONTEXT ||
	    context - AGX_VM_FIRST_CONTEXT >= nitems(sc->client_roots))
		return (NULL);
	return (&sc->client_roots[context - AGX_VM_FIRST_CONTEXT]);
}

/* Keep each published pair and its root pages for the life of the firmware. */
int
apple_agx_vm_create(device_t dev, u_int *context)
{
	struct apple_agx_softc *sc = device_get_softc(dev);
	struct agx_client_root *root;
	vm_paddr_t pa;
	u_int i;
	int error = ENOSPC;

	mtx_lock(&sc->mtx);
	for (i = 0; i < nitems(sc->client_roots); i++) {
		root = &sc->client_roots[i];
		if (root->live)
			continue;
		if (root->pa == 0) {
			error = agx_uat_new_client_root(&sc->uat, &pa,
			    AGX_CLIENT_LOW_END / AGX_LOWER_TOP_SPAN,
			    AGX_CLIENT_SHADER / AGX_LOWER_TOP_SPAN);
			if (error != 0)
				break;
			root->pa = pa;
		}
		if (!root->published) {
			error = agx_publish_context(sc, ofw_bus_get_node(dev),
			    AGX_VM_FIRST_CONTEXT + i, root->pa);
			if (error != 0)
				break;
			root->published = true;
			device_printf(dev, "installed client context %u\n",
			    AGX_VM_FIRST_CONTEXT + i);
		}
		if ((error = agx_render_map_client(sc, root)) != 0)
			break;
		if (++root->generation == 0)
			root->generation = 1;
		root->live = true;
		*context = AGX_VM_FIRST_CONTEXT + i;
		if (bootverbose)
			device_printf(dev, "client context %u generation %u\n",
			    *context, root->generation);
		error = 0;
		break;
	}
	mtx_unlock(&sc->mtx);
	return (error);
}

void
apple_agx_vm_destroy(device_t dev, u_int context)
{
	struct apple_agx_softc *sc = device_get_softc(dev);
	struct agx_client_root *root;

	mtx_lock(&sc->mtx);
	root = agx_client_root(sc, context);
	if (root != NULL)
		root->live = false;
	mtx_unlock(&sc->mtx);
}

int
apple_agx_vm_map(device_t dev, u_int context, uint64_t va, vm_paddr_t pa,
    vm_size_t size, u_int flags)
{
	struct apple_agx_softc *sc = device_get_softc(dev);
	struct agx_client_root *root;
	int error, prot;

	switch (flags & ~APPLE_AGX_VM_SINGLE) {
	case APPLE_AGX_VM_READ:
		prot = AGX_PROT_GPU_SHARED_RO;
		break;
	case APPLE_AGX_VM_WRITE:
		prot = AGX_PROT_GPU_SHARED_WO;
		break;
	case APPLE_AGX_VM_READ | APPLE_AGX_VM_WRITE:
		prot = AGX_PROT_GPU_SHARED_RW;
		break;
	default:
		return (EINVAL);
	}
	if (!apple_agx_vm_valid(dev, va, size) || (pa & AGX_UAT_PGMSK) != 0)
		return (EINVAL);
	mtx_lock(&sc->mtx);
	root = agx_client_root(sc, context);
	error = root == NULL || !root->live ? ENOENT :
	    agx_uat_map_pages(&sc->uat, root->pa, va, pa, size, prot,
	    (flags & APPLE_AGX_VM_SINGLE) != 0, true);
	mtx_unlock(&sc->mtx);
	return (error);
}

int
apple_agx_vm_map_list(device_t dev, u_int context, uint64_t va,
    vm_page_t *pages, u_int npages, u_int flags)
{
	struct apple_agx_softc *sc = device_get_softc(dev);
	struct agx_client_root *root;
	int error, prot;

	switch (flags) {
	case APPLE_AGX_VM_READ:
		prot = AGX_PROT_GPU_SHARED_RO;
		break;
	case APPLE_AGX_VM_WRITE:
		prot = AGX_PROT_GPU_SHARED_WO;
		break;
	case APPLE_AGX_VM_READ | APPLE_AGX_VM_WRITE:
		prot = AGX_PROT_GPU_SHARED_RW;
		break;
	default:
		return (EINVAL);
	}
	if (!apple_agx_vm_valid(dev, va, (vm_size_t)npages * AGX_UAT_PGSZ))
		return (EINVAL);
	mtx_lock(&sc->mtx);
	root = agx_client_root(sc, context);
	error = root == NULL || !root->live ? ENOENT :
	    agx_uat_map_list(&sc->uat, root->pa, va, pages, npages, prot);
	mtx_unlock(&sc->mtx);
	return (error);
}

int
apple_agx_vm_unmap(device_t dev, u_int context, uint64_t va, vm_size_t size)
{
	struct apple_agx_softc *sc = device_get_softc(dev);
	struct agx_client_root *root;
	int error;

	if (!apple_agx_vm_valid(dev, va, size))
		return (EINVAL);
	mtx_lock(&sc->mtx);
	root = agx_client_root(sc, context);
	error = root == NULL || !root->live ? ENOENT :
	    agx_uat_unmap(&sc->uat, root->pa, va, size);
	mtx_unlock(&sc->mtx);
	return (error);
}

u_int
apple_agx_queue_count(device_t dev)
{
	struct apple_agx_softc *sc = device_get_softc(dev);

	return (nitems(sc->queues));
}

int
apple_agx_queue_create(device_t dev, u_int *slot)
{
	struct apple_agx_softc *sc = device_get_softc(dev);
	u_int i;

	u_int best = nitems(sc->queues);

	/*
	 * DRM queues share the firmware compute queues: a submission runs to
	 * retirement, and a kick carries its own context. Take the least used.
	 */
	mtx_lock(&sc->mtx);
	for (i = 0; i < nitems(sc->queues); i++) {
		if (sc->queues[i].failed)
			continue;
		if (best == nitems(sc->queues) ||
		    sc->queues[i].users < sc->queues[best].users)
			best = i;
	}
	if (best == nitems(sc->queues)) {
		mtx_unlock(&sc->mtx);
		return (EIO);
	}
	sc->queues[best].users++;
	*slot = best;
	mtx_unlock(&sc->mtx);
	return (0);
}

void
apple_agx_queue_destroy(device_t dev, u_int slot)
{
	struct apple_agx_softc *sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);
	if (slot < nitems(sc->queues) && sc->queues[slot].users > 0)
		sc->queues[slot].users--;
	mtx_unlock(&sc->mtx);
}

/*
 * With the queue claimed (busy) and the lock held: retire its oldest
 * pending kicks until at most keep remain. agx_retire sleeps with the lock
 * dropped; busy keeps other kicks off the queue meanwhile. It reads the
 * queue's stamp, items end and work node, which name the newest kick: an
 * older one's stand in while it retires.
 */
static int
agx_compute_retire_pending(struct apple_agx_softc *sc,
    struct agx_compute_queue *queue, u_int keep)
{
	struct agx_compute_pending *e;
	uint32_t stamp = queue->stamp, items_end = queue->items_end;
	uint64_t node_va = queue->node_va;
	u_int ms;
	int error = 0;

	while (queue->npend > keep && error == 0) {
		e = &queue->pend[queue->pend_head];
		queue->stamp = e->stamp;
		queue->items_end = e->items_end;
		queue->node_va = e->node_va;
		queue->retire_skip_cursor = queue->npend > 1;
		error = agx_retire(sc, queue, &ms,
		    sc->roles[0].irq_tag != NULL, e->events);
		queue->pend_head = (queue->pend_head + 1) %
		    AGX_COMPUTE_IN_FLIGHT;
		queue->npend--;
		sc->compute_in_flight--;
	}
	queue->stamp = stamp;
	queue->items_end = items_end;
	queue->node_va = node_va;
	queue->retire_skip_cursor = false;
	if (error != 0) {
		queue->failed = true;
		queue->npend = 0;
		device_printf(sc->dev, "compute queue %u: kick %u: %d\n",
		    queue->qid, queue->ordinal - 1, error);
	}
	return (error);
}

/* Claim the queue: wait out another thread's kick or retirement. */
static int
agx_compute_claim(struct apple_agx_softc *sc, struct agx_compute_queue *queue)
{

	while (queue->busy && !queue->failed)
		mtx_sleep(&queue->busy, &sc->mtx, 0, "agxcq", 0);
	if (queue->failed)
		return (EIO);
	queue->busy = true;
	return (0);
}

static void
agx_compute_release(struct agx_compute_queue *queue)
{

	queue->busy = false;
	wakeup(&queue->busy);
}

int
apple_agx_compute_wait(device_t dev, u_int slot)
{
	struct apple_agx_softc *sc = device_get_softc(dev);
	struct agx_compute_queue *queue;
	int error;

	if (slot >= nitems(sc->queues))
		return (ENOENT);
	queue = &sc->queues[slot];
	mtx_lock(&sc->mtx);
	if ((error = agx_compute_claim(sc, queue)) == 0) {
		error = agx_compute_retire_pending(sc, queue, 0);
		agx_compute_release(queue);
	}
	mtx_unlock(&sc->mtx);
	return (error);
}

int
apple_agx_submit_compute(device_t dev, u_int slot, u_int context,
    uint64_t cdm_va, uint64_t cdm_end,
    const struct agx_render_program *programs, u_int nprograms,
    u_int timeout_ms)
{
	int error;

	(void)timeout_ms;	/* agx_retire waits 2 s */
	error = apple_agx_compute_kick(dev, slot, context, cdm_va, cdm_end,
	    programs, nprograms, NULL, 0, NULL);
	if (error == 0)
		error = apple_agx_compute_wait(dev, slot);
	return (error);
}

int
apple_agx_compute_kick(device_t dev, u_int slot, u_int context,
    uint64_t cdm_va, uint64_t cdm_end,
    const struct agx_render_program *programs, u_int nprograms,
    const uint64_t *deps, u_int ndeps, uint64_t *dep)
{
	struct apple_agx_softc *sc = device_get_softc(dev);
	struct agx_compute_queue *queue;
	struct agx_client_root *root;
	uint64_t events, policy = 0, control = 0;
	int error;

	if (slot >= nitems(sc->queues))
		return (ENOENT);
	if (ndeps > APPLE_AGX_MAX_DEPS)
		return (EINVAL);
	/* render_resources.rs plan_compute: every core of every cluster. */
	if (nprograms != 0 && (error = agx_m5_compute_plan(sc->id.core_masks,
	    sc->id.num_clusters, sc->id.num_cores * sc->id.num_clusters,
	    AGX_Q_PRIORITY, programs, nprograms, &policy, &control)) != 0)
		return (error);
	queue = &sc->queues[slot];
	if (cdm_end <= cdm_va || cdm_end - cdm_va < 4 || (cdm_va & 7) != 0)
		return (EINVAL);
	mtx_lock(&sc->mtx);
	if (queue->users == 0) {
		mtx_unlock(&sc->mtx);
		return (ENOENT);
	}
	/*
	 * Another thread's kick or retirement finishes first. A full queue
	 * retires its oldest kick; another context's kicks all retire, as
	 * the render pair drains between contexts.
	 */
	if ((error = agx_compute_claim(sc, queue)) != 0) {
		mtx_unlock(&sc->mtx);
		return (error);
	}
	if ((error = agx_compute_retire_pending(sc, queue,
	    context != queue->context ? 0 : AGX_COMPUTE_IN_FLIGHT - 1)) != 0)
		goto out;
	root = agx_client_root(sc, context);
	if (root == NULL || !root->live) {
		error = ENOENT;
		goto out;
	}
	events = queue->stamp_events;
	error = agx_kick(sc, queue, context, root->generation, cdm_va, cdm_end,
	    policy, control, deps, ndeps);
	if (error == 0) {
		struct agx_compute_pending *e = &queue->pend[(queue->pend_head +
		    queue->npend) % AGX_COMPUTE_IN_FLIGHT];

		e->stamp = queue->stamp;
		e->items_end = queue->items_end;
		e->node_va = queue->node_va;
		e->events = events;
		queue->npend++;
		if (dep != NULL)
			*dep = agx_kick_dependency(queue->qid, queue->ordinal);
		if (++sc->compute_in_flight > sc->compute_peak_in_flight)
			sc->compute_peak_in_flight = sc->compute_in_flight;
	} else
		queue->failed = true;
out:
	agx_compute_release(queue);
	mtx_unlock(&sc->mtx);
	if (error != 0)
		device_printf(sc->dev, "submit: compute at %#lx: %d\n", cdm_va,
		    error);
	return (error);
}

/* Stage 6 runs the job this many times, one kick at a time. */
#define	AGX_STAGE6_KICKS	16

static int
agx_stage6(struct apple_agx_softc *sc, struct agx_compute_queue *queue)
{
	volatile uint32_t *out;
	uint32_t v;
	u_int i, n, good, ms;
	int error;

#define	OBJ(name, field, size, root, va, prot)				\
	if ((error = agx_queue_object(sc, name, &field, size,	\
	    sc->uat.root, va, prot)) != 0)				\
		return (error)
#define	ALIAS(name, field, size, root, va, prot)			\
	if ((error = agx_queue_alias(sc, name, queue->field, size,		\
	    sc->uat.root, va, prot)) != 0)				\
		return (error)
	/* Graph::new and the binding's client aliases. */
	OBJ("preempt", queue->preempt, AGX_UAT_PGSZ, upper_root, AGX_QUEUE_VA(queue, PREEMPT),
	    AGX_PROT_GPU_FW_SHARED_RW);
	OBJ("usage", queue->usage, AGX_UAT_PGSZ, upper_root, AGX_QUEUE_VA(queue, USAGE),
	    AGX_PROT_GPU_FW_SHARED_RW);
	OBJ("operand", queue->operand, AGX_Q_OPERAND_SIZE, lower_root,
	    AGX_QUEUE_VA(queue, OPERAND_LOW), AGX_PROT_GPU_FW_SHARED_RW);
	OBJ("support", queue->support, AGX_UAT_PGSZ, upper_root, AGX_QUEUE_VA(queue, SUPPORT),
	    AGX_PROT_FW_SHARED_RW);
	OBJ("timestamps", queue->timestamps, AGX_Q_TIMESTAMPS_SIZE, upper_root,
	    AGX_QUEUE_VA(queue, M5_TIMESTAMPS), AGX_PROT_GPU_FW_SHARED_RW);
	OBJ("records", queue->records, AGX_Q_RECORDS_SIZE, upper_root,
	    AGX_QUEUE_VA(queue, RECORDS), AGX_PROT_GPU_FW_SHARED_RW);
	OBJ("cached records", queue->cached_records, AGX_Q_CACHED_RECORDS_SIZE,
	    upper_root, AGX_QUEUE_VA(queue, CACHED_RECORDS), AGX_PROT_GPU_FW_SHARED_RW);
	OBJ("descriptors", queue->desc, AGX_Q_DESC_RING_SIZE, upper_root,
	    AGX_QUEUE_VA(queue, DESC_HIGH), AGX_PROT_FW_SHARED_RW);
	ALIAS("descriptors", desc, AGX_Q_DESC_RING_SIZE, lower_root,
	    AGX_QUEUE_VA(queue, DESC_LOW), AGX_PROT_GPU_SHARED_RW);
	ALIAS("preempt", preempt, AGX_UAT_PGSZ, lower_root,
	    AGX_QUEUE_VA(queue, CLIENT_PREEMPT), AGX_PROT_GPU_SHARED_RW);
	ALIAS("usage", usage, AGX_UAT_PGSZ, lower_root,
	    AGX_QUEUE_VA(queue, CLIENT_USAGE), AGX_PROT_GPU_SHARED_RW);
	ALIAS("operand", operand, AGX_Q_OPERAND_SIZE, lower_root,
	    AGX_QUEUE_VA(queue, CLIENT_OPERAND), AGX_PROT_GPU_SHARED_RW);
	/* Both boot queues use the same self-test VM and buffers. */
	if (queue->offset == 0) {
		OBJ("cdm", sc->job_cdm, AGX_UAT_PGSZ, lower_root, AGX_JOB_CDM_VA,
		    AGX_PROT_GPU_SHARED_RW);
		OBJ("usc", sc->job_usc, AGX_UAT_PGSZ, lower_root,
		    AGX_JOB_PROGRAM_VA, AGX_PROT_GPU_SHARED_RW);
		OBJ("out", sc->job_out, AGX_UAT_PGSZ, lower_root, AGX_JOB_OUT_VA,
		    AGX_PROT_GPU_SHARED_RW);
	}
#undef OBJ
#undef ALIAS
	memcpy(sc->job_cdm, agx_job_cdm, sizeof(agx_job_cdm));
	memcpy(sc->job_usc, agx_job_program, sizeof(agx_job_program));
	memcpy(sc->job_usc + (AGX_JOB_SHADER_VA - AGX_JOB_PROGRAM_VA),
	    agx_job_shader, sizeof(agx_job_shader));
	dsb(sy);
	agx_uat_tlbi_all();

	queue->next_word = AGX_WORK_FIRST_WORD + queue->qid - AGX_Q_QID;
	out = (volatile uint32_t *)sc->job_out;
	for (n = 0; n < AGX_STAGE6_KICKS; n++) {
		memset(sc->job_out, 0xa5, AGX_JOB_THREADS * 4);
		dsb(sy);
		if ((error = agx_kick(sc, queue, AGX_Q_CONTEXT, 1, AGX_JOB_CDM_VA,
		    AGX_JOB_CDM_VA + sizeof(agx_job_cdm), 0, 0, NULL, 0)) != 0 ||
		    (error = agx_retire(sc, queue, &ms, false, 0)) != 0) {
			device_printf(sc->dev, "kick %u failed: %d\n", n,
			    error);
			return (error);
		}
		for (i = 0, good = 0; i < AGX_JOB_THREADS; i++)
			good += out[i] == AGX_JOB_STORE_BASE + i;
		memcpy(&v, queue->pool_control + AGX_Q_POOL_COMPLETED, 4);
		device_printf(sc->dev, "queue %u kick %u retired after %u ms: %u/%u "
		    "stores, stamp %#x, pool completed %u, events %lu\n",
		    queue->qid, n, ms,
		    good, AGX_JOB_THREADS, *(volatile uint32_t *)(queue->support +
		    (AGX_Q_STAMP_VA - AGX_Q_SUPPORT_VA)), v,
		    sc->roles[0].events);
		if (good != AGX_JOB_THREADS)
			return (EIO);
	}
	return (0);
}

/*
 * Stage 5: one compute queue, installed as install_m5 does on the J813
 * (no host-port registers): Context::new's scheduler and work pages, the
 * kick ring (kick.rs Queue::compute), the queue graph (Graph::new), the
 * execution context, then the installation's configuration record, its
 * item, its work slot and the pipe-2 doorbell. The firmware consumes the
 * item: the graph's consumer reaches AGX_Q_INSTALL_END.
 */
/*
 * Each compute queue's private-memory pool: the page directory (one entry
 * per 4 KiB page), the free list at AGX_Q_POOL_TABLES - 0x4000 (one entry per
 * 2 MiB block, 0x40 apart, bit 61 set) and the blocks after the tables, as
 * fw/m5/pool.rs encode_directory/encode_freelist (non-render blocks). The
 * fixture's pool had 8 blocks; a compute dispatch with scratch has the
 * firmware reserve up to ~20 MiB of it (pressure 437..512 pages x 10 cores)
 * and faulted at address 0 in 16 MiB. 20 blocks keep each pool below the
 * second queue's objects (AGX_Q_SECOND_OFFSET, 48 MiB at most).
 */
#define	AGX_POOL_BLOCK_BYTES	0x200000
#define	AGX_POOL_BLOCKS		20
#define	AGX_POOL_PAGES		(AGX_POOL_BLOCKS * (AGX_POOL_BLOCK_BYTES / 0x1000))
CTASSERT(AGX_Q_POOL_TABLES + AGX_POOL_BLOCKS * AGX_POOL_BLOCK_BYTES <=
    AGX_Q_SECOND_OFFSET - (AGX_Q_POOL_LOW_VA - AGX_Q_KICK_LOW_VA));
CTASSERT(AGX_POOL_PAGES * 8 <= AGX_Q_POOL_TABLES - 0x4000);

static void
agx_pool_tables(uint8_t *pool, uint64_t va)
{
	uint64_t data = va + AGX_Q_POOL_TABLES;
	u_int i;

	for (i = 0; i < AGX_POOL_PAGES; i++)
		((uint64_t *)pool)[i] = data + (uint64_t)i * 0x1000;
	for (i = 0; i < AGX_POOL_BLOCKS; i++)
		*(uint64_t *)(pool + AGX_Q_POOL_TABLES - 0x4000 + i * 0x40) =
		    (data + (uint64_t)i * AGX_POOL_BLOCK_BYTES) | 1ull << 61;
}

static int
agx_stage5(struct apple_agx_softc *sc, phandle_t node,
    struct agx_compute_queue *queue)
{
	volatile uint32_t *cursors;
	uint64_t *items;
	u_int i;
	int error;

	volatile uint64_t *row;
	int pp;

	/* ComputePool::new and InitData::install_compute_pool (J813 row). */
	if ((error = agx_queue_object(sc, "pool", &queue->pool,
	    AGX_Q_POOL_TABLES + AGX_POOL_BLOCKS * AGX_POOL_BLOCK_BYTES,
	    sc->uat.lower_root, AGX_QUEUE_VA(queue, POOL_LOW), AGX_PROT_GPU_SHARED_RW)) != 0 ||
	    (error = agx_queue_object(sc, "pool control", &queue->pool_control,
	    AGX_UAT_PGSZ, sc->uat.upper_root, AGX_QUEUE_VA(queue, POOL_CONTROL),
	    AGX_PROT_GPU_FW_SHARED_RW)) != 0 ||
	    (error = agx_queue_object(sc, "pool state", &queue->pool_state,
	    AGX_UAT_PGSZ, sc->uat.upper_root, AGX_QUEUE_VA(queue, POOL_STATE),
	    AGX_PROT_GPU_FW_SHARED_RW)) != 0)
		return (error);
	agx_pool_tables(queue->pool, AGX_QUEUE_VA(queue, POOL_LOW));
	if (queue->offset == 0)
		agx_put_words(queue->pool_control, agx_q_pool_control, nitems(agx_q_pool_control));
	else
		agx_put_words(queue->pool_control, agx_q2_pool_control, nitems(agx_q2_pool_control));
	/* The fixture's page counts (4096 pages, 16 MiB) become the pool's. */
	*(volatile uint32_t *)(queue->pool_control + 0x24) = AGX_POOL_PAGES;
	*(volatile uint32_t *)(queue->pool_control + 0x2c) = AGX_POOL_PAGES;
	dsb(sy);
	if ((pp = agx_graph_find("page_pool")) < 0)
		return (EINVAL);
	row = (volatile uint64_t *)(sc->graph_va[pp] + queue->pool_row);
	for (i = 0; i < nitems(agx_q_pool_row); i++)
		row[i] = queue->offset == 0 ? agx_q_pool_row[i] : agx_q2_pool_row[i];
	row[2] = (row[2] & ~0xffffffffull) | AGX_POOL_PAGES;
	dsb(sy);

	if ((error = agx_queue_object(sc, "scheduler", &queue->sched,
	    AGX_UAT_PGSZ, sc->uat.upper_root, AGX_QUEUE_VA(queue, SCHED),
	    AGX_PROT_FW_SHARED_RW)) != 0 ||
	    (error = agx_queue_object(sc, "work", &queue->work, AGX_Q_WORK_SIZE,
	    sc->uat.upper_root, AGX_QUEUE_VA(queue, WORK), AGX_PROT_FW_SHARED_RW)) != 0 ||
	    (error = agx_queue_object(sc, "graph", &queue->graph,
	    AGX_Q_QGRAPH_SIZE, sc->uat.upper_root, AGX_QUEUE_VA(queue, QGRAPH),
	    AGX_PROT_GPU_FW_SHARED_RW)) != 0 ||
	    (error = agx_queue_object(sc, "kick", &queue->kick, AGX_Q_KICK_SIZE,
	    sc->uat.lower_root, AGX_QUEUE_VA(queue, KICK_LOW),
	    AGX_PROT_GPU_SHARED_RW)) != 0)
		return (error);
	if ((error = agx_uat_map(&sc->uat, sc->uat.upper_root,
	    AGX_QUEUE_VA(queue, KICK_HIGH), pmap_kextract((vm_offset_t)queue->kick),
	    AGX_Q_KICK_SIZE, AGX_PROT_FW_SHARED_RW)) != 0) {
		device_printf(sc->dev, "queue: map kick alias: %d\n", error);
		return (error);
	}
	agx_put_words(queue->sched, agx_q_sched, nitems(agx_q_sched));
	memcpy(queue->work + (AGX_Q_WORK_HEAD_VA - AGX_Q_WORK_VA),
	    queue->offset == 0 ? agx_q_work_head : agx_q2_work_head,
	    sizeof(agx_q_work_head));
	if (queue->offset == 0)
		agx_put_words(queue->graph, agx_q_graph, nitems(agx_q_graph));
	else
		agx_put_words(queue->graph, agx_q2_graph, nitems(agx_q2_graph));
	dsb(sy);
	if (queue->offset == 0 &&
	    (error = agx_publish_context(sc, node, AGX_Q_CONTEXT,
	    sc->uat.lower_root)) != 0) {
		device_printf(sc->dev, "queue: context %u: %d\n",
		    AGX_Q_CONTEXT, error);
		return (error);
	}

	/* Graph::record(None, CONFIG) and Graph::append at item 0. */
	cursors = (volatile uint32_t *)(queue->graph + AGX_Q_CURSORS);
	items = (uint64_t *)(queue->graph + AGX_Q_ITEMS);
	if (cursors[0] != 0 || cursors[(AGX_Q_PRODUCER - AGX_Q_CURSORS) / 4] != 0 ||
	    cursors[0x60 / 4] != AGX_Q_ITEM_ENTRIES || items[0] != 0)
		return (EIO);
	memcpy(queue->graph + AGX_Q_CONFIG,
	    queue->offset == 0 ? agx_q_install_config : agx_q2_install_config,
	    sizeof(agx_q_install_config));
	dsb(sy);
	*(volatile uint64_t *)&items[0] = AGX_QUEUE_VA(queue, QGRAPH) + AGX_Q_CONFIG;
	dsb(sy);
	cursors[(AGX_Q_PRODUCER - AGX_Q_CURSORS) / 4] = AGX_Q_INSTALL_END;
	dsb(sy);
	if ((error = agx_publish_work(sc, queue->offset == 0 ?
	    agx_q_install_slot : agx_q2_install_slot)) != 0) {
		device_printf(sc->dev, "queue: work slot: %d\n", error);
		return (error);
	}
	if ((error = apple_rtkit_send(sc->roles[0].rtk, AGX_EP_DOORBELL,
	    AGX_MSG_WORK_NOTIFY | 0x02 | AGX_Q_PRIORITY << 2)) != 0)
		return (error);
	for (i = 0; i <= 1000; i++) {
		agx_poll_both(sc);
		if (apple_rtkit_crashed(sc->roles[0].rtk) ||
		    apple_rtkit_crashed(sc->roles[1].rtk))
			return (EIO);
		if (cursors[0] == AGX_Q_INSTALL_END)
			break;
		DELAY(1000);
	}
	device_printf(sc->dev, "queue %u: consumer %u producer %u after %u ms "
	    "(install end %u); work ring %u/%u\n", queue->qid, cursors[0],
	    cursors[(AGX_Q_PRODUCER - AGX_Q_CURSORS) / 4], i, AGX_Q_INSTALL_END,
	    ((volatile uint32_t *)(sc->graph_va[0] + agx_graph_state_grids[0] +
	    AGX_Q_WORK_STATE))[0],
	    ((volatile uint32_t *)(sc->graph_va[0] + agx_graph_state_grids[0] +
	    AGX_Q_WORK_STATE))[0x20 / 4]);
	return (cursors[0] == AGX_Q_INSTALL_END ? 0 : ETIMEDOUT);
}

/*
 * Record a compact-window object of a render pair. Pair 0's are also aliased
 * into the boot root; the other pairs' share those VAs and live only in the
 * client roots that render on them.
 */
static int
agx_render_alias(struct agx_render *r, const char *name, uint8_t *kva,
    vm_size_t size, uint64_t va, int prot)
{

	if (r->ncompact == nitems(r->compact))
		return (ENOSPC);
	r->compact[r->ncompact++] = (struct agx_render_object){ kva, size, va,
	    prot };
	if (r->index != 0)
		return (0);
	return (agx_queue_alias(r->sc, name, kva, size, r->sc->uat.lower_root,
	    va, prot));
}

static int
agx_render_compact(struct agx_render *r, const char *name, uint8_t **kva,
    vm_size_t size, uint64_t va)
{

	*kva = (uint8_t *)kmem_alloc_contig(size, M_WAITOK | M_ZERO, 0,
	    ~(vm_paddr_t)0, AGX_UAT_PGSZ, 0, VM_MEMATTR_WRITE_COMBINING);
	if (*kva == NULL)
		return (ENOMEM);
	return (agx_render_alias(r, name, *kva, size, va,
	    AGX_PROT_GPU_SHARED_RW));
}

static struct agx_render *agx_root_pair(struct apple_agx_softc *sc,
    struct agx_client_root *root);

/* Whether more than one live client root renders on pair r. */
static bool
agx_pair_shared(struct apple_agx_softc *sc, struct agx_render *r)
{
	u_int i, live = 0;

	for (i = 0; i < AGX_VM_SLOTS; i++)
		if (sc->client_roots[i].live &&
		    agx_root_pair(sc, &sc->client_roots[i]) == r && ++live > 1)
			return (true);
	return (false);
}

/* The pair a client root renders on; pair 0 if its own did not install. */
static struct agx_render *
agx_root_pair(struct apple_agx_softc *sc, struct agx_client_root *root)
{
	struct agx_render *r;

	r = &sc->renders[(root - sc->client_roots) % AGX_RENDER_PAIRS];
	return (r->installed ? r : &sc->renders[0]);
}

/* Alias every compact render object into a client root, once per root. */
static int
agx_render_map_client(struct apple_agx_softc *sc, struct agx_client_root *root)
{
	struct agx_render *r = agx_root_pair(sc, root);
	u_int i;
	int error;

	if (root->render_mapped || r->ncompact == 0)
		return (0);
	for (i = 0; i < r->ncompact; i++) {
		error = agx_uat_map(&sc->uat, root->pa, r->compact[i].va,
		    pmap_kextract((vm_offset_t)r->compact[i].kva),
		    r->compact[i].size, r->compact[i].prot);
		if (error != 0) {
			device_printf(sc->dev, "render: client alias %#lx: %d\n",
			    r->compact[i].va, error);
			return (error);
		}
	}
	error = agx_uat_map(&sc->uat, root->pa, AGX_RENDER_MESH_STORAGE_VA,
	    pmap_kextract((vm_offset_t)root->mesh_storage),
	    AGX_RENDER_MESH_STORAGE_SIZE, AGX_PROT_GPU_SHARED_RW);
	if (error != 0) {
		device_printf(sc->dev, "render: client mesh storage: %d\n", error);
		return (error);
	}
	root->render_mapped = true;
	return (0);
}

static int
agx_render_install_resources(struct agx_render *r)
{
	struct apple_agx_softc *sc = r->sc;
	const struct agx_render_pair_install *in = r->in;
	uint8_t *parameter, *pool, *control, *state, *metrics, *backing;
	volatile uint32_t *pb;
	volatile uint64_t *row;
	int table, pp, error;
	u_int i;

	table = agx_graph_find("pb");
	pp = agx_graph_find("page_pool");
	if (table < 0 || pp < 0)
		return (EINVAL);
	if ((error = agx_queue_object(sc, "render parameter graph", &parameter,
	    AGX_RENDER_PARAMETER_SIZE, sc->uat.upper_root,
	    AGX_RENDER_PARAMETER_VA + r->fw_off,
	    AGX_PROT_GPU_FW_SHARED_RW)) != 0 ||
	    (error = agx_render_alias(r, "render parameter page list",
	    parameter + AGX_RENDER_PAGE_LIST_OFFSET, AGX_RENDER_PAGE_LIST_SIZE,
	    AGX_RENDER_PAGE_LIST_VA, AGX_PROT_GPU_FW_SHARED_RW)) != 0 ||
	    (error = agx_queue_object(sc, "render metrics", &metrics, 0x8000,
	    sc->uat.upper_root, AGX_RENDER_METRICS_FW_VA + r->fw_off,
	    AGX_PROT_FW_SHARED_RW)) != 0 ||
	    (error = agx_render_alias(r, "render metrics", metrics, 0x8000,
	    AGX_RENDER_METRICS_VA, AGX_PROT_GPU_SHARED_RW)) != 0 ||
	    (error = agx_render_compact(r, "render scratch", &backing,
	    AGX_RENDER_SCRATCH_SIZE, AGX_RENDER_SCRATCH_VA)) != 0 ||
	    (error = agx_render_compact(r, "render discard", &backing, 0x8000,
	    AGX_RENDER_DISCARD_VA)) != 0)
		return (error);
	/*
	 * memory.rs keeps a mesh rendering buffer per queue: task renders of
	 * two clients sharing one hung the tiler. Each client root gets its
	 * own at the same VA.
	 */
	for (i = 0; r->index == 0 && i < AGX_VM_SLOTS; i++) {
		sc->client_roots[i].mesh_storage = (uint8_t *)kmem_alloc_contig(
		    AGX_RENDER_MESH_STORAGE_SIZE, M_WAITOK | M_ZERO, 0,
		    ~(vm_paddr_t)0, 0x10000, 0, VM_MEMATTR_WRITE_COMBINING);
		if (sc->client_roots[i].mesh_storage == NULL)
			return (ENOMEM);
	}
	for (i = 0; i < AGX_RENDER_TVB_BLOCKS; i++) {
		error = agx_render_compact(r, "render TVB block", &backing,
		    AGX_RENDER_TVB_BLOCK_SIZE,
		    AGX_RENDER_TVB_VA + i * AGX_RENDER_TVB_STRIDE);
		if (error != 0)
			return (error);
	}

	if ((error = agx_queue_object(sc, "render free list", &pool,
	    AGX_RENDER_POOL_SIZE, sc->uat.lower_root,
	    AGX_RENDER_POOL_LOW_VA + r->low_off, AGX_PROT_GPU_SHARED_RW)) != 0 ||
	    (error = agx_queue_object(sc, "render pool control", &control,
	    AGX_UAT_PGSZ, sc->uat.upper_root,
	    AGX_RENDER_POOL_CONTROL_VA + r->fw_off,
	    AGX_PROT_GPU_FW_SHARED_RW)) != 0 ||
	    (error = agx_queue_object(sc, "render pool state", &state,
	    AGX_UAT_PGSZ, sc->uat.upper_root,
	    AGX_RENDER_POOL_STATE_VA + r->fw_off,
	    AGX_PROT_GPU_FW_SHARED_RW)) != 0)
		return (error);
	r->parameter = parameter;
	r->pool_state = state;
	agx_put_words(parameter, in->parameter, in->nparameter);
	agx_put_words(pool, in->pool, in->npool);
	agx_put_words(control, in->pool_control, in->npool_control);
	dsb(sy);

	/* runtime/render.rs: a parameter buffer's row is its id * 16 bytes. */
	pb = (volatile uint32_t *)(sc->graph_va[table] + in->pb_slot * 16);
	pb[0] = in->pb_row[0] | (pb[0] & 6);
	pb[1] = in->pb_row[1] | (pb[1] & 0x0fc00000);
	pb[2] &= ~0x003fffffu;
	pb[3] &= ~0x803fffffu;
	row = (volatile uint64_t *)(sc->graph_va[pp] + in->pool_row);
	for (i = 0; i < nitems(agx_render_pool_row); i++)
		row[i] = in->pool_row_words[i];
	dsb(sy);
	return (0);
}

static int agx_render_objects(struct agx_render *r);

/*
 * Register pair k's QIDs (pair 0: 0 tiling, 1 fragment). This cold install
 * publishes only the configuration prefixes. No render kick or draw is
 * submitted.
 */
static int
agx_render_install(struct apple_agx_softc *sc, u_int k)
{
	struct agx_render *r = &sc->renders[k];
	const struct agx_render_pair_install *in = &agx_render_pair_installs[k];
	uint8_t *graph, *config, *scheduler, *work, *kicks;
	volatile uint32_t *ta, *fragment;
	const u_int producer_offset = (AGX_Q_PRODUCER - AGX_Q_CURSORS) / 4;
	int error, i;

	r->sc = sc;
	r->index = k;
	r->fw_off = k * AGX_RENDER_PAIR_STRIDE;
	r->low_off = k * AGX_RENDER_PAIR_LOW_STRIDE;
	r->in = in;
	if ((error = agx_queue_object(sc, "render graph", &graph,
	    AGX_RENDER_GRAPH_SIZE, sc->uat.upper_root,
	    AGX_RENDER_GRAPH_VA + r->fw_off, AGX_PROT_GPU_FW_SHARED_RW)) != 0 ||
	    (error = agx_queue_object(sc, "render configs", &config,
	    2 * AGX_RENDER_CONFIG_STRIDE, sc->uat.upper_root,
	    AGX_RENDER_CONFIG_VA + r->fw_off, AGX_PROT_GPU_FW_SHARED_RW)) != 0 ||
	    (error = agx_queue_object(sc, "render scheduler", &scheduler,
	    AGX_UAT_PGSZ, sc->uat.upper_root, AGX_RENDER_SCHED_VA + r->fw_off,
	    AGX_PROT_FW_SHARED_RW)) != 0 ||
	    (error = agx_queue_object(sc, "render work states", &work,
	    AGX_Q_WORK_SIZE, sc->uat.upper_root, AGX_RENDER_WORK_VA + r->fw_off,
	    AGX_PROT_FW_SHARED_RW)) != 0 ||
	    (error = agx_queue_object(sc, "render kick rings", &kicks,
	    2 * AGX_Q_KICK_SIZE, sc->uat.lower_root,
	    AGX_RENDER_KICK_LOW_VA + r->low_off, AGX_PROT_GPU_SHARED_RW)) != 0 ||
	    (error = agx_queue_alias(sc, "render kick rings", kicks,
	    2 * AGX_Q_KICK_SIZE, sc->uat.upper_root,
	    AGX_RENDER_KICK_HIGH_VA + r->fw_off, AGX_PROT_FW_SHARED_RW)) != 0)
		return (error);

	error = agx_render_install_resources(r);
	if (error != 0)
		return (error);

	r->graph = graph;
	r->work = work;
	r->kicks = kicks;
	r->context = AGX_Q_CONTEXT;
	r->pb_context = AGX_Q_CONTEXT;	/* the cold install's token 0 */
	agx_put_words(scheduler, agx_q_sched, nitems(agx_q_sched));
	agx_put_words(graph, in->graph, in->ngraph);
	memcpy(work + AGX_RENDER_WORK_HEAD, in->work_head,
	    sizeof(agx_render_work_head));
	memcpy(config, in->ta_config, sizeof(agx_render_ta_config));
	memcpy(config + AGX_RENDER_CONFIG_STRIDE, in->fragment_config,
	    sizeof(agx_render_fragment_config));
	ta = (volatile uint32_t *)(graph + AGX_RENDER_TA_CURSORS);
	fragment = (volatile uint32_t *)(graph + AGX_RENDER_FRAGMENT_CURSORS);
	dsb(sy);
	ta[producer_offset] = 1;
	fragment[producer_offset] = 1;
	dsb(sy);

	/* Both slots are on the shared compute ring. J813 rings pipe 0. */
	error = agx_publish_work_slots(sc, in->fragment_slot, in->ta_slot);
	if (error != 0)
		return (error);
	error = apple_rtkit_send(sc->roles[0].rtk, AGX_EP_DOORBELL,
	    AGX_MSG_WORK_NOTIFY | AGX_RENDER_PRIORITY << 2);
	if (error != 0)
		return (error);
	for (i = 0; i <= 1000; i++) {
		agx_poll_both(sc);
		if (apple_rtkit_crashed(sc->roles[0].rtk) ||
		    apple_rtkit_crashed(sc->roles[1].rtk))
			return (EIO);
		dsb(sy);
		if (ta[0] == 1 && fragment[0] == 1)
			break;
		DELAY(1000);
	}
	device_printf(sc->dev, "render pair %u configuration: QID %u %u/%u, "
	    "QID %u %u/%u after %d ms\n", k, in->qids[0], ta[0],
	    ta[producer_offset], in->qids[1], fragment[0],
	    fragment[producer_offset], i);
	if (ta[0] != 1 || fragment[0] != 1)
		return (ETIMEDOUT);
	r->items_end[0] = r->items_end[1] = 1;
	if ((error = agx_render_objects(r)) != 0)
		return (error);
	r->ts = 1;			/* KickTimestamp::FIRST */
	r->next_word = AGX_WORK_FIRST_WORD;
	r->next_id = 2;
	r->installed = true;
	return (0);
}

static void agx_recovery_task(void *arg, int pending);

/* Every render pair; one that fails to install leaves the ones before it. */
static int
agx_stage7(struct apple_agx_softc *sc)
{
	u_int k;
	int error;

	TASK_INIT(&sc->recovery_task, 0, agx_recovery_task, sc);
	for (k = 0; k < AGX_RENDER_PAIRS; k++)
		if ((error = agx_render_install(sc, k)) != 0) {
			device_printf(sc->dev, "render pair %u: install: %d\n",
			    k, error);
			return (k == 0 ? error : 0);
		}
	return (0);
}

/* The render pair's remaining kernel objects; see struct agx_render. */
#define	AGX_RENDER_SUPPORT_VA	(AGX_RENDER_GRAPH_VA + 0xb0000)
#define	AGX_RENDER_RECORDS_VA	(AGX_RENDER_GRAPH_VA + 0xb4000)
#define	AGX_RENDER_DESC_VA	(AGX_RENDER_GRAPH_VA + 0xc0000)	/* per slot */
#define	AGX_RENDER_DESC_LOW_VA	0x8020000000ul	/* past the free-list pool */
/*
 * Per slot. Sized for the largest single-layer pass agxdrm admits, 16384 x
 * 16384 with 16 x 16 utiles: a 5.25 MiB tile map and an 8 MiB TPC slice.
 */
#define	AGX_RENDER_TARGET_VA	0x1010000000ul
#define	AGX_RENDER_TARGET_SIZE	0x600000
#define	AGX_RENDER_TPC_VA	0x1010800000ul
#define	AGX_RENDER_TPC_SIZE	0x800000
#define	AGX_RENDER_SLOT_STRIDE	0x1000000	/* target and TPC */
#define	AGX_RENDER_STATE_VA	0x1020000000ul
#define	AGX_RENDER_STATE_SIZE	0x10000	/* generation.rs Layout::m5 */
#define	AGX_RENDER_STATE_STATUS	0x4000
#define	AGX_RENDER_STATE_AUX	0x8000
#define	AGX_RENDER_VIEWPORT	0x9c0	/* ContextStore stream + 0x40 */
/* The support page: both stages' stamps, then per-slot usage and timestamps. */
#define	AGX_RENDER_USAGE	0x100	/* + slot * 0x80 */
#define	AGX_RENDER_TIMESTAMPS	0x400	/* + slot * 0x20 */
/* The records page: per-slot configurations and announcements. */
#define	AGX_RENDER_RECORD_STRIDE 0x400
#define	AGX_RENDER_ANNOUNCE	0x200	/* within a slot's records */
#define	AGX_RENDER_FREE_LIST_SLOT 129
#define	AGX_RENDER_ID_PREDECESSOR 1
#define	AGX_RENDER_TIMEOUT	(2 * SBT_1S)

static void agx_render_reap_task(void *arg, int pending);
static void agx_render_watchdog(void *arg);

static int
agx_render_objects(struct agx_render *r)
{
	struct apple_agx_softc *sc = r->sc;
	struct agx_render_slot *slot;
	u_int i;
	int error;

	if ((error = agx_queue_object(sc, "render support", &r->support,
	    AGX_UAT_PGSZ, sc->uat.upper_root, AGX_RENDER_SUPPORT_VA + r->fw_off,
	    AGX_PROT_GPU_FW_SHARED_RW)) != 0 ||
	    (error = agx_queue_object(sc, "render records", &r->records,
	    AGX_UAT_PGSZ, sc->uat.upper_root, AGX_RENDER_RECORDS_VA + r->fw_off,
	    AGX_PROT_FW_SHARED_RW)) != 0)
		return (error);
	for (i = 0; i < AGX_RENDER_SLOTS; i++) {
		slot = &r->slots[i];
		if ((error = agx_queue_object(sc, "render commands", &slot->desc,
		    AGX_M5_RENDER_STORAGE_BYTES, sc->uat.upper_root,
		    AGX_RENDER_DESC_VA + r->fw_off +
		    i * AGX_M5_RENDER_STORAGE_BYTES,
		    AGX_PROT_GPU_FW_SHARED_RW)) != 0 ||
		    (error = agx_queue_alias(sc, "render commands", slot->desc,
		    AGX_M5_RENDER_STORAGE_BYTES, sc->uat.lower_root,
		    AGX_RENDER_DESC_LOW_VA + r->low_off +
		    i * AGX_M5_RENDER_STORAGE_BYTES,
		    AGX_PROT_GPU_SHARED_RW)) != 0 ||
		    (error = agx_render_compact(r, "render target", &slot->target,
		    AGX_RENDER_TARGET_SIZE,
		    AGX_RENDER_TARGET_VA + i * AGX_RENDER_SLOT_STRIDE)) != 0 ||
		    (error = agx_render_compact(r, "render TPC", &slot->tpc,
		    AGX_RENDER_TPC_SIZE,
		    AGX_RENDER_TPC_VA + i * AGX_RENDER_SLOT_STRIDE)) != 0 ||
		    (error = agx_render_compact(r, "render state", &slot->state,
		    AGX_RENDER_STATE_SIZE,
		    AGX_RENDER_STATE_VA + i * AGX_RENDER_STATE_SIZE)) != 0)
			return (error);
	}
	TASK_INIT(&r->reap_task, 0, agx_render_reap_task, r);
	callout_init_mtx(&r->watchdog, &sc->mtx, 0);
	return (0);
}

/* storage.rs Viewport::new with_control(0x3000), applied word by word. */
static int
agx_render_viewport(uint8_t *at, uint32_t width, uint32_t height)
{
	volatile uint32_t *v = (volatile uint32_t *)at;
	uint32_t x, y, dims[2] = { width, height }, half[2];
	int i, highest;

	for (i = 0; i < 2; i++) {
		if (dims[i] == 0)
			return (EINVAL);
		highest = flsl(dims[i]) - 1;
		if (highest > 23)
			return (EINVAL);
		half[i] = (uint32_t)(highest + 126) << 23 |
		    ((dims[i] << (23 - highest)) & 0x7fffff);
	}
	x = half[0];
	y = half[1];
	v[0] = 0x3000;
	v[1] = 0x80000000u | ((width + 31) / 32 - 1);
	v[2] = (height + 31) / 32 - 1;
	v[4] = x;
	v[5] = x;
	v[6] = y;
	v[7] = y | 0x80000000u;
	v[9] = 0x3f800000;
	return (0);
}

static void
agx_put_unaligned64(uint8_t *p, uint64_t v)
{
	int i;

	for (i = 0; i < 8; i++)
		((volatile uint8_t *)p)[i] = v >> (8 * i);
}

/* Free entries of an item ring (one stays empty). */
static uint32_t
agx_items_room(uint32_t producer, uint32_t consumer)
{

	return (AGX_Q_ITEM_ENTRIES - 1 -
	    (producer + AGX_Q_ITEM_ENTRIES - consumer) % AGX_Q_ITEM_ENTRIES);
}

/*
 * One render on the pair, as queue/render.rs prepare and publish build it
 * on the J813, into a free slot: the target, state and scene, the
 * work-state node, both commands (apple_agx_render.c), a configuration,
 * the descriptor and an announcement on each stage's item ring, both kick
 * entries, QoS, the free-list and parameter-manager submissions, fragment
 * then tiling on the shared work ring, and the pipe-0 doorbell. Earlier
 * renders may still run: the tiling kick waits for the last fragment kick.
 */
static int
agx_render_kick(struct agx_render *r, u_int index, u_int context,
    uint8_t generation, const struct agx_render_pass *pass,
    const struct agx_stage_plan stages[2], uint8_t mesh_class,
    const uint64_t *deps, u_int ndeps)
{
	struct apple_agx_softc *sc = r->sc;
	const struct agx_render_pair_install *in = r->in;
	uint64_t ta_barriers[1 + APPLE_AGX_MAX_DEPS];
	uint8_t ta_types[2 + APPLE_AGX_MAX_DEPS];
	u_int nb;
	struct agx_render_slot *slot = &r->slots[index];
	const struct agx_board *board = &agx_j813_board;
	static const size_t cursors_off[2] = { AGX_RENDER_TA_CURSORS,
	    AGX_RENDER_FRAGMENT_CURSORS };
	static const size_t items_off[2] = { AGX_RENDER_TA_ITEMS,
	    AGX_RENDER_FRAGMENT_ITEMS };
	const u_int producer_offset = (AGX_Q_PRODUCER - AGX_Q_CURSORS) / 4;
	/* The pair's firmware objects. */
	const uint64_t graph_va = AGX_RENDER_GRAPH_VA + r->fw_off;
	const uint64_t work_va = AGX_RENDER_WORK_VA + r->fw_off;
	const uint64_t sched_va = AGX_RENDER_SCHED_VA + r->fw_off;
	const uint64_t parameter_va = AGX_RENDER_PARAMETER_VA + r->fw_off;
	const uint64_t pool_control_va = AGX_RENDER_POOL_CONTROL_VA + r->fw_off;
	const uint64_t support_va = AGX_RENDER_SUPPORT_VA + r->fw_off;
	const uint64_t records_va = AGX_RENDER_RECORDS_VA + r->fw_off;
	const uint64_t desc_va = AGX_RENDER_DESC_VA + r->fw_off +
	    index * AGX_M5_RENDER_STORAGE_BYTES;
	const uint64_t desc_low = AGX_RENDER_DESC_LOW_VA + r->low_off +
	    index * AGX_M5_RENDER_STORAGE_BYTES;
	const uint64_t target_va = AGX_RENDER_TARGET_VA +
	    index * AGX_RENDER_SLOT_STRIDE;
	const uint64_t tpc_va = AGX_RENDER_TPC_VA + index * AGX_RENDER_SLOT_STRIDE;
	const uint64_t state_va = AGX_RENDER_STATE_VA +
	    index * AGX_RENDER_STATE_SIZE;
	const size_t records = index * AGX_RENDER_RECORD_STRIDE;
	struct agx_m5_target_layout layout;
	struct agx_render_resources res;
	struct agx_render_args a;
	struct agx_m5_render_encoded enc;
	struct agx_queue_config_args ca;
	struct agx_kick_args ka;
	uint8_t config[AGX_QUEUE_CONFIG_SIZE], announce[AGX_KICK_ANNOUNCE_SIZE];
	uint8_t slots[2][AGX_WORK_SLOT_SIZE], node[0xc8], types[2][2];
	volatile uint32_t *cursors[2];
	volatile uint64_t *items;
	uint64_t ts = r->ts, kicks[2], barriers[2], va, node_va, values[3];
	uint32_t word, base[2], counter_off, ordinal = r->ordinal;
	uint32_t scene = 1 + ordinal % AGX_RENDER_SLOTS;
	bool identity;
	int error, s, i;

	if ((error = agx_m5_target_layout(pass, 1,
	    AGX_RENDER_TVB_BLOCKS * AGX_RENDER_TVB_BLOCK_SIZE, &layout)) != 0)
		return (error);
	if (layout.used > AGX_RENDER_TARGET_SIZE ||
	    layout.slice_bytes > AGX_RENDER_TPC_SIZE)
		return (E2BIG);
	for (s = 0; s < 2; s++) {
		cursors[s] = (volatile uint32_t *)(r->graph + cursors_off[s]);
		base[s] = cursors[s][producer_offset];
		if (base[s] != r->items_end[s] ||
		    agx_items_room(base[s], cursors[s][0]) < 3) {
			device_printf(sc->dev, "render %u: stage %d cursors %u/%u\n",
			    ordinal, s, cursors[s][0], base[s]);
			return (EIO);
		}
	}

	/* The slot's target records, TPC, state generation and timestamps. */
	memset(slot->target, 0, roundup2(layout.used, AGX_UAT_PGSZ));
	memset(slot->tpc, 0, roundup2(layout.slice_bytes, AGX_UAT_PGSZ));
	memset(slot->state + AGX_RENDER_STATE_STATUS, 0, AGX_UAT_PGSZ);
	memset(slot->state + AGX_RENDER_STATE_AUX, 0, AGX_RENDER_STATE_SIZE -
	    AGX_RENDER_STATE_AUX);
	if ((error = agx_render_viewport(slot->state + AGX_RENDER_VIEWPORT,
	    pass->width, pass->height)) != 0)
		return (error);
	memset(r->support + AGX_RENDER_USAGE + index * 0x80, 0, 0x80);
	memset(r->support + AGX_RENDER_TIMESTAMPS + index * 0x20, 0, 0x20);

	/* Manager::prepare_scene */
	*(volatile uint32_t *)(r->parameter + AGX_RENDER_SCENE_TABLE +
	    scene * AGX_RENDER_SCENE_SIZE + 0x48) = 0;
	agx_put_unaligned64(r->parameter + AGX_RENDER_SCENE_TABLE +
	    scene * AGX_RENDER_SCENE_SIZE + 0x4c, AGX_RENDER_PB_GENERATION);
	/* State::buffer_id */
	*(volatile uint32_t *)(r->parameter + 0xc) = in->pb_slot;

	/* The work-state node, with both kicks submitted. */
	do {
		word = r->next_word++;
	} while ((word & 63) == AGX_WORK_STATES);
	node_va = work_va + (word & 63) * AGX_NODE_STRIDE;
	counter_off = AGX_UAT_PGSZ + (word & 63) * 4;
	memset(node, 0, sizeof(node));
	va = work_va + counter_off;
	memcpy(node, &va, 8);
	memcpy(node + 8, &word, 4);
	node[0x10] = 0x50;
	memcpy(r->work + (word & 63) * AGX_NODE_STRIDE, node, sizeof(node));
	*(volatile uint32_t *)(r->work + counter_off) = 0;
	dsb(sy);

	memset(&res, 0, sizeof(res));
	res.context_id = context;
	res.context_generation = generation;
	res.pb_slot = in->pb_slot;
	res.free_list_slot = in->pool_slot;
	res.free_list_control_va = pool_control_va;
	res.kick_record_va = node_va;
	res.pb_va = parameter_va;
	res.scene_va = parameter_va + AGX_RENDER_SCENE_TABLE +
	    scene * AGX_RENDER_SCENE_SIZE;
	res.scene_end_va = parameter_va + AGX_RENDER_SCENE_TABLE +
	    AGX_RENDER_SCENES * AGX_RENDER_SCENE_SIZE;
	res.pm_scratch = AGX_RENDER_SCRATCH_VA - AGX_COMPACT_BASE +
	    (scene % AGX_RENDER_SCRATCH_SLOTS) * AGX_RENDER_SCRATCH_STRIDE;
	res.pm_metric = (AGX_RENDER_METRICS_VA - AGX_COMPACT_BASE +
	    scene * AGX_RENDER_METRIC_STRIDE) | 1;
	res.tilemap_va = target_va;
	res.layermeta_va = target_va + layout.layer_offset;
	res.heapmeta_va = target_va + layout.heap_offset;
	res.tpc_va = tpc_va;
	res.tpc_size = layout.tpc_size;
	res.deflake_va = state_va;
	res.ta_status_va = state_va + AGX_RENDER_STATE_STATUS;
	res.fragment_status_va = res.ta_status_va + 8;
	res.aux_fb_va = state_va + AGX_RENDER_STATE_AUX;
	/* The firmware writes the stage timestamps: a firmware address. */
	res.timestamp_va = support_va + AGX_RENDER_TIMESTAMPS + index * 0x20;
	/* ids.rs render_pair: fragment, then tiling, after the graph's id. */
	res.fragment_ids.predecessor = res.ta_ids.predecessor =
	    AGX_RENDER_ID_PREDECESSOR;
	res.fragment_ids.id = r->next_id;
	res.ta_ids.id = r->next_id + 1;

	memset(&a, 0, sizeof(a));
	a.board = board;
	a.pass = pass;
	a.resources = &res;
	a.ordinal = ordinal;
	a.ta = (struct agx_render_stage){ in->qids[0], desc_low, word, support_va,
	    support_va + 8 };
	a.fragment = (struct agx_render_stage){ in->qids[1], desc_low +
	    AGX_M5_RENDER_FRAGMENT_OFFSET, word, support_va + 0x10,
	    support_va + 0x18 };
	a.has_stages = true;
	memcpy(a.stages, stages, sizeof(a.stages));
	a.mesh_class = mesh_class;
	/* render.rs: only API task kicks get the TA mesh rendering buffer. */
	if (mesh_class == 2)
		res.mesh_storage = AGX_RENDER_MESH_STORAGE_VA |
		    (uint64_t)sc->renders[0].mesh_chunks << 48;
	a.usage[0] = support_va + AGX_RENDER_USAGE + index * 0x80;
	a.usage[1] = a.usage[0] + 0x40;
	memset(slot->desc, 0, AGX_M5_RENDER_STORAGE_BYTES);
	if ((error = agx_m5_render_encode(slot->desc, desc_low, &a, &enc)) != 0 ||
	    (error = agx_m5_render_bind_work_slot(slot->desc, 0)) != 0) {
		device_printf(sc->dev, "render %u: encode: %d\n", ordinal, error);
		return (error);
	}
	kicks[0] = kicks[1] = ts;
	/* context/identity.rs payload: the channel's pass count. */
	agx_m5_render_link_completions(slot->desc, kicks, (uint8_t)(ordinal + 1),
	    false);
	dsb(sy);

	/* Each stage: configuration, descriptor, announcement. */
	identity = context != r->context || generation != r->generation;
	/*
	 * buffer.rs acquire_owner_bound: the firmware refuses to rebind a
	 * token to another context's PB state; a moved binding takes a fresh one.
	 */
	if (context != r->pb_context) {
		r->pb_token++;
		r->pb_context = context;
	}
	for (s = 0; s < 2; s++) {
		memset(&ca, 0, sizeof(ca));
		ca.kick_ring_va = AGX_RENDER_KICK_LOW_VA + r->low_off +
		    s * AGX_Q_KICK_SIZE;
		ca.kick_ring_fw_va = AGX_RENDER_KICK_HIGH_VA + r->fw_off +
		    s * AGX_Q_KICK_SIZE;
		ca.qid = in->qids[s];
		ca.context_id = context;
		ca.free_list_va = pool_control_va;
		ca.free_list_generation = 1;
		ca.free_list_slot = in->pool_slot;
		ca.scheduler_va = sched_va;
		ca.priority = AGX_RENDER_PRIORITY;
		ca.policy = AGX_Q_POLICY;
		ca.qos_slot = in->qos;
		ca.qos_update = identity;
		ca.owner_pid = 1;
		ca.context_update = identity;
		if ((error = agx_queue_config_render(config, &ca, s == 0,
		    parameter_va, r->pb_token, in->pb_slot, in->qids[1])) != 0 ||
		    (error = agx_kick_announce(announce, in->qids[s], s == 0 ?
		    AGX_DM_TILING : AGX_DM_FRAGMENT, ts,
		    AGX_RENDER_PRIORITY)) != 0)
			return (error);
		values[0] = agx_queue_record(r->records, records_va,
		    records + s * 0x100, 0x100, config, sizeof(config));
		values[1] = desc_va + s * AGX_M5_RENDER_FRAGMENT_OFFSET;
		values[2] = agx_queue_record(r->records, records_va,
		    records + AGX_RENDER_ANNOUNCE + s * 0x80, 0x80, announce,
		    sizeof(announce));
		items = (volatile uint64_t *)(r->graph + items_off[s]);
		for (i = 0; i < 3; i++)
			items[(base[s] + i) % AGX_Q_ITEM_ENTRIES] = values[i];
	}

	/*
	 * Kick entries. The fragment kick waits for its own tiling kick
	 * (classes 1, then 3 for its parent). On the J813 the tiling kick
	 * also waits for the last fragment kick (queue/render.rs
	 * J813_LAST_FRAGMENT, class 2), then its parent (class 1).
	 */
	/*
	 * The tiling kick's barriers: the last fragment kick, then the
	 * caller's kicks on other queues (completion, type 1), then the
	 * parent.
	 */
	nb = 0;
	if (ordinal != 0) {
		ta_barriers[nb] = agx_kick_dependency(in->qids[1], ts - 1);
		ta_types[nb++] = 2;
	}
	for (i = 0; i < ndeps; i++) {
		ta_barriers[nb] = deps[i];
		ta_types[nb++] = 1;
	}
	ta_types[nb] = 1;
	barriers[1] = agx_kick_dependency(in->qids[0], ts);
	types[1][0] = 1;
	types[1][1] = 3;
	for (s = 1; s >= 0; s--) {
		memset(&ka, 0, sizeof(ka));
		ka.qid = in->qids[s];
		ka.timestamp = ts;
		ka.parent = ts - 1;
		if (s == 1) {
			ka.barriers = &barriers[1];
			ka.nbarriers = 1;
			ka.dependency_types = types[1];
		} else {
			ka.barriers = nb != 0 ? ta_barriers : NULL;
			ka.nbarriers = nb;
			ka.dependency_types = ta_types;
		}
		ka.descriptor_va = desc_va + s * AGX_M5_RENDER_FRAGMENT_OFFSET;
		ka.queue_va = graph_va + in->qids[s] * AGX_RENDER_QUEUE_STRIDE;
		ka.qos_slot = in->qos;
		ka.qos_class = s == 0 ? AGX_TA_QOS_CLASS : AGX_FRAGMENT_QOS_CLASS;
		memcpy(ka.register_arrays, enc.rce[s], sizeof(ka.register_arrays));
		ka.scs_enabled = enc.scratch[s];
		ka.compute_matrix = enc.matrix[s];
		ka.priority = AGX_RENDER_PRIORITY;
		if ((error = agx_kick_entry(r->kicks + s * AGX_Q_KICK_SIZE +
		    (ts & 0xff) * AGX_KICK_ENTRY_SIZE, &ka)) != 0)
			return (error);
	}
	dsb(sy);
	if (ordinal == 0)
		agx_uat_tlbi_all();

	if ((error = agx_qos_publish(sc, in->qids[1], in->qos, AGX_DM_FRAGMENT,
	    sched_va, AGX_Q_QOS_CLASS, AGX_Q_QOS_SHARE)) != 0 ||
	    (error = agx_qos_publish(sc, in->qids[0], in->qos, AGX_DM_TILING,
	    sched_va, AGX_Q_QOS_CLASS, AGX_Q_QOS_SHARE)) != 0)
		return (error);
	for (s = 0; s < 2; s++)
		r->items_end[s] = (base[s] + 3) % AGX_Q_ITEM_ENTRIES;
	/* initdata.rs m5_work_pair: the shared ring's slots name compute. */
	if ((error = agx_work_slot(slots[0], AGX_DM_COMPUTE,
	    graph_va + in->qids[1] * AGX_RENDER_QUEUE_STRIDE, in->qids[1],
	    r->items_end[1], false)) != 0 ||
	    (error = agx_work_slot(slots[1], AGX_DM_COMPUTE,
	    graph_va + in->qids[0] * AGX_RENDER_QUEUE_STRIDE, in->qids[0],
	    r->items_end[0], false)) != 0)
		return (error);
	/* add_submitted_kicks(2), the free-list and manager submissions. */
	*(volatile uint32_t *)(r->work + counter_off) = 2;
	*(volatile uint64_t *)r->pool_state += 2;
	(*(volatile uint32_t *)(r->parameter + AGX_RENDER_PM_COUNTER))++;
	dsb(sy);
	cursors[1][producer_offset] = r->items_end[1];
	cursors[0][producer_offset] = r->items_end[0];
	dsb(sy);
	if ((error = agx_publish_work_slots(sc, slots[0], slots[1])) != 0)
		return (error);
	slot->ordinal = ordinal;
	slot->stamp = agx_stamp_value(ordinal + 1);
	slot->node_va = node_va;
	r->context = context;
	r->generation = generation;
	r->ts = ts + 1;
	r->next_id += 2;
	r->ordinal = ordinal + 1;
	agx_publish_effort(sc);
	return (apple_rtkit_send(sc->roles[0].rtk, AGX_EP_DOORBELL,
	    AGX_MSG_WORK_NOTIFY | AGX_RENDER_PRIORITY << 2));
}

/* The slot's render retired: both stamps cover it and its node counts both kicks. */
static bool
agx_render_retired(struct agx_render *r, struct agx_render_slot *slot)
{
	volatile uint32_t *stamps = (volatile uint32_t *)r->support;
	volatile uint32_t *node = (volatile uint32_t *)(r->work +
	    (slot->node_va - (AGX_RENDER_WORK_VA + r->fw_off)));

	dsb(sy);
	return (agx_stamp_covers(stamps[0], slot->stamp) &&
	    agx_stamp_covers(stamps[4], slot->stamp) &&
	    node[AGX_Q_NODE_COMPLETED / 4] >= 2);
}

struct agx_render_done {
	void		(*done)(void *, int);
	void		*arg;
	int		error;
};

/*
 * Retire renders oldest first. A render past its deadline, or a crashed
 * firmware, fails the pair and every render in flight. Completions are
 * collected for the caller to run without the mutex.
 */
static u_int
agx_render_reap(struct agx_render *r, struct agx_render_done *out)
{
	struct apple_agx_softc *sc = r->sc;
	struct agx_render_slot *slot;
	volatile uint32_t *stamps = (volatile uint32_t *)r->support;
	u_int n = 0;
	int error;

	mtx_assert(&sc->mtx, MA_OWNED);
	if (sc->roles[0].irq_tag == NULL)
		agx_poll_both(sc);
	while (r->in_flight != 0) {
		slot = &r->slots[r->oldest];
		error = 0;
		if (!r->failed && slot->error == 0 &&
		    !agx_render_retired(r, slot)) {
			/* A restart being serviced holds every deadline. */
			if (!apple_rtkit_crashed(sc->roles[0].rtk) &&
			    !apple_rtkit_crashed(sc->roles[1].rtk) &&
			    (sc->recovering || sbinuptime() < slot->deadline))
				break;
			device_printf(sc->dev, "pair %u render %u: not retired: "
			    "stamps TA %#x 3D %#x (want %#x), %u in flight\n",
			    r->index, slot->ordinal, stamps[0], stamps[4],
			    slot->stamp, r->in_flight);
			r->failed = true;
		}
		if (r->failed)
			error = apple_rtkit_crashed(sc->roles[0].rtk) ? EIO :
			    ETIMEDOUT;
		else if (slot->error != 0)
			error = slot->error;
		else {
			agx_qos_complete(sc, r->in->qids[1], r->in->qos,
			    AGX_DM_FRAGMENT);
			agx_qos_complete(sc, r->in->qids[0], r->in->qos,
			    AGX_DM_TILING);
			r->retired++;
			r->last_good = *slot;
		}
		out[n++] = (struct agx_render_done){ slot->done, slot->arg,
		    error };
		slot->used = false;
		r->oldest = (r->oldest + 1) % AGX_RENDER_SLOTS;
		r->in_flight--;
	}
	if (n != 0)
		wakeup(&r->in_flight);
	if (r->in_flight != 0)
		callout_reset_sbt(&r->watchdog, 10 * SBT_1MS, 0,
		    agx_render_watchdog, r, 0);
	else
		callout_stop(&r->watchdog);
	return (n);
}

static void
agx_render_complete(struct agx_render_done *done, u_int n)
{
	u_int i;

	for (i = 0; i < n; i++)
		if (done[i].done != NULL)
			done[i].done(done[i].arg, done[i].error);
}

static void
agx_render_reap_task(void *arg, int pending)
{
	struct agx_render *r = arg;
	struct agx_render_done done[AGX_RENDER_SLOTS];
	u_int n;

	mtx_lock(&r->sc->mtx);
	n = agx_render_reap(r, done);
	mtx_unlock(&r->sc->mtx);
	agx_render_complete(done, n);
}

/* Every 10 ms while renders run: stamps can land without an event. */
static void
agx_render_watchdog(void *arg)
{
	struct agx_render *r = arg;

	taskqueue_enqueue(taskqueue_thread, &r->reap_task);
}

/*
 * Publish one render pass and return; done(arg, error) runs once it
 * retires or fails, from a task thread, without the driver's locks. A
 * full set of slots waits for the oldest render.
 */
int
apple_agx_submit_render_async(device_t dev, u_int context,
    const struct agx_render_pass *pass,
    const struct agx_render_program *programs, u_int nprograms,
    void (*done)(void *, int), void *arg)
{

	return (apple_agx_submit_render_deps(dev, context, pass, programs,
	    nprograms, NULL, 0, NULL, done, arg));
}

int
apple_agx_submit_render_deps(device_t dev, u_int context,
    const struct agx_render_pass *pass,
    const struct agx_render_program *programs, u_int nprograms,
    const uint64_t *deps, u_int ndeps, uint64_t *dep,
    void (*done)(void *, int), void *arg)
{
	struct apple_agx_softc *sc = device_get_softc(dev);
	struct agx_render *r;
	struct agx_render_done finished[AGX_RENDER_SLOTS];
	struct agx_client_root *root;
	struct agx_render_pass p = *pass;
	struct agx_stage_plan stages[2];
	struct agx_render_slot *slot;
	u_int i, index, n = 0;
	uint8_t mesh_class = 0;
	int error;

	if (ndeps > APPLE_AGX_MAX_DEPS)
		return (EINVAL);
	p.usc_base = AGX_CLIENT_SHADER;
	/*
	 * render_resources.rs mesh_class: 1 for a hardware mesh (with its
	 * passthrough object stage), 2 with an API task stage.
	 */
	for (i = 0; i < nprograms; i++) {
		if (programs[i].task)
			mesh_class = 2;
		else if (programs[i].threads != 0 && mesh_class == 0)
			mesh_class = 1;
	}
	/* render_resources.rs M5Policy: every core of every cluster. */
	error = agx_m5_render_plan(sc->id.core_masks, sc->id.num_clusters,
	    sc->id.num_cores * sc->id.num_clusters, AGX_RENDER_PRIORITY,
	    programs, nprograms, stages);
	if (error != 0)
		return (error);
	mtx_lock(&sc->mtx);
	root = agx_client_root(sc, context);
	if (root == NULL || !root->live) {
		mtx_unlock(&sc->mtx);
		return (ENOENT);
	}
	r = agx_root_pair(sc, root);
	while (r->installed && !r->failed && r->in_flight == AGX_RENDER_SLOTS) {
		n = agx_render_reap(r, finished);
		if (n != 0)
			break;
		mtx_sleep(&r->in_flight, &sc->mtx, 0, "agxslot", hz / 100 + 1);
	}
	/*
	 * A task render for another context than the last one waits for the
	 * GPU to drain: task tiling switching among three or more clients
	 * hung the tiler with in-flight work behind it (J815, no reference).
	 * Every render does on a shared pair (see agx_ctx_drain).
	 */
	while (((mesh_class == 2 && context != r->task_context) ||
	    (context != r->context && (agx_ctx_drain ||
	    agx_pair_shared(sc, r)))) && r->installed &&
	    !r->failed && r->in_flight != 0) {
		n += agx_render_reap(r, finished + n);
		if (r->in_flight == 0)
			break;
		mtx_sleep(&r->in_flight, &sc->mtx, 0, "agxtask", hz / 100 + 1);
	}
	if (!r->installed || r->failed || !root->live) {
		error = !r->installed ? ENODEV : r->failed ? EIO : ENOENT;
		goto out;
	}
	if ((error = agx_render_map_client(sc, root)) != 0)
		goto out;
	index = (r->oldest + r->in_flight) % AGX_RENDER_SLOTS;
	slot = &r->slots[index];
	if (dep != NULL)
		*dep = agx_kick_dependency(r->in->qids[1], r->ts);
	error = agx_render_kick(r, index, context, root->generation, &p,
	    stages, mesh_class, deps, ndeps);
	if (error == 0 && mesh_class == 2) {
		sc->renders[0].mesh_storage = root->mesh_storage;
		r->task_context = context;
	}
	if (error == 0) {
		slot->used = true;
		slot->done = done;
		slot->arg = arg;
		slot->deadline = sbinuptime() + AGX_RENDER_TIMEOUT;
		slot->error = 0;
		slot->context = context;
		slot->width = pass->width;
		slot->height = pass->height;
		slot->pass = p;
		slot->nprograms = nprograms;
		memcpy(slot->programs, programs, MIN(nprograms,
		    nitems(slot->programs)) * sizeof(*programs));
		if (++r->in_flight > r->peak_in_flight)
			r->peak_in_flight = r->in_flight;
		callout_reset_sbt(&r->watchdog, 10 * SBT_1MS, 0,
		    agx_render_watchdog, r, 0);
	} else if (error != EINVAL && error != E2BIG && error != EOPNOTSUPP &&
	    r->ordinal != 0)
		/* A partly published render may still run. */
		r->failed = true;
out:
	mtx_unlock(&sc->mtx);
	agx_render_complete(finished, n);
	if (error != 0)
		device_printf(sc->dev, "submit: render %ux%u: %d\n", pass->width,
		    pass->height, error);
	return (error);
}

struct agx_render_wait {
	struct mtx	*mtx;
	bool		done;
	int		error;
};

static void
agx_render_wait_done(void *arg, int error)
{
	struct agx_render_wait *w = arg;

	mtx_lock(w->mtx);
	w->error = error;
	w->done = true;
	wakeup(w);
	mtx_unlock(w->mtx);
}

int
apple_agx_submit_render(device_t dev, u_int context,
    const struct agx_render_pass *pass,
    const struct agx_render_program *programs, u_int nprograms,
    u_int timeout_ms)
{
	struct apple_agx_softc *sc = device_get_softc(dev);
	struct agx_render_wait w = { &sc->mtx, false, 0 };
	int error;

	(void)timeout_ms;	/* each render has AGX_RENDER_TIMEOUT */
	error = apple_agx_submit_render_async(dev, context, pass, programs,
	    nprograms, agx_render_wait_done, &w);
	if (error != 0)
		return (error);
	mtx_lock(&sc->mtx);
	while (!w.done)
		mtx_sleep(&w, &sc->mtx, 0, "agxrnd", 0);
	mtx_unlock(&sc->mtx);
	return (w.error);
}

/*
 * Stage 3: boot_m5 after both coprocessors run. Adopt the roots, build
 * the graph, hand out the log buffers (GFX1, GFX), InitData (GFX1, GFX),
 * POWER_ON_DONE to both, and wait for the opening records to retire.
 */
static int
agx_stage3(struct apple_agx_softc *sc)
{
	uint64_t snap[3];
	int error, i, idx;

	if ((error = agx_uat_adopt_roots(&sc->uat, snap)) != 0)
		return (error);
	device_printf(sc->dev, "UAT: firmware root entries %#lx %#lx %#lx\n",
	    snap[0], snap[1], snap[2]);
	_Static_assert(nitems(agx_graph_objects) > 0, "graph");
	if (strcmp(agx_graph_objects[0].name, "cluster") != 0)
		return (EINVAL);
	if ((error = agx_build_graph(sc)) != 0)
		return (error);
	if ((error = agx_uat_confirm_roots(&sc->uat, snap)) != 0) {
		device_printf(sc->dev, "UAT: firmware root entries changed\n");
		return (error);
	}
	if ((error = agx_uat_mirror_secondary(&sc->uat)) != 0)
		return (error);

	for (idx = 1; idx >= 0; idx--) {
		/* FW_LOG[role]: fw_log0 is GFX's, fw_log1 GFX1's. */
		int o = agx_graph_find(idx == 0 ? "fw_log0" : "fw_log1");
		uint64_t va;

		if (o < 0)
			return (EINVAL);
		va = agx_graph_objects[o].va;
		if ((error = apple_rtkit_send(sc->roles[idx].rtk,
		    AGX_EP_CRASHLOG, AGX_MSG_LOG_BUFFER |
		    (uint64_t)(AGX_FW_LOG_SIZE >> 12) << 44 |
		    (va & AGX_MSG_ADDR_MASK))) != 0)
			return (error);
	}
	if ((error = agx_role_init(sc, 1)) != 0 ||
	    (error = agx_role_init(sc, 0)) != 0)
		return (error);
	if ((error = apple_rtkit_send(sc->roles[0].rtk, AGX_EP_DOORBELL,
	    AGX_MSG_POWER_ON_DONE)) != 0 ||
	    (error = apple_rtkit_send(sc->roles[1].rtk, AGX_EP_DOORBELL,
	    AGX_MSG_POWER_ON_DONE)) != 0)
		return (error);
	for (i = 0; i <= 1000; i++) {
		agx_poll_both(sc);
		if (apple_rtkit_crashed(sc->roles[0].rtk) ||
		    apple_rtkit_crashed(sc->roles[1].rtk))
			return (EIO);
		if (agx_opening_retired(sc, 0) && agx_opening_retired(sc, 1))
			break;
		DELAY(1000);
	}
	if (i > 1000) {
		device_printf(sc->dev, "firmware did not retire its opening "
		    "records\n");
		return (ETIMEDOUT);
	}
	device_printf(sc->dev, "M5 firmware opening records retired after "
	    "%d ms\n", i);
	sc->running = true;
	return (0);
}

static int
agx_stage2(struct apple_agx_softc *sc, phandle_t node)
{
	uint32_t v;
	int error, i;

	if ((error = agx_roles_init(sc, node)) != 0) {
		device_printf(sc->dev, "cannot set up mailboxes: %d\n", error);
		return (error);
	}
	if ((error = agx_uat_init(sc, node)) != 0)
		return (error);

	/*
	 * The firmware samples the lower root's top-level entries only when
	 * it starts: link every lower range the 26B123 graph uses first
	 * (InitData::m5_lower_links, COMMON26 layout).
	 */
	{
		static const u_long links[] = {
			0x1000080000ul,			/* BOOT_LINK */
			0x7201838000ul, 0x7201840000ul,	/* region views */
			0x7201848000ul, 0x7201870000ul,	/* completion rings */
			0x7201898000ul, 0x72018c0000ul,
			0x8000000000ul,			/* canonical arena */
			/* The first job's client VM ranges (stage 6). */
			0x1000100000ul,			/* driver: CDM stream */
			0x4000000000ul,			/* data heap */
			0x9000000000ul,			/* client-private aliases */
			0x10000000000ul,		/* USC window */
		};
		for (i = 0; i < (int)nitems(links); i++)
			if ((error = agx_uat_alloc_tables(&sc->uat,
			    sc->uat.lower_root, links[i],
			    links[i] + AGX_UAT_PGSZ)) != 0) {
				device_printf(sc->dev, "lower link %#lx: %d\n",
				    links[i], error);
				return (error);
			}
		/* Every top-level entry of the client windows (apple_agx_api.h). */
		for (u_long va = 0; va < AGX_CLIENT_SHADER_END;
		    va += AGX_LOWER_TOP_SPAN)
			if ((error = agx_uat_alloc_tables(&sc->uat,
			    sc->uat.lower_root, va, va + AGX_UAT_PGSZ)) != 0) {
				device_printf(sc->dev, "lower link %#lx: %d\n",
				    va, error);
				return (error);
			}
		device_printf(sc->dev, "UAT: %d lower links, %d tables\n",
		    (int)nitems(links), sc->uat.ntables);
	}

	/* set_m5_start_control, before the coprocessors (macOS 27.0). */
	v = agx_sgx_read(sc, AGX_M5_START_CONTROL);
	device_printf(sc->dev, "M5 start control %#x\n", v);
	if (v != 0x30000 && v != 0x30001)
		return (EIO);
	agx_sgx_write(sc, AGX_M5_START_CONTROL, v | 1);

	/* T8142: GFX1 first, then GFX. */
	if ((error = agx_role_boot(sc, &sc->roles[1])) != 0 ||
	    (error = agx_role_boot(sc, &sc->roles[0])) != 0)
		return (error);

	/* Both run: clear the gating, where macOS clears it. */
	v = agx_sgx_read(sc, AGX_G17_GATING_CONTROL);
	agx_sgx_write(sc, AGX_G17_GATING_CONTROL, v & ~AGX_G17_GATING_DISABLE);
	device_printf(sc->dev, "G17 gating %#x -> %#x\n", v,
	    agx_sgx_read(sc, AGX_G17_GATING_CONTROL));

	if (agx_stage >= 3 && (error = agx_stage3(sc)) != 0)
		device_printf(sc->dev, "stage 3 failed: %d\n", error);
	else if (agx_stage >= 4 && (error = agx_stage4(sc)) != 0)
		device_printf(sc->dev, "stage 4 failed: %d\n", error);
	else if (agx_stage >= 4)
		device_printf(sc->dev, "stage 4: control record retired\n");
	sc->queues[0].qid = AGX_Q_QID;
	sc->queues[0].qos = AGX_Q_QOS;
	sc->queues[0].pool_row = AGX_Q_POOL_ROW;
	sc->queues[1].qid = AGX_Q_SECOND_ID;
	sc->queues[1].qos = AGX_Q_SECOND_QOS;
	sc->queues[1].pool_row = AGX_Q_SECOND_POOL_ROW;
	sc->queues[1].offset = AGX_Q_SECOND_OFFSET;
	for (i = 0; error == 0 && i < nitems(sc->queues); i++) {
		if (agx_stage >= 5)
			error = agx_stage5(sc, node, &sc->queues[i]);
		if (error == 0 && agx_stage >= 6)
			error = agx_stage6(sc, &sc->queues[i]);
		if (error != 0)
			device_printf(sc->dev, "compute queue %u setup: %d\n",
			    sc->queues[i].qid, error);
	}
	if (error == 0 && agx_stage >= 7) {
		int render_error = agx_stage7(sc);

		if (render_error != 0)
			device_printf(sc->dev, "stage 7: render configuration failed: %d\n",
			    render_error);
	}
	if (error == 0 && agx_stage >= 6) {
		/* The self-test's buffers leave the client windows. */
		agx_uat_unmap(&sc->uat, sc->uat.lower_root, AGX_JOB_CDM_VA,
		    AGX_UAT_PGSZ);
		agx_uat_unmap(&sc->uat, sc->uat.lower_root, AGX_JOB_PROGRAM_VA,
		    AGX_UAT_PGSZ);
		agx_uat_unmap(&sc->uat, sc->uat.lower_root, AGX_JOB_OUT_VA,
		    AGX_UAT_PGSZ);
		apple_agx_sc = sc;
	}

	/* Listen for a while; no InitData yet. */
	for (i = 0; i < 200; i++) {
		apple_rtkit_poll(sc->roles[1].rtk);
		apple_rtkit_poll(sc->roles[0].rtk);
		DELAY(10000);
	}
	device_printf(sc->dev, "stage 2: GFX %s, GFX1 %s after 2 s\n",
	    apple_rtkit_crashed(sc->roles[0].rtk) ? "crashed" : "alive",
	    apple_rtkit_crashed(sc->roles[1].rtk) ? "crashed" : "alive");
	return (0);
}

/*
 * regs.rs fault_banks: G15+ has two MMU fault banks behind one INFO/ADDR
 * window. Aurora reads them on a restart event, with the firmware halted.
 */
#define	AGX_FAULT_BANK_SELECT	0xd800
#define	AGX_FAULT_INFO		0xd8c0
#define	AGX_FAULT_ADDR		0xd8c8

static void
agx_report_mmu_fault(struct apple_agx_softc *sc)
{
	volatile uint64_t *sel = (volatile uint64_t *)(sc->sgx +
	    AGX_FAULT_BANK_SELECT);
	uint64_t saved, info, addr;
	u_int bank;

	saved = *sel;
	for (bank = 0; bank < 2; bank++) {
		*sel = bank;
		dsb(sy);
		info = *(volatile uint64_t *)(sc->sgx + AGX_FAULT_INFO);
		addr = *(volatile uint64_t *)(sc->sgx + AGX_FAULT_ADDR);
		device_printf(sc->dev, "MMU fault bank %u: info %#jx addr %#jx\n",
		    bank, (uintmax_t)info, (uintmax_t)addr);
	}
	*sel = saved;
	dsb(sy);
}

/*
 * recovery.rs service on the M5 firmware: the handshake words of the
 * primary status B (fw/m5/initdata.rs M5_RECOVERY_*). Status B sits at
 * 0x340 of the private cluster: its fwctl pointers (status B + 0x4ac0)
 * are the graph words at 0x4e00.
 */
#define	AGX_STATUS_B		0x340
#define	AGX_RECOVERY_INFO	(AGX_STATUS_B + 0x4298)
#define	AGX_RECOVERY_SLOT_MASK	(AGX_STATUS_B + 0x4ab8)
#define	AGX_RECOVERY_STATE	(AGX_STATUS_B + 0x4ae0)
#define	AGX_HOST_RECOVERY	(AGX_STATUS_B + 0x4af0)
#define	AGX_RECOVERY_RECORDS	256
#define	AGX_RECOVERY_POLLS	10000	/* 1 ms ticks per phase */

static volatile uint32_t *
agx_recovery_word(struct apple_agx_softc *sc, size_t off)
{

	dsb(sy);
	return ((volatile uint32_t *)(sc->graph_va[0] + off));
}

/* Firmware sends no notification for states 1 and 3; poll for them. */
static int
agx_recovery_wait(struct apple_agx_softc *sc, uint32_t want)
{
	uint32_t state;
	int i;

	for (i = 0; i < AGX_RECOVERY_POLLS; i++) {
		if (apple_rtkit_crashed(sc->roles[0].rtk) ||
		    apple_rtkit_crashed(sc->roles[1].rtk))
			return (EIO);
		state = *agx_recovery_word(sc, AGX_RECOVERY_STATE);
		if (state == want)
			return (0);
		if (want == 1 && state != 0)
			return (EIO);
		pause_sbt("agxrcv", SBT_1MS, 0, 0);
	}
	return (ETIMEDOUT);
}

/* Aligned 32-bit store between full barriers, checking the old value. */
static int
agx_recovery_transition(struct apple_agx_softc *sc, uint32_t expected,
    uint32_t next)
{
	volatile uint32_t *state = agx_recovery_word(sc, AGX_RECOVERY_STATE);

	if (*state != expected)
		return (EBUSY);
	dsb(sy);
	*state = next;
	dsb(sy);
	return (0);
}

/* A queue's pointer block: done, firmware cursor, the words at 0x40/0x50. */
static void
agx_recovery_cursors(struct apple_agx_softc *sc, const char *name,
    volatile uint32_t *p, uint32_t host)
{

	device_printf(sc->dev, "recovery: %s done %#x +30 %#x +40 %#x "
	    "+50 %#x entries %#x, host producer %#x\n", name, p[0],
	    p[0x30 / 4], p[0x40 / 4], p[0x50 / 4], p[0x60 / 4], host);
}

/*
 * The firmware's own log (the FW_LOG buffers): print its text runs, which
 * may say why it restarted. The format is unknown; runs of at least 12
 * printable bytes are shown, at most 40 per buffer, last ones first.
 */
static void
agx_recovery_fw_log(struct apple_agx_softc *sc)
{
	static const char *names[2] = { "fw_log0", "fw_log1" };
	char line[120];
	const uint8_t *b;
	size_t size, i, start, len;
	u_int shown, k;
	int o;

	for (k = 0; k < 2; k++) {
		if ((o = agx_graph_find(names[k])) < 0)
			continue;
		b = sc->graph_va[o];
		size = agx_graph_objects[o].size;
		dsb(sy);
		shown = 0;
		for (i = size; i > 0 && shown < 40; ) {
			while (i > 0 && (b[i - 1] < 0x20 || b[i - 1] > 0x7e))
				i--;
			start = i;
			while (start > 0 && b[start - 1] >= 0x20 &&
			    b[start - 1] <= 0x7e)
				start--;
			len = i - start;
			if (len >= 12) {
				len = MIN(len, sizeof(line) - 1);
				memcpy(line, b + start, len);
				line[len] = '\0';
				device_printf(sc->dev, "%s @%#zx: %s\n", names[k],
				    start, line);
				shown++;
			}
			i = start;
		}
	}
}

/* What a render was built from, to compare a failed one with a good one. */
static void
agx_recovery_dump(struct apple_agx_softc *sc, const char *what,
    const struct agx_render_slot *s)
{
	const struct agx_render_pass *p = &s->pass;
	const uint32_t *w = (const uint32_t *)p;
	u_int i;

	device_printf(sc->dev, "%s render %u context %u: %ux%u layers %u "
	    "utile %ux%u samples %u empty %d msaa %#jx ppp %#jx zls %#jx "
	    "samplers %u barriers %d writes %d bg %#x eot %#x, %u programs\n",
	    what, s->ordinal, s->context, p->width, p->height, p->layers,
	    p->utile_width, p->utile_height, 1u << p->samples_log2,
	    p->process_empty_tiles, (uintmax_t)p->msaa_control,
	    (uintmax_t)p->ppp_control, (uintmax_t)p->zls_control,
	    p->sampler_count, p->fragment_barriers, p->shader_writes_memory,
	    p->bg_offset, p->eot_offset, s->nprograms);
	for (i = 0; i < MIN(s->nprograms, nitems(s->programs)); i++) {
		const struct agx_render_program *g = &s->programs[i];

		device_printf(sc->dev, "%s  program %u: stage %u primary %u "
		    "secondary %u linked %u min %u aux %u tg %u ib %u ms %u "
		    "threads %u%s\n", what, i, g->stage, g->primary_bytes,
		    g->secondary_bytes, g->linked_bytes,
		    g->minimum_primary_bytes, g->auxiliary_count,
		    g->threadgroup_bytes, g->imageblock_bytes,
		    g->multisample_bytes, g->threads, g->task ? " task" : "");
	}
	for (i = 0; i + 8 <= sizeof(*p) / 4; i += 8)
		device_printf(sc->dev, "%s  +%03x: %08x %08x %08x %08x %08x "
		    "%08x %08x %08x\n", what, i * 4, w[i], w[i + 1], w[i + 2],
		    w[i + 3], w[i + 4], w[i + 5], w[i + 6], w[i + 7]);
}

/*
 * Services one restart. The firmware halts and waits in state 1; the host
 * acknowledges with 2, the firmware drains to 3 and resumes once the host
 * writes 0. The render the firmware blames fails; the others stay in
 * flight and are retired from their own stamps as the firmware replays.
 */
/* A pair's state at a restart, for the log. */
static void
agx_recovery_pair_log(struct agx_render *r)
{
	struct apple_agx_softc *sc = r->sc;
	char name[16];
	u_int i;

	snprintf(name, sizeof(name), "pair %u TA", r->index);
	agx_recovery_cursors(sc, name, (volatile uint32_t *)(r->graph +
	    AGX_RENDER_TA_CURSORS), r->items_end[0]);
	snprintf(name, sizeof(name), "pair %u 3D", r->index);
	agx_recovery_cursors(sc, name, (volatile uint32_t *)(r->graph +
	    AGX_RENDER_FRAGMENT_CURSORS), r->items_end[1]);
	for (i = 0; i < r->in_flight; i++) {
		struct agx_render_slot *s =
		    &r->slots[(r->oldest + i) % AGX_RENDER_SLOTS];

		device_printf(sc->dev, "recovery: pair %u in flight %u: render "
		    "%u context %u %ux%u stamp %#x, %jd ms old\n", r->index, i,
		    s->ordinal, s->context, s->width, s->height, s->stamp,
		    (intmax_t)((sbinuptime() - (s->deadline -
		    AGX_RENDER_TIMEOUT)) / SBT_1MS));
	}
}

/*
 * After a restart: renders a guilty record names by stamp fail; without a
 * match, the oldest render of the pair holding the blamed QID does. The
 * rest replay. ECANCELED tells the caller the GPU recovered: only that
 * render's client is lost. Returns the renders failed.
 */
static u_int
agx_recovery_pair_settle(struct agx_render *r, const uint32_t *guilty,
    const uint8_t *guilty_qid, u_int nguilty, int blamed, bool fallback)
{
	struct apple_agx_softc *sc = r->sc;
	struct agx_render_slot *s;
	u_int i, j, nfailed = 0;

	for (i = 0; i < r->in_flight; i++) {
		s = &r->slots[(r->oldest + i) % AGX_RENDER_SLOTS];
		for (j = 0; j < nguilty; j++)
			if (guilty[j] == s->stamp &&
			    (guilty_qid[j] == r->in->qids[0] ||
			    guilty_qid[j] == r->in->qids[1])) {
				s->error = ECANCELED;
				nfailed++;
				device_printf(sc->dev, "recovery: failing pair %u "
				    "render %u (context %u)\n", r->index,
				    s->ordinal, s->context);
				agx_recovery_dump(sc, "failed", s);
				agx_recovery_dump(sc, "good", &r->last_good);
				break;
			}
		s->deadline = sbinuptime() + AGX_RENDER_TIMEOUT;
	}
	if (fallback && nfailed == 0 && blamed >= 0 &&
	    (blamed == r->in->qids[0] || blamed == r->in->qids[1]) &&
	    r->in_flight != 0) {
		s = &r->slots[r->oldest];
		s->error = ECANCELED;
		nfailed++;
		device_printf(sc->dev, "recovery: no stamp matched; failing pair "
		    "%u's oldest render %u (context %u)\n", r->index, s->ordinal,
		    s->context);
	}
	return (nfailed);
}

static void
agx_recovery_task(void *arg, int pending)
{
	struct apple_agx_softc *sc = arg;
	struct agx_render *r;
	struct agx_render_done done[AGX_RENDER_SLOTS];
	volatile uint32_t *info;
	uint64_t gen;
	uint32_t flags, host, guilty[4];
	uint8_t guilty_qid[4];
	u_int i, k, listed = 0, n, nguilty = 0, nfailed = 0;
	int blamed, error;

	gen = sc->recovery_request;
	blamed = sc->recovery_blamed;
	host = *agx_recovery_word(sc, AGX_HOST_RECOVERY);
	device_printf(sc->dev, "recovery gen %ju blamed %d: state %u host %u "
	    "slot mask %#x\n", (uintmax_t)gen, blamed,
	    *agx_recovery_word(sc, AGX_RECOVERY_STATE), host,
	    *agx_recovery_word(sc, AGX_RECOVERY_SLOT_MASK));
	for (k = 0; k < AGX_RENDER_PAIRS; k++)
		if (sc->renders[k].installed)
			agx_recovery_pair_log(&sc->renders[k]);
	for (i = 0; i < nitems(sc->queues); i++)
		if (sc->queues[i].graph != NULL && sc->queues[i].qid != 0)
			agx_recovery_cursors(sc, "compute",
			    (volatile uint32_t *)(sc->queues[i].graph +
			    AGX_Q_CURSORS), sc->queues[i].items_end);

	agx_recovery_fw_log(sc);
	error = host != 0 ? EIO : agx_recovery_wait(sc, 1);
	if (error == 0)
		error = agx_recovery_transition(sc, 1, 2);
	if (error == 0)
		error = agx_recovery_wait(sc, 3);
	if (error == 0) {
		/* The M5 table need not be packed: walk all of it. */
		info = agx_recovery_word(sc, AGX_RECOVERY_INFO);
		for (i = 0; i < AGX_RECOVERY_RECORDS; i++) {
			flags = info[i * 2];
			if ((flags & 1) == 0)
				continue;
			if ((flags & 2) != 0 && nguilty < nitems(guilty)) {
				guilty_qid[nguilty] = (flags >> 4) & 0x7f;
				guilty[nguilty++] = info[i * 2 + 1];
			}
			if (listed++ < 8)
				device_printf(sc->dev, "recovery: record %u qid "
				    "%u%s%s stamp %#x\n", i, (flags >> 4) & 0x7f,
				    flags & 2 ? " guilty" : "",
				    flags & 4 ? " cancelled" : "", info[i * 2 + 1]);
		}
		error = agx_recovery_transition(sc, 3, 0);
	}
	if (error == 0 && *agx_recovery_word(sc, AGX_RECOVERY_STATE) != 0)
		error = EIO;

	mtx_lock(&sc->mtx);
	sc->recovering = false;
	if (error != 0) {
		device_printf(sc->dev, "recovery gen %ju failed: %d (state %u)\n",
		    (uintmax_t)gen, error,
		    *agx_recovery_word(sc, AGX_RECOVERY_STATE));
		for (k = 0; k < AGX_RENDER_PAIRS; k++)
			sc->renders[k].failed = true;
	} else {
		sc->recovery_accepted = gen + 1;
		sc->recoveries++;
		/* Stamps are per pair: match them all before any fallback. */
		for (k = 0; k < AGX_RENDER_PAIRS; k++)
			if (sc->renders[k].installed)
				nfailed += agx_recovery_pair_settle(&sc->renders[k],
				    guilty, guilty_qid, nguilty, blamed, false);
		for (k = 0; nfailed == 0 && k < AGX_RENDER_PAIRS; k++)
			if (sc->renders[k].installed)
				nfailed += agx_recovery_pair_settle(&sc->renders[k],
				    NULL, NULL, 0, blamed, true);
		device_printf(sc->dev, "recovery gen %ju done (%u records, %u "
		    "renders failed)\n", (uintmax_t)gen, listed, nfailed);
	}
	mtx_unlock(&sc->mtx);
	for (k = 0; k < AGX_RENDER_PAIRS; k++) {
		r = &sc->renders[k];
		if (!r->installed)
			continue;
		mtx_lock(&sc->mtx);
		n = agx_render_reap(r, done);
		mtx_unlock(&sc->mtx);
		agx_render_complete(done, n);
	}
}

/*
 * channel.rs drain_report: each role's trace (512 slots) and statistics
 * (256 slots) rings, state words at status A + 0x240 and + 0x280 like the
 * event ring's. Nothing reads the records yet: they are discarded, as
 * Aurora does when statistics are not wanted. Left full, the firmware
 * kept notifying (GFX1 hundreds of times a second, nothing in its events).
 */
#define	AGX_STATUS_TRACE_STATE	0x240
#define	AGX_STATUS_STATS_STATE	0x280
#define	AGX_TRACE_SLOTS		512
#define	AGX_STATS_SLOTS		256

static void
agx_drain_reports(struct agx_role *r)
{
	static const struct {
		uint32_t state, slots;
	} rings[2] = {
		{ AGX_STATUS_TRACE_STATE, AGX_TRACE_SLOTS },
		{ AGX_STATUS_STATS_STATE, AGX_STATS_SLOTS },
	};
	struct apple_agx_softc *sc = r->sc;
	int idx = r == &sc->roles[0] ? 0 : 1;
	volatile uint32_t *cons, *prod;
	uint8_t *status;
	uint32_t c, p;
	u_int i;

	if (sc->graph_va[0] == NULL)
		return;
	status = sc->graph_va[0] + agx_graph_status_a[idx];
	for (i = 0; i < nitems(rings); i++) {
		cons = (volatile uint32_t *)(status + rings[i].state);
		prod = (volatile uint32_t *)(status + rings[i].state +
		    AGX_RX_PRODUCER_OFFSET);
		dsb(sy);
		c = *cons;
		p = *prod;
		if (c >= rings[i].slots || p >= rings[i].slots) {
			if (r->reports[i]++ == 0)
				device_printf(sc->dev, "%s: bad %s cursors %u/%u\n",
				    r->name, i == 0 ? "trace" : "statistics", c, p);
			continue;
		}
		if (c == p)
			continue;
		r->reports[i] += (p - c + rings[i].slots) % rings[i].slots;
		*cons = p;
		dsb(sy);
	}
}

/*
 * Drain firmware events. A stamp event carries a bit mask of ready QIDs.
 * g17/fw/channels.rs: 4 restart {generation u64, stamp}, 6 TVB grow,
 * 7 compute fault, 12 context killed, 13 free-list grow. Only stamps are
 * serviced; every other event is logged in full.
 */
static void
agx_drain_events(struct agx_role *r)
{
	struct apple_agx_softc *sc = r->sc;
	int idx = r == &sc->roles[0] ? 0 : 1;
	uint8_t *status;
	volatile uint32_t *cons, *prod, *rec;
	uint32_t c, p;
	int n;

	if (sc->graph_va[0] == NULL)
		return;
	status = sc->graph_va[0] + agx_graph_status_a[idx];
	cons = (volatile uint32_t *)(status + AGX_STATUS_EVENT_STATE);
	prod = (volatile uint32_t *)(status + AGX_STATUS_EVENT_STATE +
	    AGX_RX_PRODUCER_OFFSET);
	for (n = 0; n < AGX_EVENT_SLOTS; n++) {
		c = *cons;
		p = *prod;
		if (c >= AGX_EVENT_SLOTS || p >= AGX_EVENT_SLOTS) {
			device_printf(sc->dev, "%s: bad event cursors %u/%u\n",
			    r->name, c, p);
			return;
		}
		if (c == p)
			break;
		dsb(sy);
		rec = (volatile uint32_t *)(status + AGX_STATUS_EVENT_RING +
		    c * AGX_EVENT_WORDS * 4);
		if (r->events++ < 48)
			device_printf(sc->dev, "%s: event %u tag %u: %#x %#x %#x "
			    "%#x %#x\n", r->name, c, rec[0], rec[1], rec[2],
			    rec[3], rec[4], rec[5]);
		if (rec[0] != AGX_EVENT_STAMP) {
			device_printf(sc->dev, "%s: event %u type %u: %#x %#x %#x "
			    "%#x %#x %#x %#x %#x\n", r->name, c, rec[0], rec[1],
			    rec[2], rec[3], rec[4], rec[5], rec[6], rec[7], rec[8]);
			if (rec[0] == AGX_EVENT_RESTART)
				agx_report_mmu_fault(sc);
		}
		/*
		 * recovery.rs State::request: restart {generation u64, stamp
		 * i32}; the stamp is the blamed QID, or -1. An older generation
		 * is a duplicate.
		 */
		if (rec[0] == AGX_EVENT_RESTART) {
			uint64_t gen = rec[1] | (uint64_t)rec[2] << 32;

			if (sc->renders[0].installed &&
			    gen >= sc->recovery_accepted && !sc->recovering) {
				sc->recovering = true;
				sc->recovery_request = gen;
				sc->recovery_blamed = (int32_t)rec[3];
				taskqueue_enqueue(taskqueue_thread,
				    &sc->recovery_task);
			}
		}
		/* A stamp event's mask names ready QIDs; each pair has two. */
		for (u_int k = 0; idx == 0 && rec[0] == AGX_EVENT_STAMP &&
		    k < AGX_RENDER_PAIRS; k++) {
			struct agx_render *rr = &sc->renders[k];
			u_int q0, q1;

			if (!rr->installed)
				continue;
			q0 = rr->in->qids[0];
			q1 = rr->in->qids[1];
			if ((rec[1 + q0 / 32] & (1u << (q0 % 32))) == 0 &&
			    (rec[1 + q1 / 32] & (1u << (q1 % 32))) == 0)
				continue;
			rr->stamp_events++;
			if (rr->in_flight != 0)
				taskqueue_enqueue(taskqueue_thread,
				    &rr->reap_task);
		}
		if (idx == 0 && rec[0] == AGX_EVENT_STAMP) {
			for (u_int i = 0; i < nitems(sc->queues); i++) {
				struct agx_compute_queue *queue = &sc->queues[i];
				u_int qid = queue->qid;

				if (qid != 0 &&
				    (rec[1 + qid / 32] & (1u << (qid % 32)))) {
					queue->stamp_events++;
					wakeup(&queue->stamp_events);
				}
			}
		}
		dsb(sy);
		*cons = (c + 1) & (AGX_EVENT_SLOTS - 1);
		dsb(sy);
	}
}

static void
agx_intr(void *arg)
{
	struct agx_role *r = arg;
	struct apple_agx_softc *sc = r->sc;

	mtx_lock(&sc->mtx);
	r->irqs++;
	apple_rtkit_poll(r->rtk);
	mtx_unlock(&sc->mtx);
}

/* The mailbox node's "recv-not-empty" interrupt, as one of ours. */
static int
agx_setup_irq(struct apple_agx_softc *sc, int idx)
{
	struct agx_role *r = &sc->roles[idx];
	pcell_t mboxes[2], *cells;
	phandle_t mb, iparent;
	int n, which, ncells, error;
	u_int irq;

	if (OF_getencprop(ofw_bus_get_node(sc->dev), "mboxes", mboxes,
	    sizeof(mboxes)) != sizeof(mboxes))
		return (ENXIO);
	mb = OF_node_from_xref(mboxes[idx]);
	if (ofw_bus_find_string_index(mb, "interrupt-names", "recv-not-empty",
	    &which) != 0)
		return (ENXIO);
	iparent = ofw_bus_find_iparent(mb);	/* an xref */
	if (OF_searchencprop(OF_node_from_xref(iparent), "#interrupt-cells",
	    &ncells, sizeof(ncells)) == -1 || ncells < 1 || ncells > 4)
		return (ENXIO);
	n = OF_getencprop_alloc_multi(mb, "interrupts", sizeof(*cells),
	    (void **)&cells);
	if (n < (which + 1) * ncells) {
		OF_prop_free(cells);
		return (ENXIO);
	}
	irq = ofw_bus_map_intr(sc->dev, iparent, ncells,
	    &cells[which * ncells]);
	OF_prop_free(cells);
	r->irq_rid = idx;
	if ((error = bus_set_resource(sc->dev, SYS_RES_IRQ, r->irq_rid, irq,
	    1)) != 0)
		return (error);
	r->irq = bus_alloc_resource_any(sc->dev, SYS_RES_IRQ, &r->irq_rid,
	    RF_ACTIVE);
	if (r->irq == NULL)
		return (ENXIO);
	return (bus_setup_intr(sc->dev, r->irq, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, agx_intr, r, &r->irq_tag));
}

/* Publish IdlePowerOff(policy) from userland: exercises doorbell and IRQ. */
static int
agx_sysctl_idle(SYSCTL_HANDLER_ARGS)
{
	struct apple_agx_softc *sc = arg1;
	uint32_t record[AGX_CONTROL_RECORD_SIZE / 4] = {
		AGX_CTL_IDLE_POWER_OFF,
	};
	uint32_t target;
	int error, policy = -1;

	error = sysctl_handle_int(oidp, &policy, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (policy != AGX_IDLE_KEEP_POWERED && policy != AGX_IDLE_ALLOW_POWER_OFF)
		return (EINVAL);
	record[1] = policy;
	mtx_lock(&sc->mtx);
	error = sc->running ? agx_publish_control(sc, record, true, &target) :
	    ENXIO;
	mtx_unlock(&sc->mtx);
	return (error);
}

/*
 * Task payload layout probe: 1 clears the mesh storage, 2 reports where
 * each float key 1.0..64.0 (vktaskgrid's) landed and the highest word
 * written. Debug only: read while no task render runs.
 */
static int
agx_sysctl_mesh_probe(SYSCTL_HANDLER_ARGS)
{
	struct apple_agx_softc *sc = arg1;
	const uint32_t *w;
	size_t i, n, first[65], last[65], count[65], high = 0;
	uint32_t bits[65];
	int error, op = 0, k, e;

	error = sysctl_handle_int(oidp, &op, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	/* The last task client's storage: submit records it in pair 0. */
	if (sc->renders[0].mesh_storage == NULL)
		return (ENXIO);
	if (op == 1) {
		memset(sc->renders[0].mesh_storage, 0,
		    AGX_RENDER_MESH_STORAGE_SIZE);
		return (0);
	}
	if (op != 2)
		return (EINVAL);
	for (k = 1; k <= 64; k++) {
		for (e = 0; (2 << e) <= k; e++)
			;
		bits[k] = (uint32_t)(127 + e) << 23 |
		    (((uint32_t)k << (23 - e)) & 0x7fffff);
		first[k] = last[k] = count[k] = 0;
	}
	w = (const uint32_t *)sc->renders[0].mesh_storage;
	n = AGX_RENDER_MESH_STORAGE_SIZE / 4;
	for (i = 0; i < n; i++) {
		if (w[i] == 0)
			continue;
		high = i * 4;
		for (k = 1; k <= 64; k++) {
			if (w[i] != bits[k])
				continue;
			if (count[k]++ == 0)
				first[k] = i * 4;
			last[k] = i * 4;
		}
	}
	device_printf(sc->dev, "mesh probe: highest nonzero %#zx\n", high);
	for (k = 1; k <= 64; k++)
		if (count[k] != 0)
			device_printf(sc->dev, "mesh probe: key %d: %zu words, "
			    "%#zx..%#zx\n", k, count[k], first[k], last[k]);
	return (0);
}

static int
agx_sysctl_control(SYSCTL_HANDLER_ARGS)
{
	struct apple_agx_softc *sc = arg1;
	volatile uint32_t *st;
	char buf[48];

	if (!sc->running)
		return (ENXIO);
	st = agx_control_state(sc);
	snprintf(buf, sizeof(buf), "%u %u %u", st[0], st[0x10 / 4],
	    st[0x20 / 4]);
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static void
agx_sysctls(struct apple_agx_softc *sc)
{
	struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(sc->dev);
	struct sysctl_oid_list *t = SYSCTL_CHILDREN(
	    device_get_sysctl_tree(sc->dev));
	int i;

	SYSCTL_ADD_PROC(ctx, t, OID_AUTO, "idle_policy",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE, sc, 0,
	    agx_sysctl_idle, "I", "Publish IdlePowerOff (0 keep, 1 allow)");
	SYSCTL_ADD_PROC(ctx, t, OID_AUTO, "mesh_probe",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE, sc, 0,
	    agx_sysctl_mesh_probe, "I", "Task mesh storage probe (1 clear, 2 scan)");
	/* Every pair's kicks read pair 0's. */
	sc->renders[0].mesh_chunks = AGX_RENDER_MESH_STORAGE_CHUNKS;
	SYSCTL_ADD_UINT(ctx, t, OID_AUTO, "mesh_chunks", CTLFLAG_RW,
	    &sc->renders[0].mesh_chunks, 0,
	    "Task mesh storage register chunk count");
	SYSCTL_ADD_PROC(ctx, t, OID_AUTO, "control",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    agx_sysctl_control, "A", "Control ring consumer, unk_10, producer");
	SYSCTL_ADD_UINT(ctx, t, OID_AUTO, "compute_peak_in_flight", CTLFLAG_RD,
	    &sc->compute_peak_in_flight, 0, "Peak compute kicks awaiting retirement");
	SYSCTL_ADD_UINT(ctx, t, OID_AUTO, "recoveries", CTLFLAG_RD,
	    &sc->recoveries, 0, "Firmware restarts serviced");
	for (i = 0; i < AGX_RENDER_PAIRS; i++) {
		struct agx_render *rp = &sc->renders[i];
		struct sysctl_oid *node;
		char name[8];

		snprintf(name, sizeof(name), "pair%d", i);
		node = SYSCTL_ADD_NODE(ctx, t, OID_AUTO, name,
		    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "Render pair");
		SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(node), OID_AUTO,
		    "peak_in_flight", CTLFLAG_RD, &rp->peak_in_flight, 0,
		    "Peak renders awaiting retirement");
		SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(node), OID_AUTO, "renders",
		    CTLFLAG_RD, &rp->ordinal, 0, "Renders published");
		SYSCTL_ADD_UINT(ctx, SYSCTL_CHILDREN(node), OID_AUTO, "retired",
		    CTLFLAG_RD, &rp->retired, 0, "Renders retired");
		SYSCTL_ADD_BOOL(ctx, SYSCTL_CHILDREN(node), OID_AUTO, "installed",
		    CTLFLAG_RD, &rp->installed, 0, "Pair installed");
	}
	for (i = 0; i < 2; i++) {
		struct agx_role *r = &sc->roles[i];

		SYSCTL_ADD_ULONG(ctx, t, OID_AUTO, i == 0 ? "gfx_irqs" :
		    "gfx1_irqs", CTLFLAG_RD, &r->irqs, "Mailbox interrupts");
		SYSCTL_ADD_ULONG(ctx, t, OID_AUTO, i == 0 ? "gfx_events" :
		    "gfx1_events", CTLFLAG_RD, &r->events, "Firmware events");
		SYSCTL_ADD_ULONG(ctx, t, OID_AUTO, i == 0 ? "gfx_trace" :
		    "gfx1_trace", CTLFLAG_RD, &r->reports[0],
		    "Trace records discarded");
		SYSCTL_ADD_ULONG(ctx, t, OID_AUTO, i == 0 ? "gfx_statistics" :
		    "gfx1_statistics", CTLFLAG_RD, &r->reports[1],
		    "Statistics records discarded");
		SYSCTL_ADD_UINT(ctx, t, OID_AUTO, i == 0 ? "gfx_msgs" :
		    "gfx1_msgs", CTLFLAG_RD, &r->msgs, 0, "Endpoint messages");
	}
}

static int
apple_agx_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Apple M5 GPU");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_agx_detach(device_t dev)
{
	struct apple_agx_softc *sc = device_get_softc(dev);

	if (sc->sgx_size != 0)
		pmap_unmapdev(sc->sgx, sc->sgx_size);
	if (sc->asc_size != 0)
		pmap_unmapdev(sc->asc, sc->asc_size);
	sc->sgx_size = sc->asc_size = 0;
	return (0);
}

static int
apple_agx_attach(device_t dev)
{
	struct apple_agx_softc *sc = device_get_softc(dev);
	phandle_t node = ofw_bus_get_node(dev);
	int error;

	sc->dev = dev;
	mtx_init(&sc->mtx, "apple_agx", NULL, MTX_DEF);

	/* Check the firmware before touching any GPU register. */
	if ((error = agx_identify_fw(sc, node)) != 0)
		goto fail;

	/*
	 * The SGX window contains the ASC's, the mailboxes' and the GPU
	 * PMGR's, so map both directly rather than through rman.
	 */
	if ((error = agx_map(node, 0, &sc->asc, &sc->asc_size)) != 0 ||
	    (error = agx_map(node, 1, &sc->sgx, &sc->sgx_size)) != 0) {
		device_printf(dev, "cannot map registers: %d\n", error);
		goto fail;
	}

	/*
	 * The SGX registers fault while ps_gfx is gated; hold it on first
	 * (pmpdash.c asahi_j813_power_on_gfx: top state, auto off).
	 */
	if ((error = apple_pmgr_power_on(dev)) != 0) {
		device_printf(dev, "cannot power on GFX: %d\n", error);
		goto fail;
	}

	if ((error = agx_read_id(sc)) != 0)
		goto fail;

	device_printf(dev, "stage 1 done\n");
	if (agx_stage < 2)
		return (0);
	/*
	 * Once a coprocessor runs, nothing it was given may go away: keep
	 * the device attached whatever happens from here on.
	 */
	mtx_lock(&sc->mtx);
	error = agx_stage2(sc, node);
	mtx_unlock(&sc->mtx);
	if (error != 0) {
		device_printf(dev, "stage 2 stopped: %d\n", error);
		return (0);
	}
	if (sc->running) {
		for (int i = 0; i < 2; i++)
			if ((error = agx_setup_irq(sc, i)) != 0)
				device_printf(dev, "%s: no interrupt: %d\n",
				    sc->roles[i].name, error);
		agx_sysctls(sc);
		device_printf(dev, "interrupts on; firmware running\n");
	}
	return (0);
fail:
	apple_agx_detach(dev);
	return (error);
}

static device_method_t apple_agx_methods[] = {
	DEVMETHOD(device_probe,		apple_agx_probe),
	DEVMETHOD(device_attach,	apple_agx_attach),
	DEVMETHOD(device_detach,	apple_agx_detach),
	DEVMETHOD_END
};

static DEFINE_CLASS_0(apple_agx, apple_agx_driver, apple_agx_methods,
    sizeof(struct apple_agx_softc));

DRIVER_MODULE(apple_agx, simplebus, apple_agx_driver, NULL, NULL);
