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
#define	AGX_MSG_EVENTS		(0x42ull << 48)
/* Event ring in each role's status block (fw/initdata.rs status, channels.rs). */
#define	AGX_STATUS_EVENT_STATE	0x40
#define	AGX_STATUS_EVENT_RING	0x2c0
#define	AGX_RX_PRODUCER_OFFSET	0x20
#define	AGX_EVENT_SLOTS		256
#define	AGX_EVENT_WORDS		18
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
/* 1: identify; 2: start both coprocessors; 3: InitData; 4: control. */
static int agx_stage = 1;
SYSCTL_INT(_hw_apple_agx, OID_AUTO, stage, CTLFLAG_RDTUN, &agx_stage, 0,
    "Bring-up stage to run at attach");

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
};

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

static void
agx_recv(void *arg, uint32_t ep, uint64_t msg)
{
	struct agx_role *r = arg;

	/* recv_message_early: the InitData acknowledgment, then events. */
	if (ep == AGX_EP_FIRMWARE && msg >> 48 == AGX_MSG_INITDATA_ACK) {
		r->init_ack = true;
	} else if (ep == AGX_EP_FIRMWARE && msg == AGX_MSG_EVENTS) {
		agx_drain_events(r);
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
		};
		for (i = 0; i < (int)nitems(links); i++)
			if ((error = agx_uat_alloc_tables(&sc->uat,
			    sc->uat.lower_root, links[i],
			    links[i] + AGX_UAT_PGSZ)) != 0) {
				device_printf(sc->dev, "lower link %#lx: %d\n",
				    links[i], error);
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
 * Events::next, logging each record: no event is acted on yet. Stamp and
 * fault handling come with the queues.
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
		if (r->events++ < 32)
			device_printf(sc->dev, "%s: event %u tag %u: %#x %#x %#x "
			    "%#x %#x\n", r->name, c, rec[0], rec[1], rec[2],
			    rec[3], rec[4], rec[5]);
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
	SYSCTL_ADD_PROC(ctx, t, OID_AUTO, "control",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    agx_sysctl_control, "A", "Control ring consumer, unk_10, producer");
	for (i = 0; i < 2; i++) {
		struct agx_role *r = &sc->roles[i];

		SYSCTL_ADD_ULONG(ctx, t, OID_AUTO, i == 0 ? "gfx_irqs" :
		    "gfx1_irqs", CTLFLAG_RD, &r->irqs, "Mailbox interrupts");
		SYSCTL_ADD_ULONG(ctx, t, OID_AUTO, i == 0 ? "gfx_events" :
		    "gfx1_events", CTLFLAG_RD, &r->events, "Firmware events");
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
