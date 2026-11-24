/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple AOP (Always-On Processor), T8142 (M5). Follows aurora's
 * drivers/soc/apple/aop.rs and aop_ring.rs (HW_CFG_T8142) with the setup
 * port's registers from drivers/soc/apple/mailbox.c
 * (apple_mbox_t8140_aop_setup_hw).
 *
 * The AOP is an RTKit coprocessor whose firmware iBoot preloads and leaves
 * halted. Boot, in aurora's order:
 *
 *  1. The setup port (a second, small mailbox, "setup" in mbox-names) gets
 *     an arena of message pages mapped through its own DART stream, and
 *     is serviced from its interrupt from then on: it runs a HELLO /
 *     endpoint map handshake while the firmware boots.
 *  2. apple_rtkit_boot() starts the CPU and runs the RTKit handshake up to
 *     AP power on.
 *  3. The setup port's AP power request; then every service endpoint its
 *     endpoint map named asks for, and gets, a message page and a reply
 *     window out of the arena.
 *  4. Every AFK endpoint (0x20..0x30) the RTKit endpoint map names is
 *     started; each asks for a shared buffer, describes its TX and RX rings
 *     in it and then announces its EPIC services on the RX ring.
 *
 * The children (the DT node's subnodes: "audio") attach after that and
 * reach their services through apple_aop.h.
 *
 * Both mailboxes are serviced from their "recv-not-empty" interrupts (the
 * mailbox nodes' lines, as aurora's mailbox.c requests them), and callers
 * sleep until a handler completes their call. The mailboxes have no
 * interrupt controls of their own: the lines are level and drop once the
 * FIFO is drained; the AIC masks them while the handler's ithread runs.
 * A kernel thread delivers reports to listeners outside the lock and, as a
 * safety net for a miswired line, polls every hw.apple_aop.poll_ms (100 ms
 * by default, 0 for never). A line that keeps firing with nothing to read
 * is torn down, and the thread then polls every millisecond as before.
 *
 * DMA: the AOP's DART (aop_dart) is set up by iBoot and holds the
 * firmware's own mappings, so its stream tables are never replaced: host
 * buffers go into page tables of ours published only into vacant entries of
 * the live tables, inside the window apple,dma-range names (aurora's
 * locked-DART handling in drivers/iommu/apple-dart.c, minimally; the
 * apple_dart_fw_*() calls of apple_dart.h).
 *
 * Not ported (no consumer yet; the structure keeps room for them): the ALS
 * calibration and the source ring on the setup port, fake-HID/gyro/accel
 * listeners, system sleep, the bootargs patching and firmware-region checks
 * of other SoCs.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/queue.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/time.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_param.h>
#include <vm/pmap.h>

#include <machine/atomic.h>
#include <machine/bus.h>
#include <machine/cpufunc.h>
#include <machine/vmparam.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>
#include <dev/fdt/simplebus.h>

#include <arm64/apple/apple_aop.h>
#include <arm64/apple/apple_dart.h>
#include <arm64/apple/apple_rtkit.h>

/*
 * Newer than some running kernels: a module built from this file must still
 * load on them, and then starts the endpoints of aurora's T8140 map.
 */
#pragma weak apple_rtkit_has_endpoint

static MALLOC_DEFINE(M_APPLEAOP, "apple_aop", "Apple AOP");

#define	ASC_CPU_CONTROL			0x44
#define	ASC_CPU_CONTROL_RUN		0x10

#define	AOP_MAX_CALLS			8
#define	AFK_ENDPOINT_START		0x20
#define	AFK_ENDPOINT_COUNT		0x11
#define	AFK_OPC_GET_BUF			0x89
#define	AFK_OPC_INIT			0x80
#define	AFK_OPC_INIT_RX			0x8b
#define	AFK_OPC_INIT_TX			0x8a
#define	AFK_OPC_INIT_UNK		0x8c
#define	AFK_OPC_SEND			0xa2
#define	AFK_OPC_START_ACK		0x86
#define	AFK_OPC_SHUTDOWN_ACK		0xc1
#define	AFK_OPC_SLEEP_ACK		0x87
#define	AFK_OPC_RECV			0x85
#define	AFK_MSG_GET_BUF_ACK		(0xa1ull << 48)
#define	AFK_MSG_INIT			((uint64_t)AFK_OPC_INIT << 48)
#define	AFK_MSG_INIT_ACK		(0xa0ull << 48)
#define	AFK_MSG_START			(0xa3ull << 48)
#define	AFK_MSG_SHUTDOWN		(0xc0ull << 48)
#define	AFK_RB_BLOCK_STEP		0x40
#define	AFK_SHUTDOWN_TIMEOUT_MS		5000

#define	EPIC_TYPE_NOTIFY		0
#define	EPIC_CATEGORY_REPORT		0x00
#define	EPIC_CATEGORY_NOTIFY		0x10
#define	EPIC_CATEGORY_REPLY		0x20
#define	EPIC_SUBTYPE_STD_SERVICE	0xc0
#define	EPIC_SUBTYPE_FAKEHID_REPORT	0xc4
#define	EPIC_SUBTYPE_GYRO_REPORT	0xc1
#define	EPIC_SUBTYPE_ACCEL_REPORT	0xc2
#define	EPIC_SUBTYPE_RETCODE		0x84
#define	EPIC_SUBTYPE_RETCODE_PAYLOAD	0xa0
#define	EPIC_SUBTYPE_STRING		0x8a
#define	EPIC_SUBTYPE_CLIENT_OPEN_REPLY	0x99
#define	EPIC_SUBTYPE_CLIENT_CLOSE_REPLY	0x9a

#define	FOURCC(a, b, c, d)		((uint32_t)(a) << 24 | \
					    (uint32_t)(b) << 16 | \
					    (uint32_t)(c) << 8 | (uint32_t)(d))
#define	QE_MAGIC1			FOURCC(' ', 'P', 'O', 'I')
#define	QE_MAGIC2			FOURCC(' ', 'P', 'O', 'A')
#define	QEH_SIZE			16

/*
 * The EPIC header, 40 bytes. Version 2 sub-headers carry the tag at 24;
 * version 4 (this firmware) a timestamp there and the tag at 32. Both have
 * the reply capacity (inline_len) at 36.
 */
#define	EPIC_HDR_SIZE			40
#define	EPIC_HDR_VERSION		0
#define	EPIC_HDR_SEQ			1
#define	EPIC_HDR_LENGTH			16
#define	EPIC_HDR_SUB_VERSION		20
#define	EPIC_HDR_CATEGORY		21
#define	EPIC_HDR_SUBTYPE		22
#define	EPIC_HDR_TAG_V2			24
#define	EPIC_HDR_TAG_V4			32
#define	EPIC_HDR_INLINE_LEN		36

#define	EPIC_ANNOUNCE_NAME_LEN		32
#define	EPIC_ANNOUNCE_CHANNEL_OFFSET	32
#define	EPIC_ANNOUNCE_LEN		44

/* The setup port (aurora's SETUP_* constants). */
/* The ASC mailbox's receive control word (apple_rtkit.c's MBOX_I2A_CONTROL). */
#define	ASC_MBOX_I2A_CONTROL		0x114

#define	SETUP_A2I_CONTROL		0x000
#define	SETUP_I2A_CONTROL		0x004
#define	SETUP_A2I_SEND0			0x180
#define	SETUP_A2I_SEND1			0x188
#define	SETUP_I2A_RECV0			0x1b0
#define	SETUP_I2A_RECV1			0x1b8
#define	MBOX_CONTROL_FULL		(1u << 16)
#define	MBOX_CONTROL_EMPTY		(1u << 17)
#define	MBOX_TX_TIMEOUT_US		500000

#define	SETUP_PAGE			0x4000
#define	SETUP_ARENA_PAGES		13
#define	SETUP_MGMT_EP			0
#define	SETUP_TYPE_HELLO		1
#define	SETUP_TYPE_HELLO_REPLY		2
#define	SETUP_TYPE_UNK3			3
#define	SETUP_TYPE_UNK3_REPLY		4
#define	SETUP_TYPE_PWR_ACK		7
#define	SETUP_TYPE_EPMAP		8
#define	SETUP_TYPE_AP_PWR		0xb
#define	SETUP_HELLO_VERSION		0xc000cull
#define	SETUP_EPMAP_LAST		(1ull << 51)
#define	SETUP_AP_PWR_INIT		0x220
#define	SETUP_AP_PWR_ON			0x20
#define	SETUP_BUFFER_REQUEST		0x12ull
#define	SETUP_BUFFER_REQUEST_ACK	(SETUP_BUFFER_REQUEST << 56)
#define	SETUP_BUFFER_ENTRY_SIZE		16
#define	SETUP_MAX_ENDPOINTS		16
#define	SETUP_BOOT_TIMEOUT_MS		15000
#define	SETUP_MAP_TIMEOUT_MS		1000
#define	SETUP_REQUEST_TYPE		(3ull << 60)
#define	SETUP_REQUEST_DONE		(4ull << 60)

/* Shared buffers: DART pages, below the 2^42 a DART PTE can address. */
#define	AOP_DMA_PAGE			(16 * 1024)
#define	AOP_DMA_PA_LIMIT		(1ul << 42)

#define	AOP_MAX_SERVICES		32
#define	AOP_MAX_LISTENERS		16

#define	MS_SBT(ms)			((sbintime_t)(ms) * SBT_1MS)

struct aop_ring {
	size_t		offset;		/* in the endpoint's shared buffer */
	size_t		block;
	size_t		size;		/* bytes of entries */
};

struct aop_call {
	bool		used;
	bool		abandoned;	/* the caller gave up; tag reserved */
	bool		done;
	uint32_t	retcode;
	void		*reply;
	size_t		replycap;	/* 0 without a reply buffer */
	size_t		replylen;
	bool		want_reply;
};

struct aop_afk_ep {
	uint8_t		index;
	bool		started;
	bool		shutdown_acked;
	bool		turn;		/* a call is in flight (v4: one at once) */
	uint8_t		*iomem;
	vm_paddr_t	iomem_pa;
	uint64_t	iomem_iova;
	size_t		iomem_size;
	uint8_t		*msgbuf;	/* one received entry */
	bool		has_tx, has_rx;
	struct aop_ring	tx, rx;
	uint16_t	seq;
	struct aop_call	calls[AOP_MAX_CALLS];
	uint32_t	dropped;
};

struct aop_setup_ep {
	bool		valid;
	uint8_t		ep;
	uint64_t	tx_iova;
	uint64_t	rx_iova;
	bool		busy;
	bool		abandoned;
	bool		has_reply;
	uint64_t	reply;
};

struct aop_setup {
	uint8_t		*arena;
	vm_paddr_t	arena_pa;
	uint64_t	arena_iova;
	u_int		next_page;
	struct aop_setup_ep endpoints[SETUP_MAX_ENDPOINTS];
	uint8_t		expected[SETUP_MAX_ENDPOINTS];
	u_int		expected_count;
	bool		epmap_endpoints;	/* T8142: from the EPMAP */
	bool		wait_system;
	bool		map_done;
	bool		ap_ready;
	bool		power_sent;
	bool		failed;
};

struct aop_service_rec {
	char		name[EPIC_ANNOUNCE_NAME_LEN + 1];
	struct apple_aop_service svc;
};

struct aop_listener {
	bool		used;
	struct apple_aop_service svc;
	uint16_t	subtype;
	apple_aop_report_cb_t *cb;
	void		*arg;
};

struct aop_report {
	STAILQ_ENTRY(aop_report) link;
	int		listener;
	uint16_t	subtype;
	size_t		len;
	uint8_t		data[];
};

/* The mailboxes' recv-not-empty lines. */
#define	AOP_IRQ_SETUP		0
#define	AOP_IRQ_RTKIT		1
#define	AOP_IRQ_COUNT		2
/* Consecutive interrupts that found nothing to read before a line is cut. */
#define	AOP_IRQ_STORM_LIMIT	1000

struct apple_aop_softc;

struct aop_irq {
	struct apple_aop_softc	*sc;
	int			which;
	struct resource		*res;
	void			*tag;
	bool			live;		/* handler installed */
	bool			kill;		/* the thread tears it down */
	u_int			empty_run;
	u_long			count;
};

static int aop_poll_ms = 100;
SYSCTL_NODE(_hw, OID_AUTO, apple_aop, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Apple AOP");
SYSCTL_INT(_hw_apple_aop, OID_AUTO, poll_ms, CTLFLAG_RWTUN, &aop_poll_ms, 0,
    "Fallback mailbox poll period with interrupts (ms, 0 = none)");

struct apple_aop_softc {
	struct simplebus_softc	sb;		/* must be first */
	device_t		dev;
	struct mtx		mtx;
	vm_offset_t		asc, mbox, setup_mbox;
	bus_size_t		asc_size, mbox_size, setup_size;
	struct apple_rtkit	*rtk;
	bool			rtk_live;	/* the handlers service it */
	bool			running;	/* boot completed */
	bool			crashed;
	bool			closing;
	bool			epic_v4;
	struct proc		*thread;
	bool			thread_stop;
	bool			thread_running;
	struct aop_irq		irqs[AOP_IRQ_COUNT];
	bool			polled;		/* a line is missing: 1 ms poll */
	u_long			poll_hits;	/* messages the slow poll found */
	struct apple_dart_fw	*dart[2];	/* the AOP's, the setup port's */
	struct aop_afk_ep	eps[AFK_ENDPOINT_COUNT];
	struct aop_setup	setup;
	struct aop_service_rec	services[AOP_MAX_SERVICES];
	int			nservices;
	struct aop_listener	listeners[AOP_MAX_LISTENERS];
	STAILQ_HEAD(, aop_report) reports;
	int			delivering;	/* listener running, by index+1 */
};

#define	RD4(va, o)		(*(volatile uint32_t *)((va) + (o)))
#define	WR4(va, o, v)		(*(volatile uint32_t *)((va) + (o)) = (v))
#define	RD8(va, o)		(*(volatile uint64_t *)((va) + (o)))
#define	WR8(va, o, v)		(*(volatile uint64_t *)((va) + (o)) = (v))

static int	aop_afk_index(uint32_t ep);

/*
 * Sleep on chan until woken or deadline (an sbinuptime); EWOULDBLOCK once it
 * has passed. The interrupt handlers (and the thread) wake sc after every
 * pass over the mailboxes.
 */
static int
aop_sleep(struct apple_aop_softc *sc, sbintime_t deadline, const char *wmesg)
{
	sbintime_t now;

	now = getsbinuptime();
	if (now >= deadline)
		return (EWOULDBLOCK);
	(void)msleep_sbt(sc, &sc->mtx, 0, wmesg, deadline - now, SBT_1MS, 0);
	return (getsbinuptime() >= deadline ? EWOULDBLOCK : 0);
}

/* ---- DMA through the AOP's DART ---- */

/*
 * Uncached, physically contiguous, 16K-aligned memory below 2^42, for the
 * buffers the AOP shares. The zeroes kmem left in the
 * cacheable direct map are written back and dropped first, so no dirty line
 * can land on the memory later.
 */
static void *
aop_alloc_uncached(size_t size, vm_paddr_t *pap)
{
	void *va;
	vm_paddr_t pa;

	va = kmem_alloc_contig(size, M_NOWAIT | M_ZERO, 0,
	    AOP_DMA_PA_LIMIT - 1, AOP_DMA_PAGE, 0,
	    VM_MEMATTR_UNCACHEABLE);
	if (va == NULL)
		return (NULL);
	pa = pmap_kextract((vm_offset_t)va);
	cpu_dcache_wbinv_range((void *)PHYS_TO_DMAP(pa), size);
	memset(va, 0, size);
	dsb(sy);
	*pap = pa;
	return (va);
}

/* A buffer the AOP shares: uncached, mapped into the stream. */
static void *
aop_dma_alloc(struct apple_aop_softc *sc, struct apple_dart_fw *dart,
    size_t size, vm_paddr_t *pap, uint64_t *iovap)
{
	void *va;
	int error;

	size = roundup2(size, AOP_DMA_PAGE);
	va = aop_alloc_uncached(size, pap);
	if (va == NULL)
		return (NULL);
	error = apple_dart_fw_map(dart, *pap, size, iovap);
	if (error != 0) {
		/* It may be partly mapped: keep it. */
		device_printf(sc->dev, "DMA map of %#zx failed: %d\n", size,
		    error);
		return (NULL);
	}
	return (va);
}

/* RTKit buffers: syslog/ioreport (ours) and the crashlog (the firmware's). */
static int
aop_rtkit_buf_map(void *arg, vm_paddr_t pa, size_t size, uint64_t *iova)
{
	struct apple_aop_softc *sc = arg;

	return (apple_dart_fw_map(sc->dart[0], pa, size, iova));
}

static int
aop_rtkit_buf_lookup(void *arg, uint64_t iova, size_t size, vm_paddr_t *pa)
{
	struct apple_aop_softc *sc = arg;

	return (apple_dart_fw_lookup(sc->dart[0], iova, size, pa));
}

/* ---- the setup port ---- */

static int
aop_setup_send(struct apple_aop_softc *sc, uint8_t ep, uint64_t word)
{
	int i;

	for (i = 0; RD4(sc->setup_mbox, SETUP_A2I_CONTROL) & MBOX_CONTROL_FULL;
	    i++) {
		if (i == MBOX_TX_TIMEOUT_US) {
			device_printf(sc->dev, "setup port: mailbox full\n");
			return (ETIMEDOUT);
		}
		DELAY(1);
	}
	wmb();
	WR8(sc->setup_mbox, SETUP_A2I_SEND0, word);
	WR8(sc->setup_mbox, SETUP_A2I_SEND1, ep);
	return (0);
}

static bool
aop_setup_recv(struct apple_aop_softc *sc, uint8_t *ep, uint64_t *word)
{
	if (RD4(sc->setup_mbox, SETUP_I2A_CONTROL) & MBOX_CONTROL_EMPTY)
		return (false);
	*word = RD8(sc->setup_mbox, SETUP_I2A_RECV0);
	*ep = RD8(sc->setup_mbox, SETUP_I2A_RECV1) & 0xff;
	rmb();
	return (true);
}

static struct aop_setup_ep *
aop_setup_endpoint(struct aop_setup *st, uint8_t ep)
{
	u_int i;

	for (i = 0; i < SETUP_MAX_ENDPOINTS; i++)
		if (st->endpoints[i].valid && st->endpoints[i].ep == ep)
			return (&st->endpoints[i]);
	return (NULL);
}

static bool
aop_setup_ready(struct aop_setup *st)
{
	u_int i;

	if (!st->ap_ready || st->expected_count == 0)
		return (false);
	for (i = 0; i < st->expected_count; i++)
		if (!st->endpoints[i].valid &&
		    (st->wait_system || st->expected[i] >= AFK_ENDPOINT_START))
			return (false);
	return (true);
}

static u_int
aop_setup_endpoint_count(struct aop_setup *st)
{
	u_int i, n;

	for (i = n = 0; i < SETUP_MAX_ENDPOINTS; i++)
		n += st->endpoints[i].valid;
	return (n);
}

/* (5 << 60) | (host_to_aop << 54) | (pages << 48) | (iova >> 4) */
static uint64_t
aop_setup_descriptor(uint64_t iova, u_int pages, bool host_to_aop)
{
	return (5ull << 60 | (uint64_t)host_to_aop << 54 |
	    (uint64_t)pages << 48 | iova >> 4);
}

/*
 * A service endpoint's buffer request: one message page and a reply window
 * of the requested number of 16-byte entries, out of the arena.
 */
static int
aop_setup_endpoint_request(struct apple_aop_softc *sc, uint8_t ep,
    uint64_t word)
{
	struct aop_setup *st = &sc->setup;
	uint64_t tx_iova, rx_iova;
	size_t requested = word & 0xffffffff;
	u_int slot, rx_pages;
	int error;

	for (slot = 0; slot < st->expected_count; slot++)
		if (st->expected[slot] == ep)
			break;
	if (slot == st->expected_count)
		return (EINVAL);
	if (st->endpoints[slot].valid ||
	    (requested != 0x400 && requested != 0x1000))
		return (EINVAL);
	rx_pages = howmany(requested * SETUP_BUFFER_ENTRY_SIZE, SETUP_PAGE);
	if (st->next_page + 1 + rx_pages > SETUP_ARENA_PAGES)
		return (ENOMEM);
	tx_iova = st->arena_iova + st->next_page * SETUP_PAGE;
	rx_iova = tx_iova + SETUP_PAGE;
	st->next_page += 1 + rx_pages;
	error = aop_setup_send(sc, ep, SETUP_BUFFER_REQUEST_ACK);
	if (error == 0)
		error = aop_setup_send(sc, ep,
		    aop_setup_descriptor(tx_iova, 1, true));
	if (error == 0)
		error = aop_setup_send(sc, ep,
		    aop_setup_descriptor(rx_iova, rx_pages, false));
	if (error != 0)
		return (error);
	st->endpoints[slot].valid = true;
	st->endpoints[slot].ep = ep;
	st->endpoints[slot].tx_iova = tx_iova;
	st->endpoints[slot].rx_iova = rx_iova;
	if (bootverbose)
		device_printf(sc->dev, "setup port: endpoint %#x tx %#jx rx "
		    "%#jx (%u pages)\n", ep, (uintmax_t)tx_iova,
		    (uintmax_t)rx_iova, rx_pages);
	return (0);
}

static int
aop_setup_handle(struct apple_aop_softc *sc, uint8_t ep, uint64_t word)
{
	struct aop_setup *st = &sc->setup;
	struct aop_setup_ep *sep;
	u_int base, bit, i;
	uint8_t e;
	int error;

	if (ep == SETUP_MGMT_EP) {
		switch ((word >> 52) & 0xff) {
		case SETUP_TYPE_HELLO:
			if ((word & 0xffffffff) != SETUP_HELLO_VERSION)
				device_printf(sc->dev, "setup port: HELLO "
				    "version %#jx\n",
				    (uintmax_t)(word & 0xffffffff));
			return (aop_setup_send(sc, SETUP_MGMT_EP,
			    (uint64_t)SETUP_TYPE_HELLO_REPLY << 52 |
			    SETUP_HELLO_VERSION));
		case SETUP_TYPE_EPMAP:
			/* Each fragment is acknowledged by echoing it. */
			error = aop_setup_send(sc, SETUP_MGMT_EP, word);
			if (error != 0)
				return (error);
			if (st->epmap_endpoints) {
				/* RTKit layout: a bitmap, its base in 32..34. */
				base = ((word >> 32) & 7) * 32;
				if (bootverbose)
					device_printf(sc->dev, "setup port: "
					    "EPMAP %#jx\n", (uintmax_t)word);
				for (bit = 0; bit < 32; bit++) {
					e = base + bit;
					if ((word & (1ull << bit)) == 0 ||
					    e == SETUP_MGMT_EP)
						continue;
					for (i = 0; i < st->expected_count; i++)
						if (st->expected[i] == e)
							break;
					if (i < st->expected_count)
						continue;
					if (st->expected_count ==
					    SETUP_MAX_ENDPOINTS)
						return (ENOSPC);
					st->expected[st->expected_count++] = e;
				}
			}
			if (word & SETUP_EPMAP_LAST)
				st->map_done = true;
			return (0);
		case SETUP_TYPE_AP_PWR:
			st->ap_ready = (word & 0xffff) == SETUP_AP_PWR_ON;
			return (0);
		case SETUP_TYPE_UNK3:
			return (aop_setup_send(sc, SETUP_MGMT_EP,
			    (uint64_t)SETUP_TYPE_UNK3_REPLY << 52));
		case SETUP_TYPE_PWR_ACK:
			return (0);
		default:
			device_printf(sc->dev, "setup port: ignoring management "
			    "message type %#jx (%#jx)\n",
			    (uintmax_t)((word >> 52) & 0xff), (uintmax_t)word);
			return (0);
		}
	}
	if (word >> 56 == SETUP_BUFFER_REQUEST)
		return (aop_setup_endpoint_request(sc, ep, word));
	sep = aop_setup_endpoint(st, ep);
	if (sep != NULL) {
		if (sep->abandoned) {
			device_printf(sc->dev, "setup port: late reply %#jx on "
			    "endpoint %#x\n", (uintmax_t)word, ep);
			sep->abandoned = false;
			sep->busy = false;
			return (0);
		}
		sep->reply = word;
		sep->has_reply = true;
		return (0);
	}
	device_printf(sc->dev, "setup port: ignoring message %#jx on endpoint "
	    "%#x\n", (uintmax_t)word, ep);
	return (0);
}

/*
 * After the RTKit side reached AP power on: the setup port's AP power
 * request, then wait for it and for every endpoint's buffers. Sleeps with
 * the lock held across the waits (msleep drops it for the handlers).
 */
static int
aop_setup_finish_boot(struct apple_aop_softc *sc)
{
	struct aop_setup *st = &sc->setup;
	sbintime_t deadline;
	int error;

	mtx_assert(&sc->mtx, MA_OWNED);
	/*
	 * The map's last fragment can come a few milliseconds after RTKit's
	 * AP power on: request AP power only once it is complete.
	 */
	deadline = getsbinuptime() + MS_SBT(SETUP_MAP_TIMEOUT_MS);
	while (!st->map_done && !st->failed &&
	    aop_sleep(sc, deadline, "aopmap") == 0)
		;
	if (!st->map_done)
		device_printf(sc->dev, "setup port: endpoint map incomplete "
		    "before the AP power request\n");
	if (!st->power_sent) {
		error = aop_setup_send(sc, SETUP_MGMT_EP,
		    (uint64_t)SETUP_TYPE_AP_PWR << 52 | SETUP_AP_PWR_INIT);
		if (error != 0)
			return (error);
		st->power_sent = true;
	}
	deadline = getsbinuptime() + MS_SBT(SETUP_BOOT_TIMEOUT_MS);
	error = 0;
	while (!aop_setup_ready(st)) {
		if (st->failed) {
			error = EIO;
			break;
		}
		if (aop_sleep(sc, deadline, "aopsetup") != 0 &&
		    !aop_setup_ready(st)) {
			error = ETIMEDOUT;
			break;
		}
	}
	if (error != 0)
		device_printf(sc->dev, "setup port: boot incomplete (AP power "
		    "on: %d, endpoints: %u/%u): %d\n", st->ap_ready,
		    aop_setup_endpoint_count(st), st->expected_count, error);
	return (error);
}

/* ---- AFK endpoints and EPIC ---- */

static int
aop_afk_index(uint32_t ep)
{
	if (ep < AFK_ENDPOINT_START ||
	    ep >= AFK_ENDPOINT_START + AFK_ENDPOINT_COUNT)
		return (-1);
	return (ep - AFK_ENDPOINT_START);
}

static int
aop_send(struct apple_aop_softc *sc, uint8_t ep, uint64_t msg)
{
	if (sc->rtk == NULL)
		return (ENODEV);
	return (apple_rtkit_send(sc->rtk, ep, msg));
}

/* aop_ring.rs */
static int
aop_ring_check(const struct aop_ring *r)
{
	if (r->size < QEH_SIZE || !powerof2(r->block) || r->block == 0 ||
	    r->size % r->block != 0)
		return (EIO);
	return (0);
}

static int
aop_ring_check_ptr(const struct aop_ring *r, size_t ptr)
{
	if (ptr >= r->size || ptr % r->block != 0 || r->size - ptr < QEH_SIZE)
		return (EIO);
	return (0);
}

static int
aop_ring_end(const struct aop_ring *r, size_t ptr, size_t len, size_t *endp)
{
	size_t end;

	if (aop_ring_check_ptr(r, ptr) != 0 || len > r->size)
		return (EIO);
	end = roundup2(ptr + QEH_SIZE + len, r->block);
	if (end > r->size)
		return (EIO);
	*endp = end % r->size;
	return (0);
}

static uint32_t
aop_iomem_rd4(struct aop_afk_ep *e, size_t off)
{
	return (le32toh(*(volatile uint32_t *)(e->iomem + off)));
}

static void
aop_iomem_wr4(struct aop_afk_ep *e, size_t off, uint32_t v)
{
	*(volatile uint32_t *)(e->iomem + off) = htole32(v);
}

static void
aop_qeh_read(struct aop_afk_ep *e, size_t off, uint32_t *magic,
    uint32_t *size, uint32_t *channel)
{
	*magic = le32dec(e->iomem + off);
	*size = le32dec(e->iomem + off + 4);
	*channel = le32dec(e->iomem + off + 8);
}

/*
 * Decode an InitRX/InitTX message: a ring of size bytes at offset in the
 * shared buffer, three header blocks (buffer size, read pointer, write
 * pointer) followed by the entries.
 */
static int
aop_afk_parse_ring(struct aop_afk_ep *e, uint64_t msg, struct aop_ring *r)
{
	size_t size, offset, bufsize, block;

	size = ((msg >> 16) & 0xffff) * AFK_RB_BLOCK_STEP;
	offset = ((msg >> 32) & 0xffff) * AFK_RB_BLOCK_STEP;
	if (e->iomem == NULL)
		return (ENXIO);
	if (offset + 4 > e->iomem_size)
		return (EIO);
	bufsize = aop_iomem_rd4(e, offset);
	if (size < bufsize)
		return (EIO);
	block = (size - bufsize) / 3;
	if (offset + size > e->iomem_size || bufsize == 0 || block == 0 ||
	    !powerof2(block))
		return (EIO);
	r->offset = offset;
	r->block = block;
	r->size = bufsize;
	return (0);
}

static void
aop_afk_drop(struct apple_aop_softc *sc, struct aop_afk_ep *e,
    uint8_t category, uint16_t subtype, uint32_t channel, int error)
{
	if (e->dropped != UINT32_MAX)
		e->dropped++;
	if (powerof2(e->dropped))
		device_printf(sc->dev, "endpoint %#04x dropped a message: "
		    "category %#x subtype %#x on channel %u: %d (%u so far)\n",
		    e->index, category, subtype, channel, error, e->dropped);
}

static int
aop_register_service(struct apple_aop_softc *sc, struct aop_afk_ep *e,
    uint32_t channel, const uint8_t *name, size_t len)
{
	struct aop_service_rec *rec;
	int i;

	for (i = 0; i < sc->nservices; i++)
		if (strlen(sc->services[i].name) == len &&
		    memcmp(sc->services[i].name, name, len) == 0 &&
		    sc->services[i].svc.endpoint == e->index)
			break;
	if (i == sc->nservices) {
		if (sc->nservices == AOP_MAX_SERVICES)
			return (ENOSPC);
		sc->nservices++;
	}
	rec = &sc->services[i];
	memcpy(rec->name, name, len);
	rec->name[len] = '\0';
	rec->svc.channel = channel;
	rec->svc.endpoint = e->index;
	device_printf(sc->dev, "service \"%s\" on endpoint %#04x channel %u\n",
	    rec->name, e->index, channel);
	return (0);
}

/* A report with a listener goes to the thread's delivery queue. */
static int
aop_process_report(struct apple_aop_softc *sc, struct aop_afk_ep *e,
    uint32_t channel, uint16_t subtype, const uint8_t *data, size_t len)
{
	struct aop_listener *l;
	struct aop_report *rep;
	int i;

	for (i = 0; i < AOP_MAX_LISTENERS; i++) {
		l = &sc->listeners[i];
		if (l->used && l->svc.endpoint == e->index &&
		    l->svc.channel == channel && l->subtype == subtype)
			break;
	}
	if (i == AOP_MAX_LISTENERS)
		return (ENOENT);
	rep = malloc(sizeof(*rep) + len, M_APPLEAOP, M_NOWAIT);
	if (rep == NULL)
		return (ENOMEM);
	rep->listener = i;
	rep->subtype = subtype;
	rep->len = len;
	memcpy(rep->data, data, len);
	STAILQ_INSERT_TAIL(&sc->reports, rep, link);
	wakeup(&sc->thread);
	return (0);
}

/* Dispatch one message; an error means it was not handled. */
static int
aop_handle_ipc(struct apple_aop_softc *sc, struct aop_afk_ep *e,
    uint32_t channel, uint8_t category, uint16_t subtype, uint16_t tag,
    const uint8_t *data, size_t len)
{
	struct aop_call *c;
	size_t n, slot, namelen;
	uint32_t retcode;

	if (category == EPIC_CATEGORY_REPORT) {
		if (subtype == EPIC_SUBTYPE_STD_SERVICE) {
			if (len < EPIC_ANNOUNCE_LEN)
				return (EMSGSIZE);
			namelen = strnlen((const char *)data,
			    EPIC_ANNOUNCE_NAME_LEN);
			return (aop_register_service(sc, e,
			    le32dec(data + EPIC_ANNOUNCE_CHANNEL_OFFSET), data,
			    namelen));
		}
		if (aop_process_report(sc, e, channel, subtype, data, len) == 0)
			return (0);
		/* Fake-HID, gyroscope and accelerometer: no listeners here. */
		if (subtype == EPIC_SUBTYPE_FAKEHID_REPORT ||
		    subtype == EPIC_SUBTYPE_GYRO_REPORT ||
		    subtype == EPIC_SUBTYPE_ACCEL_REPORT)
			return (0);
		return (ENOENT);	/* a report nobody listens for */
	}
	if (category != EPIC_CATEGORY_REPLY)
		return (EINVAL);
	if (subtype != EPIC_SUBTYPE_RETCODE_PAYLOAD &&
	    subtype != EPIC_SUBTYPE_RETCODE && subtype != EPIC_SUBTYPE_STRING &&
	    subtype != EPIC_SUBTYPE_CLIENT_OPEN_REPLY &&
	    subtype != EPIC_SUBTYPE_CLIENT_CLOSE_REPLY)
		return (EINVAL);
	if (len < sizeof(uint32_t))
		return (EMSGSIZE);
	retcode = le32dec(data);
	if (tag >= 1 && tag <= AOP_MAX_CALLS && e->calls[tag - 1].used)
		slot = tag - 1;
	else if (sc->epic_v4) {
		/*
		 * Version 4 firmware does not echo the tag. Calls on such an
		 * endpoint are serialized, and none starts while an abandoned
		 * one is outstanding, so the reply is the one slot in use.
		 */
		for (slot = 0; slot < AOP_MAX_CALLS; slot++)
			if (e->calls[slot].used)
				break;
		if (slot == AOP_MAX_CALLS)
			return (ENOENT);
	} else if (tag == 0 || tag > AOP_MAX_CALLS)
		return (EINVAL);
	else
		return (ENOENT);
	c = &e->calls[slot];
	if (c->abandoned) {
		device_printf(sc->dev, "late reply for tag %zu on endpoint "
		    "%#04x\n", slot + 1, e->index);
		memset(c, 0, sizeof(*c));
		return (0);
	}
	c->retcode = retcode;
	if (c->want_reply) {
		n = MIN(len - 4, c->replycap);
		memcpy(c->reply, data + 4, n);
		c->replylen = n;
	}
	c->done = true;
	return (0);
}

static int
aop_afk_recv_rb(struct apple_aop_softc *sc, struct aop_afk_ep *e)
{
	struct aop_ring *r = &e->rx;
	uint32_t magic, size, channel;
	size_t rptr, wptr, base, budget, ptr, next, sub_ver;
	uint8_t *msg, category;
	uint16_t subtype, tag;
	int error;

	if (!e->has_rx) {
		device_printf(sc->dev, "Recv with no rxbuf at endpoint %#04x\n",
		    e->index);
		return (EIO);
	}
	if (aop_ring_check(r) != 0) {
		device_printf(sc->dev, "invalid ring on ep %#04x: size %#zx "
		    "block %#zx\n", e->index, r->size, r->block);
		return (EIO);
	}
	rptr = aop_iomem_rd4(e, r->offset + r->block);
	wptr = aop_iomem_rd4(e, r->offset + r->block * 2);
	dsb(sy);
	base = r->offset + r->block * 3;
	/* Entries beyond one ring's worth wait for their own doorbell. */
	for (budget = r->size / r->block; budget > 0; budget--) {
		if (aop_ring_check_ptr(r, rptr) != 0 ||
		    aop_ring_check_ptr(r, wptr) != 0) {
			device_printf(sc->dev, "ring pointers out of range on "
			    "ep %#04x: rptr %#zx wptr %#zx\n", e->index, rptr,
			    wptr);
			return (EIO);
		}
		if (wptr == rptr)
			break;
		/* aop_ring::read_entry */
		ptr = rptr;
		aop_qeh_read(e, base + ptr, &magic, &size, &channel);
		error = (magic != QE_MAGIC1 && magic != QE_MAGIC2) ? EIO : 0;
		if (error == 0 && size > r->size - ptr - QEH_SIZE) {
			ptr = 0;
			aop_qeh_read(e, base + ptr, &magic, &size, &channel);
			if (magic != QE_MAGIC1 && magic != QE_MAGIC2)
				error = EIO;
		}
		if (error == 0)
			error = aop_ring_end(r, ptr, size, &next);
		if (error == 0 && (next == ptr || next == rptr))
			error = EIO;
		if (error != 0) {
			device_printf(sc->dev, "invalid entry on ep %#04x at "
			    "%#zx: magic %x size %#x\n", e->index, rptr, magic,
			    size);
			return (EIO);
		}
		msg = e->msgbuf;
		memcpy(msg, e->iomem + base + ptr + QEH_SIZE, size);
		/*
		 * A message the endpoint cannot handle is skipped: not moving
		 * past it would stall the endpoint for good.
		 */
		if (size < EPIC_HDR_SIZE)
			aop_afk_drop(sc, e, 0, 0, channel, EMSGSIZE);
		else {
			sub_ver = msg[EPIC_HDR_SUB_VERSION];
			category = msg[EPIC_HDR_CATEGORY];
			subtype = le16dec(msg + EPIC_HDR_SUBTYPE);
			tag = le16dec(msg + (sub_ver >= 4 ? EPIC_HDR_TAG_V4 :
			    EPIC_HDR_TAG_V2));
			error = aop_handle_ipc(sc, e, channel, category,
			    subtype, tag, msg + EPIC_HDR_SIZE,
			    size - EPIC_HDR_SIZE);
			if (error != 0)
				aop_afk_drop(sc, e, category, subtype, channel,
				    error);
		}
		rptr = next;
		dsb(sy);
		aop_iomem_wr4(e, r->offset + r->block, rptr);
		wptr = aop_iomem_rd4(e, r->offset + r->block * 2);
		dsb(sy);
	}
	return (0);
}

static int
aop_afk_get_buf(struct apple_aop_softc *sc, struct aop_afk_ep *e,
    uint64_t msg)
{
	size_t size = ((msg & 0xffff0000) >> 16) * AFK_RB_BLOCK_STEP;

	if (e->iomem != NULL) {
		device_printf(sc->dev, "GetBuf with existing buffer on "
		    "endpoint %#04x\n", e->index);
		return (EIO);
	}
	if (size == 0)
		return (EIO);
	e->msgbuf = malloc(size, M_APPLEAOP, M_NOWAIT);
	if (e->msgbuf == NULL)
		return (ENOMEM);
	/* Owned before its address leaves the host; never freed after. */
	e->iomem = aop_dma_alloc(sc, sc->dart[0], size, &e->iomem_pa,
	    &e->iomem_iova);
	if (e->iomem == NULL)
		return (ENOMEM);
	e->iomem_size = size;
	if (bootverbose)
		device_printf(sc->dev, "endpoint %#04x: buffer %#zx at iova "
		    "%#jx\n", e->index, size, (uintmax_t)e->iomem_iova);
	return (aop_send(sc, e->index, AFK_MSG_GET_BUF_ACK | e->iomem_iova));
}

static void
aop_afk_recv_message(struct apple_aop_softc *sc, struct aop_afk_ep *e,
    uint64_t msg)
{
	int error = 0;

	switch (msg >> 48) {
	case AFK_OPC_INIT:
		error = aop_send(sc, e->index, AFK_MSG_INIT_ACK);
		break;
	case AFK_OPC_GET_BUF:
		error = aop_afk_get_buf(sc, e, msg);
		break;
	case AFK_OPC_INIT_UNK:
		break;
	case AFK_OPC_START_ACK:
	case AFK_OPC_SLEEP_ACK:
		break;
	case AFK_OPC_INIT_RX:
	case AFK_OPC_INIT_TX:
		if ((msg >> 48) == AFK_OPC_INIT_RX ? e->has_rx : e->has_tx) {
			device_printf(sc->dev, "Init%s with existing ring at "
			    "endpoint %#04x\n", (msg >> 48) == AFK_OPC_INIT_RX ?
			    "RX" : "TX", e->index);
			error = EIO;
			break;
		}
		if ((msg >> 48) == AFK_OPC_INIT_RX) {
			error = aop_afk_parse_ring(e, msg, &e->rx);
			e->has_rx = error == 0;
		} else {
			error = aop_afk_parse_ring(e, msg, &e->tx);
			e->has_tx = error == 0;
		}
		if (error == 0 && e->has_rx && e->has_tx)
			error = aop_send(sc, e->index, AFK_MSG_START);
		break;
	case AFK_OPC_RECV:
		error = aop_afk_recv_rb(sc, e);
		break;
	case AFK_OPC_SHUTDOWN_ACK:
		e->shutdown_acked = true;
		break;
	default:
		device_printf(sc->dev, "AFK endpoint %#04x got unknown message "
		    "%#jx\n", e->index, (uintmax_t)msg);
		break;
	}
	if (error != 0)
		device_printf(sc->dev, "failed to handle RTKit message %#jx on "
		    "endpoint %#04x: %d\n", (uintmax_t)msg, e->index, error);
}

/* RTKit application endpoint messages (from apple_rtkit_poll/boot). */
static void
aop_rtkit_recv(void *arg, uint32_t ep, uint64_t msg)
{
	struct apple_aop_softc *sc = arg;
	int idx;

	idx = aop_afk_index(ep);
	if (idx < 0) {
		device_printf(sc->dev, "message %#jx on unexpected endpoint "
		    "%#04x\n", (uintmax_t)msg, ep);
		return;
	}
	aop_afk_recv_message(sc, &sc->eps[idx], msg);
}

static const struct apple_rtkit_ops aop_rtkit_ops = {
	.buf_map = aop_rtkit_buf_map,
	.buf_lookup = aop_rtkit_buf_lookup,
	.recv = aop_rtkit_recv,
};

/*
 * Write one entry into the transmit ring; returns the doorbell message that
 * announces it. Nothing is visible to the firmware until that is sent.
 */
static int
aop_afk_write_entry(struct apple_aop_softc *sc, struct aop_afk_ep *e,
    uint32_t channel, uint32_t type, const uint8_t *hdr, size_t hdrlen,
    const uint8_t *data, size_t len, uint64_t *doorbell)
{
	struct aop_ring *r = &e->tx;
	size_t rptr, wptr, base, plen, head_next, ptr, next;
	uint8_t qeh[QEH_SIZE];
	bool wrapped;

	if (!e->has_tx) {
		device_printf(sc->dev, "send with no txbuf at endpoint %#04x\n",
		    e->index);
		return (EIO);
	}
	base = r->offset + r->block * 3;
	dsb(sy);
	rptr = aop_iomem_rd4(e, r->offset + r->block);
	wptr = aop_iomem_rd4(e, r->offset + r->block * 2);
	if (aop_ring_check(r) != 0 || aop_ring_check_ptr(r, rptr) != 0 ||
	    aop_ring_check_ptr(r, wptr) != 0) {
		device_printf(sc->dev, "Tx ring pointers out of range at "
		    "endpoint %#04x: rptr %#zx wptr %#zx\n", e->index, rptr,
		    wptr);
		return (EIO);
	}
	plen = hdrlen + len;
	/* aop_ring::write_entry */
	if (aop_ring_end(r, 0, plen, &head_next) != 0)
		goto full;
	wrapped = plen > r->size - wptr - QEH_SIZE;
	ptr = wrapped ? 0 : wptr;
	if (wrapped)
		next = head_next;
	else if (aop_ring_end(r, ptr, plen, &next) != 0)
		goto full;
	if (head_next == 0 || next == rptr ||
	    (wptr < rptr && (wrapped || next == 0 || next > rptr)) ||
	    (wrapped && (rptr == 0 || next > rptr)))
		goto full;

	le32enc(qeh, QE_MAGIC1);
	le32enc(qeh + 4, plen);
	le32enc(qeh + 8, channel);
	le32enc(qeh + 12, type);
	if (ptr != wptr)
		memcpy(e->iomem + base + wptr, qeh, QEH_SIZE);
	memcpy(e->iomem + base + ptr, qeh, QEH_SIZE);
	memcpy(e->iomem + base + ptr + QEH_SIZE, hdr, hdrlen);
	memcpy(e->iomem + base + ptr + QEH_SIZE + hdrlen, data, len);
	dsb(sy);
	aop_iomem_wr4(e, r->offset + r->block * 2, next);
	*doorbell = next | (uint64_t)AFK_OPC_SEND << 48;
	return (0);
full:
	device_printf(sc->dev, "Tx buffer full at endpoint %#04x\n", e->index);
	return (EIO);
}

/* Start a call; returns its slot. */
static int
aop_epic_notify(struct apple_aop_softc *sc, struct aop_afk_ep *e,
    uint32_t channel, uint16_t subtype, const void *data, size_t len,
    void *reply, size_t replycap, int *slotp)
{
	uint8_t hdr[EPIC_HDR_SIZE];
	uint64_t doorbell;
	int error, slot;

	if (sc->epic_v4)
		for (slot = 0; slot < AOP_MAX_CALLS; slot++)
			if (e->calls[slot].used && e->calls[slot].abandoned) {
				/* Its late reply could not be told apart. */
				return (EIO);
			}
	for (slot = 0; slot < AOP_MAX_CALLS; slot++)
		if (!e->calls[slot].used)
			break;
	if (slot == AOP_MAX_CALLS) {
		device_printf(sc->dev, "too many inflight calls on endpoint "
		    "%#04x\n", e->index);
		return (EIO);
	}
	memset(hdr, 0, sizeof(hdr));
	hdr[EPIC_HDR_VERSION] = 2;
	le16enc(hdr + EPIC_HDR_SEQ, e->seq);
	le32enc(hdr + EPIC_HDR_LENGTH, len);
	hdr[EPIC_HDR_CATEGORY] = EPIC_CATEGORY_NOTIFY;
	le16enc(hdr + EPIC_HDR_SUBTYPE, subtype);
	if (sc->epic_v4) {
		/* The reply capacity is advertised. */
		hdr[EPIC_HDR_SUB_VERSION] = 4;
		le16enc(hdr + EPIC_HDR_TAG_V4, slot + 1);
		le32enc(hdr + EPIC_HDR_INLINE_LEN, reply != NULL ? replycap :
		    0);
	} else {
		hdr[EPIC_HDR_SUB_VERSION] = 2;
		le16enc(hdr + EPIC_HDR_TAG_V2, slot + 1);
	}
	error = aop_afk_write_entry(sc, e, channel, EPIC_TYPE_NOTIFY, hdr,
	    sizeof(hdr), data, len, &doorbell);
	if (error != 0)
		return (error);
	e->seq++;
	memset(&e->calls[slot], 0, sizeof(e->calls[slot]));
	e->calls[slot].used = true;
	e->calls[slot].reply = reply;
	e->calls[slot].replycap = reply != NULL ? replycap : 0;
	e->calls[slot].want_reply = reply != NULL;
	error = aop_send(sc, e->index, doorbell);
	if (error != 0) {
		/* The next doorbell delivers it: keep the tag reserved. */
		e->calls[slot].abandoned = true;
		e->calls[slot].reply = NULL;
		e->calls[slot].want_reply = false;
		return (error);
	}
	*slotp = slot;
	return (0);
}

int
apple_aop_epic_call(device_t aop, const struct apple_aop_service *svc,
    uint16_t subtype, const void *req, size_t reqlen, void *reply,
    size_t *replylen, uint32_t *retcode, int timeout_ms)
{
	struct apple_aop_softc *sc = device_get_softc(aop);
	struct aop_afk_ep *e;
	struct aop_call *c;
	sbintime_t deadline;
	int error, idx, slot;

	idx = aop_afk_index(svc->endpoint);
	if (idx < 0)
		return (EINVAL);
	if (timeout_ms <= 0)
		timeout_ms = APPLE_AOP_CALL_TIMEOUT_MS;
	e = &sc->eps[idx];
	mtx_lock(&sc->mtx);
	/* Version 4 firmware answers without the tag: one call at a time. */
	while (sc->epic_v4 && e->turn && !sc->closing && !sc->crashed)
		mtx_sleep(&e->turn, &sc->mtx, 0, "aopturn", 0);
	if (sc->closing || sc->crashed || !sc->running) {
		mtx_unlock(&sc->mtx);
		return (ENODEV);
	}
	e->turn = true;
	error = aop_epic_notify(sc, e, svc->channel, subtype, req, reqlen,
	    reply, replylen != NULL ? *replylen : 0, &slot);
	if (error != 0)
		goto out;
	c = &e->calls[slot];
	deadline = getsbinuptime() + MS_SBT(timeout_ms);
	wakeup(&sc->thread);
	while (!c->done && !sc->crashed && !sc->closing)
		if (aop_sleep(sc, deadline, "aopcall") != 0)
			break;
	if (c->done) {
		*retcode = c->retcode;
		if (replylen != NULL && reply != NULL)
			*replylen = c->replylen;
		memset(c, 0, sizeof(*c));
	} else {
		/* The slot stays reserved until the late reply arrives. */
		c->abandoned = true;
		c->reply = NULL;
		c->want_reply = false;
		error = sc->crashed || sc->closing ? ENODEV : ETIMEDOUT;
		if (error == ETIMEDOUT)
			device_printf(sc->dev, "EPIC call %#x on channel %u "
			    "timed out after %d ms%s\n", subtype, svc->channel,
			    timeout_ms, sc->epic_v4 ? "; the endpoint takes "
			    "no call until the reply arrives" : "");
	}
out:
	e->turn = false;
	wakeup(&e->turn);
	mtx_unlock(&sc->mtx);
	return (error);
}

int
apple_aop_service_find(device_t aop, const char *name, int timeout_ms,
    struct apple_aop_service *svc)
{
	struct apple_aop_softc *sc = device_get_softc(aop);
	sbintime_t deadline;
	int error, i;

	deadline = getsbinuptime() + MS_SBT(timeout_ms);
	mtx_lock(&sc->mtx);
	for (error = ENOENT; error == ENOENT;) {
		if (!sc->running || sc->crashed || sc->closing) {
			error = ENXIO;
			break;
		}
		for (i = 0; i < sc->nservices; i++)
			if (strcmp(sc->services[i].name, name) == 0) {
				*svc = sc->services[i].svc;
				error = 0;
				break;
			}
		if (error == ENOENT && aop_sleep(sc, deadline, "aopsvc") != 0)
			break;
	}
	mtx_unlock(&sc->mtx);
	return (error);
}

int
apple_aop_report_listen(device_t aop, const struct apple_aop_service *svc,
    uint16_t subtype, apple_aop_report_cb_t *cb, void *arg)
{
	struct apple_aop_softc *sc = device_get_softc(aop);
	struct aop_listener *l;
	int error, i, free;

	mtx_lock(&sc->mtx);
	error = 0;
	free = -1;
	for (i = 0; i < AOP_MAX_LISTENERS; i++) {
		l = &sc->listeners[i];
		if (!l->used) {
			if (free < 0)
				free = i;
			continue;
		}
		if (l->svc.endpoint == svc->endpoint &&
		    l->svc.channel == svc->channel && l->subtype == subtype)
			error = EBUSY;
	}
	if (sc->closing)
		error = ENODEV;
	else if (error == 0 && free < 0)
		error = ENOSPC;
	if (error == 0) {
		l = &sc->listeners[free];
		l->svc = *svc;
		l->subtype = subtype;
		l->cb = cb;
		l->arg = arg;
		l->used = true;
	}
	mtx_unlock(&sc->mtx);
	return (error);
}

void
apple_aop_report_unlisten(device_t aop, const struct apple_aop_service *svc,
    uint16_t subtype)
{
	struct apple_aop_softc *sc = device_get_softc(aop);
	struct aop_listener *l;
	int i;

	mtx_lock(&sc->mtx);
	for (i = 0; i < AOP_MAX_LISTENERS; i++) {
		l = &sc->listeners[i];
		if (l->used && l->svc.endpoint == svc->endpoint &&
		    l->svc.channel == svc->channel && l->subtype == subtype)
			break;
	}
	if (i < AOP_MAX_LISTENERS) {
		l->used = false;
		while (sc->delivering == i + 1)
			mtx_sleep(&sc->delivering, &sc->mtx, 0, "aopunl", 0);
	}
	mtx_unlock(&sc->mtx);
}

/* ---- the interrupts and the thread ---- */

/*
 * Service both mailboxes until empty. Returns the AOP_IRQ_* lines (as bits)
 * whose FIFO held a message on entry.
 */
static u_int
aop_poll_locked(struct apple_aop_softc *sc)
{
	uint64_t word;
	uint8_t ep;
	u_int pending;
	int error;

	mtx_assert(&sc->mtx, MA_OWNED);
	pending = 0;
	if (sc->setup_mbox != 0 &&
	    (RD4(sc->setup_mbox, SETUP_I2A_CONTROL) & MBOX_CONTROL_EMPTY) == 0)
		pending |= 1u << AOP_IRQ_SETUP;
	if (sc->rtk_live && !sc->crashed &&
	    (RD4(sc->mbox, ASC_MBOX_I2A_CONTROL) & MBOX_CONTROL_EMPTY) == 0)
		pending |= 1u << AOP_IRQ_RTKIT;
	while (sc->setup_mbox != 0 && aop_setup_recv(sc, &ep, &word)) {
		error = aop_setup_handle(sc, ep, word);
		if (error != 0) {
			device_printf(sc->dev, "setup port: protocol error on "
			    "endpoint %#x, message %#jx: %d\n", ep,
			    (uintmax_t)word, error);
			sc->setup.failed = true;
		}
	}
	if (sc->rtk_live && !sc->crashed) {
		apple_rtkit_poll(sc->rtk);
		if (apple_rtkit_crashed(sc->rtk)) {
			sc->crashed = true;
			device_printf(sc->dev, "AOP firmware crashed\n");
		}
	}
	return (pending);
}

/*
 * A mailbox's recv-not-empty line (level; the AIC masks it while this
 * ithread handler runs). Draining the FIFO drops the line: the mailboxes
 * have no interrupt controls to acknowledge (aurora's has_irq_controls is
 * false for both).
 */
static void
aop_intr(void *arg)
{
	struct aop_irq *irq = arg;
	struct apple_aop_softc *sc = irq->sc;
	u_int pending;

	mtx_lock(&sc->mtx);
	irq->count++;
	if (irq->kill) {
		mtx_unlock(&sc->mtx);
		return;
	}
	pending = aop_poll_locked(sc);
	if ((pending & (1u << irq->which)) != 0)
		irq->empty_run = 0;
	else if (++irq->empty_run >= AOP_IRQ_STORM_LIMIT) {
		/* As aurora's ASCWrap v8 storm guard: cut it and poll. */
		device_printf(sc->dev, "%s mailbox interrupt fired %u times "
		    "without a message; disabled, polling every millisecond\n",
		    irq->which == AOP_IRQ_SETUP ? "setup" : "RTKit",
		    irq->empty_run);
		irq->kill = true;
		sc->polled = true;
		wakeup(&sc->thread);
	}
	/* A crashed RTKit is no longer read: its line would stay up. */
	if (irq->which == AOP_IRQ_RTKIT && sc->crashed && !irq->kill) {
		irq->kill = true;
		wakeup(&sc->thread);
	}
	wakeup(sc);
	mtx_unlock(&sc->mtx);
}

/* The mailbox node's "recv-not-empty" line as our IRQ rid "which". */
static int
aop_irq_setup(struct apple_aop_softc *sc, phandle_t mb, int which)
{
	struct aop_irq *irq = &sc->irqs[which];
	pcell_t *cells;
	phandle_t iparent;
	int error, idx, n, ncells, rid;
	u_int vec;

	if (ofw_bus_find_string_index(mb, "interrupt-names", "recv-not-empty",
	    &idx) != 0)
		return (ENXIO);
	iparent = ofw_bus_find_iparent(mb);	/* an xref */
	if (OF_searchencprop(OF_node_from_xref(iparent), "#interrupt-cells",
	    &ncells, sizeof(ncells)) == -1 || ncells < 1 || ncells > 4)
		return (ENXIO);
	n = OF_getencprop_alloc_multi(mb, "interrupts", sizeof(*cells),
	    (void **)&cells);
	if (n < (idx + 1) * ncells) {
		if (n > 0)
			OF_prop_free(cells);
		return (ENXIO);
	}
	vec = ofw_bus_map_intr(sc->dev, iparent, ncells, &cells[idx * ncells]);
	OF_prop_free(cells);
	irq->sc = sc;
	irq->which = which;
	rid = which;
	error = bus_set_resource(sc->dev, SYS_RES_IRQ, rid, vec, 1);
	if (error != 0)
		return (error);
	irq->res = bus_alloc_resource_any(sc->dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (irq->res == NULL)
		return (ENXIO);
	error = bus_setup_intr(sc->dev, irq->res, INTR_TYPE_MISC |
	    INTR_MPSAFE, NULL, aop_intr, irq, &irq->tag);
	if (error != 0) {
		bus_release_resource(sc->dev, SYS_RES_IRQ, rid, irq->res);
		irq->res = NULL;
		return (error);
	}
	bus_describe_intr(sc->dev, irq->res, irq->tag, which == AOP_IRQ_SETUP ?
	    "setup" : "rtkit");
	mtx_lock(&sc->mtx);
	irq->live = true;
	mtx_unlock(&sc->mtx);
	return (0);
}

/* Without the line, the thread polls every millisecond, as it used to. */
static void
aop_irq_start(struct apple_aop_softc *sc, phandle_t mb, int which)
{
	int error;

	error = aop_irq_setup(sc, mb, which);
	if (error == 0)
		return;
	device_printf(sc->dev, "%s mailbox: no interrupt (%d); polling every "
	    "millisecond\n", which == AOP_IRQ_SETUP ? "setup" : "RTKit", error);
	mtx_lock(&sc->mtx);
	sc->polled = true;
	wakeup(&sc->thread);
	mtx_unlock(&sc->mtx);
}

/* Tear down the lines marked kill (all = every line). Drops the lock. */
static void
aop_irq_teardown(struct apple_aop_softc *sc, bool all)
{
	struct aop_irq *irq;
	int i;

	mtx_assert(&sc->mtx, MA_OWNED);
	for (i = 0; i < AOP_IRQ_COUNT; i++) {
		irq = &sc->irqs[i];
		if (!irq->live || (!all && !irq->kill))
			continue;
		irq->live = false;
		irq->kill = true;
		mtx_unlock(&sc->mtx);
		bus_teardown_intr(sc->dev, irq->res, irq->tag);
		bus_release_resource(sc->dev, SYS_RES_IRQ, i, irq->res);
		mtx_lock(&sc->mtx);
		irq->res = NULL;
		irq->tag = NULL;
	}
}

static int
aop_thread_timo(struct apple_aop_softc *sc)
{
	if (sc->polled)
		return (MAX(1, hz / 1000));
	if (aop_poll_ms <= 0)
		return (0);
	return (MAX(1, (int)((int64_t)aop_poll_ms * hz / 1000)));
}

/*
 * Delivers reports outside the lock, tears down a line its handler gave
 * up on, and polls: every millisecond without interrupts, else every
 * hw.apple_aop.poll_ms as a net for a lost or miswired line.
 */
static void
aop_thread(void *arg)
{
	struct apple_aop_softc *sc = arg;
	struct aop_listener *l;
	struct aop_report *rep;

	mtx_lock(&sc->mtx);
	while (!sc->thread_stop) {
		if (aop_poll_locked(sc) != 0 && !sc->polled &&
		    sc->poll_hits++ == 0)
			device_printf(sc->dev, "the fallback poll found a "
			    "message before its interrupt (a rare race, or a "
			    "line that never fires)\n");
		if (sc->irqs[AOP_IRQ_SETUP].kill ||
		    sc->irqs[AOP_IRQ_RTKIT].kill)
			aop_irq_teardown(sc, false);
		while ((rep = STAILQ_FIRST(&sc->reports)) != NULL) {
			STAILQ_REMOVE_HEAD(&sc->reports, link);
			l = &sc->listeners[rep->listener];
			if (l->used) {
				sc->delivering = rep->listener + 1;
				mtx_unlock(&sc->mtx);
				l->cb(l->arg, rep->subtype, rep->data,
				    rep->len);
				mtx_lock(&sc->mtx);
				sc->delivering = 0;
				wakeup(&sc->delivering);
			}
			free(rep, M_APPLEAOP);
		}
		wakeup(sc);
		/* Work that came while the lock was dropped. */
		if (!STAILQ_EMPTY(&sc->reports) ||
		    (sc->irqs[AOP_IRQ_SETUP].kill &&
		    sc->irqs[AOP_IRQ_SETUP].live) ||
		    (sc->irqs[AOP_IRQ_RTKIT].kill &&
		    sc->irqs[AOP_IRQ_RTKIT].live))
			continue;
		mtx_sleep(&sc->thread, &sc->mtx, 0, "aoppoll",
		    aop_thread_timo(sc));
	}
	sc->thread_running = false;
	wakeup(&sc->thread_running);
	mtx_unlock(&sc->mtx);
	kproc_exit(0);
}

/* ---- boot ---- */

/* aurora's T8140 application map, for kernels without the endpoint map. */
static const uint8_t aop_fallback_eps[] = { 0x20, 0x21, 0x22, 0x23, 0x2b };

static bool
aop_has_endpoint(struct apple_aop_softc *sc, uint8_t ep)
{
	u_int i;

	if (apple_rtkit_has_endpoint != NULL)
		return (apple_rtkit_has_endpoint(sc->rtk, ep));
	for (i = 0; i < nitems(aop_fallback_eps); i++)
		if (aop_fallback_eps[i] == ep)
			return (true);
	return (false);
}

/*
 * Start the AFK handshake on every advertised endpoint. None is required on
 * T8142: one that does not start is skipped.
 */
static void
aop_start_afk(struct apple_aop_softc *sc)
{
	struct aop_afk_ep *e;
	int error, i;

	mtx_assert(&sc->mtx, MA_OWNED);
	if (apple_rtkit_has_endpoint == NULL)
		device_printf(sc->dev, "kernel without the RTKit endpoint map: "
		    "starting the T8140 endpoint set\n");
	for (i = 0; i < AFK_ENDPOINT_COUNT; i++) {
		e = &sc->eps[i];
		if (!aop_has_endpoint(sc, e->index))
			continue;
		error = apple_rtkit_start_ep(sc->rtk, e->index);
		if (error == 0) {
			error = aop_send(sc, e->index, AFK_MSG_INIT);
			if (error == ETIMEDOUT) {
				/* The firmware may still be finishing its boot. */
				device_printf(sc->dev, "endpoint %#04x: AFK "
				    "init not sent (mailbox full); retrying\n",
				    e->index);
				error = aop_send(sc, e->index, AFK_MSG_INIT);
			}
		}
		if (error != 0) {
			device_printf(sc->dev, "endpoint %#04x did not start "
			    "(%d); skipping it\n", e->index, error);
			continue;
		}
		e->started = true;
	}
}

static int
aop_map_mbox(phandle_t node, const char *name, vm_offset_t *va,
    bus_size_t *size, phandle_t *mbnode)
{
	pcell_t *xrefs;
	bus_addr_t pa;
	int idx, n;

	if (ofw_bus_find_string_index(node, "mbox-names", name, &idx) != 0)
		return (ENXIO);
	n = OF_getencprop_alloc_multi(node, "mboxes", sizeof(*xrefs),
	    (void **)&xrefs);
	/* #mbox-cells = <0>: one phandle per mailbox. */
	if (n <= idx) {
		if (n > 0)
			OF_prop_free(xrefs);
		return (ENXIO);
	}
	*mbnode = OF_node_from_xref(xrefs[idx]);
	OF_prop_free(xrefs);
	if (ofw_reg_to_paddr(*mbnode, 0, &pa, size, NULL) != 0)
		return (ENXIO);
	*va = (vm_offset_t)pmap_mapdev(pa, *size);
	return (0);
}

static int
aop_start(struct apple_aop_softc *sc)
{
	phandle_t node, setup_node, mbox_node;
	bus_addr_t pa;
	uint32_t ctl;
	int error, i;

	node = ofw_bus_get_node(sc->dev);
	if (ofw_bus_find_string_index(node, "reg-names", "asc", &i) != 0 ||
	    ofw_reg_to_paddr(node, i, &pa, &sc->asc_size, NULL) != 0)
		return (ENXIO);
	sc->asc = (vm_offset_t)pmap_mapdev(pa, sc->asc_size);
	error = aop_map_mbox(node, "mbox", &sc->mbox, &sc->mbox_size,
	    &mbox_node);
	if (error == 0)
		error = aop_map_mbox(node, "setup", &sc->setup_mbox,
		    &sc->setup_size, &setup_node);
	if (error != 0) {
		device_printf(sc->dev, "can't find the mailboxes\n");
		return (error);
	}

	/* The firmware's mappings stay; ours go into vacant entries. */
	error = apple_dart_fw_attach(sc->dev, node, 0, &sc->dart[0]);
	if (error == 0)
		error = apple_dart_fw_attach(sc->dev, setup_node, 0,
		    &sc->dart[1]);
	if (error != 0)
		return (error);

	/* The setup port: arena first, serviced from before the CPU runs. */
	sc->setup.arena = aop_dma_alloc(sc, sc->dart[1],
	    SETUP_ARENA_PAGES * SETUP_PAGE, &sc->setup.arena_pa,
	    &sc->setup.arena_iova);
	if (sc->setup.arena == NULL)
		return (ENOMEM);
	sc->rtk = apple_rtkit_create(sc->dev, sc->asc, sc->mbox,
	    &aop_rtkit_ops, sc, 0);

	sc->thread_running = true;
	error = kproc_create(aop_thread, sc, &sc->thread, 0, 0, "%s",
	    device_get_nameunit(sc->dev));
	if (error != 0) {
		sc->thread_running = false;
		return (error);
	}
	aop_irq_start(sc, setup_node, AOP_IRQ_SETUP);

	ctl = RD4(sc->asc, ASC_CPU_CONTROL);
	device_printf(sc->dev, "CPU_CONTROL %#x: %s\n", ctl,
	    (ctl & ASC_CPU_CONTROL_RUN) ? "running, waking it" :
	    "halted, starting it");
	/* RTKit up to AP power on; the setup port's handler runs meanwhile. */
	error = apple_rtkit_boot(sc->rtk);
	mtx_lock(&sc->mtx);
	if (error != 0) {
		device_printf(sc->dev, "RTKit wake failed: %d\n", error);
		goto out;
	}
	sc->rtk_live = true;
	mtx_unlock(&sc->mtx);
	/*
	 * Only now: the boot reads the mailbox itself, and a level line
	 * nobody drains would fire back to back. Messages already queued
	 * raise it as soon as it is unmasked.
	 */
	aop_irq_start(sc, mbox_node, AOP_IRQ_RTKIT);
	mtx_lock(&sc->mtx);
	error = aop_setup_finish_boot(sc);
	if (error != 0)
		goto out;
	aop_start_afk(sc);
	sc->running = true;
	wakeup(&sc->thread);
out:
	mtx_unlock(&sc->mtx);
	return (error);
}

/*
 * Stop every started endpoint: all shutdown requests first, then their
 * acknowledgments against one deadline.
 */
static int
aop_stop(struct apple_aop_softc *sc)
{
	struct aop_afk_ep *e;
	sbintime_t deadline;
	uint32_t sent, missing;
	bool stuck;
	int error, ret, i;

	mtx_assert(&sc->mtx, MA_OWNED);
	ret = 0;
	sent = 0;
	stuck = false;
	for (i = 0; i < AFK_ENDPOINT_COUNT; i++) {
		e = &sc->eps[i];
		if (!e->started)
			continue;
		e->started = false;
		e->shutdown_acked = false;
		if (stuck) {
			ret = ETIMEDOUT;
			continue;
		}
		error = aop_send(sc, e->index, AFK_MSG_SHUTDOWN);
		if (error == 0)
			sent |= 1u << i;
		else {
			device_printf(sc->dev, "endpoint %#04x: shutdown not "
			    "sent (%d)\n", e->index, error);
			stuck = error == ETIMEDOUT;
			ret = error;
		}
	}
	deadline = getsbinuptime() + MS_SBT(AFK_SHUTDOWN_TIMEOUT_MS);
	for (;;) {
		missing = 0;
		for (i = 0; i < AFK_ENDPOINT_COUNT; i++)
			if ((sent & (1u << i)) && !sc->eps[i].shutdown_acked)
				missing |= 1u << i;
		if (missing == 0 || sc->crashed ||
		    aop_sleep(sc, deadline, "aopstop") != 0)
			break;
	}
	if (missing != 0) {
		device_printf(sc->dev, "endpoints did not acknowledge their "
		    "shutdown within %d ms (mask %#x, bit 0 = endpoint %#04x)\n",
		    AFK_SHUTDOWN_TIMEOUT_MS, missing, AFK_ENDPOINT_START);
		ret = ETIMEDOUT;
	}
	return (ret);
}

static void
aop_thread_stop(struct apple_aop_softc *sc)
{
	mtx_lock(&sc->mtx);
	sc->thread_stop = true;
	wakeup(&sc->thread);
	while (sc->thread_running)
		mtx_sleep(&sc->thread_running, &sc->mtx, 0, "aopexit", 0);
	mtx_unlock(&sc->mtx);
}

static void
aop_attach_children(struct apple_aop_softc *sc)
{
	phandle_t node, child;

	node = ofw_bus_get_node(sc->dev);
	simplebus_init(sc->dev, node);
	for (child = OF_child(node); child > 0; child = OF_peer(child))
		simplebus_add_device(sc->dev, child, 0, NULL, -1, NULL);
	bus_attach_children(sc->dev);
}

static void
aop_start_hook(void *arg)
{
	struct apple_aop_softc *sc = arg;
	int error;

	error = aop_start(sc);
	if (error != 0) {
		/*
		 * Whatever the firmware was given stays allocated and mapped:
		 * it may still use it.
		 */
		device_printf(sc->dev, "AOP start failed: %d\n", error);
		mtx_lock(&sc->mtx);
		sc->closing = true;
		aop_irq_teardown(sc, true);
		mtx_unlock(&sc->mtx);
		if (sc->thread_running)
			aop_thread_stop(sc);
		return;
	}
	device_printf(sc->dev, "AOP running, %d services so far\n",
	    sc->nservices);
	aop_attach_children(sc);
}

static int
apple_aop_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "apple,t8142-aop"))
		return (ENXIO);
	device_set_desc(dev, "Apple AOP");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_aop_attach(device_t dev)
{
	struct apple_aop_softc *sc = device_get_softc(dev);
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *tree;
	int i;

	sc->dev = dev;
	mtx_init(&sc->mtx, "apple_aop", NULL, MTX_DEF);
	STAILQ_INIT(&sc->reports);
	sc->epic_v4 = true;
	sc->setup.epmap_endpoints = true;
	sc->setup.wait_system = true;
	for (i = 0; i < AFK_ENDPOINT_COUNT; i++)
		sc->eps[i].index = AFK_ENDPOINT_START + i;
	ctx = device_get_sysctl_ctx(dev);
	tree = SYSCTL_CHILDREN(device_get_sysctl_tree(dev));
	SYSCTL_ADD_ULONG(ctx, tree, OID_AUTO, "setup_irqs", CTLFLAG_RD,
	    &sc->irqs[AOP_IRQ_SETUP].count, "Setup mailbox interrupts");
	SYSCTL_ADD_ULONG(ctx, tree, OID_AUTO, "rtkit_irqs", CTLFLAG_RD,
	    &sc->irqs[AOP_IRQ_RTKIT].count, "RTKit mailbox interrupts");
	SYSCTL_ADD_ULONG(ctx, tree, OID_AUTO, "poll_hits", CTLFLAG_RD,
	    &sc->poll_hits, "Messages the fallback poll found first");
	SYSCTL_ADD_BOOL(ctx, tree, OID_AUTO, "polled", CTLFLAG_RD,
	    &sc->polled, 0, "Polling every millisecond (a line is missing)");
	/* The boot sleeps for seconds: after interrupts are up when cold. */
	if (cold)
		config_intrhook_oneshot(aop_start_hook, sc);
	else
		aop_start_hook(sc);
	return (0);
}

static int
apple_aop_detach(device_t dev)
{
	struct apple_aop_softc *sc = device_get_softc(dev);
	struct aop_report *rep;
	int error;

	error = bus_generic_detach(dev);
	if (error != 0)
		return (error);
	mtx_lock(&sc->mtx);
	sc->closing = true;
	wakeup(sc);
	if (sc->running && !sc->crashed && aop_stop(sc) != 0)
		device_printf(sc->dev, "failed to stop the AOP\n");
	sc->running = false;
	/* Lines first: an undrained RTKit line would fire back to back. */
	aop_irq_teardown(sc, true);
	sc->rtk_live = false;
	mtx_unlock(&sc->mtx);
	if (sc->thread_running)
		aop_thread_stop(sc);
	while ((rep = STAILQ_FIRST(&sc->reports)) != NULL) {
		STAILQ_REMOVE_HEAD(&sc->reports, link);
		free(rep, M_APPLEAOP);
	}
	/*
	 * There is no RTKit shutdown here: the firmware keeps running, and
	 * every buffer and DART mapping it was given is kept (leaked) for it,
	 * as aurora does when a shutdown can't be confirmed.
	 */
	device_printf(dev, "detached; the AOP keeps running with its "
	    "buffers\n");
	mtx_destroy(&sc->mtx);
	return (0);
}

static device_method_t apple_aop_methods[] = {
	DEVMETHOD(device_probe,		apple_aop_probe),
	DEVMETHOD(device_attach,	apple_aop_attach),
	DEVMETHOD(device_detach,	apple_aop_detach),
	DEVMETHOD_END
};

DEFINE_CLASS_1(apple_aop, apple_aop_driver, apple_aop_methods,
    sizeof(struct apple_aop_softc), simplebus_driver);
EARLY_DRIVER_MODULE(apple_aop, simplebus, apple_aop_driver, NULL, NULL,
    BUS_PASS_DEFAULT);
MODULE_VERSION(apple_aop, 1);
