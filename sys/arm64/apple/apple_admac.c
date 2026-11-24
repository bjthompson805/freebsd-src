/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple audio DMA controller (ADMAC), for the T8140-family "evolved" one on
 * T8142 (M5). This follows aurora-silicon's drivers/dma/apple-admac.c
 * (Linux, Asahi and aurora contributors) for "apple,t8140-admac", whose
 * admac_hw is { .set_unk28 = true, .evolved_layout = true }: the evolved
 * channel layout (TX blocks at 0x8000, RX at 0xc000), REG_UNK_28/2C set on
 * configuration, no CTL write, no reset (the DT has no "resets": the
 * firmware owns the fabric). The display-ADMAC, T8132 split-channel and
 * T8152 paths are left out, as are pause/resume and capture realignment.
 *
 * As aurora's probe, attach touches no register: the controller is in a
 * power domain the AOP firmware manages, powered once the audio driver has
 * set the audio power state (for admac-leap-s, raised the LEAP leaves),
 * before it configures a channel. See apple_admac.h for the API.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm64/apple/apple_admac.h>
#include <arm64/apple/apple_dart.h>

#define	NCHANNELS_MAX		64
#define	SRAM_BLOCK		2048
#define	ADMAC_PAGE		(16 * 1024)	/* DART page */
#define	ADMAC_RING_SLOTS	4

#define	RING_WRITE_SLOT(v)	((v) & 0x3)
#define	RING_READ_SLOT(v)	(((v) >> 4) & 0x3)
#define	RING_EMPTY		(1u << 8)
#define	RING_FULL		(1u << 9)
#define	RING_ERR		(1u << 10)

#define	STATUS_DESC_DONE	(1u << 0)
#define	STATUS_ERR		(1u << 6)

#define	FLAG_DESC_NOTIFY	(1u << 16)

#define	T603X_UNK_28_VAL	0x200000

#define	REG_TX_START		0x0000
#define	REG_TX_STOP		0x0004
#define	REG_RX_START		0x0008
#define	REG_RX_STOP		0x000c
#define	REG_UNK_28		0x0028
#define	REG_UNK_2C		0x002c
#define	REG_TX_SRAM_SIZE	0x0094
#define	REG_RX_SRAM_SIZE	0x0098

#define	REG_TX_INTSTATE(idx)	(0x0030 + (idx) * 4)
#define	REG_RX_INTSTATE(idx)	(0x0040 + (idx) * 4)
#define	REG_GLOBAL_INTSTATE(idx) (0x0050 + (idx) * 4)

/* Evolved layout: TX channel blocks at 0x8000, RX at 0xc000. */
#define	REG_CHAN_BASE(ch)	(((ch) & 1 ? 0xc000 : 0x8000) + ((ch) / 2) * 0x200)
#define	REG_CHAN_CTL(ch)	(REG_CHAN_BASE(ch) + 0x00)
#define	CHAN_CTL_RST_RINGS	(1u << 0)
#define	REG_CHAN_INTSTATUS(ch, idx) (REG_CHAN_BASE(ch) + 0x10 + (idx) * 4)
#define	REG_CHAN_INTMASK(ch, idx) (REG_CHAN_BASE(ch) + 0x20 + (idx) * 4)
#define	REG_BUS_WIDTH(ch)	(REG_CHAN_BASE(ch) + 0x40)
#define	BUS_WIDTH_WORD_SIZE	0x0f
#define	BUS_WIDTH_FRAME_SIZE	0xf0
#define	BUS_WIDTH_8BIT		0x00
#define	BUS_WIDTH_16BIT		0x01
#define	BUS_WIDTH_32BIT		0x02
#define	BUS_WIDTH_FRAME_2_WORDS	0x10
#define	BUS_WIDTH_FRAME_4_WORDS	0x20
#define	REG_CHAN_SRAM_CARVEOUT(ch) (REG_CHAN_BASE(ch) + 0x50)
#define	CARVEOUT(base, size)	(((uint32_t)(size) << 16) | ((base) & 0xffff))
#define	CARVEOUT_BASE(v)	((v) & 0xffff)
#define	REG_CHAN_FIFOCTL(ch)	(REG_CHAN_BASE(ch) + 0x54)
#define	FIFOCTL(limit, thresh)	(((uint32_t)(limit) << 16) | ((thresh) & 0xffff))
#define	REG_RESIDUE(ch)		(REG_CHAN_BASE(ch) + 0x64)
#define	REG_DESC_RING(ch)	(REG_CHAN_BASE(ch) + 0x70)
#define	REG_REPORT_RING(ch)	(REG_CHAN_BASE(ch) + 0x74)

#define	REG_DESC_WRITE(ch)	(0x10000 + ((ch) / 2) * 4 + ((ch) & 1) * 0x4000)
#define	REG_REPORT_READ(ch)	(0x10100 + ((ch) / 2) * 4 + ((ch) & 1) * 0x4000)

#define	ADMAC_RESIDUE_READS	16
#define	ADMAC_STOP_TIMEOUT_US	100000

struct admac_softc;

struct apple_admac_chan {
	struct admac_softc	*sc;
	u_int			no;
	bool			claimed;
	bool			has_carveout;
	uint32_t		carveout;

	/* The cyclic transfer, under sc->mtx */
	bool			running;
	bool			error;
	u_int			gen;
	bus_addr_t		buf_addr;
	size_t			buf_len;
	size_t			period_len;
	u_int			nperiods_outstanding;
	u_int			nacks;
	size_t			submitted_pos;	/* all in [0, 2 * buf_len) */
	size_t			reclaimed_pos;
	size_t			reported_pos;
	void			(*cb)(void *);
	void			*cbarg;

	/* The kept DMA buffer */
	void			*kva;
	bus_addr_t		iova;
	size_t			bufsize;
	bool			buf_busy;
};

struct admac_sram {
	uint32_t		size;
	uint32_t		allocated;
};

#define	ADMAC_STREAMS_MAX	4

/* One "iommus" entry: the firmware's live tables (fw), or a private one. */
struct admac_stream {
	struct apple_dart_fw	*fw;
	phandle_t		dart;
	u_int			sid;
};

struct admac_softc {
	device_t		dev;
	struct resource		*mem;
	struct resource		*irq;
	void			*ih;
	struct mtx		mtx;
	int			irq_index;
	int			nchannels;
	u_int			nrunning;	/* registers in use */
	u_int			cb_busy;
	struct admac_sram	txcache, rxcache;
	uint64_t		dma_base, dma_end, dma_next;
	/* Set up on the first buffer (the DART may be unpowered before). */
	bool			streams_ready, streams_busy;
	int			nstreams;
	struct admac_stream	streams[ADMAC_STREAMS_MAX];
	struct apple_admac_chan	chans[NCHANNELS_MAX];
};

#define	RD4(sc, o)	bus_read_4((sc)->mem, (o))
#define	WR4(sc, o, v)	bus_write_4((sc)->mem, (o), (v))

static MALLOC_DEFINE(M_ADMAC, "apple_admac", "Apple ADMAC buffers");

static struct ofw_compat_data compat_data[] = {
	{ "apple,t8140-admac",	1 },
	{ NULL,			0 }
};

static bool
admac_is_tx(u_int no)
{

	return ((no & 1) == 0);
}

bool
apple_admac_is_tx(struct apple_admac_chan *ch)
{

	return (admac_is_tx(ch->no));
}

static void
admac_modify(struct admac_softc *sc, bus_size_t reg, uint32_t mask,
    uint32_t val)
{

	WR4(sc, reg, (RD4(sc, reg) & ~mask) | (val & mask));
}

/* Context: sc->mtx held, controller powered. */
static int
admac_alloc_sram_carveout(struct admac_softc *sc, bool tx, uint32_t *out)
{
	struct admac_sram *sram;
	int i, nblocks;

	sc->txcache.size = RD4(sc, REG_TX_SRAM_SIZE);
	sc->rxcache.size = RD4(sc, REG_RX_SRAM_SIZE);
	sram = tx ? &sc->txcache : &sc->rxcache;

	nblocks = MIN(sram->size / SRAM_BLOCK, 32);
	for (i = 0; i < nblocks; i++)
		if ((sram->allocated & (1u << i)) == 0)
			break;
	if (i >= nblocks)
		return (EBUSY);
	*out = CARVEOUT(i * SRAM_BLOCK, SRAM_BLOCK);
	sram->allocated |= 1u << i;
	return (0);
}

static void
admac_free_sram_carveout(struct admac_softc *sc, bool tx, uint32_t carveout)
{
	struct admac_sram *sram;
	uint32_t base;

	sram = tx ? &sc->txcache : &sc->rxcache;
	base = CARVEOUT_BASE(carveout);
	if (base >= sram->size) {
		device_printf(sc->dev, "bad carveout %#x\n", carveout);
		return;
	}
	sram->allocated &= ~(1u << (base / SRAM_BLOCK));
}

static void
admac_write_hw_desc(struct admac_softc *sc, u_int no, bus_addr_t addr,
    size_t len)
{

	WR4(sc, REG_DESC_WRITE(no), (uint32_t)addr);
	WR4(sc, REG_DESC_WRITE(no), (uint32_t)(addr >> 32));
	WR4(sc, REG_DESC_WRITE(no), (uint32_t)len);
	WR4(sc, REG_DESC_WRITE(no), FLAG_DESC_NOTIFY);
}

static bool
admac_cyclic_write_one_desc(struct apple_admac_chan *ch)
{

	if (ch->error)
		return (false);
	admac_write_hw_desc(ch->sc, ch->no,
	    ch->buf_addr + ch->submitted_pos % ch->buf_len, ch->period_len);
	/* Include a descriptor already consumed while the ring is refilled. */
	ch->nperiods_outstanding++;
	ch->submitted_pos = (ch->submitted_pos + ch->period_len) %
	    (2 * ch->buf_len);
	return (true);
}

static bool
admac_cyclic_write_desc(struct apple_admac_chan *ch)
{
	int i;

	for (i = 0; i < ADMAC_RING_SLOTS; i++) {
		if (RD4(ch->sc, REG_DESC_RING(ch->no)) & RING_FULL)
			break;
		if (!admac_cyclic_write_one_desc(ch))
			return (false);
	}
	return (true);
}

static int
admac_ring_noccupied_slots(uint32_t ringval)
{
	int wr, rd;

	wr = RING_WRITE_SLOT(ringval);
	rd = RING_READ_SLOT(ringval);
	if (wr != rd)
		return ((wr + ADMAC_RING_SLOTS - rd) % ADMAC_RING_SLOTS);
	return ((ringval & RING_FULL) != 0 ? ADMAC_RING_SLOTS : 0);
}

struct admac_residue_read {
	uint32_t ring1, residue1;
	uint32_t ring2, residue2;
};

/*
 * The position in [0, 2 * buf_len) from one read, or false if the read is
 * not a consistent snapshot: at a period's end the controller runs the
 * residue to 0, posts the report, then loads the next descriptor (see
 * aurora's admac_cyclic_pos()).
 */
static bool
admac_cyclic_pos(const struct apple_admac_chan *ch,
    const struct admac_residue_read *r, size_t *pos)
{

	if (r->ring1 != r->ring2 || r->residue2 == 0 ||
	    r->residue2 > r->residue1 || r->residue2 > ch->period_len)
		return (false);
	*pos = (ch->reclaimed_pos + ch->period_len *
	    (admac_ring_noccupied_slots(r->ring2) + 1) - r->residue2) %
	    (2 * ch->buf_len);
	return (true);
}

static bool
admac_cyclic_pos_after(const struct apple_admac_chan *ch, size_t a, size_t b)
{
	size_t span, ahead;

	span = 2 * ch->buf_len;
	ahead = (b + span - a) % span;
	return (ahead != 0 && ahead < ch->buf_len);
}

static size_t
admac_cyclic_pos_at_period_end(const struct apple_admac_chan *ch,
    const struct admac_residue_read *r)
{
	size_t done;

	done = (ch->reclaimed_pos + ch->period_len *
	    admac_ring_noccupied_slots(r->ring2)) % (2 * ch->buf_len);
	return (admac_cyclic_pos_after(ch, ch->reported_pos, done) ? done :
	    ch->reported_pos);
}

/* Context: sc->mtx held, running. */
static size_t
admac_cyclic_read_pos(struct apple_admac_chan *ch)
{
	struct admac_softc *sc = ch->sc;
	struct admac_residue_read r;
	size_t pos;
	int i;

	pos = 0;
	for (i = 0; i < ADMAC_RESIDUE_READS; i++) {
		r.ring1 = RD4(sc, REG_REPORT_RING(ch->no));
		r.residue1 = RD4(sc, REG_RESIDUE(ch->no));
		r.ring2 = RD4(sc, REG_REPORT_RING(ch->no));
		r.residue2 = RD4(sc, REG_RESIDUE(ch->no));
		if (admac_cyclic_pos(ch, &r, &pos))
			break;
	}
	if (i == ADMAC_RESIDUE_READS)
		pos = admac_cyclic_pos_at_period_end(ch, &r);
	ch->reported_pos = pos;
	return (pos);
}

static void
admac_start_chan(struct apple_admac_chan *ch)
{
	struct admac_softc *sc = ch->sc;

	WR4(sc, REG_CHAN_INTSTATUS(ch->no, sc->irq_index),
	    STATUS_DESC_DONE | STATUS_ERR);
	WR4(sc, REG_CHAN_INTMASK(ch->no, sc->irq_index),
	    STATUS_DESC_DONE | STATUS_ERR);
	WR4(sc, admac_is_tx(ch->no) ? REG_TX_START : REG_RX_START,
	    1u << (ch->no / 2));
}

static void
admac_stop_chan(struct apple_admac_chan *ch)
{

	WR4(ch->sc, admac_is_tx(ch->no) ? REG_TX_STOP : REG_RX_STOP,
	    1u << (ch->no / 2));
}

static void
admac_wait_stopped(struct apple_admac_chan *ch)
{
	bus_size_t reg;
	int us;

	reg = admac_is_tx(ch->no) ? REG_TX_START : REG_RX_START;
	for (us = 0; us < ADMAC_STOP_TIMEOUT_US; us += 10) {
		if ((RD4(ch->sc, reg) & (1u << (ch->no / 2))) == 0)
			return;
		DELAY(10);
	}
	device_printf(ch->sc->dev, "ch%u still enabled %d us after stop\n",
	    ch->no, ADMAC_STOP_TIMEOUT_US);
}

static void
admac_reset_rings(struct apple_admac_chan *ch)
{

	WR4(ch->sc, REG_CHAN_CTL(ch->no), CHAN_CTL_RST_RINGS);
	WR4(ch->sc, REG_CHAN_CTL(ch->no), 0);
}

static int
admac_drain_reports(struct admac_softc *sc, u_int no)
{
	uint32_t lo, hi, unk1, flags;
	int count;

	for (count = 0; count < ADMAC_RING_SLOTS; count++) {
		if (RD4(sc, REG_REPORT_RING(no)) & RING_EMPTY)
			break;
		lo = RD4(sc, REG_REPORT_READ(no));
		hi = RD4(sc, REG_REPORT_READ(no));
		unk1 = RD4(sc, REG_REPORT_READ(no));
		flags = RD4(sc, REG_REPORT_READ(no));
		if (bootverbose)
			device_printf(sc->dev, "ch%u report: countval=%#jx "
			    "unk1=%#x flags=%#x\n", no,
			    (uintmax_t)hi << 32 | lo, unk1, flags);
	}
	return (count);
}

/* Context: sc->mtx held. */
static void
admac_handle_status_err(struct admac_softc *sc, u_int no)
{
	struct apple_admac_chan *ch = &sc->chans[no];
	bool handled;

	if (ch->running) {
		ch->error = true;
		ch->nacks = 0;
		admac_stop_chan(ch);
	}
	handled = false;
	if (RD4(sc, REG_DESC_RING(no)) & RING_ERR) {
		WR4(sc, REG_DESC_RING(no), RING_ERR);
		device_printf(sc->dev, "ch%u descriptor ring error\n", no);
		handled = true;
	}
	if (RD4(sc, REG_REPORT_RING(no)) & RING_ERR) {
		WR4(sc, REG_REPORT_RING(no), RING_ERR);
		device_printf(sc->dev, "ch%u report ring error\n", no);
		handled = true;
	}
	if (!handled) {
		device_printf(sc->dev, "ch%u unknown error, masking errors as "
		    "cause of IRQs\n", no);
		admac_modify(sc, REG_CHAN_INTMASK(no, sc->irq_index),
		    STATUS_ERR, 0);
	}
}

/* Context: sc->mtx held. */
static void
admac_handle_status_desc_done(struct admac_softc *sc, u_int no)
{
	struct apple_admac_chan *ch = &sc->chans[no];
	int nreports;

	WR4(sc, REG_CHAN_INTSTATUS(no, sc->irq_index), STATUS_DESC_DONE);
	nreports = admac_drain_reports(sc, no);
	if (!ch->running)
		return;
	/* Never turn an unowned report into a period callback. */
	if (ch->error || (u_int)nreports > ch->nperiods_outstanding)
		goto error;
	ch->nperiods_outstanding -= nreports;
	ch->nacks += nreports;
	ch->reclaimed_pos = (ch->reclaimed_pos + nreports * ch->period_len) %
	    (2 * ch->buf_len);
	if (!admac_cyclic_write_desc(ch))
		goto error;
	return;
error:
	ch->error = true;
	ch->nacks = 0;
	admac_stop_chan(ch);
	device_printf(sc->dev, "ch%u completion accounting error\n", no);
}

static void
admac_handle_chan_int(struct admac_softc *sc, u_int no)
{
	uint32_t cause;

	cause = RD4(sc, REG_CHAN_INTSTATUS(no, sc->irq_index));
	if (cause & STATUS_ERR)
		admac_handle_status_err(sc, no);
	if (cause & STATUS_DESC_DONE)
		admac_handle_status_desc_done(sc, no);
}

/* The period callbacks, as aurora's tasklet: no lock held over a call. */
static void
admac_run_callbacks(struct admac_softc *sc)
{
	struct apple_admac_chan *ch;
	void (*cb)(void *);
	void *arg;
	u_int gen, n;
	int i;

	mtx_assert(&sc->mtx, MA_OWNED);
	for (i = 0; i < sc->nchannels; i++) {
		ch = &sc->chans[i];
		n = ch->nacks;
		ch->nacks = 0;
		gen = ch->gen;
		while (n-- > 0 && ch->running && !ch->error && ch->gen == gen &&
		    ch->cb != NULL) {
			cb = ch->cb;
			arg = ch->cbarg;
			sc->cb_busy++;
			mtx_unlock(&sc->mtx);
			cb(arg);
			mtx_lock(&sc->mtx);
			if (--sc->cb_busy == 0)
				wakeup(&sc->cb_busy);
		}
	}
}

static void
admac_intr(void *arg)
{
	struct admac_softc *sc = arg;
	uint32_t tx, rx, global;
	int i;

	mtx_lock(&sc->mtx);
	/* No register access while no channel is in use: maybe unpowered. */
	if (sc->nrunning == 0) {
		mtx_unlock(&sc->mtx);
		return;
	}
	rx = RD4(sc, REG_RX_INTSTATE(sc->irq_index));
	tx = RD4(sc, REG_TX_INTSTATE(sc->irq_index));
	global = RD4(sc, REG_GLOBAL_INTSTATE(sc->irq_index));
	if (tx == 0 && rx == 0 && global == 0) {
		mtx_unlock(&sc->mtx);
		return;
	}
	for (i = 0; i < sc->nchannels; i += 2, tx >>= 1)
		if (tx & 1)
			admac_handle_chan_int(sc, i);
	for (i = 1; i < sc->nchannels; i += 2, rx >>= 1)
		if (rx & 1)
			admac_handle_chan_int(sc, i);
	/* Sticky write-one-to-clear notifications, no recovery semantics. */
	if (global != 0)
		WR4(sc, REG_GLOBAL_INTSTATE(sc->irq_index), global);
	admac_run_callbacks(sc);
	mtx_unlock(&sc->mtx);
}

int
apple_admac_get_channel(phandle_t xref, uint32_t cell,
    struct apple_admac_chan **chp)
{
	struct admac_softc *sc;
	struct apple_admac_chan *ch;
	device_t dev;

	dev = OF_device_from_xref(xref);
	if (dev == NULL || device_get_softc(dev) == NULL ||
	    !device_is_attached(dev))
		return (ENXIO);
	if (strcmp(device_get_name(dev), "apple_admac") != 0)
		return (ENXIO);
	sc = device_get_softc(dev);
	if (cell >= (uint32_t)sc->nchannels) {
		device_printf(dev, "channel index %u out of bounds\n", cell);
		return (EINVAL);
	}
	ch = &sc->chans[cell];
	mtx_lock(&sc->mtx);
	if (ch->claimed) {
		mtx_unlock(&sc->mtx);
		return (EBUSY);
	}
	ch->claimed = true;
	mtx_unlock(&sc->mtx);
	*chp = ch;
	return (0);
}

int
apple_admac_config(struct apple_admac_chan *ch, int bus_width_bytes,
    int frame_words)
{
	struct admac_softc *sc = ch->sc;
	uint32_t bus_width;
	int error;

	mtx_lock(&sc->mtx);
	if (ch->running) {
		mtx_unlock(&sc->mtx);
		return (EBUSY);
	}
	/* Registers are in use from here on (see admac_intr()). */
	sc->nrunning++;
	error = 0;
	if (!ch->has_carveout) {
		error = admac_alloc_sram_carveout(sc, admac_is_tx(ch->no),
		    &ch->carveout);
		if (error != 0) {
			device_printf(sc->dev, "ch%u: no SRAM left (tx %#x "
			    "rx %#x)\n", ch->no, sc->txcache.size,
			    sc->rxcache.size);
			goto out;
		}
		ch->has_carveout = true;
	}
	WR4(sc, REG_CHAN_SRAM_CARVEOUT(ch->no), ch->carveout);

	bus_width = RD4(sc, REG_BUS_WIDTH(ch->no)) &
	    ~(BUS_WIDTH_WORD_SIZE | BUS_WIDTH_FRAME_SIZE);
	switch (bus_width_bytes) {
	case 1:
		bus_width |= BUS_WIDTH_8BIT;
		break;
	case 2:
		bus_width |= BUS_WIDTH_16BIT;
		break;
	case 4:
		bus_width |= BUS_WIDTH_32BIT;
		break;
	default:
		error = EINVAL;
		goto out;
	}
	switch (frame_words) {
	case 0:
	case 1:
		break;
	case 2:
		bus_width |= BUS_WIDTH_FRAME_2_WORDS;
		break;
	case 4:
		bus_width |= BUS_WIDTH_FRAME_4_WORDS;
		break;
	default:
		error = EINVAL;
		goto out;
	}

	/* set_unk28 */
	WR4(sc, REG_UNK_28, T603X_UNK_28_VAL);
	WR4(sc, REG_UNK_2C, T603X_UNK_28_VAL);
	WR4(sc, REG_BUS_WIDTH(ch->no), bus_width);
	/* FIFO limit and threshold: aurora's (arbitrary) numbers. */
	WR4(sc, REG_CHAN_FIFOCTL(ch->no),
	    FIFOCTL(0x30 * bus_width_bytes, 0x18 * bus_width_bytes));
out:
	sc->nrunning--;
	mtx_unlock(&sc->mtx);
	return (error);
}

int
apple_admac_start_cyclic(struct apple_admac_chan *ch, bus_addr_t iova,
    size_t buf_len, size_t period_len, void (*period_cb)(void *), void *arg)
{
	struct admac_softc *sc = ch->sc;

	/* Reports advance whole periods, and the residue is 32 bits. */
	if (buf_len == 0 || period_len == 0 || period_len > buf_len ||
	    buf_len % period_len != 0 || buf_len > UINT32_MAX ||
	    iova + buf_len < iova)
		return (EINVAL);

	mtx_lock(&sc->mtx);
	if (ch->running || !ch->has_carveout) {
		mtx_unlock(&sc->mtx);
		return (ch->running ? EBUSY : EINVAL);
	}
	ch->buf_addr = iova;
	ch->buf_len = buf_len;
	ch->period_len = period_len;
	ch->submitted_pos = 0;
	ch->reclaimed_pos = 0;
	ch->reported_pos = 0;
	ch->nperiods_outstanding = 0;
	ch->nacks = 0;
	ch->error = false;
	ch->cb = period_cb;
	ch->cbarg = arg;
	ch->gen++;
	ch->running = true;
	sc->nrunning++;

	/* aurora's admac_start_current_tx() */
	admac_reset_rings(ch);
	WR4(sc, REG_CHAN_CTL(ch->no), 0);
	if (admac_cyclic_write_one_desc(ch)) {
		admac_start_chan(ch);
		if (!admac_cyclic_write_desc(ch))
			admac_stop_chan(ch);
	}
	mtx_unlock(&sc->mtx);
	return (0);
}

void
apple_admac_stop(struct apple_admac_chan *ch)
{
	struct admac_softc *sc = ch->sc;

	mtx_lock(&sc->mtx);
	if (!ch->running) {
		mtx_unlock(&sc->mtx);
		return;
	}
	/* aurora's admac_terminate_all() then admac_synchronize()'s wait */
	admac_stop_chan(ch);
	admac_reset_rings(ch);
	ch->running = false;
	ch->nacks = 0;
	ch->gen++;
	admac_wait_stopped(ch);
	sc->nrunning--;
	mtx_unlock(&sc->mtx);
}

size_t
apple_admac_position(struct apple_admac_chan *ch)
{
	struct admac_softc *sc = ch->sc;
	size_t pos;

	mtx_lock(&sc->mtx);
	if (!ch->running)
		pos = 0;
	else if (ch->error)
		pos = ch->reported_pos % ch->buf_len;
	else
		pos = admac_cyclic_read_pos(ch) % ch->buf_len;
	mtx_unlock(&sc->mtx);
	return (pos);
}

bool
apple_admac_error(struct apple_admac_chan *ch)
{
	bool error;

	mtx_lock(&ch->sc->mtx);
	error = ch->running && ch->error;
	mtx_unlock(&ch->sc->mtx);
	return (error);
}

/*
 * The controller's DART streams, once. A stream with the firmware's live
 * tables gets our buffers in their vacant entries (aurora's locked DART);
 * one without (apple_dart_fw_attach() ENXIO) a private table of our own.
 */
static int
admac_dart_setup(struct admac_softc *sc)
{
	struct admac_stream *st;
	pcell_t *cells;
	phandle_t node;
	int error, i, n;

	mtx_lock(&sc->mtx);
	while (sc->streams_busy)
		mtx_sleep(&sc->streams_busy, &sc->mtx, 0, "admacds", 0);
	if (sc->streams_ready) {
		mtx_unlock(&sc->mtx);
		return (0);
	}
	sc->streams_busy = true;
	mtx_unlock(&sc->mtx);

	node = ofw_bus_get_node(sc->dev);
	n = OF_getencprop_alloc_multi(node, "iommus", sizeof(*cells),
	    (void **)&cells);
	if (n < 2 || n / 2 > ADMAC_STREAMS_MAX) {
		device_printf(sc->dev, "no or too many iommus (%d cells)\n", n);
		error = ENXIO;
		goto out;
	}
	error = 0;
	for (i = 0; i < n / 2 && error == 0; i++) {
		st = &sc->streams[i];
		st->dart = OF_node_from_xref(cells[2 * i]);
		st->sid = cells[2 * i + 1];
		st->fw = NULL;
		error = apple_dart_fw_attach(sc->dev, node, i, &st->fw);
		if (error == ENXIO)
			error = apple_dart_private_stream(sc->dev, st->dart,
			    st->sid);
	}
	if (error == 0)
		sc->nstreams = n / 2;
out:
	if (n > 0)
		OF_prop_free(cells);
	mtx_lock(&sc->mtx);
	sc->streams_busy = false;
	sc->streams_ready = error == 0;
	wakeup(&sc->streams_busy);
	mtx_unlock(&sc->mtx);
	return (error);
}

/*
 * Map [pa, pa + size) at one IOVA on each of the controller's streams: the
 * first firmware-table stream picks it among its vacant entries, the others
 * take it as is. Without such a stream, the next IOVA of the window.
 */
static int
admac_dart_map(struct admac_softc *sc, vm_paddr_t pa, size_t size,
    uint64_t *iovap)
{
	struct admac_stream *st;
	uint64_t iova;
	int error, i, first;

	error = admac_dart_setup(sc);
	if (error != 0)
		return (error);
	for (first = 0; first < sc->nstreams; first++)
		if (sc->streams[first].fw != NULL)
			break;
	if (first < sc->nstreams) {
		error = apple_dart_fw_map(sc->streams[first].fw, pa, size,
		    &iova);
		if (error != 0)
			return (error);
	} else {
		mtx_lock(&sc->mtx);
		if (sc->dma_next + size > sc->dma_end ||
		    sc->dma_next + size < sc->dma_next) {
			mtx_unlock(&sc->mtx);
			return (ENOSPC);
		}
		iova = sc->dma_next;
		sc->dma_next += size;
		mtx_unlock(&sc->mtx);
	}
	for (i = 0; i < sc->nstreams && error == 0; i++) {
		st = &sc->streams[i];
		if (i == first)
			continue;
		if (st->fw != NULL)
			error = apple_dart_fw_map_at(st->fw, iova, pa, size);
		else
			error = apple_dart_map_stream(sc->dev, st->dart,
			    st->sid, iova, pa, size);
	}
	if (error == 0)
		*iovap = iova;
	return (error);
}

int
apple_admac_alloc_buffer(struct apple_admac_chan *ch, size_t len, void **kva,
    bus_addr_t *iova)
{
	struct admac_softc *sc = ch->sc;
	uint64_t addr;
	void *va;
	size_t size;
	int error;

	if (len == 0)
		return (EINVAL);
	size = roundup2(len, ADMAC_PAGE);
	mtx_lock(&sc->mtx);
	if (ch->buf_busy) {
		mtx_unlock(&sc->mtx);
		return (EBUSY);
	}
	if (ch->kva != NULL && size <= ch->bufsize) {
		ch->buf_busy = true;
		mtx_unlock(&sc->mtx);
		*kva = ch->kva;
		*iova = ch->iova;
		return (0);
	}
	/* A larger buffer: the old one stays mapped, so it is never freed. */
	ch->buf_busy = true;
	mtx_unlock(&sc->mtx);

	/*
	 * Linux's dma_alloc_coherent for a non-coherent device: uncached.
	 * Below 2^42, the PAs a DART PTE holds.
	 */
	va = kmem_alloc_contig(size, M_WAITOK | M_ZERO, 0, (1ul << 42) - 1,
	    ADMAC_PAGE, 0, VM_MEMATTR_WRITE_COMBINING);
	if (va == NULL) {
		error = ENOMEM;
		goto fail;
	}
	addr = 0;
	error = admac_dart_map(sc, pmap_kextract((vm_offset_t)va), size, &addr);
	if (error != 0) {
		/* Some pages may be in a table already: leak rather than reuse. */
		device_printf(sc->dev, "ch%u: DART map of %#zx failed: %d\n",
		    ch->no, size, error);
		goto fail;
	}
	mtx_lock(&sc->mtx);
	ch->kva = va;
	ch->iova = addr;
	ch->bufsize = size;
	mtx_unlock(&sc->mtx);
	*kva = va;
	*iova = addr;
	return (0);
fail:
	mtx_lock(&sc->mtx);
	ch->buf_busy = false;
	mtx_unlock(&sc->mtx);
	return (error);
}

void
apple_admac_free_buffer(struct apple_admac_chan *ch, void *kva)
{

	mtx_lock(&ch->sc->mtx);
	if (kva == ch->kva)
		ch->buf_busy = false;
	mtx_unlock(&ch->sc->mtx);
}

void
apple_admac_release(struct apple_admac_chan *ch)
{
	struct admac_softc *sc = ch->sc;

	apple_admac_stop(ch);
	mtx_lock(&sc->mtx);
	while (sc->cb_busy != 0)
		mtx_sleep(&sc->cb_busy, &sc->mtx, 0, "admacrl", hz / 10);
	ch->cb = NULL;
	ch->cbarg = NULL;
	if (ch->has_carveout) {
		admac_free_sram_carveout(sc, admac_is_tx(ch->no),
		    ch->carveout);
		ch->has_carveout = false;
	}
	ch->claimed = false;
	mtx_unlock(&sc->mtx);
}

static int
admac_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Apple Audio DMA Controller");
	return (BUS_PROBE_DEFAULT);
}

static int
admac_attach(device_t dev)
{
	struct admac_softc *sc = device_get_softc(dev);
	phandle_t node;
	pcell_t nch, range[4];
	int i, rid;

	sc->dev = dev;
	node = ofw_bus_get_node(dev);
	if (OF_getencprop(node, "dma-channels", &nch, sizeof(nch)) !=
	    sizeof(nch) || nch == 0 || nch > NCHANNELS_MAX) {
		device_printf(dev, "missing or invalid dma-channels\n");
		return (ENXIO);
	}
	sc->nchannels = nch;
	/* The IOVA window of the controller's DART stream. */
	if (OF_getencprop(node, "apple,dma-range", range, sizeof(range)) !=
	    sizeof(range)) {
		device_printf(dev, "missing apple,dma-range\n");
		return (ENXIO);
	}
	sc->dma_base = (uint64_t)range[0] << 32 | range[1];
	sc->dma_end = sc->dma_base + ((uint64_t)range[2] << 32 | range[3]);
	sc->dma_next = roundup2(sc->dma_base, ADMAC_PAGE);

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);
	for (i = 0; i < sc->nchannels; i++) {
		sc->chans[i].sc = sc;
		sc->chans[i].no = i;
	}

	/*
	 * Mapped, never read here: the registers answer only once the audio
	 * driver has powered the fabric.
	 */
	rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "cannot map registers\n");
		goto fail;
	}
	/*
	 * Four IRQ outputs, the first usable one taken; the DT gives output
	 * 0 and placeholders for the rest.
	 */
	for (i = 0; i < 4; i++) {
		rid = i;
		sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
		    RF_ACTIVE);
		if (sc->irq != NULL)
			break;
	}
	if (sc->irq == NULL) {
		device_printf(dev, "no usable interrupt\n");
		goto fail;
	}
	sc->irq_index = i;
	if (bus_setup_intr(dev, sc->irq, INTR_TYPE_AV | INTR_MPSAFE, NULL,
	    admac_intr, sc, &sc->ih) != 0) {
		device_printf(dev, "cannot set up interrupt\n");
		goto fail;
	}
	OF_device_register_xref(OF_xref_from_node(node), dev);
	if (bootverbose)
		device_printf(dev, "%d channels, irq output %d, IOVA "
		    "%#jx-%#jx\n", sc->nchannels, sc->irq_index,
		    (uintmax_t)sc->dma_base, (uintmax_t)sc->dma_end);
	return (0);
fail:
	if (sc->irq != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, rman_get_rid(sc->irq),
		    sc->irq);
	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem);
	mtx_destroy(&sc->mtx);
	return (ENXIO);
}

static int
admac_detach(device_t dev)
{
	struct admac_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < sc->nchannels; i++)
		if (sc->chans[i].claimed || sc->chans[i].kva != NULL)
			return (EBUSY);
	OF_device_register_xref(OF_xref_from_node(ofw_bus_get_node(dev)),
	    NULL);
	bus_teardown_intr(dev, sc->irq, sc->ih);
	bus_release_resource(dev, SYS_RES_IRQ, rman_get_rid(sc->irq), sc->irq);
	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem);
	mtx_destroy(&sc->mtx);
	return (0);
}

static device_method_t admac_methods[] = {
	DEVMETHOD(device_probe,		admac_probe),
	DEVMETHOD(device_attach,	admac_attach),
	DEVMETHOD(device_detach,	admac_detach),
	DEVMETHOD_END
};

static driver_t admac_driver = {
	"apple_admac",
	admac_methods,
	sizeof(struct admac_softc)
};

DRIVER_MODULE(apple_admac, simplebus, admac_driver, 0, 0);
MODULE_VERSION(apple_admac, 1);
