/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple T8142 (M5) AOP audio, the firmware control: the AOPAudioService's
 * device attach, property get/set, the pw0/pwrd power requests and the MCA
 * serializer power leaves the AP owns. Follows the non-ASoC part of aurora's
 * sound/soc/apple/t8152_aop_audio.rs (HW_CFG_T8142); the PCM/DAI plumbing
 * is the sound glue's (apple_aop_audio.h), and the microphone array (hpai)
 * is not ported yet (its LEAP leaves are looked up, unused).
 *
 * Power safety: the MCA leaves' parents (AUDIO_A, AUDIO_FR, AUDIO_P,
 * AUDIO_BASE_NS_A) are the firmware's and are off until a service's "pw0 ";
 * a leaf is only switched on after that, with retries while the firmware
 * raises the fabric, and only PMGR registers are touched here.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/sx.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/armreg.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <arm64/apple/apple_aop.h>
#include <arm64/apple/apple_aop_audio.h>

#define	FOURCC(s)		((uint32_t)(s)[0] << 24 | (uint32_t)(s)[1] << 16 | \
				    (uint32_t)(s)[2] << 8 | (uint32_t)(s)[3])

#define	EPIC_SUBTYPE_WRAPPED_CALL	0x20
#define	CALLTYPE_AUDIO_ATTACH_DEVICE	0xc3000002
#define	CALLTYPE_AUDIO_GET_PROP		0xc3000004
#define	CALLTYPE_AUDIO_SET_PROP		0xc3000005
#define	PROP_POWER_STATE		200
#define	PROP_POWER_REQUEST		202
#define	AUDIO_RET_BUSY			0xe00002d5	/* already attached */
#define	AOP_TIMEBASE_HZ			24000000ull

/* Wire layouts (packed(4) in the Rust), by offset. */
#define	REQ_UNK0			4
#define	REQ_CALLTYPE			8
#define	REQ_LEN				32	/* u64 */
#define	REQ_DEV_ID			40
#define	REQ_MODIFIER			44
#define	ATTACH_SIZE			48
#define	GETPROP_UNK6			48
#define	GETPROP_SIZE			52
#define	SETPROP_LEN2			48
#define	SETPROP_DATA			52
#define	POWER_REQUEST_SIZE		48	/* dev_id, sequence, ts, 0, state */
#define	SETPROP_SIZE			(SETPROP_DATA + POWER_REQUEST_SIZE)

#define	PD_GET_ATTEMPTS			60
#define	PD_GET_RETRY_MS			10
#define	PD_POLL_US			1000
#define	MCA_COUNT			6
#define	LEAP_COUNT			3

#define	PMGR_RESET		(1u << 31)
#define	PMGR_AUTO_ENABLE	(1u << 28)
#define	PMGR_PS_RESET		(1u << 12)
#define	PMGR_BUSY		(1u << 11)
#define	PMGR_DEV_DISABLE	(1u << 10)
#define	PMGR_FLAGS		((1u << 9) | (1u << 8))
#define	PMGR_PS_ACTUAL(r)	(((r) >> 4) & 0xf)
#define	PMGR_PS_TARGET		0xf
#define	PMGR_PS_ACTIVE		0xf
#define	PMGR_PS_OFF		0

static const char *const mca_names[MCA_COUNT] = {
	"mca0", "mca1", "mca2", "mca3", "mca4", "mca5"
};
static const char *const leap_names[LEAP_COUNT] = {
	"leap_s_a", "leap_rx0", "leap_mca"
};

struct aop_audio_pd {
	volatile uint32_t *reg;
	vm_paddr_t	pa;
	u_int		refs;
};

struct aop_audio_service {
	const char	*name;
	uint32_t	dev_id;
	uint32_t	sequence;
	bool		running;		/* in pwrd */
	bool		restart_on_prepare;
};

struct apple_aop_audio_stream {
	struct apple_aop_audio_softc *sc;
	const char	*name;
	struct aop_audio_service *svc;	/* NULL: sense */
	bool		open;
};

struct apple_aop_audio_softc {
	device_t		dev;
	device_t		aop;
	struct sx		lock;
	struct apple_aop_service svc;
	struct aop_audio_service cout, cin, spkr;
	struct aop_audio_pd	mca[MCA_COUNT];
	struct aop_audio_pd	leap[LEAP_COUNT];
	bool			has_leap;
	uint32_t		spkr_channels, sense_channels;
	struct apple_aop_audio_stream streams[4];	/* cout cin spkr sense */
};

/* The firmware's 24 MHz timebase, from the architected counter. */
static uint64_t
aop_audio_timestamp(void)
{
	uint64_t counter, rate;

	counter = READ_SPECIALREG(cntvct_el0);
	rate = READ_SPECIALREG(cntfrq_el0);
	return ((counter / rate) * AOP_TIMEBASE_HZ +
	    (counter % rate) * AOP_TIMEBASE_HZ / rate);
}

static int
aop_audio_call(struct apple_aop_audio_softc *sc, const uint8_t *msg,
    size_t len, void *reply, size_t *replylen, uint32_t *ret)
{
	return (apple_aop_epic_call(sc->aop, &sc->svc,
	    EPIC_SUBTYPE_WRAPPED_CALL, msg, len, reply, replylen, ret, 0));
}

static void
aop_audio_req(uint8_t *b, uint32_t calltype, uint64_t len, uint32_t dev_id)
{
	le32enc(b + REQ_UNK0, 0xffffffff);
	le32enc(b + REQ_CALLTYPE, calltype);
	le64enc(b + REQ_LEN, len);
	le32enc(b + REQ_DEV_ID, dev_id);
}

/* The firmware keeps attachments for its lifetime: a reload sees busy. */
static int
aop_audio_attach_dev(struct apple_aop_audio_softc *sc,
    struct aop_audio_service *svc)
{
	uint8_t b[ATTACH_SIZE];
	uint32_t ret;
	int error;

	memset(b, 0, sizeof(b));
	aop_audio_req(b, CALLTYPE_AUDIO_ATTACH_DEVICE, 0x2c, svc->dev_id);
	error = aop_audio_call(sc, b, sizeof(b), NULL, NULL, &ret);
	if (error != 0)
		return (error);
	if (ret != 0 && ret != AUDIO_RET_BUSY) {
		device_printf(sc->dev, "unable to attach %s: %#x\n", svc->name,
		    ret);
		return (EIO);
	}
	return (0);
}

static int
aop_audio_power_state(struct apple_aop_audio_softc *sc,
    struct aop_audio_service *svc, uint32_t *state)
{
	uint8_t b[GETPROP_SIZE], r[8];
	size_t rlen = sizeof(r);
	uint32_t ret;
	int error;

	memset(b, 0, sizeof(b));
	aop_audio_req(b, CALLTYPE_AUDIO_GET_PROP, 0x30, svc->dev_id);
	le32enc(b + REQ_MODIFIER, PROP_POWER_STATE);
	le32enc(b + GETPROP_UNK6, 1);
	error = aop_audio_call(sc, b, sizeof(b), r, &rlen, &ret);
	if (error != 0)
		return (error);
	if (ret != 0 || rlen < 8)
		return (EIO);
	*state = le32dec(r + 4);
	return (0);
}

static int
aop_audio_set_power(struct apple_aop_audio_softc *sc,
    struct aop_audio_service *svc, uint32_t state)
{
	uint8_t b[SETPROP_SIZE], *pr;
	uint32_t ret, now;
	int error;

	memset(b, 0, sizeof(b));
	aop_audio_req(b, CALLTYPE_AUDIO_SET_PROP, POWER_REQUEST_SIZE + 0x30,
	    svc->dev_id);
	le32enc(b + REQ_MODIFIER, PROP_POWER_REQUEST);
	le32enc(b + SETPROP_LEN2, POWER_REQUEST_SIZE);
	pr = b + SETPROP_DATA;
	le32enc(pr, svc->dev_id);
	le32enc(pr + 4, ++svc->sequence);
	le64enc(pr + 8, aop_audio_timestamp());
	le32enc(pr + 20, state);
	error = aop_audio_call(sc, b, sizeof(b), NULL, NULL, &ret);
	if (error != 0)
		return (error);
	if (ret != 0) {
		device_printf(sc->dev, "%s: power request %#x rejected: %#x\n",
		    svc->name, state, ret);
		return (EIO);
	}
	error = aop_audio_power_state(sc, svc, &now);
	if (error != 0)
		return (error);
	if (now != state) {
		device_printf(sc->dev, "%s: power state %#x after requesting "
		    "%#x\n", svc->name, now, state);
		return (EIO);
	}
	return (0);
}

/* ---- the AP's power leaves (aurora's apple_pmgr_ps_set) ---- */

static int
aop_audio_pd_lookup(struct apple_aop_audio_softc *sc, const char *name,
    struct aop_audio_pd *pd)
{
	phandle_t node, ps;
	pcell_t *cells, reg;
	bus_addr_t pa;
	bus_size_t size;
	int idx, n;

	node = ofw_bus_get_node(sc->dev);
	if (ofw_bus_find_string_index(node, "power-domain-names", name,
	    &idx) != 0)
		return (ENOENT);
	/* #power-domain-cells = <0>: one phandle per domain. */
	n = OF_getencprop_alloc_multi(node, "power-domains", sizeof(*cells),
	    (void **)&cells);
	if (n <= idx) {
		if (n > 0)
			OF_prop_free(cells);
		return (ENXIO);
	}
	ps = OF_node_from_xref(cells[idx]);
	OF_prop_free(cells);
	if (!ofw_bus_node_is_compatible(ps, "apple,t8103-pmgr-pwrstate") ||
	    OF_getencprop(ps, "reg", &reg, sizeof(reg)) <= 0 ||
	    ofw_reg_to_paddr(OF_parent(ps), 0, &pa, &size, NULL) != 0)
		return (ENXIO);
	pd->pa = pa + reg;
	pd->reg = pmap_mapdev(pd->pa, sizeof(uint32_t));
	return (0);
}

static int
aop_audio_ps_set(struct aop_audio_pd *pd, uint32_t state)
{
	uint32_t val;
	int i;

	val = *pd->reg;
	if (PMGR_PS_ACTUAL(val) == state && (state == PMGR_PS_OFF ||
	    (val & (PMGR_RESET | PMGR_PS_RESET | PMGR_DEV_DISABLE)) == 0))
		return (0);
	val &= ~(PMGR_FLAGS | PMGR_RESET | PMGR_PS_TARGET | PMGR_AUTO_ENABLE |
	    PMGR_PS_RESET | PMGR_BUSY | PMGR_DEV_DISABLE);
	*pd->reg = val | state;
	for (i = 0; i < PD_POLL_US; i++) {
		if (PMGR_PS_ACTUAL(*pd->reg) == state)
			return (0);
		DELAY(1);
	}
	return (ETIMEDOUT);
}

/* A reference on the leaf, retrying while the firmware raises the parents. */
static int
aop_audio_pd_get(struct aop_audio_pd *pd)
{
	int error, i;

	if (pd->refs++ > 0)
		return (0);
	for (i = 0; i < PD_GET_ATTEMPTS; i++) {
		error = aop_audio_ps_set(pd, PMGR_PS_ACTIVE);
		if (error == 0)
			return (0);
		(void)aop_audio_ps_set(pd, PMGR_PS_OFF);
		pause("aoppd", MAX(1, PD_GET_RETRY_MS * hz / 1000));
	}
	pd->refs--;
	return (error);
}

static void
aop_audio_pd_put(struct aop_audio_pd *pd)
{
	if (pd->refs == 0 || --pd->refs > 0)
		return;
	(void)aop_audio_ps_set(pd, PMGR_PS_OFF);
}

static int
aop_audio_mca_up(struct apple_aop_audio_softc *sc)
{
	int error, i;

	for (i = 0; i < MCA_COUNT; i++) {
		error = aop_audio_pd_get(&sc->mca[i]);
		if (error != 0) {
			device_printf(sc->dev, "MCA leaf %d (%#jx) did not power "
			    "up: %d\n", i, (uintmax_t)sc->mca[i].pa, error);
			while (i-- > 0)
				aop_audio_pd_put(&sc->mca[i]);
			return (error);
		}
	}
	return (0);
}

static void
aop_audio_mca_down(struct apple_aop_audio_softc *sc)
{
	int i;

	for (i = MCA_COUNT - 1; i >= 0; i--)
		aop_audio_pd_put(&sc->mca[i]);
}

/* ---- streams ---- */

static int
aop_audio_run(struct apple_aop_audio_softc *sc, struct aop_audio_service *svc)
{
	int error;

	if (svc->running)
		return (0);
	error = aop_audio_set_power(sc, svc, FOURCC("pwrd"));
	if (error == 0)
		svc->running = true;
	return (error);
}

int
apple_aop_audio_stream_open(device_t dev, const char *name,
    struct apple_aop_audio_stream **sp)
{
	struct apple_aop_audio_softc *sc = device_get_softc(dev);
	struct apple_aop_audio_stream *s;
	struct aop_audio_service *svc;
	int error, i;

	for (i = 0; i < nitems(sc->streams); i++)
		if (strcmp(sc->streams[i].name, name) == 0)
			break;
	if (i == nitems(sc->streams))
		return (ENOENT);
	s = &sc->streams[i];
	sx_xlock(&sc->lock);
	if (s->open) {
		sx_xunlock(&sc->lock);
		return (EBUSY);
	}
	svc = s->svc;
	if (svc == NULL) {
		/*
		 * The sense capture: its ADMAC channel only answers while the
		 * firmware holds the fabric up, which it does in pw0 or pwrd.
		 */
		error = aop_audio_attach_dev(sc, &sc->spkr);
		if (error == 0 && !sc->spkr.running)
			error = aop_audio_set_power(sc, &sc->spkr,
			    FOURCC("pw0 "));
	} else {
		error = aop_audio_attach_dev(sc, svc);
		if (error == 0)
			error = aop_audio_set_power(sc, svc, FOURCC("pw0 "));
		if (error == 0) {
			error = aop_audio_mca_up(sc);
			if (error != 0)
				(void)aop_audio_set_power(sc, svc,
				    FOURCC("pw0 "));
		}
		if (error == 0)
			svc->running = false;
	}
	if (error == 0) {
		s->open = true;
		*sp = s;
	}
	sx_xunlock(&sc->lock);
	return (error);
}

int
apple_aop_audio_stream_power(struct apple_aop_audio_stream *s, bool running)
{
	struct apple_aop_audio_softc *sc = s->sc;
	struct aop_audio_service *svc = s->svc;
	int error = 0;

	if (svc == NULL)
		return (0);
	sx_xlock(&sc->lock);
	if (running) {
		/* prepare */
		if (svc->restart_on_prepare && svc->running) {
			error = aop_audio_set_power(sc, svc, FOURCC("pw0 "));
			if (error == 0)
				svc->running = false;
		}
		if (error == 0)
			error = aop_audio_run(sc, svc);
	} else if (svc->running) {
		error = aop_audio_set_power(sc, svc, FOURCC("pw0 "));
		svc->running = false;
	}
	sx_xunlock(&sc->lock);
	return (error);
}

void
apple_aop_audio_stream_close(struct apple_aop_audio_stream *s)
{
	struct apple_aop_audio_softc *sc = s->sc;
	struct aop_audio_service *svc = s->svc;
	int error;

	sx_xlock(&sc->lock);
	if (s->open && svc != NULL) {
		if (svc->running) {
			error = aop_audio_set_power(sc, svc, FOURCC("pw0 "));
			if (error != 0)
				device_printf(sc->dev, "%s: pw0 failed: %d\n",
				    svc->name, error);
			svc->running = false;
		}
		aop_audio_mca_down(sc);
	}
	s->open = false;
	sx_xunlock(&sc->lock);
}

void
apple_aop_audio_stream_info(struct apple_aop_audio_stream *s,
    struct apple_aop_audio_info *info)
{
	struct apple_aop_audio_softc *sc = s->sc;

	memset(info, 0, sizeof(*info));
	info->rate = 48000;
	info->sample_bits = 24;
	info->word_bytes = 4;
	if (strcmp(s->name, "cout") == 0) {
		info->channels = 2;
		info->playback = true;
	} else if (strcmp(s->name, "cin") == 0)
		info->channels = 1;
	else if (strcmp(s->name, "spkr") == 0) {
		info->channels = sc->spkr_channels;
		info->playback = true;
	} else {
		info->channels = sc->sense_channels;
		info->sample_bits = 16;
		info->word_bytes = 2;
		info->realign = true;
	}
	info->port_window = MIN(info->channels, 4);
}

int
apple_aop_audio_dma(device_t dev, const char *name, phandle_t *xrefp,
    uint32_t *cellp)
{
	phandle_t node;
	pcell_t *cells, ncells;
	int error, i, idx, n, pos;

	node = ofw_bus_get_node(dev);
	if (ofw_bus_find_string_index(node, "dma-names", name, &idx) != 0)
		return (ENOENT);
	n = OF_getencprop_alloc_multi(node, "dmas", sizeof(*cells),
	    (void **)&cells);
	if (n <= 0)
		return (ENXIO);
	error = ENXIO;
	for (i = 0, pos = 0; pos < n; i++) {
		if (OF_getencprop(OF_node_from_xref(cells[pos]), "#dma-cells",
		    &ncells, sizeof(ncells)) <= 0 || pos + 1 + ncells > n)
			break;
		if (i == idx) {
			if (ncells < 1)
				break;
			*xrefp = cells[pos];
			*cellp = cells[pos + 1];
			error = 0;
			break;
		}
		pos += 1 + ncells;
	}
	OF_prop_free(cells);
	return (error);
}

/* ---- newbus ---- */

static int
apple_aop_audio_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "apple,t8142-aop-audio"))
		return (ENXIO);
	device_set_desc(dev, "Apple AOP audio");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_aop_audio_attach(device_t dev)
{
	struct apple_aop_audio_softc *sc = device_get_softc(dev);
	struct aop_audio_service *svcs[3];
	pcell_t amps;
	int error, i, nleap;

	sc->dev = dev;
	sc->aop = device_get_parent(dev);
	sx_init(&sc->lock, "apple_aop_audio");
	sc->spkr_channels = 4;
	sc->sense_channels = 8;
	if (OF_getencprop(ofw_bus_get_node(dev), "apple,speaker-amps", &amps,
	    sizeof(amps)) == sizeof(amps)) {
		if (amps != 4 && amps != 6) {
			device_printf(dev, "unsupported apple,speaker-amps %u\n",
			    amps);
			error = EINVAL;
			goto fail;
		}
		sc->spkr_channels = amps;
		sc->sense_channels = 2 * amps;
	}

	/* Every serializer leaf is required: the firmware's streams run on them. */
	for (i = 0; i < MCA_COUNT; i++) {
		error = aop_audio_pd_lookup(sc, mca_names[i], &sc->mca[i]);
		if (error != 0) {
			device_printf(dev, "power domain %s: %d\n",
			    mca_names[i], error);
			goto fail;
		}
	}
	for (i = nleap = 0; i < LEAP_COUNT; i++)
		if (aop_audio_pd_lookup(sc, leap_names[i], &sc->leap[i]) == 0)
			nleap++;
	if (nleap != 0 && nleap != LEAP_COUNT)
		device_printf(dev, "incomplete microphone power domains\n");
	sc->has_leap = nleap == LEAP_COUNT;

	sc->cout = (struct aop_audio_service){ "cout", FOURCC("cout"), 0,
	    false, true };
	sc->cin = (struct aop_audio_service){ "cin", FOURCC("cin "), 0,
	    false, true };
	sc->spkr = (struct aop_audio_service){ "spkr", FOURCC("spkr"), 0,
	    false, false };
	sc->streams[0] = (struct apple_aop_audio_stream){ sc, "cout",
	    &sc->cout, false };
	sc->streams[1] = (struct apple_aop_audio_stream){ sc, "cin",
	    &sc->cin, false };
	sc->streams[2] = (struct apple_aop_audio_stream){ sc, "spkr",
	    &sc->spkr, false };
	sc->streams[3] = (struct apple_aop_audio_stream){ sc, "sense", NULL,
	    false };

	error = apple_aop_service_find(sc->aop, "aop-audio",
	    APPLE_AOP_CALL_TIMEOUT_MS, &sc->svc);
	if (error != 0) {
		device_printf(dev, "no aop-audio service: %d\n", error);
		goto fail;
	}
	svcs[0] = &sc->cout;
	svcs[1] = &sc->cin;
	svcs[2] = &sc->spkr;
	for (i = 0; i < nitems(svcs); i++) {
		error = aop_audio_attach_dev(sc, svcs[i]);
		if (error != 0)
			goto fail;
	}
	device_printf(dev, "endpoint %#04x channel %u; speaker %u amps%s\n",
	    sc->svc.endpoint, sc->svc.channel, sc->spkr_channels,
	    sc->has_leap ? "; microphones not supported yet" : "");
	return (0);
fail:
	sx_destroy(&sc->lock);
	return (error);
}

static int
apple_aop_audio_detach(device_t dev)
{
	struct apple_aop_audio_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < nitems(sc->streams); i++)
		if (sc->streams[i].open)
			return (EBUSY);
	sx_destroy(&sc->lock);
	return (0);
}

static device_method_t apple_aop_audio_methods[] = {
	DEVMETHOD(device_probe,		apple_aop_audio_probe),
	DEVMETHOD(device_attach,	apple_aop_audio_attach),
	DEVMETHOD(device_detach,	apple_aop_audio_detach),
	DEVMETHOD_END
};

static driver_t apple_aop_audio_driver = {
	"apple_aop_audio",
	apple_aop_audio_methods,
	sizeof(struct apple_aop_audio_softc),
};

DRIVER_MODULE(apple_aop_audio, apple_aop, apple_aop_audio_driver, NULL, NULL);
MODULE_DEPEND(apple_aop_audio, apple_aop, 1, 1, 1);
MODULE_VERSION(apple_aop_audio, 1);
