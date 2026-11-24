/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple M5 laptop sound card (apple,macaudio on /sound): the sound(4) PCM
 * glue between the AOP audio service (apple_aop_audio), the base-ns ADMAC
 * (apple_admac) and the CS42L84 headphone codec (cs42l84). Follows the
 * "Headphone Jack" link of aurora's sound/soc/apple/macaudio.c and the
 * front-end of t8152_aop_audio.rs: 48 kHz stereo, 24-bit samples in 32-bit
 * words (ALSA S24_LE), DSP_A with an inverted bit clock, 64 bit clocks per
 * frame, the codec the clock consumer.
 *
 * Stage 1: the jack only. The speakers stay off (their amplifiers' shutdown
 * line is never raised here) until speaker protection exists.
 *
 * sound(4) calls trigger and the mixer with locks held, but the firmware and
 * the codec sleep, so power changes and volume run on a taskqueue. The DMA
 * buffer is the device's own: each period the PCM buffer's S32 samples are
 * converted into it (shifted down to 24 bits), a period ahead of the
 * hardware, so the PCM pointer runs two periods ahead of what is heard.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/taskqueue.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#ifdef HAVE_KERNEL_OPTION_HEADERS
#include "opt_snd.h"
#endif
#include <dev/sound/pcm/sound.h>
#include "mixer_if.h"

#include <arm64/apple/apple_admac.h>
#include <arm64/apple/apple_aop_audio.h>
#include <arm64/apple/cs42l84.h>

#define	MA_RATE		48000
#define	MA_CHANNELS	2
#define	MA_WORD		4
#define	MA_FRAME	(MA_CHANNELS * MA_WORD)
#define	MA_BCLK_RATIO	64			/* macaudio primary front-end */
#define	MA_SLOT_WIDTH	32
#define	MA_BUFSZ	(64 * 1024)
#define	MA_BLK_MIN	1024
#define	MA_NBLK_MIN	3

struct ma_softc;

struct ma_chan {
	struct ma_softc		*sc;
	struct pcm_channel	*pcm;
	struct snd_dbuf		*buf;
	struct apple_admac_chan	*dma;
	uint32_t		*dmabuf;	/* the device's buffer, S24 in 32 */
	bus_addr_t		dmaiova;
	uint32_t		blksz;
	uint32_t		nblk;
	/* Under sc->mtx: */
	bool			running;	/* PCM data flows into dmabuf */
	uint32_t		done;		/* periods the DMA finished */
};

struct ma_softc {
	struct snddev_info	info;		/* first: pcm_register needs it */
	device_t		dev;
	struct mtx		mtx;
	struct intr_config_hook	hook;
	device_t		aop_audio;
	device_t		codec;
	struct apple_aop_audio_stream *cout;
	bool			powered;	/* cout pwrd, codec on, DMA running */
	bool			want_play;
	u_int			vol_l, vol_r;
	struct taskqueue	*tq;
	struct task		power_task;
	struct task		vol_task;
	struct ma_chan		play;
};

static struct ofw_compat_data compat_data[] = {
	{ "apple,macaudio",	1 },
	{ NULL,			0 }
};

static uint32_t ma_fmt[] = {
	SND_FORMAT(AFMT_S32_LE, MA_CHANNELS, 0),
	0
};
static struct pcmchan_caps ma_caps = { MA_RATE, MA_RATE, ma_fmt, 0 };

/* ---- data path ---- */

/* Converts period blk of the PCM buffer into the device buffer. */
static void
ma_copy_period(struct ma_chan *ch, uint32_t blk)
{
	const int32_t *src;
	uint32_t *dst;
	u_int i, n;

	src = (const int32_t *)(ch->buf->buf +
	    blk * ch->blksz);
	dst = (uint32_t *)((uint8_t *)ch->dmabuf + blk * ch->blksz);
	n = ch->blksz / MA_WORD;
	if (!ch->running) {
		memset(dst, 0, ch->blksz);
		return;
	}
	for (i = 0; i < n; i++)
		dst[i] = (uint32_t)(src[i] >> 8);
}

/* ADMAC period callback (its interrupt thread; may not sleep). */
static void
ma_period(void *arg)
{
	struct ma_chan *ch = arg;
	struct ma_softc *sc = ch->sc;
	bool running;

	mtx_lock(&sc->mtx);
	ch->done++;
	/* The hardware plays period done; fill the one after it. */
	ma_copy_period(ch, (ch->done + 1) % ch->nblk);
	running = ch->running;
	mtx_unlock(&sc->mtx);
	if (running)
		chn_intr(ch->pcm);
}

/* ---- power (taskqueue) ---- */

static int
ma_power_up(struct ma_softc *sc)
{
	struct ma_chan *ch = &sc->play;
	uint32_t i;
	int error;

	if ((error = apple_aop_audio_stream_open(sc->aop_audio, "cout",
	    &sc->cout)) != 0) {
		device_printf(sc->dev, "cout open: %d\n", error);
		return (error);
	}
	if ((error = apple_admac_config(ch->dma, MA_WORD, MA_CHANNELS)) != 0 ||
	    (error = cs42l84_set_fmt(sc->codec, CS42L84_FMT_DSP_A)) != 0 ||
	    (error = cs42l84_set_sysclk(sc->codec,
	    MA_RATE * MA_BCLK_RATIO)) != 0 ||
	    (error = cs42l84_hw_params(sc->codec, CS42L84_DIR_PLAYBACK, MA_RATE,
	    24, MA_SLOT_WIDTH)) != 0) {
		device_printf(sc->dev, "stream setup: %d\n", error);
		goto close;
	}
	/* Prefill: two periods of PCM data, the rest silence. */
	mtx_lock(&sc->mtx);
	ch->done = 0;
	memset(ch->dmabuf, 0, ch->blksz * ch->nblk);
	for (i = 0; i < 2; i++)
		ma_copy_period(ch, i);
	mtx_unlock(&sc->mtx);
	/* The firmware starts the serializer and its clocks, then the codec. */
	if ((error = apple_aop_audio_stream_power(sc->cout, true)) != 0) {
		device_printf(sc->dev, "cout pwrd: %d\n", error);
		goto close;
	}
	if ((error = apple_admac_start_cyclic(ch->dma, ch->dmaiova,
	    ch->blksz * ch->nblk, ch->blksz, ma_period, ch)) != 0) {
		device_printf(sc->dev, "DMA start: %d\n", error);
		goto unpower;
	}
	if ((error = cs42l84_set_power(sc->codec, CS42L84_DIR_PLAYBACK,
	    true)) != 0) {
		device_printf(sc->dev, "codec power: %d\n", error);
		apple_admac_stop(ch->dma);
		goto unpower;
	}
	sc->powered = true;
	return (0);
unpower:
	apple_aop_audio_stream_power(sc->cout, false);
close:
	apple_aop_audio_stream_close(sc->cout);
	sc->cout = NULL;
	return (error);
}

static void
ma_power_down(struct ma_softc *sc)
{

	/* Codec first (muted), then the data, then the clocks. */
	cs42l84_set_power(sc->codec, CS42L84_DIR_PLAYBACK, false);
	apple_admac_stop(sc->play.dma);
	apple_aop_audio_stream_power(sc->cout, false);
	apple_aop_audio_stream_close(sc->cout);
	sc->cout = NULL;
	sc->powered = false;
}

static void
ma_power_task(void *arg, int pending)
{
	struct ma_softc *sc = arg;
	bool want;

	mtx_lock(&sc->mtx);
	want = sc->want_play;
	mtx_unlock(&sc->mtx);
	if (want && !sc->powered)
		ma_power_up(sc);
	else if (!want && sc->powered)
		ma_power_down(sc);
}

static void
ma_vol_task(void *arg, int pending)
{
	struct ma_softc *sc = arg;
	u_int l, r;

	mtx_lock(&sc->mtx);
	l = sc->vol_l;
	r = sc->vol_r;
	mtx_unlock(&sc->mtx);
	cs42l84_set_volume(sc->codec, l, r);
}

/* ---- sound(4) channel ---- */

static void *
ma_chan_init(kobj_t obj, void *devinfo, struct snd_dbuf *b,
    struct pcm_channel *c, int dir)
{
	struct ma_softc *sc = devinfo;
	struct ma_chan *ch = &sc->play;
	void *buf;

	if (dir != PCMDIR_PLAY)
		return (NULL);
	buf = malloc(MA_BUFSZ, M_DEVBUF, M_WAITOK | M_ZERO);
	if (sndbuf_setup(b, buf, MA_BUFSZ) != 0) {
		free(buf, M_DEVBUF);
		return (NULL);
	}
	ch->sc = sc;
	ch->pcm = c;
	ch->buf = b;
	ch->blksz = MA_BUFSZ / 8;
	ch->nblk = 8;
	return (ch);
}

static int
ma_chan_free(kobj_t obj, void *data)
{
	struct ma_chan *ch = data;

	free(ch->buf->buf, M_DEVBUF);
	return (0);
}

static int
ma_chan_setformat(kobj_t obj, void *data, uint32_t format)
{

	return (format == ma_fmt[0] ? 0 : EINVAL);
}

static uint32_t
ma_chan_setspeed(kobj_t obj, void *data, uint32_t speed)
{

	return (MA_RATE);
}

static uint32_t
ma_chan_setblocksize(kobj_t obj, void *data, uint32_t blksz)
{
	struct ma_chan *ch = data;

	/* Whole frames, a power of two dividing the buffer, three or more. */
	blksz = MAX(rounddown_pow_of_two(blksz), MA_BLK_MIN);
	blksz = MIN(blksz, MA_BUFSZ / MA_NBLK_MIN);
	blksz = rounddown_pow_of_two(blksz);
	if (ch->sc->powered)
		return (ch->blksz);	/* fixed while the DMA runs */
	ch->blksz = blksz;
	ch->nblk = MA_BUFSZ / blksz;
	sndbuf_resize(ch->buf, ch->nblk, ch->blksz);
	return (ch->blksz);
}

static int
ma_chan_trigger(kobj_t obj, void *data, int go)
{
	struct ma_chan *ch = data;
	struct ma_softc *sc = ch->sc;

	if (!PCMTRIG_COMMON(go))
		return (0);
	mtx_lock(&sc->mtx);
	switch (go) {
	case PCMTRIG_START:
		ch->running = true;
		sc->want_play = true;
		break;
	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		/* Silence from the next period on; the task powers down. */
		ch->running = false;
		sc->want_play = false;
		break;
	}
	mtx_unlock(&sc->mtx);
	taskqueue_enqueue(sc->tq, &sc->power_task);
	return (0);
}

static uint32_t
ma_chan_getptr(kobj_t obj, void *data)
{
	struct ma_chan *ch = data;
	uint32_t ptr;

	mtx_lock(&ch->sc->mtx);
	/* Converted up to the end of period done + 1. */
	ptr = ((ch->done + 2) % ch->nblk) * ch->blksz;
	mtx_unlock(&ch->sc->mtx);
	return (ptr);
}

static struct pcmchan_caps *
ma_chan_getcaps(kobj_t obj, void *data)
{

	return (&ma_caps);
}

static kobj_method_t ma_chan_methods[] = {
	KOBJMETHOD(channel_init,	ma_chan_init),
	KOBJMETHOD(channel_free,	ma_chan_free),
	KOBJMETHOD(channel_setformat,	ma_chan_setformat),
	KOBJMETHOD(channel_setspeed,	ma_chan_setspeed),
	KOBJMETHOD(channel_setblocksize, ma_chan_setblocksize),
	KOBJMETHOD(channel_trigger,	ma_chan_trigger),
	KOBJMETHOD(channel_getptr,	ma_chan_getptr),
	KOBJMETHOD(channel_getcaps,	ma_chan_getcaps),
	KOBJMETHOD_END
};
CHANNEL_DECLARE(ma_chan);

/* ---- mixer ---- */

static int
ma_mix_init(struct snd_mixer *m)
{

	mix_setdevs(m, SOUND_MASK_VOLUME | SOUND_MASK_PCM);
	return (0);
}

static int
ma_mix_set(struct snd_mixer *m, unsigned dev, unsigned left, unsigned right)
{
	struct ma_softc *sc = mix_getdevinfo(m);

	if (dev != SOUND_MIXER_VOLUME && dev != SOUND_MIXER_PCM)
		return (-1);
	/*
	 * Hearing safety for now: the mixer's 100 is the codec's power-up
	 * ceiling (-12 dB), so sound(4)'s default 75 lands near -24 dB.
	 */
	mtx_lock(&sc->mtx);
	sc->vol_l = MIN(left, 100) * CS42L84_VOL_POWERUP_MAX / 100;
	sc->vol_r = MIN(right, 100) * CS42L84_VOL_POWERUP_MAX / 100;
	mtx_unlock(&sc->mtx);
	taskqueue_enqueue(sc->tq, &sc->vol_task);
	return (left | (right << 8));
}

static kobj_method_t ma_mixer_methods[] = {
	KOBJMETHOD(mixer_init,	ma_mix_init),
	KOBJMETHOD(mixer_set,	ma_mix_set),
	KOBJMETHOD_END
};
MIXER_DECLARE(ma_mixer);

/* ---- attach ---- */

/* The apple_aop_audio device for an "aop_audio" node. */
static device_t
ma_find_aop_audio(phandle_t node)
{
	devclass_t dc;
	device_t dev;
	int i;

	if ((dc = devclass_find("apple_aop_audio")) == NULL)
		return (NULL);
	for (i = 0; i < devclass_get_maxunit(dc); i++)
		if ((dev = devclass_get_device(dc, i)) != NULL &&
		    device_is_attached(dev) && ofw_bus_get_node(dev) == node)
			return (dev);
	return (NULL);
}

/* The "Headphone Jack" link's cpu and codec devices. */
static int
ma_find_jack(struct ma_softc *sc)
{
	phandle_t link, cpu, codec;
	pcell_t cells[2];
	char name[32];

	for (link = OF_child(ofw_bus_get_node(sc->dev)); link != 0;
	    link = OF_peer(link)) {
		if (OF_getprop(link, "link-name", name, sizeof(name)) > 0 &&
		    strcmp(name, "Headphone Jack") == 0)
			break;
	}
	if (link == 0 || (cpu = ofw_bus_find_child(link, "cpu")) == 0 ||
	    (codec = ofw_bus_find_child(link, "codec")) == 0) {
		device_printf(sc->dev, "no Headphone Jack link\n");
		return (ENXIO);
	}
	if (OF_getencprop(cpu, "sound-dai", cells, sizeof(cells)) <= 0 ||
	    (sc->aop_audio = ma_find_aop_audio(OF_node_from_xref(cells[0]))) ==
	    NULL) {
		device_printf(sc->dev, "AOP audio is not attached\n");
		return (ENXIO);
	}
	if (OF_getencprop(codec, "sound-dai", cells, sizeof(cells[0])) <= 0 ||
	    (sc->codec = OF_device_from_xref(cells[0])) == NULL) {
		device_printf(sc->dev, "the jack codec is not attached\n");
		return (ENXIO);
	}
	return (0);
}

static void
ma_init(void *arg)
{
	struct ma_softc *sc = arg;
	struct ma_chan *ch = &sc->play;
	phandle_t xref;
	uint32_t cell;
	void *kva;
	int error;

	if (sc->hook.ich_func != NULL)
		config_intrhook_disestablish(&sc->hook);
	if (ma_find_jack(sc) != 0)
		return;
	if ((error = apple_aop_audio_dma(sc->aop_audio, "cout", &xref,
	    &cell)) != 0 ||
	    (error = apple_admac_get_channel(xref, cell, &ch->dma)) != 0) {
		device_printf(sc->dev, "cout DMA channel: %d\n", error);
		return;
	}
	if ((error = apple_admac_alloc_buffer(ch->dma, MA_BUFSZ, &kva,
	    &ch->dmaiova)) != 0) {
		device_printf(sc->dev, "DMA buffer: %d\n", error);
		apple_admac_release(ch->dma);
		ch->dma = NULL;
		return;
	}
	ch->dmabuf = kva;
	cs42l84_get_volume(sc->codec, &sc->vol_l, &sc->vol_r);

	pcm_init(sc->dev, sc);
	pcm_addchan(sc->dev, PCMDIR_PLAY, &ma_chan_class, sc);
	if (pcm_register(sc->dev, "at macaudio (headphone jack)") != 0) {
		device_printf(sc->dev, "pcm_register failed\n");
		return;
	}
	mixer_init(sc->dev, &ma_mixer_class, sc);
}

static int
ma_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev) ||
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Apple M5 audio (headphone jack)");
	return (BUS_PROBE_DEFAULT);
}

static int
ma_attach(device_t dev)
{
	struct ma_softc *sc = device_get_softc(dev);

	sc->dev = dev;
	mtx_init(&sc->mtx, device_get_nameunit(dev), "macaudio", MTX_DEF);
	sc->tq = taskqueue_create("macaudio", M_WAITOK,
	    taskqueue_thread_enqueue, &sc->tq);
	taskqueue_start_threads(&sc->tq, 1, PI_SOFT, "%s",
	    device_get_nameunit(dev));
	TASK_INIT(&sc->power_task, 0, ma_power_task, sc);
	TASK_INIT(&sc->vol_task, 0, ma_vol_task, sc);
	if (cold) {
		sc->hook.ich_func = ma_init;
		sc->hook.ich_arg = sc;
		if (config_intrhook_establish(&sc->hook) != 0)
			return (ENOMEM);
	} else
		ma_init(sc);
	return (0);
}

static int
ma_detach(device_t dev)
{
	struct ma_softc *sc = device_get_softc(dev);
	int error;

	if ((error = pcm_unregister(dev)) != 0)
		return (error);
	mtx_lock(&sc->mtx);
	sc->want_play = false;
	mtx_unlock(&sc->mtx);
	taskqueue_enqueue(sc->tq, &sc->power_task);
	taskqueue_drain_all(sc->tq);
	taskqueue_free(sc->tq);
	if (sc->play.dma != NULL) {
		apple_admac_free_buffer(sc->play.dma, sc->play.dmabuf);
		apple_admac_release(sc->play.dma);
	}
	mtx_destroy(&sc->mtx);
	return (0);
}

static device_method_t ma_methods[] = {
	DEVMETHOD(device_probe,		ma_probe),
	DEVMETHOD(device_attach,	ma_attach),
	DEVMETHOD(device_detach,	ma_detach),
	DEVMETHOD_END
};

static driver_t ma_driver = {
	"pcm",
	ma_methods,
	sizeof(struct ma_softc),
};

DRIVER_MODULE(apple_macaudio, ofwbus, ma_driver, NULL, NULL);
DRIVER_MODULE(apple_macaudio, simplebus, ma_driver, NULL, NULL);
MODULE_DEPEND(apple_macaudio, sound, SOUND_MINVER, SOUND_PREFVER,
    SOUND_MAXVER);
MODULE_DEPEND(apple_macaudio, apple_aop, 1, 1, 1);
MODULE_DEPEND(apple_macaudio, apple_admac, 1, 1, 1);
MODULE_DEPEND(apple_macaudio, cs42l84, 1, 1, 1);
MODULE_VERSION(apple_macaudio, 1);
