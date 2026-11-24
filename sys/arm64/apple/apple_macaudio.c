/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple M5 laptop sound card (apple,macaudio on /sound): the sound(4) PCM
 * glue between the AOP audio service (apple_aop_audio), the base-ns ADMAC
 * (apple_admac) and the CS42L84 headphone codec (cs42l84). Follows the
 * "Headphone Jack" link of aurora's sound/soc/apple/macaudio.c and the
 * front-end of t8152_aop_audio.rs: 48 kHz, 24-bit samples in 32-bit words
 * (ALSA S24_LE), DSP_A with an inverted bit clock, 64 bit clocks per frame,
 * the codec the clock consumer. Playback is "cout" (stereo), capture "cin"
 * (the headset microphone, mono).
 *
 * Stage 1: the jack only. The speakers stay off (their amplifiers' shutdown
 * line is never raised here) until speaker protection exists.
 *
 * sound(4) calls trigger and the mixer with locks held, but the firmware and
 * the codec sleep, so power changes and volume run on a taskqueue. The DMA
 * buffers are the device's own; each period is converted between them and
 * the PCM buffers (S32 <-> 24 bits in 32), playback a period ahead of the
 * hardware, capture as each period completes.
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
#define	MA_WORD		4
#define	MA_BCLK_RATIO	64			/* macaudio primary front-end */
#define	MA_SLOT_WIDTH	32
#define	MA_BUFSZ	(64 * 1024)
#define	MA_BLK_MIN	1024
#define	MA_NBLK_MIN	3

struct ma_softc;

struct ma_chan {
	struct ma_softc		*sc;
	int			dir;		/* PCMDIR_PLAY or PCMDIR_REC */
	const char		*stream;	/* "cout" or "cin" */
	int			codec_dir;	/* CS42L84_DIR_* */
	u_int			channels;
	uint32_t		fmt[2];
	struct pcmchan_caps	caps;
	struct pcm_channel	*pcm;
	struct snd_dbuf		*buf;
	struct apple_admac_chan	*dma;
	uint32_t		*dmabuf;	/* the device's buffer, S24 in 32 */
	bus_addr_t		dmaiova;
	uint32_t		blksz;
	uint32_t		nblk;
	/* Taskqueue only: */
	struct apple_aop_audio_stream *aop;
	bool			powered;	/* stream pwrd, codec on, DMA on */
	/* Under sc->mtx: */
	bool			want;
	bool			running;	/* data flows to or from the PCM */
	uint32_t		done;		/* periods the DMA finished */
};

struct ma_softc {
	struct snddev_info	info;		/* first: pcm_register needs it */
	device_t		dev;
	struct mtx		mtx;
	struct intr_config_hook	hook;
	device_t		aop_audio;
	device_t		codec;
	bool			codec_fmt;	/* format and clock set */
	u_int			vol_l, vol_r;
	struct taskqueue	*tq;
	struct task		power_task;
	struct task		vol_task;
	struct ma_chan		play;
	struct ma_chan		rec;
};

static struct ofw_compat_data compat_data[] = {
	{ "apple,macaudio",	1 },
	{ NULL,			0 }
};

/* ---- data path ---- */

/* Playback: PCM period blk (S32) into the device buffer, or silence. */
static void
ma_play_period(struct ma_chan *ch, uint32_t blk)
{
	const int32_t *src;
	uint32_t *dst;
	u_int i, n;

	src = (const int32_t *)(ch->buf->buf + blk * ch->blksz);
	dst = (uint32_t *)((uint8_t *)ch->dmabuf + blk * ch->blksz);
	n = ch->blksz / MA_WORD;
	if (!ch->running) {
		memset(dst, 0, ch->blksz);
		return;
	}
	for (i = 0; i < n; i++)
		dst[i] = (uint32_t)(src[i] >> 8);
}

/* Capture: device period blk (24 bits in 32) into the PCM buffer as S32. */
static void
ma_rec_period(struct ma_chan *ch, uint32_t blk)
{
	const uint32_t *src;
	int32_t *dst;
	u_int i, n;

	src = (const uint32_t *)((uint8_t *)ch->dmabuf + blk * ch->blksz);
	dst = (int32_t *)(ch->buf->buf + blk * ch->blksz);
	n = ch->blksz / MA_WORD;
	for (i = 0; i < n; i++)
		dst[i] = (int32_t)(src[i] << 8);
}

/* ADMAC period callback (its interrupt thread; may not sleep). */
static void
ma_period(void *arg)
{
	struct ma_chan *ch = arg;
	struct ma_softc *sc = ch->sc;
	bool running;

	mtx_lock(&sc->mtx);
	if (ch->dir == PCMDIR_PLAY) {
		/* The hardware plays period done + 1; fill the one after. */
		ch->done++;
		ma_play_period(ch, (ch->done + 1) % ch->nblk);
	} else {
		/* Period done is complete. */
		if (ch->running)
			ma_rec_period(ch, ch->done % ch->nblk);
		ch->done++;
	}
	running = ch->running;
	mtx_unlock(&sc->mtx);
	if (running)
		chn_intr(ch->pcm);
}

/* ---- power (taskqueue) ---- */

static int
ma_codec_setup(struct ma_softc *sc)
{
	int error;

	if (sc->codec_fmt)
		return (0);
	if ((error = cs42l84_set_fmt(sc->codec, CS42L84_FMT_DSP_A)) != 0 ||
	    (error = cs42l84_set_sysclk(sc->codec,
	    MA_RATE * MA_BCLK_RATIO)) != 0)
		return (error);
	sc->codec_fmt = true;
	return (0);
}

static int
ma_power_up(struct ma_softc *sc, struct ma_chan *ch)
{
	uint32_t i;
	int error;

	if ((error = apple_aop_audio_stream_open(sc->aop_audio, ch->stream,
	    &ch->aop)) != 0) {
		device_printf(sc->dev, "%s open: %d\n", ch->stream, error);
		return (error);
	}
	if ((error = apple_admac_config(ch->dma, MA_WORD, ch->channels)) != 0 ||
	    (error = ma_codec_setup(sc)) != 0 ||
	    (error = cs42l84_hw_params(sc->codec, ch->codec_dir, MA_RATE, 24,
	    MA_SLOT_WIDTH)) != 0) {
		device_printf(sc->dev, "%s setup: %d\n", ch->stream, error);
		goto close;
	}
	mtx_lock(&sc->mtx);
	ch->done = 0;
	memset(ch->dmabuf, 0, ch->blksz * ch->nblk);
	/* Playback prefill: two periods of PCM data, the rest silence. */
	if (ch->dir == PCMDIR_PLAY)
		for (i = 0; i < 2; i++)
			ma_play_period(ch, i);
	mtx_unlock(&sc->mtx);
	/* The firmware starts the serializer and its clocks, then the codec. */
	if ((error = apple_aop_audio_stream_power(ch->aop, true)) != 0) {
		device_printf(sc->dev, "%s pwrd: %d\n", ch->stream, error);
		goto close;
	}
	if ((error = apple_admac_start_cyclic(ch->dma, ch->dmaiova,
	    ch->blksz * ch->nblk, ch->blksz, ma_period, ch)) != 0) {
		device_printf(sc->dev, "%s DMA start: %d\n", ch->stream, error);
		goto unpower;
	}
	if ((error = cs42l84_set_power(sc->codec, ch->codec_dir, true)) != 0) {
		device_printf(sc->dev, "%s codec power: %d\n", ch->stream,
		    error);
		apple_admac_stop(ch->dma);
		goto unpower;
	}
	ch->powered = true;
	return (0);
unpower:
	apple_aop_audio_stream_power(ch->aop, false);
close:
	apple_aop_audio_stream_close(ch->aop);
	ch->aop = NULL;
	return (error);
}

static void
ma_power_down(struct ma_softc *sc, struct ma_chan *ch)
{

	/* Codec first (muted), then the data, then the clocks. */
	cs42l84_set_power(sc->codec, ch->codec_dir, false);
	apple_admac_stop(ch->dma);
	apple_aop_audio_stream_power(ch->aop, false);
	apple_aop_audio_stream_close(ch->aop);
	ch->aop = NULL;
	ch->powered = false;
}

static void
ma_power_chan(struct ma_softc *sc, struct ma_chan *ch)
{
	bool want;

	if (ch->dma == NULL)
		return;
	mtx_lock(&sc->mtx);
	want = ch->want;
	mtx_unlock(&sc->mtx);
	if (want && !ch->powered)
		ma_power_up(sc, ch);
	else if (!want && ch->powered)
		ma_power_down(sc, ch);
}

static void
ma_power_task(void *arg, int pending)
{
	struct ma_softc *sc = arg;

	ma_power_chan(sc, &sc->play);
	ma_power_chan(sc, &sc->rec);
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
	struct ma_chan *ch = dir == PCMDIR_PLAY ? &sc->play : &sc->rec;
	void *buf;

	buf = malloc(MA_BUFSZ, M_DEVBUF, M_WAITOK | M_ZERO);
	if (sndbuf_setup(b, buf, MA_BUFSZ) != 0) {
		free(buf, M_DEVBUF);
		return (NULL);
	}
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
	struct ma_chan *ch = data;

	return (format == ch->fmt[0] ? 0 : EINVAL);
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

	if (ch->powered)
		return (ch->blksz);	/* fixed while the DMA runs */
	/* A power of two dividing the buffer, three periods or more. */
	blksz = MAX(rounddown_pow_of_two(blksz), MA_BLK_MIN);
	blksz = rounddown_pow_of_two(MIN(blksz, MA_BUFSZ / MA_NBLK_MIN));
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
		ch->want = true;
		break;
	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		/* No more data from the next period on; the task powers down. */
		ch->running = false;
		ch->want = false;
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
	if (ch->dir == PCMDIR_PLAY)
		/* Converted up to the end of period done + 1. */
		ptr = ((ch->done + 2) % ch->nblk) * ch->blksz;
	else
		/* Captured up to the end of period done - 1. */
		ptr = (ch->done % ch->nblk) * ch->blksz;
	mtx_unlock(&ch->sc->mtx);
	return (ptr);
}

static struct pcmchan_caps *
ma_chan_getcaps(kobj_t obj, void *data)
{
	struct ma_chan *ch = data;

	return (&ch->caps);
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

/* A direction's DMA channel and buffer; false if it has none. */
static bool
ma_chan_setup(struct ma_softc *sc, struct ma_chan *ch, int dir)
{
	phandle_t xref;
	uint32_t cell;
	void *kva;
	int error;

	ch->sc = sc;
	ch->dir = dir;
	ch->stream = dir == PCMDIR_PLAY ? "cout" : "cin";
	ch->codec_dir = dir == PCMDIR_PLAY ? CS42L84_DIR_PLAYBACK :
	    CS42L84_DIR_CAPTURE;
	ch->channels = dir == PCMDIR_PLAY ? 2 : 1;
	ch->fmt[0] = SND_FORMAT(AFMT_S32_LE, ch->channels, 0);
	ch->fmt[1] = 0;
	ch->caps = (struct pcmchan_caps){ MA_RATE, MA_RATE, ch->fmt, 0 };
	if ((error = apple_aop_audio_dma(sc->aop_audio, ch->stream, &xref,
	    &cell)) != 0 ||
	    (error = apple_admac_get_channel(xref, cell, &ch->dma)) != 0) {
		device_printf(sc->dev, "%s DMA channel: %d\n", ch->stream,
		    error);
		return (false);
	}
	if ((error = apple_admac_alloc_buffer(ch->dma, MA_BUFSZ, &kva,
	    &ch->dmaiova)) != 0) {
		device_printf(sc->dev, "%s DMA buffer: %d\n", ch->stream,
		    error);
		apple_admac_release(ch->dma);
		ch->dma = NULL;
		return (false);
	}
	ch->dmabuf = kva;
	return (true);
}

static void
ma_init(void *arg)
{
	struct ma_softc *sc = arg;
	bool play, rec;

	if (sc->hook.ich_func != NULL)
		config_intrhook_disestablish(&sc->hook);
	if (ma_find_jack(sc) != 0)
		return;
	play = ma_chan_setup(sc, &sc->play, PCMDIR_PLAY);
	rec = ma_chan_setup(sc, &sc->rec, PCMDIR_REC);
	if (!play && !rec)
		return;
	cs42l84_get_volume(sc->codec, &sc->vol_l, &sc->vol_r);

	pcm_init(sc->dev, sc);
	if (play)
		pcm_addchan(sc->dev, PCMDIR_PLAY, &ma_chan_class, sc);
	if (rec)
		pcm_addchan(sc->dev, PCMDIR_REC, &ma_chan_class, sc);
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
	struct ma_chan *chans[] = { &sc->play, &sc->rec };
	int error, i;

	if ((error = pcm_unregister(dev)) != 0)
		return (error);
	mtx_lock(&sc->mtx);
	sc->play.want = sc->rec.want = false;
	mtx_unlock(&sc->mtx);
	taskqueue_enqueue(sc->tq, &sc->power_task);
	taskqueue_drain_all(sc->tq);
	taskqueue_free(sc->tq);
	for (i = 0; i < nitems(chans); i++) {
		if (chans[i]->dma == NULL)
			continue;
		apple_admac_free_buffer(chans[i]->dma, chans[i]->dmabuf);
		apple_admac_release(chans[i]->dma);
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
