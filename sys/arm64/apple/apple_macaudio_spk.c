/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Brandon Thompson
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * Apple M5 laptop speakers: the "Speaker" link of the macaudio card and its
 * speaker protection interlock.  Follows aurora-silicon's Linux
 * sound/soc/apple/macaudio.c (macaudio_j815_cfg: SN012776 amps, safe_vol
 * -20 dB, I2S NB_IF, the speakers on the secondary front-end) and the
 * "spkr" / "sense" streams of t8152_aop_audio.rs.
 *
 * Two sound(4) devices, children of the card's jack device (pcm0):
 *
 *   speaker (pcm1)  playback, 48 kHz S32_LE stereo, converted to the AOP
 *                   "spkr" stream (one 24-in-32 word per amp, amp k in TDM
 *                   slot k, left on the even slots and right on the odd ones,
 *                   as macaudio's left_mask / right_mask).
 *   sense (pcm2)    capture only, bit-perfect, 48 kHz S16_LE with two words
 *                   per amp (12 channels on J815): the amps' V/I sense, the
 *                   speaker stream's RX ("sense").  speakersafetyd's
 *                   visense_pcm.
 *
 * The interlock (macaudio's "Speaker Volume Unlock" and "Speaker
 * Protection" lease) is card state here, driven through sysctls under
 * dev.pcm.<speaker>.speaker and an owner device, /dev/spkctl<unit>: the
 * process that holds it open owns every speaker control, as speakersafetyd
 * holds SNDRV_CTL_IOCTL_ELEM_LOCK on each.  See
 * ~/m1n1-env/docs/speaker-interface.md.
 *
 * SAFETY INVARIANTS (never louder than macaudio in the same state):
 *  - Without an unlocked volume lock (owner + magic ping within 250 ms of
 *    playing time) every "Speaker Volume" is held at <= 1 (-100 dB), the
 *    data path plays silence and the amps stay in shutdown.
 *  - Unlocked, the volumes may rise to macaudio's 0 dB, but tas2764 caps
 *    them at -20 dB (TAS2764_VOL_CAP) and the amp gain at its lowest step
 *    whatever is asked; the daemon's own ceilings (-24 / -32 dB) sit below.
 *  - Under the lease the speakers play only while a grant runs; the gate is
 *    checked against the lease's end on every period.
 *  - A volume lock or floor that did not take on every amp refuses
 *    speaker playback until the module is reloaded.
 *  - No speakers at all unless every amp has its operating level from the
 *    boot loader (apple,speaker-config), as macaudio's admission.
 *
 * Locking: sp->lock (sx) is macaudio's spk_mutex and volume_lock_mutex in
 * one; it covers the interlock, the amps (I2C, sleeps) and the power task.
 * sp->mtx covers the data path.  Lock order: sp->lock, sp->mtx, then the
 * ADMAC's and tas2764's own.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/conf.h>
#include <sys/eventhandler.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/queue.h>
#include <sys/sx.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/time.h>

#include <machine/atomic.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#ifdef HAVE_KERNEL_OPTION_HEADERS
#include "opt_snd.h"
#endif
#include <dev/sound/pcm/sound.h>

#include <arm64/apple/apple_admac.h>
#include <arm64/apple/apple_aop_audio.h>
#include <arm64/apple/apple_macaudio_spk.h>
#include <arm64/apple/tas2764.h>

#define	MAS_RATE		48000
#define	MAS_WIDTH		24		/* S24_LE in 32-bit words */
#define	MAS_SLOT_WIDTH		32		/* MACAUDIO_SLOTWIDTH */
#define	MAS_BCLK_RATIO		256		/* macaudio_fe_links: Secondary */
#define	MAS_MAX_AMPS		8
#define	MAS_FRAMES		8192		/* buffer length, frames */
#define	MAS_BLK_MIN		128		/* period, frames */
#define	MAS_BLK_DEF		1024
#define	MAS_NBLK_MIN		4
#define	MAS_PLAY_FRAME		8		/* S32_LE stereo */

/* macaudio.c */
#define	SPEAKER_MAGIC_VALUE	((int)0xdec1be15)
#define	SPEAKER_LOCK_TIMEOUT	250		/* ms */
#define	SPEAKER_LEASE_UNOWNED_MS 30000
#define	SPEAKER_LEASE_MIN_MS	10
#define	SPEAKER_LEASE_MAX_MS	100
#define	SPEAKER_RAMP_MAX_US	10000
#define	SPEAKER_RAMP_STEP_US	500
#define	SN012776_0DB		TAS2764_VOL_0DB
#define	SN012776_DB(x)		TAS2764_VOL_DB(x)

/* The controls' dB scales (tas2764.c TLVs), in millibel */
#define	MAS_VOL_DB_MIN		(-10050)	/* at 0, which is mute */
#define	MAS_VOL_DB_STEP		50
#define	MAS_GAIN_DB_MIN		1100
#define	MAS_GAIN_DB_STEP	50

/* enum macaudio_spk_iface */
#define	MAS_IFACE_UNLOCK	0		/* "Speaker Volume Unlock" */
#define	MAS_IFACE_LEASE		1		/* "Speaker Protection" */

/* macaudio_spk_ramp_db[] */
static const int8_t mas_ramp_db[] = {
	-2, -4, -6, -9, -12, -16, -20, -26, -32, -40, -50,
};

struct ma_spk;
struct mas_chan;

/* A DMA start's period callback argument: stale callbacks are dropped. */
struct mas_tok {
	struct mas_chan		*ch;
	u_int			gen;
};

struct mas_chan {
	struct ma_spk		*sp;
	device_t		dev;		/* its pcm device */
	int			dir;
	const char		*stream;	/* "spkr" or "sense" */
	struct apple_aop_audio_info info;
	uint32_t		fmt[2];
	struct pcmchan_caps	caps;
	struct pcm_channel	*pcm;
	struct snd_dbuf		*buf;
	struct apple_admac_chan	*dma;
	uint8_t			*dmabuf;
	bus_addr_t		dmaiova;
	u_int			pcm_frame;	/* bytes */
	u_int			dma_frame;
	u_int			bufsz;		/* PCM buffer bytes */
	u_int			dmalen;		/* DMA buffer bytes */
	uint32_t		blksz;		/* PCM period bytes */
	uint32_t		dmablk;		/* DMA period bytes */
	uint32_t		nblk;
	/* sp->lock (the power task): */
	struct apple_aop_audio_stream *aop;
	bool			powered;
	/* sp->mtx: */
	bool			want;
	bool			running;	/* data to or from the PCM */
	bool			live;		/* the DMA buffer is in use */
	uint32_t		dma_done;	/* DMA periods since its start */
	uint32_t		pcm_done;	/* capture: PCM periods filled */
	u_int			gen;
	struct mas_tok		tok[2];
};

struct mas_amp {
	struct ma_spk		*sp;
	device_t		dev;
	int			index;		/* codec order = TDM slot */
	char			name[64];	/* sound-name-prefix */
	char			node[64];	/* its sysctl node */
	/* sp->lock: the controls' values */
	u_int			vol;		/* "Speaker Volume" */
	u_int			gain;		/* "Amp Gain Volume" */
	bool			vsense, isense;	/* "VSENSE/ISENSE Switch" */
	u_int			pm_vol;
};

struct ma_spk {
	device_t		card;
	device_t		aop_audio;
	device_t		spk_dev, sense_dev;
	SLIST_ENTRY(ma_spk)	link;
	struct mtx		mtx;
	struct sx		lock;
	struct taskqueue	*tq;
	struct task		power_task;
	struct task		lock_update_task;	/* lock_update_work */
	struct timeout_task	lock_timeout_task;	/* lock_timeout_work */
	struct task		expire_task;		/* spk_expire_work */
	struct timeout_task	unowned_task;		/* spk_unowned_work */
	struct callout		lease_callout;		/* spk_lease_timer */
	struct cdev		*cdev;
	eventhandler_tag	suspend_tag, resume_tag;

	int			namps;
	struct mas_amp		amps[MAS_MAX_AMPS];
	struct mas_chan		play, sense;

	/* sp->lock */
	bool			amps_on;
	bool			amps_broken;	/* until the next start */
	bool			owned;
	pid_t			owner_pid;
	u_int			vol_max;	/* the volumes' current limit */
	bool			pm_saved;
	bool			dead;

	/* "Speaker Volume Unlock" (sp->lock unless noted) */
	int			unlocked;	/* atomic reads */
	bool			was_locked;
	bool			timeout_enabled;
	sbintime_t		lock_remain;
	sbintime_t		lock_deadline;
	int			lock_failed;	/* atomic */
	bool			be_active;	/* sp->mtx */
	int			sample_rate;	/* sp->mtx */
	int			pm_quiesced;	/* atomic */

	/* "Speaker Protection" (sp->lock unless noted) */
	int			iface;		/* atomic reads */
	int			granted;	/* atomic */
	int			ramping;	/* atomic */
	uint64_t		lease_end;	/* atomic, sbintime */
	bool			spk_active;
	bool			lapsed;
	int			floor_failed;	/* atomic */
	u_int			refused;	/* atomic */
	u_int			refused_since_grant;
	sbintime_t		gate_closed_at;
	const char		*gate_reason;
};

static SLIST_HEAD(, ma_spk) mas_cards = SLIST_HEAD_INITIALIZER(mas_cards);
static struct mtx mas_cards_mtx;
MTX_SYSINIT(mas_cards, &mas_cards_mtx, "macaudio speakers", MTX_DEF);

static void	mas_amps_sync(struct ma_spk *sp);
static void	mas_spk_floor_(struct ma_spk *sp, const char *reason,
		    bool ramp);

/* Where the interlock logs: the speaker device, or the card before it. */
static device_t
mas_dev(struct ma_spk *sp)
{

	return (sp->spk_dev != NULL ? sp->spk_dev : sp->card);
}

/* ---- the gate ---- */

static bool
mas_leased(struct ma_spk *sp)
{

	return (atomic_load_int(&sp->iface) == MAS_IFACE_LEASE);
}

/*
 * macaudio_spk_admitted(): protection is granted and its lease has not run
 * out.  Any context.
 */
static bool
mas_spk_admitted(struct ma_spk *sp)
{

	if (atomic_load_acq_int(&sp->granted) == 0 ||
	    atomic_load_int(&sp->floor_failed) != 0)
		return (false);
	return (sbinuptime() < (sbintime_t)atomic_load_acq_64(&sp->lease_end));
}

/*
 * The speakers may sound: speaker data reaches the amps and the amps may
 * be up.  Under "Speaker Volume Unlock", while the volume lock is unlocked
 * (macaudio plays at the -100 dB floor while locked; this plays nothing);
 * under the lease, while admitted, or during a withdrawal's ramp (the
 * stream plays on under it, within the lease).  Any context.
 */
static bool
mas_sound_ok(struct ma_spk *sp)
{

	if (atomic_load_int(&sp->lock_failed) != 0 ||
	    atomic_load_int(&sp->floor_failed) != 0)
		return (false);
	if (!mas_leased(sp))
		return (atomic_load_int(&sp->unlocked) != 0);
	if (atomic_load_int(&sp->ramping) != 0)
		return (sbinuptime() <
		    (sbintime_t)atomic_load_acq_64(&sp->lease_end));
	return (mas_spk_admitted(sp));
}

/* The speaker DMA buffer to silence: macaudio_stop_speaker_playback(). */
static void
mas_play_silence(struct ma_spk *sp)
{
	struct mas_chan *ch = &sp->play;

	mtx_lock(&sp->mtx);
	if (ch->live)
		memset(ch->dmabuf, 0, ch->dmalen);
	mtx_unlock(&sp->mtx);
}

/* ---- amps (sp->lock) ---- */

static int
mas_amps_power(struct ma_spk *sp, bool on)
{
	int error, error2, i;

	error = 0;
	for (i = 0; i < sp->namps; i++) {
		error2 = tas2764_set_power(sp->amps[i].dev, on);
		if (error2 != 0 && error == 0)
			error = error2;
		if (on && error != 0)
			break;
	}
	/* One amp that did not come up takes the others down. */
	if (on && error != 0)
		for (i = 0; i < sp->namps; i++)
			(void)tas2764_set_power(sp->amps[i].dev, false);
	return (error);
}

/*
 * The amps up only while the speaker stream runs and the interlock lets
 * them sound (mas_sound_ok()), down otherwise: their volumes were set
 * before this goes down, and are set again before it comes up.
 */
static void
mas_amps_sync(struct ma_spk *sp)
{
	struct mas_amp *amp;
	bool want;
	int error, i;

	sx_assert(&sp->lock, SA_XLOCKED);
	want = sp->play.powered && !sp->amps_broken && !sp->dead &&
	    mas_sound_ok(sp);
	if (want == sp->amps_on)
		return;
	if (!want) {
		error = mas_amps_power(sp, false);
		if (error != 0)
			device_printf(mas_dev(sp), "amp power-off: %d\n",
			    error);
		sp->amps_on = false;
		return;
	}
	for (i = 0; i < sp->namps; i++) {
		amp = &sp->amps[i];
		(void)tas2764_set_volume(amp->dev, MIN(amp->vol, sp->vol_max));
		(void)tas2764_set_amp_gain(amp->dev, amp->gain);
		(void)tas2764_set_sense(amp->dev, amp->vsense, amp->isense);
	}
	if ((error = mas_amps_power(sp, true)) != 0) {
		device_printf(mas_dev(sp), "amp power-on failed (%d): "
		    "the speakers stay off until the next start\n", error);
		sp->amps_broken = true;
		return;
	}
	sp->amps_on = true;
}

/* ---- "Speaker Volume Unlock" (macaudio_vlimit_*) ---- */

/*
 * macaudio_vlimit_unlock(): every "Speaker Volume" limited to
 * macaudio_sn012776_vlimit() (the floor 1 while locked, 0 dB unlocked;
 * tas2764 caps that at -20 dB) and clipped to it, as snd_soc_limit_volume().
 */
static void
mas_vlimit_unlock(struct ma_spk *sp, bool unlock)
{
	struct mas_amp *amp;
	bool failed;
	u_int max;
	int error, i;

	sx_assert(&sp->lock, SA_XLOCKED);
	max = MIN(unlock ? SN012776_0DB : 1, TAS2764_VOL_CAP);
	sp->vol_max = max;
	failed = false;
	for (i = 0; i < sp->namps; i++) {
		amp = &sp->amps[i];
		if (amp->vol > max)
			amp->vol = max;
		if ((error = tas2764_set_volume(amp->dev, amp->vol)) != 0) {
			device_printf(mas_dev(sp),
			    "Failed to %slock volume %s: %d\n",
			    unlock ? "un" : "", amp->name, error);
			failed = true;
		}
	}
	/* macaudio_lock_fail_work() */
	if (failed && !unlock) {
		atomic_store_int(&sp->lock_failed, 1);
		device_printf(mas_dev(sp), "speaker volume lock failed: "
		    "stopping and refusing playback\n");
		mas_play_silence(sp);
		mas_amps_sync(sp);
	}
}

static void
mas_pm_volumes_drop(struct ma_spk *sp)
{

	sp->pm_saved = false;
}

/* macaudio_pm_volumes_save(): before the system-sleep lock clips them. */
static void
mas_pm_volumes_save(struct ma_spk *sp)
{
	int i;

	if (!atomic_load_int(&sp->unlocked))
		return;
	for (i = 0; i < sp->namps; i++)
		sp->amps[i].pm_vol = sp->amps[i].vol;
	sp->pm_saved = true;
}

/* macaudio_pm_volumes_restore(): at the unlock after a system sleep. */
static void
mas_pm_volumes_restore(struct ma_spk *sp)
{
	struct mas_amp *amp;
	int error, i;

	if (!sp->pm_saved)
		return;
	for (i = 0; i < sp->namps; i++) {
		amp = &sp->amps[i];
		amp->vol = MIN(amp->pm_vol, sp->vol_max);
		if ((error = tas2764_set_volume(amp->dev, amp->vol)) != 0)
			device_printf(mas_dev(sp),
			    "Failed to restore volume %s: %d\n", amp->name,
			    error);
	}
	mas_pm_volumes_drop(sp);
}

/* macaudio_vlimit_update() */
static void
mas_vlimit_update(struct ma_spk *sp)
{
	const char *reason;
	bool unlock;

	sx_assert(&sp->lock, SA_XLOCKED);

	/* "Speaker Protection" owns the speakers: the lock stays as it is */
	if (mas_leased(sp))
		return;

	unlock = true;
	reason = NULL;
	if (!sp->owned) {
		reason = "Main control not locked";
		unlock = false;
	}
	if (sp->lock_remain <= 0) {
		reason = "Lock timeout";
		unlock = false;
	}
	if (atomic_load_int(&sp->pm_quiesced)) {
		reason = "System suspend";
		unlock = false;
	}
	/*
	 * "Every limited control locked by the same owner": the owner device
	 * holds every speaker control at once (mas_owner_open()).
	 */

	if (unlock == (atomic_load_int(&sp->unlocked) != 0))
		return;
	if (unlock)
		device_printf(mas_dev(sp), "Speaker volumes unlocked\n");
	else {
		device_printf(mas_dev(sp), "Speaker volumes locked: %s\n",
		    reason);
		/* The system-sleep lock is the kernel's, not a lost lease */
		if (!atomic_load_int(&sp->pm_quiesced))
			sp->was_locked = true;
	}
	/* Volumes down before the data and the amps; up after them */
	mas_vlimit_unlock(sp, unlock);
	if (unlock)
		mas_pm_volumes_restore(sp);
	atomic_store_int(&sp->unlocked, unlock ? 1 : 0);
	mas_amps_sync(sp);
}

/* macaudio_vlimit_pm() */
static void
mas_vlimit_pm(struct ma_spk *sp, bool quiesce)
{

	sx_assert(&sp->lock, SA_XLOCKED);
	if (quiesce) {
		taskqueue_cancel_timeout(sp->tq, &sp->lock_timeout_task, NULL);
		sp->lock_remain = 0;
		mas_pm_volumes_save(sp);
	}
	mas_vlimit_update(sp);
}

/* macaudio_vlimit_enable_timeout(): the lease runs while the speakers play */
static void
mas_vlimit_enable_timeout(struct ma_spk *sp)
{

	sx_assert(&sp->lock, SA_XLOCKED);
	if (sp->timeout_enabled)
		return;
	if (sp->lock_remain > 0) {
		sp->lock_deadline = sbinuptime() + sp->lock_remain;
		taskqueue_enqueue_timeout_sbt(sp->tq, &sp->lock_timeout_task,
		    sp->lock_remain, 0, 0);
	}
	mas_vlimit_update(sp);
	sp->timeout_enabled = true;
}

/* macaudio_vlimit_disable_timeout(): and stops while they do not */
static void
mas_vlimit_disable_timeout(struct ma_spk *sp)
{
	sbintime_t now;

	sx_assert(&sp->lock, SA_XLOCKED);
	if (!sp->timeout_enabled)
		return;
	now = sbinuptime();
	taskqueue_cancel_timeout(sp->tq, &sp->lock_timeout_task, NULL);
	if (now > sp->lock_deadline)
		sp->lock_remain = 0;
	else if (sp->lock_remain > 0)
		sp->lock_remain = sp->lock_deadline - now;
	mas_vlimit_update(sp);
	sp->timeout_enabled = false;
}

/* macaudio_vlimit_timeout_work() */
static void
mas_lock_timeout_task(void *arg, int pending __unused)
{
	struct ma_spk *sp = arg;
	sbintime_t now;

	sx_xlock(&sp->lock);
	/* A ping or a disabled timeout may have raced this */
	if (!sp->timeout_enabled || !sp->owned || sp->lock_remain <= 0)
		goto out;
	now = sbinuptime();
	if (now < sp->lock_deadline) {
		taskqueue_enqueue_timeout_sbt(sp->tq, &sp->lock_timeout_task,
		    sp->lock_deadline - now, 0, 0);
		goto out;
	}
	sp->lock_remain = 0;
	mas_vlimit_update(sp);
out:
	sx_xunlock(&sp->lock);
}

/* macaudio_vlimit_update_work() */
static void
mas_lock_update_task(void *arg, int pending __unused)
{
	struct ma_spk *sp = arg;
	bool active;

	sx_xlock(&sp->lock);
	mtx_lock(&sp->mtx);
	active = sp->be_active;
	mtx_unlock(&sp->mtx);
	if (active)
		mas_vlimit_enable_timeout(sp);
	else
		mas_vlimit_disable_timeout(sp);
	sx_xunlock(&sp->lock);
}

/* macaudio_slk_put(): the ping */
static int
mas_slk_put(struct ma_spk *sp, int val, pid_t pid)
{
	int error;

	/* A saved state's 0 or 1 written back is a no-op */
	if (val == 0 || val == 1)
		return (0);
	if (val != SPEAKER_MAGIC_VALUE)
		return (EINVAL);

	error = 0;
	sx_xlock(&sp->lock);
	if (!sp->owned || pid != sp->owner_pid)
		error = EPERM;
	else if (mas_leased(sp))
		error = EBUSY;
	else if (sp->was_locked) {
		/* The lock was lost at some point: tell the owner once */
		sp->was_locked = false;
		error = ETIMEDOUT;
	} else if (atomic_load_int(&sp->pm_quiesced))
		error = 0;
	else {
		taskqueue_cancel_timeout(sp->tq, &sp->lock_timeout_task, NULL);
		sp->lock_remain = SPEAKER_LOCK_TIMEOUT * SBT_1MS;
		sp->lock_deadline = sbinuptime() + sp->lock_remain;
		mas_vlimit_update(sp);
		if (sp->timeout_enabled)
			taskqueue_enqueue_timeout_sbt(sp->tq,
			    &sp->lock_timeout_task, sp->lock_remain, 0, 0);
	}
	sx_xunlock(&sp->lock);
	return (error);
}

/* ---- "Speaker Protection" (macaudio_spk_*) ---- */

/* macaudio_spk_gain() for the SN012776 without spk_fixed_ceiling */
static u_int
mas_spk_gain(bool granted)
{

	return (granted ? SN012776_0DB : 0);
}

/* macaudio_spk_set_level(), under the volumes' limit and tas2764's cap */
static int
mas_spk_set_level(struct ma_spk *sp, u_int gain)
{
	struct mas_amp *amp;
	int error, error2, i;

	sx_assert(&sp->lock, SA_XLOCKED);
	error = 0;
	for (i = 0; i < sp->namps; i++) {
		amp = &sp->amps[i];
		amp->vol = MIN(gain, sp->vol_max);
		if ((error2 = tas2764_set_volume(amp->dev, amp->vol)) != 0) {
			device_printf(mas_dev(sp),
			    "Failed to set %s to %u: %d\n", amp->name,
			    amp->vol, error2);
			if (error == 0)
				error = error2;
		}
	}
	return (error);
}

/*
 * macaudio_spk_ramp_down(): from the operating gain towards the floor while
 * the stream still plays, so that its stop does not click.  Steps at or
 * above the operating gain (here the -20 dB cap, as macaudio skips them
 * under spk_fixed_ceiling) change nothing and are skipped.
 */
static void
mas_spk_ramp_down(struct ma_spk *sp, sbintime_t deadline)
{
	sbintime_t next, wait;
	u_int i, top;

	top = MIN(mas_spk_gain(true), sp->vol_max);
	next = sbinuptime();
	for (i = 0; i < nitems(mas_ramp_db); i++) {
		if (SN012776_DB(mas_ramp_db[i]) >= (int)top)
			continue;
		if (sbinuptime() >= deadline)
			return;
		/* A failure here is the floor's to catch */
		(void)mas_spk_set_level(sp, SN012776_DB(mas_ramp_db[i]));
		next += SPEAKER_RAMP_STEP_US * SBT_1US;
		wait = MIN(next, deadline) - sbinuptime();
		if (wait > 0)
			pause_sbt("spkramp", wait, 50 * SBT_1US, 0);
	}
}

/*
 * __macaudio_spk_floor(): no new speaker playback, the running one silenced
 * and every amp at the safe level, then down.  A safe level that did not
 * take on every amp refuses speaker playback until the module is reloaded.
 */
static void
mas_spk_floor_(struct ma_spk *sp, const char *reason, bool ramp)
{
	bool was, closed_ahead;
	sbintime_t deadline;

	sx_assert(&sp->lock, SA_XLOCKED);
	was = sp->spk_active;
	/* The lease timer or a system sleep closed the gate before this */
	closed_ahead = was && atomic_load_int(&sp->granted) == 0;
	ramp = ramp && was && !closed_ahead;

	if (ramp)
		atomic_store_int(&sp->ramping, 1);
	atomic_store_rel_int(&sp->granted, 0);
	sp->spk_active = false;
	if (sp->dead) {
		atomic_store_int(&sp->ramping, 0);
		return;
	}
	callout_stop(&sp->lease_callout);

	/* While the lease still runs, the amps ramp down under the stream */
	if (ramp && sp->amps_on) {
		deadline = MIN((sbintime_t)atomic_load_acq_64(&sp->lease_end),
		    sbinuptime() + SPEAKER_RAMP_MAX_US * SBT_1US);
		mas_spk_ramp_down(sp, deadline);
	}
	atomic_store_int(&sp->ramping, 0);
	/* Stop what already runs (macaudio_spk_stop_work) */
	mas_play_silence(sp);

	if (mas_spk_set_level(sp, mas_spk_gain(false)) != 0) {
		atomic_store_int(&sp->floor_failed, 1);
		device_printf(mas_dev(sp), "speaker safe level not set on "
		    "every amp: speaker playback refused\n");
	}
	/* No tweeter high-pass to restore: J815 has no tweeter_hpf_granted */
	mas_amps_sync(sp);

	if (was) {
		if (closed_ahead)
			device_printf(mas_dev(sp), "Speaker protection "
			    "withdrawn: %s (start gate closed by %s %jd us "
			    "before)\n", reason,
			    sp->gate_reason != NULL ? sp->gate_reason : "?",
			    (intmax_t)((sbinuptime() - sp->gate_closed_at) /
			    SBT_1US));
		else
			device_printf(mas_dev(sp),
			    "Speaker protection withdrawn: %s\n", reason);
	}
}

/* macaudio_spk_floor(): at once (lease expired, sleep, failure) */
static void
mas_spk_floor(struct ma_spk *sp, const char *reason)
{

	mas_spk_floor_(sp, reason, false);
}

/* macaudio_spk_floor_ramped(): from a lease that still runs */
static void
mas_spk_floor_ramped(struct ma_spk *sp, const char *reason)
{

	mas_spk_floor_(sp, reason, true);
}

/*
 * macaudio_spk_close_gate(): ahead of the floor, from the lease timer and
 * system sleep.  No sleep; takes only sp->mtx.
 */
static void
mas_spk_close_gate(struct ma_spk *sp, const char *why)
{

	if (atomic_load_int(&sp->granted) != 0) {
		sp->gate_closed_at = sbinuptime();
		sp->gate_reason = why;
	}
	atomic_store_rel_int(&sp->granted, 0);
	mas_play_silence(sp);
}

/* macaudio_spk_lease_timer(): the lease's end */
static void
mas_lease_timer(void *arg)
{
	struct ma_spk *sp = arg;
	sbintime_t end, now;

	end = (sbintime_t)atomic_load_acq_64(&sp->lease_end);
	now = sbinuptime();
	/* Renewed while the timer was already firing */
	if (now < end) {
		callout_reset_sbt(&sp->lease_callout, end - now, 0,
		    mas_lease_timer, sp, 0);
		return;
	}
	if (atomic_load_int(&sp->granted) == 0)
		return;
	mas_spk_close_gate(sp, "the lease's end");
	taskqueue_enqueue(sp->tq, &sp->expire_task);
}

/* macaudio_spk_grant(): for ms from now */
static int
mas_spk_grant(struct ma_spk *sp, int ms)
{
	u_int refused;
	int error;

	sx_assert(&sp->lock, SA_XLOCKED);
	if (atomic_load_int(&sp->floor_failed))
		return (EIO);
	/* Only a powered card is granted */
	if (atomic_load_int(&sp->pm_quiesced))
		return (EBUSY);
	if ((error = mas_spk_set_level(sp, mas_spk_gain(true))) != 0) {
		mas_spk_floor(sp, "operating gain not set");
		return (error);
	}
	/* A system sleep began while the amps went up */
	if (atomic_load_int(&sp->pm_quiesced)) {
		mas_spk_floor(sp, "system sleep");
		return (EBUSY);
	}
	sp->spk_active = true;
	atomic_store_rel_64(&sp->lease_end, sbinuptime() + ms * SBT_1MS);
	atomic_store_rel_int(&sp->granted, 1);
	callout_reset_sbt(&sp->lease_callout, ms * SBT_1MS, 0,
	    mas_lease_timer, sp, 0);
	refused = atomic_readandclear_int(&sp->refused_since_grant);
	if (refused != 0)
		device_printf(mas_dev(sp), "Speaker protection granted "
		    "(%u speaker starts refused before)\n", refused);
	else
		device_printf(mas_dev(sp), "Speaker protection granted\n");
	mas_amps_sync(sp);
	return (0);
}

/* macaudio_spk_expire_work() */
static void
mas_expire_task(void *arg, int pending __unused)
{
	struct ma_spk *sp = arg;

	sx_xlock(&sp->lock);
	/* A release, a new owner or a late write may have dealt with it */
	if (sp->spk_active && atomic_load_int(&sp->granted) == 0) {
		sp->lapsed = true;
		mas_spk_floor(sp, "lease expired");
	}
	sx_xunlock(&sp->lock);
}

/* macaudio_spk_fix_amp_gains(): every amp's own gain at its limit */
static int
mas_spk_fix_amp_gains(struct ma_spk *sp)
{
	struct mas_amp *amp;
	int error, i;

	for (i = 0; i < sp->namps; i++) {
		amp = &sp->amps[i];
		amp->gain = tas2764_get_amp_gain_max(amp->dev);
		if ((error = tas2764_set_amp_gain(amp->dev, amp->gain)) != 0) {
			device_printf(mas_dev(sp), "Failed to fix %s amp gain "
			    "at its limit: %d\n", amp->name, error);
			return (error);
		}
	}
	return (0);
}

/* macaudio_spk_enter_lease() */
static int
mas_spk_enter_lease(struct ma_spk *sp)
{
	int error;

	sx_assert(&sp->lock, SA_XLOCKED);
	if (sp->dead)
		return (ENODEV);
	if (atomic_load_int(&sp->pm_quiesced))
		return (EBUSY);
	if (atomic_load_int(&sp->floor_failed) ||
	    atomic_load_int(&sp->lock_failed))
		return (EIO);

	/* The volume lock's ceiling first */
	taskqueue_cancel_timeout(sp->tq, &sp->lock_timeout_task, NULL);
	sp->lock_remain = 0;
	mas_pm_volumes_drop(sp);
	mas_vlimit_update(sp);
	if (atomic_load_int(&sp->lock_failed))
		return (EIO);
	/* No speaker start from here on: nothing is granted yet */
	atomic_store_int(&sp->iface, MAS_IFACE_LEASE);
	mas_play_silence(sp);

	/* The lease's floor; a failed one refuses speaker playback for good */
	mas_spk_floor(sp, "interface switched");
	if (atomic_load_int(&sp->floor_failed))
		error = EIO;
	else
		error = mas_spk_fix_amp_gains(sp);
	if (error != 0) {
		atomic_store_int(&sp->floor_failed, 1);
		device_printf(mas_dev(sp), "speaker floor for the lease not "
		    "set: speaker playback refused\n");
		mas_amps_sync(sp);
		return (error);
	}
	/* The volumes stay at the safe level: the ceiling no longer applies */
	mas_vlimit_unlock(sp, true);
	if (!sp->owned)
		taskqueue_enqueue_timeout_sbt(sp->tq, &sp->unowned_task,
		    SPEAKER_LEASE_UNOWNED_MS * SBT_1MS, 0, 0);
	device_printf(mas_dev(sp), "Speaker protection interface: Speaker "
	    "Protection (lease); Speaker Volume Unlock refused\n");
	return (0);
}

/* macaudio_spk_leave_lease(): back to "Speaker Volume Unlock", locked */
static void
mas_spk_leave_lease(struct ma_spk *sp, const char *why)
{

	sx_assert(&sp->lock, SA_XLOCKED);
	if (!mas_leased(sp))
		return;
	taskqueue_cancel_timeout(sp->tq, &sp->unowned_task, NULL);

	mas_spk_floor_ramped(sp, why);
	/* The lock's ceiling over the safe level; a fresh ping lifts it */
	if (!sp->dead)
		mas_vlimit_unlock(sp, false);
	atomic_store_int(&sp->unlocked, 0);
	taskqueue_cancel_timeout(sp->tq, &sp->lock_timeout_task, NULL);
	sp->lock_remain = 0;
	sp->lock_deadline = 0;
	mas_pm_volumes_drop(sp);
	/* A daemon still holding the lock learns that it lost it */
	sp->was_locked = true;
	if (atomic_load_int(&sp->floor_failed) &&
	    !atomic_load_int(&sp->lock_failed))
		atomic_store_int(&sp->lock_failed, 1);
	atomic_store_int(&sp->iface, MAS_IFACE_UNLOCK);
	mas_vlimit_update(sp);
	mas_amps_sync(sp);
	device_printf(mas_dev(sp), "Speaker protection interface: Speaker "
	    "Volume Unlock (%s)\n", why);
}

/* macaudio_spk_unowned_work() */
static void
mas_unowned_task(void *arg, int pending __unused)
{
	struct ma_spk *sp = arg;

	sx_xlock(&sp->lock);
	if (!sp->owned && !sp->dead)
		mas_spk_leave_lease(sp, "no owner of Speaker Protection");
	sx_xunlock(&sp->lock);
}

/* macaudio_spk_iface_put() (the caller is root: a sysctl write) */
static int
mas_spk_iface_put(struct ma_spk *sp, int item)
{
	int error;

	if (item != MAS_IFACE_UNLOCK && item != MAS_IFACE_LEASE)
		return (EINVAL);
	error = 0;
	sx_xlock(&sp->lock);
	if (item == atomic_load_int(&sp->iface))
		error = 0;
	else if (item == MAS_IFACE_LEASE)
		error = mas_spk_enter_lease(sp);
	else
		mas_spk_leave_lease(sp, "switched by the administrator");
	sx_xunlock(&sp->lock);
	return (error);
}

/* macaudio_spk_put(): grant, renew or release */
static int
mas_spk_put(struct ma_spk *sp, int ms, pid_t pid)
{
	int error;

	if (ms != 0 && (ms < SPEAKER_LEASE_MIN_MS || ms > SPEAKER_LEASE_MAX_MS))
		return (EINVAL);
	error = 0;
	sx_xlock(&sp->lock);
	if (!sp->owned || pid != sp->owner_pid || sp->dead) {
		error = EPERM;
		goto out;
	}
	if (!mas_leased(sp)) {
		error = EBUSY;
		goto out;
	}
	if (ms == 0) {
		sp->lapsed = false;
		mas_spk_floor_ramped(sp, "released by the owner");
		goto out;
	}
	if (atomic_load_int(&sp->pm_quiesced)) {
		error = EBUSY;
		goto out;
	}
	/* The owner stalled past its lease: tell it once, grant nothing */
	if (sp->lapsed) {
		sp->lapsed = false;
		error = ETIMEDOUT;
		goto out;
	}
	if (!sp->spk_active) {
		error = mas_spk_grant(sp, ms);
		goto out;
	}
	/* A renewal; a lease that has run out is not renewed */
	if (!mas_spk_admitted(sp)) {
		mas_spk_floor(sp, "lease expired");
		error = ETIMEDOUT;
		goto out;
	}
	atomic_store_rel_64(&sp->lease_end, sbinuptime() + ms * SBT_1MS);
	callout_reset_sbt(&sp->lease_callout, ms * SBT_1MS, 0,
	    mas_lease_timer, sp, 0);
	/* The old lease ran out between the check and the new end */
	if (atomic_load_int(&sp->granted) == 0) {
		mas_spk_floor(sp, "lease expired");
		error = ETIMEDOUT;
	}
out:
	sx_xunlock(&sp->lock);
	return (error);
}

/* ---- the owner device (ELEM_LOCK on every speaker control) ---- */

static void
mas_owner_dtor(void *arg)
{
	struct ma_spk *sp = arg;

	sx_xlock(&sp->lock);
	sp->owned = false;
	sp->owner_pid = 0;
	/* macaudio_spk_unlock() */
	sp->lapsed = false;
	if (mas_leased(sp)) {
		mas_spk_floor_ramped(sp, "owner gone");
		/* A restarting engine takes it back in time */
		if (!sp->dead)
			taskqueue_enqueue_timeout_sbt(sp->tq,
			    &sp->unowned_task,
			    SPEAKER_LEASE_UNOWNED_MS * SBT_1MS, 0, 0);
	}
	/* macaudio_slk_unlock() */
	taskqueue_cancel_timeout(sp->tq, &sp->lock_timeout_task, NULL);
	sp->lock_remain = 0;
	sp->lock_deadline = 0;
	mas_vlimit_update(sp);
	sx_xunlock(&sp->lock);
}

static int
mas_owner_open(struct cdev *cdev, int oflags __unused, int devtype __unused,
    struct thread *td)
{
	struct ma_spk *sp = cdev->si_drv1;
	int error;

	sx_xlock(&sp->lock);
	if (sp->dead)
		error = ENXIO;
	else if (sp->owned)
		error = EBUSY;		/* locked by another file */
	else
		error = devfs_set_cdevpriv(sp, mas_owner_dtor);
	if (error != 0) {
		sx_xunlock(&sp->lock);
		return (error);
	}
	sp->owned = true;
	sp->owner_pid = td->td_proc->p_pid;
	/*
	 * macaudio_slk_lock(): a lease belongs to the owner that pinged for
	 * it; the new owner sets the volumes itself.
	 */
	taskqueue_cancel_timeout(sp->tq, &sp->lock_timeout_task, NULL);
	sp->lock_remain = 0;
	sp->lock_deadline = 0;
	mas_pm_volumes_drop(sp);
	mas_vlimit_update(sp);
	sp->was_locked = false;
	/* macaudio_spk_lock(): a new owner starts without protection */
	sp->lapsed = false;
	if (mas_leased(sp)) {
		taskqueue_cancel_timeout(sp->tq, &sp->unowned_task, NULL);
		mas_spk_floor_ramped(sp, "new owner");
	}
	sx_xunlock(&sp->lock);
	return (0);
}

static struct cdevsw mas_owner_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	mas_owner_open,
	.d_name =	"spkctl",
};

/* ---- system sleep (macaudio_pm_notify()) ---- */

static void
mas_suspend(void *arg)
{
	struct ma_spk *sp = arg;

	/* Before anything that can wait: no grant, no sound */
	atomic_store_int(&sp->pm_quiesced, 1);
	mas_spk_close_gate(sp, "system sleep");
	sx_xlock(&sp->lock);
	if (mas_leased(sp))
		mas_spk_floor(sp, "system sleep");
	mas_vlimit_pm(sp, true);
	sx_xunlock(&sp->lock);
}

static void
mas_resume(void *arg)
{
	struct ma_spk *sp = arg;

	sx_xlock(&sp->lock);
	atomic_store_int(&sp->pm_quiesced, 0);
	if (sp->owned)
		sp->lapsed = true;
	mas_vlimit_pm(sp, false);
	sx_xunlock(&sp->lock);
}

/* ---- data path ---- */

/* Speaker period blk: PCM S32 stereo to one 24-in-32 word per amp slot. */
static void
mas_play_fill(struct ma_spk *sp, struct mas_chan *ch, uint32_t blk)
{
	const int32_t *src;
	uint32_t *dst, l, r;
	u_int f, k, nf;

	dst = (uint32_t *)(ch->dmabuf + blk * ch->dmablk);
	if (!ch->running || !mas_sound_ok(sp)) {
		memset(dst, 0, ch->dmablk);
		return;
	}
	src = (const int32_t *)(ch->buf->buf + blk * ch->blksz);
	nf = ch->blksz / ch->pcm_frame;
	for (f = 0; f < nf; f++) {
		l = (uint32_t)(src[2 * f] >> 8);
		r = (uint32_t)(src[2 * f + 1] >> 8);
		/* macaudio left_mask (even slots), right_mask (odd) */
		for (k = 0; k < (u_int)sp->namps; k++)
			*dst++ = (k & 1) != 0 ? r : l;
	}
}

/* ADMAC period callback (its interrupt thread; may not sleep). */
static void
mas_period(void *arg)
{
	struct mas_tok *tok = arg;
	struct mas_chan *ch = tok->ch;
	struct ma_spk *sp = ch->sp;
	bool running;

	mtx_lock(&sp->mtx);
	if (tok->gen != ch->gen || !ch->live) {
		mtx_unlock(&sp->mtx);
		return;
	}
	if (ch->dir == PCMDIR_PLAY) {
		/* The hardware plays period done + 1; fill the one after. */
		ch->dma_done++;
		mas_play_fill(sp, ch, (ch->dma_done + 1) % ch->nblk);
	} else {
		/* Period dma_done is complete. */
		if (ch->running) {
			memcpy(ch->buf->buf + (ch->pcm_done % ch->nblk) *
			    ch->blksz, ch->dmabuf + (ch->dma_done % ch->nblk) *
			    ch->dmablk, ch->blksz);
			ch->pcm_done++;
		}
		ch->dma_done++;
	}
	running = ch->running;
	mtx_unlock(&sp->mtx);
	if (running)
		chn_intr(ch->pcm);
}

/* A fresh token: callbacks of the DMA's previous start are dropped. */
static struct mas_tok *
mas_newtok(struct mas_chan *ch)
{
	struct mas_tok *tok;

	mtx_assert(&ch->sp->mtx, MA_OWNED);
	ch->gen++;
	tok = &ch->tok[ch->gen & 1];
	tok->ch = ch;
	tok->gen = ch->gen;
	return (tok);
}

/*
 * aurora's realign_sense(): before the speaker's serializer starts, a
 * running sense capture restarts at a frame boundary, since the firmware
 * stops the serializer anywhere in a frame.  Nothing flows while it is
 * stopped; the period in progress is dropped (no ADMAC pause here, so the
 * DMA restarts at the buffer's start and the PCM side carries on).
 */
static void
mas_sense_realign(struct ma_spk *sp)
{
	struct mas_chan *ch = &sp->sense;
	struct mas_tok *tok;
	int error;

	if (!ch->powered || !ch->info.realign)
		return;
	mtx_lock(&sp->mtx);
	apple_admac_stop(ch->dma);
	tok = mas_newtok(ch);
	ch->dma_done = 0;
	error = apple_admac_start_cyclic(ch->dma, ch->dmaiova, ch->dmalen,
	    ch->dmablk, mas_period, tok);
	if (error != 0)
		ch->live = false;
	mtx_unlock(&sp->mtx);
	if (error != 0)
		device_printf(ch->dev, "sense realign: DMA restart %d\n",
		    error);
}

/* ---- power (the taskqueue, sp->lock) ---- */

static int
mas_power_up(struct ma_spk *sp, struct mas_chan *ch)
{
	struct mas_tok *tok;
	uint32_t i;
	int error;

	if ((error = apple_aop_audio_stream_open(sp->aop_audio, ch->stream,
	    &ch->aop)) != 0) {
		device_printf(ch->dev, "%s open: %d\n", ch->stream, error);
		return (error);
	}
	if ((error = apple_admac_config(ch->dma, ch->info.word_bytes,
	    ch->info.port_window)) != 0) {
		device_printf(ch->dev, "%s DMA config: %d\n", ch->stream,
		    error);
		goto close;
	}
	mtx_lock(&sp->mtx);
	tok = mas_newtok(ch);
	ch->dma_done = 0;
	memset(ch->dmabuf, 0, ch->dmalen);
	ch->live = true;
	/* Playback prefill: two periods, data only if the gate allows */
	if (ch->dir == PCMDIR_PLAY)
		for (i = 0; i < 2; i++)
			mas_play_fill(sp, ch, i);
	mtx_unlock(&sp->mtx);

	if (ch->dir == PCMDIR_PLAY) {
		mas_sense_realign(sp);
		/* The firmware starts the serializer and its clocks */
		if ((error = apple_aop_audio_stream_power(ch->aop, true)) !=
		    0) {
			device_printf(ch->dev, "%s pwrd: %d\n", ch->stream,
			    error);
			goto unlive;
		}
	}
	if ((error = apple_admac_start_cyclic(ch->dma, ch->dmaiova,
	    ch->dmalen, ch->dmablk, mas_period, tok)) != 0) {
		device_printf(ch->dev, "%s DMA start: %d\n", ch->stream, error);
		goto unpower;
	}
	ch->powered = true;
	/* The clocks run: now the amps, if the interlock lets them */
	if (ch->dir == PCMDIR_PLAY)
		mas_amps_sync(sp);
	return (0);
unpower:
	(void)apple_aop_audio_stream_power(ch->aop, false);
unlive:
	mtx_lock(&sp->mtx);
	ch->live = false;
	mtx_unlock(&sp->mtx);
close:
	apple_aop_audio_stream_close(ch->aop);
	ch->aop = NULL;
	return (error);
}

static void
mas_power_down(struct ma_spk *sp, struct mas_chan *ch)
{

	/* The amps first (muted, shut down), then the data, then the clocks */
	ch->powered = false;
	if (ch->dir == PCMDIR_PLAY) {
		mas_amps_sync(sp);
		sp->amps_broken = false;
	}
	apple_admac_stop(ch->dma);
	mtx_lock(&sp->mtx);
	ch->live = false;
	ch->gen++;
	mtx_unlock(&sp->mtx);
	(void)apple_aop_audio_stream_power(ch->aop, false);
	apple_aop_audio_stream_close(ch->aop);
	ch->aop = NULL;
}

static void
mas_power_chan(struct ma_spk *sp, struct mas_chan *ch)
{
	bool want;

	if (ch->dma == NULL)
		return;
	mtx_lock(&sp->mtx);
	want = ch->want && !sp->dead;
	mtx_unlock(&sp->mtx);
	if (want && !ch->powered)
		(void)mas_power_up(sp, ch);
	else if (!want && ch->powered)
		mas_power_down(sp, ch);
}

static void
mas_power_task(void *arg, int pending __unused)
{
	struct ma_spk *sp = arg;

	sx_xlock(&sp->lock);
	/* The sense first: the speaker's start realigns it */
	mas_power_chan(sp, &sp->sense);
	mas_power_chan(sp, &sp->play);
	sx_xunlock(&sp->lock);
}

/* ---- sound(4) channel ---- */

static void *
mas_chan_init(kobj_t obj, void *devinfo, struct snd_dbuf *b,
    struct pcm_channel *c, int dir)
{
	struct mas_chan *ch = devinfo;
	void *buf;

	buf = malloc(ch->bufsz, M_DEVBUF, M_WAITOK | M_ZERO);
	if (sndbuf_setup(b, buf, ch->bufsz) != 0) {
		free(buf, M_DEVBUF);
		return (NULL);
	}
	ch->pcm = c;
	ch->buf = b;
	ch->nblk = MAS_FRAMES / MAS_BLK_DEF;
	ch->blksz = MAS_BLK_DEF * ch->pcm_frame;
	ch->dmablk = MAS_BLK_DEF * ch->dma_frame;
	sndbuf_resize(b, ch->nblk, ch->blksz);
	return (ch);
}

static int
mas_chan_free(kobj_t obj, void *data)
{
	struct mas_chan *ch = data;
	struct ma_spk *sp = ch->sp;

	/* macaudio_be_hw_free(): the speakers are closed */
	if (ch->dir == PCMDIR_PLAY) {
		mtx_lock(&sp->mtx);
		sp->sample_rate = 0;
		mtx_unlock(&sp->mtx);
	}
	free(ch->buf->buf, M_DEVBUF);
	return (0);
}

static int
mas_chan_setformat(kobj_t obj, void *data, uint32_t format)
{
	struct mas_chan *ch = data;

	return (format == ch->fmt[0] ? 0 : EINVAL);
}

static uint32_t
mas_chan_setspeed(kobj_t obj, void *data, uint32_t speed)
{

	return (MAS_RATE);
}

static uint32_t
mas_chan_setblocksize(kobj_t obj, void *data, uint32_t blksz)
{
	struct mas_chan *ch = data;
	u_int frames;

	if (ch->powered)
		return (ch->blksz);	/* fixed while the DMA runs */
	/* A power of two of frames, MAS_NBLK_MIN periods or more */
	frames = MAX(blksz / ch->pcm_frame, MAS_BLK_MIN);
	frames = rounddown_pow_of_two(MIN(frames, MAS_FRAMES / MAS_NBLK_MIN));
	ch->nblk = MAS_FRAMES / frames;
	ch->blksz = frames * ch->pcm_frame;
	ch->dmablk = frames * ch->dma_frame;
	sndbuf_resize(ch->buf, ch->nblk, ch->blksz);
	return (ch->blksz);
}

static int
mas_chan_trigger(kobj_t obj, void *data, int go)
{
	struct mas_chan *ch = data;
	struct ma_spk *sp = ch->sp;

	if (!PCMTRIG_COMMON(go))
		return (0);
	mtx_lock(&sp->mtx);
	switch (go) {
	case PCMTRIG_START:
		ch->running = true;
		ch->want = true;
		if (ch->dir == PCMDIR_PLAY) {
			sp->be_active = true;
			/* macaudio_dpcm_hw_params(): "Speaker Sample Rate" */
			sp->sample_rate = MAS_RATE;
			/* macaudio_spk_refuse(): counted, never logged */
			if (mas_leased(sp) && !mas_spk_admitted(sp)) {
				atomic_add_int(&sp->refused, 1);
				atomic_add_int(&sp->refused_since_grant, 1);
			}
		} else
			ch->pcm_done = 0;
		break;
	case PCMTRIG_STOP:
	case PCMTRIG_ABORT:
		ch->running = false;
		ch->want = false;
		if (ch->dir == PCMDIR_PLAY)
			sp->be_active = false;
		break;
	}
	mtx_unlock(&sp->mtx);
	taskqueue_enqueue(sp->tq, &sp->power_task);
	/* The volume lock's lease runs only while the speakers play */
	if (ch->dir == PCMDIR_PLAY)
		taskqueue_enqueue(sp->tq, &sp->lock_update_task);
	return (0);
}

static uint32_t
mas_chan_getptr(kobj_t obj, void *data)
{
	struct mas_chan *ch = data;
	uint32_t ptr;

	mtx_lock(&ch->sp->mtx);
	if (ch->dir == PCMDIR_PLAY)
		/* Converted up to the end of period done + 1 */
		ptr = ((ch->dma_done + 2) % ch->nblk) * ch->blksz;
	else
		ptr = (ch->pcm_done % ch->nblk) * ch->blksz;
	mtx_unlock(&ch->sp->mtx);
	return (ptr);
}

static struct pcmchan_caps *
mas_chan_getcaps(kobj_t obj, void *data)
{
	struct mas_chan *ch = data;

	return (&ch->caps);
}

static kobj_method_t mas_chan_methods[] = {
	KOBJMETHOD(channel_init,	mas_chan_init),
	KOBJMETHOD(channel_free,	mas_chan_free),
	KOBJMETHOD(channel_setformat,	mas_chan_setformat),
	KOBJMETHOD(channel_setspeed,	mas_chan_setspeed),
	KOBJMETHOD(channel_setblocksize, mas_chan_setblocksize),
	KOBJMETHOD(channel_trigger,	mas_chan_trigger),
	KOBJMETHOD(channel_getptr,	mas_chan_getptr),
	KOBJMETHOD(channel_getcaps,	mas_chan_getcaps),
	KOBJMETHOD_END
};
CHANNEL_DECLARE(mas_chan);

/* ---- sysctls ---- */

enum {
	MAS_AMP_VOLUME,
	MAS_AMP_VOLUME_MAX,
	MAS_AMP_GAIN,
	MAS_AMP_GAIN_MAX,
	MAS_AMP_VSENSE,
	MAS_AMP_ISENSE,
};

/* An amp's control: "<prefix> Speaker Volume" and the others. */
static int
mas_amp_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct mas_amp *amp = arg1;
	struct ma_spk *sp = amp->sp;
	int error, val;

	sx_xlock(&sp->lock);
	switch (arg2) {
	case MAS_AMP_VOLUME:
		val = amp->vol;
		break;
	case MAS_AMP_VOLUME_MAX:
		val = sp->vol_max;
		break;
	case MAS_AMP_GAIN:
		val = amp->gain;
		break;
	case MAS_AMP_GAIN_MAX:
		val = tas2764_get_amp_gain_max(amp->dev);
		break;
	case MAS_AMP_VSENSE:
		val = amp->vsense;
		break;
	default:
		val = amp->isense;
		break;
	}
	sx_xunlock(&sp->lock);
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);

	sx_xlock(&sp->lock);
	if (sp->dead)
		error = ENXIO;
	else if (sp->owned && req->td->td_proc->p_pid != sp->owner_pid)
		error = EPERM;		/* ELEM_LOCK held by another file */
	else
		switch (arg2) {
		case MAS_AMP_VOLUME:
			/* macaudio_spk_owned_write_check() */
			if (mas_leased(sp))
				error = EPERM;
			else if (val < 0 || (u_int)val > sp->vol_max)
				error = EINVAL;
			else {
				amp->vol = val;
				error = tas2764_set_volume(amp->dev, amp->vol);
			}
			break;
		case MAS_AMP_GAIN:
			if (mas_leased(sp))
				error = EPERM;
			else if (val < 0 ||
			    (u_int)val > tas2764_get_amp_gain_max(amp->dev))
				error = EINVAL;
			else {
				amp->gain = val;
				error = tas2764_set_amp_gain(amp->dev,
				    amp->gain);
			}
			break;
		case MAS_AMP_VSENSE:
		case MAS_AMP_ISENSE:
			/* macaudio_sense_write_check(): the owner, if any */
			if (val != 0 && val != 1) {
				error = EINVAL;
				break;
			}
			if (arg2 == MAS_AMP_VSENSE)
				amp->vsense = val != 0;
			else
				amp->isense = val != 0;
			error = tas2764_set_sense(amp->dev, amp->vsense,
			    amp->isense);
			break;
		default:
			error = EPERM;
			break;
		}
	sx_xunlock(&sp->lock);
	return (error);
}

enum {
	MAS_CTL_UNLOCK,
	MAS_CTL_LEASE,
	MAS_CTL_IFACE,
	MAS_CTL_SAMPLE_RATE,
	MAS_CTL_OWNER,
	MAS_CTL_PLAYING,
	MAS_CTL_AMPS_ON,
	MAS_CTL_FAILED,
};

static int
mas_ctl_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct ma_spk *sp = arg1;
	int error, val;

	switch (arg2) {
	case MAS_CTL_UNLOCK:
		/* macaudio_slk_get() */
		val = atomic_load_int(&sp->unlocked);
		break;
	case MAS_CTL_LEASE:
		/* macaudio_spk_get() */
		val = mas_spk_admitted(sp) ? 1 : 0;
		break;
	case MAS_CTL_IFACE:
		val = atomic_load_int(&sp->iface);
		break;
	case MAS_CTL_SAMPLE_RATE:
	case MAS_CTL_PLAYING:
		mtx_lock(&sp->mtx);
		val = arg2 == MAS_CTL_SAMPLE_RATE ? sp->sample_rate :
		    sp->be_active;
		mtx_unlock(&sp->mtx);
		break;
	case MAS_CTL_OWNER:
		sx_xlock(&sp->lock);
		val = sp->owned ? sp->owner_pid : 0;
		sx_xunlock(&sp->lock);
		break;
	case MAS_CTL_AMPS_ON:
		sx_xlock(&sp->lock);
		val = sp->amps_on;
		sx_xunlock(&sp->lock);
		break;
	default:
		val = (atomic_load_int(&sp->lock_failed) ? 1 : 0) |
		    (atomic_load_int(&sp->floor_failed) ? 2 : 0);
		break;
	}
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	switch (arg2) {
	case MAS_CTL_UNLOCK:
		return (mas_slk_put(sp, val, req->td->td_proc->p_pid));
	case MAS_CTL_LEASE:
		return (mas_spk_put(sp, val, req->td->td_proc->p_pid));
	case MAS_CTL_IFACE:
		return (mas_spk_iface_put(sp, val));
	default:
		return (EPERM);
	}
}

static void
mas_sysctls(struct ma_spk *sp)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *top, *ch;
	struct sysctl_oid *node, *anode;
	struct mas_amp *amp;
	int i;

	ctx = device_get_sysctl_ctx(sp->spk_dev);
	node = SYSCTL_ADD_NODE(ctx,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sp->spk_dev)), OID_AUTO,
	    "speaker", CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
	    "Speaker protection controls (see speaker-interface.md)");
	top = SYSCTL_CHILDREN(node);

#define	MAS_CTL(name, flags, which, descr)				\
	SYSCTL_ADD_PROC(ctx, top, OID_AUTO, name,			\
	    CTLTYPE_INT | CTLFLAG_MPSAFE | (flags), sp, (which),	\
	    mas_ctl_sysctl, "I", descr)
	MAS_CTL("unlock", CTLFLAG_RW, MAS_CTL_UNLOCK,
	    "Speaker Volume Unlock: 1 unlocked; owner writes the magic");
	MAS_CTL("lease", CTLFLAG_RW, MAS_CTL_LEASE,
	    "Speaker Protection: 1 granted; owner writes ms (0 releases)");
	MAS_CTL("interface", CTLFLAG_RW, MAS_CTL_IFACE,
	    "Speaker Protection Interface: 0 volume unlock, 1 lease");
	MAS_CTL("sample_rate", CTLFLAG_RD, MAS_CTL_SAMPLE_RATE,
	    "Speaker Sample Rate (0: speakers closed)");
	MAS_CTL("owner_pid", CTLFLAG_RD, MAS_CTL_OWNER,
	    "Process holding the owner device (0: none)");
	MAS_CTL("playing", CTLFLAG_RD, MAS_CTL_PLAYING,
	    "Speaker stream started (the unlock timeout runs)");
	MAS_CTL("amps_on", CTLFLAG_RD, MAS_CTL_AMPS_ON,
	    "Amps out of shutdown");
	MAS_CTL("failed", CTLFLAG_RD, MAS_CTL_FAILED,
	    "1: volume lock failed, 2: floor failed (reload to clear)");
#undef MAS_CTL
	SYSCTL_ADD_UINT(ctx, top, OID_AUTO, "refused", CTLFLAG_RD,
	    &sp->refused, 0, "Speaker starts without a grant (lease)");
	SYSCTL_ADD_INT(ctx, top, OID_AUTO, "unlock_magic", CTLFLAG_RD,
	    NULL, SPEAKER_MAGIC_VALUE, "Ping value (0xdec1be15)");
	SYSCTL_ADD_INT(ctx, top, OID_AUTO, "unlock_timeout_ms", CTLFLAG_RD,
	    NULL, SPEAKER_LOCK_TIMEOUT, "Ping lease, ms of playing time");
	SYSCTL_ADD_INT(ctx, top, OID_AUTO, "lease_min_ms", CTLFLAG_RD,
	    NULL, SPEAKER_LEASE_MIN_MS, "Shortest protection lease");
	SYSCTL_ADD_INT(ctx, top, OID_AUTO, "lease_max_ms", CTLFLAG_RD,
	    NULL, SPEAKER_LEASE_MAX_MS, "Longest protection lease");
	SYSCTL_ADD_INT(ctx, top, OID_AUTO, "lease_unowned_ms", CTLFLAG_RD,
	    NULL, SPEAKER_LEASE_UNOWNED_MS,
	    "Lease without owner falls back to the volume unlock after");
	SYSCTL_ADD_INT(ctx, top, OID_AUTO, "amps", CTLFLAG_RD, NULL,
	    sp->namps, "Number of amps");

	for (i = 0; i < sp->namps; i++) {
		amp = &sp->amps[i];
		anode = SYSCTL_ADD_NODE(ctx, top, OID_AUTO, amp->node,
		    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, amp->name);
		ch = SYSCTL_CHILDREN(anode);
		SYSCTL_ADD_STRING(ctx, ch, OID_AUTO, "name", CTLFLAG_RD,
		    amp->name, 0, "sound-name-prefix (the ALSA prefix)");
		SYSCTL_ADD_INT(ctx, ch, OID_AUTO, "index", CTLFLAG_RD, NULL,
		    i, "Codec order = TDM slot");
#define	MAS_AMP(name, flags, which, descr)				\
	SYSCTL_ADD_PROC(ctx, ch, OID_AUTO, name,			\
	    CTLTYPE_INT | CTLFLAG_MPSAFE | (flags), amp, (which),	\
	    mas_amp_sysctl, "I", descr)
		MAS_AMP("volume", CTLFLAG_RW, MAS_AMP_VOLUME,
		    "Speaker Volume (0 mute, mB = -10050 + 50 * v)");
		MAS_AMP("volume_max", CTLFLAG_RD, MAS_AMP_VOLUME_MAX,
		    "Speaker Volume's current maximum (the lock's ceiling)");
		MAS_AMP("amp_gain", CTLFLAG_RW, MAS_AMP_GAIN,
		    "Amp Gain Volume (mB = 1100 + 50 * v)");
		MAS_AMP("amp_gain_max", CTLFLAG_RD, MAS_AMP_GAIN_MAX,
		    "Amp Gain Volume's maximum");
		MAS_AMP("vsense", CTLFLAG_RW, MAS_AMP_VSENSE,
		    "VSENSE Switch");
		MAS_AMP("isense", CTLFLAG_RW, MAS_AMP_ISENSE,
		    "ISENSE Switch");
#undef MAS_AMP
		SYSCTL_ADD_INT(ctx, ch, OID_AUTO, "volume_min", CTLFLAG_RD,
		    NULL, 0, "Speaker Volume minimum (mute)");
		SYSCTL_ADD_INT(ctx, ch, OID_AUTO, "volume_db_min",
		    CTLFLAG_RD, NULL, MAS_VOL_DB_MIN,
		    "Speaker Volume mB at 0 (0 is mute)");
		SYSCTL_ADD_INT(ctx, ch, OID_AUTO, "volume_db_step",
		    CTLFLAG_RD, NULL, MAS_VOL_DB_STEP,
		    "Speaker Volume mB per step");
		SYSCTL_ADD_INT(ctx, ch, OID_AUTO, "amp_gain_min",
		    CTLFLAG_RD, NULL, 0, "Amp Gain Volume minimum");
		SYSCTL_ADD_INT(ctx, ch, OID_AUTO, "amp_gain_db_min",
		    CTLFLAG_RD, NULL, MAS_GAIN_DB_MIN,
		    "Amp Gain Volume mB at 0");
		SYSCTL_ADD_INT(ctx, ch, OID_AUTO, "amp_gain_db_step",
		    CTLFLAG_RD, NULL, MAS_GAIN_DB_STEP,
		    "Amp Gain Volume mB per step");
	}
}

/* ---- the pcm children ---- */

struct mas_pcm_softc {
	struct snddev_info	info;	/* first: pcm_register needs it */
	struct mas_chan		*ch;
};

static struct mas_chan *
mas_find_chan(device_t dev)
{
	struct ma_spk *sp;
	struct mas_chan *ch;

	ch = NULL;
	mtx_lock(&mas_cards_mtx);
	SLIST_FOREACH(sp, &mas_cards, link) {
		if (sp->spk_dev == dev)
			ch = &sp->play;
		else if (sp->sense_dev == dev)
			ch = &sp->sense;
		if (ch != NULL)
			break;
	}
	mtx_unlock(&mas_cards_mtx);
	return (ch);
}

static int
mas_pcm_probe(device_t dev)
{
	struct mas_chan *ch;

	if ((ch = mas_find_chan(dev)) == NULL)
		return (ENXIO);
	device_set_desc(dev, ch->dir == PCMDIR_PLAY ?
	    "Apple M5 audio (speakers)" : "Apple M5 audio (speaker sense)");
	return (BUS_PROBE_SPECIFIC);
}

static int
mas_pcm_attach(device_t dev)
{
	struct mas_pcm_softc *sc = device_get_softc(dev);
	struct mas_chan *ch;
	struct ma_spk *sp;

	if ((ch = mas_find_chan(dev)) == NULL)
		return (ENXIO);
	sc->ch = ch;
	sp = ch->sp;
	ch->dev = dev;
	pcm_init(dev, sc);
	pcm_addchan(dev, ch->dir, &mas_chan_class, ch);
	if (pcm_register(dev, ch->dir == PCMDIR_PLAY ?
	    "at macaudio (speakers)" : "at macaudio (speaker sense)") != 0) {
		device_printf(dev, "pcm_register failed\n");
		return (ENXIO);
	}
	if (ch->dir == PCMDIR_REC) {
		/*
		 * Twelve channels is more than sound(4)'s matrices: the sense
		 * is bit-perfect (no feeders, no vchans), its one format.
		 */
		pcm_setflags(dev, (pcm_getflags(dev) | SD_F_BITPERFECT) &
		    ~SD_F_RVCHANS);
		return (0);
	}
	mas_sysctls(sp);
	sp->cdev = make_dev(&mas_owner_cdevsw, device_get_unit(dev),
	    UID_ROOT, GID_WHEEL, 0600, "spkctl%d", device_get_unit(dev));
	sp->cdev->si_drv1 = sp;
	return (0);
}

static int
mas_pcm_detach(device_t dev)
{
	struct mas_pcm_softc *sc = device_get_softc(dev);
	struct mas_chan *ch = sc->ch;
	struct ma_spk *sp = ch->sp;
	int error;

	if ((error = pcm_unregister(dev)) != 0)
		return (error);
	/* Its channel is gone: power the stream down now */
	mtx_lock(&sp->mtx);
	ch->want = false;
	ch->running = false;
	mtx_unlock(&sp->mtx);
	taskqueue_enqueue(sp->tq, &sp->power_task);
	taskqueue_drain(sp->tq, &sp->power_task);
	if (ch->dir == PCMDIR_PLAY && sp->cdev != NULL) {
		/* Runs the owner's close (mas_owner_dtor()) */
		destroy_dev(sp->cdev);
		sp->cdev = NULL;
	}
	return (0);
}

static device_method_t mas_pcm_methods[] = {
	DEVMETHOD(device_probe,		mas_pcm_probe),
	DEVMETHOD(device_attach,	mas_pcm_attach),
	DEVMETHOD(device_detach,	mas_pcm_detach),
	DEVMETHOD_END
};

static driver_t mas_pcm_driver = {
	"pcm",
	mas_pcm_methods,
	sizeof(struct mas_pcm_softc),
};

/*
 * The children of the jack device, whose driver is "pcm" too.  Registered
 * ahead of the jack's driver: a kldload attaches the jack (and adds these)
 * as soon as its own driver registers.
 */
DRIVER_MODULE_ORDERED(apple_macaudio_spk, pcm, mas_pcm_driver, NULL, NULL,
    SI_ORDER_FIRST);

/* ---- the card ---- */

/* The "Speaker" link of the card node. */
static phandle_t
mas_find_link(device_t card)
{
	phandle_t link;
	char name[32];

	for (link = OF_child(ofw_bus_get_node(card)); link != 0;
	    link = OF_peer(link))
		if (OF_getprop(link, "link-name", name, sizeof(name)) > 0 &&
		    (strcmp(name, "Speaker") == 0 ||
		    strcmp(name, "Speakers") == 0))
			return (link);
	return (0);
}

/* "Left Woofer 1" -> "left_woofer_1" */
static void
mas_node_name(struct mas_amp *amp)
{
	char *p;

	if (amp->name[0] == '\0') {
		snprintf(amp->node, sizeof(amp->node), "amp%d", amp->index);
		return;
	}
	strlcpy(amp->node, amp->name, sizeof(amp->node));
	for (p = amp->node; *p != '\0'; p++) {
		if (*p >= 'A' && *p <= 'Z')
			*p += 'a' - 'A';
		else if ((*p < 'a' || *p > 'z') && (*p < '0' || *p > '9'))
			*p = '_';
	}
}

/*
 * The amps of the link (macaudio_parse_of(), macaudio_be_assign_tdm(),
 * macaudio_get_codec_idle_props(), macaudio_dpcm_hw_params()), and
 * macaudio's admission: speakers only for a known machine whose every amp
 * has its operating level from the boot loader.
 */
static int
mas_setup_amps(struct ma_spk *sp, phandle_t link)
{
	struct mas_amp *amp;
	phandle_t codec;
	pcell_t cells[MAS_MAX_AMPS];
	uint32_t mask, left_mask, right_mask, idle;
	char prop[40], mode[16];
	ssize_t len;
	int error, i, nslots, slot;

	if (!ofw_bus_node_is_compatible(ofw_bus_get_node(sp->card),
	    "apple,j815-macaudio") &&
	    !ofw_bus_node_is_compatible(ofw_bus_get_node(sp->card),
	    "apple,j813-macaudio")) {
		device_printf(sp->card, "driver can't assure safety on this "
		    "model, disabling speakers\n");
		return (ENXIO);
	}
	if ((codec = ofw_bus_find_child(link, "codec")) == 0 ||
	    (len = OF_getencprop(codec, "sound-dai", cells, sizeof(cells))) <=
	    0) {
		device_printf(sp->card, "Speaker link: no codecs\n");
		return (ENXIO);
	}
	sp->namps = len / sizeof(cells[0]);
	for (i = 0; i < sp->namps; i++) {
		amp = &sp->amps[i];
		amp->sp = sp;
		amp->index = i;
		amp->dev = OF_device_from_xref(cells[i]);
		if (amp->dev == NULL || !device_is_attached(amp->dev) ||
		    strcmp(device_get_name(amp->dev), "tas2764") != 0) {
			device_printf(sp->card, "speaker amp %d is not "
			    "attached\n", i);
			return (ENXIO);
		}
		strlcpy(amp->name, tas2764_get_prefix(amp->dev),
		    sizeof(amp->name));
		mas_node_name(amp);
		/* macaudio_sn012776_levels_present() */
		if (tas2764_get_adt_level(amp->dev) < 0) {
			device_printf(sp->card, "%s: no amp level from the "
			    "boot loader\n", amp->name);
			device_printf(sp->card, "no usable amp levels from the "
			    "boot loader, disabling speakers\n");
			return (ENXIO);
		}
	}

	/* One channel per amp: left on the even slots, right on the odd */
	left_mask = 0;
	for (i = 0; i < MAX(sp->namps, 2); i += 2)
		left_mask = left_mask << 2 | 1;
	right_mask = left_mask << 1;
	mask = left_mask | right_mask;
	nslots = flsl(mask);

	for (i = 0; i < sp->namps; i++) {
		amp = &sp->amps[i];
		slot = ffs(mask) - 1;
		mask &= ~(1u << slot);
		if ((error = tas2764_set_fmt(amp->dev, TAS2764_FMT_I2S,
		    TAS2764_INV_NB_IF)) != 0 ||
		    (error = tas2764_set_tdm(amp->dev, slot, -1, -1,
		    MAS_SLOT_WIDTH, nslots)) != 0)
			goto fail;
		snprintf(prop, sizeof(prop), "dai-tdm-idle-mode-%d", i);
		if (OF_getprop(link, prop, mode, sizeof(mode)) > 0) {
			mode[sizeof(mode) - 1] = '\0';
			snprintf(prop, sizeof(prop),
			    "dai-tdm-slot-tx-idle-mask-%d", i);
			idle = 0;
			(void)OF_getencprop(link, prop, &idle, sizeof(idle));
			if (strcmp(mode, "zero") == 0) {
				if ((error = tas2764_set_tdm_idle(amp->dev,
				    idle)) != 0)
					goto fail;
			} else
				device_printf(sp->card, "%s: idle mode %s "
				    "not supported\n", amp->name, mode);
		}
		if ((error = tas2764_set_bclk_ratio(amp->dev,
		    MAS_BCLK_RATIO)) != 0 ||
		    (error = tas2764_hw_params(amp->dev, MAS_RATE,
		    MAS_WIDTH)) != 0)
			goto fail;
		amp->vol = 0;
		amp->gain = tas2764_get_amp_gain(amp->dev);
		amp->vsense = amp->isense = false;
		(void)tas2764_set_sense(amp->dev, false, false);
	}
	return (0);
fail:
	device_printf(sp->card, "%s: setup: %d\n", sp->amps[i].name, error);
	return (error);
}

/* A stream's DMA channel and buffer, and its formats. */
static int
mas_chan_setup(struct ma_spk *sp, struct mas_chan *ch, int dir)
{
	struct apple_aop_audio_stream *s;
	phandle_t xref;
	uint32_t cell;
	void *kva;
	int error;

	ch->sp = sp;
	ch->dir = dir;
	ch->stream = dir == PCMDIR_PLAY ? "spkr" : "sense";
	/* The stream's shape: opened only to read it */
	if ((error = apple_aop_audio_stream_open(sp->aop_audio, ch->stream,
	    &s)) != 0)
		return (error);
	apple_aop_audio_stream_info(s, &ch->info);
	apple_aop_audio_stream_close(s);
	ch->dma_frame = ch->info.channels * ch->info.word_bytes;
	if (dir == PCMDIR_PLAY) {
		if (ch->info.channels != (uint32_t)sp->namps ||
		    ch->info.word_bytes != 4) {
			device_printf(sp->card, "spkr: %u channels for %d "
			    "amps\n", ch->info.channels, sp->namps);
			return (ENXIO);
		}
		ch->pcm_frame = MAS_PLAY_FRAME;
		ch->fmt[0] = SND_FORMAT(AFMT_S32_LE, 2, 0);
	} else {
		if (ch->info.word_bytes != 2 ||
		    ch->info.channels != 2 * (uint32_t)sp->namps) {
			device_printf(sp->card, "sense: %u words for %d amps\n",
			    ch->info.channels, sp->namps);
			return (ENXIO);
		}
		ch->pcm_frame = ch->dma_frame;
		ch->fmt[0] = SND_FORMAT(AFMT_S16_LE, ch->info.channels, 0);
	}
	ch->fmt[1] = 0;
	ch->caps = (struct pcmchan_caps){ MAS_RATE, MAS_RATE, ch->fmt, 0 };
	ch->bufsz = MAS_FRAMES * ch->pcm_frame;
	ch->dmalen = MAS_FRAMES * ch->dma_frame;
	ch->nblk = MAS_FRAMES / MAS_BLK_DEF;
	ch->blksz = MAS_BLK_DEF * ch->pcm_frame;
	ch->dmablk = MAS_BLK_DEF * ch->dma_frame;
	if ((error = apple_aop_audio_dma(sp->aop_audio, ch->stream, &xref,
	    &cell)) != 0 ||
	    (error = apple_admac_get_channel(xref, cell, &ch->dma)) != 0) {
		device_printf(sp->card, "%s DMA channel: %d\n", ch->stream,
		    error);
		return (error);
	}
	if ((error = apple_admac_alloc_buffer(ch->dma, ch->dmalen, &kva,
	    &ch->dmaiova)) != 0) {
		device_printf(sp->card, "%s DMA buffer: %d\n", ch->stream,
		    error);
		apple_admac_release(ch->dma);
		ch->dma = NULL;
		return (error);
	}
	ch->dmabuf = kva;
	return (0);
}

static void
mas_free(struct ma_spk *sp)
{
	struct mas_chan *chans[] = { &sp->play, &sp->sense };
	int i;

	mtx_lock(&mas_cards_mtx);
	SLIST_REMOVE(&mas_cards, sp, ma_spk, link);
	mtx_unlock(&mas_cards_mtx);
	if (sp->suspend_tag != NULL)
		EVENTHANDLER_DEREGISTER(power_suspend_early, sp->suspend_tag);
	if (sp->resume_tag != NULL)
		EVENTHANDLER_DEREGISTER(power_resume, sp->resume_tag);
	sx_xlock(&sp->lock);
	sp->dead = true;
	atomic_store_rel_int(&sp->granted, 0);
	sx_xunlock(&sp->lock);
	callout_drain(&sp->lease_callout);
	if (sp->tq != NULL) {
		while (taskqueue_cancel_timeout(sp->tq, &sp->lock_timeout_task,
		    NULL) != 0)
			taskqueue_drain_timeout(sp->tq, &sp->lock_timeout_task);
		while (taskqueue_cancel_timeout(sp->tq, &sp->unowned_task,
		    NULL) != 0)
			taskqueue_drain_timeout(sp->tq, &sp->unowned_task);
		taskqueue_drain_all(sp->tq);
		taskqueue_free(sp->tq);
	}
	/* The amps down, whatever happened before */
	if (sp->amps_on)
		(void)mas_amps_power(sp, false);
	for (i = 0; i < nitems(chans); i++) {
		if (chans[i]->dma == NULL)
			continue;
		apple_admac_free_buffer(chans[i]->dma, chans[i]->dmabuf);
		apple_admac_release(chans[i]->dma);
	}
	sx_destroy(&sp->lock);
	mtx_destroy(&sp->mtx);
	free(sp, M_DEVBUF);
}

int
ma_spk_attach(device_t card, device_t aop_audio, struct ma_spk **spp)
{
	struct ma_spk *sp;
	phandle_t link;
	int error;

	*spp = NULL;
	if ((link = mas_find_link(card)) == 0)
		return (0);

	sp = malloc(sizeof(*sp), M_DEVBUF, M_WAITOK | M_ZERO);
	sp->card = card;
	sp->aop_audio = aop_audio;
	mtx_init(&sp->mtx, "macaudio speakers", NULL, MTX_DEF);
	sx_init(&sp->lock, "macaudio speaker protection");
	callout_init(&sp->lease_callout, 1);
	mtx_lock(&mas_cards_mtx);
	SLIST_INSERT_HEAD(&mas_cards, sp, link);
	mtx_unlock(&mas_cards_mtx);

	if (mas_setup_amps(sp, link) != 0 ||
	    mas_chan_setup(sp, &sp->play, PCMDIR_PLAY) != 0) {
		mas_free(sp);
		return (0);
	}
	if (mas_chan_setup(sp, &sp->sense, PCMDIR_REC) != 0)
		device_printf(card, "no speaker sense: no protection daemon "
		    "can unlock the speakers\n");

	/* The withdrawal's workers: macaudio_spk_worker() runs SCHED_FIFO */
	sp->tq = taskqueue_create("macaudio_spk", M_WAITOK,
	    taskqueue_thread_enqueue, &sp->tq);
	taskqueue_start_threads(&sp->tq, 1, PI_REALTIME, "%s spk",
	    device_get_nameunit(card));
	TASK_INIT(&sp->power_task, 0, mas_power_task, sp);
	TASK_INIT(&sp->lock_update_task, 0, mas_lock_update_task, sp);
	TASK_INIT(&sp->expire_task, 0, mas_expire_task, sp);
	TIMEOUT_TASK_INIT(sp->tq, &sp->lock_timeout_task, 0,
	    mas_lock_timeout_task, sp);
	TIMEOUT_TASK_INIT(sp->tq, &sp->unowned_task, 0, mas_unowned_task, sp);

	/*
	 * The volume lock's floor holds from here on, before userland sees
	 * the devices: "Speaker Volume Unlock" in charge (macaudio_late_probe).
	 */
	sx_xlock(&sp->lock);
	atomic_store_int(&sp->iface, MAS_IFACE_UNLOCK);
	mas_vlimit_unlock(sp, false);
	error = atomic_load_int(&sp->lock_failed) ? EIO : 0;
	sx_xunlock(&sp->lock);
	if (error != 0) {
		device_printf(card, "speaker volume lock not set\n");
		mas_free(sp);
		return (0);
	}
	sp->suspend_tag = EVENTHANDLER_REGISTER(power_suspend_early,
	    mas_suspend, sp, EVENTHANDLER_PRI_FIRST);
	sp->resume_tag = EVENTHANDLER_REGISTER(power_resume, mas_resume, sp,
	    EVENTHANDLER_PRI_LAST);

	error = ENXIO;
	sp->spk_dev = device_add_child(card, "pcm", DEVICE_UNIT_ANY);
	if (sp->sense.dma != NULL)
		sp->sense_dev = device_add_child(card, "pcm", DEVICE_UNIT_ANY);
	if (sp->spk_dev == NULL ||
	    (error = device_probe_and_attach(sp->spk_dev)) != 0) {
		device_printf(card, "speaker device: %d\n", error);
		if (sp->spk_dev != NULL)
			device_delete_child(card, sp->spk_dev);
		if (sp->sense_dev != NULL)
			device_delete_child(card, sp->sense_dev);
		mas_free(sp);
		return (0);
	}
	if (sp->sense_dev != NULL &&
	    (error = device_probe_and_attach(sp->sense_dev)) != 0) {
		device_printf(card, "speaker sense device: %d\n", error);
		device_delete_child(card, sp->sense_dev);
		sp->sense_dev = NULL;
	}
	device_printf(card, "%d speaker amps; speakers locked until a "
	    "protection daemon unlocks them\n", sp->namps);
	*spp = sp;
	return (0);
}

int
ma_spk_detach(struct ma_spk *sp)
{
	int error;

	if (sp == NULL)
		return (0);
	if (sp->sense_dev != NULL) {
		if ((error = device_delete_child(sp->card, sp->sense_dev)) != 0)
			return (error);
		sp->sense_dev = NULL;
	}
	if (sp->spk_dev != NULL) {
		if ((error = device_delete_child(sp->card, sp->spk_dev)) != 0)
			return (error);
		sp->spk_dev = NULL;
	}
	mas_free(sp);
	return (0);
}
