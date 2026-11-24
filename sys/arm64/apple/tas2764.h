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
 * TI TAS2764 / SN012776 speaker amplifier: interface for the sound glue.
 *
 * Follows aurora-silicon's Linux sound/soc/codecs/tas2764.c (ASoC).  The
 * setters only record the configuration; everything is written to the amp
 * by tas2764_set_power(dev, true), because the amps share one shutdown
 * (SDZ) line and lose every register whenever it is low.  Call order, as
 * macaudio drives the speaker link (spk_dai_fmt, speaker_init, hw_params,
 * mute_stream):
 *
 *   card bind     tas2764_set_fmt()        (set_fmt; default I2S + NB_IF)
 *                 tas2764_set_tdm()        (set_tdm_slot)
 *                 tas2764_set_tdm_idle()   (set_tdm_idle, codecs 0 and 1)
 *   hw_params     tas2764_set_bclk_ratio() (set_bclk_ratio: 250)
 *                 tas2764_hw_params()      (hw_params)
 *   trigger       tas2764_set_power(on)    (mute_stream(0))
 *   stop          tas2764_set_power(off)   (mute_stream(1))
 *
 * For the J815 speaker link: tas2764_set_fmt(dev, TAS2764_FMT_I2S,
 * TAS2764_INV_NB_IF); amp k of the dai-link's codec list (ADT
 * speaker-config order) gets tas2764_set_tdm(dev, k, -1, -1, 32, 8)
 * (macaudio: nslots = fls(tdm_mask), MACAUDIO_SLOTWIDTH 32); then
 * tas2764_hw_params(dev, 48000, 24).
 *
 * HARD LIMITS, enforced inside the driver whatever the caller asks:
 * the digital volume never exceeds TAS2764_VOL_CAP (-20 dB) and the amp
 * gain never exceeds TAS2764_AMP_GAIN_CAP (the lowest setting).  A
 * tweeter (sound-name-prefix containing "Tweeter") always gets the 800 Hz
 * playback high-pass.  Over-temperature or over-current on any amp drops
 * the shared shutdown line and refuses power-on until the module reloads.
 *
 * The device_t is the amp's own; the glue finds it from the dai-link's
 * sound-dai phandle with OF_device_from_xref().  All functions sleep
 * (I2C); do not call them with a mutex held or from interrupt context.
 * They return 0 or an errno.
 */

#ifndef _ARM64_APPLE_TAS2764_H_
#define	_ARM64_APPLE_TAS2764_H_

/* Link formats (SND_SOC_DAIFMT_FORMAT_MASK). */
#define	TAS2764_FMT_I2S		0
#define	TAS2764_FMT_DSP_A	1
#define	TAS2764_FMT_DSP_B	2
#define	TAS2764_FMT_LEFT_J	3

/* Clock polarities (SND_SOC_DAIFMT_INV_MASK). */
#define	TAS2764_INV_NB_NF	0
#define	TAS2764_INV_NB_IF	1
#define	TAS2764_INV_IB_NF	2
#define	TAS2764_INV_IB_IF	3

/*
 * Volume, in the units of the Linux "Speaker Volume" control (and so of
 * macaudio's SN012776_0DB / SN012776_DB()): 0 is mute, v is
 * -100.5 + v / 2 dB, 201 is 0 dB.  Clamped to TAS2764_VOL_CAP.
 */
#define	TAS2764_VOL_0DB		201
#define	TAS2764_VOL_DB(x)	(TAS2764_VOL_0DB + 2 * (x))
#define	TAS2764_VOL_CAP		TAS2764_VOL_DB(-20)	/* 161: -20 dB */
#define	TAS2764_VOL_MUTE	0

/*
 * Amp gain ("Amp Gain Volume"): level 0..20 is 11 + level / 2 dB.  Clamped
 * to TAS2764_AMP_GAIN_CAP (and to the ADT level, when the DT has one).
 */
#define	TAS2764_AMP_GAIN_MAX	20
#define	TAS2764_AMP_GAIN_CAP	0			/* 11 dB, the lowest */

/*
 * Link format and polarity.  The amp is always the clock consumer.
 * EBUSY while the amp is powered.
 */
int	tas2764_set_fmt(device_t dev, int fmt, int inv);

/*
 * The playback slot (tx_slot, Linux set_tdm_slot's tx_mask = 1 << tx_slot;
 * the amp takes that one slot for its "Left" input), the frame's slot count
 * and slot width (16, 24 or 32).  imon_slot / vmon_slot are the V/I sense
 * transmit slots (0..63); -1 keeps the device tree's ti,imon-slot-no /
 * ti,vmon-slot-no.  The sense ADCs stay powered off: nothing captures them
 * yet.  EBUSY while powered.
 */
int	tas2764_set_tdm(device_t dev, int tx_slot, int imon_slot, int vmon_slot,
	    int slot_width, int nslots);

/* The V/I sense slots in use (-1: none), for a later speakersafetyd port. */
void	tas2764_get_sense_slots(device_t dev, int *imon_slot, int *vmon_slot);

/*
 * SDOUT idle slots driven to zero (set_tdm_idle, IDLE_ZERO); 0 sets them
 * high-impedance.  Ignored, as on Linux, when the DT has
 * apple,amp-tx-zd-config.  EBUSY while powered.
 */
int	tas2764_set_tdm_idle(device_t dev, uint32_t tx_mask);

/* set_bclk_ratio: crops the idle mask to ratio / 8 slots. */
int	tas2764_set_bclk_ratio(device_t dev, u_int ratio);

/*
 * Rate (44100, 48000, 88200, 96000) and sample width (16, 24, 32 bits:
 * S16_LE, S24_LE, S32_LE).  EBUSY while powered.
 */
int	tas2764_hw_params(device_t dev, uint32_t rate, uint32_t width);

/*
 * on: raise the shared shutdown line if it is low, program the amp in
 * software shutdown (reset, quirks, TDM, format, rate, gain and volume
 * capped), then mute, then active last.  Any failure puts the amp back in
 * software shutdown and drops its vote on the line.  off: mute, software
 * shutdown, then the line goes low when no amp is on any more.
 */
int	tas2764_set_power(device_t dev, bool on);

/* Volume (see above).  Clamped; applies immediately when powered. */
int	tas2764_set_volume(device_t dev, u_int level);
u_int	tas2764_get_volume(device_t dev);

/* Amp gain level (see above).  Clamped; applies immediately when powered. */
int	tas2764_set_amp_gain(device_t dev, u_int level);

/* True when this amp is a tweeter (sound-name-prefix). */
bool	tas2764_is_tweeter(device_t dev);

#endif /* _ARM64_APPLE_TAS2764_H_ */
