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
 * Cirrus Logic CS42L84 headphone-jack codec: interface for the sound glue.
 *
 * Follows aurora-silicon's Linux sound/soc/codecs/cs42l84.c (ASoC).  The
 * ASoC callbacks map onto these functions as follows; call them in this
 * order, as ASoC does:
 *
 *   card bind          cs42l84_set_fmt()        (snd_soc_dai_set_fmt)
 *   hw_params          cs42l84_set_sysclk()     (macaudio: rate * bclk ratio)
 *                      cs42l84_hw_params()      (dai_ops.hw_params)
 *   prepare/trigger    cs42l84_set_power(on)    (DAPM power-up + unmute)
 *   stop/hw_free       cs42l84_set_power(off)   (mute + DAPM power-down)
 *                      cs42l84_set_sysclk(0)
 *
 * The codec is a clock consumer (ASoC BC_FC): BCLK and FSYNC come from the
 * AOP.  For the M5 jack (macaudio j813/j815 cfg) the link is DSP_A with an
 * inverted bit clock (IB_NF), a 64-BCLK frame (primary FE bclk ratio 64),
 * two 32-bit slots, so at 48 kHz: CS42L84_FMT_DSP_A, sysclk 3072000,
 * hw_params(48000, 24, 32).
 *
 * The device_t is the codec's own; the glue finds it from the dai-link's
 * sound-dai phandle with OF_device_from_xref() (the driver registers its
 * xref at attach).  All functions sleep (I2C); do not call them with a
 * mutex held or from interrupt context.  They return 0 or an errno.
 */

#ifndef _ARM64_APPLE_CS42L84_H_
#define	_ARM64_APPLE_CS42L84_H_

/* Stream directions, as SNDRV_PCM_STREAM_PLAYBACK/CAPTURE. */
#define	CS42L84_DIR_PLAYBACK	0
#define	CS42L84_DIR_CAPTURE	1

/*
 * Link formats (set_dai_fmt).  The bus polarities are fixed by the format:
 * I2S implies IB_IF, DSP_A implies IB_NF; both mean data one BCLK after the
 * rising FSYNC edge.
 */
#define	CS42L84_FMT_I2S		0
#define	CS42L84_FMT_DSP_A	1

/*
 * Volume: 0-100 per channel.  0 is the DAC's mute step (-128 dB); 1-100 map
 * linearly in dB onto -59.5 .. 0 dB (0.6 dB per step, rounded to the DAC's
 * 0.5 dB grid).  0 dB is the ceiling, as macaudio's snd_soc_limit_volume()
 * at CS42L84_DAC_0DB: above it the DAC only adds clipping digital gain.
 */
#define	CS42L84_VOL_MAX		100
#define	CS42L84_VOL_DEFAULT	50	/* -30 dB */
#define	CS42L84_VOL_POWERUP_MAX	80	/* -12 dB: ceiling when powering up */

/* Jack state / event bits passed to the callback. */
#define	CS42L84_JACK_HEADPHONE	0x0001	/* plug present */
#define	CS42L84_JACK_MIC	0x0002	/* headset with mic (load on HS4) */
#define	CS42L84_JACK_BTN_PLAY	0x0100	/* remote centre button pressed */
#define	CS42L84_JACK_BTN_VOLUP	0x0200	/* remote volume up pressed */
#define	CS42L84_JACK_BTN_VOLDN	0x0400	/* remote volume down pressed */
#define	CS42L84_JACK_STATE_MASK	0x00ff
#define	CS42L84_JACK_BTN_MASK	0xff00

/*
 * Called from the driver's taskqueue thread, without the driver's lock,
 * whenever the plug state changes (state = HEADPHONE[|MIC], or 0 for an
 * unplug) and once per remote button press (state bits plus one BTN bit,
 * press and release folded into one event as Linux reports a pair).  It
 * may call back into the cs42l84_* functions.
 */
typedef void cs42l84_jack_cb_t(void *arg, uint32_t state);

/* Link format, CS42L84_FMT_*.  The codec is always the clock consumer. */
int	cs42l84_set_fmt(device_t dev, int fmt);

/*
 * BCLK frequency in Hz (Linux set_sysclk: macaudio passes rate * bclk
 * ratio).  Supported: 2822400, 3072000, 5644800, 6144000, 11289600,
 * 12288000, 24576000; 0 clears it.
 */
int	cs42l84_set_sysclk(device_t dev, uint32_t bclk);

/*
 * Rate (44100, 48000, 88200, 96000, 176400, 192000), sample width in bits
 * (16, 24 or 32: params_width) and slot width (params_physical_width,
 * 32 for S24_LE).  Programs the PLL, the frame length and the
 * direction's ASP channel widths and slot positions.  Fails with EBUSY if
 * a stream is running at another BCLK.  Channels are not a parameter:
 * the codec always takes slot 0 to DACA and slot 1 to DACB (mono drives
 * slot 0 only), and transmits the ADC in slot 0.
 */
int	cs42l84_hw_params(device_t dev, int dir, uint32_t rate,
	    uint32_t width, uint32_t slot_width);

/*
 * Power a direction's path up (DAPM supplies, ASP channel, DAC or ADC, then
 * the clock switch to the PLL and the unmute) or down (the reverse).
 * hw_params must have been called for "on".  For playback the DAC volume
 * is limited to CS42L84_VOL_POWERUP_MAX at power-up.
 */
int	cs42l84_set_power(device_t dev, int dir, bool on);

/* Headphone volume, 0-100 per channel (see above).  Applies immediately. */
int	cs42l84_set_volume(device_t dev, u_int left, u_int right);
void	cs42l84_get_volume(device_t dev, u_int *left, u_int *right);

/*
 * User mute of the headphone DAC, independent of the stream: while set,
 * set_power(on) leaves the DAC muted.
 */
int	cs42l84_set_mute(device_t dev, bool mute);

/*
 * Headset mic gain: preamp 0, 10 or 20 dB (preamp_db) and PGA -12 .. 0 dB
 * in 0.5 dB steps (pga_half_db, -24 .. 0), as the Linux controls' TLVs.
 * The chip's reset values are left alone until this is called.
 */
int	cs42l84_set_mic_gain(device_t dev, u_int preamp_db, int pga_half_db);

/* Current plug state: headphone (plug present) and mic (headset). */
int	cs42l84_jack_status(device_t dev, bool *headphone, bool *mic);

/*
 * Set (or clear, cb NULL) the jack callback.  Like Linux set_jack, the
 * current state is reported to a new callback right away.
 */
void	cs42l84_set_jack_callback(device_t dev, cs42l84_jack_cb_t *cb,
	    void *arg);

/*
 * Service the codec's plug interrupt sources (the Linux IRQ thread).  The
 * driver polls this itself; an interrupt handler for the jack line
 * (pinctrl_ap 100, level-low) can call it instead from a taskqueue, and
 * then cs42l84_set_poll(dev, false) stops the plug poll (the remote button
 * poll keeps running while a headset is in, as on Linux).
 */
void	cs42l84_jack_event(device_t dev);
void	cs42l84_set_poll(device_t dev, bool on);

#endif /* _ARM64_APPLE_CS42L84_H_ */
