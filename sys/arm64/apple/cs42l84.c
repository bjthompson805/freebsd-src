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
 * Cirrus Logic CS42L84 headphone-jack codec (Apple silicon Macs).
 *
 * Follows aurora-silicon's Linux sound/soc/codecs/cs42l84.c and cs42l84.h
 * (Asahi Linux contributors, based on Cirrus' cs42l42), register for
 * register.  The ASoC DAPM graph is flattened into fixed power-up and
 * power-down sequences per stream direction (cs42l84_set_power()); the
 * plug interrupt thread is run from a poll, since the jack line's pin
 * controller has no interrupt support yet.  The interface for the sound
 * glue is in cs42l84.h.
 *
 * Registers are 16-bit (big-endian on the wire), values 8-bit, one
 * register per transfer (regmap use_single_read/write).  There is no
 * register cache: read-modify-write goes to the chip, and like regmap's
 * update_bits a write is skipped when the value does not change.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/sx.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <dev/gpio/gpiobusvar.h>
#include <dev/iicbus/iic.h>
#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

#include <arm64/apple/cs42l84.h>

#include "iicbus_if.h"

/* Registers (Linux cs42l84.h). */
#define	CS42L84_CHIP_ID				0x42a84
#define	CS42L84_DEVID				0x0000
#define	CS42L84_REVID				0x73fe
#define	CS42L84_FRZ_CTL				0x0006
#define	 CS42L84_FRZ_CTL_ENGAGE			0x01
#define	CS42L84_TSRS_PLUG_INT_STATUS		0x0400
#define	CS42L84_TSRS_PLUG_INT_MASK		0x0418
#define	 CS42L84_RS_PLUG_SHIFT			0
#define	 CS42L84_RS_PLUG			0x01
#define	 CS42L84_RS_UNPLUG			0x02
#define	 CS42L84_TS_PLUG_SHIFT			2
#define	 CS42L84_TS_PLUG			0x04
#define	 CS42L84_TS_UNPLUG			0x08
#define	 CS42L84_TSRS_PLUG_VAL_MASK		0x0f
#define	CS42L84_PLL_LOCK_STATUS			0x040e
#define	 CS42L84_PLL_LOCK_STATUS_LOCKED		0x10
#define	 CS42L84_PLL_LOCK_STATUS_ERROR		0x20
#define	CS42L84_PLUG				3
#define	CS42L84_UNPLUG				0
#define	CS42L84_TRANS				1
#define	CS42L84_CCM_CTL1			0x0600
#define	 CS42L84_CCM_CTL1_MCLK_SRC_MASK		0x03
#define	 CS42L84_CCM_CTL1_MCLK_SRC_RCO		0
#define	 CS42L84_CCM_CTL1_MCLK_SRC_BCLK		2
#define	 CS42L84_CCM_CTL1_MCLK_SRC_PLL		3
#define	 CS42L84_CCM_CTL1_MCLK_FREQ_SHIFT	2
#define	 CS42L84_CCM_CTL1_MCLK_FREQ_MASK	0x0c
#define	 CS42L84_CCM_CTL1_MCLK_F_12MHZ		0
#define	 CS42L84_CCM_CTL1_MCLK_F_24MHZ		1
#define	 CS42L84_CCM_CTL1_MCLK_F_12_288KHZ	2
#define	 CS42L84_CCM_CTL1_MCLK_F_24_576KHZ	3
#define	 CS42L84_CCM_CTL1_RCO			0x00	/* RCO, 12 MHz */
#define	CS42L84_CCM_SAMP_RATE			0x0601
#define	CS42L84_CCM_CTL3			0x0602
#define	 CS42L84_CCM_CTL3_REFCLK_DIV_SHIFT	1
#define	 CS42L84_CCM_CTL3_REFCLK_DIV_MASK	0x06
#define	CS42L84_CCM_CTL4			0x0603
#define	 CS42L84_CCM_CTL4_REFCLK_EN		0x01
#define	CS42L84_PLL_CTL1			0x0800
#define	 CS42L84_PLL_CTL1_EN			0x01
#define	 CS42L84_PLL_CTL1_MODE_SHIFT		1
#define	 CS42L84_PLL_CTL1_MODE_MASK		0x06
#define	CS42L84_PLL_DIV_FRAC0			0x0804
#define	CS42L84_PLL_DIV_FRAC1			0x0805
#define	CS42L84_PLL_DIV_FRAC2			0x0806
#define	CS42L84_PLL_DIV_INT			0x0807
#define	CS42L84_PLL_DIVOUT			0x0808
#define	CS42L84_RING_SENSE_CTL			0x1282
#define	 CS42L84_RING_SENSE_CTL_INV		0x80
#define	 CS42L84_RING_SENSE_CTL_UNK1		0x40
#define	 CS42L84_RING_SENSE_CTL_FALLTIME_SHIFT	3
#define	 CS42L84_RING_SENSE_CTL_FALLTIME_MASK	0x38
#define	 CS42L84_RING_SENSE_CTL_RISETIME_MASK	0x07
#define	CS42L84_TIP_SENSE_CTL			0x1283
#define	 CS42L84_TIP_SENSE_CTL_INV		0x80
#define	 CS42L84_TIP_SENSE_CTL_FALLTIME_SHIFT	3
#define	 CS42L84_TIP_SENSE_CTL_FALLTIME_MASK	0x38
#define	 CS42L84_TIP_SENSE_CTL_RISETIME_MASK	0x07
#define	CS42L84_TSRS_PLUG_STATUS		0x1288
#define	CS42L84_DET_UNK1471			0x1471	/* ring-sense setup */
#define	CS42L84_TIP_SENSE_CTL2			0x1473
#define	 CS42L84_TIP_SENSE_CTL2_MODE_MASK	0xc0
#define	 CS42L84_TIP_SENSE_CTL2_MODE_SHORT_DET	0xc0	/* 0b11 << 6 */
#define	CS42L84_MISC_DET_CTL			0x1474
#define	 CS42L84_MISC_DET_CTL_DETECT_MODE_SHIFT	3
#define	 CS42L84_MISC_DET_CTL_DETECT_MODE_MASK	0x18
#define	 CS42L84_MISC_DET_CTL_HSBIAS_CTL_SHIFT	1
#define	 CS42L84_MISC_DET_CTL_HSBIAS_CTL_MASK	0x06
#define	 CS42L84_MISC_DET_CTL_PDN_MIC_LVL_DET	0x01
#define	CS42L84_MIC_DET_CTL1			0x1475
#define	 CS42L84_MIC_DET_CTL1_HS_DET_LEVEL_MASK	0x3f
#define	CS42L84_MIC_DET_CTL4			0x1477
#define	 CS42L84_MIC_DET_CTL4_LATCH_TO_VP	0x02
#define	CS42L84_DET_UNK1478			0x1478	/* ring-sense setup */
#define	CS42L84_HS_DET_STATUS2			0x147d
#define	 CS42L84_HS_DET_STATUS2_SHORT_TRUE	0x01
#define	CS42L84_DET_UNK1480			0x1480	/* ring-sense setup */
#define	CS42L84_REMOTE_BUTTON_STATUS1		0x1484
#define	 CS42L84_REMOTE_VOLUME_DOWN		0x01
#define	 CS42L84_REMOTE_VOLUME_UP		0x02
#define	CS42L84_MSM_BLOCK_EN2			0x1801
#define	 CS42L84_MSM_BLOCK_EN2_ASP		0x40
#define	 CS42L84_MSM_BLOCK_EN2_BUS		0x20
#define	 CS42L84_MSM_BLOCK_EN2_DAC		0x10
#define	 CS42L84_MSM_BLOCK_EN2_ADC		0x08
#define	CS42L84_MSM_BLOCK_EN3			0x1802
#define	 CS42L84_MSM_BLOCK_EN3_TR_SENSE		0x08
#define	CS42L84_HS_DET_CTL2			0x1811
#define	 CS42L84_HS_DET_CTL2_CTL_MASK		0xc0
#define	 CS42L84_HS_DET_CTL2_SET_SHIFT		4
#define	 CS42L84_HS_DET_CTL2_SET_MASK		0x30
#define	CS42L84_HS_SWITCH_CTL			0x1812
#define	 CS42L84_HS_SWITCH_CTL_REF_HS3		0x80
#define	 CS42L84_HS_SWITCH_CTL_REF_HS4		0x40
#define	 CS42L84_HS_SWITCH_CTL_HSB_FILT_HS3	0x20
#define	 CS42L84_HS_SWITCH_CTL_HSB_FILT_HS4	0x10
#define	 CS42L84_HS_SWITCH_CTL_HSB_HS3		0x08
#define	 CS42L84_HS_SWITCH_CTL_HSB_HS4		0x04
#define	 CS42L84_HS_SWITCH_CTL_GNDHS_HS3	0x02
#define	 CS42L84_HS_SWITCH_CTL_GNDHS_HS4	0x01
#define	CS42L84_HS_CLAMP_DISABLE		0x1813
#define	CS42L84_ADC_CTL1			0x2000
#define	 CS42L84_ADC_CTL1_PREAMP_GAIN_SHIFT	6
#define	 CS42L84_ADC_CTL1_PREAMP_GAIN_MASK	0xc0
#define	 CS42L84_ADC_CTL1_PGA_GAIN_MASK		0x1f	/* max 24 */
#define	CS42L84_DAC_CTL1			0x3000
#define	 CS42L84_DAC_CTL1_UNMUTE		0x01
#define	CS42L84_DAC_CHA_VOL_LSB			0x3004
#define	CS42L84_DAC_CHA_VOL_MSB			0x3005
#define	CS42L84_DAC_CHB_VOL_LSB			0x3006
#define	CS42L84_DAC_CHB_VOL_MSB			0x3007
#define	CS42L84_HP_VOL_CTL			0x3020
#define	 CS42L84_HP_VOL_CTL_ZERO_CROSS		0x02
#define	 CS42L84_HP_VOL_CTL_SOFT		0x01
#define	CS42L84_BUS_ASP_TX_SRC			0x4000
#define	CS42L84_BUS_DAC_SRC			0x4001
#define	 CS42L84_SRC_ADC			0x7	/* 0b0111 */
#define	 CS42L84_SRC_ASP_RX_CH1			0xd	/* 0b1101 */
#define	 CS42L84_SRC_ASP_RX_CH2			0xe	/* 0b1110 */
#define	CS42L84_ASP_CTL				0x5000
#define	 CS42L84_ASP_CTL_TDM_MODE		0x04
#define	 CS42L84_ASP_CTL_BCLK_EN		0x02
#define	CS42L84_ASP_FSYNC_CTL2			0x5010
#define	 CS42L84_ASP_FSYNC_CTL2_BCLK_PERIOD_LO	0xfe	/* bits 7:1 */
#define	CS42L84_ASP_FSYNC_CTL3			0x5011
#define	 CS42L84_ASP_FSYNC_CTL3_BCLK_PERIOD_HI	0x1f
#define	CS42L84_ASP_RX_EN			0x5020
#define	 CS42L84_ASP_RX_EN_CH1			0x01
#define	 CS42L84_ASP_RX_EN_CH2			0x02
#define	CS42L84_ASP_TX_EN			0x5024
#define	 CS42L84_ASP_TX_EN_CH1			0x01
#define	CS42L84_ASP_RX_CH1_CTL1			0x5028
#define	CS42L84_ASP_RX_CH1_CTL2			0x5029
#define	CS42L84_ASP_RX_CH1_WIDTH		0x502a
#define	CS42L84_ASP_RX_CH2_CTL1			0x502c
#define	CS42L84_ASP_RX_CH2_CTL2			0x502d
#define	CS42L84_ASP_RX_CH2_WIDTH		0x502e
#define	 CS42L84_ASP_CHx_CTL1_EDGE		0x01
#define	 CS42L84_ASP_CHx_CTL1_SLOT_START_LSB	0xfe	/* bits 7:1 */
#define	 CS42L84_ASP_CHx_CTL2_SLOT_START_MSB	0x07
#define	CS42L84_ASP_TX_CH1_CTL1			0x5068
#define	CS42L84_ASP_TX_CH1_CTL2			0x5069
#define	CS42L84_ASP_TX_CH1_WIDTH		0x506a
#define	CS42L84_ASP_TX_CH2_CTL1			0x506c
#define	CS42L84_ASP_TX_CH2_CTL2			0x506d
#define	CS42L84_ASP_TX_CH2_WIDTH		0x506e

#define	CS42L84_DEBOUNCE_TIME_125MS		1
#define	CS42L84_DEBOUNCE_TIME_500MS		3

#define	CS42L84_BOOT_TIME_US			3000
#define	CS42L84_CLOCK_SWITCH_DELAY_US		150
#define	CS42L84_PLL_LOCK_POLL_US		250
#define	CS42L84_PLL_LOCK_TIMEOUT_US		1250
#define	CS42L84_REMOTE_POLL_MS			30	/* Linux remote poll */
#define	CS42L84_PLUG_POLL_MS			300	/* stands in for the IRQ */

/*
 * DAC volume: 9-bit two's complement in 0.5 dB, -256 (-128 dB, the TLV
 * mute step) .. 24 (+12 dB).  We stop at 0 (0 dB), see cs42l84.h.
 */
#define	CS42L84_DAC_VOL_MUTE			(-256)

#define	CS42L84_NEVENTS				8

static const struct cs42l84_pll_params {
	uint32_t	bclk;
	uint8_t		mclk_src_sel;
	uint8_t		bclk_prediv;
	uint8_t		pll_div_int;
	uint32_t	pll_div_frac;
	uint8_t		pll_mode;
	uint8_t		pll_divout;
	uint32_t	mclk_int;
} pll_ratio_table[] = {
	{  2822400, 1, 0, 0x40, 0x000000, 0x03, 0x10, 11289600 },
	{  3072000, 1, 0, 0x40, 0x000000, 0x03, 0x10, 12288000 },
	{  5644800, 1, 0, 0x40, 0x000000, 0x03, 0x10, 11289600 },
	{  6144000, 1, 1, 0x40, 0x000000, 0x03, 0x10, 12288000 },
	{ 11289600, 0, 0, 0, 0, 0, 0,                 11289600 },
	{ 12288000, 0, 0, 0, 0, 0, 0,                 12288000 },
	{ 24576000, 1, 3, 0x40, 0x000000, 0x03, 0x10, 12288000 },
};

static struct ofw_compat_data compat_data[] = {
	{ "cirrus,cs42l84",	1 },
	{ NULL,			0 }
};

struct cs42l84_softc {
	device_t		dev;
	struct sx		lock;
	gpio_pin_t		reset_gpio;
	struct timeout_task	poll_task;
	bool			detaching;
	bool			poll_plug;

	/* Jack (Linux irq_lock state). */
	uint8_t			tip_state;
	uint8_t			ring_state;
	uint32_t		hs_type;	/* CS42L84_JACK_* state bits */
	bool			ring_detect;
	bool			remote_active;
	uint8_t			remote_btn1_prev;
	uint8_t			remote_short_prev;
	cs42l84_jack_cb_t	*jack_cb;
	void			*jack_cb_arg;
	uint32_t		events[CS42L84_NEVENTS];
	int			nevents;

	/* Clocking and streams. */
	int			pll_config;
	uint32_t		bclk;
	uint8_t			pll_mclk_f;
	uint32_t		srate;
	uint8_t			stream_use;	/* bit per direction */
	bool			dsp_a;
	bool			path_on[2];

	/* Mixer. */
	u_int			vol[2];
	bool			mute;
};

static int	cs42l84_probe(device_t);
static int	cs42l84_attach(device_t);
static int	cs42l84_detach(device_t);

/*
 * Register access.
 */

static int
cs42l84_read(struct cs42l84_softc *sc, uint16_t reg, uint8_t *val)
{
	uint8_t addr[2];
	struct iic_msg msgs[2];
	int error;

	addr[0] = reg >> 8;
	addr[1] = reg & 0xff;
	msgs[0].slave = iicbus_get_addr(sc->dev);
	msgs[0].flags = IIC_M_WR | IIC_M_NOSTOP;
	msgs[0].len = sizeof(addr);
	msgs[0].buf = addr;
	msgs[1].slave = iicbus_get_addr(sc->dev);
	msgs[1].flags = IIC_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = val;
	error = iicbus_transfer_excl(sc->dev, msgs, nitems(msgs), IIC_WAIT);
	if (error != 0)
		device_printf(sc->dev, "read 0x%04x failed: %d\n", reg, error);
	return (error);
}

static int
cs42l84_write(struct cs42l84_softc *sc, uint16_t reg, uint8_t val)
{
	uint8_t buf[3];
	struct iic_msg msg;
	int error;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val;
	msg.slave = iicbus_get_addr(sc->dev);
	msg.flags = IIC_M_WR;
	msg.len = sizeof(buf);
	msg.buf = buf;
	error = iicbus_transfer_excl(sc->dev, &msg, 1, IIC_WAIT);
	if (error != 0)
		device_printf(sc->dev, "write 0x%04x <- 0x%02x failed: %d\n",
		    reg, val, error);
	return (error);
}

/* regmap_update_bits(): skips the write when nothing changes. */
static int
cs42l84_update(struct cs42l84_softc *sc, uint16_t reg, uint8_t mask,
    uint8_t val)
{
	uint8_t old, new;
	int error;

	if ((error = cs42l84_read(sc, reg, &old)) != 0)
		return (error);
	new = (old & ~mask) | (val & mask);
	if (new == old)
		return (0);
	return (cs42l84_write(sc, reg, new));
}

/* regmap_write_bits(): always writes. */
static int
cs42l84_write_bits(struct cs42l84_softc *sc, uint16_t reg, uint8_t mask,
    uint8_t val)
{
	uint8_t old;
	int error;

	if ((error = cs42l84_read(sc, reg, &old)) != 0)
		return (error);
	return (cs42l84_write(sc, reg, (old & ~mask) | (val & mask)));
}

/* cirrus_read_device_id() (cirrus_legacy.h): 20 bits from three bytes. */
static int
cs42l84_read_device_id(struct cs42l84_softc *sc, uint32_t *devid)
{
	uint8_t id[3];
	int error, i;

	for (i = 0; i < 3; i++)
		if ((error = cs42l84_read(sc, CS42L84_DEVID + i, &id[i])) != 0)
			return (error);
	*devid = ((uint32_t)id[0] << 12) | ((uint32_t)id[1] << 4) |
	    ((id[2] & 0xf0) >> 4);
	return (0);
}

/*
 * Jack events: queued under the lock, delivered after it is dropped.
 */

static void
cs42l84_report(struct cs42l84_softc *sc, uint32_t state)
{

	sx_assert(&sc->lock, SA_XLOCKED);
	if (sc->nevents < CS42L84_NEVENTS)
		sc->events[sc->nevents++] = state;
}

static void
cs42l84_unlock_deliver(struct cs42l84_softc *sc)
{
	uint32_t events[CS42L84_NEVENTS];
	cs42l84_jack_cb_t *cb;
	void *arg;
	int i, n;

	cb = sc->jack_cb;
	arg = sc->jack_cb_arg;
	n = sc->nevents;
	memcpy(events, sc->events, n * sizeof(events[0]));
	sc->nevents = 0;
	sx_xunlock(&sc->lock);

	/* devd/userland: system CS42L84, subsystem JACK or BUTTON. */
	for (i = 0; i < n; i++) {
		char data[48];

		if (events[i] & CS42L84_JACK_BTN_MASK) {
			snprintf(data, sizeof(data), "button=%s",
			    events[i] & CS42L84_JACK_BTN_PLAY ? "play" :
			    events[i] & CS42L84_JACK_BTN_VOLUP ? "volup" :
			    "voldown");
			devctl_notify("CS42L84", device_get_nameunit(sc->dev),
			    "BUTTON", data);
		} else {
			snprintf(data, sizeof(data), "headphone=%d mic=%d",
			    (events[i] & CS42L84_JACK_HEADPHONE) != 0,
			    (events[i] & CS42L84_JACK_MIC) != 0);
			devctl_notify("CS42L84", device_get_nameunit(sc->dev),
			    "JACK", data);
		}
	}
	if (cb == NULL)
		return;
	for (i = 0; i < n; i++)
		cb(arg, events[i]);
}

/* dev.cs42l84.N.headphone / .mic: arg2 is the CS42L84_JACK_* bit. */
static int
cs42l84_jack_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct cs42l84_softc *sc = arg1;
	int val;

	sx_xlock(&sc->lock);
	val = (sc->hs_type & arg2) != 0;
	sx_xunlock(&sc->lock);
	return (sysctl_handle_int(oidp, &val, 0, req));
}

/*
 * Headset detection (Linux cs42l84_detect_hs, cs42l84_revert_hs).
 */

static void
cs42l84_remote_start(struct cs42l84_softc *sc)
{

	sc->remote_active = true;
	/* The first poll only takes a baseline. */
	sc->remote_btn1_prev = CS42L84_REMOTE_VOLUME_DOWN |
	    CS42L84_REMOTE_VOLUME_UP;
	sc->remote_short_prev = CS42L84_HS_DET_STATUS2_SHORT_TRUE;
}

static void
cs42l84_detect_hs(struct cs42l84_softc *sc)
{
	uint8_t reg;

	sc->remote_active = false;

	/* Power up HSBIAS: 2.7 V, detect mode 0 */
	cs42l84_update(sc, CS42L84_MISC_DET_CTL,
	    CS42L84_MISC_DET_CTL_HSBIAS_CTL_MASK |
	    CS42L84_MISC_DET_CTL_DETECT_MODE_MASK,
	    3 << CS42L84_MISC_DET_CTL_HSBIAS_CTL_SHIFT);
	/* Power up level detection circuitry */
	cs42l84_update(sc, CS42L84_MISC_DET_CTL,
	    CS42L84_MISC_DET_CTL_PDN_MIC_LVL_DET, 0);
	pause_sbt("cs42hs", mstosbt(50), 0, 0);

	/* Connect HSBIAS in CTIA wiring */
	cs42l84_write(sc, CS42L84_HS_SWITCH_CTL,
	    CS42L84_HS_SWITCH_CTL_REF_HS3 | CS42L84_HS_SWITCH_CTL_HSB_FILT_HS3 |
	    CS42L84_HS_SWITCH_CTL_GNDHS_HS3 | CS42L84_HS_SWITCH_CTL_HSB_HS4);
	cs42l84_update(sc, CS42L84_HS_DET_CTL2, CS42L84_HS_DET_CTL2_SET_MASK,
	    0);
	cs42l84_update(sc, CS42L84_MISC_DET_CTL,
	    CS42L84_MISC_DET_CTL_DETECT_MODE_MASK,
	    3 << CS42L84_MISC_DET_CTL_DETECT_MODE_SHIFT);
	pause_sbt("cs42hs", mstosbt(50), 0, 0);

	reg = 0;
	cs42l84_read(sc, CS42L84_HS_DET_STATUS2, &reg);
	cs42l84_update(sc, CS42L84_MISC_DET_CTL,
	    CS42L84_MISC_DET_CTL_PDN_MIC_LVL_DET,
	    CS42L84_MISC_DET_CTL_PDN_MIC_LVL_DET);

	switch (reg & 0x3) {
	case 0x2:	/* load: a mic */
		sc->hs_type = CS42L84_JACK_HEADPHONE | CS42L84_JACK_MIC;
		cs42l84_remote_start(sc);
		break;
	case 0x3:	/* shorted */
	case 0x0:	/* open */
	default:
		/* Power down HSBIAS: 0.0 V */
		cs42l84_update(sc, CS42L84_MISC_DET_CTL,
		    CS42L84_MISC_DET_CTL_HSBIAS_CTL_MASK,
		    1 << CS42L84_MISC_DET_CTL_HSBIAS_CTL_SHIFT);
		sc->hs_type = CS42L84_JACK_HEADPHONE;
		break;
	}
	if (bootverbose)
		device_printf(sc->dev, "detected %s (HS_DET_STATUS2 0x%02x)\n",
		    (sc->hs_type & CS42L84_JACK_MIC) ? "headset" : "headphone",
		    reg);
	cs42l84_report(sc, sc->hs_type);
}

static void
cs42l84_revert_hs(struct cs42l84_softc *sc)
{

	sc->remote_active = false;

	/* Power down HSBIAS */
	cs42l84_update(sc, CS42L84_MISC_DET_CTL,
	    CS42L84_MISC_DET_CTL_HSBIAS_CTL_MASK |
	    CS42L84_MISC_DET_CTL_DETECT_MODE_MASK,
	    1 << CS42L84_MISC_DET_CTL_HSBIAS_CTL_SHIFT);
	/* Disconnect HSBIAS */
	cs42l84_write(sc, CS42L84_HS_SWITCH_CTL,
	    CS42L84_HS_SWITCH_CTL_REF_HS3 | CS42L84_HS_SWITCH_CTL_REF_HS4 |
	    CS42L84_HS_SWITCH_CTL_HSB_FILT_HS3 |
	    CS42L84_HS_SWITCH_CTL_HSB_FILT_HS4 |
	    CS42L84_HS_SWITCH_CTL_GNDHS_HS3 | CS42L84_HS_SWITCH_CTL_GNDHS_HS4);
	cs42l84_update(sc, CS42L84_HS_DET_CTL2, CS42L84_HS_DET_CTL2_SET_MASK,
	    2 << CS42L84_HS_DET_CTL2_SET_SHIFT);
}

static void
cs42l84_set_interrupt_masks(struct cs42l84_softc *sc, uint8_t val)
{

	cs42l84_update(sc, CS42L84_TSRS_PLUG_INT_MASK,
	    CS42L84_RS_PLUG | CS42L84_RS_UNPLUG | CS42L84_TS_PLUG |
	    CS42L84_TS_UNPLUG, val);
}

static void
cs42l84_unplugged(struct cs42l84_softc *sc)
{

	cs42l84_revert_hs(sc);
	sc->hs_type = 0;
	cs42l84_report(sc, 0);
}

/*
 * Linux cs42l84_irq_thread(), called with the lock held.  Reading the
 * sticky status clears it; the plug decisions come from TSRS_PLUG_STATUS
 * compared with the saved state, so running it from a poll is equivalent.
 */
static void
cs42l84_irq_handle(struct cs42l84_softc *sc)
{
	uint8_t sticky, mask, reg, tip, ring;

	sticky = mask = reg = 0;
	cs42l84_read(sc, CS42L84_TSRS_PLUG_INT_STATUS, &sticky);
	cs42l84_read(sc, CS42L84_TSRS_PLUG_INT_MASK, &mask);

	if (sc->ring_detect) {
		/* Clears the latched 0x1480 bits 5/6 sources. */
		cs42l84_read(sc, CS42L84_REMOTE_BUTTON_STATUS1, &reg);
		if (cs42l84_read(sc, CS42L84_TSRS_PLUG_STATUS, &reg) != 0)
			return;
		ring = (reg & (CS42L84_RS_PLUG | CS42L84_RS_UNPLUG)) >>
		    CS42L84_RS_PLUG_SHIFT;
		if (ring == sc->ring_state)
			return;
		sc->ring_state = ring;
		switch (ring) {
		case CS42L84_PLUG:
			sc->tip_state = CS42L84_PLUG;
			cs42l84_detect_hs(sc);
			break;
		case CS42L84_UNPLUG:
			sc->tip_state = CS42L84_UNPLUG;
			cs42l84_unplugged(sc);
			break;
		}
		return;
	}

	if ((~mask & CS42L84_TSRS_PLUG_VAL_MASK) == 0)
		return;
	if (cs42l84_read(sc, CS42L84_TSRS_PLUG_STATUS, &reg) != 0)
		return;
	tip = (reg & (CS42L84_TS_PLUG | CS42L84_TS_UNPLUG)) >>
	    CS42L84_TS_PLUG_SHIFT;

	if (tip != sc->tip_state) {
		sc->tip_state = tip;
		switch (tip) {
		case CS42L84_PLUG:
			cs42l84_detect_hs(sc);
			/*
			 * Thanks to debounce, a second read tells whether the
			 * tip was lost during detection.
			 */
			if (cs42l84_read(sc, CS42L84_TSRS_PLUG_STATUS,
			    &reg) == 0 &&
			    ((reg & (CS42L84_TS_PLUG | CS42L84_TS_UNPLUG)) >>
			    CS42L84_TS_PLUG_SHIFT) != CS42L84_PLUG) {
				device_printf(sc->dev, "wobbly connection, "
				    "detection invalidated\n");
				sc->tip_state = CS42L84_UNPLUG;
				cs42l84_revert_hs(sc);
			}
			/* Unmask ring sense interrupts */
			cs42l84_set_interrupt_masks(sc, 0);
			break;
		case CS42L84_UNPLUG:
			sc->ring_state = CS42L84_UNPLUG;
			cs42l84_unplugged(sc);
			/* Mask ring sense interrupts */
			cs42l84_set_interrupt_masks(sc,
			    CS42L84_RS_PLUG | CS42L84_RS_UNPLUG);
			break;
		default:
			sc->ring_state = CS42L84_TRANS;
			break;
		}
		return;
	}

	/* Tip state didn't change: a ring sense event. */
	ring = (reg & (CS42L84_RS_PLUG | CS42L84_RS_UNPLUG)) >>
	    CS42L84_RS_PLUG_SHIFT;
	if (ring != sc->ring_state) {
		sc->ring_state = ring;
		if (ring == CS42L84_PLUG)
			cs42l84_detect_hs(sc);
	}
}

/* Linux cs42l84_remote_poll(), with the lock held. */
static void
cs42l84_remote_poll(struct cs42l84_softc *sc)
{
	uint8_t btn1, st, short_bit;

	if (cs42l84_read(sc, CS42L84_REMOTE_BUTTON_STATUS1, &btn1) != 0 ||
	    cs42l84_read(sc, CS42L84_HS_DET_STATUS2, &st) != 0)
		return;
	short_bit = st & CS42L84_HS_DET_STATUS2_SHORT_TRUE;

	if ((btn1 & CS42L84_REMOTE_VOLUME_DOWN) &&
	    !(sc->remote_btn1_prev & CS42L84_REMOTE_VOLUME_DOWN))
		cs42l84_report(sc, sc->hs_type | CS42L84_JACK_BTN_VOLDN);
	if ((btn1 & CS42L84_REMOTE_VOLUME_UP) &&
	    !(sc->remote_btn1_prev & CS42L84_REMOTE_VOLUME_UP))
		cs42l84_report(sc, sc->hs_type | CS42L84_JACK_BTN_VOLUP);
	if (short_bit && !sc->remote_short_prev)
		cs42l84_report(sc, sc->hs_type | CS42L84_JACK_BTN_PLAY);
	sc->remote_btn1_prev = btn1;
	sc->remote_short_prev = short_bit;
}

static void
cs42l84_schedule(struct cs42l84_softc *sc)
{
	int ms;

	sx_assert(&sc->lock, SA_XLOCKED);
	if (sc->detaching)
		return;
	if (sc->remote_active)
		ms = CS42L84_REMOTE_POLL_MS;
	else if (sc->poll_plug)
		ms = CS42L84_PLUG_POLL_MS;
	else
		return;
	taskqueue_enqueue_timeout_sbt(taskqueue_thread, &sc->poll_task,
	    mstosbt(ms), 0, C_PREL(2));
}

static void
cs42l84_poll(void *arg, int pending __unused)
{
	struct cs42l84_softc *sc = arg;

	sx_xlock(&sc->lock);
	if (sc->detaching) {
		sx_xunlock(&sc->lock);
		return;
	}
	if (sc->poll_plug)
		cs42l84_irq_handle(sc);
	if (sc->remote_active)
		cs42l84_remote_poll(sc);
	cs42l84_schedule(sc);
	cs42l84_unlock_deliver(sc);
}

/*
 * Plug detect setup (Linux cs42l84_setup_plug_detect,
 * cs42l84_ring_detect_init, cs42l84_ring_detect_setup).
 */

static void
cs42l84_setup_plug_detect(struct cs42l84_softc *sc)
{
	uint8_t reg;

	cs42l84_update(sc, CS42L84_MIC_DET_CTL4,
	    CS42L84_MIC_DET_CTL4_LATCH_TO_VP, CS42L84_MIC_DET_CTL4_LATCH_TO_VP);
	cs42l84_update(sc, CS42L84_TIP_SENSE_CTL2,
	    CS42L84_TIP_SENSE_CTL2_MODE_MASK,
	    CS42L84_TIP_SENSE_CTL2_MODE_SHORT_DET);
	cs42l84_update(sc, CS42L84_RING_SENSE_CTL,
	    CS42L84_RING_SENSE_CTL_INV | CS42L84_RING_SENSE_CTL_UNK1 |
	    CS42L84_RING_SENSE_CTL_RISETIME_MASK |
	    CS42L84_RING_SENSE_CTL_FALLTIME_MASK,
	    CS42L84_RING_SENSE_CTL_INV | CS42L84_RING_SENSE_CTL_UNK1 |
	    CS42L84_DEBOUNCE_TIME_125MS |
	    (CS42L84_DEBOUNCE_TIME_125MS <<
	    CS42L84_RING_SENSE_CTL_FALLTIME_SHIFT));
	cs42l84_update(sc, CS42L84_TIP_SENSE_CTL,
	    CS42L84_TIP_SENSE_CTL_INV | CS42L84_TIP_SENSE_CTL_RISETIME_MASK |
	    CS42L84_TIP_SENSE_CTL_FALLTIME_MASK,
	    CS42L84_TIP_SENSE_CTL_INV | CS42L84_DEBOUNCE_TIME_500MS |
	    (CS42L84_DEBOUNCE_TIME_125MS <<
	    CS42L84_TIP_SENSE_CTL_FALLTIME_SHIFT));
	cs42l84_update(sc, CS42L84_MSM_BLOCK_EN3,
	    CS42L84_MSM_BLOCK_EN3_TR_SENSE, CS42L84_MSM_BLOCK_EN3_TR_SENSE);

	/* Save the initial status of the tip sense */
	reg = 0;
	cs42l84_read(sc, CS42L84_TSRS_PLUG_STATUS, &reg);
	sc->tip_state = (reg & (CS42L84_TS_PLUG | CS42L84_TS_UNPLUG)) >>
	    CS42L84_TS_PLUG_SHIFT;

	/* Mic-detection threshold, ~1.9 V */
	cs42l84_update(sc, CS42L84_MIC_DET_CTL1,
	    CS42L84_MIC_DET_CTL1_HS_DET_LEVEL_MASK, 0x2c);

	/* Disconnect HSBIAS (initially) */
	cs42l84_write(sc, CS42L84_HS_SWITCH_CTL,
	    CS42L84_HS_SWITCH_CTL_REF_HS3 | CS42L84_HS_SWITCH_CTL_REF_HS4 |
	    CS42L84_HS_SWITCH_CTL_HSB_FILT_HS3 |
	    CS42L84_HS_SWITCH_CTL_HSB_FILT_HS4 |
	    CS42L84_HS_SWITCH_CTL_GNDHS_HS3 | CS42L84_HS_SWITCH_CTL_GNDHS_HS4);
	cs42l84_update(sc, CS42L84_HS_DET_CTL2,
	    CS42L84_HS_DET_CTL2_SET_MASK | CS42L84_HS_DET_CTL2_CTL_MASK,
	    2 << CS42L84_HS_DET_CTL2_SET_SHIFT);
	cs42l84_update(sc, CS42L84_HS_CLAMP_DISABLE, 1, 1);
}

static int
cs42l84_ring_detect_init(struct cs42l84_softc *sc)
{
	int error;

	error = cs42l84_write(sc, CS42L84_TIP_SENSE_CTL2,
	    CS42L84_TIP_SENSE_CTL2_MODE_SHORT_DET);
	if (error == 0)
		error = cs42l84_write_bits(sc, CS42L84_DET_UNK1471, 0x20, 0x20);
	if (error == 0)
		error = cs42l84_write(sc, CS42L84_DET_UNK1478, 0x8f);
	if (error == 0)
		error = cs42l84_write(sc, CS42L84_DET_UNK1480, 0x80);
	return (error);
}

/* Linux cs42l84_component_probe(). */
static void
cs42l84_component_init(struct cs42l84_softc *sc)
{

	cs42l84_update(sc, CS42L84_ASP_CTL, CS42L84_ASP_CTL_TDM_MODE, 0);
	cs42l84_update(sc, CS42L84_HP_VOL_CTL,
	    CS42L84_HP_VOL_CTL_SOFT | CS42L84_HP_VOL_CTL_ZERO_CROSS,
	    CS42L84_HP_VOL_CTL_ZERO_CROSS);

	/* TDM settings */
	cs42l84_update(sc, CS42L84_ASP_RX_CH1_CTL1,
	    CS42L84_ASP_CHx_CTL1_EDGE | CS42L84_ASP_CHx_CTL1_SLOT_START_LSB, 0);
	cs42l84_update(sc, CS42L84_ASP_RX_CH1_CTL2,
	    CS42L84_ASP_CHx_CTL2_SLOT_START_MSB, 0);
	cs42l84_update(sc, CS42L84_ASP_RX_CH2_CTL1,
	    CS42L84_ASP_CHx_CTL1_EDGE | CS42L84_ASP_CHx_CTL1_SLOT_START_LSB,
	    CS42L84_ASP_CHx_CTL1_EDGE);
	cs42l84_update(sc, CS42L84_ASP_RX_CH2_CTL2,
	    CS42L84_ASP_CHx_CTL2_SLOT_START_MSB, 0);
	cs42l84_update(sc, CS42L84_ASP_TX_CH1_CTL1,
	    CS42L84_ASP_CHx_CTL1_EDGE | CS42L84_ASP_CHx_CTL1_SLOT_START_LSB, 0);
	cs42l84_update(sc, CS42L84_ASP_TX_CH1_CTL2,
	    CS42L84_ASP_CHx_CTL2_SLOT_START_MSB, 0);
	cs42l84_update(sc, CS42L84_ASP_TX_CH2_CTL1,
	    CS42L84_ASP_CHx_CTL1_EDGE | CS42L84_ASP_CHx_CTL1_SLOT_START_LSB,
	    CS42L84_ASP_CHx_CTL1_EDGE);
	cs42l84_update(sc, CS42L84_ASP_TX_CH2_CTL2,
	    CS42L84_ASP_CHx_CTL2_SLOT_START_MSB, 0);

	/* Routing defaults: DACA <- ASP RX CH1, DACB <- RX CH2, TX CH1 <- ADC */
	cs42l84_write(sc, CS42L84_BUS_DAC_SRC,
	    CS42L84_SRC_ASP_RX_CH1 | (CS42L84_SRC_ASP_RX_CH2 << 4));
	cs42l84_write(sc, CS42L84_BUS_ASP_TX_SRC, CS42L84_SRC_ADC);
}

/*
 * Volume (Linux cs42l84_put_dac_vol, limited to 0 dB).
 */

static int
cs42l84_vol_to_reg(u_int vol)
{

	if (vol == 0)
		return (CS42L84_DAC_VOL_MUTE);
	if (vol > CS42L84_VOL_MAX)
		vol = CS42L84_VOL_MAX;
	/* 0.6 dB per step = 1.2 half-dB, rounded */
	return (-(int)(((CS42L84_VOL_MAX - vol) * 6 + 2) / 5));
}

static int
cs42l84_write_volume(struct cs42l84_softc *sc, u_int left, u_int right)
{
	int vola, volb, error, error2;

	vola = cs42l84_vol_to_reg(left);
	volb = cs42l84_vol_to_reg(right);

	error = cs42l84_update(sc, CS42L84_FRZ_CTL, CS42L84_FRZ_CTL_ENGAGE,
	    CS42L84_FRZ_CTL_ENGAGE);
	if (error == 0)
		error = cs42l84_update(sc, CS42L84_DAC_CHA_VOL_LSB, 0xff,
		    vola & 0xff);
	if (error == 0)
		error = cs42l84_update(sc, CS42L84_DAC_CHA_VOL_MSB, 0xff,
		    (vola >> 8) & 0x01);
	if (error == 0)
		error = cs42l84_update(sc, CS42L84_DAC_CHB_VOL_LSB, 0xff,
		    volb & 0xff);
	if (error == 0)
		error = cs42l84_update(sc, CS42L84_DAC_CHB_VOL_MSB, 0xff,
		    (volb >> 8) & 0x01);
	error2 = cs42l84_update(sc, CS42L84_FRZ_CTL, CS42L84_FRZ_CTL_ENGAGE,
	    0);
	return (error != 0 ? error : error2);
}

/*
 * Clocking (Linux cs42l84_pll_config, cs42l84_pcm_hw_params,
 * cs42l84_mute_stream).
 */

static int
cs42l84_pll_config(struct cs42l84_softc *sc)
{
	const struct cs42l84_pll_params *p;
	uint32_t clk, fsync;
	int i;

	clk = sc->bclk;
	if (sc->stream_use != 0)
		return (pll_ratio_table[sc->pll_config].bclk == clk ? 0 : EBUSY);

	for (i = 0; i < nitems(pll_ratio_table); i++)
		if (pll_ratio_table[i].bclk == clk)
			break;
	if (i == nitems(pll_ratio_table))
		return (EINVAL);
	sc->pll_config = i;
	p = &pll_ratio_table[i];

	fsync = clk / sc->srate;
	if (fsync * sc->srate != clk || (fsync % 2) != 0) {
		device_printf(sc->dev, "unsupported bclk %u/sample rate %u\n",
		    clk, sc->srate);
		return (EINVAL);
	}

	/* The LRCLK period */
	cs42l84_update(sc, CS42L84_ASP_FSYNC_CTL2,
	    CS42L84_ASP_FSYNC_CTL2_BCLK_PERIOD_LO, (fsync & 0x7f) << 1);
	cs42l84_update(sc, CS42L84_ASP_FSYNC_CTL3,
	    CS42L84_ASP_FSYNC_CTL3_BCLK_PERIOD_HI, fsync >> 7);

	switch (p->mclk_int) {
	case 12000000:
		sc->pll_mclk_f = CS42L84_CCM_CTL1_MCLK_F_12MHZ;
		break;
	case 11289600:
	case 12288000:
		sc->pll_mclk_f = CS42L84_CCM_CTL1_MCLK_F_12_288KHZ;
		break;
	case 24000000:
		sc->pll_mclk_f = CS42L84_CCM_CTL1_MCLK_F_24MHZ;
		break;
	case 24576000:
		sc->pll_mclk_f = CS42L84_CCM_CTL1_MCLK_F_24_576KHZ;
		break;
	}

	cs42l84_update(sc, CS42L84_PLL_CTL1, CS42L84_PLL_CTL1_EN, 0);
	if (p->mclk_src_sel) {
		cs42l84_update(sc, CS42L84_CCM_CTL3,
		    CS42L84_CCM_CTL3_REFCLK_DIV_MASK,
		    p->bclk_prediv << CS42L84_CCM_CTL3_REFCLK_DIV_SHIFT);
		cs42l84_write(sc, CS42L84_PLL_DIV_INT, p->pll_div_int);
		cs42l84_write(sc, CS42L84_PLL_DIV_FRAC0, p->pll_div_frac & 0xff);
		cs42l84_write(sc, CS42L84_PLL_DIV_FRAC1,
		    (p->pll_div_frac >> 8) & 0xff);
		cs42l84_write(sc, CS42L84_PLL_DIV_FRAC2,
		    (p->pll_div_frac >> 16) & 0xff);
		cs42l84_update(sc, CS42L84_PLL_CTL1, CS42L84_PLL_CTL1_MODE_MASK,
		    p->pll_mode << CS42L84_PLL_CTL1_MODE_SHIFT);
		cs42l84_write(sc, CS42L84_PLL_DIVOUT, p->pll_divout);
	}
	return (0);
}

static void
cs42l84_set_slot_start(struct cs42l84_softc *sc, uint16_t ctl1,
    uint16_t ctl2, u_int bit)
{

	cs42l84_update(sc, ctl1, CS42L84_ASP_CHx_CTL1_SLOT_START_LSB,
	    (bit & 0x7f) << 1);
	cs42l84_update(sc, ctl2, CS42L84_ASP_CHx_CTL2_SLOT_START_MSB,
	    bit >> 7);
}

/* The DAC unmute, unless the user mute is on. */
static int
cs42l84_dac_unmute(struct cs42l84_softc *sc)
{

	return (cs42l84_update(sc, CS42L84_DAC_CTL1, CS42L84_DAC_CTL1_UNMUTE,
	    sc->mute ? 0 : CS42L84_DAC_CTL1_UNMUTE));
}

/* Linux cs42l84_mute_stream(dai, 0, stream). */
static int
cs42l84_stream_unmute(struct cs42l84_softc *sc, int dir)
{
	uint8_t val;
	int elapsed, error;

	if (sc->stream_use == 0) {
		/*
		 * BCLK must be running before codec unmute.  The PLL must
		 * not be started with ADC and HP both off (FILT+ would not
		 * charge): the DAC or ADC is already up here.
		 */
		cs42l84_update(sc, CS42L84_CCM_CTL4, CS42L84_CCM_CTL4_REFCLK_EN,
		    CS42L84_CCM_CTL4_REFCLK_EN);
		if (pll_ratio_table[sc->pll_config].mclk_src_sel) {
			cs42l84_update(sc, CS42L84_PLL_CTL1,
			    CS42L84_PLL_CTL1_EN, CS42L84_PLL_CTL1_EN);
			val = 0;
			error = ETIMEDOUT;
			for (elapsed = 0; elapsed <= CS42L84_PLL_LOCK_TIMEOUT_US;
			    elapsed += CS42L84_PLL_LOCK_POLL_US) {
				if (cs42l84_read(sc, CS42L84_PLL_LOCK_STATUS,
				    &val) == 0 &&
				    (val & CS42L84_PLL_LOCK_STATUS_LOCKED)) {
					error = 0;
					break;
				}
				DELAY(CS42L84_PLL_LOCK_POLL_US);
			}
			if (error != 0) {
				/*
				 * Without a locked PLL the codec has no clock
				 * off the oscillator and I2C stops working:
				 * stay on the RCO and undo.
				 */
				device_printf(sc->dev, "PLL failed to lock "
				    "(0x%02x)\n", val);
				cs42l84_update(sc, CS42L84_PLL_CTL1,
				    CS42L84_PLL_CTL1_EN, 0);
				cs42l84_update(sc, CS42L84_CCM_CTL4,
				    CS42L84_CCM_CTL4_REFCLK_EN, 0);
				return (error);
			}
			if (val & CS42L84_PLL_LOCK_STATUS_ERROR)
				device_printf(sc->dev, "PLL lock error\n");
			/* PLL must run to drive the glitchless switch */
			cs42l84_update(sc, CS42L84_CCM_CTL1,
			    CS42L84_CCM_CTL1_MCLK_SRC_MASK |
			    CS42L84_CCM_CTL1_MCLK_FREQ_MASK,
			    CS42L84_CCM_CTL1_MCLK_SRC_PLL |
			    (sc->pll_mclk_f << CS42L84_CCM_CTL1_MCLK_FREQ_SHIFT));
		} else {
			cs42l84_update(sc, CS42L84_CCM_CTL1,
			    CS42L84_CCM_CTL1_MCLK_SRC_MASK |
			    CS42L84_CCM_CTL1_MCLK_FREQ_MASK,
			    CS42L84_CCM_CTL1_MCLK_SRC_BCLK |
			    (sc->pll_mclk_f << CS42L84_CCM_CTL1_MCLK_FREQ_SHIFT));
		}
		DELAY(CS42L84_CLOCK_SWITCH_DELAY_US);
	}
	sc->stream_use |= 1 << dir;

	if (dir == CS42L84_DIR_PLAYBACK)
		return (cs42l84_dac_unmute(sc));
	return (0);
}

/* Linux cs42l84_mute_stream(dai, 1, stream). */
static void
cs42l84_stream_mute(struct cs42l84_softc *sc, int dir)
{

	if (dir == CS42L84_DIR_PLAYBACK)
		cs42l84_update(sc, CS42L84_DAC_CTL1, CS42L84_DAC_CTL1_UNMUTE, 0);
	sc->stream_use &= ~(1 << dir);
	if (sc->stream_use == 0) {
		/* Must disconnect the PLL before stopping it */
		cs42l84_write(sc, CS42L84_CCM_CTL1, CS42L84_CCM_CTL1_RCO);
		DELAY(150);
		cs42l84_update(sc, CS42L84_PLL_CTL1, CS42L84_PLL_CTL1_EN, 0);
		cs42l84_update(sc, CS42L84_CCM_CTL4, CS42L84_CCM_CTL4_REFCLK_EN,
		    0);
	}
}

/*
 * The DAPM graph, flattened.  Playback: HP <- DAC <- DACA/DACB Select <-
 * SDIN1/SDIN2 <- Playback, with DAC needing BUS and SDINx needing ASP and
 * BCLK.  Capture: Capture <- SDOUT1 <- SDOUT1 Select (ADC) <- ADC <- HS,
 * SDOUT1 needing BUS, ASP and BCLK.  DAPM powers supplies first, then the
 * AIF widgets, then the DAC/ADC, and the reverse on the way down; the
 * supplies are shared by both directions.
 */
static void
cs42l84_dapm(struct cs42l84_softc *sc, int dir, bool on)
{
	bool other;

	other = sc->path_on[dir == CS42L84_DIR_PLAYBACK ?
	    CS42L84_DIR_CAPTURE : CS42L84_DIR_PLAYBACK];

	if (on) {
		/* Supplies: BUS, ASP (MSM_BLOCK_EN2), BCLK (ASP_CTL) */
		cs42l84_update(sc, CS42L84_MSM_BLOCK_EN2,
		    CS42L84_MSM_BLOCK_EN2_BUS | CS42L84_MSM_BLOCK_EN2_ASP,
		    CS42L84_MSM_BLOCK_EN2_BUS | CS42L84_MSM_BLOCK_EN2_ASP);
		cs42l84_update(sc, CS42L84_ASP_CTL, CS42L84_ASP_CTL_BCLK_EN,
		    CS42L84_ASP_CTL_BCLK_EN);
		if (dir == CS42L84_DIR_PLAYBACK) {
			cs42l84_update(sc, CS42L84_ASP_RX_EN,
			    CS42L84_ASP_RX_EN_CH1 | CS42L84_ASP_RX_EN_CH2,
			    CS42L84_ASP_RX_EN_CH1 | CS42L84_ASP_RX_EN_CH2);
			cs42l84_update(sc, CS42L84_MSM_BLOCK_EN2,
			    CS42L84_MSM_BLOCK_EN2_DAC, CS42L84_MSM_BLOCK_EN2_DAC);
		} else {
			cs42l84_update(sc, CS42L84_ASP_TX_EN,
			    CS42L84_ASP_TX_EN_CH1, CS42L84_ASP_TX_EN_CH1);
			cs42l84_update(sc, CS42L84_MSM_BLOCK_EN2,
			    CS42L84_MSM_BLOCK_EN2_ADC, CS42L84_MSM_BLOCK_EN2_ADC);
		}
	} else {
		if (dir == CS42L84_DIR_PLAYBACK) {
			cs42l84_update(sc, CS42L84_MSM_BLOCK_EN2,
			    CS42L84_MSM_BLOCK_EN2_DAC, 0);
			cs42l84_update(sc, CS42L84_ASP_RX_EN,
			    CS42L84_ASP_RX_EN_CH1 | CS42L84_ASP_RX_EN_CH2, 0);
		} else {
			cs42l84_update(sc, CS42L84_MSM_BLOCK_EN2,
			    CS42L84_MSM_BLOCK_EN2_ADC, 0);
			cs42l84_update(sc, CS42L84_ASP_TX_EN,
			    CS42L84_ASP_TX_EN_CH1, 0);
		}
		if (!other) {
			cs42l84_update(sc, CS42L84_MSM_BLOCK_EN2,
			    CS42L84_MSM_BLOCK_EN2_BUS |
			    CS42L84_MSM_BLOCK_EN2_ASP, 0);
			cs42l84_update(sc, CS42L84_ASP_CTL,
			    CS42L84_ASP_CTL_BCLK_EN, 0);
		}
	}
}

/*
 * Exported interface (cs42l84.h).
 */

int
cs42l84_set_fmt(device_t dev, int fmt)
{
	struct cs42l84_softc *sc = device_get_softc(dev);
	uint8_t ch2_edge;

	switch (fmt) {
	case CS42L84_FMT_I2S:
	case CS42L84_FMT_DSP_A:
		break;
	default:
		return (EINVAL);
	}
	sx_xlock(&sc->lock);
	sc->dsp_a = (fmt == CS42L84_FMT_DSP_A);
	ch2_edge = sc->dsp_a ? 0 : CS42L84_ASP_CHx_CTL1_EDGE;
	cs42l84_update(sc, CS42L84_ASP_CTL, CS42L84_ASP_CTL_TDM_MODE,
	    sc->dsp_a ? CS42L84_ASP_CTL_TDM_MODE : 0);
	cs42l84_update(sc, CS42L84_ASP_RX_CH2_CTL1, CS42L84_ASP_CHx_CTL1_EDGE,
	    ch2_edge);
	cs42l84_update(sc, CS42L84_ASP_TX_CH2_CTL1, CS42L84_ASP_CHx_CTL1_EDGE,
	    ch2_edge);
	sx_xunlock(&sc->lock);
	return (0);
}

int
cs42l84_set_sysclk(device_t dev, uint32_t bclk)
{
	struct cs42l84_softc *sc = device_get_softc(dev);
	int i;

	sx_xlock(&sc->lock);
	if (bclk == 0) {
		sc->bclk = 0;
		sx_xunlock(&sc->lock);
		return (0);
	}
	for (i = 0; i < nitems(pll_ratio_table); i++) {
		if (pll_ratio_table[i].bclk == bclk) {
			sc->bclk = bclk;
			sx_xunlock(&sc->lock);
			return (0);
		}
	}
	sx_xunlock(&sc->lock);
	device_printf(dev, "BCLK %u not supported\n", bclk);
	return (EINVAL);
}

int
cs42l84_hw_params(device_t dev, int dir, uint32_t rate, uint32_t width,
    uint32_t slot_width)
{
	struct cs42l84_softc *sc = device_get_softc(dev);
	uint8_t ccm;
	u_int ch2_start;
	int error;

	switch (rate) {
	case 44100:
		ccm = 12;
		break;
	case 48000:
		ccm = 4;
		break;
	case 88200:
		ccm = 13;
		break;
	case 96000:
		ccm = 5;
		break;
	case 176400:
		ccm = 14;
		break;
	case 192000:
		ccm = 6;
		break;
	default:
		return (EINVAL);
	}
	if ((width != 16 && width != 24 && width != 32) || slot_width < width)
		return (EINVAL);
	if (dir != CS42L84_DIR_PLAYBACK && dir != CS42L84_DIR_CAPTURE)
		return (EINVAL);

	sx_xlock(&sc->lock);
	sc->srate = rate;
	if ((error = cs42l84_pll_config(sc)) != 0) {
		sx_xunlock(&sc->lock);
		return (error);
	}
	cs42l84_write(sc, CS42L84_CCM_SAMP_RATE, ccm);

	/* DSP_A: channel 2 starts one slot after channel 1 */
	ch2_start = sc->dsp_a ? slot_width : 0;
	if (dir == CS42L84_DIR_PLAYBACK) {
		cs42l84_write(sc, CS42L84_ASP_RX_CH1_WIDTH, width - 1);
		cs42l84_write(sc, CS42L84_ASP_RX_CH2_WIDTH, width - 1);
		cs42l84_set_slot_start(sc, CS42L84_ASP_RX_CH2_CTL1,
		    CS42L84_ASP_RX_CH2_CTL2, ch2_start);
	} else {
		cs42l84_write(sc, CS42L84_ASP_TX_CH1_WIDTH, width - 1);
		cs42l84_write(sc, CS42L84_ASP_TX_CH2_WIDTH, width - 1);
		cs42l84_set_slot_start(sc, CS42L84_ASP_TX_CH2_CTL1,
		    CS42L84_ASP_TX_CH2_CTL2, ch2_start);
	}
	sx_xunlock(&sc->lock);
	return (0);
}

int
cs42l84_set_power(device_t dev, int dir, bool on)
{
	struct cs42l84_softc *sc = device_get_softc(dev);
	int error;

	if (dir != CS42L84_DIR_PLAYBACK && dir != CS42L84_DIR_CAPTURE)
		return (EINVAL);

	sx_xlock(&sc->lock);
	if (sc->path_on[dir] == on) {
		sx_xunlock(&sc->lock);
		return (0);
	}
	error = 0;
	if (on) {
		if (sc->bclk == 0 || sc->srate == 0) {
			sx_xunlock(&sc->lock);
			return (EINVAL);
		}
		if (dir == CS42L84_DIR_PLAYBACK) {
			/* Never power the output stage up loud. */
			if (sc->vol[0] > CS42L84_VOL_POWERUP_MAX ||
			    sc->vol[1] > CS42L84_VOL_POWERUP_MAX) {
				device_printf(dev, "volume limited to %d at "
				    "power-up\n", CS42L84_VOL_POWERUP_MAX);
				sc->vol[0] = min(sc->vol[0],
				    CS42L84_VOL_POWERUP_MAX);
				sc->vol[1] = min(sc->vol[1],
				    CS42L84_VOL_POWERUP_MAX);
			}
			error = cs42l84_write_volume(sc, sc->vol[0],
			    sc->vol[1]);
		}
		if (error == 0) {
			cs42l84_dapm(sc, dir, true);
			sc->path_on[dir] = true;
			error = cs42l84_stream_unmute(sc, dir);
			if (error != 0) {
				cs42l84_dapm(sc, dir, false);
				sc->path_on[dir] = false;
			}
		}
	} else {
		cs42l84_stream_mute(sc, dir);
		cs42l84_dapm(sc, dir, false);
		sc->path_on[dir] = false;
	}
	sx_xunlock(&sc->lock);
	return (error);
}

int
cs42l84_set_volume(device_t dev, u_int left, u_int right)
{
	struct cs42l84_softc *sc = device_get_softc(dev);
	int error;

	left = min(left, CS42L84_VOL_MAX);
	right = min(right, CS42L84_VOL_MAX);
	sx_xlock(&sc->lock);
	sc->vol[0] = left;
	sc->vol[1] = right;
	error = cs42l84_write_volume(sc, left, right);
	sx_xunlock(&sc->lock);
	return (error);
}

void
cs42l84_get_volume(device_t dev, u_int *left, u_int *right)
{
	struct cs42l84_softc *sc = device_get_softc(dev);

	sx_xlock(&sc->lock);
	*left = sc->vol[0];
	*right = sc->vol[1];
	sx_xunlock(&sc->lock);
}

int
cs42l84_set_mute(device_t dev, bool mute)
{
	struct cs42l84_softc *sc = device_get_softc(dev);
	int error;

	error = 0;
	sx_xlock(&sc->lock);
	sc->mute = mute;
	if (sc->stream_use & (1 << CS42L84_DIR_PLAYBACK))
		error = cs42l84_dac_unmute(sc);
	sx_xunlock(&sc->lock);
	return (error);
}

int
cs42l84_set_mic_gain(device_t dev, u_int preamp_db, int pga_half_db)
{
	struct cs42l84_softc *sc = device_get_softc(dev);
	int error;

	/*
	 * Linux "ADC Preamp Capture Volume": 0..2, 10 dB steps from 0;
	 * "ADC PGA Capture Volume": 0..24, 0.5 dB steps from -12 dB.
	 */
	if ((preamp_db != 0 && preamp_db != 10 && preamp_db != 20) ||
	    pga_half_db < -24 || pga_half_db > 0)
		return (EINVAL);
	sx_xlock(&sc->lock);
	error = cs42l84_update(sc, CS42L84_ADC_CTL1,
	    CS42L84_ADC_CTL1_PREAMP_GAIN_MASK | CS42L84_ADC_CTL1_PGA_GAIN_MASK,
	    ((preamp_db / 10) << CS42L84_ADC_CTL1_PREAMP_GAIN_SHIFT) |
	    ((pga_half_db + 24) & CS42L84_ADC_CTL1_PGA_GAIN_MASK));
	sx_xunlock(&sc->lock);
	return (error);
}

int
cs42l84_jack_status(device_t dev, bool *headphone, bool *mic)
{
	struct cs42l84_softc *sc = device_get_softc(dev);

	sx_xlock(&sc->lock);
	*headphone = (sc->hs_type & CS42L84_JACK_HEADPHONE) != 0;
	*mic = (sc->hs_type & CS42L84_JACK_MIC) != 0;
	sx_xunlock(&sc->lock);
	return (0);
}

void
cs42l84_set_jack_callback(device_t dev, cs42l84_jack_cb_t *cb, void *arg)
{
	struct cs42l84_softc *sc = device_get_softc(dev);

	sx_xlock(&sc->lock);
	sc->jack_cb = cb;
	sc->jack_cb_arg = arg;
	if (cb != NULL)
		cs42l84_report(sc, sc->hs_type);
	cs42l84_unlock_deliver(sc);
}

void
cs42l84_jack_event(device_t dev)
{
	struct cs42l84_softc *sc = device_get_softc(dev);
	bool was_active;

	sx_xlock(&sc->lock);
	if (sc->detaching) {
		sx_xunlock(&sc->lock);
		return;
	}
	was_active = sc->remote_active;
	cs42l84_irq_handle(sc);
	/* A new headset starts the remote poll, as remote_start() does. */
	if (sc->remote_active && !was_active)
		cs42l84_schedule(sc);
	cs42l84_unlock_deliver(sc);
}

void
cs42l84_set_poll(device_t dev, bool on)
{
	struct cs42l84_softc *sc = device_get_softc(dev);

	sx_xlock(&sc->lock);
	sc->poll_plug = on;
	if (on)
		cs42l84_schedule(sc);
	sx_xunlock(&sc->lock);
}

/*
 * Newbus.
 */

static int
cs42l84_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Cirrus Logic CS42L84 headphone codec");
	return (BUS_PROBE_DEFAULT);
}

static int
cs42l84_attach(device_t dev)
{
	struct cs42l84_softc *sc = device_get_softc(dev);
	phandle_t node;
	uint32_t devid;
	uint8_t rev, reg;
	int error;

	sc->dev = dev;
	node = ofw_bus_get_node(dev);
	sx_init(&sc->lock, "cs42l84");
	TIMEOUT_TASK_INIT(taskqueue_thread, &sc->poll_task, 0, cs42l84_poll,
	    sc);
	sc->vol[0] = sc->vol[1] = CS42L84_VOL_DEFAULT;

	/*
	 * Reset: Linux requests the line driven inactive (GPIOD_OUT_LOW),
	 * then sets it active and waits the boot time.  On this board the
	 * line is active-high, so "active" is high and releases the codec.
	 */
	error = gpio_pin_get_by_ofw_property(dev, node, "reset-gpios",
	    &sc->reset_gpio);
	if (error == 0) {
		gpio_pin_setflags(sc->reset_gpio, GPIO_PIN_OUTPUT);
		gpio_pin_set_active(sc->reset_gpio, false);
		DELAY(1000);
		gpio_pin_set_active(sc->reset_gpio, true);
	} else {
		sc->reset_gpio = NULL;
		if (error != ENOENT)
			device_printf(dev, "no reset GPIO: %d\n", error);
	}
	DELAY(CS42L84_BOOT_TIME_US);

	sx_xlock(&sc->lock);
	error = cs42l84_read_device_id(sc, &devid);
	if (error != 0) {
		device_printf(dev, "failed to read device ID: %d\n", error);
		goto fail;
	}
	if (devid != CS42L84_CHIP_ID) {
		device_printf(dev, "device ID 0x%x, expected 0x%x\n", devid,
		    CS42L84_CHIP_ID);
		error = ENXIO;
		goto fail;
	}
	if ((error = cs42l84_read(sc, CS42L84_REVID, &rev)) != 0)
		goto fail;
	device_printf(dev, "Cirrus Logic CS42L84, revision %02X\n", rev);

	sc->ring_detect = OF_hasprop(node, "cirrus,ring-sense-detect");
	if (sc->ring_detect) {
		if ((error = cs42l84_ring_detect_init(sc)) != 0)
			goto fail;
		/* What the detect setup unmasks comes up latched: clear it. */
		cs42l84_read(sc, CS42L84_REMOTE_BUTTON_STATUS1, &reg);
	}

	cs42l84_setup_plug_detect(sc);

	if (sc->ring_detect) {
		reg = 0;
		cs42l84_read(sc, CS42L84_TSRS_PLUG_STATUS, &reg);
		sc->ring_state = (reg & (CS42L84_RS_PLUG | CS42L84_RS_UNPLUG)) >>
		    CS42L84_RS_PLUG_SHIFT;
		cs42l84_set_interrupt_masks(sc,
		    CS42L84_TS_PLUG | CS42L84_TS_UNPLUG);
		if (sc->ring_state == CS42L84_PLUG) {
			sc->tip_state = CS42L84_PLUG;
			cs42l84_detect_hs(sc);
		} else
			sc->tip_state = CS42L84_UNPLUG;
	} else {
		/*
		 * Linux masks ring sense here and relies on a tip plug
		 * interrupt; a plug that is already in raises none until
		 * the poll's first pass, which sees tip_state == PLUG from
		 * setup_plug_detect() and so detects nothing.  Force that
		 * first pass to run the detection by starting unplugged.
		 */
		sc->tip_state = CS42L84_UNPLUG;
		sc->ring_state = CS42L84_UNPLUG;
		cs42l84_set_interrupt_masks(sc,
		    CS42L84_RS_PLUG | CS42L84_RS_UNPLUG);
	}

	/* ASoC component probe */
	cs42l84_component_init(sc);

	/* Our safe default volume, before anything can power the DAC. */
	cs42l84_update(sc, CS42L84_DAC_CTL1, CS42L84_DAC_CTL1_UNMUTE, 0);
	cs42l84_write_volume(sc, sc->vol[0], sc->vol[1]);

	sc->poll_plug = true;
	cs42l84_schedule(sc);
	sc->nevents = 0;
	sx_xunlock(&sc->lock);

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "headphone",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc,
	    CS42L84_JACK_HEADPHONE, cs42l84_jack_sysctl, "I",
	    "Headphones plugged in");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "mic",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc,
	    CS42L84_JACK_MIC, cs42l84_jack_sysctl, "I",
	    "Headset with a microphone plugged in");
	OF_device_register_xref(OF_xref_from_node(node), dev);
	return (0);

fail:
	sx_xunlock(&sc->lock);
	if (sc->reset_gpio != NULL) {
		gpio_pin_set_active(sc->reset_gpio, false);
		gpio_pin_release(sc->reset_gpio);
	}
	sx_destroy(&sc->lock);
	return (error);
}

static int
cs42l84_detach(device_t dev)
{
	struct cs42l84_softc *sc = device_get_softc(dev);

	sx_xlock(&sc->lock);
	sc->detaching = true;
	sc->jack_cb = NULL;
	sx_xunlock(&sc->lock);
	while (taskqueue_cancel_timeout(taskqueue_thread, &sc->poll_task,
	    NULL) != 0)
		taskqueue_drain_timeout(taskqueue_thread, &sc->poll_task);

	sx_xlock(&sc->lock);
	if (sc->path_on[CS42L84_DIR_PLAYBACK]) {
		cs42l84_stream_mute(sc, CS42L84_DIR_PLAYBACK);
		cs42l84_dapm(sc, CS42L84_DIR_PLAYBACK, false);
		sc->path_on[CS42L84_DIR_PLAYBACK] = false;
	}
	if (sc->path_on[CS42L84_DIR_CAPTURE]) {
		cs42l84_stream_mute(sc, CS42L84_DIR_CAPTURE);
		cs42l84_dapm(sc, CS42L84_DIR_CAPTURE, false);
		sc->path_on[CS42L84_DIR_CAPTURE] = false;
	}
	sx_xunlock(&sc->lock);

	OF_device_register_xref(OF_xref_from_node(ofw_bus_get_node(dev)),
	    NULL);
	if (sc->reset_gpio != NULL) {
		gpio_pin_set_active(sc->reset_gpio, false);
		gpio_pin_release(sc->reset_gpio);
	}
	sx_destroy(&sc->lock);
	return (0);
}

static device_method_t cs42l84_methods[] = {
	DEVMETHOD(device_probe,		cs42l84_probe),
	DEVMETHOD(device_attach,	cs42l84_attach),
	DEVMETHOD(device_detach,	cs42l84_detach),
	DEVMETHOD_END
};

static driver_t cs42l84_driver = {
	"cs42l84",
	cs42l84_methods,
	sizeof(struct cs42l84_softc)
};

DRIVER_MODULE(cs42l84, iicbus, cs42l84_driver, NULL, NULL);
MODULE_DEPEND(cs42l84, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
MODULE_VERSION(cs42l84, 1);
IICBUS_FDT_PNP_INFO(compat_data);
