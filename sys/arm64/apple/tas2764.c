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
 * TI TAS2764 / SN012776 speaker amplifier (Apple silicon Macs).
 *
 * Follows aurora-silicon's Linux sound/soc/codecs/tas2764.c, tas2764.h and
 * tas2764-quirks.h, with the fixed controls macaudio.c sets on SN012776
 * amps (macaudio_set_speaker(), macaudio_limit_sn012776_levels()).  The
 * ASoC component probe, DAI ops and mute_stream are folded into one
 * power-up sequence (tas2764_power_on()), since every amp on the shared
 * shutdown (SDZ) line comes back from a power-on reset whenever the line
 * has been low.  The fault interrupt is polled while an amp is on: the
 * GPIO controller has no interrupt support yet.
 *
 * SAFETY.  There is no speakersafetyd: macaudio then holds the SN012776
 * volume at its lock ceiling, and its configured safe level is
 * safe_vol = -20 dB (macaudio_j815_cfg).  This driver clamps the digital
 * volume to -20 dB (TAS2764_VOL_CAP) and the amp gain to its lowest step
 * (TAS2764_AMP_GAIN_CAP) on every write, reads both back before the amp
 * leaves software shutdown, forces the tweeters' 800 Hz playback high-pass
 * (MACAUDIO_TWEETER_HPF_SAFE), and drops the shared SDZ line for good on
 * an over-temperature or over-current latch (Linux's fail-closed SDZ
 * group).
 *
 * Registers: 8-bit address within a 128-register page (register 0 of every
 * page selects the page), 8-bit values.  No cache: read-modify-write goes
 * to the chip.  One lock (tas2764_sx) covers every amp and the SDZ group:
 * the amps share one bus and one line, and the power sequence of one
 * changes the state of all.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/queue.h>
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

#include <arm64/apple/tas2764.h>

#include "iicbus_if.h"

/* Registers (Linux tas2764.h). */
#define	TAS2764_REG(page, reg)		((page) * 128 + (reg))
#define	TAS2764_PAGE			TAS2764_REG(0x0, 0x00)
#define	TAS2764_SW_RST			TAS2764_REG(0x0, 0x01)
#define	 TAS2764_RST			0x01
#define	TAS2764_PWR_CTRL		TAS2764_REG(0x0, 0x02)
#define	 TAS2764_PWR_CTRL_MASK		0x07
#define	 TAS2764_PWR_CTRL_ACTIVE	0x00
#define	 TAS2764_PWR_CTRL_MUTE		0x01
#define	 TAS2764_PWR_CTRL_SHUTDOWN	0x02
#define	 TAS2764_PWR_CTRL_RESET		0x1a	/* reset value */
/* TAS2764_VSENSE_POWER_EN / _ISENSE_POWER_EN: set = sense ADC down */
#define	 TAS2764_PWR_CTRL_VSNS_PD	0x08
#define	 TAS2764_PWR_CTRL_ISNS_PD	0x10
#define	 TAS2764_PWR_CTRL_BOP_SRC	0x80
#define	TAS2764_CHNL_0			TAS2764_REG(0x0, 0x03)
#define	 TAS2764_CHNL_0_GAIN_SHIFT	1
#define	 TAS2764_CHNL_0_GAIN_MASK	0x3e
#define	TAS2764_DC_BLK0			TAS2764_REG(0x0, 0x04)
#define	 TAS2764_DC_BLK0_HPF_MASK	0x07
#define	 TAS2764_HPF_800HZ		6	/* "800 Hz" in tas2764_hpf_texts */
#define	TAS2764_DC_BLK1			TAS2764_REG(0x0, 0x05)
#define	TAS2764_MISC_CFG1		TAS2764_REG(0x0, 0x06)
#define	 TAS2764_MISC_CFG1_OCE_RETRY	0x20
#define	TAS2764_TDM_CFG0		TAS2764_REG(0x0, 0x08)
#define	 TAS2764_TDM_CFG0_FIXED_RATIO	0x40
#define	 TAS2764_TDM_CFG0_SMP_MASK	0x20
#define	 TAS2764_TDM_CFG0_SMP_48KHZ	0x00
#define	 TAS2764_TDM_CFG0_SMP_44_1KHZ	0x20
#define	 TAS2764_TDM_CFG0_MASK		0x0e
#define	 TAS2764_TDM_CFG0_44_1_48KHZ	0x08
#define	 TAS2764_TDM_CFG0_88_2_96KHZ	0x0a
#define	 TAS2764_TDM_CFG0_FRAME_START	0x01
#define	TAS2764_TDM_CFG1		TAS2764_REG(0x0, 0x09)
#define	 TAS2764_TDM_CFG1_MASK		0x3e
#define	 TAS2764_TDM_CFG1_51_SHIFT	1
#define	 TAS2764_TDM_CFG1_RX_MASK	0x01
#define	 TAS2764_TDM_CFG1_RX_RISING	0x00
#define	 TAS2764_TDM_CFG1_RX_FALLING	0x01
#define	TAS2764_TDM_CFG2		TAS2764_REG(0x0, 0x0a)
#define	 TAS2764_TDM_CFG2_SCFG_MASK	0x30
#define	 TAS2764_TDM_CFG2_SCFG_LEFT	0x10	/* "ASI1 Sel" = "Left" */
#define	 TAS2764_TDM_CFG2_RXW_MASK	0x0c
#define	 TAS2764_TDM_CFG2_RXW_16BITS	0x00
#define	 TAS2764_TDM_CFG2_RXW_24BITS	0x08
#define	 TAS2764_TDM_CFG2_RXW_32BITS	0x0c
#define	 TAS2764_TDM_CFG2_RXS_MASK	0x03
#define	 TAS2764_TDM_CFG2_RXS_16BITS	0x00
#define	 TAS2764_TDM_CFG2_RXS_24BITS	0x01
#define	 TAS2764_TDM_CFG2_RXS_32BITS	0x02
#define	TAS2764_TDM_CFG3		TAS2764_REG(0x0, 0x0c)
#define	 TAS2764_TDM_CFG3_RXS_SHIFT	4
#define	TAS2764_TDM_CFG4		TAS2764_REG(0x0, 0x0d)
#define	 TAS2764_TDM_CFG4_TX_MASK	0x01
#define	 TAS2764_TDM_CFG4_TX_RISING	0x00
#define	 TAS2764_TDM_CFG4_TX_FALLING	0x01
#define	TAS2764_TDM_CFG5		TAS2764_REG(0x0, 0x0e)
#define	 TAS2764_TDM_CFG5_VSNS_ENABLE	0x40
#define	 TAS2764_TDM_CFG5_50_MASK	0x3f
#define	TAS2764_TDM_CFG6		TAS2764_REG(0x0, 0x0f)
#define	 TAS2764_TDM_CFG6_ISNS_ENABLE	0x40
#define	 TAS2764_TDM_CFG6_50_MASK	0x3f
#define	TAS2764_DVC			TAS2764_REG(0x0, 0x1a)
#define	 TAS2764_DVC_MAX		0xc9	/* register: mute; 0 is 0 dB */
#define	TAS2764_BOP_CFG0		TAS2764_REG(0x0, 0x1d)
#define	 TAS2764_BOP_CFG_LEN		24
#define	TAS2764_INT_MASK0		TAS2764_REG(0x0, 0x3b)
#define	TAS2764_INT_MASK1		TAS2764_REG(0x0, 0x3c)
#define	TAS2764_INT_MASK4		TAS2764_REG(0x0, 0x3d)
#define	TAS2764_INT_MASK2		TAS2764_REG(0x0, 0x40)
#define	TAS2764_INT_MASK3		TAS2764_REG(0x0, 0x41)
#define	TAS2764_INT_LTCH0		TAS2764_REG(0x0, 0x49)
#define	 TAS2764_INT_LTCH0_OT		0x01
#define	 TAS2764_INT_LTCH0_OC		0x02
#define	 TAS2764_INT_LTCH0_GROUP_FAULTS	(TAS2764_INT_LTCH0_OT | TAS2764_INT_LTCH0_OC)
#define	 TAS2764_INT_LTCH0_TDM_CLOCK	0x04
#define	TAS2764_TEMP			TAS2764_REG(0x0, 0x56)
#define	TAS2764_INT_CLK_CFG		TAS2764_REG(0x0, 0x5c)
#define	 TAS2764_INT_CLK_CFG_IRQZ_CLR	0x04
#define	TAS2764_CLK_CFG			TAS2764_REG(0x0, 0x60)
#define	TAS2764_SDOUT_HIZ_1		TAS2764_REG(0x1, 0x3d)
#define	TAS2764_SDOUT_HIZ_9		TAS2764_REG(0x1, 0x45)
#define	 TAS2764_SDOUT_HIZ_9_FORCE_0_EN	0x80

/* The hard limits, as DVC register values (the register counts down). */
CTASSERT(TAS2764_VOL_CAP <= TAS2764_VOL_DB(-20));
CTASSERT(TAS2764_AMP_GAIN_CAP == 0);
#define	TAS2764_DVC_REG(vol)		(TAS2764_DVC_MAX - (vol))
#define	TAS2764_DVC_REG_MIN		TAS2764_DVC_REG(TAS2764_VOL_CAP)

/* Linux: usleep_range(1000, 2000) after SDZ high and after the reset. */
#define	TAS2764_SDZ_SETTLE_US		2000
#define	TAS2764_RESET_US		2000
/* tas2764_mute(): ramp-down, then the wait after shutdown */
#define	TAS2764_RAMP_DOWN_MS		7
#define	TAS2764_SHUTDOWN_MS		3
/* Fault latch poll while powered (no interrupt support) */
#define	TAS2764_POLL_MS			250

/* The fault latch names (Linux tas2764_int_ltch0_msgs). */
static const char *tas2764_ltch0_msgs[8] = {
	"fault: over temperature",
	"fault: over current",
	"fault: bad TDM clock",
	"limiter active",
	"fault: PVDD below limiter inflection point",
	"fault: limiter max attenuation",
	"fault: BOP infinite hold",
	"fault: BOP mute",
};

/*
 * The Apple init quirks (tas2764-quirks.h), in tas2764_quirk_init_sequences
 * order.  ENABLED_APPLE_QUIRKS is 0x3f there: the first six only (the
 * thermal threshold 1 and shutdown-dance sequences are not applied).
 */
struct tas2764_regval {
	uint16_t	reg;
	uint8_t		val;
};

static const struct tas2764_regval tas2764_init_quirks[] = {
	/* TAS2764_NOISE_GATE_DISABLE */
	{ TAS2764_REG(0x0, 0x35), 0xb0 },
	/* TAS2764_DMOD_RST */
	{ TAS2764_REG(0x0, 0x76), 0x00 },
	/* TAS2764_CONV_VBAT_PVDD_MODE */
	{ TAS2764_REG(0x0, 0x6b), 0x41 },
	/* TAS2764_UNK_SEQ0 */
	{ TAS2764_REG(0x1, 0x33), 0x80 },
	{ TAS2764_REG(0x1, 0x37), 0x3a },
	/* TAS2764_APPLE_UNK_SEQ1 */
	{ TAS2764_REG(0x6, 0x14), 0x00 },
	{ TAS2764_REG(0x6, 0x15), 0x13 },
	{ TAS2764_REG(0x6, 0x16), 0x52 },
	{ TAS2764_REG(0x6, 0x17), 0x00 },
	{ TAS2764_REG(0x6, 0x18), 0xe4 },
	{ TAS2764_REG(0x6, 0x19), 0x0c },
	{ TAS2764_REG(0x6, 0x16), 0xaa },
	{ TAS2764_REG(0x6, 0x1b), 0x00 },
	{ TAS2764_REG(0x6, 0x1c), 0x12 },
	{ TAS2764_REG(0x6, 0x1d), 0xa0 },
	{ TAS2764_REG(0x6, 0x1e), 0xd8 },
	{ TAS2764_REG(0x6, 0x1f), 0x00 },
	/* TAS2764_APPLE_UNK_SEQ2 (secondary paging in page 0xfd) */
	{ TAS2764_REG(0xfd, 0x0d), 0x0d },
	{ TAS2764_REG(0xfd, 0x6c), 0x02 },
	{ TAS2764_REG(0xfd, 0x6d), 0x0f },
	{ TAS2764_REG(0xfd, 0x0d), 0x00 },
};

/* BCLK/FS ratio codes for TAS2764_CLK_CFG (tas2764_bclk_ratio_codes). */
static const struct {
	uint16_t	ratio;
	uint8_t		code;
} tas2764_bclk_ratio_codes[] = {
	{ 250, 0x30 },
};

#define	DEVID_TAS2764	1
#define	DEVID_SN012776	2

static struct ofw_compat_data compat_data[] = {
	{ "ti,sn012776",	DEVID_SN012776 },
	{ "ti,tas2764",		DEVID_TAS2764 },
	{ NULL,			0 }
};

struct tas2764_softc;

/*
 * The amps on one shutdown line.  The first amp to attach takes the GPIO;
 * the line is high exactly while votes > 0 (or for an attach probe while
 * votes == 0).  failed: an over-temperature / over-current latch dropped
 * it for good.
 */
struct tas2764_group {
	LIST_ENTRY(tas2764_group) link;
	gpio_pin_t		pin;
	LIST_HEAD(, tas2764_softc) members;
	int			users;
	int			votes;
	bool			high;
	bool			failed;
};

struct tas2764_softc {
	device_t		dev;
	LIST_ENTRY(tas2764_softc) glink;
	struct tas2764_group	*group;
	struct timeout_task	poll_task;
	bool			detaching;
	int			devid;
	int			page;		/* selected page, -1 unknown */
	bool			powered;

	/* Device tree. */
	bool			tweeter;
	bool			has_irq;
	int			dt_imon, dt_vmon;
	int			bclk_code;	/* -1: auto-detect */
	u_int			bclk_ratio;
	uint8_t			bop[TAS2764_BOP_CFG_LEN];
	bool			have_bop;
	uint8_t			dc_blk[2];
	bool			have_dc_blk;
	uint32_t		tx_zd_mask;
	bool			have_tx_zd;
	int			adt_level;	/* -1: none */

	/* Configuration from the glue. */
	int			fmt, inv;
	int			tx_slot;	/* -1: not set */
	int			nslots, slot_width;
	int			imon, vmon;
	uint32_t		idle_mask;
	uint32_t		rate, width;
	u_int			vol;
	u_int			gain;
	bool			vsense, isense;	/* "VSENSE/ISENSE Switch" */
	char			prefix[64];	/* sound-name-prefix */

	/* Faults. */
	uint8_t			fault_bits;
	u_int			fault_events;
};

static struct sx tas2764_sx;
SX_SYSINIT(tas2764_sx, &tas2764_sx, "tas2764");
static LIST_HEAD(, tas2764_group) tas2764_groups =
    LIST_HEAD_INITIALIZER(tas2764_groups);

static MALLOC_DEFINE(M_TAS2764, "tas2764", "TAS2764 amplifier group");

static int	tas2764_probe(device_t);
static int	tas2764_attach(device_t);
static int	tas2764_detach(device_t);

/*
 * Register access.
 */

static int
tas2764_raw_write(struct tas2764_softc *sc, uint8_t reg, uint8_t val)
{
	uint8_t buf[2];
	struct iic_msg msg;

	buf[0] = reg;
	buf[1] = val;
	msg.slave = iicbus_get_addr(sc->dev);
	msg.flags = IIC_M_WR;
	msg.len = sizeof(buf);
	msg.buf = buf;
	return (iicbus_transfer_excl(sc->dev, &msg, 1, IIC_WAIT));
}

static int
tas2764_select_page(struct tas2764_softc *sc, uint16_t reg)
{
	int error, page;

	page = reg / 128;
	if (sc->page == page)
		return (0);
	error = tas2764_raw_write(sc, TAS2764_PAGE, page);
	sc->page = error == 0 ? page : -1;
	if (error != 0)
		device_printf(sc->dev, "page %d select failed: %d\n", page,
		    error);
	return (error);
}

static int
tas2764_read(struct tas2764_softc *sc, uint16_t reg, uint8_t *val)
{
	uint8_t addr;
	struct iic_msg msgs[2];
	int error;

	if ((error = tas2764_select_page(sc, reg)) != 0)
		return (error);
	addr = reg % 128;
	msgs[0].slave = iicbus_get_addr(sc->dev);
	msgs[0].flags = IIC_M_WR | IIC_M_NOSTOP;
	msgs[0].len = 1;
	msgs[0].buf = &addr;
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
tas2764_write(struct tas2764_softc *sc, uint16_t reg, uint8_t val)
{
	int error;

	if ((error = tas2764_select_page(sc, reg)) != 0)
		return (error);
	error = tas2764_raw_write(sc, reg % 128, val);
	if (error != 0)
		device_printf(sc->dev, "write 0x%04x <- 0x%02x failed: %d\n",
		    reg, val, error);
	return (error);
}

/* regmap_update_bits(): skips the write when nothing changes. */
static int
tas2764_update(struct tas2764_softc *sc, uint16_t reg, uint8_t mask,
    uint8_t val)
{
	uint8_t old, new;
	int error;

	if ((error = tas2764_read(sc, reg, &old)) != 0)
		return (error);
	new = (old & ~mask) | (val & mask);
	if (new == old)
		return (0);
	return (tas2764_write(sc, reg, new));
}

static void
tas2764_msleep(int ms)
{

	pause_sbt("tas2764", ms * SBT_1MS, 0, C_PREL(1));
}

/*
 * The shared shutdown line.
 */

static void
tas2764_line_set(struct tas2764_group *g, bool high)
{
	struct tas2764_softc *m;

	sx_assert(&tas2764_sx, SA_XLOCKED);
	gpio_pin_set_active(g->pin, high);
	if (high && !g->high) {
		DELAY(TAS2764_SDZ_SETTLE_US);
		/* Every amp on the line is out of a power-on reset: page 0 */
		LIST_FOREACH(m, &g->members, glink)
			m->page = 0;
	} else if (!high) {
		LIST_FOREACH(m, &g->members, glink) {
			m->page = -1;
			m->powered = false;
		}
	}
	g->high = high;
}

/*
 * Fail the group closed (Linux tas2764_group_shutdown()): the line goes
 * low, every amp is in hardware shutdown, and power-on is refused until
 * the driver is reloaded.
 */
static void
tas2764_group_fail(struct tas2764_softc *sc, uint8_t cause)
{
	struct tas2764_group *g = sc->group;

	g->failed = true;
	g->votes = 0;
	tas2764_line_set(g, false);
	device_printf(sc->dev, "SDZ group shut down (cause %#x): every amp "
	    "is in hardware shutdown until the driver is reloaded\n", cause);
}

static int
tas2764_group_join(struct tas2764_softc *sc, gpio_pin_t pin)
{
	struct tas2764_group *g;

	sx_assert(&tas2764_sx, SA_XLOCKED);
	LIST_FOREACH(g, &tas2764_groups, link) {
		if (g->pin->dev == pin->dev && g->pin->pin == pin->pin) {
			/* The first amp's handle drives the line. */
			gpio_pin_release(pin);
			goto found;
		}
	}
	g = malloc(sizeof(*g), M_TAS2764, M_WAITOK | M_ZERO);
	g->pin = pin;
	LIST_INIT(&g->members);
	/* Low before the pin becomes an output, and low after. */
	gpio_pin_set_active(pin, false);
	gpio_pin_setflags(pin, GPIO_PIN_OUTPUT);
	gpio_pin_set_active(pin, false);
	LIST_INSERT_HEAD(&tas2764_groups, g, link);
found:
	g->users++;
	LIST_INSERT_HEAD(&g->members, sc, glink);
	sc->group = g;
	sc->page = -1;
	return (0);
}

static void
tas2764_group_leave(struct tas2764_softc *sc)
{
	struct tas2764_group *g = sc->group;

	sx_assert(&tas2764_sx, SA_XLOCKED);
	LIST_REMOVE(sc, glink);
	sc->group = NULL;
	if (--g->users > 0)
		return;
	gpio_pin_set_active(g->pin, false);
	gpio_pin_release(g->pin);
	LIST_REMOVE(g, link);
	free(g, M_TAS2764);
}

/*
 * Power sequences.
 */

static int
tas2764_set_mode(struct tas2764_softc *sc, uint8_t mode)
{

	return (tas2764_update(sc, TAS2764_PWR_CTRL, TAS2764_PWR_CTRL_MASK,
	    mode));
}

static int
tas2764_write_dvc(struct tas2764_softc *sc)
{
	u_int vol;

	vol = MIN(sc->vol, TAS2764_VOL_CAP);
	return (tas2764_write(sc, TAS2764_DVC, TAS2764_DVC_REG(vol)));
}

static u_int
tas2764_gain_limit(struct tas2764_softc *sc)
{
	u_int lim;

	lim = TAS2764_AMP_GAIN_CAP;
	if (sc->adt_level >= 0)
		lim = MIN(lim, (u_int)sc->adt_level);
	return (lim);
}

static int
tas2764_write_gain(struct tas2764_softc *sc)
{
	u_int gain;

	gain = MIN(sc->gain, tas2764_gain_limit(sc));
	return (tas2764_update(sc, TAS2764_CHNL_0, TAS2764_CHNL_0_GAIN_MASK,
	    gain << TAS2764_CHNL_0_GAIN_SHIFT));
}

/*
 * The V/I sense ADCs: Linux's "VSENSE Switch" / "ISENSE Switch", DAPM
 * switches on PWR_CTRL bits 3 and 4 (inverted).  Leaves the mode bits.
 */
static int
tas2764_write_sense(struct tas2764_softc *sc)
{

	return (tas2764_update(sc, TAS2764_PWR_CTRL,
	    TAS2764_PWR_CTRL_VSNS_PD | TAS2764_PWR_CTRL_ISNS_PD,
	    (sc->vsense ? 0 : TAS2764_PWR_CTRL_VSNS_PD) |
	    (sc->isense ? 0 : TAS2764_PWR_CTRL_ISNS_PD)));
}

/* tas2764_write_sdout_idle_mask(): slots 0-31 */
static int
tas2764_write_idle_mask(struct tas2764_softc *sc, uint32_t mask)
{
	int error, i;

	for (i = 0; i < 4; i++) {
		error = tas2764_write(sc, TAS2764_SDOUT_HIZ_1 + i,
		    (mask >> (i * 8)) & 0xff);
		if (error != 0)
			return (error);
	}
	return (tas2764_update(sc, TAS2764_SDOUT_HIZ_9,
	    TAS2764_SDOUT_HIZ_9_FORCE_0_EN,
	    mask != 0 ? TAS2764_SDOUT_HIZ_9_FORCE_0_EN : 0));
}

/*
 * Linux tas2764_codec_probe() after SDZ high (reset, interrupt masks, V/I
 * sense transmit, BOP source and config, DC blockers, TX zero-drive,
 * quirks, BCLK ratio), macaudio_set_speaker()'s fixed controls, then the
 * DAI ops (set_fmt, set_tdm_slot, set_tdm_idle, hw_params) and the gain
 * and volume.  The amp stays in software shutdown throughout.
 */
static int
tas2764_program(struct tas2764_softc *sc)
{
	uint8_t v, cfg0, cfg1, cfg4;
	u_int i;
	int error;

#define	W(r, v)		do { if ((error = tas2764_write(sc, (r), (v))) != 0) \
			    return (error); } while (0)
#define	U(r, m, v)	do { if ((error = tas2764_update(sc, (r), (m), (v))) \
			    != 0) return (error); } while (0)

	/* tas2764_reset(): no reset GPIO on these boards */
	W(TAS2764_SW_RST, TAS2764_RST);
	DELAY(TAS2764_RESET_US);
	sc->page = 0;

	/* Must still be in software shutdown, as from reset. */
	if ((error = tas2764_read(sc, TAS2764_PWR_CTRL, &v)) != 0)
		return (error);
	if ((v & TAS2764_PWR_CTRL_MASK) != TAS2764_PWR_CTRL_SHUTDOWN) {
		device_printf(sc->dev, "PWR_CTRL 0x%02x after reset\n", v);
		return (EIO);
	}

	if (sc->has_irq) {
		W(TAS2764_INT_MASK0, 0x00);
		W(TAS2764_INT_MASK1, 0xff);
		W(TAS2764_INT_MASK2, 0xff);
		W(TAS2764_INT_MASK3, 0xff);
		W(TAS2764_INT_MASK4, 0xff);
	}

	/* tas2764_set_ivsense_transmit(); the ADCs follow the switches */
	if (sc->imon >= 0 && sc->vmon >= 0) {
		U(TAS2764_TDM_CFG5, TAS2764_TDM_CFG5_VSNS_ENABLE |
		    TAS2764_TDM_CFG5_50_MASK,
		    TAS2764_TDM_CFG5_VSNS_ENABLE | sc->vmon);
		U(TAS2764_TDM_CFG6, TAS2764_TDM_CFG6_ISNS_ENABLE |
		    TAS2764_TDM_CFG6_50_MASK,
		    TAS2764_TDM_CFG6_ISNS_ENABLE | sc->imon);
	}

	if (sc->devid == DEVID_SN012776) {
		U(TAS2764_PWR_CTRL, TAS2764_PWR_CTRL_BOP_SRC,
		    TAS2764_PWR_CTRL_BOP_SRC);
		for (i = 0; sc->have_bop && i < TAS2764_BOP_CFG_LEN; i++)
			W(TAS2764_BOP_CFG0 + i, sc->bop[i]);
		if (sc->have_dc_blk) {
			v = sc->dc_blk[0];
			/* macaudio: tweeters at MACAUDIO_TWEETER_HPF_SAFE */
			if (sc->tweeter)
				v = (v & ~TAS2764_DC_BLK0_HPF_MASK) |
				    TAS2764_HPF_800HZ;
			W(TAS2764_DC_BLK0, v);
			W(TAS2764_DC_BLK1, sc->dc_blk[1]);
		}
		if (sc->have_tx_zd && sc->tx_zd_mask != 0) {
			if ((error = tas2764_write_idle_mask(sc,
			    sc->tx_zd_mask)) != 0)
				return (error);
		}
		for (i = 0; i < nitems(tas2764_init_quirks); i++)
			W(tas2764_init_quirks[i].reg, tas2764_init_quirks[i].val);
	}
	if (sc->tweeter && !(sc->devid == DEVID_SN012776 && sc->have_dc_blk))
		U(TAS2764_DC_BLK0, TAS2764_DC_BLK0_HPF_MASK, TAS2764_HPF_800HZ);

	/* tas2764_apply_bclk_ratio() */
	if (sc->bclk_code >= 0) {
		W(TAS2764_CLK_CFG, sc->bclk_code);
		U(TAS2764_TDM_CFG0, TAS2764_TDM_CFG0_FIXED_RATIO,
		    TAS2764_TDM_CFG0_FIXED_RATIO);
	}

	/* macaudio_set_speaker(): "ASI1 Sel" Left (stereo), "OCE" Retry */
	U(TAS2764_TDM_CFG2, TAS2764_TDM_CFG2_SCFG_MASK,
	    TAS2764_TDM_CFG2_SCFG_LEFT);
	U(TAS2764_MISC_CFG1, TAS2764_MISC_CFG1_OCE_RETRY,
	    TAS2764_MISC_CFG1_OCE_RETRY);

	/* tas2764_set_fmt() */
	cfg0 = 0;
	switch (sc->inv) {
	case TAS2764_INV_NB_IF:
		cfg0 ^= TAS2764_TDM_CFG0_FRAME_START;
		/* FALLTHROUGH */
	case TAS2764_INV_NB_NF:
		cfg1 = TAS2764_TDM_CFG1_RX_RISING;
		cfg4 = TAS2764_TDM_CFG4_TX_FALLING;
		break;
	case TAS2764_INV_IB_IF:
		cfg0 ^= TAS2764_TDM_CFG0_FRAME_START;
		/* FALLTHROUGH */
	default:
		cfg1 = TAS2764_TDM_CFG1_RX_FALLING;
		cfg4 = TAS2764_TDM_CFG4_TX_RISING;
		break;
	}
	U(TAS2764_TDM_CFG1, TAS2764_TDM_CFG1_RX_MASK, cfg1);
	U(TAS2764_TDM_CFG4, TAS2764_TDM_CFG4_TX_MASK, cfg4);
	if (sc->fmt == TAS2764_FMT_I2S)
		cfg0 ^= TAS2764_TDM_CFG0_FRAME_START;
	v = (sc->fmt == TAS2764_FMT_I2S || sc->fmt == TAS2764_FMT_DSP_A) ?
	    1 : 0;
	U(TAS2764_TDM_CFG0, TAS2764_TDM_CFG0_FRAME_START, cfg0);
	U(TAS2764_TDM_CFG1, TAS2764_TDM_CFG1_MASK,
	    v << TAS2764_TDM_CFG1_51_SHIFT);

	/* tas2764_set_dai_tdm_slot(): one slot, left = right */
	W(TAS2764_TDM_CFG3, (sc->tx_slot << TAS2764_TDM_CFG3_RXS_SHIFT) |
	    sc->tx_slot);
	switch (sc->slot_width) {
	case 16:
		v = TAS2764_TDM_CFG2_RXS_16BITS;
		break;
	case 24:
		v = TAS2764_TDM_CFG2_RXS_24BITS;
		break;
	default:
		v = TAS2764_TDM_CFG2_RXS_32BITS;
		break;
	}
	U(TAS2764_TDM_CFG2, TAS2764_TDM_CFG2_RXS_MASK, v);

	/* tas2764_set_dai_tdm_idle() (fresh from reset: only a set mask) */
	if (!sc->have_tx_zd && sc->idle_mask != 0) {
		if ((error = tas2764_write_idle_mask(sc, sc->idle_mask)) != 0)
			return (error);
	}

	/* tas2764_hw_params(): set_bitwidth, set_samplerate */
	switch (sc->width) {
	case 16:
		v = TAS2764_TDM_CFG2_RXW_16BITS;
		break;
	case 24:
		v = TAS2764_TDM_CFG2_RXW_24BITS;
		break;
	default:
		v = TAS2764_TDM_CFG2_RXW_32BITS;
		break;
	}
	U(TAS2764_TDM_CFG2, TAS2764_TDM_CFG2_RXW_MASK, v);
	switch (sc->rate) {
	case 44100:
		v = TAS2764_TDM_CFG0_SMP_44_1KHZ | TAS2764_TDM_CFG0_44_1_48KHZ;
		break;
	case 96000:
		v = TAS2764_TDM_CFG0_SMP_48KHZ | TAS2764_TDM_CFG0_88_2_96KHZ;
		break;
	case 88200:
		v = TAS2764_TDM_CFG0_SMP_44_1KHZ | TAS2764_TDM_CFG0_88_2_96KHZ;
		break;
	default:
		v = TAS2764_TDM_CFG0_SMP_48KHZ | TAS2764_TDM_CFG0_44_1_48KHZ;
		break;
	}
	U(TAS2764_TDM_CFG0, TAS2764_TDM_CFG0_SMP_MASK | TAS2764_TDM_CFG0_MASK,
	    v);

	if ((error = tas2764_write_sense(sc)) != 0)
		return (error);

	/* Gain and volume, capped; reset left them at maximum and 0 dB. */
	if ((error = tas2764_write_gain(sc)) != 0)
		return (error);
	if ((error = tas2764_write_dvc(sc)) != 0)
		return (error);

	/* Read both back: the amp must not leave shutdown above the caps. */
	if ((error = tas2764_read(sc, TAS2764_DVC, &v)) != 0)
		return (error);
	if (v < TAS2764_DVC_REG_MIN) {
		device_printf(sc->dev, "DVC reads 0x%02x, above the cap\n", v);
		return (EIO);
	}
	if ((error = tas2764_read(sc, TAS2764_CHNL_0, &v)) != 0)
		return (error);
	if (((v & TAS2764_CHNL_0_GAIN_MASK) >> TAS2764_CHNL_0_GAIN_SHIFT) >
	    tas2764_gain_limit(sc)) {
		device_printf(sc->dev, "CHNL_0 reads 0x%02x, above the cap\n",
		    v);
		return (EIO);
	}

	/* Stale latches from before the clock ran */
	U(TAS2764_INT_CLK_CFG, TAS2764_INT_CLK_CFG_IRQZ_CLR,
	    TAS2764_INT_CLK_CFG_IRQZ_CLR);
#undef W
#undef U
	return (0);
}

static void
tas2764_vote_drop(struct tas2764_softc *sc)
{
	struct tas2764_group *g = sc->group;

	if (!sc->powered)
		return;
	sc->powered = false;
	if (--g->votes == 0)
		tas2764_line_set(g, false);
}

static int
tas2764_power_on(struct tas2764_softc *sc)
{
	struct tas2764_group *g = sc->group;
	int error;

	sx_assert(&tas2764_sx, SA_XLOCKED);
	if (sc->powered)
		return (0);
	if (g->failed)
		return (EIO);
	if (sc->tx_slot < 0 || sc->rate == 0)
		return (EINVAL);

	if (!g->high)
		tas2764_line_set(g, true);
	g->votes++;
	sc->powered = true;

	error = tas2764_program(sc);
	/* tas2764_mute(0): mute (dac_powered), then active (unmuted) */
	if (error == 0)
		error = tas2764_set_mode(sc, TAS2764_PWR_CTRL_MUTE);
	if (error == 0)
		error = tas2764_set_mode(sc, TAS2764_PWR_CTRL_ACTIVE);
	if (error != 0) {
		device_printf(sc->dev, "power-on failed (%d): shutdown\n",
		    error);
		sc->page = -1;
		(void)tas2764_set_mode(sc, TAS2764_PWR_CTRL_SHUTDOWN);
		tas2764_vote_drop(sc);
		return (error);
	}
	taskqueue_enqueue_timeout(taskqueue_thread, &sc->poll_task,
	    MAX(1, TAS2764_POLL_MS * hz / 1000));
	return (0);
}

/* tas2764_mute(1): mute, ramp-down, shutdown, then the line */
static int
tas2764_power_off(struct tas2764_softc *sc)
{
	int error, error2;

	sx_assert(&tas2764_sx, SA_XLOCKED);
	if (!sc->powered)
		return (0);
	error = tas2764_set_mode(sc, TAS2764_PWR_CTRL_MUTE);
	tas2764_msleep(TAS2764_RAMP_DOWN_MS);
	error2 = tas2764_set_mode(sc, TAS2764_PWR_CTRL_SHUTDOWN);
	tas2764_msleep(TAS2764_SHUTDOWN_MS);
	tas2764_vote_drop(sc);
	return (error != 0 ? error : error2);
}

/* Linux tas2764_irq_locked(), polled. */
static void
tas2764_poll(void *arg, int pending __unused)
{
	struct tas2764_softc *sc = arg;
	uint8_t latched, new;
	int i;

	sx_xlock(&tas2764_sx);
	if (sc->detaching || !sc->powered) {
		sx_xunlock(&tas2764_sx);
		return;
	}
	if (tas2764_read(sc, TAS2764_INT_LTCH0, &latched) == 0 &&
	    latched != 0) {
		new = latched & ~sc->fault_bits;
		for (i = 0; i < 8; i++)
			if (new & (1 << i))
				device_printf(sc->dev, "%s\n",
				    tas2764_ltch0_msgs[i]);
		sc->fault_bits |= latched;
		if (sc->fault_events != UINT_MAX)
			sc->fault_events++;
		if (latched & TAS2764_INT_LTCH0_GROUP_FAULTS)
			tas2764_group_fail(sc, latched);
		else
			(void)tas2764_update(sc, TAS2764_INT_CLK_CFG,
			    TAS2764_INT_CLK_CFG_IRQZ_CLR,
			    TAS2764_INT_CLK_CFG_IRQZ_CLR);
	}
	if (sc->powered)
		taskqueue_enqueue_timeout(taskqueue_thread, &sc->poll_task,
		    MAX(1, TAS2764_POLL_MS * hz / 1000));
	sx_xunlock(&tas2764_sx);
}

/*
 * Interface for the sound glue (tas2764.h).
 */

int
tas2764_set_fmt(device_t dev, int fmt, int inv)
{
	struct tas2764_softc *sc = device_get_softc(dev);
	int error = 0;

	if (fmt < TAS2764_FMT_I2S || fmt > TAS2764_FMT_LEFT_J ||
	    inv < TAS2764_INV_NB_NF || inv > TAS2764_INV_IB_IF)
		return (EINVAL);
	sx_xlock(&tas2764_sx);
	if (sc->powered)
		error = EBUSY;
	else {
		sc->fmt = fmt;
		sc->inv = inv;
	}
	sx_xunlock(&tas2764_sx);
	return (error);
}

int
tas2764_set_tdm(device_t dev, int tx_slot, int imon_slot, int vmon_slot,
    int slot_width, int nslots)
{
	struct tas2764_softc *sc = device_get_softc(dev);
	int error = 0;

	/* TDM_CFG3 holds 4-bit slot numbers; the sense slots are 6-bit. */
	if (tx_slot < 0 || tx_slot > 15 || tx_slot >= nslots ||
	    (slot_width != 16 && slot_width != 24 && slot_width != 32) ||
	    imon_slot < -1 || imon_slot > 63 ||
	    vmon_slot < -1 || vmon_slot > 63)
		return (EINVAL);
	sx_xlock(&tas2764_sx);
	if (sc->powered)
		error = EBUSY;
	else {
		sc->tx_slot = tx_slot;
		sc->nslots = nslots;
		sc->slot_width = slot_width;
		sc->imon = imon_slot >= 0 ? imon_slot : sc->dt_imon;
		sc->vmon = vmon_slot >= 0 ? vmon_slot : sc->dt_vmon;
	}
	sx_xunlock(&tas2764_sx);
	return (error);
}

void
tas2764_get_sense_slots(device_t dev, int *imon_slot, int *vmon_slot)
{
	struct tas2764_softc *sc = device_get_softc(dev);

	sx_xlock(&tas2764_sx);
	*imon_slot = sc->imon;
	*vmon_slot = sc->vmon;
	sx_xunlock(&tas2764_sx);
}

int
tas2764_set_tdm_idle(device_t dev, uint32_t tx_mask)
{
	struct tas2764_softc *sc = device_get_softc(dev);
	int error = 0;

	sx_xlock(&tas2764_sx);
	if (sc->powered)
		error = EBUSY;
	else if (!sc->have_tx_zd)
		sc->idle_mask = tx_mask;
	sx_xunlock(&tas2764_sx);
	return (error);
}

int
tas2764_set_bclk_ratio(device_t dev, u_int ratio)
{
	struct tas2764_softc *sc = device_get_softc(dev);
	int error = 0;

	if (ratio < 8)
		return (EINVAL);
	sx_xlock(&tas2764_sx);
	if (sc->powered)
		error = EBUSY;
	else if (ratio / 8 < 32)
		sc->idle_mask &= (1u << (ratio / 8)) - 1;
	sx_xunlock(&tas2764_sx);
	return (error);
}

int
tas2764_hw_params(device_t dev, uint32_t rate, uint32_t width)
{
	struct tas2764_softc *sc = device_get_softc(dev);
	int error = 0;

	if (rate != 44100 && rate != 48000 && rate != 88200 && rate != 96000)
		return (EINVAL);
	if (width != 16 && width != 24 && width != 32)
		return (EINVAL);
	sx_xlock(&tas2764_sx);
	if (sc->powered)
		error = EBUSY;
	else {
		sc->rate = rate;
		sc->width = width;
	}
	sx_xunlock(&tas2764_sx);
	return (error);
}

int
tas2764_set_power(device_t dev, bool on)
{
	struct tas2764_softc *sc = device_get_softc(dev);
	int error;

	sx_xlock(&tas2764_sx);
	if (sc->detaching)
		error = ENXIO;
	else if (on)
		error = tas2764_power_on(sc);
	else
		error = tas2764_power_off(sc);
	sx_xunlock(&tas2764_sx);
	return (error);
}

int
tas2764_set_volume(device_t dev, u_int level)
{
	struct tas2764_softc *sc = device_get_softc(dev);
	int error = 0;

	sx_xlock(&tas2764_sx);
	sc->vol = MIN(level, TAS2764_VOL_CAP);
	if (sc->powered)
		error = tas2764_write_dvc(sc);
	sx_xunlock(&tas2764_sx);
	return (error);
}

u_int
tas2764_get_volume(device_t dev)
{
	struct tas2764_softc *sc = device_get_softc(dev);
	u_int vol;

	sx_xlock(&tas2764_sx);
	vol = sc->vol;
	sx_xunlock(&tas2764_sx);
	return (vol);
}

int
tas2764_set_amp_gain(device_t dev, u_int level)
{
	struct tas2764_softc *sc = device_get_softc(dev);
	int error = 0;

	sx_xlock(&tas2764_sx);
	sc->gain = MIN(level, tas2764_gain_limit(sc));
	if (sc->powered)
		error = tas2764_write_gain(sc);
	sx_xunlock(&tas2764_sx);
	return (error);
}

u_int
tas2764_get_amp_gain(device_t dev)
{
	struct tas2764_softc *sc = device_get_softc(dev);
	u_int gain;

	sx_xlock(&tas2764_sx);
	gain = sc->gain;
	sx_xunlock(&tas2764_sx);
	return (gain);
}

u_int
tas2764_get_amp_gain_max(device_t dev)
{
	struct tas2764_softc *sc = device_get_softc(dev);

	return (tas2764_gain_limit(sc));
}

int
tas2764_get_adt_level(device_t dev)
{
	struct tas2764_softc *sc = device_get_softc(dev);

	return (sc->adt_level);
}

int
tas2764_set_sense(device_t dev, bool vsense, bool isense)
{
	struct tas2764_softc *sc = device_get_softc(dev);
	int error = 0;

	sx_xlock(&tas2764_sx);
	sc->vsense = vsense;
	sc->isense = isense;
	if (sc->powered)
		error = tas2764_write_sense(sc);
	sx_xunlock(&tas2764_sx);
	return (error);
}

const char *
tas2764_get_prefix(device_t dev)
{
	struct tas2764_softc *sc = device_get_softc(dev);

	return (sc->prefix);
}

bool
tas2764_is_tweeter(device_t dev)
{
	struct tas2764_softc *sc = device_get_softc(dev);

	return (sc->tweeter);
}

/*
 * Sysctls.
 */

static int
tas2764_temp_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct tas2764_softc *sc = arg1;
	uint8_t reg;
	int temp;

	/* Linux tas2764_read_die_temp(): raw - 93 C; 0 means no reading */
	temp = -1000;
	sx_xlock(&tas2764_sx);
	if (sc->powered && tas2764_read(sc, TAS2764_TEMP, &reg) == 0 &&
	    reg != 0)
		temp = reg - 93;
	sx_xunlock(&tas2764_sx);
	return (sysctl_handle_int(oidp, &temp, 0, req));
}

/*
 * Device tree (Linux tas2764_parse_dt()).
 */

static int
tas2764_parse_dt(struct tas2764_softc *sc, phandle_t node)
{
	char prefix[64];
	uint8_t row[8];
	uint32_t val;
	ssize_t len;
	u_int i;

	sc->dt_imon = sc->dt_vmon = -1;
	if (OF_getencprop(node, "ti,imon-slot-no", &val, sizeof(val)) > 0 &&
	    val <= 63)
		sc->dt_imon = val;
	if (OF_getencprop(node, "ti,vmon-slot-no", &val, sizeof(val)) > 0 &&
	    val <= 63)
		sc->dt_vmon = val;

	sc->has_irq = OF_hasprop(node, "interrupts") ||
	    OF_hasprop(node, "interrupts-extended");

	len = OF_getprop(node, "sound-name-prefix", prefix,
	    sizeof(prefix) - 1);
	if (len > 0) {
		prefix[len] = '\0';
		strlcpy(sc->prefix, prefix, sizeof(sc->prefix));
		sc->tweeter = strstr(prefix, "Tweeter") != NULL;
		device_printf(sc->dev, "%s\n", prefix);
	}

	sc->bclk_code = -1;
	if (OF_getencprop(node, "ti,bclk-ratio", &val, sizeof(val)) > 0) {
		for (i = 0; i < nitems(tas2764_bclk_ratio_codes); i++)
			if (tas2764_bclk_ratio_codes[i].ratio == val)
				sc->bclk_code = tas2764_bclk_ratio_codes[i].code;
		if (sc->bclk_code < 0) {
			device_printf(sc->dev, "unsupported ti,bclk-ratio %u\n",
			    val);
			return (EINVAL);
		}
		sc->bclk_ratio = val;
	}

	/* macaudio_sn012776_level(): an upper bound on the amp gain */
	sc->adt_level = -1;
	if (OF_getproplen(node, "apple,speaker-config") == 4 &&
	    OF_getprop(node, "apple,speaker-config", row, 4) == 4 &&
	    (row[1] & 0x1f) <= TAS2764_AMP_GAIN_MAX)
		sc->adt_level = row[1] & 0x1f;

	if (sc->devid != DEVID_SN012776)
		return (0);

	/* tas2764_parse_bop_config() */
	len = OF_getproplen(node, "apple,bop-config");
	if (len > 0) {
		if (len != TAS2764_BOP_CFG_LEN) {
			device_printf(sc->dev, "apple,bop-config: %zd bytes, "
			    "the block has %d\n", len, TAS2764_BOP_CFG_LEN);
			return (EINVAL);
		}
		OF_getprop(node, "apple,bop-config", sc->bop,
		    TAS2764_BOP_CFG_LEN);
		sc->have_bop = true;
	}

	/* tas2764_parse_dcblocker_config(): default 0x01, 0x00 */
	len = OF_getproplen(node, "apple,amp-dcblocker-config");
	if (len <= 0) {
		sc->dc_blk[0] = 0x01;
		sc->dc_blk[1] = 0x00;
	} else if (len < 2) {
		device_printf(sc->dev, "apple,amp-dcblocker-config: %zd bytes, "
		    "need 2\n", len);
		return (EINVAL);
	} else
		OF_getprop(node, "apple,amp-dcblocker-config", sc->dc_blk, 2);
	sc->have_dc_blk = true;

	/* tas2764_parse_tx_zd_config() */
	len = OF_getproplen(node, "apple,amp-tx-zd-config");
	if (len > 0) {
		if (len != sizeof(row)) {
			device_printf(sc->dev, "apple,amp-tx-zd-config: %zd "
			    "bytes, the row has %zu\n", len, sizeof(row));
			return (EINVAL);
		}
		OF_getprop(node, "apple,amp-tx-zd-config", row, sizeof(row));
		for (i = 4; i < sizeof(row); i++)
			if (row[i] != 0)
				device_printf(sc->dev, "idle slots from %u on "
				    "are not supported\n", i * 8);
		sc->tx_zd_mask = row[0] | row[1] << 8 | row[2] << 16 |
		    (uint32_t)row[3] << 24;
		sc->have_tx_zd = true;
	}
	return (0);
}

/*
 * Newbus.
 */

static int
tas2764_probe(device_t dev)
{
	int devid;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	devid = ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	if (devid == 0)
		return (ENXIO);
	device_set_desc(dev, devid == DEVID_SN012776 ?
	    "TI SN012776 speaker amplifier" : "TI TAS2764 speaker amplifier");
	return (BUS_PROBE_DEFAULT);
}

static int
tas2764_attach(device_t dev)
{
	struct tas2764_softc *sc = device_get_softc(dev);
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *tree;
	struct tas2764_group *g;
	gpio_pin_t pin;
	phandle_t node;
	uint8_t pwr;
	bool raised;
	int error;

	sc->dev = dev;
	node = ofw_bus_get_node(dev);
	sc->devid = ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	sc->page = -1;
	sc->tx_slot = -1;
	sc->fmt = TAS2764_FMT_I2S;	/* macaudio spk_dai_fmt (J813/J815) */
	sc->inv = TAS2764_INV_NB_IF;
	sc->vol = TAS2764_VOL_MUTE;
	sc->gain = 0;
	TIMEOUT_TASK_INIT(taskqueue_thread, &sc->poll_task, 0, tas2764_poll,
	    sc);

	if ((error = tas2764_parse_dt(sc, node)) != 0)
		return (error);
	sc->imon = sc->dt_imon;
	sc->vmon = sc->dt_vmon;

	/* The line is the only way to reach the amp: no GPIO, no driver. */
	error = gpio_pin_get_by_ofw_property(dev, node, "shutdown-gpios",
	    &pin);
	if (error != 0) {
		device_printf(dev, "no shutdown GPIO: %d\n", error);
		return (ENXIO);
	}

	sx_xlock(&tas2764_sx);
	tas2764_group_join(sc, pin);
	g = sc->group;

	/*
	 * Probe: raise the line if nothing holds it, check the amp answers
	 * in software shutdown, then the line goes low again.
	 */
	raised = !g->high;
	if (raised)
		tas2764_line_set(g, true);
	error = tas2764_read(sc, TAS2764_PWR_CTRL, &pwr);
	if (raised)
		tas2764_line_set(g, false);
	if (error != 0) {
		device_printf(dev, "no answer on I2C: %d\n", error);
		tas2764_group_leave(sc);
		sx_xunlock(&tas2764_sx);
		return (ENXIO);
	}
	if (raised && pwr != TAS2764_PWR_CTRL_RESET)
		device_printf(dev, "PWR_CTRL 0x%02x after power-on, "
		    "expected 0x%02x\n", pwr, TAS2764_PWR_CTRL_RESET);
	sx_xunlock(&tas2764_sx);

	device_printf(dev, "%s at 0x%02x, sense I slot %d V slot %d%s\n",
	    sc->devid == DEVID_SN012776 ? "SN012776" : "TAS2764",
	    iicbus_get_addr(dev) >> 1, sc->dt_imon, sc->dt_vmon,
	    sc->tweeter ? ", tweeter (800 Hz high-pass)" : "");

	ctx = device_get_sysctl_ctx(dev);
	tree = SYSCTL_CHILDREN(device_get_sysctl_tree(dev));
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "volume", CTLFLAG_RD, &sc->vol,
	    0, "Digital volume (201 = 0 dB, 0.5 dB steps, capped at 161)");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "amp_gain", CTLFLAG_RD,
	    &sc->gain, 0, "Amp gain level (11 dB + level / 2)");
	SYSCTL_ADD_BOOL(ctx, tree, OID_AUTO, "powered", CTLFLAG_RD,
	    &sc->powered, 0, "Amp out of software shutdown");
	SYSCTL_ADD_U8(ctx, tree, OID_AUTO, "fault_bits", CTLFLAG_RD,
	    &sc->fault_bits, 0, "INT_LTCH0 bits seen since attach");
	SYSCTL_ADD_UINT(ctx, tree, OID_AUTO, "fault_events", CTLFLAG_RD,
	    &sc->fault_events, 0, "Fault latch reports");
	SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, "temperature",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    tas2764_temp_sysctl, "I", "Die temperature, C (-1000: none)");

	OF_device_register_xref(OF_xref_from_node(node), dev);
	return (0);
}

static int
tas2764_detach(device_t dev)
{
	struct tas2764_softc *sc = device_get_softc(dev);

	sx_xlock(&tas2764_sx);
	sc->detaching = true;
	(void)tas2764_power_off(sc);
	sx_xunlock(&tas2764_sx);
	while (taskqueue_cancel_timeout(taskqueue_thread, &sc->poll_task,
	    NULL) != 0)
		taskqueue_drain_timeout(taskqueue_thread, &sc->poll_task);

	OF_device_register_xref(OF_xref_from_node(ofw_bus_get_node(dev)),
	    NULL);
	sx_xlock(&tas2764_sx);
	tas2764_group_leave(sc);
	sx_xunlock(&tas2764_sx);
	return (0);
}

static device_method_t tas2764_methods[] = {
	DEVMETHOD(device_probe,		tas2764_probe),
	DEVMETHOD(device_attach,	tas2764_attach),
	DEVMETHOD(device_detach,	tas2764_detach),
	DEVMETHOD_END
};

static driver_t tas2764_driver = {
	"tas2764",
	tas2764_methods,
	sizeof(struct tas2764_softc)
};

DRIVER_MODULE(tas2764, iicbus, tas2764_driver, NULL, NULL);
MODULE_DEPEND(tas2764, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
MODULE_VERSION(tas2764, 1);
IICBUS_FDT_PNP_INFO(compat_data);
