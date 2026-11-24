/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple I2C controller (apple,t8103-i2c), a PA Semi SMBus block. Follows
 * aurora-silicon's Linux i2c-pasemi-core.c / i2c-pasemi-platform.c: the
 * transmit FIFO takes START/STOP/READ commands along with the bytes, and the
 * receive FIFO returns read data. Polled; transfers are short (codecs and
 * amplifiers).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm64/apple/apple_pmgr.h>

#include "iicbus_if.h"

#define	REG_MTXFIFO	0x00
#define	REG_MRXFIFO	0x04
#define	REG_XFSTA	0x0c
#define	REG_SMSTA	0x14
#define	REG_IMASK	0x18
#define	REG_CTL		0x1c
#define	REG_REV		0x28

#define	MTXFIFO_READ	(1u << 10)
#define	MTXFIFO_STOP	(1u << 9)
#define	MTXFIFO_START	(1u << 8)

#define	MRXFIFO_EMPTY	(1u << 8)
#define	MRXFIFO_DATA_M	0xff

#define	SMSTA_XIP	(1u << 28)
#define	SMSTA_XEN	(1u << 27)
#define	SMSTA_JMD	(1u << 25)
#define	SMSTA_JAM	(1u << 24)
#define	SMSTA_MTO	(1u << 23)
#define	SMSTA_MTA	(1u << 22)
#define	SMSTA_MTN	(1u << 21)
#define	SMSTA_MRNE	(1u << 19)
#define	SMSTA_MTE	(1u << 16)
#define	SMSTA_TOM	(1u << 6)

#define	CTL_EN		(1u << 11)
#define	CTL_MRR		(1u << 10)
#define	CTL_MTR		(1u << 9)
#define	CTL_UJM		(1u << 8)
#define	CTL_CLK_M	0xff

#define	TRANSFER_TIMEOUT_US	100000
#define	POLL_US			10
#define	DEFAULT_FREQ		100000

struct apple_i2c_softc {
	device_t		dev;
	device_t		iicbus;
	struct resource		*mem;
	struct mtx		mtx;
	clk_t			clk;
	uint32_t		clk_div;
	uint32_t		hw_rev;
};

static struct ofw_compat_data compat_data[] = {
	{ "apple,t8103-i2c",	1 },
	{ "apple,i2c",		1 },
	{ NULL,			0 }
};

#define	RD4(sc, reg)		bus_read_4((sc)->mem, (reg))
#define	WR4(sc, reg, val)	bus_write_4((sc)->mem, (reg), (val))

static void
apple_i2c_hw_reset(struct apple_i2c_softc *sc)
{
	uint32_t val;

	val = CTL_MTR | CTL_MRR | CTL_UJM | (sc->clk_div & CTL_CLK_M);
	if (sc->hw_rev >= 6)
		val |= CTL_EN;
	WR4(sc, REG_CTL, val);
}

static void
apple_i2c_init(struct apple_i2c_softc *sc)
{
	sc->hw_rev = RD4(sc, REG_REV);
	WR4(sc, REG_IMASK, 0);
	apple_i2c_hw_reset(sc);
}

/* Polls SMSTA until (status & mask) == want or the transfer timeout. */
static int
apple_i2c_poll(struct apple_i2c_softc *sc, uint32_t mask, uint32_t want,
    uint32_t *status)
{
	int us;

	for (us = 0; us < TRANSFER_TIMEOUT_US; us += POLL_US) {
		*status = RD4(sc, REG_SMSTA);
		if ((*status & mask) == want)
			return (0);
		DELAY(POLL_US);
	}
	*status = RD4(sc, REG_SMSTA);
	return (ETIMEDOUT);
}

static int
apple_i2c_clear(struct apple_i2c_softc *sc)
{
	uint32_t status;

	/* First wait for the bus to go idle. */
	if (apple_i2c_poll(sc, SMSTA_XIP | SMSTA_JAM, 0, &status) != 0) {
		device_printf(sc->dev,
		    "bus is stuck (status 0x%08x xfstatus 0x%08x)\n",
		    status, RD4(sc, REG_XFSTA));
		return (IIC_EBUSBSY);
	}
	/* If anything went wrong or the FIFOs hold data, reset the FIFOs. */
	if ((status & (SMSTA_MRNE | SMSTA_JMD | SMSTA_MTO | SMSTA_TOM |
	    SMSTA_MTN | SMSTA_MTA)) != 0 || (status & SMSTA_MTE) == 0)
		apple_i2c_hw_reset(sc);
	/* Clear the flags. */
	WR4(sc, REG_SMSTA, status);
	return (0);
}

static int
apple_i2c_waitready(struct apple_i2c_softc *sc, uint16_t slave)
{
	uint32_t status;

	if (apple_i2c_poll(sc, SMSTA_XEN, SMSTA_XEN, &status) != 0) {
		device_printf(sc->dev, "timeout, status 0x%08x\n", status);
		return (IIC_ETIMEOUT);
	}
	if (status & SMSTA_TOM) {
		device_printf(sc->dev, "controller timeout, status 0x%08x\n",
		    status);
		return (IIC_EBUSERR);
	}
	if (status & SMSTA_MTO) {
		device_printf(sc->dev, "peripheral timeout, status 0x%08x\n",
		    status);
		return (IIC_ETIMEOUT);
	}
	if (status & SMSTA_MTA)
		return (IIC_EBUSBSY);
	/* NACK arrives before the STOP is out, so XIP may still be set. */
	if (status & SMSTA_MTN)
		return (IIC_ENOACK);
	if (status & SMSTA_XIP) {
		device_printf(sc->dev, "bus stuck, status 0x%08x\n", status);
		return (IIC_EBUSERR);
	}
	WR4(sc, REG_SMSTA, SMSTA_XEN);
	return (0);
}

static int
apple_i2c_xfer_msg(struct apple_i2c_softc *sc, struct iic_msg *msg, bool stop)
{
	uint32_t rd;
	int error, i;

	if (msg->len == 0)
		return (IIC_ENOTSUPP);
	if (msg->flags & IIC_M_RD) {
		WR4(sc, REG_MTXFIFO, MTXFIFO_START | (msg->slave | 1));
		WR4(sc, REG_MTXFIFO, msg->len | MTXFIFO_READ |
		    (stop ? MTXFIFO_STOP : 0));
		if ((error = apple_i2c_waitready(sc, msg->slave)) != 0)
			goto reset;
		for (i = 0; i < msg->len; i++) {
			rd = RD4(sc, REG_MRXFIFO);
			if (rd & MRXFIFO_EMPTY) {
				error = IIC_EUNDERFLOW;
				goto reset;
			}
			msg->buf[i] = rd & MRXFIFO_DATA_M;
		}
	} else {
		WR4(sc, REG_MTXFIFO, MTXFIFO_START | (msg->slave & ~1));
		for (i = 0; i < msg->len - 1; i++)
			WR4(sc, REG_MTXFIFO, msg->buf[i]);
		WR4(sc, REG_MTXFIFO, msg->buf[msg->len - 1] |
		    (stop ? MTXFIFO_STOP : 0));
		if (stop && (error = apple_i2c_waitready(sc, msg->slave)) != 0)
			goto reset;
	}
	return (0);
reset:
	apple_i2c_hw_reset(sc);
	return (error);
}

static int
apple_i2c_transfer(device_t dev, struct iic_msg *msgs, uint32_t nmsgs)
{
	struct apple_i2c_softc *sc = device_get_softc(dev);
	uint32_t i;
	int error;

	mtx_lock(&sc->mtx);
	error = apple_i2c_clear(sc);
	for (i = 0; i < nmsgs && error == 0; i++)
		error = apple_i2c_xfer_msg(sc, &msgs[i], i == nmsgs - 1 ||
		    (msgs[i].flags & IIC_M_NOSTOP) == 0);
	mtx_unlock(&sc->mtx);
	return (error);
}

static int
apple_i2c_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{
	struct apple_i2c_softc *sc = device_get_softc(dev);

	mtx_lock(&sc->mtx);
	apple_i2c_init(sc);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
apple_i2c_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Apple I2C");
	return (BUS_PROBE_DEFAULT);
}

static int apple_i2c_detach(device_t dev);

static int
apple_i2c_attach(device_t dev)
{
	struct apple_i2c_softc *sc = device_get_softc(dev);
	uint64_t rate;
	uint32_t freq;
	int error, rid;

	sc->dev = dev;
	mtx_init(&sc->mtx, device_get_nameunit(dev), "apple_i2c", MTX_DEF);

	if ((error = apple_pmgr_power_on(dev)) != 0) {
		device_printf(dev, "cannot power on: %d\n", error);
		goto fail;
	}
	rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "cannot map registers\n");
		error = ENXIO;
		goto fail;
	}
	if ((error = clk_get_by_ofw_index(dev, 0, 0, &sc->clk)) != 0 ||
	    (error = clk_enable(sc->clk)) != 0 ||
	    (error = clk_get_freq(sc->clk, &rate)) != 0) {
		device_printf(dev, "cannot get the reference clock: %d\n",
		    error);
		goto fail;
	}
	if (OF_getencprop(ofw_bus_get_node(dev), "clock-frequency", &freq,
	    sizeof(freq)) <= 0 || freq == 0)
		freq = DEFAULT_FREQ;
	sc->clk_div = howmany(rate, 16 * (uint64_t)freq);
	if (sc->clk_div < 4 || sc->clk_div > 0xff) {
		device_printf(dev, "bus frequency %u Hz out of range\n", freq);
		error = EINVAL;
		goto fail;
	}
	apple_i2c_init(sc);
	if (bootverbose)
		device_printf(dev, "rev %u, %u Hz (divider %u)\n", sc->hw_rev,
		    freq, sc->clk_div);

	sc->iicbus = device_add_child(dev, "iicbus", DEVICE_UNIT_ANY);
	if (sc->iicbus == NULL) {
		error = ENXIO;
		goto fail;
	}
	bus_attach_children(dev);
	return (0);
fail:
	apple_i2c_detach(dev);
	return (error);
}

static int
apple_i2c_detach(device_t dev)
{
	struct apple_i2c_softc *sc = device_get_softc(dev);
	int error;

	if ((error = bus_generic_detach(dev)) != 0)
		return (error);
	if (sc->clk != NULL)
		clk_release(sc->clk);
	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem);
	mtx_destroy(&sc->mtx);
	return (0);
}

static phandle_t
apple_i2c_get_node(device_t bus, device_t dev)
{
	return (ofw_bus_get_node(bus));
}

static device_method_t apple_i2c_methods[] = {
	DEVMETHOD(device_probe,		apple_i2c_probe),
	DEVMETHOD(device_attach,	apple_i2c_attach),
	DEVMETHOD(device_detach,	apple_i2c_detach),

	DEVMETHOD(ofw_bus_get_node,	apple_i2c_get_node),

	/* Children's interrupts (codec jack, amp faults) go up the tree. */
	DEVMETHOD(bus_alloc_resource,	bus_generic_alloc_resource),
	DEVMETHOD(bus_release_resource,	bus_generic_release_resource),
	DEVMETHOD(bus_activate_resource, bus_generic_activate_resource),
	DEVMETHOD(bus_deactivate_resource, bus_generic_deactivate_resource),
	DEVMETHOD(bus_setup_intr,	bus_generic_setup_intr),
	DEVMETHOD(bus_teardown_intr,	bus_generic_teardown_intr),

	DEVMETHOD(iicbus_callback,	iicbus_null_callback),
	DEVMETHOD(iicbus_reset,		apple_i2c_reset),
	DEVMETHOD(iicbus_transfer,	apple_i2c_transfer),

	DEVMETHOD_END
};

static driver_t apple_i2c_driver = {
	"apple_i2c",
	apple_i2c_methods,
	sizeof(struct apple_i2c_softc),
};

EARLY_DRIVER_MODULE(apple_i2c, simplebus, apple_i2c_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
EARLY_DRIVER_MODULE(ofw_iicbus, apple_i2c, ofw_iicbus_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
MODULE_DEPEND(apple_i2c, iicbus, 1, 1, 1);
MODULE_VERSION(apple_i2c, 1);
