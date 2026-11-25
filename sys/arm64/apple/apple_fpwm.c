/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple fPWM (apple,s5l-fpwm), a one-channel PWM clocked from the 24 MHz
 * reference. Follows Linux's pwm-apple.c: the period is split into ON and OFF
 * cycle counts, and CONTROL's UPDATE/TRIGGER bits latch them. On the M5
 * laptops it drives the keyboard backlight (ADT /arm-io/fpwm0/kbd-backlight).
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
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm64/apple/apple_pmgr.h>

#include "pwmbus_if.h"

#define	REG_CONTROL	0x00
#define	REG_OFF_CYCLES	0x18
#define	REG_ON_CYCLES	0x1c

#define	CONTROL_ENABLE		(1u << 0)
#define	CONTROL_MODE		(1u << 2)
#define	CONTROL_UPDATE		(1u << 5)
#define	CONTROL_TRIGGER		(1u << 9)
#define	CONTROL_INVERT		(1u << 10)
#define	CONTROL_OUTPUT_ENABLE	(1u << 14)

struct apple_fpwm_softc {
	device_t		dev;
	device_t		busdev;
	struct resource		*mem;
	clk_t			clk;
	uint64_t		rate;
	struct mtx		mtx;
	u_int			period;		/* ns */
	u_int			duty;		/* ns */
	bool			enabled;
};

static struct ofw_compat_data compat_data[] = {
	{ "apple,s5l-fpwm",	1 },
	{ NULL,			0 }
};

#define	RD4(sc, r)	bus_read_4((sc)->mem, (r))
#define	WR4(sc, r, v)	bus_write_4((sc)->mem, (r), (v))

static void
apple_fpwm_apply(struct apple_fpwm_softc *sc)
{
	uint64_t on, total;

	mtx_assert(&sc->mtx, MA_OWNED);
	if (!sc->enabled || sc->period == 0) {
		WR4(sc, REG_CONTROL, 0);
		return;
	}
	total = sc->rate * sc->period / 1000000000;
	on = sc->rate * sc->duty / 1000000000;
	if (on > total)
		on = total;
	WR4(sc, REG_ON_CYCLES, on);
	WR4(sc, REG_OFF_CYCLES, total - on);
	WR4(sc, REG_CONTROL, CONTROL_ENABLE | CONTROL_MODE | CONTROL_UPDATE |
	    CONTROL_TRIGGER | CONTROL_OUTPUT_ENABLE);
}

static int
apple_fpwm_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Apple fPWM");
	return (BUS_PROBE_DEFAULT);
}

static int apple_fpwm_detach(device_t dev);

static int
apple_fpwm_attach(device_t dev)
{
	struct apple_fpwm_softc *sc = device_get_softc(dev);
	uint32_t ctl, on, off;
	int error, rid;

	sc->dev = dev;
	mtx_init(&sc->mtx, device_get_nameunit(dev), "apple_fpwm", MTX_DEF);

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
	    (error = clk_get_freq(sc->clk, &sc->rate)) != 0 || sc->rate == 0) {
		device_printf(dev, "cannot get the reference clock: %d\n",
		    error);
		if (error == 0)
			error = ENXIO;
		goto fail;
	}

	/* Take over whatever iBoot left running. */
	ctl = RD4(sc, REG_CONTROL);
	on = RD4(sc, REG_ON_CYCLES);
	off = RD4(sc, REG_OFF_CYCLES);
	sc->enabled = (ctl & CONTROL_ENABLE) != 0 &&
	    (ctl & CONTROL_OUTPUT_ENABLE) != 0;
	sc->period = (uint64_t)(on + off) * 1000000000 / sc->rate;
	sc->duty = (uint64_t)on * 1000000000 / sc->rate;
	if (bootverbose)
		device_printf(dev, "%ju Hz, control 0x%x, on %u off %u\n",
		    (uintmax_t)sc->rate, ctl, on, off);

	OF_device_register_xref(OF_xref_from_node(ofw_bus_get_node(dev)), dev);
	sc->busdev = device_add_child(dev, "pwmbus", DEVICE_UNIT_ANY);
	bus_attach_children(dev);
	return (0);
fail:
	apple_fpwm_detach(dev);
	return (error);
}

static int
apple_fpwm_detach(device_t dev)
{
	struct apple_fpwm_softc *sc = device_get_softc(dev);
	int error;

	if ((error = bus_generic_detach(dev)) != 0)
		return (error);
	if (sc->clk != NULL)
		clk_release(sc->clk);
	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem);
	if (mtx_initialized(&sc->mtx))
		mtx_destroy(&sc->mtx);
	return (0);
}

static phandle_t
apple_fpwm_get_node(device_t bus, device_t dev)
{
	/* pwmbus walks our node for its children. */
	return (ofw_bus_get_node(bus));
}

static int
apple_fpwm_channel_count(device_t dev, u_int *nchannel)
{
	*nchannel = 1;
	return (0);
}

static int
apple_fpwm_channel_config(device_t dev, u_int channel, u_int period,
    u_int duty)
{
	struct apple_fpwm_softc *sc = device_get_softc(dev);

	if (channel != 0 || duty > period)
		return (EINVAL);
	/* At least one reference cycle. */
	if (period == 0 || sc->rate * period / 1000000000 == 0)
		return (EINVAL);
	mtx_lock(&sc->mtx);
	sc->period = period;
	sc->duty = duty;
	apple_fpwm_apply(sc);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
apple_fpwm_channel_get_config(device_t dev, u_int channel, u_int *period,
    u_int *duty)
{
	struct apple_fpwm_softc *sc = device_get_softc(dev);

	if (channel != 0)
		return (EINVAL);
	mtx_lock(&sc->mtx);
	*period = sc->period;
	*duty = sc->duty;
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
apple_fpwm_channel_enable(device_t dev, u_int channel, bool enable)
{
	struct apple_fpwm_softc *sc = device_get_softc(dev);

	if (channel != 0)
		return (EINVAL);
	mtx_lock(&sc->mtx);
	sc->enabled = enable;
	apple_fpwm_apply(sc);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
apple_fpwm_channel_is_enabled(device_t dev, u_int channel, bool *enabled)
{
	struct apple_fpwm_softc *sc = device_get_softc(dev);

	if (channel != 0)
		return (EINVAL);
	*enabled = sc->enabled;
	return (0);
}

static device_method_t apple_fpwm_methods[] = {
	DEVMETHOD(device_probe,		apple_fpwm_probe),
	DEVMETHOD(device_attach,	apple_fpwm_attach),
	DEVMETHOD(device_detach,	apple_fpwm_detach),

	DEVMETHOD(ofw_bus_get_node,	apple_fpwm_get_node),

	DEVMETHOD(pwmbus_channel_count,		apple_fpwm_channel_count),
	DEVMETHOD(pwmbus_channel_config,	apple_fpwm_channel_config),
	DEVMETHOD(pwmbus_channel_get_config,	apple_fpwm_channel_get_config),
	DEVMETHOD(pwmbus_channel_enable,	apple_fpwm_channel_enable),
	DEVMETHOD(pwmbus_channel_is_enabled,	apple_fpwm_channel_is_enabled),

	DEVMETHOD_END
};

static driver_t apple_fpwm_driver = {
	"pwm",
	apple_fpwm_methods,
	sizeof(struct apple_fpwm_softc),
};

/* Before pwm_leds, which looks up our xref. */
EARLY_DRIVER_MODULE(apple_fpwm, simplebus, apple_fpwm_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
MODULE_VERSION(apple_fpwm, 1);
SIMPLEBUS_PNP_INFO(compat_data);
