/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * pwm-leds: an LED dimmed by a PWM channel, exposed through backlight(9) as
 * /dev/backlight/<label><unit> (percent of max-brightness). Only the first
 * child LED node is used; Apple laptops have one, the keyboard backlight.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/backlight.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/backlight/backlight.h>
#include <dev/pwm/ofw_pwm.h>

#include "backlight_if.h"
#include "pwmbus_if.h"

struct pwm_leds_softc {
	device_t	dev;
	pwm_channel_t	channel;
	struct cdev	*cdev;
	char		label[BACKLIGHTMAXNAMELENGTH];
	int		brightness;	/* percent */
	bool		keyboard;
};

static struct ofw_compat_data compat_data[] = {
	{ "pwm-leds",	1 },
	{ NULL,		0 }
};

static int
pwm_leds_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "PWM LED");
	return (BUS_PROBE_DEFAULT);
}

static int
pwm_leds_attach(device_t dev)
{
	struct pwm_leds_softc *sc = device_get_softc(dev);
	phandle_t child;
	char func[32];
	u_int period, duty;
	bool enabled;
	int error;

	sc->dev = dev;
	for (child = OF_child(ofw_bus_get_node(dev)); child != 0;
	    child = OF_peer(child))
		if (ofw_bus_node_status_okay(child) &&
		    OF_hasprop(child, "pwms"))
			break;
	if (child == 0) {
		device_printf(dev, "no LED node\n");
		return (ENXIO);
	}
	error = pwm_get_by_ofw_propidx(dev, child, "pwms", 0, &sc->channel);
	if (error != 0) {
		device_printf(dev, "cannot get the PWM channel: %d\n", error);
		return (error);
	}
	if (OF_getprop(child, "label", sc->label, sizeof(sc->label)) <= 0)
		strlcpy(sc->label, "pwm_led", sizeof(sc->label));
	if (OF_getprop(child, "function", func, sizeof(func)) > 0)
		sc->keyboard = strcmp(func, "kbd_backlight") == 0;

	/* default-state "keep": start from what the controller is doing. */
	if (PWMBUS_CHANNEL_GET_CONFIG(sc->channel->dev, sc->channel->channel,
	    &period, &duty) == 0 &&
	    PWMBUS_CHANNEL_IS_ENABLED(sc->channel->dev, sc->channel->channel,
	    &enabled) == 0 && enabled && period != 0)
		sc->brightness = (uint64_t)duty * 100 / period;
	else
		sc->brightness = 0;

	sc->cdev = backlight_register(sc->label, dev);
	if (sc->cdev == NULL) {
		device_printf(dev, "cannot register as a backlight\n");
		free(sc->channel, M_DEVBUF);
		return (ENXIO);
	}
	return (0);
}

static int
pwm_leds_detach(device_t dev)
{
	struct pwm_leds_softc *sc = device_get_softc(dev);

	backlight_destroy(sc->cdev);
	free(sc->channel, M_DEVBUF);
	return (0);
}

static int
pwm_leds_update_status(device_t dev, struct backlight_props *props)
{
	struct pwm_leds_softc *sc = device_get_softc(dev);
	u_int period, duty;
	int error;

	if (props->brightness < 0 || props->brightness > 100)
		return (ERANGE);
	period = sc->channel->period;
	duty = (uint64_t)period * props->brightness / 100;
	if (props->brightness == 0)
		error = PWMBUS_CHANNEL_ENABLE(sc->channel->dev,
		    sc->channel->channel, false);
	else if ((error = PWMBUS_CHANNEL_CONFIG(sc->channel->dev,
	    sc->channel->channel, period, duty)) == 0)
		error = PWMBUS_CHANNEL_ENABLE(sc->channel->dev,
		    sc->channel->channel, true);
	if (error == 0)
		sc->brightness = props->brightness;
	return (error);
}

static int
pwm_leds_get_status(device_t dev, struct backlight_props *props)
{
	struct pwm_leds_softc *sc = device_get_softc(dev);

	props->brightness = sc->brightness;
	props->nlevels = 0;
	return (0);
}

static int
pwm_leds_get_info(device_t dev, struct backlight_info *info)
{
	struct pwm_leds_softc *sc = device_get_softc(dev);

	info->type = sc->keyboard ? BACKLIGHT_TYPE_KEYBOARD :
	    BACKLIGHT_TYPE_PANEL;
	strlcpy(info->name, sc->label, BACKLIGHTMAXNAMELENGTH);
	return (0);
}

static device_method_t pwm_leds_methods[] = {
	DEVMETHOD(device_probe,		pwm_leds_probe),
	DEVMETHOD(device_attach,	pwm_leds_attach),
	DEVMETHOD(device_detach,	pwm_leds_detach),

	DEVMETHOD(backlight_update_status,	pwm_leds_update_status),
	DEVMETHOD(backlight_get_status,		pwm_leds_get_status),
	DEVMETHOD(backlight_get_info,		pwm_leds_get_info),

	DEVMETHOD_END
};

static driver_t pwm_leds_driver = {
	"pwm_leds",
	pwm_leds_methods,
	sizeof(struct pwm_leds_softc),
};

DRIVER_MODULE(pwm_leds, simplebus, pwm_leds_driver, 0, 0);
MODULE_DEPEND(pwm_leds, backlight, 1, 1, 1);
SIMPLEBUS_PNP_INFO(compat_data);
