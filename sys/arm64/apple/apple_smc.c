/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Placeholder for the Apple SMC: no RTKit session yet, it only enumerates
 * its DT children (rtc, gpio, reboot, ...) like a simple-bus, so drivers
 * that don't need the SMC itself (apple_pmu_rtc) can attach.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/simplebus.h>

static int
apple_smc_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "apple,t8142-smc"))
		return (ENXIO);
	device_set_desc(dev, "Apple SMC (children only)");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_smc_attach(device_t dev)
{
	phandle_t node, child;

	node = ofw_bus_get_node(dev);
	simplebus_init(dev, node);
	for (child = OF_child(node); child > 0; child = OF_peer(child))
		simplebus_add_device(dev, child, 0, NULL, -1, NULL);
	bus_attach_children(dev);
	return (0);
}

static device_method_t apple_smc_methods[] = {
	DEVMETHOD(device_probe,		apple_smc_probe),
	DEVMETHOD(device_attach,	apple_smc_attach),
	DEVMETHOD_END
};

DEFINE_CLASS_1(apple_smc, apple_smc_driver, apple_smc_methods,
    sizeof(struct simplebus_softc), simplebus_driver);
EARLY_DRIVER_MODULE(apple_smc, simplebus, apple_smc_driver, NULL, NULL,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
