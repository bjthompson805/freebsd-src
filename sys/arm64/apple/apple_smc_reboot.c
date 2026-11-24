/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple SMC power off: /smc/reboot { compatible = "apple,smc-reboot"; }.
 * As aurora's macsmc-reboot: set the PMU shutdown flag (else the machine
 * powers straight back on), then write MBSE = off1. Restart stays with
 * apple_wdog.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/eventhandler.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/reboot.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <arm64/apple/apple_smc.h>

struct apple_smc_reboot_softc {
	device_t	dev;
	uint16_t	flag_reg;
	uint8_t		flag_mask;
};

static void
apple_smc_reboot_poweroff(void *arg, int howto)
{
	struct apple_smc_reboot_softc *sc = arg;
	int error;

	if ((howto & RB_POWEROFF) == 0)
		return;

	error = apple_pmu_update(sc->flag_reg, sc->flag_mask, sc->flag_mask);
	if (error != 0)
		device_printf(sc->dev, "can't set shutdown flag: %d\n", error);
	printf("Powering off (SMC off1) ... ");
	error = apple_smc_write_u32(APPLE_SMC_KEY("MBSE"), APPLE_SMC_KEY("off1"));
	if (error != 0) {
		printf("failed: %d\n", error);
		return;
	}
	DELAY(2000000);
	printf("still on.\n");
}

static int
apple_smc_reboot_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "apple,smc-reboot"))
		return (ENXIO);
	device_set_desc(dev, "Apple SMC power off");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_smc_reboot_attach(device_t dev)
{
	struct apple_smc_reboot_softc *sc = device_get_softc(dev);
	phandle_t node = ofw_bus_get_node(dev), cell;
	pcell_t cells[8], reg[2], bits[2];
	int i, n;

	sc->dev = dev;
	/* The "shutdown_flag" cell: one bit of a PMU scratch byte. */
	n = OF_getencprop(node, "nvmem-cells", cells, sizeof(cells));
	if (ofw_bus_find_string_index(node, "nvmem-cell-names",
	    "shutdown_flag", &i) != 0 || n < (i + 1) * (int)sizeof(pcell_t) ||
	    (cell = OF_node_from_xref(cells[i])) <= 0 ||
	    OF_getencprop(cell, "reg", reg, sizeof(reg)) != sizeof(reg) ||
	    reg[1] != 1 ||
	    OF_getencprop(cell, "bits", bits, sizeof(bits)) != sizeof(bits) ||
	    bits[0] + bits[1] > 8) {
		device_printf(dev, "no usable shutdown_flag cell\n");
		return (ENXIO);
	}
	sc->flag_reg = reg[0];
	sc->flag_mask = ((1u << bits[1]) - 1) << bits[0];
	device_printf(dev, "shutdown flag %#x mask %#x\n", sc->flag_reg,
	    sc->flag_mask);
	EVENTHANDLER_REGISTER(shutdown_final, apple_smc_reboot_poweroff, sc,
	    SHUTDOWN_PRI_LAST);
	return (0);
}

static device_method_t apple_smc_reboot_methods[] = {
	DEVMETHOD(device_probe,		apple_smc_reboot_probe),
	DEVMETHOD(device_attach,	apple_smc_reboot_attach),
	DEVMETHOD_END
};

static driver_t apple_smc_reboot_driver = {
	"apple_smc_reboot",
	apple_smc_reboot_methods,
	sizeof(struct apple_smc_reboot_softc),
};

DRIVER_MODULE(apple_smc_reboot, apple_smc, apple_smc_reboot_driver, NULL, NULL);
