/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple PMC voter interface (T8142), after Asahi/Aurora
 * drivers/pmdomain/apple/pmc.c.
 *
 * The PMC sets the performance state of shared rails (the DRAM controller,
 * DCS, and the SoC fabric) from the floors of voter agents: PMP 0, CLPC 1,
 * GFX 2 and AP 3 on the M5. It honours an agent's floor only while that
 * agent's bit is set in the rail's interface enable word; iBoot leaves them
 * all clear, so DRAM and fabric stay at their lowest states.
 *
 * At attach it logs every agent's floor and state word and each rail's
 * enable word (dev.apple_pmc.0.dump re-reads them), then enables the
 * static voters (hw.apple_pmc.static_voters). dev.apple_pmc.0.enable
 * writes an agent mask to every rail's enable word and reads it back, as
 * pmc.c apple_pmc_enable_write does: 0xb is the static voters (PMP, CLPC,
 * AP), 0xf adds GFX.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/systm.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <machine/bus.h>

/* pmc.c apple_pmc_hw_t8142 */
#define	PMC_AGENTS		4
#define	PMC_RAILS		3
#define	PMC_AGENT_STRIDE	0x4000
#define	PMC_AGENT_STATE		0x0	/* in "rails" */
#define	PMC_RAIL_ENABLE		0x2000	/* in "rails" */
#define	PMC_CLPC_AGENT		1

static const char *pmc_agent_names[PMC_AGENTS] = { "PMP", "CLPC", "GFX",
    "AP" };

/*
 * Agents enabled at attach: the DT's apple,static-voters (PMP, CLPC, AP),
 * as pmc.c's probe does. With them the PMP's DCS vote holds: CPU DRAM
 * latency 380 -> 207 ns, Hyprland's 2880x1800 composite 7.4 -> 1.4 ms.
 * -1 leaves the enables as iBoot left them (all clear).
 */
static int apple_pmc_static = 0xb;
SYSCTL_NODE(_hw, OID_AUTO, apple_pmc, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Apple PMC");
SYSCTL_INT(_hw_apple_pmc, OID_AUTO, static_voters, CTLFLAG_RDTUN,
    &apple_pmc_static, 0, "Voter agents enabled at attach (-1: none)");

struct apple_pmc_softc {
	device_t		dev;
	struct resource		*voters;
	struct resource		*rails;
};

static struct ofw_compat_data compat_data[] = {
	{ "apple,t8142-pmc",	1 },
	{ NULL,			0 }
};

static void
apple_pmc_dump(struct apple_pmc_softc *sc)
{
	u_int i;

	for (i = 0; i < PMC_AGENTS; i++)
		device_printf(sc->dev, "agent %u (%s): floor %#x state %#x\n",
		    i, pmc_agent_names[i],
		    bus_read_4(sc->voters, i * PMC_AGENT_STRIDE),
		    bus_read_4(sc->rails, PMC_AGENT_STATE + 4 * i));
	for (i = 0; i < PMC_RAILS; i++)
		device_printf(sc->dev, "rail %u: enable %#x\n", i,
		    bus_read_4(sc->rails, PMC_RAIL_ENABLE + 4 * i));
}

static int
apple_pmc_sysctl_dump(SYSCTL_HANDLER_ARGS)
{
	struct apple_pmc_softc *sc = arg1;
	int error, v = 0;

	error = sysctl_handle_int(oidp, &v, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	apple_pmc_dump(sc);
	return (0);
}

/* Every write is read back, as macOS does. */
static int
apple_pmc_set_enable(struct apple_pmc_softc *sc, uint32_t mask)
{
	uint32_t got;
	u_int rail;

	for (rail = 0; rail < PMC_RAILS; rail++) {
		bus_write_4(sc->rails, PMC_RAIL_ENABLE + 4 * rail, mask);
		got = bus_read_4(sc->rails, PMC_RAIL_ENABLE + 4 * rail);
		if (got != mask) {
			device_printf(sc->dev, "rail %u enable %#x did not "
			    "stick (%#x)\n", rail, mask, got);
			return (EIO);
		}
	}
	return (0);
}

static int
apple_pmc_sysctl_enable(SYSCTL_HANDLER_ARGS)
{
	struct apple_pmc_softc *sc = arg1;
	int error, mask;

	mask = bus_read_4(sc->rails, PMC_RAIL_ENABLE);
	error = sysctl_handle_int(oidp, &mask, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (mask < 0 || mask >= (1 << PMC_AGENTS))
		return (EINVAL);
	return (apple_pmc_set_enable(sc, mask));
}

/*
 * The CLPC agent's floor word: 4 bits per rail (DCS in 3:0, fabric in
 * 7:4). Experiment knob until a CLPC policy exists; clpc.c's T8142 map
 * reaches DCS 7 and fabric 4 for a busy CPU at its top quartile.
 */
static int
apple_pmc_sysctl_clpc_floor(SYSCTL_HANDLER_ARGS)
{
	struct apple_pmc_softc *sc = arg1;
	int error, floor;

	floor = bus_read_4(sc->voters, PMC_CLPC_AGENT * PMC_AGENT_STRIDE);
	error = sysctl_handle_int(oidp, &floor, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (floor < 0 || floor > 0xfff)
		return (EINVAL);
	bus_write_4(sc->voters, PMC_CLPC_AGENT * PMC_AGENT_STRIDE, floor);
	return (bus_read_4(sc->voters, PMC_CLPC_AGENT * PMC_AGENT_STRIDE) ==
	    (uint32_t)floor ? 0 : EIO);
}

static int
apple_pmc_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Apple PMC voter interface");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_pmc_attach(device_t dev)
{
	struct apple_pmc_softc *sc = device_get_softc(dev);
	int rid;

	sc->dev = dev;
	rid = 0;
	sc->voters = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	rid = 1;
	sc->rails = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->voters == NULL || sc->rails == NULL) {
		device_printf(dev, "cannot map registers\n");
		if (sc->voters != NULL)
			bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->voters);
		if (sc->rails != NULL)
			bus_release_resource(dev, SYS_RES_MEMORY, 1, sc->rails);
		return (ENXIO);
	}
	apple_pmc_dump(sc);
	if (apple_pmc_static >= 0 && apple_pmc_static < (1 << PMC_AGENTS) &&
	    apple_pmc_set_enable(sc, apple_pmc_static) == 0)
		device_printf(dev, "voter agents %#x enabled\n",
		    apple_pmc_static);
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "dump",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    apple_pmc_sysctl_dump, "I", "Write 1 to log the voter registers");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "enable",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    apple_pmc_sysctl_enable, "I", "Voter agents enabled on every rail");
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "clpc_floor",
	    CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    apple_pmc_sysctl_clpc_floor, "I", "CLPC agent floor word");
	return (0);
}

/* The enables stay as they are: the rails keep their votes. */
static int
apple_pmc_detach(device_t dev)
{
	struct apple_pmc_softc *sc = device_get_softc(dev);

	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->voters);
	bus_release_resource(dev, SYS_RES_MEMORY, 1, sc->rails);
	return (0);
}

static device_method_t apple_pmc_methods[] = {
	DEVMETHOD(device_probe,		apple_pmc_probe),
	DEVMETHOD(device_attach,	apple_pmc_attach),
	DEVMETHOD(device_detach,	apple_pmc_detach),
	DEVMETHOD_END
};

static driver_t apple_pmc_driver = {
	"apple_pmc",
	apple_pmc_methods,
	sizeof(struct apple_pmc_softc),
};

DRIVER_MODULE(apple_pmc, simplebus, apple_pmc_driver, 0, 0);
