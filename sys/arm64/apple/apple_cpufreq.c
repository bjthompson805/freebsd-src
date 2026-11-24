/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple cluster DVFS ("apple,t8142-cluster-cpufreq" performance controllers):
 * one cpufreq instance per CPU cluster, attached to the cluster's first CPU.
 * A request writes the OPP's level into the PS1 field of the cluster's DVFS
 * command register together with SET, then waits for BUSY to clear and the
 * command to read back the new level, as aurora-silicon's Linux
 * apple-soc-cpufreq does for T8142 (soc_t8142_info: verified transitions,
 * levels 1-31). Voltage, PLLs and thermal limits stay with the firmware. If
 * the command register ever holds a level we did not request, someone else
 * owns the cluster and no further requests are made.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include "cpufreq_if.h"

#define	APPLE_DVFS_CMD		0x20
#define	 APPLE_DVFS_CMD_BUSY	(1ULL << 31)
#define	 APPLE_DVFS_CMD_SET	(1ULL << 25)
#define	 APPLE_DVFS_CMD_PS1	0x1fULL
#define	APPLE_DVFS_TIMEOUT_US	400
#define	APPLE_DVFS_MAX_OPPS	32

struct apple_cpufreq_opp {
	uint64_t	hz;
	uint32_t	level;
};

struct apple_cpufreq_softc {
	device_t		dev;
	struct mtx		mtx;
	vm_paddr_t		pa;
	vm_size_t		size;
	volatile uint64_t	*regs;
	int			nopp;
	struct apple_cpufreq_opp opp[APPLE_DVFS_MAX_OPPS];	/* fastest first */
	uint32_t		expected;
	bool			failed;
};

static const struct ofw_compat_data compat_data[] = {
	{ "apple,t8142-cluster-cpufreq",	1 },
	{ NULL,					0 }
};

/* The performance controller node a CPU node points at, or 0. */
static phandle_t
apple_cpufreq_domain(phandle_t cpu)
{
	pcell_t xref;

	if (OF_getencprop(cpu, "performance-domains", &xref, sizeof(xref)) !=
	    sizeof(xref))
		return (0);
	return (OF_node_from_xref(xref));
}

/* Is this CPU node the first one in its performance domain? */
static bool
apple_cpufreq_first_in_domain(phandle_t cpu, phandle_t domain)
{
	phandle_t child;

	for (child = OF_child(OF_parent(cpu)); child > 0;
	    child = OF_peer(child)) {
		if (apple_cpufreq_domain(child) == domain)
			return (child == cpu);
	}
	return (false);
}

static bool
apple_cpufreq_wanted(device_t cpudev)
{
	phandle_t cpu, domain;

	cpu = ofw_bus_get_node(cpudev);
	if (cpu <= 0 || (domain = apple_cpufreq_domain(cpu)) <= 0)
		return (false);
	if (ofw_bus_node_is_compatible(domain, compat_data[0].ocd_str) == 0)
		return (false);
	return (apple_cpufreq_first_in_domain(cpu, domain));
}

static void
apple_cpufreq_identify(driver_t *driver, device_t parent)
{

	if (!apple_cpufreq_wanted(parent))
		return;
	if (device_find_child(parent, "apple_cpufreq", DEVICE_UNIT_ANY) != NULL)
		return;
	if (BUS_ADD_CHILD(parent, 0, "apple_cpufreq", DEVICE_UNIT_ANY) == NULL)
		device_printf(parent, "add apple_cpufreq child failed\n");
}

static int
apple_cpufreq_probe(device_t dev)
{

	if (!apple_cpufreq_wanted(device_get_parent(dev)))
		return (ENXIO);
	device_set_desc(dev, "Apple cluster DVFS");
	return (BUS_PROBE_DEFAULT);
}

static uint64_t
apple_cpufreq_read(struct apple_cpufreq_softc *sc)
{

	return (sc->regs[APPLE_DVFS_CMD / 8]);
}

/* Waits for BUSY to clear (and, if want >= 0, for PS1 == want). */
static int
apple_cpufreq_wait(struct apple_cpufreq_softc *sc, int want, uint64_t *cmdp)
{
	uint64_t cmd;

	for (int i = 0; i <= APPLE_DVFS_TIMEOUT_US; i++) {
		cmd = apple_cpufreq_read(sc);
		if ((cmd & APPLE_DVFS_CMD_BUSY) == 0 &&
		    (want < 0 || (cmd & APPLE_DVFS_CMD_PS1) == (uint64_t)want)) {
			*cmdp = cmd;
			return (0);
		}
		DELAY(1);
	}
	*cmdp = cmd;
	return (ETIMEDOUT);
}

static int
apple_cpufreq_set_level(struct apple_cpufreq_softc *sc, uint32_t level)
{
	uint64_t cmd;
	int error;

	mtx_lock_spin(&sc->mtx);
	if (sc->failed) {
		error = EIO;
		goto out;
	}
	if ((error = apple_cpufreq_wait(sc, -1, &cmd)) != 0) {
		sc->failed = true;
		device_printf(sc->dev, "DVFS busy timeout, cmd %#jx; "
		    "stopping requests\n", (uintmax_t)cmd);
		goto out;
	}
	if ((cmd & APPLE_DVFS_CMD_PS1) != sc->expected) {
		/* Someone else changed the cluster's level: don't fight it. */
		sc->failed = true;
		device_printf(sc->dev, "unexpected DVFS level %ju (expected "
		    "%u); stopping requests\n",
		    (uintmax_t)(cmd & APPLE_DVFS_CMD_PS1), sc->expected);
		error = EIO;
		goto out;
	}
	if ((cmd & APPLE_DVFS_CMD_PS1) == level)
		goto out;
	cmd = (cmd & ~APPLE_DVFS_CMD_PS1) | level | APPLE_DVFS_CMD_SET;
	sc->regs[APPLE_DVFS_CMD / 8] = cmd;
	if ((error = apple_cpufreq_wait(sc, level, &cmd)) != 0) {
		sc->failed = true;
		device_printf(sc->dev, "DVFS completion timeout, cmd %#jx; "
		    "stopping requests\n", (uintmax_t)cmd);
		goto out;
	}
	sc->expected = level;
out:
	mtx_unlock_spin(&sc->mtx);
	return (error);
}

static void
apple_cpufreq_to_setting(struct apple_cpufreq_softc *sc,
    const struct apple_cpufreq_opp *opp, struct cf_setting *set)
{

	memset(set, CPUFREQ_VAL_UNKNOWN, sizeof(*set));
	set->freq = opp->hz / 1000000;
	set->lat = 50;
	set->dev = sc->dev;
}

static int
apple_cpufreq_get(device_t dev, struct cf_setting *set)
{
	struct apple_cpufreq_softc *sc = device_get_softc(dev);
	uint32_t level;

	level = apple_cpufreq_read(sc) & APPLE_DVFS_CMD_PS1;
	for (int i = 0; i < sc->nopp; i++)
		if (sc->opp[i].level == level) {
			apple_cpufreq_to_setting(sc, &sc->opp[i], set);
			return (0);
		}
	return (ENOENT);
}

static int
apple_cpufreq_set(device_t dev, const struct cf_setting *set)
{
	struct apple_cpufreq_softc *sc = device_get_softc(dev);

	for (int i = 0; i < sc->nopp; i++)
		if (sc->opp[i].hz / 1000000 == (uint64_t)set->freq)
			return (apple_cpufreq_set_level(sc, sc->opp[i].level));
	return (EINVAL);
}

static int
apple_cpufreq_type(device_t dev, int *type)
{

	*type = CPUFREQ_TYPE_ABSOLUTE;
	return (0);
}

static int
apple_cpufreq_settings(device_t dev, struct cf_setting *sets, int *count)
{
	struct apple_cpufreq_softc *sc = device_get_softc(dev);

	if (*count < sc->nopp)
		return (E2BIG);
	for (int i = 0; i < sc->nopp; i++)
		apple_cpufreq_to_setting(sc, &sc->opp[i], &sets[i]);
	*count = sc->nopp;
	return (0);
}

/* Reads the CPU node's operating-points-v2 table, fastest first. */
static int
apple_cpufreq_parse_opps(struct apple_cpufreq_softc *sc, phandle_t cpu)
{
	phandle_t table, opp;
	pcell_t xref, hz[2], level;

	if (OF_getencprop(cpu, "operating-points-v2", &xref, sizeof(xref)) !=
	    sizeof(xref) || (table = OF_node_from_xref(xref)) <= 0)
		return (ENXIO);
	sc->nopp = 0;
	for (opp = OF_child(table); opp > 0 && sc->nopp < APPLE_DVFS_MAX_OPPS;
	    opp = OF_peer(opp)) {
		struct apple_cpufreq_opp o;
		int j;

		if (OF_getencprop(opp, "opp-hz", hz, sizeof(hz)) != sizeof(hz) ||
		    OF_getencprop(opp, "opp-level", &level, sizeof(level)) !=
		    sizeof(level))
			continue;
		o.hz = ((uint64_t)hz[0] << 32) | hz[1];
		o.level = level;
		if (o.hz == 0 || o.level == 0 || o.level > APPLE_DVFS_CMD_PS1)
			continue;
		/* Insertion sort, fastest first. */
		for (j = sc->nopp; j > 0 && sc->opp[j - 1].hz < o.hz; j--)
			sc->opp[j] = sc->opp[j - 1];
		sc->opp[j] = o;
		sc->nopp++;
	}
	return (sc->nopp > 0 ? 0 : ENXIO);
}

static int
apple_cpufreq_attach(device_t dev)
{
	struct apple_cpufreq_softc *sc = device_get_softc(dev);
	phandle_t cpu, domain;
	uint32_t level;
	int error;

	sc->dev = dev;
	cpu = ofw_bus_get_node(device_get_parent(dev));
	domain = apple_cpufreq_domain(cpu);
	if ((error = apple_cpufreq_parse_opps(sc, cpu)) != 0) {
		device_printf(dev, "no usable operating-points-v2 table\n");
		return (error);
	}
	{
		bus_addr_t pa;
		bus_size_t size;

		if (ofw_reg_to_paddr(domain, 0, &pa, &size, NULL) != 0) {
			device_printf(dev, "no performance controller "
			    "registers\n");
			return (ENXIO);
		}
		sc->pa = pa;
		sc->size = size;
	}
	sc->regs = pmap_mapdev(sc->pa, sc->size);
	mtx_init(&sc->mtx, "apple_cpufreq", NULL, MTX_SPIN);

	level = apple_cpufreq_read(sc) & APPLE_DVFS_CMD_PS1;
	sc->expected = level;
	device_printf(dev, "cluster DVFS at %#jx: %d levels, %ju-%ju MHz, "
	    "current level %u\n", (uintmax_t)sc->pa, sc->nopp,
	    (uintmax_t)(sc->opp[sc->nopp - 1].hz / 1000000),
	    (uintmax_t)(sc->opp[0].hz / 1000000), level);

	cpufreq_register(dev);
	return (0);
}

static device_method_t apple_cpufreq_methods[] = {
	DEVMETHOD(device_identify,	apple_cpufreq_identify),
	DEVMETHOD(device_probe,		apple_cpufreq_probe),
	DEVMETHOD(device_attach,	apple_cpufreq_attach),

	DEVMETHOD(cpufreq_drv_get,	apple_cpufreq_get),
	DEVMETHOD(cpufreq_drv_set,	apple_cpufreq_set),
	DEVMETHOD(cpufreq_drv_type,	apple_cpufreq_type),
	DEVMETHOD(cpufreq_drv_settings,	apple_cpufreq_settings),

	DEVMETHOD_END
};

static driver_t apple_cpufreq_driver = {
	"apple_cpufreq",
	apple_cpufreq_methods,
	sizeof(struct apple_cpufreq_softc),
};

DRIVER_MODULE(apple_cpufreq, cpu, apple_cpufreq_driver, 0, 0);
