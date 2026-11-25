/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Minimal Apple PMGR power-state support: switch on the power domains a
 * device lists in "power-domains" (parents first). Domains are never switched
 * off again; there is no power-domain framework to track users.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <arm64/apple/apple_pmgr.h>

#define	PMGR_RESET		(1u << 31)
#define	PMGR_AUTO_ENABLE	(1u << 28)
#define	PMGR_PS_RESET		(1u << 12)
#define	PMGR_BUSY		(1u << 11)	/* hardware's; never written back */
#define	PMGR_DEV_DISABLE	(1u << 10)
#define	PMGR_WAS_CLKGATED	(1u << 9)
#define	PMGR_WAS_PWRGATED	(1u << 8)
#define	PMGR_FLAGS		(PMGR_WAS_CLKGATED | PMGR_WAS_PWRGATED)
#define	PMGR_PS_ACTUAL(r)	(((r) >> 4) & 0xf)
#define	PMGR_PS_TARGET		0xf
#define	PMGR_PS_ACTIVE		0xf

#define	PMGR_MAX_DEPTH		8

static int apple_pmgr_enable_domains(device_t dev, phandle_t node, int depth);

static int
apple_pmgr_enable_one(device_t dev, phandle_t ps, int depth)
{
	bus_addr_t pa;
	bus_size_t size;
	pcell_t reg;
	vm_offset_t va;
	uint32_t old, val;
	int error, i;

	error = apple_pmgr_enable_domains(dev, ps, depth + 1);
	if (error != 0)
		return (error);

	if (!ofw_bus_node_is_compatible(ps, "apple,t8103-pmgr-pwrstate") ||
	    OF_getencprop(ps, "reg", &reg, sizeof(reg)) <= 0 ||
	    ofw_reg_to_paddr(OF_parent(ps), 0, &pa, &size, NULL) != 0)
		return (ENXIO);
	pa += reg;

	va = (vm_offset_t)pmap_mapdev(pa, sizeof(uint32_t));
	old = *(volatile uint32_t *)va;
	if (PMGR_PS_ACTUAL(old) != PMGR_PS_ACTIVE ||
	    (old & (PMGR_RESET | PMGR_PS_RESET | PMGR_DEV_DISABLE)) != 0) {
		/* As aurora's apple_pmgr_ps_set for the active state. */
		val = old & ~(PMGR_FLAGS | PMGR_RESET | PMGR_PS_TARGET |
		    PMGR_AUTO_ENABLE | PMGR_PS_RESET | PMGR_BUSY |
		    PMGR_DEV_DISABLE);
		*(volatile uint32_t *)va = val | PMGR_PS_ACTIVE;
		for (i = 0; i < 100000; i++) {
			val = *(volatile uint32_t *)va;
			if (PMGR_PS_ACTUAL(val) == PMGR_PS_ACTIVE)
				break;
			DELAY(1);
		}
		if (bootverbose || PMGR_PS_ACTUAL(val) != PMGR_PS_ACTIVE)
			device_printf(dev, "power domain %#lx: %#x -> %#x\n",
			    pa, old, val);
		if (PMGR_PS_ACTUAL(val) != PMGR_PS_ACTIVE)
			error = ETIMEDOUT;
	}
	pmap_unmapdev((void *)va, sizeof(uint32_t));
	return (error);
}

static int
apple_pmgr_enable_domains(device_t dev, phandle_t node, int depth)
{
	pcell_t *cells;
	int i, n, error;

	if (depth > PMGR_MAX_DEPTH)
		return (ELOOP);
	n = OF_getencprop_alloc_multi(node, "power-domains", sizeof(*cells),
	    (void **)&cells);
	if (n <= 0)
		return (0);
	error = 0;
	for (i = 0; i < n && error == 0; i++)
		error = apple_pmgr_enable_one(dev, OF_node_from_xref(cells[i]),
		    depth);
	OF_prop_free(cells);
	return (error);
}

/* Switch on every power domain dev depends on. */
int
apple_pmgr_power_on(device_t dev)
{

	return (apple_pmgr_power_on_node(dev, ofw_bus_get_node(dev)));
}

int
apple_pmgr_power_on_node(device_t dev, phandle_t node)
{

	return (apple_pmgr_enable_domains(dev, node, 0));
}

/*
 * Switch one power domain, the power-domains entry #index of dev's node, on
 * or off, without its parents (which must already be on). For drivers that
 * cycle a coprocessor's domains themselves (apple_isp).
 */
int
apple_pmgr_set_domain(device_t dev, int index, bool on)
{
	bus_addr_t pa;
	bus_size_t size;
	pcell_t *cells, reg;
	phandle_t ps;
	vm_offset_t va;
	uint32_t old, val, want;
	int error, i, n;

	n = OF_getencprop_alloc_multi(ofw_bus_get_node(dev), "power-domains",
	    sizeof(*cells), (void **)&cells);
	if (n <= index) {
		OF_prop_free(cells);
		return (ENOENT);
	}
	ps = OF_node_from_xref(cells[index]);
	OF_prop_free(cells);
	if (!ofw_bus_node_is_compatible(ps, "apple,t8103-pmgr-pwrstate") ||
	    OF_getencprop(ps, "reg", &reg, sizeof(reg)) <= 0 ||
	    ofw_reg_to_paddr(OF_parent(ps), 0, &pa, &size, NULL) != 0)
		return (ENXIO);
	pa += reg;

	want = on ? PMGR_PS_ACTIVE : 0;
	error = 0;
	va = (vm_offset_t)pmap_mapdev(pa, sizeof(uint32_t));
	old = *(volatile uint32_t *)va;
	val = old & ~(PMGR_FLAGS | PMGR_RESET | PMGR_PS_TARGET |
	    PMGR_AUTO_ENABLE | PMGR_PS_RESET | PMGR_BUSY | PMGR_DEV_DISABLE);
	*(volatile uint32_t *)va = val | want;
	for (i = 0; i < 100000; i++) {
		val = *(volatile uint32_t *)va;
		if (PMGR_PS_ACTUAL(val) == want)
			break;
		DELAY(1);
	}
	if (bootverbose || PMGR_PS_ACTUAL(val) != want)
		device_printf(dev, "power domain %#lx %s: %#x -> %#x\n", pa,
		    on ? "on" : "off", old, val);
	if (PMGR_PS_ACTUAL(val) != want)
		error = ETIMEDOUT;
	pmap_unmapdev((void *)va, sizeof(uint32_t));
	return (error);
}
