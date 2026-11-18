/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Read-only time-of-day clock for Apple PMUs on SPMI (Abbey, M5): the PMU
 * counter runs at 65536 Hz from an arbitrary base; the wall-clock offset that
 * macOS maintains lives in PMU scratch space as a signed 48-bit count of
 * 1/32768 s. The DT describes it as /rtc { nvmem-cells = <&rtc_offset>; },
 * with the cell inside the PMIC node on the SPMI controller; this walks that
 * path and talks to the controller directly.
 *
 * Setting the clock is refused: iBoot also reads the PMU scratch area.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/clock.h>
#include <sys/endian.h>
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

#include "clock_if.h"

#define	SPMI_CMD_EXT_READL	0x38
#define	SPMI_CMD_PARITY		(1u << 15)
#define	SPMI_REPLY_CMD(r)	((r) & 0xff)
#define	SPMI_REPLY_SID(r)	(((r) >> 8) & 0x7f)

#define	PMU_ABBEY_COUNTER	0x2002	/* 48 bits, 65536 Hz */
#define	PMU_HZ_SHIFT		16

struct spmi_regs {
	bus_size_t	status, command, reply;
	uint32_t	rx_empty;
};

static const struct spmi_regs spmi_gen1 = { 0x0, 0x4, 0x8, 1u << 24 };
static const struct spmi_regs spmi_gen4 = { 0x200, 0x210, 0x220, 1u << 30 };

struct apple_pmu_rtc_softc {
	device_t		dev;
	struct mtx		mtx;
	vm_offset_t		spmi;
	const struct spmi_regs	*regs;
	uint8_t			sid;
	uint16_t		offset_reg;
};

#define	SPMI_RD4(sc, o)		(*(volatile uint32_t *)((sc)->spmi + (o)))
#define	SPMI_WR4(sc, o, v)	(*(volatile uint32_t *)((sc)->spmi + (o)) = (v))

static bool
spmi_rx_empty(struct apple_pmu_rtc_softc *sc)
{
	return ((SPMI_RD4(sc, sc->regs->status) & sc->regs->rx_empty) != 0);
}

/* Extended register read long: up to 8 bytes from a 16-bit address. */
static int
spmi_readl(struct apple_pmu_rtc_softc *sc, uint16_t addr, uint8_t *buf,
    size_t len)
{
	uint8_t opc = SPMI_CMD_EXT_READL | (len - 1);
	uint32_t reply, word;
	size_t i;
	int n;

	mtx_assert(&sc->mtx, MA_OWNED);
	/* Drop anything a failed exchange left behind. */
	for (n = 0; n < 64 && !spmi_rx_empty(sc); n++)
		(void)SPMI_RD4(sc, sc->regs->reply);

	SPMI_WR4(sc, sc->regs->command, opc | sc->sid << 8 |
	    (uint32_t)addr << 16 | SPMI_CMD_PARITY);
	for (n = 0; spmi_rx_empty(sc); n++) {
		if (n == 5000)
			return (ETIMEDOUT);
		DELAY(10);
	}
	reply = SPMI_RD4(sc, sc->regs->reply);
	if (SPMI_REPLY_CMD(reply) != opc || SPMI_REPLY_SID(reply) != sc->sid)
		return (EIO);
	for (i = 0; i < len; i += 4) {
		if (spmi_rx_empty(sc))
			return (EIO);
		word = SPMI_RD4(sc, sc->regs->reply);
		memcpy(buf + i, &word, MIN(4, len - i));
	}
	return (0);
}

static int
apple_pmu_rtc_gettime(device_t dev, struct timespec *ts)
{
	struct apple_pmu_rtc_softc *sc = device_get_softc(dev);
	uint8_t cnt[8] = { 0 }, off[8] = { 0 };
	int64_t offset, ticks;
	int error;

	mtx_lock(&sc->mtx);
	error = spmi_readl(sc, PMU_ABBEY_COUNTER, cnt, 6);
	if (error == 0)
		error = spmi_readl(sc, sc->offset_reg, off, 6);
	mtx_unlock(&sc->mtx);
	if (error != 0)
		return (error);

	/* Sign-extend the 48-bit offset and convert 1/32768 s to 1/65536 s. */
	offset = (int64_t)(le64dec(off) << 16) >> 16;
	ticks = (int64_t)le64dec(cnt) + offset * 2;
	if (ticks < 0)
		return (EINVAL);
	ts->tv_sec = ticks >> PMU_HZ_SHIFT;
	ts->tv_nsec = ((ticks & ((1 << PMU_HZ_SHIFT) - 1)) * 1000000000LL) >>
	    PMU_HZ_SHIFT;
	return (0);
}

static int
apple_pmu_rtc_settime(device_t dev, struct timespec *ts)
{
	return (EOPNOTSUPP);
}

static int
apple_pmu_rtc_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "apple,smc-rtc"))
		return (ENXIO);
	device_set_desc(dev, "Apple PMU RTC (read-only)");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_pmu_rtc_attach(device_t dev)
{
	struct apple_pmu_rtc_softc *sc = device_get_softc(dev);
	phandle_t cell, pmic, spmi;
	pcell_t cellreg[2], pmicreg[2], xref;
	bus_addr_t pa;
	bus_size_t size;

	sc->dev = dev;

	/* /rtc → offset cell → nvmem-layout → PMIC (SPMI slave) → controller */
	if (OF_getencprop(ofw_bus_get_node(dev), "nvmem-cells", &xref,
	    sizeof(xref)) <= 0 || (cell = OF_node_from_xref(xref)) <= 0 ||
	    OF_getencprop(cell, "reg", cellreg, sizeof(cellreg)) !=
	    sizeof(cellreg) || cellreg[1] != 6) {
		device_printf(dev, "no usable rtc_offset cell\n");
		return (ENXIO);
	}
	pmic = OF_parent(OF_parent(cell));
	spmi = OF_parent(pmic);
	if (pmic <= 0 || spmi <= 0 ||
	    OF_getencprop(pmic, "reg", pmicreg, sizeof(pmicreg)) <= 0 ||
	    ofw_reg_to_paddr(spmi, 0, &pa, &size, NULL) != 0) {
		device_printf(dev, "can't find the PMU on SPMI\n");
		return (ENXIO);
	}
	if (!ofw_bus_node_is_compatible(pmic, "apple,abbey-pmic")) {
		device_printf(dev, "unsupported PMU\n");
		return (ENXIO);
	}
	sc->sid = pmicreg[0];
	sc->offset_reg = cellreg[0];
	sc->regs = ofw_bus_node_is_compatible(spmi, "apple,t8142-spmi") ?
	    &spmi_gen4 : &spmi_gen1;
	sc->spmi = (vm_offset_t)pmap_mapdev(pa, size);
	mtx_init(&sc->mtx, "apple_pmu_rtc", NULL, MTX_DEF);

	clock_register_flags(dev, 1000000 / 65536, CLOCKF_GETTIME_NO_ADJ);
	return (0);
}

static device_method_t apple_pmu_rtc_methods[] = {
	DEVMETHOD(device_probe,		apple_pmu_rtc_probe),
	DEVMETHOD(device_attach,	apple_pmu_rtc_attach),
	DEVMETHOD(clock_gettime,	apple_pmu_rtc_gettime),
	DEVMETHOD(clock_settime,	apple_pmu_rtc_settime),
	DEVMETHOD_END
};

static driver_t apple_pmu_rtc_driver = {
	"apple_rtc",
	apple_pmu_rtc_methods,
	sizeof(struct apple_pmu_rtc_softc),
};

DRIVER_MODULE(apple_pmu_rtc, simplebus, apple_pmu_rtc_driver, NULL, NULL);
DRIVER_MODULE(apple_pmu_rtc, ofwbus, apple_pmu_rtc_driver, NULL, NULL);
DRIVER_MODULE(apple_pmu_rtc, apple_smc, apple_pmu_rtc_driver, NULL, NULL);
