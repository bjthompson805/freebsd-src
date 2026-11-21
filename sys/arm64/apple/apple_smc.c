/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple SMC: enumerates its DT children (rtc, gpio, reboot, ...) like a
 * simple-bus, and offers key writes (apple_smc_write_u32) over a polled
 * RTKit session started on first use.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/sx.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_subr.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/fdt/simplebus.h>

#include <arm64/apple/apple_rtkit.h>
#include <arm64/apple/apple_smc.h>

#define	SMC_ENDPOINT		0x20
#define	SMC_READ_KEY		0x10
#define	SMC_WRITE_KEY		0x11
#define	SMC_INITIALIZE		0x17
#define	SMC_NOTIFICATION	0x18
#define	SMC_MSG_ID(id)		((uint64_t)((id) & 0xf) << 12)
#define	SMC_RESULT(m)		((m) & 0xff)
#define	SMC_RESULT_ID(m)	(((m) >> 12) & 0xf)

struct apple_smc_softc {
	struct simplebus_softc	sb;		/* must be first */
	device_t		dev;
	struct sx		lock;
	struct apple_rtkit	*rtk;
	uint64_t		shmem_pa;
	void			*shmem;
	u_int			msgid;
	u_int			wait_id;	/* ID of the pending command */
	bool			pending;
	uint64_t		reply;
};

static struct apple_smc_softc *apple_smc_sc;

static void
apple_smc_recv(void *arg, uint32_t ep, uint64_t msg)
{
	struct apple_smc_softc *sc = arg;

	if (ep != SMC_ENDPOINT)
		return;
	if (sc->shmem_pa == 0) {
		sc->shmem_pa = msg;
		return;
	}
	if (SMC_RESULT(msg) == SMC_NOTIFICATION)
		return;
	/* A late reply to an earlier command is not this one's. */
	if (!sc->pending || SMC_RESULT_ID(msg) != sc->wait_id) {
		device_printf(sc->dev, "stray reply %#jx\n", (uintmax_t)msg);
		return;
	}
	sc->reply = msg;
	sc->pending = false;
}

static const struct apple_rtkit_ops apple_smc_rtkit_ops = {
	.recv = apple_smc_recv,
};

static int
apple_smc_wait(struct apple_smc_softc *sc, bool *done_when_false, uint64_t *pa)
{
	int i;

	for (i = 0; i < 1000000; i++) {
		apple_rtkit_poll(sc->rtk);
		if (apple_rtkit_crashed(sc->rtk))
			return (EIO);
		if (done_when_false != NULL ? !*done_when_false : *pa != 0)
			return (0);
		DELAY(1);
	}
	return (ETIMEDOUT);
}

static int
apple_smc_start(struct apple_smc_softc *sc)
{
	phandle_t node = ofw_bus_get_node(sc->dev);
	vm_offset_t asc, mbox;
	bus_addr_t pa;
	bus_size_t size;
	pcell_t xref;
	int error;

	if (ofw_reg_to_paddr(node, 0, &pa, &size, NULL) != 0)
		return (ENXIO);
	asc = (vm_offset_t)pmap_mapdev(pa, size);
	if (OF_getencprop(node, "mboxes", &xref, sizeof(xref)) <= 0 ||
	    ofw_reg_to_paddr(OF_node_from_xref(xref), 0, &pa, &size,
	    NULL) != 0)
		return (ENXIO);
	mbox = (vm_offset_t)pmap_mapdev(pa, size);

	sc->rtk = apple_rtkit_create(sc->dev, asc, mbox, &apple_smc_rtkit_ops,
	    sc, 0);
	error = apple_rtkit_boot(sc->rtk);
	if (error == 0)
		error = apple_rtkit_start_ep(sc->rtk, SMC_ENDPOINT);
	if (error == 0)
		error = apple_rtkit_send(sc->rtk, SMC_ENDPOINT,
		    SMC_INITIALIZE | SMC_MSG_ID(sc->msgid++));
	if (error == 0)
		error = apple_smc_wait(sc, NULL, &sc->shmem_pa);
	if (error != 0) {
		device_printf(sc->dev, "RTKit start failed: %d\n", error);
		return (error);
	}
	sc->shmem = pmap_mapdev(sc->shmem_pa, PAGE_SIZE);
	device_printf(sc->dev, "shared buffer at %#jx\n",
	    (uintmax_t)sc->shmem_pa);
	return (0);
}

int
apple_smc_write_u32(uint32_t key, uint32_t value)
{
	struct apple_smc_softc *sc = apple_smc_sc;
	int error = 0;

	if (sc == NULL)
		return (ENXIO);
	sx_xlock(&sc->lock);
	if (sc->shmem == NULL)
		error = apple_smc_start(sc);
	if (error == 0) {
		memcpy(sc->shmem, &value, sizeof(value));
		sc->wait_id = sc->msgid++ & 0xf;
		sc->pending = true;
		error = apple_rtkit_send(sc->rtk, SMC_ENDPOINT, SMC_WRITE_KEY |
		    SMC_MSG_ID(sc->wait_id) | (uint64_t)sizeof(value) << 16 |
		    (uint64_t)key << 32);
	}
	if (error == 0)
		error = apple_smc_wait(sc, &sc->pending, NULL);
	if (error == 0 && SMC_RESULT(sc->reply) != 0) {
		device_printf(sc->dev, "write %#x failed: %ju\n", key,
		    (uintmax_t)SMC_RESULT(sc->reply));
		error = EIO;
	}
	sx_xunlock(&sc->lock);
	return (error);
}

static int
apple_smc_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "apple,t8142-smc"))
		return (ENXIO);
	device_set_desc(dev, "Apple SMC");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_smc_attach(device_t dev)
{
	struct apple_smc_softc *sc = device_get_softc(dev);
	phandle_t node, child;

	sc->dev = dev;
	sx_init(&sc->lock, "apple_smc");
	apple_smc_sc = sc;
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
    sizeof(struct apple_smc_softc), simplebus_driver);
EARLY_DRIVER_MODULE(apple_smc, simplebus, apple_smc_driver, NULL, NULL,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE);
