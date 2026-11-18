/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple ANS2 NVMe front-end: boots the ANS coprocessor over its ASC mailbox
 * (RTKit), lets it DMA its log buffers through the SART, then hands the
 * controller to nvme(4) with QUIRK_APPLE_ANS (linear SQ + NVMMU).
 *
 * The mailbox and SART are separate DT nodes ("mboxes", "apple,sart"); they
 * have no other users, so they are mapped directly here. RTKit is polled:
 * at boot until the IOP reports power ON, then from a callout that acks
 * syslog and ioreport traffic.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/hash.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <dev/nvme/nvme_private.h>

/* ASC wrapper (DT reg "ans") */
#define	ASC_CPU_CONTROL			0x44
#define	ASC_CPU_CONTROL_RUN		0x10

/* ASC mailbox */
#define	MBOX_A2I_CONTROL		0x110
#define	MBOX_I2A_CONTROL		0x114
#define	MBOX_CONTROL_FULL		(1u << 16)
#define	MBOX_CONTROL_EMPTY		(1u << 17)
#define	MBOX_A2I_SEND0			0x800
#define	MBOX_A2I_SEND1			0x808
#define	MBOX_I2A_RECV0			0x830
#define	MBOX_I2A_RECV1			0x838

/* SART v3 */
#define	SART_MAX_ENTRIES		16
#define	SART3_CONFIG(i)			(0x00 + 4 * (i))
#define	SART3_PADDR(i)			(0x40 + 4 * (i))
#define	SART3_SIZE(i)			(0x80 + 4 * (i))
#define	SART3_FLAGS_ALLOW		0xff

/* NVMe vendor registers */
#define	ANS_BOOT_STATUS			0x1300
#define	ANS_BOOT_STATUS_OK		0xde71ce55

/* RTKit */
#define	EP_MGMT				0
#define	EP_CRASHLOG			1
#define	EP_SYSLOG			2
#define	EP_DEBUG			3
#define	EP_IOREPORT			4
#define	EP_OSLOG			8

#define	MSG_TYPE(m)			(((m) >> 52) & 0xff)
#define	MSG_TYPE_SET(t)			((uint64_t)(t) << 52)

#define	MGMT_HELLO			1
#define	MGMT_HELLO_ACK			2
#define	MGMT_START_EP			5
#define	MGMT_IOP_PWR			6
#define	MGMT_IOP_PWR_ACK		7
#define	MGMT_EPMAP			8
#define	MGMT_AP_PWR			0xb
#define	MGMT_EPMAP_DONE			(1ull << 51)
#define	MGMT_EPMAP_MORE			1ull

#define	PWR_ON				0x20
#define	PWR_INIT			0x220

#define	MSG_BUFFER_REQUEST		1
#define	BUFREQ_SIZE(m)			(((m) >> 44) & 0xff)	/* 4K pages */
#define	BUFREQ_ADDR(m)			((m) & ((1ull << 42) - 1))
#define	MSG_SYSLOG_LOG			5
#define	MSG_SYSLOG_INIT			8

#define	RTKIT_MIN_VERSION		11
#define	RTKIT_MAX_VERSION		12

struct ans_buffer {
	void		*va;
	vm_paddr_t	pa;
	size_t		size;
};

struct ans_syslog_entry {
	uint32_t	hdr;
	uint32_t	unk;
	char		context[24];
	char		msg[];
};

struct apple_ans_softc {
	struct nvme_controller	ctrlr;		/* must be first: DEVICE2SOFTC */
	device_t		dev;
	struct resource		*asc_res;
	vm_offset_t		mbox;
	vm_size_t		mbox_size;
	vm_offset_t		sart;
	vm_size_t		sart_size;

	struct mtx		mtx;		/* mailbox */
	struct callout		poll;
	uint32_t		iop_power;
	bool			crashed;
	struct ans_buffer	crashlog, syslog, ioreport, oslog;
	uint32_t		syslog_count, syslog_size;
	uint32_t		syslog_last, syslog_repeats;

	bus_dma_tag_t		tcb_tag;
	bus_dmamap_t		tcb_map;
	void			*tcb_va;
};

static struct ofw_compat_data compat_data[] = {
	{ "apple,t8142-nvme-ans2",	1 },
	{ "apple,t8140-nvme-ans2",	1 },
	{ NULL,				0 }
};

static int ans_verbose = 1;
SYSCTL_DECL(_hw_nvme);
SYSCTL_INT(_hw_nvme, OID_AUTO, apple_ans_verbose, CTLFLAG_RWTUN, &ans_verbose,
    0, "Print Apple ANS firmware syslog");

#define	MBOX_RD4(sc, o)		(*(volatile uint32_t *)((sc)->mbox + (o)))
#define	MBOX_RD8(sc, o)		(*(volatile uint64_t *)((sc)->mbox + (o)))
#define	MBOX_WR8(sc, o, v)	(*(volatile uint64_t *)((sc)->mbox + (o)) = (v))
#define	SART_RD4(sc, o)		(*(volatile uint32_t *)((sc)->sart + (o)))
#define	SART_WR4(sc, o, v)	(*(volatile uint32_t *)((sc)->sart + (o)) = (v))

static int
ans_map_phandle(device_t dev, const char *prop, vm_offset_t *va,
    vm_size_t *size)
{
	phandle_t xref;
	bus_addr_t pa;
	bus_size_t sz;

	if (OF_getencprop(ofw_bus_get_node(dev), prop, &xref,
	    sizeof(xref)) <= 0) {
		device_printf(dev, "no %s property\n", prop);
		return (ENXIO);
	}
	if (ofw_reg_to_paddr(OF_node_from_xref(xref), 0, &pa, &sz, NULL) != 0) {
		device_printf(dev, "can't get %s registers\n", prop);
		return (ENXIO);
	}
	*va = (vm_offset_t)pmap_mapdev(pa, sz);
	*size = sz;
	return (0);
}

static int
ans_send(struct apple_ans_softc *sc, uint32_t ep, uint64_t msg)
{
	int i;

	for (i = 0; MBOX_RD4(sc, MBOX_A2I_CONTROL) & MBOX_CONTROL_FULL; i++) {
		if (i == 200000) {
			device_printf(sc->dev, "mailbox full\n");
			return (ETIMEDOUT);
		}
		DELAY(1);
	}
	wmb();
	MBOX_WR8(sc, MBOX_A2I_SEND0, msg);
	MBOX_WR8(sc, MBOX_A2I_SEND1, ep);
	return (0);
}

static bool
ans_recv(struct apple_ans_softc *sc, uint32_t *ep, uint64_t *msg)
{
	if (MBOX_RD4(sc, MBOX_I2A_CONTROL) & MBOX_CONTROL_EMPTY)
		return (false);
	*msg = MBOX_RD8(sc, MBOX_I2A_RECV0);
	*ep = (uint32_t)MBOX_RD8(sc, MBOX_I2A_RECV1) & 0xff;
	rmb();
	return (true);
}

static int
ans_recv_timeout(struct apple_ans_softc *sc, uint32_t *ep, uint64_t *msg,
    int usec)
{
	for (; usec > 0; usec -= 10) {
		if (ans_recv(sc, ep, msg))
			return (0);
		DELAY(10);
	}
	return (ETIMEDOUT);
}

static int
ans_sart_allow(struct apple_ans_softc *sc, vm_paddr_t pa, size_t size)
{
	int i;

	for (i = 0; i < SART_MAX_ENTRIES; i++) {
		if (SART_RD4(sc, SART3_CONFIG(i)) != 0)
			continue;
		SART_WR4(sc, SART3_PADDR(i), pa >> 12);
		SART_WR4(sc, SART3_SIZE(i), size >> 12);
		SART_WR4(sc, SART3_CONFIG(i), SART3_FLAGS_ALLOW);
		return (0);
	}
	device_printf(sc->dev, "no free SART entry\n");
	return (ENOSPC);
}

static int
ans_buffer_request(struct apple_ans_softc *sc, uint32_t ep, uint64_t msg,
    struct ans_buffer *buf)
{
	size_t npages = BUFREQ_SIZE(msg);
	uint64_t addr = BUFREQ_ADDR(msg);
	int error;

	buf->size = npages << 12;
	if (addr != 0) {
		/* The IOP brought its own buffer; SART-filtered, so physical. */
		buf->pa = addr;
		buf->va = pmap_mapdev(addr, buf->size);
		return (0);
	}

	if (buf->va == NULL) {
		buf->size = roundup2(buf->size, 16384);
		buf->va = contigmalloc(buf->size, M_DEVBUF, M_ZERO | M_NOWAIT,
		    0, ~(vm_paddr_t)0, 16384, 0);
		if (buf->va == NULL)
			return (ENOMEM);
		buf->pa = pmap_kextract((vm_offset_t)buf->va);
		error = ans_sart_allow(sc, buf->pa, buf->size);
		if (error != 0)
			return (error);
	}
	if (bootverbose)
		device_printf(sc->dev, "ep %u buffer %#zx at %#jx\n", ep,
		    buf->size, (uintmax_t)buf->pa);

	return (ans_send(sc, ep, MSG_TYPE_SET(MSG_BUFFER_REQUEST) |
	    (uint64_t)npages << 44 | buf->pa));
}

static void
ans_syslog(struct apple_ans_softc *sc, uint64_t msg)
{
	struct ans_syslog_entry *e;
	size_t idx = msg & 0xff, len, stride;
	uint32_t hash;

	stride = sc->syslog_size + sizeof(*e);
	if (!ans_verbose || sc->syslog.va == NULL ||
	    (idx + 1) * stride > sc->syslog.size)
		return;
	e = (struct ans_syslog_entry *)((char *)sc->syslog.va + idx * stride);
	len = strnlen(e->msg, sc->syslog_size);
	if (len > 0 && e->msg[len - 1] == '\n')
		len--;

	/* The firmware repeats some lines many times; print a line once. */
	hash = murmur3_32_hash(e->msg, len, 0);
	if (hash == sc->syslog_last) {
		sc->syslog_repeats++;
		return;
	}
	if (sc->syslog_repeats > 0)
		device_printf(sc->dev, "fw: (last line repeated %u times)\n",
		    sc->syslog_repeats);
	sc->syslog_last = hash;
	sc->syslog_repeats = 0;
	device_printf(sc->dev, "fw: [%.24s] %.*s\n", e->context, (int)len,
	    e->msg);
}

static void
ans_crashed(struct apple_ans_softc *sc)
{
	sc->crashed = true;
	device_printf(sc->dev, "ANS firmware crashed\n");
}

/* Handle one system-endpoint message. */
static int
ans_handle(struct apple_ans_softc *sc, uint32_t ep, uint64_t msg)
{
	uint32_t type = MSG_TYPE(msg);

	switch (ep) {
	case EP_MGMT:
		if (type == MGMT_IOP_PWR_ACK)
			sc->iop_power = msg & 0xffff;
		else if (type != MGMT_AP_PWR)
			device_printf(sc->dev, "mgmt message %#jx\n",
			    (uintmax_t)msg);
		return (0);
	case EP_CRASHLOG:
		if (type != MSG_BUFFER_REQUEST)
			break;
		if (sc->crashlog.va != NULL) {
			ans_crashed(sc);
			return (EIO);
		}
		return (ans_buffer_request(sc, ep, msg, &sc->crashlog));
	case EP_SYSLOG:
		if (type == MSG_BUFFER_REQUEST)
			return (ans_buffer_request(sc, ep, msg, &sc->syslog));
		if (type == MSG_SYSLOG_INIT) {
			sc->syslog_count = msg & 0xffff;
			sc->syslog_size = (msg >> 24) & 0xffff;
			return (0);
		}
		if (type == MSG_SYSLOG_LOG) {
			ans_syslog(sc, msg);
			return (ans_send(sc, ep, msg));
		}
		break;
	case EP_IOREPORT:
		if (type == MSG_BUFFER_REQUEST)
			return (ans_buffer_request(sc, ep, msg, &sc->ioreport));
		if (type == 0x8 || type == 0xc)		/* must be echoed */
			return (ans_send(sc, ep, msg));
		break;
	case EP_OSLOG:
		if (type == MSG_BUFFER_REQUEST)
			return (ans_buffer_request(sc, ep, msg, &sc->oslog));
		break;
	}
	device_printf(sc->dev, "unhandled message ep %u: %#jx\n", ep,
	    (uintmax_t)msg);
	return (0);
}

static void
ans_poll(void *arg)
{
	struct apple_ans_softc *sc = arg;
	uint64_t msg;
	uint32_t ep;

	mtx_assert(&sc->mtx, MA_OWNED);
	while (!sc->crashed && ans_recv(sc, &ep, &msg))
		ans_handle(sc, ep, msg);
	if (!sc->crashed)
		callout_schedule(&sc->poll, hz / 20);
}

static int
ans_rtkit_boot(struct apple_ans_softc *sc)
{
	uint64_t msg, eps = 0;
	uint32_t ep, ver, base;
	int error, i;

	/* Start the IOP if a previous stage stopped it, then wake it. */
	if ((bus_read_4(sc->asc_res, ASC_CPU_CONTROL) &
	    ASC_CPU_CONTROL_RUN) == 0) {
		device_printf(sc->dev, "starting stopped ANS\n");
		bus_write_4(sc->asc_res, ASC_CPU_CONTROL, ASC_CPU_CONTROL_RUN);
	}
	error = ans_send(sc, EP_MGMT, MSG_TYPE_SET(MGMT_IOP_PWR) | PWR_INIT);
	if (error != 0)
		return (error);

	if (ans_recv_timeout(sc, &ep, &msg, 1000000) != 0 || ep != EP_MGMT ||
	    MSG_TYPE(msg) != MGMT_HELLO) {
		device_printf(sc->dev, "no RTKit HELLO\n");
		return (ENXIO);
	}
	ver = MIN(RTKIT_MAX_VERSION, (msg >> 16) & 0xffff);
	if ((msg & 0xffff) > RTKIT_MAX_VERSION || ver < RTKIT_MIN_VERSION) {
		device_printf(sc->dev, "unsupported RTKit versions %ju-%ju\n",
		    (uintmax_t)(msg & 0xffff), (uintmax_t)((msg >> 16) & 0xffff));
		return (ENXIO);
	}
	error = ans_send(sc, EP_MGMT, MSG_TYPE_SET(MGMT_HELLO_ACK) |
	    ver << 16 | ver);
	if (error != 0)
		return (error);

	do {
		if (ans_recv_timeout(sc, &ep, &msg, 1000000) != 0 ||
		    ep != EP_MGMT || MSG_TYPE(msg) != MGMT_EPMAP) {
			device_printf(sc->dev, "no RTKit endpoint map\n");
			return (ENXIO);
		}
		base = (msg >> 32) & 0x7;
		if (base == 0)
			eps = msg & 0xffffffff;	/* system endpoints */
		error = ans_send(sc, EP_MGMT, MSG_TYPE_SET(MGMT_EPMAP) |
		    (uint64_t)base << 32 | ((msg & MGMT_EPMAP_DONE) ?
		    MGMT_EPMAP_DONE : MGMT_EPMAP_MORE));
		if (error != 0)
			return (error);
	} while ((msg & MGMT_EPMAP_DONE) == 0);

	const int sys_eps[] = { EP_DEBUG, EP_CRASHLOG, EP_SYSLOG, EP_IOREPORT,
	    EP_OSLOG };
	for (i = 0; i < nitems(sys_eps); i++) {
		if ((eps & (1u << sys_eps[i])) == 0)
			continue;
		error = ans_send(sc, EP_MGMT, MSG_TYPE_SET(MGMT_START_EP) |
		    (uint64_t)sys_eps[i] << 32 | 2);
		if (error != 0)
			return (error);
	}

	for (i = 0; sc->iop_power != PWR_ON; i++) {
		if (sc->crashed)
			return (EIO);
		if (i == 100000) {
			device_printf(sc->dev, "IOP power state %#x, not ON\n",
			    sc->iop_power);
			return (ETIMEDOUT);
		}
		if (ans_recv(sc, &ep, &msg))
			error = ans_handle(sc, ep, msg);
		else
			DELAY(10);
		if (error != 0)
			return (error);
	}
	return (ans_send(sc, EP_MGMT, MSG_TYPE_SET(MGMT_AP_PWR) | PWR_ON));
}

static void
ans_dma_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	if (error == 0)
		*(bus_addr_t *)arg = segs[0].ds_addr;
}

static int
apple_ans_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Apple ANS2 NVMe controller");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_ans_attach(device_t dev)
{
	struct apple_ans_softc *sc = device_get_softc(dev);
	struct nvme_controller *ctrlr = &sc->ctrlr;
	const size_t tcbsz = NVME_APPLE_QUEUE_ENTRIES *
	    sizeof(struct nvme_apple_tcb);
	bus_addr_t tcb_pa;
	int error, i, rid;

	sc->dev = dev;
	mtx_init(&sc->mtx, "apple_ans mbox", NULL, MTX_DEF);
	callout_init_mtx(&sc->poll, &sc->mtx, 0);

	ctrlr->resource_id = 0;
	ctrlr->resource = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &ctrlr->resource_id, RF_ACTIVE);
	rid = 1;
	sc->asc_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	rid = 2;
	ctrlr->apple_nvmmu = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (ctrlr->resource == NULL || sc->asc_res == NULL ||
	    ctrlr->apple_nvmmu == NULL) {
		device_printf(dev, "can't map registers\n");
		return (ENXIO);
	}
	if (ans_map_phandle(dev, "mboxes", &sc->mbox, &sc->mbox_size) != 0 ||
	    ans_map_phandle(dev, "apple,sart", &sc->sart, &sc->sart_size) != 0)
		return (ENXIO);

	/* Both queues' TCB arrays, programmed before the controller is up. */
	error = bus_dma_tag_create(bus_get_dma_tag(dev), PAGE_SIZE, 0,
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL, NULL, 2 * tcbsz, 1,
	    2 * tcbsz, 0, NULL, NULL, &sc->tcb_tag);
	if (error == 0)
		error = bus_dmamem_alloc(sc->tcb_tag, &sc->tcb_va,
		    BUS_DMA_COHERENT | BUS_DMA_ZERO | BUS_DMA_WAITOK,
		    &sc->tcb_map);
	if (error == 0)
		error = bus_dmamap_load(sc->tcb_tag, sc->tcb_map, sc->tcb_va,
		    2 * tcbsz, ans_dma_cb, &tcb_pa, BUS_DMA_NOWAIT);
	if (error != 0) {
		device_printf(dev, "can't allocate NVMMU TCBs: %d\n", error);
		return (error);
	}
	for (i = 0; i < 2; i++) {
		ctrlr->apple_tcb[i] = (struct nvme_apple_tcb *)
		    ((char *)sc->tcb_va + i * tcbsz);
		ctrlr->apple_tcb_bus[i] = tcb_pa + i * tcbsz;
	}

	if (bootverbose)
		device_printf(dev, "CPU_CONTROL %#x BOOT_STATUS %#x CC %#x "
		    "CSTS %#x\n", bus_read_4(sc->asc_res, ASC_CPU_CONTROL),
		    bus_read_4(ctrlr->resource, ANS_BOOT_STATUS),
		    nvme_mmio_read_4(ctrlr, cc), nvme_mmio_read_4(ctrlr, csts));

	mtx_lock(&sc->mtx);
	error = ans_rtkit_boot(sc);
	if (error == 0)
		callout_reset(&sc->poll, hz / 20, ans_poll, sc);
	mtx_unlock(&sc->mtx);
	if (error != 0) {
		device_printf(dev, "RTKit boot failed: %d\n", error);
		return (error);
	}

	for (i = 0; bus_read_4(ctrlr->resource, ANS_BOOT_STATUS) !=
	    ANS_BOOT_STATUS_OK; i++) {
		if (i == 1000) {
			device_printf(dev, "ANS did not boot: status %#x\n",
			    bus_read_4(ctrlr->resource, ANS_BOOT_STATUS));
			return (ENXIO);
		}
		DELAY(1000);
	}
	device_printf(dev, "RTKit up\n");

	ctrlr->rid = 0;
	ctrlr->res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &ctrlr->rid,
	    RF_SHAREABLE | RF_ACTIVE);
	if (ctrlr->res == NULL) {
		device_printf(dev, "can't allocate interrupt\n");
		return (ENXIO);
	}
	ctrlr->msi_count = 0;
	ctrlr->num_io_queues = 1;
	if (bus_setup_intr(dev, ctrlr->res, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, nvme_ctrlr_shared_handler, ctrlr, &ctrlr->tag) != 0) {
		device_printf(dev, "can't set up interrupt\n");
		return (ENXIO);
	}
	ctrlr->tag = (void *)0x1;
	/* Not on PCI: QUIRK_AHCI keeps nvme(4) away from config space. */
	ctrlr->quirks |= QUIRK_APPLE_ANS | QUIRK_AHCI;

	return (nvme_attach(dev));
}

static int
apple_ans_detach(device_t dev)
{
	return (EBUSY);
}

static device_method_t apple_ans_methods[] = {
	DEVMETHOD(device_probe,		apple_ans_probe),
	DEVMETHOD(device_attach,	apple_ans_attach),
	DEVMETHOD(device_detach,	apple_ans_detach),
	DEVMETHOD(device_shutdown,	nvme_shutdown),
	DEVMETHOD_END
};

static driver_t apple_ans_driver = {
	"nvme",
	apple_ans_methods,
	sizeof(struct apple_ans_softc),
};

DRIVER_MODULE(apple_ans, simplebus, apple_ans_driver, NULL, NULL);
MODULE_DEPEND(apple_ans, nvme, 1, 1, 1);
