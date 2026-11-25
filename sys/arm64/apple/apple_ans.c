/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple ANS2 NVMe front-end: boots the ANS coprocessor over its ASC mailbox
 * (RTKit), lets it DMA its log buffers through the SART, then hands the
 * controller to nvme(4) with QUIRK_APPLE_ANS (linear SQ + NVMMU).
 *
 * The mailbox and SART are separate DT nodes ("mboxes", "apple,sart"); they
 * have no other users, so they are mapped directly here. RTKit (apple_rtkit)
 * is polled: at boot until the IOP reports power ON, then from a callout that
 * acks syslog and ioreport traffic.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/sched.h>
#include <sys/smp.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <dev/nvme/nvme_private.h>

#include <arm64/apple/apple_rtkit.h>

/* ASC wrapper (DT reg "ans") */
#define	ASC_CPU_CONTROL			0x44
#define	ASC_CPU_CONTROL_RUN		0x10

/* SART v3 */
#define	SART_MAX_ENTRIES		16
#define	SART3_CONFIG(i)			(0x00 + 4 * (i))
#define	SART3_PADDR(i)			(0x40 + 4 * (i))
#define	SART3_SIZE(i)			(0x80 + 4 * (i))
#define	SART3_FLAGS_ALLOW		0xff

/* NVMe vendor registers */
#define	ANS_BOOT_STATUS			0x1300
#define	ANS_BOOT_STATUS_OK		0xde71ce55
#define	ANS_FLUSH_PAGE_SIZE		16384

/*
 * ANS raises one interrupt per completion, and it can arrive before the entry
 * is visible in the CQ (2-16 us on T6040, per Linux apple.c). No second
 * interrupt follows, so an empty pass would leave the command waiting for the
 * I/O timeout. Give a late entry time to land.
 */
#define	ANS_LATE_CQE_US			100

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
	struct apple_rtkit	*rtk;

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
/* M5 SMP experiment: spin this many seconds before touching ANS at attach. */
static int ans_attach_delay;
SYSCTL_INT(_hw_nvme, OID_AUTO, apple_ans_delay, CTLFLAG_RDTUN,
    &ans_attach_delay, 0, "Seconds to wait at attach before starting ANS");
/* M5 SMP experiment: bind the nvme interrupt thread to this CPU (-1: no). */
static int ans_bind_cpu = -1;
SYSCTL_INT(_hw_nvme, OID_AUTO, apple_ans_bind_cpu, CTLFLAG_RDTUN, &ans_bind_cpu,
    0, "CPU the ANS interrupt thread is bound to (-1: unbound)");
/*
 * M5 SMP experiment: a thread bound to CPU 1 that sleeps (WFI) and touches
 * the controller on each wake. 1: read CSTS and NVMMU TCB_STAT, 2: write
 * INTMS then INTMC, 3: read the ASC CPU_CONTROL register.
 */
static int ans_probe_mode;
SYSCTL_INT(_hw_nvme, OID_AUTO, apple_ans_cpu1_probe, CTLFLAG_RDTUN,
    &ans_probe_mode, 0, "CPU 1 wake-and-touch probe mode (0: off)");

static void
ans_cpu1_probe(void *arg)
{
	struct apple_ans_softc *sc = arg;
	struct nvme_controller *ctrlr = &sc->ctrlr;
	uint32_t v = 0, n = 0;

	while (!smp_started)
		pause("anspw", hz);
	thread_lock(curthread);
	sched_bind(curthread, 1);
	thread_unlock(curthread);
	device_printf(sc->dev, "cpu1 probe mode %d: running on CPU %d\n",
	    ans_probe_mode, PCPU_GET(cpuid));
	for (;;) {
		pause("ansp1", hz / 50);
		switch (ans_probe_mode) {
		case 1:
			v = nvme_mmio_read_4(ctrlr, csts);
			v += bus_read_4(ctrlr->apple_nvmmu,
			    NVME_APPLE_NVMMU_TCB_STAT) << 16;
			break;
		case 2:
			nvme_mmio_write_4(ctrlr, intms, 1);
			nvme_mmio_write_4(ctrlr, intmc, 1);
			break;
		case 3:
			v = bus_read_4(sc->asc_res, ASC_CPU_CONTROL);
			break;
		}
		if (++n % 250 == 0)
			device_printf(sc->dev, "cpu1 probe %u: cpu %d v %#x\n",
			    n, PCPU_GET(cpuid), v);
	}
}

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
ans_sart_allow(void *arg, vm_paddr_t pa, size_t size, uint64_t *iova)
{
	struct apple_ans_softc *sc = arg;
	int i;

	for (i = 0; i < SART_MAX_ENTRIES; i++) {
		if (SART_RD4(sc, SART3_CONFIG(i)) != 0)
			continue;
		SART_WR4(sc, SART3_PADDR(i), pa >> 12);
		SART_WR4(sc, SART3_SIZE(i), size >> 12);
		SART_WR4(sc, SART3_CONFIG(i), SART3_FLAGS_ALLOW);
		*iova = pa;		/* the SART filters, it doesn't translate */
		return (0);
	}
	device_printf(sc->dev, "no free SART entry\n");
	return (ENOSPC);
}

static const struct apple_rtkit_ops ans_rtkit_ops = {
	.buf_map = ans_sart_allow,
};

static void
ans_poll(void *arg)
{
	struct apple_ans_softc *sc = arg;

	mtx_assert(&sc->mtx, MA_OWNED);
	apple_rtkit_poll(sc->rtk);
	if (!apple_rtkit_crashed(sc->rtk))
		callout_schedule(&sc->poll, hz / 20);
}

static void
ans_dma_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	if (error == 0 && nseg == 1)
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

static bool
apple_ans_process(struct nvme_controller *ctrlr)
{
	bool found;

	found = nvme_qpair_process_completions(&ctrlr->adminq);
	if (ctrlr->ioq != NULL && ctrlr->ioq[0].cpl != NULL)
		found |= nvme_qpair_process_completions(&ctrlr->ioq[0]);
	return (found);
}

/* nvme_ctrlr_shared_handler, plus the late-entry wait. */
static void
apple_ans_intr(void *arg)
{
	struct nvme_controller *ctrlr = arg;
	int us;

	nvme_mmio_write_4(ctrlr, intms, 1);
	if (!apple_ans_process(ctrlr)) {
		for (us = 0; us < ANS_LATE_CQE_US; us++) {
			DELAY(1);
			if (apple_ans_process(ctrlr))
				break;
		}
	}
	nvme_mmio_write_4(ctrlr, intmc, 1);
}

static int
apple_ans_attach(device_t dev)
{
	struct apple_ans_softc *sc = device_get_softc(dev);
	struct nvme_controller *ctrlr = &sc->ctrlr;
	const size_t tcbsz = NVME_APPLE_QUEUE_ENTRIES *
	    sizeof(struct nvme_apple_tcb);
	bus_addr_t tcb_pa = 0;
	size_t dma_align, dma_size;
	bool flush_prps;
	int error, i, rid;

	sc->dev = dev;
	if (ans_attach_delay > 0) {
		device_printf(dev, "waiting %d s before starting ANS\n",
		    ans_attach_delay);
		DELAY(ans_attach_delay * 1000000);
		device_printf(dev, "wait done\n");
	}
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

	/* Both TCB arrays, followed by an M5 Flush scratch page. */
	flush_prps = ofw_bus_is_compatible(dev, "apple,t8142-nvme-ans2");
	dma_align = PAGE_SIZE;
	dma_size = 2 * tcbsz;
	if (flush_prps) {
		dma_align = MAX(dma_align, ANS_FLUSH_PAGE_SIZE);
		dma_size = roundup2(dma_size, dma_align) + dma_align;
	}
	error = bus_dma_tag_create(bus_get_dma_tag(dev), dma_align, 0,
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXADDR, NULL, NULL, dma_size, 1,
	    dma_size, 0, NULL, NULL, &sc->tcb_tag);
	if (error == 0)
		error = bus_dmamem_alloc(sc->tcb_tag, &sc->tcb_va,
		    BUS_DMA_COHERENT | BUS_DMA_ZERO | BUS_DMA_WAITOK,
		    &sc->tcb_map);
	if (error == 0)
		error = bus_dmamap_load(sc->tcb_tag, sc->tcb_map, sc->tcb_va,
		    dma_size, ans_dma_cb, &tcb_pa, BUS_DMA_NOWAIT);
	if (error == 0 && tcb_pa == 0)
		error = EFAULT;
	if (error != 0) {
		device_printf(dev, "can't allocate NVMMU DMA memory: %d\n",
		    error);
		if (sc->tcb_va != NULL)
			bus_dmamem_free(sc->tcb_tag, sc->tcb_va, sc->tcb_map);
		if (sc->tcb_tag != NULL)
			bus_dma_tag_destroy(sc->tcb_tag);
		return (error);
	}
	for (i = 0; i < 2; i++) {
		ctrlr->apple_tcb[i] = (struct nvme_apple_tcb *)
		    ((char *)sc->tcb_va + i * tcbsz);
		ctrlr->apple_tcb_bus[i] = tcb_pa + i * tcbsz;
	}

	if (flush_prps)
		ctrlr->apple_flush_bus = tcb_pa + dma_size - dma_align;

	if (bootverbose)
		device_printf(dev, "CPU_CONTROL %#x BOOT_STATUS %#x CC %#x "
		    "CSTS %#x\n", bus_read_4(sc->asc_res, ASC_CPU_CONTROL),
		    bus_read_4(ctrlr->resource, ANS_BOOT_STATUS),
		    nvme_mmio_read_4(ctrlr, cc), nvme_mmio_read_4(ctrlr, csts));

	sc->rtk = apple_rtkit_create(dev,
	    (vm_offset_t)rman_get_virtual(sc->asc_res), sc->mbox,
	    &ans_rtkit_ops, sc, 0);
	apple_rtkit_set_syslog(sc->rtk, ans_verbose != 0);
	mtx_lock(&sc->mtx);
	error = apple_rtkit_boot(sc->rtk);
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

	mtx_init(&ctrlr->apple_lock, "nvme apple", NULL, MTX_SPIN);
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
	    NULL, apple_ans_intr, ctrlr, &ctrlr->tag) != 0) {
		device_printf(dev, "can't set up interrupt\n");
		return (ENXIO);
	}
	/*
	 * M5 SMP experiment: hw.nvme.apple_ans_bind_cpu=N binds the interrupt
	 * thread to CPU N (the AIC v3 has no per-IRQ target, so the IRQ itself
	 * keeps landing on the boot CPU).
	 */
	if (ans_bind_cpu >= 0) {
		error = bus_bind_intr(dev, ctrlr->res, ans_bind_cpu);
		device_printf(dev, "interrupt thread bound to CPU %d: %d\n",
		    ans_bind_cpu, error);
	}
	ctrlr->tag = (void *)0x1;
	/* Not on PCI: QUIRK_AHCI keeps nvme(4) away from config space. */
	ctrlr->quirks |= QUIRK_APPLE_ANS | QUIRK_AHCI;

	if (ans_probe_mode > 0)
		kproc_create(ans_cpu1_probe, sc, NULL, 0, 0, "ans_cpu1_probe");

	return (nvme_attach(dev));
}

static int
apple_ans_detach(device_t dev)
{
	return (EBUSY);
}

/*
 * After the NVMe shutdown (CC.SHN), quiesce the IOP and put it to sleep, then
 * stop its CPU, as Linux's apple_nvme_shutdown: the SSD keeps acknowledged
 * writes in a cache that only this writes back. Without it a reset after
 * shutdown lost the pool's last ~37 s.
 */
static int
apple_ans_shutdown(device_t dev)
{
	struct apple_ans_softc *sc = device_get_softc(dev);
	int error;

	nvme_shutdown(dev);
	if (sc->rtk == NULL || apple_rtkit_crashed(sc->rtk))
		return (0);
	mtx_lock(&sc->mtx);
	callout_stop(&sc->poll);
	error = apple_rtkit_shutdown(sc->rtk);
	mtx_unlock(&sc->mtx);
	if (error != 0) {
		device_printf(dev, "IOP shutdown failed: %d\n", error);
		return (0);
	}
	bus_write_4(sc->asc_res, ASC_CPU_CONTROL, 0);
	if (bootverbose)
		device_printf(dev, "IOP asleep, CPU stopped\n");
	return (0);
}

static device_method_t apple_ans_methods[] = {
	DEVMETHOD(device_probe,		apple_ans_probe),
	DEVMETHOD(device_attach,	apple_ans_attach),
	DEVMETHOD(device_detach,	apple_ans_detach),
	DEVMETHOD(device_shutdown,	apple_ans_shutdown),
	DEVMETHOD_END
};

static driver_t apple_ans_driver = {
	"nvme",
	apple_ans_methods,
	sizeof(struct apple_ans_softc),
};

DRIVER_MODULE(apple_ans, simplebus, apple_ans_driver, NULL, NULL);
MODULE_DEPEND(apple_ans, nvme, 1, 1, 1);
