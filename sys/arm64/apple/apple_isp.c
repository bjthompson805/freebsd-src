// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple Image Signal Processor driver: attach, memory and the address filter.
 * Ported from aurora-silicon's Linux isp-drv.c and isp-iommu.c (T8142 only).
 *
 * Copyright (C) 2023 The Asahi Linux Contributors
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <arm64/apple/apple_dart.h>
#include <arm64/apple/apple_pmgr.h>
#include <arm64/apple/apple_isp.h>

MALLOC_DEFINE(M_APPLEISP, "apple_isp", "Apple ISP");

int apple_isp_debug;
SYSCTL_NODE(_hw, OID_AUTO, apple_isp, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Apple ISP");
SYSCTL_INT(_hw_apple_isp, OID_AUTO, debug, CTLFLAG_RWTUN, &apple_isp_debug,
    0, "Log the firmware's commands and its console");

/* The DART page size; the ISP DARTs are T8110s. */
#define	ISP_PAGE_SIZE	16384

/*
 * T8142 (M5, J813): the Neo's ISP17a. The J813 ADT's dart-isp
 * dapf-instance-0 ranges, rounded out to 16 KiB pages; mostly the T8140
 * windows at +0x80000000, with one DSID window and two new ranges.
 */
static const struct isp_mmio_window apple_isp_fw_mmio_t8142[] = {
	{ 0x220104000, 0x14000 },	/* DSID broadcast-clear window */
	{ 0x3803c0000, 0x28000 },	/* PMGR, with the PMP clock scratch */
	{ 0x380704000, 0x4000 },	/* PMGR, ISP power states */
	{ 0x380730000, 0x4000 },
	{ 0x380e3c000, 0x4000 },
	{ 0x382724000, 0x4000 },	/* PMP bandwidth scratch */
	{ 0x39062c000, 0x30000 },
	{ 0x3a1340000, 0x4000 },
	{ 0x3a1358000, 0xc000 },
	{ 0x421660000, 0x4000 },
};

static const struct apple_isp_hw apple_isp_hw_t8142 = {
	.gen = ISP_GEN_T8140,
	.fw_abi = ISP_FW_ABI_H17,

	.dsid_count = 1,
	.dsid_clr_base0 = 0x220116000,
	.dsid_clr_range0 = 0x2fc,

	.clock_scratch = 0x3803d0ca0,
	.clock_base = 0x0,
	.clock_bit = 0x0,
	.clock_size = 0x8,
	.bandwidth_scratch = 0x382724000,
	.bandwidth_base = 0x0,
	.bandwidth_bit = 0x0,
	.bandwidth_size = 0x8,
	.mbox_irq_enable = ISP_MBOX_IRQ_ENABLE_T6031,

	/*
	 * No BES scaler: the firmware asserts on CH_CROP_SET ("BesProc is
	 * removed/not present. Must config AuxSclPr") and takes the SCL1 set.
	 */
	.scl1 = true,
	.lpdp = true,
	.meta_size = ISP_META_SIZE_T8140,

	.fw_iova_mask = GENMASK_ULL(35, 0),
	.mbox2_offset = 0x410,
	.coproc_control = ISP_COPROC_CONTROL_T8140,
	.coproc_status = ISP_COPROC_STATUS_T8140,
	.coproc_quiesce = ISP_COPROC_QUIESCE_T8140,
	.fw_mmio = apple_isp_fw_mmio_t8142,
	.num_fw_mmio = nitems(apple_isp_fw_mmio_t8142),
	.boot_mode = 1,
	.capture_meta_size = ISP_CAPTURE_META_SIZE_T8140,
	/*
	 * As on T6040: the firmware starts to detect the camera and stops
	 * again, each stream starts it from its own memory, and it stays up
	 * for 5 s after a stream. The coprocessor is quiesced before its
	 * domains go off.
	 */
	.fw_idle_ms = 5000,
	/* The J813 IMX958 runs the xx24 setfile 2224_01, as T6040 does. */
	.setfile_stride_fixup = true,
};

static struct ofw_compat_data compat_data[] = {
	{ "apple,t8142-isp",	(uintptr_t)&apple_isp_hw_t8142 },
	{ NULL,			0 }
};

/*
 * Map [pa, pa + size) at iova on all three ISP DARTs. Where two share a
 * table, the second finds the pages mapped already, to the same place.
 */
static int
isp_dart_map(struct apple_isp *isp, u64 iova, vm_paddr_t pa, u64 size)
{
	vm_paddr_t got;
	int err, i;

	for (i = 0; i < ISP_NUM_DARTS; i++) {
		err = apple_dart_fw_map_vacant(isp->darts[i], iova, pa, size);
		if (err == EBUSY &&
		    apple_dart_fw_lookup(isp->darts[i], iova, size, &got) == 0 &&
		    got == pa)
			err = 0;
		if (err != 0) {
			dev_err(isp->dev, "DART %d: cannot map %#jx+%#jx: %d\n",
			    i, (uintmax_t)iova, (uintmax_t)size, err);
			while (--i >= 0)
				apple_dart_fw_unmap(isp->darts[i], iova, size);
			return (-err);
		}
	}
	return (0);
}

static void
isp_dart_unmap(struct apple_isp *isp, u64 iova, u64 size)
{
	for (int i = 0; i < ISP_NUM_DARTS; i++)
		apple_dart_fw_unmap(isp->darts[i], iova, size);
}

/*
 * A surface: memory the firmware and the host share, physically contiguous
 * and mapped write-combining on the host, at an IOVA from the range above
 * the firmware heap.
 */
struct isp_surf *
isp_alloc_surface(struct apple_isp *isp, u64 size, bool gc)
{
	struct isp_surf *surf;
	vmem_addr_t iova;

	/*
	 * The firmware chooses the size of the extra heap and of SHAREDMALLOC
	 * surfaces. Nothing larger than the IOVA range can be mapped.
	 */
	if (!size || size > isp->iova_size) {
		dev_err(isp->dev, "invalid surface size 0x%jx\n",
		    (uintmax_t)size);
		return (NULL);
	}

	surf = malloc(sizeof(*surf), M_APPLEISP, M_WAITOK | M_ZERO);
	surf->size = roundup2(size, ISP_PAGE_SIZE);
	surf->gc = gc;
	surf->virt = kmem_alloc_contig(surf->size, M_WAITOK | M_ZERO, 0,
	    (1ul << 42) - 1, ISP_PAGE_SIZE, 0, VM_MEMATTR_WRITE_COMBINING);
	if (surf->virt == NULL) {
		dev_err(isp->dev, "cannot allocate 0x%jx\n",
		    (uintmax_t)surf->size);
		goto free_surf;
	}
	surf->pa = pmap_kextract((vm_offset_t)surf->virt);

	if (vmem_alloc(isp->iovas, surf->size, M_BESTFIT | M_WAITOK,
	    &iova) != 0) {
		dev_err(isp->dev, "failed to reserve 0x%jx of iova space\n",
		    (uintmax_t)surf->size);
		goto free_pages;
	}
	surf->iova = iova;
	surf->iova_reserved = true;

	if (isp_dart_map(isp, surf->iova, surf->pa, surf->size) != 0)
		goto unreserve_iova;

	surf->refcount = 1;
	if (surf->gc) {
		sx_xlock(&isp->gc_lock);
		TAILQ_INSERT_TAIL(&isp->gc, surf, head);
		sx_xunlock(&isp->gc_lock);
	}
	return (surf);

unreserve_iova:
	vmem_free(isp->iovas, surf->iova, surf->size);
free_pages:
	kmem_free(surf->virt, surf->size);
free_surf:
	free(surf, M_APPLEISP);
	return (NULL);
}

void
isp_free_surface(struct apple_isp *isp, struct isp_surf *surf)
{
	if (atomic_fetchadd_int(&surf->refcount, -1) != 1)
		return;
	isp_dart_unmap(isp, surf->iova, surf->size);
	if (surf->iova_reserved)
		vmem_free(isp->iovas, surf->iova, surf->size);
	kmem_free(surf->virt, surf->size);
	if (surf->gc) {
		sx_xlock(&isp->gc_lock);
		/* On @gc or, once the firmware freed it, @gc_released */
		TAILQ_REMOVE(&isp->gc, surf, head);
		sx_xunlock(&isp->gc_lock);
	}
	free(surf, M_APPLEISP);
}

/*
 * Free the SHAREDMALLOC surfaces the firmware has released. Only call this
 * once the firmware no longer uses them: after the stream that released them
 * has stopped, or after the firmware has stopped.
 */
void
isp_free_released_surfaces(struct apple_isp *isp)
{
	struct isp_surf *surf;

	sx_xlock(&isp->gc_lock);
	while ((surf = TAILQ_FIRST(&isp->gc_released)) != NULL) {
		TAILQ_REMOVE(&isp->gc_released, surf, head);
		/* isp_free_surface() takes it off @gc */
		TAILQ_INSERT_TAIL(&isp->gc, surf, head);
		sx_xunlock(&isp->gc_lock);
		isp_dbg(isp, "shared surface freed: iova 0x%jx size 0x%jx\n",
		    (uintmax_t)surf->iova, (uintmax_t)surf->size);
		isp_free_surface(isp, surf);
		sx_xlock(&isp->gc_lock);
	}
	sx_xunlock(&isp->gc_lock);
}

void *
apple_isp_translate(struct apple_isp *isp, struct isp_surf *surf,
    dma_addr_t iova, size_t size)
{
	dma_addr_t end = iova + size;

	if (!surf) {
		dev_err(isp->dev, "Failed to translate IPC iova 0x%jx (0x%zx): "
		    "No surface\n", (uintmax_t)iova, size);
		return (NULL);
	}
	if (end < iova || iova < surf->iova || end > surf->iova + surf->size) {
		dev_err(isp->dev, "Failed to translate IPC iova 0x%jx (0x%zx): "
		    "Out of bounds\n", (uintmax_t)iova, size);
		return (NULL);
	}
	return ((char *)surf->virt + (iova - surf->iova));
}

void
isp_wake(struct apple_isp *isp)
{
	mtx_lock(&isp->wait_mtx);
	wakeup(&isp->wait);
	mtx_unlock(&isp->wait_mtx);
}

/*
 * The firmware reaches its fixed MMIO windows through the first DART, whose
 * address filter the ADT describes for them. They are lost with ISP_SYS, so
 * they are programmed again before every firmware start, from isp_dart0's
 * apple,dapf-ranges and apple,dapf-r20 (as Linux's DART driver replays them).
 */
static int
apple_isp_init_dapf(struct apple_isp *isp)
{
	device_t dev = isp->dev;
	pcell_t *cells, xref;
	phandle_t dart;
	bus_addr_t pa;
	bus_size_t size;
	u64 *ranges;
	int n;

	if (OF_getencprop(ofw_bus_get_node(dev), "iommus", &xref,
	    sizeof(xref)) <= 0)
		return (-ENODEV);
	dart = OF_node_from_xref(xref);
	n = OF_getprop_alloc_multi(dart, "apple,dapf-ranges", sizeof(u64),
	    (void **)&ranges);
	if (n <= 0 || (n & 1)) {
		dev_err(dev, "no address filter ranges on the ISP DART\n");
		return (-ENODEV);
	}
	isp->num_dapf = n / 2;
	isp->dapf_windows = malloc(isp->num_dapf * sizeof(*isp->dapf_windows),
	    M_APPLEISP, M_WAITOK);
	for (unsigned int i = 0; i < isp->num_dapf; i++) {
		isp->dapf_windows[i].start = be64toh(ranges[2 * i]);
		isp->dapf_windows[i].end = be64toh(ranges[2 * i + 1]);
	}
	OF_prop_free(ranges);
	if (OF_getencprop_alloc_multi(dart, "apple,dapf-r20", sizeof(u32),
	    (void **)&cells) != (int)isp->num_dapf) {
		dev_err(dev, "apple,dapf-r20 doesn't match the ranges\n");
		return (-EINVAL);
	}
	isp->dapf_r20 = cells;

	/* reg-names = "dart", "dapf" */
	if (ofw_reg_to_paddr(dart, 1, &pa, &size, NULL) != 0 ||
	    size < isp->num_dapf * DAPF_WINDOW_STRIDE) {
		dev_err(dev, "no address filter registers on the ISP DART\n");
		return (-ENODEV);
	}
	isp->dapf = pmap_mapdev(pa, size);
	isp->dapf_size = size;
	return (0);
}

#define	DAPF_RD4(isp, r)	(*(volatile u32 *)((char *)(isp)->dapf + (r)))
#define	DAPF_RD8(isp, r)	(*(volatile u64 *)((char *)(isp)->dapf + (r)))
#define	DAPF_WR4(isp, r, v)	(*(volatile u32 *)((char *)(isp)->dapf + (r)) = (v))
#define	DAPF_WR8(isp, r, v)	(*(volatile u64 *)((char *)(isp)->dapf + (r)) = (v))

/* Program the address filter windows, then check that they took. */
int
apple_isp_program_dapf(struct apple_isp *isp)
{
	bus_size_t w;
	unsigned int i;

	for (i = 0; i < isp->num_dapf; i++) {
		w = i * DAPF_WINDOW_STRIDE;
		DAPF_WR4(isp, w + DAPF_CTRL1, 0);
		DAPF_WR8(isp, w + DAPF_START, isp->dapf_windows[i].start);
		DAPF_WR8(isp, w + DAPF_END, isp->dapf_windows[i].end);
		DAPF_WR4(isp, w + DAPF_CTRL0, DAPF_CTRL0_ISP);
		DAPF_WR4(isp, w + DAPF_CTRL2, isp->dapf_r20[i]);
	}

	/* Only bits 7:0 of CTRL0 and bits 63:2 of END read back. */
	for (i = 0; i < isp->num_dapf; i++) {
		w = i * DAPF_WINDOW_STRIDE;
		if (DAPF_RD4(isp, w + DAPF_CTRL1) ||
		    DAPF_RD8(isp, w + DAPF_START) !=
		    isp->dapf_windows[i].start ||
		    (DAPF_RD8(isp, w + DAPF_END) | 3) !=
		    (isp->dapf_windows[i].end | 3) ||
		    (DAPF_RD4(isp, w + DAPF_CTRL0) & 0xff) != DAPF_CTRL0_ISP ||
		    DAPF_RD4(isp, w + DAPF_CTRL2) != isp->dapf_r20[i]) {
			dev_err(isp->dev, "DAPF window %u (0x%jx-0x%jx) failed "
			    "readback\n", i,
			    (uintmax_t)isp->dapf_windows[i].start,
			    (uintmax_t)isp->dapf_windows[i].end);
			return (-EIO);
		}
	}
	return (0);
}

/*
 * The firmware heap, from /reserved-memory/isp-heap (m1n1): its physical
 * memory and, in iommu-addresses, where the firmware sees it.
 */
/*
 * T8142: the firmware's text and data end at 0x16ec000 (J813 and J815 ADTs,
 * macOS 26 and 27); the heap runs from there to the next 2 MiB boundary.
 */
#define	ISP_T8142_SEG_END	0x16ec000
#define	ISP_T8142_HEAP_TOP	0x1800000

/*
 * Without an isp-heap from m1n1 (a stub older than its ISP17a support,
 * which reserves the firmware segments but carves no heap and disables the
 * ISP): find the end of the firmware from the asc-firmware regions m1n1
 * reserved for this device, and allocate the heap above it ourselves.
 */
static int
apple_isp_alloc_heap(struct apple_isp *isp)
{
	device_t dev = isp->dev;
	phandle_t resv, child;
	pcell_t self, ia[5];
	u64 end = 0;
	void *va;

	self = OF_xref_from_node(ofw_bus_get_node(dev));
	resv = OF_finddevice("/reserved-memory");
	for (child = resv > 0 ? OF_child(resv) : 0; child != 0;
	    child = OF_peer(child)) {
		if (OF_getencprop(child, "iommu-addresses", ia, sizeof(ia)) !=
		    sizeof(ia) || ia[0] != self)
			continue;
		end = MAX(end, ((((u64)ia[1] << 32 | ia[2]) +
		    ((u64)ia[3] << 32 | ia[4])) & isp->fw_iova_mask));
	}
	end = roundup2(end, ISP_PAGE_SIZE);
	if (end != ISP_T8142_SEG_END) {
		dev_err(dev, "firmware segments end at 0x%jx, not 0x%x\n",
		    (uintmax_t)end, ISP_T8142_SEG_END);
		return (-ENODEV);
	}
	isp->fw.heap_base = end;
	isp->fw.heap_top = ISP_T8142_HEAP_TOP;
	/* A heap an earlier attach allocated stays mapped (see detach). */
	if (apple_dart_fw_lookup(isp->darts[0], isp->fw.heap_base,
	    isp->fw.heap_top - isp->fw.heap_base, &isp->fw.heap_pa) == 0) {
		device_printf(dev, "firmware heap 0x%jx-0x%jx kept at %#jx\n",
		    (uintmax_t)isp->fw.heap_base, (uintmax_t)isp->fw.heap_top,
		    (uintmax_t)isp->fw.heap_pa);
		return (0);
	}
	va = kmem_alloc_contig(isp->fw.heap_top - isp->fw.heap_base,
	    M_WAITOK | M_ZERO, 0, (1ul << 42) - 1, ISP_PAGE_SIZE, 0,
	    VM_MEMATTR_WRITE_COMBINING);
	if (va == NULL)
		return (-ENOMEM);
	isp->fw.heap_pa = pmap_kextract((vm_offset_t)va);
	device_printf(dev, "firmware heap 0x%jx-0x%jx allocated at %#jx\n",
	    (uintmax_t)isp->fw.heap_base, (uintmax_t)isp->fw.heap_top,
	    (uintmax_t)isp->fw.heap_pa);
	return (0);
}

static int
apple_isp_init_heap(struct apple_isp *isp)
{
	device_t dev = isp->dev;
	phandle_t node = ofw_bus_get_node(dev), mem;
	pcell_t xref, ia[5], reg[4];

	if (OF_getencprop(node, "memory-region", &xref, sizeof(xref)) <= 0)
		return (apple_isp_alloc_heap(isp));
	mem = OF_node_from_xref(xref);
	/* iommu-addresses = <&isp hi lo hi lo>; reg = <hi lo hi lo> */
	if (OF_getencprop(mem, "iommu-addresses", ia, sizeof(ia)) !=
	    sizeof(ia) ||
	    OF_getencprop(mem, "reg", reg, sizeof(reg)) != sizeof(reg))
		return (apple_isp_alloc_heap(isp));
	isp->fw.heap_base = ((u64)ia[1] << 32 | ia[2]) & isp->fw_iova_mask;
	isp->fw.heap_top = isp->fw.heap_base + ((u64)ia[3] << 32 | ia[4]);
	isp->fw.heap_pa = (u64)reg[0] << 32 | reg[1];
	if (((u64)reg[2] << 32 | reg[3]) != isp->fw.heap_top -
	    isp->fw.heap_base || isp->fw.heap_top <= isp->fw.heap_base) {
		dev_err(dev, "isp-heap reg and iommu-addresses disagree\n");
		return (-EINVAL);
	}
	device_printf(dev, "firmware heap 0x%jx-0x%jx at %#jx\n",
	    (uintmax_t)isp->fw.heap_base, (uintmax_t)isp->fw.heap_top,
	    (uintmax_t)isp->fw.heap_pa);
	return (0);
}

/*
 * Some firmware accesses registers of other blocks, such as the PMGR
 * scratch registers named in PMP_CTRL_SET, through its DART at their
 * physical addresses. Map those windows 1:1.
 */
static int
apple_isp_map_fw_mmio(struct apple_isp *isp)
{
	u64 start = isp->fw.heap_top, end = start + isp->iova_size;
	unsigned int i;
	int err;

	for (i = 0; i < isp->hw->num_fw_mmio; i++) {
		const struct isp_mmio_window *w = &isp->hw->fw_mmio[i];

		/* Keep them out of the range the surfaces come from. */
		if (w->base < end && w->base + w->size > start) {
			dev_err(isp->dev, "MMIO window 0x%jx+0x%jx overlaps "
			    "the IOVA range\n", (uintmax_t)w->base,
			    (uintmax_t)w->size);
			err = -EINVAL;
		} else
			err = isp_dart_map(isp, w->base, w->base, w->size);
		if (err) {
			while (i-- > 0)
				isp_dart_unmap(isp, isp->hw->fw_mmio[i].base,
				    isp->hw->fw_mmio[i].size);
			return (err);
		}
	}
	return (0);
}

/*
 * The firmware's text and data, the asc-firmware regions m1n1 reserved for
 * this device: map them at their IOVAs where the DART lacks them (a table of
 * our own, apple_dart_fw_attach_own()), as Linux's DART driver maps a
 * device's reserved regions.
 */
static int
apple_isp_map_fw_segments(struct apple_isp *isp)
{
	phandle_t resv, child;
	pcell_t self, ia[5], reg[4];
	vm_paddr_t pa, got;
	u64 iova, size;
	int err;

	self = OF_xref_from_node(ofw_bus_get_node(isp->dev));
	resv = OF_finddevice("/reserved-memory");
	for (child = resv > 0 ? OF_child(resv) : 0; child != 0;
	    child = OF_peer(child)) {
		if (OF_getencprop(child, "iommu-addresses", ia, sizeof(ia)) !=
		    sizeof(ia) || ia[0] != self ||
		    !ofw_bus_node_is_compatible(child, "apple,asc-mem") ||
		    OF_getencprop(child, "reg", reg, sizeof(reg)) != sizeof(reg))
			continue;
		iova = ((u64)ia[1] << 32 | ia[2]) & isp->fw_iova_mask;
		size = (u64)ia[3] << 32 | ia[4];
		pa = (u64)reg[0] << 32 | reg[1];
		if (apple_dart_fw_lookup(isp->darts[0], iova, size, &got) == 0) {
			if (got != pa) {
				dev_err(isp->dev, "firmware 0x%jx is mapped to "
				    "%#jx, not %#jx\n", (uintmax_t)iova,
				    (uintmax_t)got, (uintmax_t)pa);
				return (-EEXIST);
			}
			continue;
		}
		err = isp_dart_map(isp, iova, pa, size);
		if (err)
			return (err);
		device_printf(isp->dev, "firmware 0x%jx-0x%jx mapped from "
		    "%#jx\n", (uintmax_t)iova, (uintmax_t)(iova + size),
		    (uintmax_t)pa);
	}
	return (0);
}

#define	ISP_DART_ERROR		0x100
#define	ISP_DART_ERROR_MASK	0x104
#define	ISP_DART_ERROR_MASK_V2	0x4060
#define	ISP_DART_ERROR_ADDR_LO	0x170
#define	ISP_DART_ERROR_ADDR_HI	0x174
#define	ISP_DART_ERROR_STREAMS	0x1c0
#define	ISP_DART_ENABLE_STREAMS	0xc00
#define	ISP_DART_TCR(sid)	(0x1000 + 4 * (sid))
#define	ISP_DART_TCR_REMAP_ENABLE BIT(7)
#define	ISP_DART_TCR_REMAP_SHIFT 8
#define	DART_RD4(isp, i, r)	\
	(*(volatile u32 *)((char *)(isp)->dart_regs[i] + (r)))
#define	DART_WR4(isp, i, r, v)	\
	(*(volatile u32 *)((char *)(isp)->dart_regs[i] + (r)) = (v))

/*
 * The rest of the DART setup macOS 27.2 does on T8142 (HV trace,
 * logs/isp-macos-trace-keep.txt): the J815 ADT's dart-tunables-instance-N
 * (mask, value), streams 0, 4, 5, 8 and 9 enabled (4/5 and 8/9 are the ADT's
 * exclave-sid pairs), and the protection words of dart-isp reg[6] and [7].
 */
static int apple_isp_exclave_remap;
TUNABLE_INT("hw.apple_isp.exclave_remap", &apple_isp_exclave_remap);

static const struct {
	u32 off, mask, val;
} isp_dart_tunables[ISP_NUM_DARTS][6] = {
	{ { 0x20c, 0x80000121, 0x80000121 }, { 0x220, 0xf0f, 0xf0f },
	  { 0x224, 0xffff, 0x909 }, { 0x300, 0x1f31, 0x1 },
	  { 0x308, 0x3ffffffc, 0x10000000 }, { 0x310, 0x3ffffffc, 0x3ffffffc } },
	{ { 0x20c, 0x80000121, 0x80000121 }, { 0x220, 0xf0f0f, 0xf0f0f },
	  { 0x224, 0xffffff, 0x90909 }, { 0x300, 0x1f31, 0x1 },
	  { 0x308, 0x3ffffffc, 0x10000000 }, { 0x310, 0x3ffffffc, 0x3ffffffc } },
	{ { 0x20c, 0x80000021, 0x80000021 }, { 0x220, 0xf0f0f, 0xf0f0f },
	  { 0x224, 0xffffff, 0x90909 }, { 0x300, 0x1f31, 0x1 },
	  { 0x308, 0x3ffffffc, 0x10000000 }, { 0x310, 0x3ffffffc, 0x3ffffffc } },
};
#define	ISP_T8142_DART_PS6	0x380714000ul	/* dart-isp reg[6] */
#define	ISP_T8142_DART_PS7	0x388298000ul	/* dart-isp reg[7] */

static void
isp_or32(vm_paddr_t pa, u32 set)
{
	volatile u32 *w = pmap_mapdev(pa, sizeof(u32));

	*w |= set;
	pmap_unmapdev(__DEVOLATILE(void *, w), sizeof(u32));
}

void
apple_isp_dart_macos_init(struct apple_isp *isp)
{
	for (int i = 0; i < ISP_NUM_DARTS; i++) {
		for (int j = 0; j < 6; j++) {
			u32 off = isp_dart_tunables[i][j].off;

			DART_WR4(isp, i, off, (DART_RD4(isp, i, off) &
			    ~isp_dart_tunables[i][j].mask) |
			    isp_dart_tunables[i][j].val);
		}
		DART_WR4(isp, i, ISP_DART_ERROR_MASK_V2, 0);
		DART_WR4(isp, i, ISP_DART_ENABLE_STREAMS, 0x331);
		/*
		 * The exclave-sid streams (4/5, 8/9): XNU never programs them,
		 * the exclave side does. Show them, and with
		 * hw.apple_isp.exclave_remap fold them onto stream 0, so their
		 * traffic is translated (and faults show) instead of passing.
		 */
		for (int s = 4; s <= 9; s++) {
			if (s == 6 || s == 7)
				continue;
			device_printf(isp->dev, "DART %d sid %d: TCR %#x TTBR "
			    "%#x\n", i, s, DART_RD4(isp, i, ISP_DART_TCR(s)),
			    DART_RD4(isp, i, 0x1400 + 4 * s));
			if (apple_isp_exclave_remap)
				DART_WR4(isp, i, ISP_DART_TCR(s),
				    ISP_DART_TCR_REMAP_ENABLE);
		}
		device_printf(isp->dev, "DART %d: macOS init, 0x20c %#x, TCR0 "
		    "%#x, streams %#x\n", i, DART_RD4(isp, i, 0x20c),
		    DART_RD4(isp, i, ISP_DART_TCR(0)),
		    DART_RD4(isp, i, ISP_DART_ENABLE_STREAMS));
	}
	isp_or32(ISP_T8142_DART_PS6 + 0x3fec, 0x420);
	isp_or32(ISP_T8142_DART_PS6 + 0x3fe8, 0x400);
	isp_or32(ISP_T8142_DART_PS7 + 0x3ff8, BIT(29));
	isp_or32(ISP_T8142_DART_PS7 + 0x3fec, 0x4);
}

/*
 * Map the DARTs' registers, and fold the streams apple,remap-streams names
 * onto their targets (the ISP firmware uses several; aurora's DART driver
 * writes these remaps, which a bare DART from the boot loader lacks).
 */
static int
apple_isp_init_darts(struct apple_isp *isp)
{
	pcell_t *iommus, *remap;
	phandle_t dart;
	bus_addr_t pa;
	int i, n;

	n = OF_getencprop_alloc_multi(ofw_bus_get_node(isp->dev), "iommus",
	    sizeof(*iommus), (void **)&iommus);
	if (n < 2 * ISP_NUM_DARTS) {
		OF_prop_free(iommus);
		return (-EINVAL);
	}
	for (i = 0; i < ISP_NUM_DARTS; i++) {
		dart = OF_node_from_xref(iommus[2 * i]);
		if (ofw_reg_to_paddr(dart, 0, &pa, &isp->dart_regs_size[i],
		    NULL) != 0) {
			OF_prop_free(iommus);
			return (-ENXIO);
		}
		isp->dart_regs[i] = pmap_mapdev(pa, isp->dart_regs_size[i]);

		n = OF_getencprop_alloc_multi(dart, "apple,remap-streams",
		    sizeof(*remap), (void **)&remap);
		for (int j = 0; j + 1 < n; j += 2) {
			u32 src = remap[j], dst = remap[j + 1];

			if (src >= 256 || dst > 0xf)
				continue;
			DART_WR4(isp, i, ISP_DART_TCR(src),
			    ISP_DART_TCR_REMAP_ENABLE |
			    dst << ISP_DART_TCR_REMAP_SHIFT);
			DART_WR4(isp, i, ISP_DART_ENABLE_STREAMS + 4 * (src / 32),
			    DART_RD4(isp, i, ISP_DART_ENABLE_STREAMS +
			    4 * (src / 32)) | BIT(src % 32));
			dev_dbg(isp->dev, "DART %d: stream %u -> %u, TCR %#x\n",
			    i, src, dst, DART_RD4(isp, i, ISP_DART_TCR(src)));
		}
		if (n > 0)
			device_printf(isp->dev, "DART %d: %d stream remaps\n",
			    i, n / 2);
		OF_prop_free(remap);
	}
	OF_prop_free(iommus);
	apple_isp_dart_macos_init(isp);
	return (0);
}

/* Report (and with clear, acknowledge) the DARTs' latched faults. */
void
apple_isp_dart_report(struct apple_isp *isp, bool clear)
{
	u32 err;

	for (int i = 0; i < ISP_NUM_DARTS; i++) {
		if (isp->dart_regs[i] == NULL)
			continue;
		if (clear) {
			/* A masked fault never latches: unmask them all. */
			device_printf(isp->dev, "DART %d: error mask %#x/%#x, "
			    "streams %#x, unmasking\n", i,
			    DART_RD4(isp, i, ISP_DART_ERROR_MASK),
			    DART_RD4(isp, i, ISP_DART_ERROR_MASK_V2),
			    DART_RD4(isp, i, ISP_DART_ENABLE_STREAMS));
			DART_WR4(isp, i, ISP_DART_ERROR_MASK, 0);
			DART_WR4(isp, i, ISP_DART_ERROR_MASK_V2, 0);
		}
		err = DART_RD4(isp, i, ISP_DART_ERROR);
		if (err & BIT(31))
			device_printf(isp->dev, "DART %d fault: error %#x "
			    "(stream %u) at 0x%x%08x, streams %#x\n", i, err,
			    (err >> 20) & 0xff,
			    DART_RD4(isp, i, ISP_DART_ERROR_ADDR_HI),
			    DART_RD4(isp, i, ISP_DART_ERROR_ADDR_LO),
			    DART_RD4(isp, i, ISP_DART_ERROR_STREAMS));
		else if (!clear)
			device_printf(isp->dev, "DART %d: no fault\n", i);
		if (clear && (err & BIT(31)))
			DART_WR4(isp, i, ISP_DART_ERROR, err);
	}
}

static int
apple_isp_init_iommu(struct apple_isp *isp)
{
	device_t dev = isp->dev;
	pcell_t vm[2];
	int err, i;

	isp->fw_iova_mask = isp->hw->fw_iova_mask ?: U64_MAX;
	for (i = 0; i < ISP_NUM_DARTS; i++) {
		err = apple_dart_fw_attach_own(dev, ofw_bus_get_node(dev), i, 2,
		    &isp->darts[i]);
		if (err != 0) {
			dev_err(dev, "DART %d: %d\n", i, err);
			return (-err);
		}
	}
	err = apple_isp_init_darts(isp);
	if (err)
		return (err);

	err = apple_isp_map_fw_segments(isp);
	if (err)
		return (err);

	err = apple_isp_init_heap(isp);
	if (err)
		return (err);

	if (OF_getencprop(ofw_bus_get_node(dev), "apple,dart-vm-size", vm,
	    sizeof(vm)) != sizeof(vm)) {
		dev_err(dev, "failed to read 'apple,dart-vm-size'\n");
		return (-EINVAL);
	}
	isp->iova_size = ((u64)vm[0] << 32 | vm[1]) -
	    (isp->fw.heap_base & 0xffffffff);
	isp->iovas = vmem_create("apple_isp", roundup2(isp->fw.heap_top,
	    ISP_PAGE_SIZE), rounddown2(isp->iova_size, ISP_PAGE_SIZE),
	    ISP_PAGE_SIZE, 0, M_WAITOK);

	/* The heap m1n1 carved, where the firmware expects it */
	err = isp_dart_map(isp, isp->fw.heap_base, isp->fw.heap_pa,
	    isp->fw.heap_top - isp->fw.heap_base);
	if (err)
		return (err);

	return (apple_isp_map_fw_mmio(isp));
}

static int
apple_isp_init_presets(struct apple_isp *isp)
{
	device_t dev = isp->dev;
	struct isp_preset *preset;
	phandle_t np, child;
	u32 xy[2], xywh[4];

	np = ofw_bus_find_child(ofw_bus_get_node(dev), "sensor-presets");
	if (np <= 0) {
		dev_err(dev, "failed to get DT node 'presets'\n");
		return (-EINVAL);
	}
	isp->num_presets = 0;
	for (child = OF_child(np); child != 0; child = OF_peer(child))
		isp->num_presets++;
	if (!isp->num_presets) {
		dev_err(dev, "no sensor presets found\n");
		return (-EINVAL);
	}
	isp->presets = malloc(isp->num_presets * sizeof(*isp->presets),
	    M_APPLEISP, M_WAITOK | M_ZERO);

	preset = isp->presets;
	for (child = OF_child(np); child != 0; child = OF_peer(child)) {
		if (OF_getencprop(child, "apple,config-index", &preset->index,
		    sizeof(preset->index)) <= 0) {
			dev_err(dev, "no apple,config-index property\n");
			return (-EINVAL);
		}
		if (OF_getencprop(child, "apple,input-size", xy,
		    sizeof(xy)) != sizeof(xy)) {
			dev_err(dev, "failed to read 'apple,input-size'\n");
			return (-EINVAL);
		}
		preset->input_dim = (struct coord){ xy[0], xy[1] };
		if (OF_getencprop(child, "apple,output-size", xy,
		    sizeof(xy)) != sizeof(xy)) {
			dev_err(dev, "failed to read 'apple,output-size'\n");
			return (-EINVAL);
		}
		preset->output_dim = (struct coord){ xy[0], xy[1] };
		if (OF_getencprop(child, "apple,crop", xywh,
		    sizeof(xywh)) != sizeof(xywh)) {
			dev_err(dev, "failed to read 'apple,crop' property\n");
			return (-EINVAL);
		}
		preset->crop_offset = (struct coord){ xywh[0], xywh[1] };
		preset->crop_size = (struct coord){ xywh[2], xywh[3] };
		preset++;
	}
	return (0);
}

static int
apple_isp_intr(void *arg)
{
	struct apple_isp *isp = arg;

	isp_mbox2_write32(isp, ISP_MBOX2_IRQ_ACK,
	    isp_mbox_read32(isp, ISP_MBOX_IRQ_INTERRUPT));
	taskqueue_enqueue(isp->tq, &isp->irq_task);
	return (FILTER_HANDLED);
}

static int
apple_isp_probe(device_t dev)
{
	/*
	 * Not ofw_bus_status_okay(): an m1n1 without ISP17a support disables
	 * the node only for want of a heap, which attach allocates instead.
	 */
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Apple ISP (" APPLE_ISP_CARD_NAME ")");
	return (BUS_PROBE_DEFAULT);
}

static int apple_isp_detach(device_t dev);

static struct resource *
isp_map_named(device_t dev, const char *name)
{
	struct resource *res;
	int rid;

	if (ofw_bus_find_string_index(ofw_bus_get_node(dev), "reg-names", name,
	    &rid) != 0)
		return (NULL);
	res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (res == NULL)
		device_printf(dev, "cannot map '%s'\n", name);
	return (res);
}

static int
apple_isp_attach(device_t dev)
{
	struct apple_isp *isp = device_get_softc(dev);
	phandle_t node = ofw_bus_get_node(dev);
	pcell_t *pds;
	int err, rid;

	isp->dev = dev;
	isp->hw = (const struct apple_isp_hw *)
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data;
	sx_init(&isp->video_lock, "apple_isp video");
	sx_init(&isp->gc_lock, "apple_isp gc");
	mtx_init(&isp->wait_mtx, "apple_isp wait", NULL, MTX_DEF);
	TAILQ_INIT(&isp->gc);
	TAILQ_INIT(&isp->gc_released);
	TASK_INIT(&isp->irq_task, 0, apple_isp_irq_task, isp);
	isp->tq = taskqueue_create("apple_isp", M_WAITOK,
	    taskqueue_thread_enqueue, &isp->tq);
	taskqueue_start_threads(&isp->tq, 1, PI_DISK, "%s",
	    device_get_nameunit(dev));

	if (OF_getencprop(node, "apple,platform-id", &isp->platform_id,
	    sizeof(isp->platform_id)) <= 0) {
		device_printf(dev, "failed to get 'apple,platform-id'\n");
		err = ENXIO;
		goto fail;
	}
	if (OF_getencprop(node, "apple,temporal-filter", &isp->temporal_filter,
	    sizeof(isp->temporal_filter)) <= 0)
		isp->temporal_filter = 0;
	/* the sensor connection layout, for firmware that takes it at boot */
	OF_getencprop(node, "apple,cam-connections-scheme",
	    &isp->cam_connections_scheme, sizeof(isp->cam_connections_scheme));

	if ((err = -apple_isp_init_presets(isp)) != 0)
		goto fail;

	/*
	 * ISP_SYS (domain 0) is the DARTs' too: on, and left on, so the boot
	 * loader's DART tables stay. The firmware boot switches the others.
	 */
	isp->pd_count = OF_getencprop_alloc_multi(node, "power-domains",
	    sizeof(*pds), (void **)&pds);
	OF_prop_free(pds);
	if (isp->pd_count < 2 || (err = apple_pmgr_set_domain(dev, 0,
	    true)) != 0) {
		device_printf(dev, "cannot power ISP_SYS on\n");
		err = ENXIO;
		goto fail;
	}

	isp->coproc = isp_map_named(dev, "coproc");
	isp->mbox = isp_map_named(dev, "mbox");
	isp->gpio = isp_map_named(dev, "gpio");
	isp->wdt = isp_map_named(dev, "wdt");
	if (isp->coproc == NULL || isp->mbox == NULL || isp->gpio == NULL) {
		err = ENXIO;
		goto fail;
	}
	if (rman_get_size(isp->mbox) < isp->hw->mbox2_offset + ISP_MBOX2_SIZE) {
		device_printf(dev, "mbox window too small for the doorbells\n");
		err = EINVAL;
		goto fail;
	}
	isp->mbox2 = isp->hw->mbox2_offset;

	if ((err = -apple_isp_init_dapf(isp)) != 0)
		goto fail;

	rid = 0;
	isp->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE);
	if (isp->irq_res == NULL) {
		device_printf(dev, "cannot allocate the interrupt\n");
		err = ENXIO;
		goto fail;
	}

	if ((err = -apple_isp_init_iommu(isp)) != 0)
		goto fail;

	if ((err = -apple_isp_alloc_firmware_surface(isp)) != 0)
		goto fail;

	err = bus_setup_intr(dev, isp->irq_res, INTR_TYPE_MISC | INTR_MPSAFE,
	    apple_isp_intr, NULL, isp, &isp->irq_cookie);
	if (err != 0) {
		device_printf(dev, "cannot set up the interrupt\n");
		goto fail;
	}

	sx_xlock(&isp->video_lock);
	err = -apple_isp_detect_camera(isp);
	sx_xunlock(&isp->video_lock);
	if (err != 0) {
		device_printf(dev, "failed to detect camera: %d\n", err);
		goto fail;
	}
	if ((err = -apple_isp_setup_video(isp)) != 0) {
		device_printf(dev, "failed to set up the video device: %d\n",
		    err);
		goto fail;
	}
	return (0);

fail:
	apple_isp_detach(dev);
	return (err);
}

static int
apple_isp_detach(device_t dev)
{
	struct apple_isp *isp = device_get_softc(dev);

	apple_isp_remove_video(isp);
	if (isp->tq != NULL) {
		sx_xlock(&isp->video_lock);
		apple_isp_firmware_halt(isp);
		sx_xunlock(&isp->video_lock);
	}
	if (isp->irq_cookie != NULL)
		bus_teardown_intr(dev, isp->irq_res, isp->irq_cookie);
	if (isp->tq != NULL) {
		taskqueue_drain(isp->tq, &isp->irq_task);
		taskqueue_free(isp->tq);
	}
	if (isp->ipc_surf != NULL)
		apple_isp_free_firmware_surface(isp);
	/*
	 * The heap and MMIO windows stay mapped, and the IOVA arena with
	 * them: the DART tables are the boot loader's, kept for a reattach.
	 */
	if (isp->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ,
		    rman_get_rid(isp->irq_res), isp->irq_res);
	if (isp->dapf != NULL)
		pmap_unmapdev(isp->dapf, isp->dapf_size);
	for (int i = 0; i < ISP_NUM_DARTS; i++)
		if (isp->dart_regs[i] != NULL)
			pmap_unmapdev(isp->dart_regs[i], isp->dart_regs_size[i]);
	if (isp->wdt != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(isp->wdt), isp->wdt);
	if (isp->gpio != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(isp->gpio), isp->gpio);
	if (isp->mbox != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(isp->mbox), isp->mbox);
	if (isp->coproc != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(isp->coproc), isp->coproc);
	free(isp->presets, M_APPLEISP);
	free(isp->dapf_windows, M_APPLEISP);
	OF_prop_free(isp->dapf_r20);
	mtx_destroy(&isp->wait_mtx);
	sx_destroy(&isp->gc_lock);
	sx_destroy(&isp->video_lock);
	return (0);
}

static device_method_t apple_isp_methods[] = {
	DEVMETHOD(device_probe,		apple_isp_probe),
	DEVMETHOD(device_attach,	apple_isp_attach),
	DEVMETHOD(device_detach,	apple_isp_detach),

	DEVMETHOD_END
};

static driver_t apple_isp_driver = {
	"apple_isp",
	apple_isp_methods,
	sizeof(struct apple_isp),
};

DRIVER_MODULE(apple_isp, simplebus, apple_isp_driver, 0, 0);
MODULE_VERSION(apple_isp, 1);
SIMPLEBUS_PNP_INFO(compat_data);
