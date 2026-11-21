/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Minimal Apple DART (T8110 layout) support for bring-up: point a device's
 * DART streams at one shared page table that maps every RAM page at
 * IOVA == PA, so its DMA works without a busdma/IOMMU backend. There is no
 * isolation. The USB DARTs on T8142 (DART 2.4) ignore the bypass bits, hence
 * the identity table instead. apple_dart_map() adds pages at other IOVAs to
 * the same table, for devices whose firmware wants addresses in a window of
 * its own (MTP: "apple,dma-range"). RAM is above 4 GB on these SoCs, so low
 * IOVAs never collide with the identity part.
 *
 * Table format (16 KiB granule, 42-bit IOVA, as Linux io-pgtable-dart):
 * three levels of 2048 8-byte entries indexed by IOVA[41:36], [35:25] and
 * [24:14]; table and page entries hold PA[41:14] in bits [37:10].
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sx.h>
#include <sys/time.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/vm_dumpset.h>
#include <vm/vm_extern.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/cpufunc.h>
#include <machine/md_var.h>
#include <machine/vmparam.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <arm64/apple/apple_dart.h>
#include <arm64/apple/apple_pmgr.h>

#define	DART_PARAMS3			0x08
#define	DART_PARAMS3_VA_WIDTH(r)	(((r) >> 16) & 0x3f)
#define	DART_PARAMS3_VERSION(r)		((r) & 0xffff)	/* major.minor */
#define	DART_PARAMS4			0x0c
#define	DART_PARAMS4_NUM_SIDS(r)		((r) & 0x1ff)

#define	DART_T8110_TLB_CMD		0x80
#define	DART_T8110_TLB_CMD_BUSY		(1u << 31)
#define	DART_T8110_TLB_CMD_V2		(1u << 15)	/* rev 2.2+ */
#define	DART_T8110_TLB_CMD_FLUSH_SID	(1u << 8)
#define	DART_T8110_PROTECT		0x200
#define	DART_T8110_PROTECT_TTBR_TCR	(1u << 0)
#define	DART_T8110_ENABLE_STREAMS	0xc00
#define	DART_T8110_TCR(sid)		(0x1000 + 4 * (sid))
#define	DART_T8110_TCR_FOUR_LEVEL	(1u << 3)
#define	DART_T8110_TCR_TRANSLATE	(1u << 0)
#define	DART_T8110_TTBR(sid)		(0x1400 + 4 * (sid))
#define	DART_T8110_TTBR_VALID		(1u << 0)
#define	DART_T8110_MAX_SID		256
#define	DART_V2_EXCEPTION(word)		(0x4000 + 4 * (word))
#define	DART_V2_ERROR_MASK		0x4060
#define	DART_V24_ERROR_ENABLE		0x4200
#define	DART_V24_ERROR_STATUS		0x4204
#define	DART_V24_SID_ENABLE(word)	(0x4220 + 4 * (word))
#define	DART_V2_SID_STATUS(sid)		(0x8000 + 0x40 * (sid))
#define	DART_V2_SID_ADDR_LO(sid)		(DART_V2_SID_STATUS(sid) + 8)
#define	DART_V2_SID_ADDR_HI(sid)		(DART_V2_SID_STATUS(sid) + 12)
#define	DART_V2_DIAG_LOCK		0x210
#define	DART_V2_BLOCK_ERROR		0x704
#define	DART_V2_BLOCK_ERROR_MASK	0x777

#define	DART_PAGE_SHIFT		14
#define	DART_PAGE_SIZE		(1ul << DART_PAGE_SHIFT)
#define	DART_PTES		2048
#define	DART_IOVA_BITS		42

#define	DART_PTE_VALID		(1ul << 0)
#define	DART_PTE_SP_END		(0xffful << 40)
#define	DART_PTE_PA(pa)		(((pa) >> 4) & 0x3ffffffc00ul)

#define	DART_RD4(va, o)		(*(volatile uint32_t *)((va) + (o)))
#define	DART_WR4(va, o, v)	(*(volatile uint32_t *)((va) + (o)) = (v))

static struct sx apple_dart_lock;
SX_SYSINIT(apple_dart_lock, &apple_dart_lock, "apple_dart");
static uint64_t *apple_dart_l1;		/* shared identity table */
static uint64_t *apple_dart_priv_l1;	/* private table: only mapped buffers */

static uint64_t *
apple_dart_alloc_table(void)
{

	return (kmem_alloc_contig(DART_PAGE_SIZE, M_WAITOK | M_ZERO, 0,
	    ~(vm_paddr_t)0, DART_PAGE_SIZE, 0, VM_MEMATTR_DEFAULT));
}

static uint64_t *
apple_dart_next(uint64_t *table, u_int idx)
{
	uint64_t *next;

	if ((table[idx] & DART_PTE_VALID) == 0) {
		next = apple_dart_alloc_table();
		table[idx] = DART_PTE_PA(pmap_kextract((vm_offset_t)next)) |
		    DART_PTE_VALID;
		return (next);
	}
	return ((uint64_t *)PHYS_TO_DMAP(((table[idx] & 0x3ffffffc00ul) << 4)));
}

static void
apple_dart_flush_tables(uint64_t *table, int level)
{
	u_int i;

	if (level < 3)
		for (i = 0; i < DART_PTES; i++)
			if (table[i] & DART_PTE_VALID)
				apple_dart_flush_tables((uint64_t *)PHYS_TO_DMAP(
				    (table[i] & 0x3ffffffc00ul) << 4), level + 1);
	cpu_dcache_wb_range(table, DART_PAGE_SIZE);
}

/* Build the IOVA == PA table over all of RAM (dump_avail includes the kernel). */
static void
apple_dart_build_identity(void)
{
	uint64_t *l2, *l3;
	vm_paddr_t pa, end;
	u_long pages;
	int i;

	apple_dart_l1 = apple_dart_alloc_table();
	pages = 0;
	for (i = 0; dump_avail[i + 1] != 0; i += 2) {
		pa = rounddown2(dump_avail[i], DART_PAGE_SIZE);
		end = roundup2(dump_avail[i + 1], DART_PAGE_SIZE);
		for (; pa < end && pa < (1ul << DART_IOVA_BITS);
		    pa += DART_PAGE_SIZE, pages++) {
			l2 = apple_dart_next(apple_dart_l1, (pa >> 36) & 0x7ff);
			l3 = apple_dart_next(l2, (pa >> 25) & 0x7ff);
			l3[(pa >> 14) & 0x7ff] = DART_PTE_PA(pa) |
			    DART_PTE_SP_END | DART_PTE_VALID;
		}
	}
	apple_dart_flush_tables(apple_dart_l1, 1);
	dsb(sy);
	printf("apple_dart: identity table maps %lu MB of RAM\n",
	    pages * DART_PAGE_SIZE >> 20);
}

/*
 * Flush one stream's TLB. From revision 2.2 the stream flush is the V2
 * command (aurora's apple_dart_t8110_hw_tlb_command).
 */
static void
apple_dart_tlb_flush(device_t dev, vm_offset_t va, u_int sid)
{
	uint32_t cmd;
	int i;

	for (i = 0; i < 1000 &&
	    (DART_RD4(va, DART_T8110_TLB_CMD) & DART_T8110_TLB_CMD_BUSY); i++)
		DELAY(1);
	cmd = DART_T8110_TLB_CMD_FLUSH_SID | sid;
	if (DART_PARAMS3_VERSION(DART_RD4(va, DART_PARAMS3)) >= 0x0202)
		cmd |= DART_T8110_TLB_CMD_V2;
	dsb(sy);
	DART_WR4(va, DART_T8110_TLB_CMD, cmd);
	for (i = 0; i < 1000 &&
	    (DART_RD4(va, DART_T8110_TLB_CMD) & DART_T8110_TLB_CMD_BUSY); i++)
		DELAY(1);
	if (i == 1000)
		device_printf(dev, "DART TLB flush of sid %u timed out\n", sid);
}

/* The PA that iova maps to in the shared table, or ~0. */
static vm_paddr_t
apple_dart_walk(uint64_t *t, uint64_t iova)
{
	uint64_t pte;
	int shift;

	for (shift = 36; t != NULL && shift >= 14; shift -= 11) {
		pte = t[(iova >> shift) & 0x7ff];
		if ((pte & DART_PTE_VALID) == 0)
			break;
		if (shift == 14)
			return (((pte & 0x3ffffffc00ul) << 4) |
			    (iova & (DART_PAGE_SIZE - 1)));
		t = (uint64_t *)PHYS_TO_DMAP((pte & 0x3ffffffc00ul) << 4);
	}
	return (~(vm_paddr_t)0);
}

/*
 * Where the range [iova, iova + size) sits in RAM, if contiguous, from the
 * identity table (for buffers firmware placed itself).
 */
int
apple_dart_lookup(uint64_t iova, size_t size, vm_paddr_t *pap)
{
	vm_paddr_t pa;
	size_t off;
	int error;

	error = 0;
	sx_slock(&apple_dart_lock);
	pa = apple_dart_walk(apple_dart_l1, iova);
	for (off = DART_PAGE_SIZE - (iova & (DART_PAGE_SIZE - 1));
	    pa != ~(vm_paddr_t)0 && off < size; off += DART_PAGE_SIZE)
		if (apple_dart_walk(apple_dart_l1, iova + off) != pa + off) {
			error = EFAULT;
			break;
		}
	sx_sunlock(&apple_dart_lock);
	if (pa == ~(vm_paddr_t)0)
		return (ENOENT);
	if (error == 0)
		*pap = pa;
	return (error);
}

static int
apple_dart_setup_table(device_t dev, phandle_t dart, u_int sid, uint64_t *l1);

static int
apple_dart_setup_one(device_t dev, phandle_t dart, u_int sid)
{
	return (apple_dart_setup_table(dev, dart, sid, apple_dart_l1));
}

static int
apple_dart_setup_table(device_t dev, phandle_t dart, u_int sid, uint64_t *l1)
{
	bus_addr_t pa;
	bus_size_t size;
	vm_offset_t va;
	vm_paddr_t l1pa;
	uint32_t tcr, streams;
	int error;

	if (!ofw_bus_node_is_compatible(dart, "apple,t8110-dart")) {
		device_printf(dev, "unsupported DART type\n");
		return (ENXIO);
	}
	if (sid >= DART_T8110_MAX_SID ||
	    ofw_reg_to_paddr(dart, 0, &pa, &size, NULL) != 0)
		return (ENXIO);

	va = (vm_offset_t)pmap_mapdev(pa, size);
	error = 0;
	if (DART_PARAMS3_VA_WIDTH(DART_RD4(va, DART_PARAMS3)) < DART_IOVA_BITS ||
	    (DART_RD4(va, DART_T8110_PROTECT) & DART_T8110_PROTECT_TTBR_TCR)) {
		device_printf(dev, "DART %#lx: params3 %#x protect %#x: "
		    "can't use it\n", pa, DART_RD4(va, DART_PARAMS3),
		    DART_RD4(va, DART_T8110_PROTECT));
		error = ENXIO;
		goto out;
	}

	l1pa = pmap_kextract((vm_offset_t)l1);
	DART_WR4(va, DART_T8110_TCR(sid), 0);
	DART_WR4(va, DART_T8110_TTBR(sid),
	    (l1pa >> DART_PAGE_SHIFT) << 2 | DART_T8110_TTBR_VALID);
	apple_dart_tlb_flush(dev, va, sid);
	DART_WR4(va, DART_T8110_TCR(sid),
	    DART_T8110_TCR_TRANSLATE | DART_T8110_TCR_FOUR_LEVEL);
	streams = DART_T8110_ENABLE_STREAMS + 4 * (sid / 32);
	DART_WR4(va, streams, DART_RD4(va, streams) | (1u << (sid % 32)));

	tcr = DART_RD4(va, DART_T8110_TCR(sid));
	if (bootverbose || (tcr & DART_T8110_TCR_TRANSLATE) == 0)
		device_printf(dev, "DART %#lx sid %u: TCR %#x TTBR %#x\n", pa,
		    sid, tcr, DART_RD4(va, DART_T8110_TTBR(sid)));
	if ((tcr & DART_T8110_TCR_TRANSLATE) == 0)
		error = EIO;
out:
	pmap_unmapdev((void *)va, size);
	return (error);
}

static void
apple_dart_flush_sid(device_t dev, phandle_t dart, u_int sid)
{
	bus_addr_t pa;
	bus_size_t size;
	vm_offset_t va;

	if (sid >= DART_T8110_MAX_SID ||
	    ofw_reg_to_paddr(dart, 0, &pa, &size, NULL) != 0)
		return;
	va = (vm_offset_t)pmap_mapdev(pa, size);
	apple_dart_tlb_flush(dev, va, sid);
	pmap_unmapdev((void *)va, size);
}

/*
 * Map [pa, pa + size) at iova for dev (after apple_dart_identity), then
 * flush dev's streams. Page-aligned; never unmapped.
 */
int
apple_dart_map(device_t dev, uint64_t iova, vm_paddr_t pa, size_t size)
{
	phandle_t node;
	pcell_t *cells;
	uint64_t *l2, *l3, end;
	int i, n;

	if (((iova | pa | size) & (DART_PAGE_SIZE - 1)) != 0 ||
	    iova + size > (1ul << DART_IOVA_BITS))
		return (EINVAL);
	sx_xlock(&apple_dart_lock);
	if (apple_dart_l1 == NULL) {
		sx_xunlock(&apple_dart_lock);
		return (ENXIO);
	}
	for (end = iova + size; iova < end; iova += DART_PAGE_SIZE,
	    pa += DART_PAGE_SIZE) {
		l2 = apple_dart_next(apple_dart_l1, (iova >> 36) & 0x7ff);
		l3 = apple_dart_next(l2, (iova >> 25) & 0x7ff);
		l3[(iova >> 14) & 0x7ff] = DART_PTE_PA(pa) | DART_PTE_SP_END |
		    DART_PTE_VALID;
	}
	apple_dart_flush_tables(apple_dart_l1, 1);
	dsb(sy);

	node = ofw_bus_get_node(dev);
	n = OF_getencprop_alloc_multi(node, "iommus", sizeof(*cells),
	    (void **)&cells);
	for (i = 0; i + 1 < n; i += 2)
		apple_dart_flush_sid(dev, OF_node_from_xref(cells[i]),
		    cells[i + 1]);
	sx_xunlock(&apple_dart_lock);
	if (n > 0)
		OF_prop_free(cells);
	return (0);
}

/*
 * Stream-level variants for devices without an "iommus" property (PCI
 * functions: their stream comes from the host bridge's "iommu-map").
 */
int
apple_dart_identity_stream(device_t dev, phandle_t dart, u_int sid)
{
	int error;

	sx_xlock(&apple_dart_lock);
	if (apple_dart_l1 == NULL)
		apple_dart_build_identity();
	error = apple_dart_setup_one(dev, dart, sid);
	sx_xunlock(&apple_dart_lock);
	return (error);
}

/*
 * Translate a stream through a table holding only what apple_dart_map_stream()
 * puts there: for a device whose firmware DMAs to addresses of its own
 * choosing, which the identity table would let reach any RAM.
 */
int
apple_dart_private_stream(device_t dev, phandle_t dart, u_int sid)
{
	int error;

	sx_xlock(&apple_dart_lock);
	if (apple_dart_priv_l1 == NULL) {
		apple_dart_priv_l1 = apple_dart_alloc_table();
		cpu_dcache_wb_range(apple_dart_priv_l1, DART_PAGE_SIZE);
	}
	error = apple_dart_setup_table(dev, dart, sid, apple_dart_priv_l1);
	sx_xunlock(&apple_dart_lock);
	return (error);
}

int
apple_dart_map_stream(device_t dev, phandle_t dart, u_int sid, uint64_t iova,
    vm_paddr_t pa, size_t size)
{
	uint64_t *l2, *l3, *tbl, end;

	if (((iova | pa | size) & (DART_PAGE_SIZE - 1)) != 0 ||
	    iova + size > (1ul << DART_IOVA_BITS))
		return (EINVAL);
	sx_xlock(&apple_dart_lock);
	tbl = apple_dart_priv_l1 != NULL ? apple_dart_priv_l1 : apple_dart_l1;
	if (tbl == NULL) {
		sx_xunlock(&apple_dart_lock);
		return (ENXIO);
	}
	for (end = iova + size; iova < end; iova += DART_PAGE_SIZE,
	    pa += DART_PAGE_SIZE) {
		l2 = apple_dart_next(tbl, (iova >> 36) & 0x7ff);
		l3 = apple_dart_next(l2, (iova >> 25) & 0x7ff);
		l3[(iova >> 14) & 0x7ff] = DART_PTE_PA(pa) | DART_PTE_SP_END |
		    DART_PTE_VALID;
	}
	apple_dart_flush_tables(tbl, 1);
	dsb(sy);
	apple_dart_flush_sid(dev, dart, sid);
	sx_xunlock(&apple_dart_lock);
	return (0);
}

/* XXX bring-up: walk the shared table; returns the PA an IOVA maps to, or ~0 */
vm_paddr_t
apple_dart_translate(uint64_t iova)
{
	uint64_t *t, pte;
	int shift;

	if (apple_dart_l1 == NULL && apple_dart_priv_l1 == NULL)
		return (~(vm_paddr_t)0);
	t = apple_dart_priv_l1 != NULL ? apple_dart_priv_l1 : apple_dart_l1;
	for (shift = 36; shift >= 14; shift -= 11) {
		pte = t[(iova >> shift) & 0x7ff];
		if ((pte & DART_PTE_VALID) == 0)
			return (~(vm_paddr_t)0);
		if (shift == 14)
			return (((pte & 0x3ffffffc00ul) << 4) |
			    (iova & (DART_PAGE_SIZE - 1)));
		t = (uint64_t *)PHYS_TO_DMAP((pte & 0x3ffffffc00ul) << 4);
	}
	return (~(vm_paddr_t)0);
}

/* XXX bring-up: OR bits into the leaf PTE of an IOVA, then flush the stream */
int
apple_dart_pte_bits(device_t dev, phandle_t dart, u_int sid, uint64_t iova,
    uint64_t bits)
{
	uint64_t *t, pte;
	int shift;

	t = apple_dart_priv_l1 != NULL ? apple_dart_priv_l1 : apple_dart_l1;
	if (t == NULL)
		return (ENXIO);
	for (shift = 36; shift >= 14; shift -= 11) {
		pte = t[(iova >> shift) & 0x7ff];
		if ((pte & DART_PTE_VALID) == 0)
			return (ENOENT);
		if (shift == 14) {
			t[(iova >> shift) & 0x7ff] = pte | bits;
			cpu_dcache_wb_range(&t[(iova >> shift) & 0x7ff], 8);
			apple_dart_flush_sid(dev, dart, sid);
			return (0);
		}
		t = (uint64_t *)PHYS_TO_DMAP((pte & 0x3ffffffc00ul) << 4);
	}
	return (ENOENT);
}

/* Map all RAM at IOVA == PA on every stream in dev's "iommus" (<&dart sid>). */
int
apple_dart_identity(device_t dev)
{
	phandle_t node;
	pcell_t *cells;
	int i, n, error;

	node = ofw_bus_get_node(dev);
	n = OF_getencprop_alloc_multi(node, "iommus", sizeof(*cells),
	    (void **)&cells);
	if (n <= 0)
		return (0);
	sx_xlock(&apple_dart_lock);
	if (apple_dart_l1 == NULL)
		apple_dart_build_identity();
	error = 0;
	for (i = 0; i + 1 < n && error == 0; i += 2)
		error = apple_dart_setup_one(dev,
		    OF_node_from_xref(cells[i]), cells[i + 1]);
	sx_xunlock(&apple_dart_lock);
	OF_prop_free(cells);
	return (error);
}

/* Look up a PCI requester ID in a host bridge's one-cell DART map. */
int
apple_dart_pci_stream(phandle_t host, u_int rid, phandle_t *dart, u_int *sid)
{
	pcell_t *map, mask;
	phandle_t node;
	int error, i, n;

	if (OF_getencprop(host, "iommu-map-mask", &mask, sizeof(mask)) ==
	    sizeof(mask))
		rid &= mask;
	n = OF_getencprop_alloc_multi(host, "iommu-map", sizeof(*map),
	    (void **)&map);
	if (n <= 0)
		return (ENOENT);
	error = ENOENT;
	for (i = 0; i + 3 < n; i += 4) {
		if (rid < map[i] || rid - map[i] >= map[i + 3])
			continue;
		node = OF_node_from_xref(map[i + 1]);
		if (node <= 0 || map[i + 2] >= DART_T8110_MAX_SID ||
		    rid - map[i] >= DART_T8110_MAX_SID - map[i + 2]) {
			error = EINVAL;
			break;
		}
		*dart = node;
		*sid = map[i + 2] + rid - map[i];
		error = 0;
		break;
	}
	OF_prop_free(map);
	return (error);
}

struct apple_dart_irq_softc {
	device_t dev;
	struct resource *mem;
	struct resource *irq;
	void *ih;
	u_int nsids;
	struct timeval last_log;
	int log_count;
};

static void
apple_dart_error_intr(void *arg)
{
	struct apple_dart_irq_softc *sc = arg;
	uint64_t addr;
	uint32_t block, error, pending, bits, status;
	u_int bit, sid, word;

	for (word = 0; word < howmany(sc->nsids, 32); word++) {
		pending = bus_read_4(sc->mem, DART_V2_EXCEPTION(word));
		for (bits = pending; bits != 0; bits &= bits - 1) {
			bit = ffs(bits) - 1;
			sid = 32 * word + bit;
			if (sid >= sc->nsids)
				break;
			status = bus_read_4(sc->mem, DART_V2_SID_STATUS(sid));
			addr = bus_read_4(sc->mem, DART_V2_SID_ADDR_LO(sid));
			addr |= (uint64_t)bus_read_4(sc->mem,
			    DART_V2_SID_ADDR_HI(sid)) << 32;
			if (ppsratecheck(&sc->last_log, &sc->log_count, 5))
				device_printf(sc->dev,
				    "SID %u fault at %#jx, status %#x\n", sid,
				    (uintmax_t)addr, status);
		}
		if (pending != 0)
			bus_write_4(sc->mem, DART_V2_EXCEPTION(word), pending);
	}

	error = bus_read_4(sc->mem, DART_V24_ERROR_STATUS);
	block = bus_read_4(sc->mem, DART_V2_BLOCK_ERROR) &
	    DART_V2_BLOCK_ERROR_MASK;
	if (error != 0)
		bus_write_4(sc->mem, DART_V24_ERROR_STATUS, error);
	if (block != 0)
		bus_write_4(sc->mem, DART_V2_BLOCK_ERROR, block);
	if ((error != 0 || block != 0) &&
	    ppsratecheck(&sc->last_log, &sc->log_count, 5))
		device_printf(sc->dev, "error status %#x, block status %#x\n",
		    error, block);
	dsb(sy);
	(void)bus_read_4(sc->mem, DART_V2_EXCEPTION(0));
}

/* m1n1 leaves the N1 PCIe port and its DART powered for its enabled DT node. */
static bool
apple_dart_is_n1(phandle_t dart)
{
	phandle_t node, host, mapped;
	pcell_t reg[5];
	u_int sid, rid;

	node = OF_finddevice("wifi0");
	if (node <= 0 || !ofw_bus_node_status_okay(node) ||
	    !ofw_bus_node_is_compatible(node, "pci106b,1902") ||
	    OF_getencprop(node, "reg", reg, sizeof(reg)) != sizeof(reg))
		return (false);
	rid = (reg[0] >> 8) & 0xffff;
	for (host = OF_parent(node); host > 0; host = OF_parent(host)) {
		if (!ofw_bus_node_status_okay(host))
			return (false);
		if (OF_hasprop(host, "iommu-map"))
			return (apple_dart_pci_stream(host, rid, &mapped,
			    &sid) == 0 && mapped == dart);
	}
	return (false);
}

/* Match DARTs used by the native N1, MTP and USB drivers. */
static int
apple_dart_irq_probe(device_t dev)
{
	phandle_t dart, node;
	pcell_t *refs;
	int i, n;
	bool mtp, usb;

	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "apple,t8142-dart"))
		return (ENXIO);
	dart = ofw_bus_get_node(dev);
	if (apple_dart_is_n1(dart)) {
		device_set_desc(dev, "Apple N1 PCIe DART error interrupt");
		return (BUS_PROBE_DEFAULT);
	}
	for (node = OF_child(OF_parent(dart)); node != 0; node = OF_peer(node)) {
		if (!ofw_bus_node_status_okay(node))
			continue;
		mtp = ofw_bus_node_is_compatible(node, "apple,t8142-mtp");
		usb = ofw_bus_node_is_compatible(node, "apple,t8142-dwc3");
		if (!mtp && !usb)
			continue;
		/* USB DART registers require their port's PMGR domain. */
		if (usb && OF_getproplen(dart, "power-domains") <= 0)
			continue;
		n = OF_getencprop_alloc_multi(node, "iommus", sizeof(*refs),
		    (void **)&refs);
		if (n <= 0)
			continue;
		for (i = 0; i + 1 < n; i += 2)
			if (OF_node_from_xref(refs[i]) == dart)
				break;
		OF_prop_free(refs);
		if (i + 1 >= n)
			continue;
		device_set_desc(dev, mtp ? "Apple MTP DART error interrupt" :
		    "Apple USB DART error interrupt");
		return (BUS_PROBE_DEFAULT);
	}
	return (ENXIO);
}

static int
apple_dart_irq_attach(device_t dev)
{
	struct apple_dart_irq_softc *sc;
	u_int remaining, word;
	int error, rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	error = apple_pmgr_power_on(dev);
	if (error != 0)
		return (error);
	rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->mem == NULL)
		return (ENXIO);

	error = ENXIO;
	if (DART_PARAMS3_VERSION(bus_read_4(sc->mem, DART_PARAMS3)) != 0x0204)
		goto fail_mem;
	sc->nsids = DART_PARAMS4_NUM_SIDS(bus_read_4(sc->mem, DART_PARAMS4));
	if (sc->nsids == 0 || sc->nsids > DART_T8110_MAX_SID ||
	    rman_get_size(sc->mem) < DART_V2_SID_ADDR_HI(sc->nsids - 1) + 4)
		goto fail_mem;

	rid = 0;
	sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->irq == NULL)
		goto fail_mem;
	error = bus_setup_intr(dev, sc->irq, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, apple_dart_error_intr, sc, &sc->ih);
	if (error != 0) {
		bus_release_resource(dev, SYS_RES_IRQ, rid, sc->irq);
		goto fail_mem;
	}

	bus_write_4(sc->mem, DART_V2_DIAG_LOCK, 0);
	bus_write_4(sc->mem, DART_V2_ERROR_MASK, 0);
	for (word = 0; word < howmany(sc->nsids, 32); word++) {
		remaining = MIN(sc->nsids - 32 * word, 32);
		bus_write_4(sc->mem, DART_V24_SID_ENABLE(word),
		    UINT32_MAX >> (32 - remaining));
	}
	bus_write_4(sc->mem, DART_V24_ERROR_ENABLE, UINT32_MAX);
	if (bootverbose)
		device_printf(dev, "enabled fault interrupts for %u streams\n",
		    sc->nsids);
	return (0);

fail_mem:
	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem);
	return (error);
}

static int
apple_dart_irq_detach(device_t dev)
{
	struct apple_dart_irq_softc *sc = device_get_softc(dev);

	bus_write_4(sc->mem, DART_V24_ERROR_ENABLE, 0);
	bus_write_4(sc->mem, DART_V2_ERROR_MASK, UINT32_MAX);
	(void)bus_read_4(sc->mem, DART_V2_ERROR_MASK);
	bus_teardown_intr(dev, sc->irq, sc->ih);
	bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq);
	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem);
	return (0);
}

static device_method_t apple_dart_irq_methods[] = {
	DEVMETHOD(device_probe,		apple_dart_irq_probe),
	DEVMETHOD(device_attach,	apple_dart_irq_attach),
	DEVMETHOD(device_detach,	apple_dart_irq_detach),
	DEVMETHOD_END
};

static driver_t apple_dart_irq_driver = {
	"apple_dart_irq",
	apple_dart_irq_methods,
	sizeof(struct apple_dart_irq_softc),
};

DRIVER_MODULE(apple_dart_irq, simplebus, apple_dart_irq_driver, 0, 0);
