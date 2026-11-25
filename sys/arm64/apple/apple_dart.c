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
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/rman.h>
#include <sys/sx.h>
#include <sys/time.h>
#include <sys/vmem.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/vm_page.h>
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

#include <arm64/include/bus_dma_impl.h>
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

#define	DART_PTE_NO_READ	(1ul << 3)
#define	DART_PTE_NO_WRITE	(1ul << 2)
#define	DART_PTE_NO_CACHE	(1ul << 1)
#define	DART_PTE_VALID		(1ul << 0)
#define	DART_PTE_SP_END		(0xffful << 40)
#define	DART_PTE_PA(pa)		(((pa) >> 4) & 0x3ffffffc00ul)

#define	DART_RD4(va, o)		(*(volatile uint32_t *)((va) + (o)))
#define	DART_WR4(va, o, v)	(*(volatile uint32_t *)((va) + (o)) = (v))

static MALLOC_DEFINE(M_APPLEDART, "apple_dart", "Apple DART domains");

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
static int
apple_dart_tlb_flush(device_t dev, vm_offset_t va, u_int sid)
{
	uint32_t cmd;
	int i;

	for (i = 0; i < 1000 &&
	    (DART_RD4(va, DART_T8110_TLB_CMD) & DART_T8110_TLB_CMD_BUSY); i++)
		DELAY(1);
	if (i == 1000) {
		device_printf(dev, "DART TLB command busy before sid %u flush\n",
		    sid);
		return (ETIMEDOUT);
	}
	cmd = DART_T8110_TLB_CMD_FLUSH_SID | sid;
	if (DART_PARAMS3_VERSION(DART_RD4(va, DART_PARAMS3)) >= 0x0202)
		cmd |= DART_T8110_TLB_CMD_V2;
	dsb(sy);
	DART_WR4(va, DART_T8110_TLB_CMD, cmd);
	for (i = 0; i < 1000 &&
	    (DART_RD4(va, DART_T8110_TLB_CMD) & DART_T8110_TLB_CMD_BUSY); i++)
		DELAY(1);
	if (i == 1000) {
		device_printf(dev, "DART TLB flush of sid %u timed out\n", sid);
		return (ETIMEDOUT);
	}
	return (0);
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
	error = apple_dart_tlb_flush(dev, va, sid);
	if (error != 0)
		goto out;
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

static int
apple_dart_flush_sid(device_t dev, phandle_t dart, u_int sid)
{
	bus_addr_t pa;
	bus_size_t size;
	vm_offset_t va;
	int error;

	if (sid >= DART_T8110_MAX_SID ||
	    ofw_reg_to_paddr(dart, 0, &pa, &size, NULL) != 0)
		return (ENXIO);
	va = (vm_offset_t)pmap_mapdev(pa, size);
	error = apple_dart_tlb_flush(dev, va, sid);
	pmap_unmapdev((void *)va, size);
	return (error);
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
	int error, i, n, stream_error;

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
	error = 0;
	for (i = 0; i + 1 < n; i += 2) {
		stream_error = apple_dart_flush_sid(dev,
		    OF_node_from_xref(cells[i]), cells[i + 1]);
		if (error == 0)
			error = stream_error;
	}
	sx_xunlock(&apple_dart_lock);
	if (n > 0)
		OF_prop_free(cells);
	return (error);
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
	int error;

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
	error = apple_dart_flush_sid(dev, dart, sid);
	sx_xunlock(&apple_dart_lock);
	return (error);
}

/*
 * Streams whose tables firmware (iBoot, a coprocessor) set up and owns, as
 * on a locked DART (aurora's locked-DART handling): apple_dart_fw_*() add
 * host buffers to the live tables and never replace or rewrite any entry.
 * A table of ours goes only into a vacant entry, at any level; a valid
 * entry is followed, never changed. Pages of ours go only into leaf tables
 * of ours, so IOVAs are allocated from the window "apple,dma-range" names a
 * leaf's worth (DART_FW_CHUNK) at a time, from leaves that are vacant or
 * already ours. Nothing is ever unmapped. The DART must be powered (for the
 * AOP DART: the AOP is running).
 *
 * Our tables are uncached and the firmware's are mapped as device memory,
 * so the DART sees each store once it completes. The tables of a stream are
 * under fw->mtx, not apple_dart_lock: the AOP maps from its poll thread
 * with a mutex of its own held, so nothing below attach may sleep.
 */
#define	DART_PTE_TO_PA(pte)	(((pte) & 0x3ffffffc00ul) << 4)
#define	DART_FW_CHUNK		(DART_PAGE_SIZE * DART_PTES)	/* one leaf */
#define	DART_FW_TABLES		32

struct apple_dart_fw {
	SLIST_ENTRY(apple_dart_fw) link;	/* under apple_dart_lock */
	struct mtx	mtx;
	device_t	dev;		/* the last attacher, for messages */
	phandle_t	dart;
	u_int		sid;
	vm_offset_t	regs;
	bus_size_t	regs_size;
	vm_paddr_t	root_pa;
	uint64_t	*root;
	int		levels;		/* 3: IOVA[41:36], [35:25], [24:14] */
	uint64_t	base, end;	/* apple,dma-range */
	uint64_t	next;		/* allocation cursor */
	int		ntables;
	struct {
		vm_paddr_t	pa;
		uint64_t	*va;
		bool		ours;
	} tables[DART_FW_TABLES];
};

static SLIST_HEAD(, apple_dart_fw) apple_dart_fws =
    SLIST_HEAD_INITIALIZER(apple_dart_fws);

/* The table at pa, mapped; *oursp: whether it is one of ours. */
static uint64_t *
apple_dart_fw_table(struct apple_dart_fw *fw, vm_paddr_t pa, bool *oursp)
{
	int i;

	for (i = 0; i < fw->ntables; i++)
		if (fw->tables[i].pa == pa) {
			if (oursp != NULL)
				*oursp = fw->tables[i].ours;
			return (fw->tables[i].va);
		}
	if (fw->ntables == DART_FW_TABLES)
		return (NULL);
	/* One of the firmware's: mapped as is, never cached by us. */
	fw->tables[fw->ntables].pa = pa;
	fw->tables[fw->ntables].va = pmap_mapdev(pa, DART_PAGE_SIZE);
	fw->tables[fw->ntables].ours = false;
	if (oursp != NULL)
		*oursp = false;
	return (fw->tables[fw->ntables++].va);
}

/*
 * A table of ours: uncached, below 2^42. The zeroes kmem left in the
 * cacheable direct map are written back and dropped first, so no dirty line
 * can land on the table later.
 */
static uint64_t *
apple_dart_fw_new_table(struct apple_dart_fw *fw, vm_paddr_t *pap)
{
	uint64_t *va;
	vm_paddr_t pa;

	if (fw->ntables == DART_FW_TABLES)
		return (NULL);
	va = kmem_alloc_contig(DART_PAGE_SIZE, M_NOWAIT | M_ZERO, 0,
	    (1ul << DART_IOVA_BITS) - 1, DART_PAGE_SIZE, 0,
	    VM_MEMATTR_UNCACHEABLE);
	if (va == NULL)
		return (NULL);
	pa = pmap_kextract((vm_offset_t)va);
	cpu_dcache_wbinv_range((void *)PHYS_TO_DMAP(pa), DART_PAGE_SIZE);
	memset(va, 0, DART_PAGE_SIZE);
	dsb(sy);
	fw->tables[fw->ntables].pa = pa;
	fw->tables[fw->ntables].va = va;
	fw->tables[fw->ntables].ours = true;
	fw->ntables++;
	*pap = pa;
	return (va);
}

static u_int
apple_dart_fw_shift(struct apple_dart_fw *fw, int level)
{

	/* The leaf is always indexed by IOVA[24:14]. */
	return (DART_PAGE_SHIFT + 11 * (fw->levels - 1 - level));
}

/*
 * The leaf table for iova, or NULL. With create, vacant entries on the way
 * get tables of ours; a valid entry is followed, never rewritten. *oursp:
 * whether the leaf is one of ours.
 */
static uint64_t *
apple_dart_fw_leaf(struct apple_dart_fw *fw, uint64_t iova, bool create,
    bool *oursp)
{
	uint64_t *t, *next, e;
	vm_paddr_t pa;
	u_int idx;
	int level;
	bool ours;

	ours = false;
	t = fw->root;
	for (level = 0; level < fw->levels - 1 && t != NULL; level++) {
		idx = (iova >> apple_dart_fw_shift(fw, level)) & 0x7ff;
		e = t[idx];
		if ((e & DART_PTE_VALID) == 0) {
			if (!create)
				return (NULL);
			next = apple_dart_fw_new_table(fw, &pa);
			if (next == NULL)
				return (NULL);
			t[idx] = DART_PTE_PA(pa) | DART_PTE_VALID;
			t = next;
			ours = true;
			continue;
		}
		t = apple_dart_fw_table(fw, DART_PTE_TO_PA(e), &ours);
	}
	if (oursp != NULL)
		*oursp = ours;
	return (t);
}

/* Whether pages of ours may go into the leaf covering iova: vacant or ours. */
static bool
apple_dart_fw_chunk_ok(struct apple_dart_fw *fw, uint64_t iova)
{
	uint64_t *t, e;
	int level;
	bool ours;

	ours = false;
	t = fw->root;
	for (level = 0; level < fw->levels - 1; level++) {
		e = t[(iova >> apple_dart_fw_shift(fw, level)) & 0x7ff];
		if ((e & DART_PTE_VALID) == 0)
			return (true);
		t = apple_dart_fw_table(fw, DART_PTE_TO_PA(e), &ours);
		if (t == NULL)
			return (false);
	}
	return (ours);
}

/*
 * Whether [iova, iova + size), inside one leaf, is all vacant and ours to use;
 * with vacant, also in a leaf of the firmware's.
 */
static bool
apple_dart_fw_free(struct apple_dart_fw *fw, uint64_t iova, size_t size,
    uint64_t *busyp, bool vacant)
{
	uint64_t *leaf, off;

	if (!vacant && !apple_dart_fw_chunk_ok(fw, iova)) {
		*busyp = rounddown2(iova, DART_FW_CHUNK) + DART_FW_CHUNK - 1;
		return (false);
	}
	leaf = apple_dart_fw_leaf(fw, iova, false, NULL);
	for (off = 0; leaf != NULL && off < size; off += DART_PAGE_SIZE)
		if (leaf[((iova + off) >> DART_PAGE_SHIFT) & 0x7ff] &
		    DART_PTE_VALID) {
			*busyp = iova + off;
			return (false);
		}
	return (true);
}

/* Map [pa, pa + size) at iova, all checked free, and flush the stream. */
static int
apple_dart_fw_enter(struct apple_dart_fw *fw, uint64_t iova, vm_paddr_t pa,
    size_t size)
{
	uint64_t *leaf, off;

	for (off = 0; off < size; off += DART_PAGE_SIZE) {
		leaf = apple_dart_fw_leaf(fw, iova + off, true, NULL);
		if (leaf == NULL)
			return (ENOMEM);
		leaf[((iova + off) >> DART_PAGE_SHIFT) & 0x7ff] =
		    DART_PTE_PA(pa + off) | DART_PTE_SP_END | DART_PTE_VALID;
	}
	return (apple_dart_tlb_flush(fw->dev, fw->regs, fw->sid));
}

int
apple_dart_fw_attach(device_t dev, phandle_t node, int index,
    struct apple_dart_fw **fwp)
{
	struct apple_dart_fw *fw;
	pcell_t *cells, range[4];
	phandle_t dart;
	bus_addr_t pa;
	bus_size_t size;
	vm_offset_t regs;
	vm_paddr_t root;
	uint64_t base, end;
	uint32_t tcr, ttbr;
	u_int sid;
	int error, i, levels, n;

	n = OF_getencprop_alloc_multi(node, "iommus", sizeof(*cells),
	    (void **)&cells);
	if (index < 0 || n < 2 * index + 2) {
		if (n > 0)
			OF_prop_free(cells);
		return (EINVAL);
	}
	dart = OF_node_from_xref(cells[2 * index]);
	sid = cells[2 * index + 1];
	OF_prop_free(cells);
	/* The client's window, or else the whole DART's (the ISP's). */
	if (OF_getencprop(node, "apple,dma-range", range, sizeof(range)) !=
	    sizeof(range) &&
	    OF_getencprop(dart, "apple,dma-range", range, sizeof(range)) !=
	    sizeof(range))
		return (EINVAL);
	base = roundup2((uint64_t)range[0] << 32 | range[1], DART_PAGE_SIZE);
	end = rounddown2(((uint64_t)range[0] << 32 | range[1]) +
	    ((uint64_t)range[2] << 32 | range[3]), DART_PAGE_SIZE);
	if (end <= base || end > (1ul << DART_IOVA_BITS) ||
	    sid >= DART_T8110_MAX_SID)
		return (EINVAL);
	if (!ofw_bus_node_is_compatible(dart, "apple,t8110-dart") ||
	    ofw_reg_to_paddr(dart, 0, &pa, &size, NULL) != 0)
		return (ENODEV);

	sx_xlock(&apple_dart_lock);
	SLIST_FOREACH(fw, &apple_dart_fws, link)
		if (fw->dart == dart && fw->sid == sid)
			break;
	regs = fw != NULL ? fw->regs : (vm_offset_t)pmap_mapdev(pa, size);
	tcr = DART_RD4(regs, DART_T8110_TCR(sid));
	ttbr = DART_RD4(regs, DART_T8110_TTBR(sid));
	device_printf(dev, "DART %#jx sid %u: TCR %#x TTBR %#x, window "
	    "%#jx-%#jx\n", (uintmax_t)pa, sid, tcr, ttbr, (uintmax_t)base,
	    (uintmax_t)end);
	error = 0;
	levels = (tcr & DART_T8110_TCR_FOUR_LEVEL) ? 3 : 2;
	root = (vm_paddr_t)(ttbr >> 2) << DART_PAGE_SHIFT;
	if ((tcr & DART_T8110_TCR_TRANSLATE) == 0 ||
	    (ttbr & DART_T8110_TTBR_VALID) == 0) {
		device_printf(dev, "DART sid %u has no live table\n", sid);
		error = ENXIO;
	} else if (end > (1ul << (DART_PAGE_SHIFT + 11 * levels))) {
		device_printf(dev, "DART sid %u: window beyond its %d-level "
		    "table\n", sid, levels);
		error = ERANGE;
	} else if (fw != NULL && (fw->base != base || fw->end != end)) {
		device_printf(dev, "DART sid %u: in use with window "
		    "%#jx-%#jx\n", sid, (uintmax_t)fw->base,
		    (uintmax_t)fw->end);
		error = EBUSY;
	}
	if (error != 0) {
		if (fw == NULL)
			pmap_unmapdev((void *)regs, size);
		sx_xunlock(&apple_dart_lock);
		return (error);
	}

	if (fw == NULL) {
		fw = malloc(sizeof(*fw), M_APPLEDART, M_WAITOK | M_ZERO);
		mtx_init(&fw->mtx, "apple_dart_fw", NULL, MTX_DEF);
		fw->dart = dart;
		fw->sid = sid;
		fw->regs = regs;
		fw->regs_size = size;
		fw->base = base;
		fw->end = end;
		SLIST_INSERT_HEAD(&apple_dart_fws, fw, link);
	}
	mtx_lock(&fw->mtx);
	fw->dev = dev;		/* an earlier attacher may be gone (kldunload) */
	if (fw->root != NULL && (fw->root_pa != root || fw->levels != levels)) {
		/* Firmware rebuilt its tables (a restart): ours went too. */
		device_printf(dev, "DART sid %u: root moved %#jx -> %#jx\n",
		    sid, (uintmax_t)fw->root_pa, (uintmax_t)root);
		for (i = 0; i < fw->ntables; i++)
			if (!fw->tables[i].ours)
				pmap_unmapdev(fw->tables[i].va, DART_PAGE_SIZE);
		/* Our old tables are leaked: the DART may have walked them. */
		fw->ntables = 0;
		fw->root = NULL;
	}
	if (fw->root == NULL) {
		fw->root_pa = root;
		fw->levels = levels;
		fw->next = fw->base;
		fw->root = apple_dart_fw_table(fw, root, NULL);
	}
	mtx_unlock(&fw->mtx);
	sx_xunlock(&apple_dart_lock);
	*fwp = fw;
	return (0);
}

/*
 * apple_dart_fw_attach(), and where the stream has no live table, first give
 * it an empty one of ours (uncached, as apple_dart_fw_new_table()'s), so the
 * fw interface manages it the same way. For a coprocessor whose boot loader
 * left its DART bare (the M5 ISP under an m1n1 without ISP17a support).
 */
int
apple_dart_fw_attach_own(device_t dev, phandle_t node, int index,
    struct apple_dart_fw **fwp)
{
	pcell_t *cells;
	phandle_t dart;
	uint64_t *root;
	vm_paddr_t pa;
	u_int sid;
	int error, n;

	error = apple_dart_fw_attach(dev, node, index, fwp);
	if (error != ENXIO)
		return (error);

	n = OF_getencprop_alloc_multi(node, "iommus", sizeof(*cells),
	    (void **)&cells);
	if (n < 2 * index + 2) {
		OF_prop_free(cells);
		return (EINVAL);
	}
	dart = OF_node_from_xref(cells[2 * index]);
	sid = cells[2 * index + 1];
	OF_prop_free(cells);

	root = kmem_alloc_contig(DART_PAGE_SIZE, M_WAITOK | M_ZERO, 0,
	    (1ul << DART_IOVA_BITS) - 1, DART_PAGE_SIZE, 0,
	    VM_MEMATTR_UNCACHEABLE);
	if (root == NULL)
		return (ENOMEM);
	pa = pmap_kextract((vm_offset_t)root);
	cpu_dcache_wbinv_range((void *)PHYS_TO_DMAP(pa), DART_PAGE_SIZE);
	memset(root, 0, DART_PAGE_SIZE);
	dsb(sy);
	sx_xlock(&apple_dart_lock);
	error = apple_dart_setup_table(dev, dart, sid, root);
	sx_xunlock(&apple_dart_lock);
	if (error != 0) {
		kmem_free(root, DART_PAGE_SIZE);
		return (error);
	}
	device_printf(dev, "DART sid %u: own table at %#jx\n", sid,
	    (uintmax_t)pa);
	/* The root is leaked on purpose: the DART walks it from now on. */
	return (apple_dart_fw_attach(dev, node, index, fwp));
}

int
apple_dart_fw_map(struct apple_dart_fw *fw, vm_paddr_t pa, size_t size,
    uint64_t *iovap)
{
	uint64_t iova, busy;
	int error;

	size = roundup2(size, DART_PAGE_SIZE);
	if ((pa & (DART_PAGE_SIZE - 1)) != 0 || size == 0 ||
	    size > DART_FW_CHUNK)
		return (EINVAL);
	mtx_lock(&fw->mtx);
	for (iova = fw->next; iova + size <= fw->end;) {
		/* Within one leaf. */
		if (rounddown2(iova, DART_FW_CHUNK) !=
		    rounddown2(iova + size - 1, DART_FW_CHUNK))
			iova = rounddown2(iova + size - 1, DART_FW_CHUNK);
		else if (apple_dart_fw_free(fw, iova, size, &busy, false))
			break;
		else
			iova = rounddown2(busy, DART_PAGE_SIZE) + DART_PAGE_SIZE;
	}
	if (iova + size > fw->end) {
		device_printf(fw->dev, "DART sid %u: no vacant %#zx of IOVA "
		    "in %#jx-%#jx\n", fw->sid, size, (uintmax_t)fw->base,
		    (uintmax_t)fw->end);
		mtx_unlock(&fw->mtx);
		return (ENOSPC);
	}
	error = apple_dart_fw_enter(fw, iova, pa, size);
	if (error != ENOMEM)
		fw->next = iova + size;
	mtx_unlock(&fw->mtx);
	if (error == 0)
		*iovap = iova;
	return (error);
}

static int
apple_dart_fw_map_at_flags(struct apple_dart_fw *fw, uint64_t iova,
    vm_paddr_t pa, size_t size, bool vacant)
{
	uint64_t off, len, busy;
	int error;

	size = roundup2(size, DART_PAGE_SIZE);
	if (((iova | pa) & (DART_PAGE_SIZE - 1)) != 0 || size == 0 ||
	    iova < fw->base || iova + size > fw->end || iova + size < iova)
		return (EINVAL);
	mtx_lock(&fw->mtx);
	error = 0;
	for (off = 0; off < size; off += len) {
		len = MIN(size - off, roundup2(iova + off + 1, DART_FW_CHUNK) -
		    (iova + off));
		if (!apple_dart_fw_free(fw, iova + off, len, &busy, vacant)) {
			device_printf(fw->dev, "DART sid %u: IOVA %#jx is "
			    "taken\n", fw->sid, (uintmax_t)busy);
			error = EBUSY;
			break;
		}
	}
	if (error == 0)
		error = apple_dart_fw_enter(fw, iova, pa, size);
	mtx_unlock(&fw->mtx);
	return (error);
}

int
apple_dart_fw_map_at(struct apple_dart_fw *fw, uint64_t iova, vm_paddr_t pa,
    size_t size)
{

	return (apple_dart_fw_map_at_flags(fw, iova, pa, size, false));
}

int
apple_dart_fw_map_vacant(struct apple_dart_fw *fw, uint64_t iova,
    vm_paddr_t pa, size_t size)
{

	return (apple_dart_fw_map_at_flags(fw, iova, pa, size, true));
}

int
apple_dart_fw_lookup(struct apple_dart_fw *fw, uint64_t iova, size_t size,
    vm_paddr_t *pap)
{
	uint64_t *leaf, pte, va, last;
	vm_paddr_t pa, first;
	int error;

	if (iova + size < iova)
		return (EINVAL);
	first = 0;
	error = 0;
	last = size != 0 ? iova + size - 1 : iova;
	mtx_lock(&fw->mtx);
	for (va = rounddown2(iova, DART_PAGE_SIZE); va <= last;
	    va += DART_PAGE_SIZE) {
		leaf = apple_dart_fw_leaf(fw, va, false, NULL);
		pte = leaf != NULL ? leaf[(va >> DART_PAGE_SHIFT) & 0x7ff] : 0;
		if ((pte & DART_PTE_VALID) == 0) {
			error = ENOENT;
			break;
		}
		pa = DART_PTE_TO_PA(pte);
		if (va == rounddown2(iova, DART_PAGE_SIZE))
			first = pa;
		else if (pa != first + (va - rounddown2(iova, DART_PAGE_SIZE))) {
			error = EFAULT;
			break;
		}
	}
	mtx_unlock(&fw->mtx);
	if (error == 0)
		*pap = first | (iova & (DART_PAGE_SIZE - 1));
	return (error);
}

int
apple_dart_fw_unmap(struct apple_dart_fw *fw, uint64_t iova, size_t size)
{
	uint64_t *leaf, off;
	int error;

	size = roundup2(size, DART_PAGE_SIZE);
	if ((iova & (DART_PAGE_SIZE - 1)) != 0 || size == 0 ||
	    iova < fw->base || iova + size > fw->end || iova + size < iova)
		return (EINVAL);
	error = 0;
	mtx_lock(&fw->mtx);
	/* The caller's own pages, which may be in a firmware leaf (map_vacant). */
	for (off = 0; off < size; off += DART_PAGE_SIZE) {
		leaf = apple_dart_fw_leaf(fw, iova + off, false, NULL);
		if (leaf == NULL) {
			error = ENOENT;
			continue;
		}
		leaf[((iova + off) >> DART_PAGE_SHIFT) & 0x7ff] = 0;
	}
	if (iova < fw->next)
		fw->next = iova;
	if (apple_dart_tlb_flush(fw->dev, fw->regs, fw->sid) != 0)
		error = EIO;
	mtx_unlock(&fw->mtx);
	return (error);
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
			return (apple_dart_flush_sid(dev, dart, sid));
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

struct apple_dart_stream {
	vm_offset_t va;
	bus_size_t size;
	u_int sid;
};

struct apple_dart_domain {
	device_t dev;
	struct mtx lock;
	uint64_t *root;
	vmem_t *iovas;
	struct apple_dart_stream streams[2];
	bool failed;
};

static uint64_t *
apple_dart_domain_table(int flags)
{
	vm_page_t m;
	vm_paddr_t pa;
	uint64_t *table;
	u_int i;

	/* One table is a whole DART page, whatever the kernel page size. */
	m = vm_page_alloc_noobj_contig(flags | VM_ALLOC_WIRED | VM_ALLOC_ZERO,
	    DART_PAGE_SIZE / PAGE_SIZE, 0, (1ul << DART_IOVA_BITS) - 1,
	    DART_PAGE_SIZE, 0, VM_MEMATTR_DEFAULT);
	if (m == NULL)
		return (NULL);
	for (i = 0; i < DART_PAGE_SIZE / PAGE_SIZE; i++)
		if ((m[i].flags & PG_ZERO) == 0)
			pmap_zero_page(&m[i]);
	pa = VM_PAGE_TO_PHYS(m);
	table = (uint64_t *)PHYS_TO_DMAP(pa);
	cpu_dcache_wb_range(table, DART_PAGE_SIZE);
	dsb(sy);
	return (table);
}

static uint64_t *
apple_dart_domain_leaf(struct apple_dart_domain *domain, vm_offset_t iova,
    bool alloc)
{
	uint64_t *table, *pte, *next;
	int shift;

	mtx_assert(&domain->lock, MA_OWNED);
	table = domain->root;
	for (shift = 36; shift > DART_PAGE_SHIFT; shift -= 11) {
		pte = &table[(iova >> shift) & (DART_PTES - 1)];
		if ((*pte & DART_PTE_VALID) == 0) {
			if (!alloc)
				return (NULL);
			next = apple_dart_domain_table(0);
			if (next == NULL)
				return (NULL);
			*pte = DART_PTE_PA(DMAP_TO_PHYS((vm_offset_t)next)) |
			    DART_PTE_VALID;
			cpu_dcache_wb_range(pte, sizeof(*pte));
		}
		table = (uint64_t *)PHYS_TO_DMAP((*pte & 0x3ffffffc00ul) << 4);
	}
	return (&table[(iova >> DART_PAGE_SHIFT) & (DART_PTES - 1)]);
}

static void
apple_dart_domain_free_table(uint64_t *table, int level)
{
	vm_page_t m;
	u_int i;

	if (level < 3) {
		for (i = 0; i < DART_PTES; i++) {
			if ((table[i] & DART_PTE_VALID) != 0)
				apple_dart_domain_free_table(
				    (uint64_t *)PHYS_TO_DMAP(
				    (table[i] & 0x3ffffffc00ul) << 4),
				    level + 1);
		}
	}
	m = PHYS_TO_VM_PAGE(DMAP_TO_PHYS((vm_offset_t)table));
	for (i = 0; i < DART_PAGE_SIZE / PAGE_SIZE; i++) {
		vm_page_unwire_noq(&m[i]);
		vm_page_free(&m[i]);
	}
}

static void
apple_dart_domain_disable(struct apple_dart_domain *domain)
{
	struct apple_dart_stream *stream;
	u_int i, tcr, ttbr;

	/* ENABLE is write-one-to-set on T8142. Block DMA through TCR. */
	for (i = 0; i < nitems(domain->streams); i++) {
		stream = &domain->streams[i];
		tcr = DART_T8110_TCR(stream->sid);
		ttbr = DART_T8110_TTBR(stream->sid);
		DART_WR4(stream->va, tcr, 0);
		DART_WR4(stream->va, ttbr, 0);
		if (DART_RD4(stream->va, tcr) != 0 ||
		    DART_RD4(stream->va, ttbr) != 0)
			panic("cannot disable DART stream %u", stream->sid);
	}
	dsb(sy);
}

static int
apple_dart_domain_flush(struct apple_dart_domain *domain)
{
	struct apple_dart_stream *stream;
	u_int i;
	int error;

	for (i = 0; i < nitems(domain->streams); i++) {
		stream = &domain->streams[i];
		error = apple_dart_tlb_flush(domain->dev, stream->va, stream->sid);
		if (error != 0) {
			apple_dart_domain_disable(domain);
			domain->failed = true;
			return (error);
		}
	}
	return (0);
}

/* The requester and both DARTs must be powered, with requester DMA stopped. */
int
apple_dart_domain_create(device_t dev, struct apple_dart_domain **result)
{
	struct apple_dart_domain *domain;
	struct apple_dart_stream *stream;
	phandle_t dart, node;
	pcell_t refs[4];
	bus_addr_t pa;
	uint32_t ttbr, tcr;
	u_int i, reg;
	int error;

	if (!ofw_bus_is_compatible(dev, "apple,t8142-dwc3"))
		return (ENXIO);
	node = ofw_bus_get_node(dev);
	if (OF_getproplen(node, "iommus") != sizeof(refs) ||
	    OF_getencprop(node, "iommus", refs, sizeof(refs)) != sizeof(refs) ||
	    refs[0] == refs[2])
		return (EINVAL);

	domain = malloc(sizeof(*domain), M_APPLEDART, M_WAITOK | M_ZERO);
	domain->dev = dev;
	domain->root = apple_dart_domain_table(VM_ALLOC_WAITOK);
	if (domain->root == NULL) {
		free(domain, M_APPLEDART);
		return (ENOMEM);
	}
	domain->iovas = vmem_create("USB DART IOVAs", DART_PAGE_SIZE,
	    (1ull << 32) - DART_PAGE_SIZE, DART_PAGE_SIZE, 0, M_WAITOK);
	mtx_init(&domain->lock, "Apple DART domain", NULL, MTX_DEF);
	error = ENXIO;
	for (i = 0; i < nitems(domain->streams); i++) {
		stream = &domain->streams[i];
		dart = OF_node_from_xref(refs[2 * i]);
		stream->sid = refs[2 * i + 1];
		if (!ofw_bus_node_status_okay(dart) ||
		    !ofw_bus_node_is_compatible(dart, "apple,t8142-dart") ||
		    stream->sid >= DART_T8110_MAX_SID ||
		    ofw_reg_to_paddr(dart, 0, &pa, &stream->size, NULL) != 0 ||
		    stream->size < DART_T8110_TTBR(stream->sid) + 4)
			goto fail;
		error = apple_pmgr_power_on_node(dev, dart);
		if (error != 0)
			goto fail;
		error = ENXIO;
		stream->va = (vm_offset_t)pmap_mapdev(pa, stream->size);
		if (DART_PARAMS3_VERSION(DART_RD4(stream->va, DART_PARAMS3)) != 0x0204 ||
		    DART_PARAMS3_VA_WIDTH(DART_RD4(stream->va, DART_PARAMS3)) <
		    DART_IOVA_BITS || stream->sid >= DART_PARAMS4_NUM_SIDS(
		    DART_RD4(stream->va, DART_PARAMS4)) ||
		    (DART_RD4(stream->va, DART_T8110_PROTECT) &
		    DART_T8110_PROTECT_TTBR_TCR) != 0)
			goto fail;
	}

	apple_dart_domain_disable(domain);
	ttbr = (DMAP_TO_PHYS((vm_offset_t)domain->root) >> DART_PAGE_SHIFT) << 2 |
	    DART_T8110_TTBR_VALID;
	for (i = 0; i < nitems(domain->streams); i++) {
		stream = &domain->streams[i];
		DART_WR4(stream->va, DART_T8110_TTBR(stream->sid), ttbr);
	}
	error = apple_dart_domain_flush(domain);
	if (error != 0)
		goto fail;
	tcr = DART_T8110_TCR_TRANSLATE | DART_T8110_TCR_FOUR_LEVEL;
	for (i = 0; i < nitems(domain->streams); i++) {
		stream = &domain->streams[i];
		reg = DART_T8110_ENABLE_STREAMS + 4 * (stream->sid / 32);
		DART_WR4(stream->va, DART_T8110_TCR(stream->sid), tcr);
		DART_WR4(stream->va, reg, 1u << (stream->sid % 32));
		if (DART_RD4(stream->va, DART_T8110_TCR(stream->sid)) != tcr ||
		    (DART_RD4(stream->va, reg) & (1u << (stream->sid % 32))) == 0) {
			apple_dart_domain_disable(domain);
			error = EIO;
			goto fail;
		}
	}
	*result = domain;
	return (0);

fail:
	for (i = 0; i < nitems(domain->streams); i++) {
		stream = &domain->streams[i];
		if (stream->va != 0)
			pmap_unmapdev((void *)stream->va, stream->size);
	}
	apple_dart_domain_free_table(domain->root, 1);
	vmem_destroy(domain->iovas);
	mtx_destroy(&domain->lock);
	free(domain, M_APPLEDART);
	return (error);
}

void
apple_dart_domain_destroy(struct apple_dart_domain *domain)
{
	u_int i;

	/* The requester must be stopped and its DMA maps unloaded. */
	apple_dart_domain_disable(domain);
	for (i = 0; i < nitems(domain->streams); i++)
		pmap_unmapdev((void *)domain->streams[i].va, domain->streams[i].size);
	apple_dart_domain_free_table(domain->root, 1);
	vmem_destroy(domain->iovas);
	mtx_destroy(&domain->lock);
	free(domain, M_APPLEDART);
}

static bool
apple_dart_domain_range_valid(vm_offset_t iova, bus_size_t size)
{
	const uint64_t limit = 1ull << DART_IOVA_BITS;

	return (size != 0 && ((iova | size) & (DART_PAGE_SIZE - 1)) == 0 &&
	    iova < limit && size <= limit - iova);
}

static int
apple_dart_domain_map(struct apple_dart_domain *domain, vm_offset_t iova,
    vm_paddr_t pa, bus_size_t size, vm_prot_t prot)
{
	bus_size_t offset;
	uint64_t *pte, bits;
	int error;

	if (!apple_dart_domain_range_valid(iova, size) ||
	    (prot & (VM_PROT_READ | VM_PROT_WRITE)) == 0)
		return (EINVAL);
	if ((DART_PTE_PA(pa) << 4) != pa || pa >= (1ull << 42) ||
	    size > (1ull << 42) - pa)
		return (ERANGE);
	bits = DART_PTE_SP_END | DART_PTE_NO_CACHE | DART_PTE_VALID;
	if ((prot & VM_PROT_READ) == 0)
		bits |= DART_PTE_NO_READ;
	if ((prot & VM_PROT_WRITE) == 0)
		bits |= DART_PTE_NO_WRITE;

	mtx_lock(&domain->lock);
	error = EIO;
	if (domain->failed)
		goto out;

	/* Reserve and check the whole range before publishing any leaf. */
	for (offset = 0; offset < size; offset += DART_PAGE_SIZE) {
		pte = apple_dart_domain_leaf(domain, iova + offset, true);
		error = ENOMEM;
		if (pte == NULL)
			goto out;
		error = EEXIST;
		if ((*pte & DART_PTE_VALID) != 0)
			goto out;
	}
	for (offset = 0; offset < size; offset += DART_PAGE_SIZE) {
		pte = apple_dart_domain_leaf(domain, iova + offset, false);
		*pte = DART_PTE_PA(pa + offset) | bits;
		cpu_dcache_wb_range(pte, sizeof(*pte));
	}
	error = apple_dart_domain_flush(domain);
	if (error != 0) {
		for (offset = 0; offset < size; offset += DART_PAGE_SIZE) {
			pte = apple_dart_domain_leaf(domain, iova + offset, false);
			*pte = 0;
			cpu_dcache_wb_range(pte, sizeof(*pte));
		}
	}
out:
	mtx_unlock(&domain->lock);
	return (error);
}

static int
apple_dart_domain_unmap(struct apple_dart_domain *domain, vm_offset_t iova,
    bus_size_t size)
{
	bus_size_t offset;
	uint64_t *pte;
	int error;

	if (!apple_dart_domain_range_valid(iova, size))
		return (EINVAL);
	mtx_lock(&domain->lock);
	for (offset = 0; offset < size; offset += DART_PAGE_SIZE) {
		pte = apple_dart_domain_leaf(domain, iova + offset, false);
		if (pte == NULL || (*pte & DART_PTE_VALID) == 0) {
			mtx_unlock(&domain->lock);
			return (ENOENT);
		}
	}
	for (offset = 0; offset < size; offset += DART_PAGE_SIZE) {
		pte = apple_dart_domain_leaf(domain, iova + offset, false);
		*pte = 0;
		cpu_dcache_wb_range(pte, sizeof(*pte));
	}
	error = domain->failed ? 0 : apple_dart_domain_flush(domain);
	mtx_unlock(&domain->lock);
	return (error);
}

static int
apple_dart_dma_map(void *cookie, const struct bus_dma_tag_common *tag,
    bus_dma_segment_t *seg)
{
	struct apple_dart_domain *domain = cookie;
	vmem_addr_t iova;
	bus_addr_t pa, offset;
	bus_size_t align, size;
	int error;

	pa = rounddown2(seg->ds_addr, DART_PAGE_SIZE);
	offset = seg->ds_addr - pa;
	size = roundup2(offset + seg->ds_len, DART_PAGE_SIZE);

	/* Keep the physical segment's alignment and boundary phase. */
	align = MAX(tag->alignment, tag->boundary);
	align = MAX(align, DART_PAGE_SIZE);
	if (tag->lowaddr < DART_PAGE_SIZE || size - 1 > tag->lowaddr)
		return (EFBIG);
	error = vmem_xalloc(domain->iovas, size, align, pa & (align - 1),
	    0, DART_PAGE_SIZE, tag->lowaddr, M_NOWAIT | M_FIRSTFIT, &iova);
	if (error != 0)
		return (error);
	error = apple_dart_domain_map(domain, iova, pa, size,
	    VM_PROT_READ | VM_PROT_WRITE);
	if (error != 0) {
		vmem_xfree(domain->iovas, iova, size);
		return (error);
	}
	seg->ds_addr = iova + offset;
	return (0);
}

static void
apple_dart_dma_unmap(void *cookie, const bus_dma_segment_t *seg)
{
	struct apple_dart_domain *domain = cookie;
	vmem_addr_t iova;
	bus_size_t size;
	int error;

	iova = rounddown2(seg->ds_addr, DART_PAGE_SIZE);
	size = roundup2(seg->ds_addr - iova + seg->ds_len, DART_PAGE_SIZE);
	error = apple_dart_domain_unmap(domain, iova, size);
	if (error != 0 && error != ETIMEDOUT)
		panic("invalid USB DART unmap: %d", error);
	vmem_xfree(domain->iovas, iova, size);
}

static const struct bus_dma_bounce_translation apple_dart_dma_ops = {
	.map = apple_dart_dma_map,
	.unmap = apple_dart_dma_unmap,
	.granule = DART_PAGE_SIZE,
};

int
apple_dart_domain_set_dma_tag(struct apple_dart_domain *domain,
    bus_dma_tag_t tag)
{

	return (bus_dma_bounce_set_translation(tag, &apple_dart_dma_ops, domain));
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
