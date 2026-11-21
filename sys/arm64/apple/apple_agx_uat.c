/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple M5 GPU: UAT page tables, after Aurora's drm/asahi pgtable.rs and the
 * PublishedRoots parts of mmu.rs (adopt/confirm_firmware_root_entries,
 * mirror_secondary_root, prepare_lower_tables).
 *
 * The UAT is an ARM64-style three-level table with 16K pages. The GPU
 * snoops the CPU caches on T8142 (UatQuirks::DEFAULT: no cleaning) and the
 * CPU's broadcast TLB invalidations.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/malloc.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/atomic.h>
#include <machine/cpufunc.h>
#include <machine/vmparam.h>

#include <arm64/apple/apple_agx.h>

MALLOC_DECLARE(M_AGX);

#define	PTE_TYPE_BITS		3ull
#define	PTE_TYPE_LEAF_TABLE	3ull
#define	UAT_NON_GLOBAL		(1ull << 11)
#define	UAT_AP_SHIFT		6
#define	UAT_HIGH_BITS_SHIFT	52
#define	UAT_MEMATTR_SHIFT	2
#define	UAT_AF			(1ull << 10)

#define	MEMATTR_CACHED		0
#define	MEMATTR_DEV		1
#define	MEMATTR_UNCACHED	2

#define	AP_FW_GPU		0
#define	AP_FW			1

#define	HIGH_BITS_PXN		(1u << 1)
#define	HIGH_BITS_UXN		(1u << 2)
#define	HIGH_BITS_GPU_ACCESS	(1u << 3)

#define	UAT_LVBIT		(AGX_UAT_PGBIT - 3)
#define	UAT_LVMSK		((1ull << UAT_LVBIT) - 1)
#define	UAT_LEVELS		3
#define	UAT_NPTE		(AGX_UAT_PGSZ / sizeof(uint64_t))

/* Prot::from_bits(ap, uxn, pxn).memattr(m).as_pte() */
#define	PROT(ap, uxn, pxn, m)						\
	((uint64_t)(ap) << UAT_AP_SHIFT |				\
	 (uint64_t)(HIGH_BITS_GPU_ACCESS | (pxn) * HIGH_BITS_PXN |	\
	     (uxn) * HIGH_BITS_UXN) << UAT_HIGH_BITS_SHIFT |		\
	 (uint64_t)(m) << UAT_MEMATTR_SHIFT | UAT_AF)

static const uint64_t agx_prot_bits[] = {
	[AGX_PROT_FW_MMIO_RW] = PROT(AP_FW, 1, 0, MEMATTR_DEV),
	[AGX_PROT_FW_SHARED_RW] = PROT(AP_FW, 1, 0, MEMATTR_UNCACHED),
	[AGX_PROT_FW_SHARED_RO] = PROT(AP_FW, 0, 0, MEMATTR_UNCACHED),
	[AGX_PROT_FW_PRIV_RW] = PROT(AP_FW, 1, 0, MEMATTR_CACHED),
	[AGX_PROT_GPU_FW_SHARED_RW] = PROT(AP_FW_GPU, 1, 1, MEMATTR_UNCACHED),
	/* PROT_GPU_RW (AP_GPU 2, uxn 1, pxn 0), stored as is. */
	[AGX_PROT_GPU_SHARED_RW] = PROT(2, 1, 0, MEMATTR_UNCACHED),
};

/* IOVA_KERN_TOP_LEVEL_INDEX: root entries 0..2 are the firmware's. */
#define	KERN_TOP_LEVEL_INDEX	2

void
agx_uat_tlbi_all(void)
{

	dsb(sy);
	__asm __volatile(".arch armv8.4-a\n tlbi vmalle1os" ::: "memory");
	dsb(sy);
	isb();
}

int
agx_uat_attach(struct agx_uat *uat, device_t dev, vm_paddr_t lower,
    vm_paddr_t upper, vm_paddr_t secondary)
{

	memset(uat, 0, sizeof(*uat));
	uat->dev = dev;
	uat->ias_mask = (1ull << 42) - 1;	/* t8142 uat_ias */
	uat->oas_mask = (1ull << 42) - 1;	/* t8142 uat_oas */
	uat->lower_root = lower;
	uat->upper_root = upper;
	uat->secondary_root = secondary;
	uat->tables[uat->ntables++] = lower;
	return (0);
}

int
agx_uat_add_region(struct agx_uat *uat, vm_paddr_t pa, vm_size_t size)
{

	if (uat->nregions == AGX_UAT_MAX_REGIONS)
		return (ENOSPC);
	uat->regions[uat->nregions].pa = pa;
	uat->regions[uat->nregions].size = size;
	uat->regions[uat->nregions].va = pmap_mapdev_attr(pa, size,
	    VM_MEMATTR_WRITE_BACK);
	uat->nregions++;
	return (0);
}

/*
 * A table page's CPU address: in a bootloader table region, or one this
 * driver allocated. Never anything else, whose words a walk would write.
 */
static volatile uint64_t *
agx_table(struct agx_uat *uat, vm_paddr_t pa)
{
	int i;

	for (i = 0; i < uat->nregions; i++)
		if (pa >= uat->regions[i].pa &&
		    pa - uat->regions[i].pa < uat->regions[i].size)
			return ((volatile uint64_t *)(uat->regions[i].va +
			    (pa - uat->regions[i].pa)));
	for (i = 0; i < uat->ntables; i++)
		if (uat->tables[i] == pa)
			return ((volatile uint64_t *)PHYS_TO_DMAP(pa));
	return (NULL);
}

static int
agx_new_table(struct agx_uat *uat, vm_paddr_t *pa)
{
	void *p;

	if (uat->ntables == AGX_UAT_MAX_TABLES)
		return (ENOSPC);
	p = contigmalloc(AGX_UAT_PGSZ, M_AGX, M_WAITOK | M_ZERO, 0,
	    uat->oas_mask, AGX_UAT_PGSZ, 0);
	if (p == NULL)
		return (ENOMEM);
	*pa = pmap_kextract((vm_offset_t)p);
	uat->tables[uat->ntables++] = *pa;
	return (0);
}

/*
 * walk_inner for one page: the leaf PTE of iova below root, allocating the
 * tables on the way when alloc is set. ENOENT: no table and !alloc.
 */
static int
agx_walk(struct agx_uat *uat, vm_paddr_t root, uint64_t iova, bool alloc,
    volatile uint64_t **leaf)
{
	volatile uint64_t *table, *pte;
	vm_paddr_t pa = root, child;
	uint64_t v;
	int level, error;

	iova &= uat->ias_mask;
	for (level = UAT_LEVELS - 2; level >= 0; level--) {
		if ((table = agx_table(uat, pa)) == NULL) {
			device_printf(uat->dev, "UAT: table %#lx at iova %#lx "
			    "is outside the reserved and driver tables\n", pa,
			    iova);
			return (EIO);
		}
		pte = &table[(iova >> (AGX_UAT_PGBIT + (level + 1) *
		    UAT_LVBIT)) & UAT_LVMSK];
		v = *pte;
		if (v == 0) {
			if (!alloc)
				return (ENOENT);
			if ((error = agx_new_table(uat, &child)) != 0)
				return (error);
			/* The child is zeroed before the parent names it. */
			dsb(sy);
			v = child | PTE_TYPE_LEAF_TABLE;
			*pte = v;
		}
		if ((v & PTE_TYPE_BITS) != PTE_TYPE_LEAF_TABLE) {
			device_printf(uat->dev, "UAT: unexpected table PTE %#lx "
			    "at iova %#lx level %d\n", v, iova, level + 1);
			return (EIO);
		}
		pa = v & uat->oas_mask & ~(uint64_t)AGX_UAT_PGMSK;
	}
	if ((table = agx_table(uat, pa)) == NULL) {
		device_printf(uat->dev, "UAT: leaf table %#lx at iova %#lx is "
		    "outside the reserved and driver tables\n", pa, iova);
		return (EIO);
	}
	*leaf = &table[(iova >> AGX_UAT_PGBIT) & UAT_LVMSK];
	return (0);
}

/* prepare_lower_tables / alloc_pages: tables only, no leaves. */
int
agx_uat_alloc_tables(struct agx_uat *uat, vm_paddr_t root, uint64_t start,
    uint64_t end)
{
	volatile uint64_t *leaf;
	uint64_t iova;
	int error;

	if (((start | end) & AGX_UAT_PGMSK) != 0 || end <= start)
		return (EINVAL);
	for (iova = start; iova < end; iova += AGX_UAT_PGSZ)
		if ((error = agx_walk(uat, root, iova, true, &leaf)) != 0)
			return (error);
	dsb(sy);
	return (0);
}

/*
 * map_pages: never replace a live leaf. Driver-owned roots hold user
 * (non-global) leaves, the firmware's kernel root global ones.
 */
int
agx_uat_map(struct agx_uat *uat, vm_paddr_t root, uint64_t iova,
    vm_paddr_t pa, vm_size_t size, int prot)
{
	volatile uint64_t *leaf;
	uint64_t bits, off;
	int error;

	if (((iova | pa | size) & AGX_UAT_PGMSK) != 0 || size == 0 ||
	    prot < 0 || prot >= (int)nitems(agx_prot_bits))
		return (EINVAL);
	bits = agx_prot_bits[prot] | PTE_TYPE_LEAF_TABLE;
	if (root == uat->lower_root)
		bits |= UAT_NON_GLOBAL;
	for (off = 0; off < size; off += AGX_UAT_PGSZ) {
		if ((error = agx_walk(uat, root, iova + off, true, &leaf)) != 0)
			return (error);
		if (*leaf != 0) {
			device_printf(uat->dev, "UAT: page at iova %#lx is "
			    "mapped (PTE %#lx)\n", iova + off, *leaf);
			return (EBUSY);
		}
		*leaf = (pa + off) | bits;
	}
	agx_uat_tlbi_all();
	return (0);
}

int
agx_uat_lookup(struct agx_uat *uat, vm_paddr_t root, uint64_t iova,
    uint64_t *pte)
{
	volatile uint64_t *leaf;
	int error;

	if ((error = agx_walk(uat, root, iova, false, &leaf)) != 0)
		return (error);
	*pte = *leaf;
	return (0);
}

/*
 * adopt_firmware_root_entries: snapshot the firmware's entries 0..2 of the
 * kernel root and clear the rest, which this driver now owns.
 */
int
agx_uat_adopt_roots(struct agx_uat *uat, uint64_t snapshot[3])
{
	volatile uint64_t *root;
	u_int i;

	if ((root = agx_table(uat, uat->upper_root)) == NULL)
		return (EIO);
	for (i = 0; i < 3; i++)
		snapshot[i] = root[i];
	for (i = KERN_TOP_LEVEL_INDEX + 1; i < UAT_NPTE; i++)
		root[i] = 0;
	agx_uat_tlbi_all();
	return (0);
}

int
agx_uat_confirm_roots(struct agx_uat *uat, const uint64_t snapshot[3])
{
	volatile uint64_t *root;
	u_int i;

	if ((root = agx_table(uat, uat->upper_root)) == NULL)
		return (EIO);
	for (i = 0; i < 3; i++)
		if (root[i] != snapshot[i])
			return (EIO);
	return (0);
}

/* mirror_secondary_root: the kernel window's top-level entries into GFX1's. */
int
agx_uat_mirror_secondary(struct agx_uat *uat)
{
	volatile uint64_t *primary, *secondary;
	uint64_t pte;
	u_int i;

	if ((primary = agx_table(uat, uat->upper_root)) == NULL ||
	    (secondary = agx_table(uat, uat->secondary_root)) == NULL)
		return (EIO);
	for (i = KERN_TOP_LEVEL_INDEX; i < UAT_NPTE; i++) {
		pte = primary[i];
		if (pte != 0 && secondary[i] != pte)
			secondary[i] = pte;
	}
	agx_uat_tlbi_all();
	return (0);
}
