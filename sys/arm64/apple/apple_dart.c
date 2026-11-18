/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Minimal Apple DART (T8110 layout) support for bring-up: point a device's
 * DART streams at one shared page table that maps every RAM page at
 * IOVA == PA, so its DMA works without a busdma/IOMMU backend. There is no
 * isolation. The USB DARTs on T8142 (DART 2.4) ignore the bypass bits, hence
 * the identity table instead.
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
#include <sys/sx.h>

#include <vm/vm.h>
#include <vm/vm_param.h>
#include <vm/vm_dumpset.h>
#include <vm/vm_extern.h>
#include <vm/pmap.h>

#include <machine/cpufunc.h>
#include <machine/md_var.h>
#include <machine/vmparam.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <arm64/apple/apple_dart.h>

#define	DART_PARAMS3			0x08
#define	DART_PARAMS3_VA_WIDTH(r)	(((r) >> 16) & 0x3f)

#define	DART_T8110_TLB_CMD		0x80
#define	DART_T8110_TLB_CMD_BUSY		(1u << 31)
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

static int
apple_dart_setup_one(device_t dev, phandle_t dart, u_int sid)
{
	bus_addr_t pa;
	bus_size_t size;
	vm_offset_t va;
	vm_paddr_t l1pa;
	uint32_t tcr;
	int i, error;

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

	l1pa = pmap_kextract((vm_offset_t)apple_dart_l1);
	DART_WR4(va, DART_T8110_TCR(sid), 0);
	DART_WR4(va, DART_T8110_TTBR(sid),
	    (l1pa >> DART_PAGE_SHIFT) << 2 | DART_T8110_TTBR_VALID);
	DART_WR4(va, DART_T8110_TLB_CMD, DART_T8110_TLB_CMD_FLUSH_SID | sid);
	for (i = 0; i < 1000 &&
	    (DART_RD4(va, DART_T8110_TLB_CMD) & DART_T8110_TLB_CMD_BUSY); i++)
		DELAY(1);
	DART_WR4(va, DART_T8110_TCR(sid),
	    DART_T8110_TCR_TRANSLATE | DART_T8110_TCR_FOUR_LEVEL);
	DART_WR4(va, DART_T8110_ENABLE_STREAMS + 4 * (sid / 32),
	    1u << (sid % 32));

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
