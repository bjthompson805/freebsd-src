/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple M5 GPU: shared definitions of the apple_agx files.
 */

#ifndef	_ARM64_APPLE_APPLE_AGX_H_
#define	_ARM64_APPLE_APPLE_AGX_H_

#define	AGX_UAT_PGBIT		14
#define	AGX_UAT_PGSZ		(1ul << AGX_UAT_PGBIT)
#define	AGX_UAT_PGMSK		(AGX_UAT_PGSZ - 1)

/* Leaf protections (pgtable.rs prot). */
#define	AGX_PROT_FW_MMIO_RW		0
#define	AGX_PROT_FW_SHARED_RW		1
#define	AGX_PROT_FW_SHARED_RO		2
#define	AGX_PROT_FW_PRIV_RW		3
#define	AGX_PROT_GPU_FW_SHARED_RW	4
#define	AGX_PROT_GPU_SHARED_RW		5
#define	AGX_PROT_GPU_SHARED_RO		6
#define	AGX_PROT_GPU_SHARED_WO		7

/* Kernel window base the M5 graph lives in (initdata/m5.rs BASE). */
#define	AGX_M5_BASE		0xfffffc2000000000ul

#define	AGX_UAT_MAX_TABLES	512
#define	AGX_UAT_MAX_REGIONS	2

struct agx_uat {
	device_t	dev;
	uint64_t	ias_mask;
	uint64_t	oas_mask;
	vm_paddr_t	lower_root;	/* driver-owned, ttb0 */
	vm_paddr_t	upper_root;	/* iBoot "pagetables", ttb1 */
	vm_paddr_t	secondary_root;	/* GFX1's root, upper + 0x40000 */
	/* Bootloader-reserved table regions, mapped write-back. */
	struct {
		vm_paddr_t	pa;
		vm_size_t	size;
		uint8_t		*va;
	} regions[AGX_UAT_MAX_REGIONS];
	int		nregions;
	/* Tables this driver allocated and their original allocation addresses. */
	struct {
		vm_paddr_t	pa;
		void		*va;
	} tables[AGX_UAT_MAX_TABLES];
	int		ntables;
};

int	agx_uat_attach(struct agx_uat *uat, device_t dev, vm_paddr_t lower,
	    vm_paddr_t upper, vm_paddr_t secondary);
int	agx_uat_add_region(struct agx_uat *uat, vm_paddr_t pa, vm_size_t size);
int	agx_uat_new_client_root(struct agx_uat *uat, vm_paddr_t *root,
	    unsigned int shared_start, unsigned int shared_end);
int	agx_uat_alloc_tables(struct agx_uat *uat, vm_paddr_t root,
	    uint64_t start, uint64_t end);
int	agx_uat_map(struct agx_uat *uat, vm_paddr_t root, uint64_t iova,
	    vm_paddr_t pa, vm_size_t size, int prot);
int	agx_uat_map_pages(struct agx_uat *uat, vm_paddr_t root, uint64_t iova,
	    vm_paddr_t pa, vm_size_t size, int prot, bool single_page,
	    bool replace);
int	agx_uat_unmap(struct agx_uat *uat, vm_paddr_t root, uint64_t iova,
	    vm_size_t size);
int	agx_uat_lookup(struct agx_uat *uat, vm_paddr_t root, uint64_t iova,
	    uint64_t *pte);
int	agx_uat_adopt_roots(struct agx_uat *uat, uint64_t snapshot[3]);
int	agx_uat_confirm_roots(struct agx_uat *uat, const uint64_t snapshot[3]);
int	agx_uat_mirror_secondary(struct agx_uat *uat);
void	agx_uat_tlbi_all(void);

#endif /* _ARM64_APPLE_APPLE_AGX_H_ */
