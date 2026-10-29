/*-
 * Copyright (c) 2023-2025 Bjoern A. Zeeb
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef	_LINUXKPI_NET_PAGE_POOL_HELPERS_H
#define	_LINUXKPI_NET_PAGE_POOL_HELPERS_H

#include <linux/kernel.h>	/* pr_debug */
#include <linux/types.h>
#include <linux/err.h>
#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <net/page_pool/types.h>

/*
 * A minimal page_pool.  It does not pool: every fragment is a page of
 * its own and pages go back to the VM when they are returned.  This
 * matches what the skb code does with pages it frees (__free_page() and
 * page_frag_free()), which knows nothing about pools.
 *
 * Because a freed skb never tells us about its pages, a page cannot
 * carry DMA mapping state we would have to undo.  We therefore only
 * accept mappings which are 1:1 with the physical address (no IOMMU, no
 * bounce buffers) and derive the DMA address from the page.  Anything
 * else fails the allocation rather than leaking mappings.
 */

static inline struct page_pool *
page_pool_create(const struct page_pool_params *ppparams)
{
	struct page_pool *ppool;

	/* Fragments are whole pages and freed as order-0 pages by skbs. */
	if (ppparams->order != 0)
		return (ERR_PTR(-EINVAL));
	if ((ppparams->flags & PP_FLAG_DMA_MAP) != 0 && ppparams->dev == NULL)
		return (ERR_PTR(-EINVAL));

	ppool = kzalloc(sizeof(*ppool), GFP_KERNEL);
	if (ppool == NULL)
		return (ERR_PTR(-ENOMEM));
	ppool->p = *ppparams;

	return (ppool);
}

static inline void
page_pool_destroy(struct page_pool *ppool)
{

	/* Pages hold no reference to the pool, so in-flight ones are fine. */
	kfree(ppool);
}

static inline struct page *
page_pool_alloc_frag(struct page_pool *ppool, uint32_t *offset,
    size_t size, gfp_t gfp)
{
	struct page *page;
	dma_addr_t dma;

	if (size > PAGE_SIZE)
		return (NULL);

	page = alloc_page(gfp);
	if (page == NULL)
		return (NULL);

	if ((ppool->p.flags & PP_FLAG_DMA_MAP) != 0) {
		/* Also syncs for the device, covering PP_FLAG_DMA_SYNC_DEV. */
		dma = dma_map_page(ppool->p.dev, page, 0, PAGE_SIZE,
		    ppool->p.dma_dir);
		if (dma_mapping_error(ppool->p.dev, dma)) {
			__free_page(page);
			return (NULL);
		}
		if (dma != page_to_phys(page)) {
			dma_unmap_page(ppool->p.dev, dma, PAGE_SIZE,
			    ppool->p.dma_dir);
			__free_page(page);
			pr_warn_once("%s: DMA address %#jx is not 1:1 with the "
			    "physical address; not supported\n", __func__,
			    (uintmax_t)dma);
			return (NULL);
		}
	}

	*offset = 0;
	return (page);
}

static inline struct page *
page_pool_dev_alloc_frag(struct page_pool *ppool, uint32_t *offset,
    size_t size)
{

	return (page_pool_alloc_frag(ppool, offset, size,
	    GFP_ATOMIC | __GFP_NOWARN));
}

static inline dma_addr_t
page_pool_get_dma_addr(struct page *page)
{

	/* page_pool_alloc_frag() only hands out 1:1 mapped pages. */
	return (page_to_phys(page));
}

static inline enum dma_data_direction
page_pool_get_dma_dir(const struct page_pool *ppool)
{

	return (ppool->p.dma_dir);
}

static inline void
page_pool_put_full_page(struct page_pool *ppool __unused, struct page *page,
    bool allow_direct __unused)
{

	/*
	 * Callers find the pool through pp_page_to_nmdesc(), which does not
	 * point at real state, so ppool must not be used here.  The page
	 * was 1:1 mapped, so there is no mapping to tear down.
	 */
	__free_page(page);
}

static inline int
page_pool_ethtool_stats_get_count(void)
{

	pr_debug("%s: TODO\n", __func__);
	return (0);
}

static inline uint8_t *
page_pool_ethtool_stats_get_strings(uint8_t *x)
{

	pr_debug("%s: TODO\n", __func__);
	return (x);
}

#endif	/* _LINUXKPI_NET_PAGE_POOL_HELPERS_H */
