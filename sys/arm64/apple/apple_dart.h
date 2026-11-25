/*-
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef	_ARM64_APPLE_APPLE_DART_H_
#define	_ARM64_APPLE_APPLE_DART_H_

int	apple_dart_identity(device_t dev);
int	apple_dart_pci_stream(phandle_t host, u_int rid, phandle_t *dart,
	    u_int *sid);
int	apple_dart_lookup(uint64_t iova, size_t size, vm_paddr_t *pa);
int	apple_dart_map(device_t dev, uint64_t iova, vm_paddr_t pa,
	    size_t size);
vm_paddr_t apple_dart_translate(uint64_t iova);
int	apple_dart_pte_bits(device_t dev, phandle_t dart, u_int sid,
	    uint64_t iova, uint64_t bits);
int	apple_dart_private_stream(device_t dev, phandle_t dart, u_int sid);
int	apple_dart_identity_stream(device_t dev, phandle_t dart, u_int sid);
int	apple_dart_map_stream(device_t dev, phandle_t dart, u_int sid,
	    uint64_t iova, vm_paddr_t pa, size_t size);

/*
 * Streams whose page tables firmware set up and owns (locked DARTs, the
 * AOP's): buffers go into vacant entries of the live tables, inside the
 * device's "apple,dma-range" window (else the DART's). A valid entry is
 * never rewritten; only apple_dart_fw_unmap() takes entries away. No call
 * after attach sleeps (callers may hold a mutex); the DART must be powered.
 *
 * apple_dart_fw_attach: the stream of node's "iommus" entry #index, with
 *   node's window. Prints TCR/TTBR. ENXIO only when the stream has no live
 *   table (translation off or TTBR invalid): only then may a caller give it
 *   a table of its own (apple_dart_private_stream()). EINVAL: bad or
 *   missing properties; ENODEV: not a T8110 DART; ERANGE: window beyond
 *   the table; EBUSY: the stream is attached with another window.
 *   Attaching the same stream again returns the same handle, never freed.
 * apple_dart_fw_map: [pa, pa + size) at a fresh IOVA in the window, then
 *   flushes the stream. size <= 32 MiB (one leaf table); pa 16K-aligned.
 * apple_dart_fw_map_at: the same at a caller's IOVA (to give a buffer one
 *   address on several streams); EBUSY if any page of it is taken.
 * apple_dart_fw_lookup: where [iova, iova + size) is in memory, if mapped
 *   contiguously (ENOENT, EFAULT otherwise), for firmware-placed buffers.
 * apple_dart_fw_map_vacant: map_at that may also fill vacant entries of a
 *   firmware leaf (a coprocessor whose own memory shares the leaf, the ISP).
 * apple_dart_fw_unmap: drop the caller's pages at [iova, iova + size) (only
 *   what it mapped), then flush.
 */
struct apple_dart_fw;
int	apple_dart_fw_attach(device_t dev, phandle_t node, int index,
	    struct apple_dart_fw **fwp);
int	apple_dart_fw_attach_own(device_t dev, phandle_t node, int index,
	    struct apple_dart_fw **fwp);
int	apple_dart_fw_map(struct apple_dart_fw *fw, vm_paddr_t pa,
	    size_t size, uint64_t *iova);
int	apple_dart_fw_map_at(struct apple_dart_fw *fw, uint64_t iova,
	    vm_paddr_t pa, size_t size);
int	apple_dart_fw_map_vacant(struct apple_dart_fw *fw, uint64_t iova,
	    vm_paddr_t pa, size_t size);
int	apple_dart_fw_lookup(struct apple_dart_fw *fw, uint64_t iova,
	    size_t size, vm_paddr_t *pa);
int	apple_dart_fw_unmap(struct apple_dart_fw *fw, uint64_t iova, size_t size);

struct apple_dart_domain;
int	apple_dart_domain_create(device_t dev, struct apple_dart_domain **result);
void	apple_dart_domain_destroy(struct apple_dart_domain *domain);
int	apple_dart_domain_set_dma_tag(struct apple_dart_domain *domain,
	    bus_dma_tag_t tag);

#endif /* _ARM64_APPLE_APPLE_DART_H_ */
