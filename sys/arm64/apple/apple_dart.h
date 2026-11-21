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

struct vm_page;
struct apple_dart_domain;
int	apple_dart_domain_create(device_t dev, struct apple_dart_domain **result);
void	apple_dart_domain_destroy(struct apple_dart_domain *domain);
int	apple_dart_domain_map(struct apple_dart_domain *domain, vm_offset_t iova,
	    struct vm_page **pages, bus_size_t size, vm_prot_t prot);
int	apple_dart_domain_unmap(struct apple_dart_domain *domain, vm_offset_t iova,
	    bus_size_t size);

#endif /* _ARM64_APPLE_APPLE_DART_H_ */
