/*-
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef	_ARM64_APPLE_APPLE_DART_H_
#define	_ARM64_APPLE_APPLE_DART_H_

int	apple_dart_identity(device_t dev);
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

#endif /* _ARM64_APPLE_APPLE_DART_H_ */
