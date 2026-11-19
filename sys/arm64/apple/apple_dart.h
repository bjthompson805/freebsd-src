/*-
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef	_ARM64_APPLE_APPLE_DART_H_
#define	_ARM64_APPLE_APPLE_DART_H_

int	apple_dart_identity(device_t dev);
int	apple_dart_map(device_t dev, uint64_t iova, vm_paddr_t pa,
	    size_t size);

#endif /* _ARM64_APPLE_APPLE_DART_H_ */
