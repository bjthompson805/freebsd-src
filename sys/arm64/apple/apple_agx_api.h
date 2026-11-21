/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple M5 GPU: the interface apple_agx exports to its DRM driver (the
 * drm-kmod "agx" module). One client VM (execution context 5, the driver's
 * lower root) and one compute queue serve every client for now; submission
 * is synchronous, as Aurora's 25G76 M5 backend submits.
 */

#ifndef	_ARM64_APPLE_APPLE_AGX_API_H_
#define	_ARM64_APPLE_APPLE_AGX_API_H_

struct apple_agx_params {
	uint32_t	chip_id;		/* 0x8142 */
	uint32_t	generation;		/* 17 */
	uint32_t	variant;		/* 'G' */
	uint32_t	revision;
	uint32_t	num_dies;
	uint32_t	num_clusters;
	uint32_t	num_cores_per_cluster;
	uint32_t	core_masks[3];
	uint64_t	vm_start;		/* client VA the UAPI may bind */
	uint64_t	vm_end;
	uint64_t	usc_base;		/* the queue's USC window */
};

/* The attached GPU whose firmware runs and whose queue is installed. */
device_t apple_agx_device(void);
void	apple_agx_get_params(device_t dev, struct apple_agx_params *p);
/* Map pa..pa+size at va in the client VM; refuses driver-owned ranges. */
int	apple_agx_vm_map(device_t dev, uint64_t va, vm_paddr_t pa,
	    vm_size_t size, bool writable);
int	apple_agx_vm_unmap(device_t dev, uint64_t va, vm_size_t size);
/*
 * Run one compute command (its CDM stream at cdm_va..cdm_end, terminator
 * included) on the queue and wait for it to retire.
 */
int	apple_agx_submit_compute(device_t dev, uint64_t cdm_va,
	    uint64_t cdm_end, u_int timeout_ms);

#endif /* _ARM64_APPLE_APPLE_AGX_API_H_ */
