/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple M5 GPU: the interface apple_agx exports to its DRM driver (the
 * drm-kmod "agx" module). Each client VM has a lower root and a tagged
 * execution context. The DRM driver leases an installed compute queue.
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
#define	APPLE_AGX_VM_READ	0x1
#define	APPLE_AGX_VM_WRITE	0x2
#define	APPLE_AGX_VM_SINGLE	0x4

bool	apple_agx_vm_valid(device_t dev, uint64_t va, vm_size_t size);
int	apple_agx_vm_create(device_t dev, u_int *context);
void	apple_agx_vm_destroy(device_t dev, u_int context);
u_int	apple_agx_queue_count(device_t dev);
int	apple_agx_queue_create(device_t dev, u_int *slot);
void	apple_agx_queue_destroy(device_t dev, u_int slot);

/* Map physical pages at va in the client VM. SINGLE repeats the first page. */
int	apple_agx_vm_map(device_t dev, u_int context, uint64_t va, vm_paddr_t pa,
	    vm_size_t size, u_int flags);
/* Map npages scattered pages at va, invalidating the TLBs once. No SINGLE. */
int	apple_agx_vm_map_list(device_t dev, u_int context, uint64_t va,
	    vm_page_t *pages, u_int npages, u_int flags);
int	apple_agx_vm_unmap(device_t dev, u_int context, uint64_t va, vm_size_t size);
/*
 * Run one compute command (its CDM stream at cdm_va..cdm_end, terminator
 * included) on the queue and wait for it to retire. Its dispatch programs
 * (SET_COMPUTE_RESOURCES; nprograms 0 for none) size its storage demand.
 */
struct agx_render_program;
int	apple_agx_submit_compute(device_t dev, u_int slot, u_int context,
	    uint64_t cdm_va, uint64_t cdm_end,
	    const struct agx_render_program *programs, u_int nprograms,
	    u_int timeout_ms);

/*
 * Run one render pass (struct agx_render_pass, its USC offsets relative to
 * the queue's USC window) on the render pair in the client VM and wait for
 * both stages to retire. Its linked programs (at least one) size the
 * stages' resource budgets.
 */
struct agx_render_pass;
int	apple_agx_submit_render(device_t dev, u_int context,
	    const struct agx_render_pass *pass,
	    const struct agx_render_program *programs, u_int nprograms,
	    u_int timeout_ms);

/*
 * As apple_agx_submit_render, but return once the pass is published:
 * done(arg, error) runs when it retires or fails, from a task thread with
 * no driver lock held. Only waits while every render slot is in use.
 */
int	apple_agx_submit_render_async(device_t dev, u_int context,
	    const struct agx_render_pass *pass,
	    const struct agx_render_program *programs, u_int nprograms,
	    void (*done)(void *, int), void *arg);

/*
 * Firmware dependencies between kicks: a word names one kick of one
 * firmware queue (its completion), so work on another queue can wait for
 * it on the GPU instead of the host. At most APPLE_AGX_MAX_DEPS a kick.
 */
#define	APPLE_AGX_MAX_DEPS	8

/*
 * As apple_agx_submit_render_async; the pass's tiling also waits for the
 * kicks in deps, and *dep (if not NULL) names the pass's completion.
 */
int	apple_agx_submit_render_deps(device_t dev, u_int context,
	    const struct agx_render_pass *pass,
	    const struct agx_render_program *programs, u_int nprograms,
	    const uint64_t *deps, u_int ndeps, uint64_t *dep,
	    void (*done)(void *, int), void *arg);

/*
 * Publish one compute command after the kicks in deps, without waiting
 * for it; *dep (if not NULL) names its completion. The queue keeps one
 * command in flight: the next kick, or apple_agx_compute_wait, retires
 * it. apple_agx_submit_compute is the kick and the wait.
 */
int	apple_agx_compute_kick(device_t dev, u_int slot, u_int context,
	    uint64_t cdm_va, uint64_t cdm_end,
	    const struct agx_render_program *programs, u_int nprograms,
	    const uint64_t *deps, u_int ndeps, uint64_t *dep);
int	apple_agx_compute_wait(device_t dev, u_int slot);

#endif /* _ARM64_APPLE_APPLE_AGX_API_H_ */
