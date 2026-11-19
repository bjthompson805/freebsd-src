/*-
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef	_ARM64_APPLE_APPLE_RTKIT_H_
#define	_ARM64_APPLE_APPLE_RTKIT_H_

struct apple_rtkit;

struct apple_rtkit_ops {
	/*
	 * Let the IOP DMA to a buffer it asked for (a SART entry, a DART
	 * mapping) and return the address to give it. Without this hook the
	 * IOP gets the physical address.
	 */
	int	(*buf_map)(void *arg, vm_paddr_t pa, size_t size,
		    uint64_t *iova);
	/* A message on an application endpoint (>= 0x20). */
	void	(*recv)(void *arg, uint32_t ep, uint64_t msg);
};

/* Send AP power ON before waiting for the IOP power ack (MTP). */
#define	APPLE_RTKIT_EARLY_AP	0x1

/*
 * No locking inside: the caller serializes all calls on one instance.
 * asc is the mapped ASC wrapper (CPU_CONTROL), mbox the mapped mailbox.
 */
struct apple_rtkit *apple_rtkit_create(device_t dev, vm_offset_t asc,
	    vm_offset_t mbox, const struct apple_rtkit_ops *ops, void *arg,
	    u_int flags);
int	apple_rtkit_boot(struct apple_rtkit *rtk);
void	apple_rtkit_poll(struct apple_rtkit *rtk);
int	apple_rtkit_send(struct apple_rtkit *rtk, uint32_t ep, uint64_t msg);
int	apple_rtkit_start_ep(struct apple_rtkit *rtk, uint32_t ep);
bool	apple_rtkit_crashed(struct apple_rtkit *rtk);
void	apple_rtkit_set_syslog(struct apple_rtkit *rtk, bool on);

#endif /* _ARM64_APPLE_APPLE_RTKIT_H_ */
