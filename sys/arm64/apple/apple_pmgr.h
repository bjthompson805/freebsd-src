/*-
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef	_ARM64_APPLE_APPLE_PMGR_H_
#define	_ARM64_APPLE_APPLE_PMGR_H_

int	apple_pmgr_power_on(device_t dev);

int	apple_pmgr_power_on_node(device_t dev, phandle_t node);
int	apple_pmgr_set_domain(device_t dev, int index, bool on);

#endif /* _ARM64_APPLE_APPLE_PMGR_H_ */
