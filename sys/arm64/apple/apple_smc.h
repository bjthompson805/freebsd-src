/*-
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef	_ARM64_APPLE_APPLE_SMC_H_
#define	_ARM64_APPLE_APPLE_SMC_H_

#define	APPLE_SMC_KEY(s)	((uint32_t)(s)[0] << 24 | (uint32_t)(s)[1] << 16 | \
				    (uint32_t)(s)[2] << 8 | (uint32_t)(s)[3])

int	apple_smc_read(uint32_t key, void *buf, size_t size, size_t *lenp);
int	apple_smc_write(uint32_t key, const void *buf, size_t size);
int	apple_smc_write_u32(uint32_t key, uint32_t value);
void	apple_smc_battery_attach(device_t smc);
void	apple_smc_temp_attach(device_t smc);
int	apple_pmu_update(uint16_t addr, uint8_t mask, uint8_t val);

#endif /* _ARM64_APPLE_APPLE_SMC_H_ */
