/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * SoC temperatures from the Apple SMC, as sysctls in deciKelvin (IK):
 * dev.apple_smc.0.temperature.{cpu,gpu} (the hottest die sensor of each)
 * and dev.cpu.0.temperature (the CPU reading), the name coretemp(4) uses.
 * The die keys are aurora-silicon's T8142 list (apple,soc-die-keys in
 * t8142-j815.dts); each is an IEEE 754 float in degrees Celsius. Keys the
 * SMC lacks are dropped at attach.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/sysctl.h>

#include <arm64/apple/apple_smc.h>

static const char *const cpu_keys[] = {
	"Te04", "Te08", "Te0C", "Te0R",
	"Tp00", "Tp04", "Tp0C", "Tp0G", "Tp0O", "Tp0R", "Tp0X", "Tp0a",
	"Tp0p", "Tp0u", "Tp0y", "Tp12", "Tp16", "Tp1E",
};

static const char *const gpu_keys[] = {
	"Tg0U", "Tg0X", "Tg0d", "Tg0g", "Tg0j", "Tg1Y", "Tg1c", "Tg1g",
};

struct smc_zone {
	uint32_t	keys[nitems(cpu_keys)];
	int		nkeys;
};

static struct smc_zone cpu_zone, gpu_zone;

/*
 * An SMC 'flt ' (little-endian binary32, degrees Celsius) as deciKelvin,
 * without the FPU. Readings outside (0, 200] C are refused.
 */
static int
smc_flt_to_dk(const uint8_t *buf, int *dk)
{
	uint32_t bits, mant;
	int exp;
	uint64_t tenths;

	bits = le32dec(buf);
	if (bits & 0x80000000)
		return (ERANGE);
	exp = (int)((bits >> 23) & 0xff) - 127;
	if (exp < -24 || exp > 7)
		return (ERANGE);
	mant = (bits & 0x7fffff) | 0x800000;
	/* value = mant * 2^(exp - 23); tenths rounded. */
	tenths = ((uint64_t)mant * 10 + (1ULL << (22 - exp))) >> (23 - exp);
	if (tenths > 2000)
		return (ERANGE);
	*dk = (int)tenths + 2732;
	return (0);
}

static int
smc_read_dk(uint32_t key, int *dk)
{
	uint8_t buf[4];
	size_t len;
	int error;

	if ((error = apple_smc_read(key, buf, sizeof(buf), &len)) != 0)
		return (error);
	if (len != sizeof(buf))
		return (EIO);
	return (smc_flt_to_dk(buf, dk));
}

/* The hottest sensor of the zone; EIO when none reads. */
static int
smc_zone_read(struct smc_zone *zone, int *dk)
{
	int i, max, val;

	max = -1;
	for (i = 0; i < zone->nkeys; i++)
		if (smc_read_dk(zone->keys[i], &val) == 0 && val > max)
			max = val;
	if (max < 0)
		return (EIO);
	*dk = max;
	return (0);
}

static int
smc_zone_sysctl(SYSCTL_HANDLER_ARGS)
{
	int error, val;

	if ((error = smc_zone_read(arg1, &val)) != 0)
		return (error);
	return (sysctl_handle_int(oidp, &val, 0, req));
}

static void
smc_zone_init(struct smc_zone *zone, const char *const *keys, int n)
{
	int i, val;

	for (i = 0; i < n; i++)
		if (smc_read_dk(APPLE_SMC_KEY(keys[i]), &val) == 0)
			zone->keys[zone->nkeys++] = APPLE_SMC_KEY(keys[i]);
}

void
apple_smc_temp_attach(device_t dev)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *temp;
	devclass_t dc;
	device_t cpu;

	smc_zone_init(&cpu_zone, cpu_keys, nitems(cpu_keys));
	smc_zone_init(&gpu_zone, gpu_keys, nitems(gpu_keys));
	device_printf(dev, "temperature: %d CPU and %d GPU die sensors\n",
	    cpu_zone.nkeys, gpu_zone.nkeys);
	if (cpu_zone.nkeys == 0 && gpu_zone.nkeys == 0)
		return;

	ctx = device_get_sysctl_ctx(dev);
	temp = SYSCTL_ADD_NODE(ctx,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "temperature", CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "Temperatures");
	if (cpu_zone.nkeys > 0)
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(temp), OID_AUTO, "cpu",
		    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, &cpu_zone, 0,
		    smc_zone_sysctl, "IK", "Hottest CPU die sensor");
	if (gpu_zone.nkeys > 0)
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(temp), OID_AUTO, "gpu",
		    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, &gpu_zone, 0,
		    smc_zone_sysctl, "IK", "Hottest GPU die sensor");

	/* dev.cpu.0.temperature, as coretemp(4) has it on x86. */
	if (cpu_zone.nkeys == 0 || (dc = devclass_find("cpu")) == NULL ||
	    (cpu = devclass_get_device(dc, 0)) == NULL)
		return;
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(cpu),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(cpu)), OID_AUTO,
	    "temperature", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    &cpu_zone, 0, smc_zone_sysctl, "IK",
	    "Hottest CPU die sensor (SMC)");
}
