/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Read-only battery and AC adapter state from the Apple SMC, as sysctls
 * under the SMC device (dev.apple_smc.0.acline, dev.apple_smc.0.battery.*).
 * The keys and their meaning follow aurora-silicon's Linux macsmc-power for
 * SMC firmware 27.x (B0RM in native byte order). Every read asks the SMC;
 * nothing is written, so the firmware's charge policy stays as it is.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/sysctl.h>

#include <arm64/apple/apple_smc.h>

/* hw.acpi.battery.state bits, so the same tools can read either. */
#define	BATT_DISCHARGING	0x0001
#define	BATT_CHARGING		0x0002
#define	BATT_CRITICAL		0x0004

#define	BATT_CRITICAL_PERCENT	5

static int
smc_read_int(const char *key, size_t size, bool is_signed, int *val)
{
	uint8_t buf[4] = { 0 };
	size_t len;
	int error;

	error = apple_smc_read(APPLE_SMC_KEY(key), buf, size, &len);
	if (error != 0)
		return (error);
	if (len != size)
		return (EIO);
	switch (size) {
	case 1:
		*val = is_signed ? (int8_t)buf[0] : buf[0];
		break;
	case 2:
		*val = is_signed ? (int16_t)le16dec(buf) : le16dec(buf);
		break;
	case 4:
		*val = (int32_t)le32dec(buf);
		break;
	default:
		return (EINVAL);
	}
	return (0);
}

static int
smc_acline(int *val)
{
	int chis, error;

	if ((error = smc_read_int("CHIS", 4, false, &chis)) != 0)
		return (error);
	*val = chis != 0;
	return (0);
}

static int
smc_battery_state(int *val)
{
	int ac, current, full, life, error;

	if ((error = smc_acline(&ac)) != 0 ||
	    (error = smc_read_int("B0AC", 2, true, &current)) != 0 ||
	    (error = smc_read_int("BSFC", 1, false, &full)) != 0 ||
	    (error = smc_read_int("BUIC", 1, false, &life)) != 0)
		return (error);
	if (!ac || current < 0)
		*val = BATT_DISCHARGING;
	else if (full || current == 0)
		*val = 0;		/* on AC, not charging */
	else
		*val = BATT_CHARGING;
	if (life <= BATT_CRITICAL_PERCENT && (*val & BATT_CHARGING) == 0)
		*val |= BATT_CRITICAL;
	return (0);
}

/* arg2: key size (1, 2, 4), negative for signed; arg1: the key name. */
static int
smc_key_sysctl(SYSCTL_HANDLER_ARGS)
{
	int error, size, val;

	size = arg2 < 0 ? -arg2 : arg2;
	if ((error = smc_read_int(arg1, size, arg2 < 0, &val)) != 0)
		return (error);
	return (sysctl_handle_int(oidp, &val, 0, req));
}

/* Minutes to empty or to full; -1 when the SMC has no estimate. */
static int
smc_time_sysctl(SYSCTL_HANDLER_ARGS)
{
	int error, val;

	if ((error = smc_read_int(arg1, 2, false, &val)) != 0)
		return (error);
	if (val == 0xffff)
		val = -1;
	return (sysctl_handle_int(oidp, &val, 0, req));
}

static int
smc_acline_sysctl(SYSCTL_HANDLER_ARGS)
{
	int error, val;

	if ((error = smc_acline(&val)) != 0)
		return (error);
	return (sysctl_handle_int(oidp, &val, 0, req));
}

static int
smc_state_sysctl(SYSCTL_HANDLER_ARGS)
{
	int error, val;

	if ((error = smc_battery_state(&val)) != 0)
		return (error);
	return (sysctl_handle_int(oidp, &val, 0, req));
}

/*
 * Charge limit, as aurora's macsmc-power sets charge_control_end_threshold.
 * CHLS (16 bits): the limit in the low byte (0: none; the SMC resumes
 * charging 5% below it), CHLS_FORCE_DISCHARGE kept as it is, CHLS_HOLD
 * (a hold at the current charge, not ours) left alone. Older firmware has
 * only CHWA, a flag for a fixed 80% limit. 100 means no limit. The SMC
 * keeps the setting until it is changed (or power is lost).
 */
#define	CHLS_FORCE_DISCHARGE	0x100
#define	CHLS_HOLD		0x200
#define	CHLS_LIMIT_MASK		0xff
#define	CHLS_MIN_LIMIT		10
#define	CHWA_LIMIT		80

static enum { LIMIT_NONE, LIMIT_CHLS, LIMIT_CHWA } limit_method;

static int
smc_charge_limit_sysctl(SYSCTL_HANDLER_ARGS)
{
	uint8_t flag;
	uint16_t chls;
	int error, val;

	if (limit_method == LIMIT_CHWA) {
		if ((error = smc_read_int("CHWA", 1, false, &val)) != 0)
			return (error);
		val = val ? CHWA_LIMIT : 100;
	} else {
		if ((error = smc_read_int("CHLS", 2, false, &val)) != 0)
			return (error);
		chls = val;
		val = chls & CHLS_LIMIT_MASK;
		if (val == 0)
			val = 100;
	}
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val < 0 || val > 100)
		return (EINVAL);

	if (limit_method == LIMIT_CHWA) {
		flag = val < 100;
		return (apple_smc_write(APPLE_SMC_KEY("CHWA"), &flag, 1));
	}
	if (chls & CHLS_HOLD)
		return (EBUSY);
	chls &= CHLS_FORCE_DISCHARGE;
	if (val >= 100)
		chls = 0;	/* no limit: force-discharge ends with it */
	else
		chls |= MAX(val, CHLS_MIN_LIMIT);
	chls = htole16(chls);
	return (apple_smc_write(APPLE_SMC_KEY("CHLS"), &chls, 2));
}

#define	KEY_SYSCTL(name, key, size, desc)			\
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(batt), OID_AUTO, name,	\
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, __DECONST(char *, key), \
	    size, smc_key_sysctl, "I", desc)

void
apple_smc_battery_attach(device_t dev)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid *batt;
	int val;

	/* No battery (or no SMC): add nothing. */
	if (smc_read_int("BUIC", 1, false, &val) != 0) {
		device_printf(dev, "no battery\n");
		return;
	}
	device_printf(dev, "battery at %d%%\n", val);

	ctx = device_get_sysctl_ctx(dev);
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(device_get_sysctl_tree(dev)),
	    OID_AUTO, "acline", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE,
	    NULL, 0, smc_acline_sysctl, "I", "AC adapter online");
	batt = SYSCTL_ADD_NODE(ctx,
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "battery",
	    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "Battery");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(batt), OID_AUTO, "state",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, 0,
	    smc_state_sysctl, "I",
	    "State (hw.acpi.battery.state bits: 1 discharging, 2 charging, "
	    "4 critical)");
	KEY_SYSCTL("life", "BUIC", 1, "Charge (%)");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(batt), OID_AUTO, "time_empty",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, __DECONST(char *, "B0TE"),
	    0, smc_time_sysctl, "I", "Minutes to empty (-1: unknown)");
	SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(batt), OID_AUTO, "time_full",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, __DECONST(char *, "B0TF"),
	    0, smc_time_sysctl, "I", "Minutes to full (-1: unknown)");
	KEY_SYSCTL("power", "B0AP", -4, "Power (mW, negative discharging)");
	KEY_SYSCTL("voltage", "B0AV", 2, "Voltage (mV)");
	KEY_SYSCTL("current", "B0AC", -2, "Current (mA, negative discharging)");
	KEY_SYSCTL("remaining", "B0RM", 2, "Remaining capacity (mAh)");
	KEY_SYSCTL("full", "B0FC", 2, "Full charge capacity (mAh)");
	KEY_SYSCTL("design", "B0DC", 2, "Design capacity (mAh)");
	KEY_SYSCTL("cycles", "B0CT", 2, "Cycle count");

	if (smc_read_int("CHWA", 1, false, &val) == 0)
		limit_method = LIMIT_CHWA;
	else if (smc_read_int("CHLS", 2, false, &val) == 0)
		limit_method = LIMIT_CHLS;
	if (limit_method != LIMIT_NONE)
		SYSCTL_ADD_PROC(ctx, SYSCTL_CHILDREN(batt), OID_AUTO,
		    "charge_limit", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
		    NULL, 0, smc_charge_limit_sysctl, "I",
		    limit_method == LIMIT_CHWA ?
		    "Charge limit (%): 80 or 100 (none)" :
		    "Charge limit (%): 10-99, 100 for none");
	device_printf(dev, "charge limit: %s\n",
	    limit_method == LIMIT_CHLS ? "CHLS" :
	    limit_method == LIMIT_CHWA ? "CHWA (fixed 80%)" : "none");
}
