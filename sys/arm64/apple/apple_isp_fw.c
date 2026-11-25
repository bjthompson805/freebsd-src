// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */
/*
 * Apple ISP firmware boot and stop: aurora-silicon's Linux isp-fw.c, the
 * T8140/T8142 (ISP17a, H17, fw_restart) path.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <arm64/apple/apple_isp.h>
#include <arm64/apple/apple_isp_cmd.h>
#include <arm64/apple/apple_dart.h>
#include <arm64/apple/apple_pmgr.h>

#define	ISP_FIRMWARE_POLL_US	1000
#define	ISP_FIRMWARE_TIMEOUT_US	1000000

#define	ISP_FIRMWARE_IPC_SIZE	0x1c000
#define	ISP_FIRMWARE_DATA_SIZE	0x28000

/* the waits for firmware that has been told to power down */
#define	ISP_STOP_KEY_TRIES	20
#define	ISP_STOP_KEY_MS		10
#define	ISP_STOP_WFI_POLL_US	(10 * 1000)
#define	ISP_STOP_WFI_US		(190 * 1000)

static inline u32
isp_coproc_read32(struct apple_isp *isp, u32 reg)
{
	return (bus_read_4(isp->coproc, reg));
}

static inline void
isp_coproc_write32(struct apple_isp *isp, u32 reg, u32 val)
{
	bus_write_4(isp->coproc, reg, val);
}

static inline u32
isp_gpio_read32(struct apple_isp *isp, u32 reg)
{
	return (bus_read_4(isp->gpio, reg));
}

static inline void
isp_gpio_write32(struct apple_isp *isp, u32 reg, u32 val)
{
	bus_write_4(isp->gpio, reg, val);
}

/* readl_poll_timeout(): sleeps between reads, as Linux's does. */
static int
isp_poll32(struct resource *res, u32 reg, u32 mask, u32 want, int poll_us,
    int timeout_us, u32 *valp)
{
	u32 val;
	int waited;

	for (waited = 0;; waited += poll_us) {
		val = bus_read_4(res, reg);
		if ((val & mask) == want)
			break;
		if (waited >= timeout_us) {
			if (valp)
				*valp = val;
			return (-ETIMEDOUT);
		}
		pause_sbt("isppoll", ustosbt(poll_us), 0, C_PREL(1));
	}
	if (valp)
		*valp = val;
	return (0);
}

/* Wait for the firmware to write @expected to a GPIO word. */
static int
isp_gpio_wait(struct apple_isp *isp, u32 reg, u32 expected)
{
	return (isp_poll32(isp->gpio, reg, ~0u, expected, ISP_FIRMWARE_POLL_US,
	    ISP_FIRMWARE_TIMEOUT_US, NULL));
}

/* The CPU, FE, ... domains (1 on): ISP_SYS (0) stays on, see attach. */
static int
apple_isp_power_up_domains(struct apple_isp *isp)
{
	int i, ret;

	if (isp->pds_active)
		return (0);

	for (i = 1; i < isp->pd_count; i++) {
		ret = apple_pmgr_set_domain(isp->dev, i, true);
		if (ret != 0) {
			dev_err(isp->dev,
			    "Failed to power up power domain %d: %d\n", i, ret);
			while (--i >= 1)
				apple_pmgr_set_domain(isp->dev, i, false);
			return (-ret);
		}
	}
	isp->pds_active = true;
	return (0);
}

static void
apple_isp_power_down_domains(struct apple_isp *isp)
{
	int i, ret;

	if (!isp->pds_active)
		return;

	for (i = isp->pd_count - 1; i >= 1; i--) {
		ret = apple_pmgr_set_domain(isp->dev, i, false);
		if (ret != 0)
			dev_err(isp->dev,
			    "Failed to power down power domain %d: %d\n", i, ret);
	}
	isp->pds_active = false;
}

/*
 * Boot descriptor of the H17 firmware. The values the driver fills in are
 * the ones this firmware boots with; how they derive from the rest of the
 * setup is not known.
 */
#define	ISP_H16_DESCRIPTOR_SIZE		0x200
#define	ISP_H16_SHARED_SIZE		0xee000000ULL
#define	ISP_H16_DESC_CAM_SCHEME		0x60
#define	ISP_H16_DESC_NO_OPTICAL_CARD_ID	0xb0

struct isp_firmware_bootargs_h16 {
	u32 pad_0[2];
	u64 ipc_iova;
	u64 shared_base;
	u64 shared_size;
	u64 extra_iova;
	u64 extra_size;
	u32 platform_id;
	u32 pad_34;
	u64 logbuf_addr;
	u64 logbuf_size;
	u64 logbuf_entsize;
	u64 ipc_queue_size;
	u32 pad_58[4];
	u32 unk_68;
	u32 pad_6c;
	u8 descriptor[ISP_H16_DESCRIPTOR_SIZE];
	u32 pad_270[8];
} __packed;
static_assert(sizeof(struct isp_firmware_bootargs_h16) == 0x290);

/* set in ISP_GPIO_2 by firmware that takes the H16 boot descriptor */
#define	ISP_GPIO2_H16_DESCRIPTOR	BIT(1)

struct isp_chan_desc {
	char name[64];
	u32 type;
	u32 src;
	u32 num;
	u32 pad;
	u64 iova;
	u32 padding[0x2a];
} __packed;
static_assert(sizeof(struct isp_chan_desc) == 0x100);

static const struct isp_chan_ops tm_ops = {
	.handle = ipc_tm_handle,
};

static const struct isp_chan_ops sm_ops = {
	.handle = ipc_sm_handle,
};

static const struct isp_chan_ops bt_ops = {
	.handle = ipc_bt_handle,
	.complete = ipc_bt_complete,
};

/* Linux's apple_isp_isr_thread(), on the taskqueue apple_isp_intr() wakes. */
void
apple_isp_irq_task(void *arg, int pending)
{
	struct apple_isp *isp = arg;
	bool more;

	if (!isp->irq_on)
		return;
	do {
		isp_wake(isp);

		more = ipc_chan_handle(isp, isp->chan_sm) == -EAGAIN;
		isp_wake(isp); /* Some commands depend on sm */

		more |= ipc_chan_handle(isp, isp->chan_tm) == -EAGAIN;
		more |= ipc_chan_handle(isp, isp->chan_bt) == -EAGAIN;
		isp_wake(isp);

		/* Fair turns across channels, without depending on another IRQ. */
		maybe_yield();
	} while (more && !READ_ONCE(isp->ipc_error) &&
	    !READ_ONCE(isp->ipc_quiescing));
}

static void
isp_disable_irq(struct apple_isp *isp)
{
	isp_mbox_write32(isp, isp->hw->mbox_irq_enable, 0x0);
	isp_mbox_write32(isp, ISP_MBOX_IRQ_ENABLE1_T8140, 0x0);
	isp_mbox_write32(isp, ISP_MBOX_IRQ_ENABLE2_T8140, 0x0);
	isp_mbox_write32(isp, ISP_MBOX_IRQ_ROUTE_T8140, 0x0);
	isp->irq_on = false;
	taskqueue_drain(isp->tq, &isp->irq_task);
	isp_gpio_write32(isp, ISP_GPIO_1, 0xfeedbabe); /* real funny */
	DELAY(10);
}

static int
isp_enable_irq(struct apple_isp *isp)
{
	isp_dbg(isp, "about to enable interrupts...\n");
	isp->irq_on = true;
	isp_mbox_write32(isp, isp->hw->mbox_irq_enable, 0xf);
	isp_mbox_write32(isp, ISP_MBOX_IRQ_ENABLE1_T8140,
	    ISP_MBOX_IRQ_ENABLE1_T8140_VAL);
	isp_mbox_write32(isp, ISP_MBOX_IRQ_ENABLE2_T8140,
	    ISP_MBOX_IRQ_ENABLE2_T8140_VAL);
	isp_mbox_write32(isp, ISP_MBOX_IRQ_ROUTE_T8140,
	    ISP_MBOX_IRQ_ROUTE_T8140_VAL);
	return (0);
}

/*
 * The ISP17a reset as observed when the firmware boots on T8140. Only the
 * reset request bit is known to signal completion. The acknowledge words
 * change between the eight reads, but no condition on them is known, and
 * the coprocessor status is not polled for WFI on this generation. Which
 * of the writes, reads and waits are required has not been established.
 */
static int
isp_reset_coproc_t8140(struct apple_isp *isp)
{
	isp_coproc_write32(isp, ISP_COPROC_EDPRCR, 0x2);
	if (isp_poll32(isp->coproc, ISP_COPROC_EDPRCR, 0x2, 0,
	    ISP_FIRMWARE_POLL_US, 100 * 1000, NULL)) {
		isp_err(isp, "coprocessor reset request did not clear\n");
		return (-ETIMEDOUT);
	}
	pause("ispreset", MAX(1, hz / 20));

	isp_coproc_write32(isp, ISP_COPROC_FABRIC_0_T8140, 0xffffffff);
	isp_coproc_write32(isp, ISP_COPROC_FABRIC_1_T8140, 0xffffffff);
	isp_coproc_write32(isp, ISP_COPROC_FABRIC_2_T8140, 0xffffffff);
	isp_coproc_write32(isp, ISP_COPROC_FABRIC_3_T8140, 0xffffffff);

	isp_coproc_write32(isp, ISP_COPROC_IRQ_MASK_0_T8140, 0xffffffff);
	isp_coproc_write32(isp, ISP_COPROC_IRQ_MASK_1_T8140, 0xffffffff);
	isp_coproc_write32(isp, ISP_COPROC_IRQ_MASK_2_T8140, 0xffffffff);
	isp_coproc_write32(isp, ISP_COPROC_IRQ_MASK_3_T8140, 0xffffffff);

	for (int i = 0; i < 8; i++) {
		isp_coproc_read32(isp, ISP_COPROC_RESET_ACK_0_T8140);
		isp_coproc_read32(isp, ISP_COPROC_RESET_ACK_1_T8140);
	}
	pause("ispreset", MAX(1, hz / 20));

	return (0);
}

/*
 * The coprocessor starts at its reset vector, which the bootloader sets and
 * which bit 0 latches. Note it at the first start; where a power-off cleared
 * it, set it again.
 */
static int
isp_coproc_entry(struct apple_isp *isp)
{
	struct isp_firmware *fw = &isp->fw;
	u64 rvbar = bus_read_8(isp->coproc, ISP_COPROC_RVBAR);

	if (!fw->started) {
		fw->entry = rvbar & ~1ULL;
		device_printf(isp->dev, "firmware reset vector %#jx\n",
		    (uintmax_t)rvbar);
		return (0);
	}

	if (!fw->entry || rvbar == (fw->entry | 1))
		return (0);

	if (rvbar & 1) {
		isp_err(isp, "reset vector %#jx latched, not the firmware's "
		    "%#jx\n", (uintmax_t)rvbar, (uintmax_t)fw->entry);
		return (-EIO);
	}

	bus_write_8(isp->coproc, ISP_COPROC_RVBAR, fw->entry | 1);
	dev_dbg(isp->dev, "reset vector was %#jx, set to %#jx\n",
	    (uintmax_t)rvbar,
	    (uintmax_t)bus_read_8(isp->coproc, ISP_COPROC_RVBAR));

	return ((bus_read_8(isp->coproc, ISP_COPROC_RVBAR) & ~1ULL) ==
	    fw->entry ? 0 : -EIO);
}

static void
isp_firmware_shutdown_stage1(struct apple_isp *isp)
{
	/* Only a coprocessor that was started has to settle and quiesce */
	bool started = isp->hw->coproc_quiesce &&
	    isp_coproc_read32(isp, isp->hw->coproc_control);
	u32 val;

	/* A coprocessor that powered down waits in WFI */
	if (started && isp_poll32(isp->coproc, isp->hw->coproc_status,
	    ISP_COPROC_IN_WFI, ISP_COPROC_IN_WFI, 100, 200 * 1000, &val))
		dev_warn(isp->dev,
		    "coproc not in WFI before power-down (status 0x%x)\n", val);

	/*
	 * macOS powers T8140's ISP off from WFI with the run bit still set: no
	 * write of the control word here.
	 */
	if (started) {
		/* The request bit alone asks for it, and reads back once done */
		isp_coproc_write32(isp, isp->hw->coproc_quiesce,
		    ISP_COPROC_QUIESCED);
		if (isp_poll32(isp->coproc, isp->hw->coproc_quiesce,
		    ISP_COPROC_QUIESCED, ISP_COPROC_QUIESCED, 10, 50 * 1000,
		    &val))
			dev_warn(isp->dev, "coproc did not quiesce (0x%x)\n",
			    val);
	}

	apple_isp_power_down_domains(isp);
}

static int
isp_firmware_boot_stage1(struct apple_isp *isp)
{
	int err;

	err = apple_isp_power_up_domains(isp);
	if (err < 0)
		return (err);

	err = apple_isp_program_dapf(isp);
	if (err)
		goto shutdown;

	err = isp_reset_coproc_t8140(isp);
	if (err < 0)
		goto shutdown;

	err = isp_coproc_entry(isp);
	if (err < 0)
		goto shutdown;

	isp_gpio_write32(isp, ISP_GPIO_0, 0x0);
	isp_gpio_write32(isp, ISP_GPIO_1, 0x0);
	isp_gpio_write32(isp, ISP_GPIO_2, 0x0);
	isp_gpio_write32(isp, ISP_GPIO_3, 0x0);
	isp_gpio_write32(isp, ISP_GPIO_4, 0x0);
	isp_gpio_write32(isp, ISP_GPIO_5, 0x0);
	isp_gpio_write32(isp, ISP_GPIO_6, isp->hw->boot_mode);
	isp_gpio_write32(isp, ISP_GPIO_7, 0x0);

	isp_mbox_write32(isp, isp->hw->mbox_irq_enable, 0x0);

	apple_isp_dart_report(isp, true);
	isp_coproc_write32(isp, isp->hw->coproc_control, 0x0);
	isp_coproc_write32(isp, isp->hw->coproc_control, 0x10);
	isp->fw.started = true;

	/* Wait for ISP_GPIO_7 to 0x0 -> 0x8042006 */
	if (isp_gpio_wait(isp, ISP_GPIO_7, 0x8042006)) {
		isp_err(isp,
		    "never received first magic number from firmware (0x%x)\n",
		    isp_gpio_read32(isp, ISP_GPIO_7));
		apple_isp_dart_report(isp, false);
		device_printf(isp->dev, "coproc control %#x status %#x\n",
		    isp_coproc_read32(isp, isp->hw->coproc_control),
		    isp_coproc_read32(isp, isp->hw->coproc_status));
		err = -ENODEV;
		goto shutdown;
	}
	isp_dbg(isp, "got first magic number from firmware\n");

	return (0);

shutdown:
	isp_firmware_shutdown_stage1(isp);
	return (err);
}

int
apple_isp_alloc_firmware_surface(struct apple_isp *isp)
{
	/* These are static, so let's do it once and for all */
	isp->ipc_surf = isp_alloc_surface_vmap(isp, ISP_FIRMWARE_IPC_SIZE);
	if (!isp->ipc_surf) {
		isp_err(isp, "failed to alloc shared surface for ipc\n");
		return (-ENOMEM);
	}
	dev_dbg(isp->dev, "IPC surface iova: 0x%jx\n",
	    (uintmax_t)isp->ipc_surf->iova);

	isp->data_surf = isp_alloc_surface_vmap(isp, ISP_FIRMWARE_DATA_SIZE);
	if (!isp->data_surf) {
		isp_err(isp, "failed to alloc shared surface for data files\n");
		isp_free_surface(isp, isp->ipc_surf);
		isp->ipc_surf = NULL;
		return (-ENOMEM);
	}
	dev_dbg(isp->dev, "Data surface iova: 0x%jx\n",
	    (uintmax_t)isp->data_surf->iova);

	return (0);
}

void
apple_isp_free_firmware_surface(struct apple_isp *isp)
{
	isp_free_surface(isp, isp->data_surf);
	isp_free_surface(isp, isp->ipc_surf);
	isp->data_surf = NULL;
	isp->ipc_surf = NULL;
}

static void
isp_firmware_shutdown_stage2(struct apple_isp *isp)
{
	if (!isp->extra_surf)
		return;

	isp_free_surface(isp, isp->extra_surf);
	isp->extra_surf = NULL;
}

static void
isp_write_bootargs_h16(struct apple_isp *isp, void *virt, u32 args_offset)
{
	struct isp_firmware_bootargs_h16 args;

	memset(&args, 0, sizeof(args));
	args.ipc_iova = isp->ipc_surf->iova;
	args.shared_base = isp->fw.heap_top & 0xffffffff;
	args.shared_size = ISP_H16_SHARED_SIZE;
	args.extra_iova = isp->extra_surf->iova;
	args.extra_size = isp->extra_surf->size;
	args.platform_id = isp->platform_id;
	args.ipc_queue_size = (u64)args_offset + 1;
	args.unk_68 = 0x40;
	put_unaligned_le64(isp->cam_connections_scheme,
	    &args.descriptor[ISP_H16_DESC_CAM_SCHEME]);
	args.descriptor[ISP_H16_DESC_NO_OPTICAL_CARD_ID] = 1;
	memcpy(virt, &args, sizeof(args));
}

static int
isp_firmware_boot_stage2(struct apple_isp *isp)
{
	size_t args_size = sizeof(struct isp_firmware_bootargs_h16);
	size_t cmd_size = ISP_CMD_AREA_SIZE(isp_num_capmeta(isp));
	dma_addr_t args_iova, cmd_iova;
	void *args_virt, *cmd_virt;

	u32 num_ipc_chans = isp_gpio_read32(isp, ISP_GPIO_0);
	u32 args_offset = isp_gpio_read32(isp, ISP_GPIO_1);
	u32 desc_flags = isp_gpio_read32(isp, ISP_GPIO_2);
	u32 extra_size = isp_gpio_read32(isp, ISP_GPIO_3);
	isp->num_ipc_chans = num_ipc_chans;

	if (!num_ipc_chans || num_ipc_chans > ISP_IPC_MAX_CHANNELS) {
		dev_err(isp->dev, "invalid IPC channel count %u\n",
		    num_ipc_chans);
		return (-ENODEV);
	}

	if (isp->num_ipc_chans != 7)
		dev_warn(isp->dev, "unexpected channel count (%d)\n",
		    num_ipc_chans);

	/* Only hand the H16 descriptor to firmware that asks for it. */
	if (!(desc_flags & ISP_GPIO2_H16_DESCRIPTOR)) {
		dev_err(isp->dev,
		    "firmware did not request the H16 boot descriptor (0x%x)\n",
		    desc_flags);
		return (-ENODEV);
	}

	/*
	 * The firmware picks the offset of the boot arguments in the IPC
	 * surface, and the command area follows them. Both have to lie
	 * inside the surface, and the command area must hold the largest
	 * buffer batch, which is more than any command needs.
	 */
	args_iova = isp->ipc_surf->iova + (u64)args_offset + 0x40;
	cmd_iova = args_iova + args_size + 0x40;
	args_virt = apple_isp_ipc_translate(isp, args_iova, args_size);
	cmd_virt = apple_isp_ipc_translate(isp, cmd_iova, cmd_size);
	if (!args_virt || !cmd_virt) {
		dev_err(isp->dev, "invalid boot arguments offset 0x%x\n",
		    args_offset);
		return (-EIO);
	}

	isp->extra_surf = isp_alloc_surface_vmap(isp, extra_size);
	if (!isp->extra_surf) {
		isp_err(isp, "failed to alloc surface for extra heap\n");
		return (-ENOMEM);
	}

	isp->cmd_iova = cmd_iova;
	isp->ipc_boot_iova = args_iova;
	isp->ipc_boot_size = args_size + 0x40 + cmd_size;
	isp->cmd_virt = cmd_virt;

	isp_write_bootargs_h16(isp, args_virt, args_offset);

	isp_gpio_write32(isp, ISP_GPIO_0, args_iova);
	isp_gpio_write32(isp, ISP_GPIO_1, args_iova >> 32);
	dma_wmb();

	/* Wait for ISP_GPIO_7 to 0xf7fbdff9 -> 0x8042006 */
	isp_gpio_write32(isp, ISP_GPIO_7, 0xf7fbdff9);

	/*
	 * The firmware may already use the extra heap: it is freed only once
	 * the coprocessor has stopped, see isp_firmware_boot().
	 */
	if (isp_gpio_wait(isp, ISP_GPIO_7, 0x8042006)) {
		isp_err(isp,
		    "never received second magic number from firmware\n");
		return (-ENODEV);
	}
	isp_dbg(isp, "got second magic number from firmware\n");

	return (0);
}

static struct isp_channel *
isp_get_chan_index(struct apple_isp *isp, const char *name)
{
	for (int i = 0; i < isp->num_ipc_chans; i++) {
		if (!strcasecmp(isp->ipc_chans[i]->name, name))
			return (isp->ipc_chans[i]);
	}
	return (NULL);
}

static void
isp_free_channel_info(struct apple_isp *isp)
{
	if (!isp->ipc_chans)
		return;

	for (int i = 0; i < isp->num_ipc_chans; i++) {
		struct isp_channel *chan = isp->ipc_chans[i];
		if (!chan)
			continue;
		free(chan->name, M_APPLEISP);
		sx_destroy(&chan->lock);
		free(chan, M_APPLEISP);
		isp->ipc_chans[i] = NULL;
	}
	free(isp->ipc_chans, M_APPLEISP);
	isp->ipc_chans = NULL;
	isp->chan_tm = isp->chan_io = isp->chan_dg = isp->chan_bh = NULL;
	isp->chan_bt = isp->chan_sm = isp->chan_it = NULL;
}

/* A ring inside the IPC surface, 64-byte slots, overlapping nothing before it. */
static bool
isp_ipc_ring_ok(struct apple_isp *isp, u64 iova, u64 size, const u64 *res,
    unsigned int nres)
{
	u64 base = isp->ipc_surf->iova, end = base + isp->ipc_surf->size;

	if ((iova & 7) || (size & 63) || !size || iova < base || iova > end ||
	    size > end - iova)
		return (false);
	for (unsigned int i = 0; i < nres; i++)
		if (iova < res[2 * i] + res[2 * i + 1] && res[2 * i] < iova + size)
			return (false);
	return (true);
}

static int
isp_fill_channel_info(struct apple_isp *isp)
{
	u64 reserved[2 * (ISP_IPC_MAX_CHANNELS + 2)];
	u64 table_iova = isp_gpio_read32(isp, ISP_GPIO_0) |
	    ((u64)isp_gpio_read32(isp, ISP_GPIO_1)) << 32;
	size_t table_size = sizeof(struct isp_chan_desc) * isp->num_ipc_chans;
	char *table_virt = apple_isp_ipc_translate(isp,
	    isp_fw_iova(isp, table_iova), table_size);
	int err = -EIO;

	if (!table_virt) {
		dev_err(isp->dev, "Failed to find channel table\n");
		return (-EIO);
	}
	reserved[0] = isp_fw_iova(isp, table_iova);
	reserved[1] = table_size;
	/* The argument/command span validated by stage 2 */
	reserved[2] = isp->ipc_boot_iova;
	reserved[3] = isp->ipc_boot_size;
	if (reserved[0] < reserved[2] + reserved[3] &&
	    reserved[2] < reserved[0] + reserved[1]) {
		isp_err(isp, "invalid ipc channel table or boot storage\n");
		return (-EIO);
	}

	isp->ipc_chans = malloc(isp->num_ipc_chans * sizeof(*isp->ipc_chans),
	    M_APPLEISP, M_WAITOK | M_ZERO);

	for (int i = 0; i < isp->num_ipc_chans; i++) {
		struct isp_chan_desc desc;
		struct isp_channel *chan = malloc(sizeof(*chan), M_APPLEISP,
		    M_WAITOK | M_ZERO);
		isp->ipc_chans[i] = chan;

		memcpy(&desc, table_virt + i * sizeof(desc), sizeof(desc));
		/* The firmware does not have to NUL-terminate the name. */
		chan->name = malloc(sizeof(desc.name) + 1, M_APPLEISP,
		    M_WAITOK | M_ZERO);
		memcpy(chan->name, desc.name, sizeof(desc.name));
		chan->type = desc.type;
		chan->src = desc.src;
		chan->num = desc.num;
		chan->size = (u64)desc.num * ISP_IPC_MESSAGE_SIZE;
		chan->iova = isp_fw_iova(isp, desc.iova);
		chan->cursor = 0;
		sx_init(&chan->lock, "apple_isp chan");

		if ((chan->type != ISP_IPC_CHAN_TYPE_COMMAND) &&
		    (chan->type != ISP_IPC_CHAN_TYPE_REPLY) &&
		    (chan->type != ISP_IPC_CHAN_TYPE_REPORT)) {
			isp_err(isp, "invalid ipc chan type (%d)\n",
			    chan->type);
			goto out;
		}

		/* Each channel has its own doorbell bit and at least one slot. */
		if (chan->src >= ISP_IPC_MAX_CHANNELS || !chan->num) {
			isp_err(isp, "invalid ipc chan %s (src %u, num %u)\n",
			    chan->name, chan->src, chan->num);
			goto out;
		}
		chan->doorbell = BIT(chan->src);
		if (!isp_ipc_ring_ok(isp, chan->iova, chan->size, reserved,
		    i + 2)) {
			isp_err(isp, "invalid ipc chan %s layout\n", chan->name);
			goto out;
		}
		reserved[2 * (i + 2)] = chan->iova;
		reserved[2 * (i + 2) + 1] = chan->size;

		chan->virt = apple_isp_ipc_translate(isp, chan->iova,
		    chan->size);
		if (!chan->virt) {
			dev_err(isp->dev, "Failed to find channel buffer\n");
			goto out;
		}

		dev_info(isp->dev, "chan: %s type: %d src: %d num: %d iova: "
		    "0x%jx\n", chan->name, chan->type, chan->src, chan->num,
		    (uintmax_t)chan->iova);
	}

	isp->chan_tm = isp_get_chan_index(isp, "TERMINAL");
	isp->chan_io = isp_get_chan_index(isp, "IO");
	isp->chan_dg = isp_get_chan_index(isp, "DEBUG");
	isp->chan_bh = isp_get_chan_index(isp, "BUF_H2T");
	isp->chan_bt = isp_get_chan_index(isp, "BUF_T2H");
	isp->chan_sm = isp_get_chan_index(isp, "SHAREDMALLOC");
	isp->chan_it = isp_get_chan_index(isp, "IO_T2H");

	if (!isp->chan_tm || !isp->chan_io || !isp->chan_dg ||
	    !isp->chan_bh || !isp->chan_bt || !isp->chan_sm || !isp->chan_it) {
		isp_err(isp, "did not find all of the required ipc chans\n");
		goto out;
	}

	isp->chan_tm->ops = &tm_ops;
	isp->chan_sm->ops = &sm_ops;
	isp->chan_bt->ops = &bt_ops;

	return (0);
out:
	isp_free_channel_info(isp);
	return (err);
}

static int
isp_firmware_boot_stage3(struct apple_isp *isp)
{
	int err;

	err = isp_fill_channel_info(isp);
	if (err < 0)
		return (err);

	/* Mask the command channels to prepare for submission */
	for (int i = 0; i < isp->num_ipc_chans; i++) {
		struct isp_channel *chan = isp->ipc_chans[i];
		if (chan->type != ISP_IPC_CHAN_TYPE_COMMAND)
			continue;
		for (u32 j = 0; j < chan->num; j++) {
			struct isp_message msg;

			memset(&msg, 0, sizeof(msg));
			msg.arg0 = ISP_IPC_FLAG_ACK;
			memcpy((char *)chan->virt + j * sizeof(msg), &msg,
			    sizeof(msg));
		}
	}
	dma_wmb();

	/* Wait for ISP_GPIO_3 to 0x8042006 -> 0x0 */
	isp_gpio_write32(isp, ISP_GPIO_3, 0x8042006);

	if (isp_gpio_wait(isp, ISP_GPIO_3, 0x0)) {
		isp_err(isp,
		    "never received third magic number from firmware\n");
		isp_free_channel_info(isp);
		return (-ENODEV);
	}

	device_printf(isp->dev, "firmware booted\n");

	return (0);
}

/* Wait for the word of its own memory the firmware sets once it has stopped. */
static void
isp_wait_stop_key(struct apple_isp *isp, u64 addr, u32 key)
{
	u64 iova = isp_fw_iova(isp, addr);
	vm_paddr_t pa;
	volatile u32 *word;

	if (iova >= isp->fw.heap_top ||
	    apple_dart_fw_lookup(isp->darts[0], iova, sizeof(*word), &pa)) {
		dev_warn(isp->dev, "stop key at 0x%jx isn't in firmware "
		    "memory\n", (uintmax_t)addr);
		return;
	}
	word = (volatile u32 *)((char *)pmap_mapdev_attr(trunc_page(pa),
	    PAGE_SIZE, VM_MEMATTR_WRITE_COMBINING) + (pa & PAGE_MASK));

	for (int i = 0; i < ISP_STOP_KEY_TRIES && *word != key; i++)
		pause("ispkey", MAX(1, hz * ISP_STOP_KEY_MS / 1000));

	if (*word != key)
		dev_warn(isp->dev, "stop key at 0x%jx reads 0x%x, not 0x%x\n",
		    (uintmax_t)addr, *word, key);

	pmap_unmapdev((void *)trunc_page((vm_offset_t)__DEVOLATILE(u32 *, word)),
	    PAGE_SIZE);
}

/*
 * Stop firmware that starts cold next time. CISP_CMD_SUSPEND is for firmware
 * that resumes: it isn't followed by a cold start.
 */
static int
isp_stop_firmware_cold(struct apple_isp *isp)
{
	struct cmd_stop_key stop_key = {};
	u32 status;
	int err;

	/* Without the key the rest of the stop still works. */
	if (isp_cmd_stop_key_get(isp, &stop_key))
		stop_key.addr = 0;

	err = isp_cmd_stop_cold(isp, 0);
	if (err) {
		isp_err(isp, "isp_cmd_stop_cold() failed\n");
		return (err);
	}

	/* The firmware answers the power-down in this word, not on the ring. */
	isp_gpio_write32(isp, ISP_GPIO_0, 0xf7fbdff9);

	err = isp_cmd_power_down(isp);
	if (err) {
		isp_err(isp, "isp_cmd_power_down() failed\n");
		return (err);
	}

	if (isp_gpio_wait(isp, ISP_GPIO_0, 0x8042006)) {
		isp_err(isp, "never received magic number from firmware\n");
		return (-ENODEV);
	}

	if (stop_key.addr)
		isp_wait_stop_key(isp, stop_key.addr, stop_key.key);

	/* Its domains go off next, so its CPU should be idle by now. */
	if (isp_poll32(isp->coproc, ISP_COPROC_STATUS_T8140, ISP_COPROC_IN_WFI,
	    ISP_COPROC_IN_WFI, ISP_STOP_WFI_POLL_US, ISP_STOP_WFI_US, &status))
		dev_warn(isp->dev, "coprocessor not in WFI (status: 0x%x)\n",
		    status);

	return (0);
}

static int
isp_stop_command_processor(struct apple_isp *isp)
{
	if (READ_ONCE(isp->ipc_error))
		return (READ_ONCE(isp->ipc_error));

	return (isp_stop_firmware_cold(isp));
}

/* The H17 firmware takes its broadcast-clear windows in the multi-window form. */
static int
isp_set_dsid_clr(struct apple_isp *isp)
{
	const struct apple_isp_hw *hw = isp->hw;
	u64 bases[] = { hw->dsid_clr_base0 };

	return (isp_cmd_set_dsid_clr_multi_bc(isp, bases, hw->dsid_count,
	    hw->dsid_clr_range0));
}

static int
isp_start_command_processor(struct apple_isp *isp)
{
	int err;

	err = isp_cmd_print_enable(isp, 1);
	if (err)
		return (err);

	err = isp_set_dsid_clr(isp);
	if (err)
		return (err);

	err = isp_cmd_pmp_ctrl_set(isp, isp->hw->clock_scratch,
	    isp->hw->clock_base, isp->hw->clock_bit, isp->hw->clock_size,
	    isp->hw->bandwidth_scratch, isp->hw->bandwidth_base,
	    isp->hw->bandwidth_bit, isp->hw->bandwidth_size);
	if (err)
		return (err);

	err = isp_cmd_start(isp, 0);
	if (err)
		return (err);

	/* Now we can access CISP_CMD_CH_* commands */

	return (0);
}

static void
isp_collect_gc_surface(struct apple_isp *isp)
{
	struct isp_surf *surf;

	isp->log_surf = NULL;
	isp->bt_surf = NULL;

	isp_free_released_surfaces(isp);

	while ((surf = TAILQ_LAST(&isp->gc, isp_surf_list)) != NULL) {
		isp_dbg(isp, "freeing iova: 0x%jx size: 0x%jx\n",
		    (uintmax_t)surf->iova, (uintmax_t)surf->size);
		isp_free_surface(isp, surf);
	}
}

/*
 * Stop the coprocessor and power its domains down, then release the memory
 * the firmware may still be using: its channel table, extra heap and the
 * surfaces it requested.
 */
static void
isp_firmware_stop(struct apple_isp *isp)
{
	isp_firmware_shutdown_stage1(isp);

	isp_free_channel_info(isp);
	isp_firmware_shutdown_stage2(isp);
	isp_collect_gc_surface(isp);
}

static int
isp_firmware_boot(struct apple_isp *isp)
{
	int err;

	err = isp_firmware_boot_stage1(isp);
	if (err < 0) {
		/* Stage 1 stops the coprocessor and powers it down itself. */
		isp_err(isp, "failed firmware boot stage 1: %d\n", err);
		isp_collect_gc_surface(isp);
		atomic_cmpset_int((volatile u_int *)&isp->ipc_error, 0, err);
		return (err);
	}

	/* From here on the firmware runs, and may use what it was given. */
	err = isp_firmware_boot_stage2(isp);
	if (err < 0) {
		isp_err(isp, "failed firmware boot stage 2: %d\n", err);
		goto stop;
	}

	err = isp_firmware_boot_stage3(isp);
	if (err < 0) {
		isp_err(isp, "failed firmware boot stage 3: %d\n", err);
		goto stop;
	}

	/* The old coprocessor stopped and all channel slots were initialized. */
	WRITE_ONCE(isp->ipc_error, 0);
	WRITE_ONCE(isp->ipc_quiescing, false);

	err = isp_enable_irq(isp);
	if (err < 0) {
		isp_err(isp, "failed to enable interrupts: %d\n", err);
		goto stop;
	}

	err = isp_start_command_processor(isp);
	if (err < 0) {
		isp_err(isp, "failed to start command processor: %d\n", err);
		goto disable_irqs;
	}

	return (0);

disable_irqs:
	atomic_cmpset_int((volatile u_int *)&isp->ipc_error, 0, err);
	isp_disable_irq(isp);
stop:
	atomic_cmpset_int((volatile u_int *)&isp->ipc_error, 0, err);
	isp_firmware_stop(isp);
	return (err);
}

static void
isp_firmware_shutdown(struct apple_isp *isp)
{
	/*
	 * Stop the coprocessor before releasing any memory the firmware
	 * uses, also when it did not acknowledge the stop request.
	 */
	if (isp_stop_command_processor(isp))
		dev_warn(isp->dev,
		    "firmware did not stop, stopping the coprocessor\n");
	/* Also stop a receive task whose producer never leaves a gap. */
	atomic_cmpset_int((volatile u_int *)&isp->ipc_error, 0, -ESHUTDOWN);
	isp_disable_irq(isp);
	isp_firmware_stop(isp);
}

/*
 * After a restart, the firmware must find the camera found at the first
 * start. The query also shows that it takes commands.
 */
static int
isp_check_restarted(struct apple_isp *isp)
{
	struct cmd_config_get args = {};
	int err;

	err = isp_cmd_config_get(isp, &args);
	if (err)
		return (err);

	if (min_t(u32, args.num_channels, ISP_MAX_CHANNELS) !=
	    (u32)isp->num_channels) {
		isp_err(isp, "firmware found %u channels after a restart, %d "
		    "before\n", args.num_channels, isp->num_channels);
		return (-ENODEV);
	}

	return (0);
}

/*
 * The firmware is loaded by the bootloader and changes its own memory as it
 * runs. On T8142 it starts again after a stop and rebuilds that memory itself.
 * Called with video_lock held.
 */
int
apple_isp_firmware_boot(struct apple_isp *isp)
{
	bool restart = isp->fw.started;
	int err;

	sx_assert(&isp->video_lock, SA_XLOCKED);
	switch (isp->fw_state) {
	case ISP_FW_OFF:
		break;
	case ISP_FW_RUNNING:
		err = READ_ONCE(isp->ipc_error);
		return (err);
	case ISP_FW_DEAD:
		dev_err(isp->dev, "firmware was stopped and cannot be "
		    "restarted before the next boot\n");
		return (-EIO);
	}

	/* The firmware starts with new rings. */
	isp->ipc_stale = NULL;
	isp->ipc_failed = false;
	WRITE_ONCE(isp->ipc_error, 0);

	err = isp_firmware_boot(isp);
	if (!err && restart && isp->camera_detected) {
		err = isp_check_restarted(isp);
		if (err)
			isp_firmware_shutdown(isp);
	}
	if (err) {
		dev_err(isp->dev, "failed to boot firmware: %d\n", err);
		return (err);
	}

	isp->fw_state = ISP_FW_RUNNING;

	return (0);
}

/* Stop the firmware; it can start again. Called with video_lock held. */
void
apple_isp_firmware_halt(struct apple_isp *isp)
{
	if (isp->fw_state != ISP_FW_RUNNING)
		return;

	isp_firmware_shutdown(isp);
	isp->ipc_stale = NULL;
	isp->fw_state = ISP_FW_OFF;
}
