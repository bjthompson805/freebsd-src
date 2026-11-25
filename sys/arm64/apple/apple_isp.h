// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */
/*
 * Apple ISP (camera) driver: a port of aurora-silicon's Linux
 * drivers/media/platform/apple/isp to FreeBSD, for the M5 (T8142, ISP17a,
 * H17 firmware ABI) only. The firmware is resident: iBoot loads it, m1n1
 * powers the ISP and carves its heap (isp-heap), and the driver starts and
 * stops the coprocessor around streams.
 *
 * The files keep the Linux driver's structure, names and negative errno
 * convention; the block below gives the ported code the Linux types and
 * helpers it uses. FreeBSD interfaces (newbus, the DART, the video device)
 * take positive errnos, converted where they meet.
 */

#ifndef _ARM64_APPLE_APPLE_ISP_H_
#define _ARM64_APPLE_APPLE_ISP_H_

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/condvar.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/rman.h>
#include <sys/selinfo.h>
#include <sys/sx.h>
#include <sys/callout.h>
#include <sys/taskqueue.h>
#include <sys/vmem.h>
#include <machine/atomic.h>
#include <machine/bus.h>

#include <dev/ofw/openfirm.h>

/* Linux types and helpers for the ported code */
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef unsigned long long u64;	/* as Linux's, for its %llx */
typedef uint64_t dma_addr_t;
/* Linux's one-argument form */
#define	static_assert(...)	_Static_assert(__VA_ARGS__, #__VA_ARGS__)
#define	IS_ENABLED(x)		0
#define	BIT(n)			(1u << (n))
#define	BIT_ULL(n)		(1ull << (n))
#define	GENMASK_ULL(h, l)	(((~0ull) >> (63 - (h))) & ~((1ull << (l)) - 1))
#define	ARRAY_SIZE(a)		nitems(a)
#define	U64_MAX			UINT64_MAX
#define	min_t(t, a, b)		MIN((t)(a), (t)(b))
#define	max_t(t, a, b)		MAX((t)(a), (t)(b))
#define	READ_ONCE(x)		(*(volatile __typeof(x) *)&(x))
#define	WRITE_ONCE(x, v)	(*(volatile __typeof(x) *)&(x) = (v))
#define	dma_wmb()		wmb()
#define	dma_rmb()		rmb()
#define	get_unaligned_le64(p)	le64dec(p)
#define	put_unaligned_le64(v, p) le64enc((p), (v))
/* Timeouts are in milliseconds (ipc_chan_send()). */
#define	msecs_to_jiffies(ms)	(ms)

MALLOC_DECLARE(M_APPLEISP);
#define	kzalloc_obj(x)		malloc(sizeof(x), M_APPLEISP, M_WAITOK | M_ZERO)
#define	kfree(p)		free((p), M_APPLEISP)

#define	dev_err(d, fmt, ...)	device_printf((d), fmt, ##__VA_ARGS__)
#define	dev_warn(d, fmt, ...)	device_printf((d), fmt, ##__VA_ARGS__)
#define	dev_info(d, fmt, ...)	device_printf((d), fmt, ##__VA_ARGS__)
#define	dev_dbg(d, fmt, ...)	do {					\
	if (apple_isp_debug)						\
		device_printf((d), fmt, ##__VA_ARGS__);			\
} while (0)
#define	dev_err_ratelimited	dev_err
#define	dev_warn_ratelimited	dev_warn
#define	isp_dbg(isp, fmt, ...)	\
	dev_dbg((isp)->dev, "[%s] " fmt, __func__, ##__VA_ARGS__)
#define	isp_err(isp, fmt, ...)	\
	dev_err((isp)->dev, "[%s] " fmt, __func__, ##__VA_ARGS__)

extern int apple_isp_debug;

#define	APPLE_ISP_CARD_NAME	"FaceTime HD Camera"

#define	ISP_MAX_CHANNELS	6
#define	ISP_IPC_MESSAGE_SIZE	64
#define	ISP_IPC_FLAG_ACK	0x1
#define	ISP_META_SIZE_T8140	0x4d00
#define	ISP_CAPTURE_META_SIZE_T8140 0x29fc0

/* used to limit the user space buffers to the buffer_pool_config */
#define	ISP_MAX_BUFFERS		16
#define	ISP_MAX_SENSOR_CONFIGS	64
#define	ISP_SENSOR_CONFIG_SIZE	0x120

/* capture metadata buffers, for firmware that has that pool */
#define	ISP_CAPMETA_BUFFERS	8

/*
 * The command area holds one command or one buffer batch at a time. A
 * batch is a 16-byte header and a 64-byte descriptor per buffer, and
 * carries at most every metadata buffer, the @capmeta capture metadata
 * buffers and one full rendered pool.
 */
#define	ISP_BUFLIST_HDR_SIZE	0x10
#define	ISP_BUFLIST_DESC_SIZE	0x40
#define	ISP_CMD_AREA_SIZE(capmeta) \
	(ISP_BUFLIST_HDR_SIZE + ISP_BUFLIST_DESC_SIZE * \
	 (ISP_MAX_BUFFERS + (capmeta) + ISP_MAX_BUFFERS))

enum isp_generation {
	ISP_GEN_T8140,
};

enum isp_fw_abi {
	ISP_FW_ABI_LEGACY,
	ISP_FW_ABI_H17,
};

enum isp_fw_state {
	ISP_FW_OFF,
	ISP_FW_RUNNING,
	/* resident firmware that stopped: it cannot be started again */
	ISP_FW_DEAD,
};

enum isp_firmware_version {
	ISP_FIRMWARE_V_UNKNOWN,
	ISP_FIRMWARE_V_12_3,
	ISP_FIRMWARE_V_12_4,
	ISP_FIRMWARE_V_13_5,
	ISP_FIRMWARE_V_14_7,
	ISP_FIRMWARE_V_26_6_2,
};

struct isp_surf {
	TAILQ_ENTRY(isp_surf) head;
	u64 size;
	u64 type;
	vm_paddr_t pa;		/* physically contiguous */
	dma_addr_t iova;
	void *virt;		/* write-combining kernel mapping */
	u_int refcount;
	bool gc;
	bool submitted;		/* a metadata buffer the firmware holds */
	bool iova_reserved;	/* @iova comes from &apple_isp.iovas */
};
TAILQ_HEAD(isp_surf_list, isp_surf);

struct isp_message {
	u64 arg0;
	u64 arg1;
	u64 arg2;
	u64 arg3;
	u64 arg4;
	u64 arg5;
	u64 arg6;
	u64 arg7;
} __packed;
static_assert(sizeof(struct isp_message) == ISP_IPC_MESSAGE_SIZE);

struct apple_isp;
struct apple_dart_fw;
struct isp_channel;

struct isp_chan_ops {
	int (*handle)(struct apple_isp *isp, struct isp_channel *chan);
	/* Called after the response slot and its doorbell are published. */
	void (*complete)(struct apple_isp *isp);
};

struct isp_channel {
	char *name;
	u32 type;
	u32 src;
	u32 num;
	u64 size;
	dma_addr_t iova;
	void *virt;
	u32 doorbell;
	u32 cursor;
	struct sx lock;
	struct isp_message req;
	struct isp_message rsp;
	const struct isp_chan_ops *ops;
};

struct coord {
	u32 x;
	u32 y;
};

/* MMIO window the firmware accesses at its physical address */
struct isp_mmio_window {
	u64 base;
	u64 size;
};

/* A DART address filter window, both bounds inclusive. */
struct isp_dapf_window {
	u64 start;
	u64 end;
};

struct isp_preset {
	u32 index;
	struct coord input_dim;
	struct coord output_dim;
	struct coord crop_offset;
	struct coord crop_size;
	bool p010;
};

/* The T8142 subset of Linux's struct apple_isp_hw (see apple_isp.c). */
struct apple_isp_hw {
	enum isp_generation gen;
	enum isp_fw_abi fw_abi;

	int dsid_count;
	u64 dsid_clr_base0;
	u32 dsid_clr_range0;

	u64 clock_scratch;
	u64 clock_base;
	u8 clock_bit;
	u8 clock_size;
	u64 bandwidth_scratch;
	u64 bandwidth_base;
	u8 bandwidth_bit;
	u8 bandwidth_size;

	u32 mbox_irq_enable;
	u32 meta_size;
	bool scl1;
	bool lpdp;

	u64 fw_iova_mask;
	u32 mbox2_offset;
	u32 coproc_control;
	u32 coproc_status;
	u32 coproc_quiesce;

	const struct isp_mmio_window *fw_mmio;
	unsigned int num_fw_mmio;

	u32 boot_mode;
	u32 capture_meta_size;
	unsigned int fw_idle_ms;
	bool setfile_stride_fixup;
};

enum isp_sensor_id {
	ISP_IMX958_2224_01,
	ISP_IMX958_2226_02,
};

struct isp_sensor_config {
	struct coord input_dim;
	struct coord output_dim;
	u32 sensor_clk;
	bool usable;
	u8 reply[ISP_SENSOR_CONFIG_SIZE];
};

#define	VB2_MAX_PLANES	8

struct isp_format {
	enum isp_sensor_id id;
	u32 version;
	u32 pixelformat;
	struct isp_preset *preset;
	unsigned int num_planes;
	u32 strides[VB2_MAX_PLANES];
	size_t plane_size[VB2_MAX_PLANES];
	size_t total_size;
};

#define	ISP_NUM_DARTS	3

struct apple_isp {
	device_t dev;
	const struct apple_isp_hw *hw;
	enum isp_firmware_version fw_compat;
	u32 platform_id;
	u32 temporal_filter;
	u32 cam_connections_scheme;
	struct isp_preset *presets;
	int num_presets;

	int num_channels;
	struct isp_format fmts[ISP_MAX_CHANNELS];
	unsigned int current_ch;

	/* serializes firmware boots, stops and commands (Linux's video_lock) */
	struct sx video_lock;

	int pd_count;
	bool pds_active;

	struct resource *coproc;
	struct resource *mbox;
	struct resource *gpio;
	struct resource *wdt;
	/* isp_dart0's address filter registers, not ours: mapped directly */
	void *dapf;
	bus_size_t dapf_size;
	bus_size_t mbox2;	/* offset of the doorbells in @mbox */
	struct resource *irq_res;
	void *irq_cookie;
	bool irq_on;

	/* the firmware's DART address filter, from isp_dart0 */
	struct isp_dapf_window *dapf_windows;
	u32 *dapf_r20;
	unsigned int num_dapf;

	struct apple_dart_fw *darts[ISP_NUM_DARTS];
	/* their registers, for the stream remaps and fault reports */
	void *dart_regs[ISP_NUM_DARTS];
	bus_size_t dart_regs_size[ISP_NUM_DARTS];
	vmem_t *iovas;
	u64 iova_size;
	u64 fw_iova_mask;

	struct isp_firmware {
		u64 heap_base;
		u64 heap_top;
		vm_paddr_t heap_pa;
		u64 entry;
		bool started;
	} fw;
	enum isp_fw_state fw_state;
	int ipc_error;
	bool ipc_quiescing;
	bool camera_detected;
	u32 num_fw_presets;

	struct isp_surf *ipc_surf;
	struct isp_surf *extra_surf;
	struct isp_surf *data_surf;
	struct isp_surf *log_surf;
	struct isp_surf *bt_surf;
	struct isp_surf_list gc;
	struct isp_surf_list gc_released;
	struct sx gc_lock;

	struct taskqueue *tq;
	struct task irq_task;

	int num_ipc_chans;
	struct isp_channel **ipc_chans;
	struct isp_channel *chan_tm; /* TERMINAL */
	struct isp_channel *chan_io; /* IO */
	struct isp_channel *chan_dg; /* DEBUG */
	struct isp_channel *chan_bh; /* BUF_H2T */
	struct isp_channel *chan_bt; /* BUF_T2H */
	struct isp_channel *chan_sm; /* SHAREDMALLOC */
	struct isp_channel *chan_it; /* IO_T2H */

	/* wait_event: sleepers on &wait under wait_mtx, see isp_wake() */
	struct mtx wait_mtx;
	int wait;
	dma_addr_t cmd_iova;
	void *cmd_virt;
	dma_addr_t ipc_boot_iova;
	size_t ipc_boot_size;

	struct isp_channel *ipc_stale;
	bool ipc_failed;

	/* Streaming (apple_isp_video.c) */
	struct cdev *cdev;
	bool opened;
	bool streaming;
	unsigned int frame_rate;	/* frames per second */
	unsigned int sequence;
	struct mtx buf_mtx;		/* the buffer lists and states */
	struct isp_vbuf *vbufs;
	unsigned int nbufs;
	TAILQ_HEAD(, isp_vbuf) bufs_pending;	/* queued, not given yet */
	TAILQ_HEAD(, isp_vbuf) bufs_submitted;	/* the firmware has them */
	TAILQ_HEAD(, isp_vbuf) bufs_done;	/* filled, for DQBUF */
	struct selinfo sel;
	struct isp_surf *meta_surfs[ISP_MAX_BUFFERS];
	struct isp_surf *capmeta_surfs[ISP_CAPMETA_BUFFERS];
	struct callout wdt_callout;
	bool wdt_running;
};

/* A capture buffer: one surface holding both NV12 planes */
struct isp_vbuf {
	TAILQ_ENTRY(isp_vbuf) link;
	struct isp_surf *surf;
	unsigned int index;
	enum { ISP_VBUF_USER, ISP_VBUF_PENDING, ISP_VBUF_SUBMITTED,
	    ISP_VBUF_DONE } state;
	bool error;
	unsigned int sequence;
	struct timeval timestamp;
};

/* Coprocessor registers */
#define	ISP_COPROC_RVBAR		0x1050000
#define	ISP_COPROC_EDPRCR		0x1010310
#define	ISP_COPROC_CONTROL_T8140	0x1600044
#define	ISP_COPROC_STATUS_T8140		0x1600048
#define	ISP_COPROC_QUIESCE_T8140	0x1600040
#define	ISP_COPROC_QUIESCED		BIT(25)
#define	ISP_COPROC_FABRIC_0_T8140	0x748
#define	ISP_COPROC_FABRIC_1_T8140	0x848
#define	ISP_COPROC_FABRIC_2_T8140	0x948
#define	ISP_COPROC_FABRIC_3_T8140	0xa50
#define	ISP_COPROC_IRQ_MASK_0_T8140	0x1600a00
#define	ISP_COPROC_IRQ_MASK_1_T8140	0x1600a04
#define	ISP_COPROC_IRQ_MASK_2_T8140	0x1600a08
#define	ISP_COPROC_IRQ_MASK_3_T8140	0x1600a0c
#define	ISP_COPROC_RESET_ACK_0_T8140	0x1600818
#define	ISP_COPROC_RESET_ACK_1_T8140	0x160081c
#define	ISP_COPROC_IN_WFI		0x3

/* DART address filter (T8110) */
#define	DAPF_WINDOW_STRIDE	0x40
#define	DAPF_CTRL0		0x00
#define	DAPF_CTRL0_ISP		0x31
#define	DAPF_CTRL1		0x04
#define	DAPF_START		0x08
#define	DAPF_END		0x10
#define	DAPF_CTRL2		0x20

/* Mailbox */
#define	ISP_MBOX_IRQ_INTERRUPT		0x00
#define	ISP_MBOX_IRQ_ENABLE_T6031	0x08
#define	ISP_MBOX2_IRQ_DOORBELL		0x00
#define	ISP_MBOX2_IRQ_ACK		0x0c
#define	ISP_MBOX2_SIZE			0x10
#define	ISP_MBOX_IRQ_ENABLE1_T8140	0x18
#define	ISP_MBOX_IRQ_ENABLE2_T8140	0x38
#define	ISP_MBOX_IRQ_ROUTE_T8140	0x73c
#define	ISP_MBOX_IRQ_ENABLE1_T8140_VAL	0x70000000
#define	ISP_MBOX_IRQ_ENABLE2_T8140_VAL	0x700000
#define	ISP_MBOX_IRQ_ROUTE_T8140_VAL	0x8

/* capture watchdog ("wdt") */
#define	ISP_WDT_KICK		0x00
#define	ISP_WDT_RELOAD		0x08
#define	ISP_WDT_CLEAR		0x0c
#define	ISP_WDT_RELOAD_VAL	0x244140

#define	ISP_GPIO_0	0x00
#define	ISP_GPIO_1	0x04
#define	ISP_GPIO_2	0x08
#define	ISP_GPIO_3	0x0c
#define	ISP_GPIO_4	0x10
#define	ISP_GPIO_5	0x14
#define	ISP_GPIO_6	0x18
#define	ISP_GPIO_7	0x1c

/* IPC */
#define	ISP_IPC_CHAN_TYPE_COMMAND	0
#define	ISP_IPC_CHAN_TYPE_REPLY		1
#define	ISP_IPC_CHAN_TYPE_REPORT	2
#define	ISP_IPC_MAX_CHANNELS		32

static inline u32
isp_mbox_read32(struct apple_isp *isp, u32 reg)
{
	return (bus_read_4(isp->mbox, reg));
}

static inline void
isp_mbox_write32(struct apple_isp *isp, u32 reg, u32 val)
{
	bus_write_4(isp->mbox, reg, val);
}

static inline void
isp_mbox2_write32(struct apple_isp *isp, u32 reg, u32 val)
{
	bus_write_4(isp->mbox, isp->mbox2 + reg, val);
}

/*
 * Addresses from the firmware may carry bits above the ones the DART
 * translates (the vm-base); strip them before comparing or translating.
 */
#define	isp_fw_iova(isp, x)	((x) & (isp)->fw_iova_mask)
#define	isp_get_format(isp, ch)	(&(isp)->fmts[(ch)])
#define	isp_num_capmeta(isp) \
	((isp)->hw->capture_meta_size ? ISP_CAPMETA_BUFFERS : 0)
#define	isp_get_current_format(isp) (isp_get_format(isp, isp->current_ch))

/* P010 output is a T6040 (IMX958, J616s) path: never on T8142. */
static inline bool
isp_is_p010(struct apple_isp *isp)
{
	return (false);
}

static inline unsigned int
isp_num_rendered(struct apple_isp *isp)
{
	return (ISP_MAX_BUFFERS);
}

static inline unsigned int
isp_num_meta(struct apple_isp *isp)
{
	return (ISP_MAX_BUFFERS);
}

/* apple_isp.c: surfaces */
struct isp_surf *isp_alloc_surface(struct apple_isp *isp, u64 size, bool gc);
#define	isp_alloc_surface_vmap(isp, size)	isp_alloc_surface((isp), (size), false)
#define	isp_alloc_surface_gc(isp, size)		isp_alloc_surface((isp), (size), true)
void isp_free_surface(struct apple_isp *isp, struct isp_surf *surf);
void isp_free_released_surfaces(struct apple_isp *isp);
void *apple_isp_translate(struct apple_isp *isp, struct isp_surf *surf,
    dma_addr_t iova, size_t size);
#define	apple_isp_ipc_translate(isp, iova, size) \
	apple_isp_translate((isp), (isp)->ipc_surf, (iova), (size))
void isp_wake(struct apple_isp *isp);
int apple_isp_program_dapf(struct apple_isp *isp);
void apple_isp_dart_report(struct apple_isp *isp, bool clear);
void apple_isp_dart_macos_init(struct apple_isp *isp);

/* apple_isp_fw.c */
int apple_isp_alloc_firmware_surface(struct apple_isp *isp);
void apple_isp_free_firmware_surface(struct apple_isp *isp);
int apple_isp_firmware_boot(struct apple_isp *isp);
void apple_isp_firmware_shutdown(struct apple_isp *isp);
void apple_isp_firmware_halt(struct apple_isp *isp);
void apple_isp_irq_task(void *arg, int pending);

/* apple_isp_ipc.c */
int ipc_chan_handle(struct apple_isp *isp, struct isp_channel *chan);
int ipc_chan_send(struct apple_isp *isp, struct isp_channel *chan,
    unsigned long timeout);
int isp_ipc_resync(struct apple_isp *isp);
int ipc_tm_handle(struct apple_isp *isp, struct isp_channel *chan);
int ipc_sm_handle(struct apple_isp *isp, struct isp_channel *chan);
int ipc_bt_handle(struct apple_isp *isp, struct isp_channel *chan);
void ipc_bt_complete(struct apple_isp *isp);

/* apple_isp_cam.c */
int apple_isp_detect_camera(struct apple_isp *isp);
int apple_isp_start_camera(struct apple_isp *isp);
void apple_isp_stop_camera(struct apple_isp *isp);
int apple_isp_start_capture(struct apple_isp *isp);
void apple_isp_stop_capture(struct apple_isp *isp);
void apple_isp_wdt_stop(struct apple_isp *isp);
int isp_set_preset(struct apple_isp *isp, struct isp_format *fmt,
    struct isp_preset *preset);

/* apple_isp_video.c */
int apple_isp_setup_video(struct apple_isp *isp);
void apple_isp_remove_video(struct apple_isp *isp);

#define	ISP_FRAME_RATE_SCALE	256
#define	ISP_FRAME_RATE_MIN	15
#define	ISP_FRAME_RATE_DEFAULT	30

#endif /* _ARM64_APPLE_APPLE_ISP_H_ */
