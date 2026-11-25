// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */
/*
 * Apple ISP camera: aurora-silicon's Linux isp-cam.c (detection, capture
 * setup, the capture watchdog), its T8142 (H17) paths.
 */

#include <sys/param.h>
#include <sys/endian.h>
#include <sys/firmware.h>

#include <arm64/apple/apple_isp.h>
#include <arm64/apple/apple_isp_cmd.h>

struct isp_setfile {
	u32 version;
	u32 magic;
	const char *path;
	size_t size;
};

static const struct isp_setfile isp_setfiles[] = {
	[ISP_IMX958_2224_01] = { 0x958, 0x22240106, "apple/isp_2224_01XX.dat",
	    0x6ff4 },
};

static int
isp_ch_get_sensor_id(struct apple_isp *isp, u32 ch)
{
	struct isp_format *fmt = isp_get_format(isp, ch);

	switch (fmt->version) {
	case 0x958:
		/* Section 0x50f of 2224 names sensor 0x958; 2924's names 0x614. */
		fmt->id = ISP_IMX958_2224_01;
		return (0);
	default:
		dev_err(isp->dev, "invalid sensor version: 0x%x\n",
		    fmt->version);
		return (-EINVAL);
	}
}

/*
 * The presets come from the device tree. The firmware numbers its own
 * presets from zero, so a config index at or beyond the count it reports
 * names a preset it does not have, and selecting it can only fail.
 */
static int
isp_check_presets(struct apple_isp *isp, u32 num_fw_presets)
{
	int num = 0;

	/* Nothing to check against */
	if (!num_fw_presets)
		return (0);

	for (int i = 0; i < isp->num_presets; i++) {
		struct isp_preset *preset = &isp->presets[i];

		if (preset->index >= num_fw_presets) {
			dev_warn(isp->dev, "ignoring %ux%u preset: config %u, "
			    "firmware has %u\n", preset->output_dim.x,
			    preset->output_dim.y, preset->index,
			    num_fw_presets);
			continue;
		}
		isp->presets[num++] = *preset;
	}

	if (!num) {
		dev_err(isp->dev, "no usable sensor presets\n");
		return (-ENODEV);
	}

	isp->num_presets = num;
	isp->num_fw_presets = num_fw_presets;
	return (0);
}

static u32
isp_gcd(u32 a, u32 b)
{
	while (b) {
		u32 t = a % b;
		a = b;
		b = t;
	}
	return (a);
}

/* Common call sizes, offered only when a known processed crop contains them. */
static const struct coord isp_call_sizes[] = {
	{ 640, 480 }, { 1280, 720 }, { 1920, 1080 }, { 1080, 1920 },
	{ 1760, 1328 }, { 1328, 1760 }, { 1552, 1552 },
	{ 1280, 960 }, { 960, 1280 },
};

/*
 * CH_CROP[_SCL1]_SET takes x/y/width/height independently of the output
 * dimensions in CH_OUTPUT_CONFIG[_SCL1]_SET. Stay within an established
 * processed crop, rather than the sensor input (which may contain stacked
 * rows). Use the largest exact-aspect crop, centred on the chroma grid,
 * with no upscaling. Neither command reports a scaler limit or rotation.
 */
static bool
isp_size_from_crop(const struct isp_preset *source, struct coord size,
    struct isp_preset *preset)
{
	u32 divisor, x, y, scale;

	if (!size.x || !size.y || (size.x | size.y) & 1 ||
	    !source->crop_size.x || !source->crop_size.y ||
	    (source->crop_offset.x | source->crop_offset.y) & 1 ||
	    source->crop_size.x > 16384 || source->crop_size.y > 16384 ||
	    source->crop_offset.x > source->input_dim.x ||
	    source->crop_size.x > source->input_dim.x - source->crop_offset.x ||
	    source->crop_offset.y > source->input_dim.y ||
	    source->crop_size.y > source->input_dim.y - source->crop_offset.y)
		return (false);

	divisor = isp_gcd(size.x, size.y);
	x = size.x / divisor;
	y = size.y / divisor;
	scale = rounddown2(MIN(source->crop_size.x / x,
	    source->crop_size.y / y), 2);
	if (scale < divisor)
		return (false);

	*preset = *source;
	preset->output_dim = size;
	preset->crop_size = (struct coord){ x * scale, y * scale };
	preset->crop_offset.x += rounddown2((source->crop_size.x - x * scale) / 2,
	    2);
	preset->crop_offset.y += rounddown2((source->crop_size.y - y * scale) / 2,
	    2);
	return (true);
}

static unsigned int
isp_append_size(struct isp_preset *presets, unsigned int num,
    unsigned int sources, struct coord size)
{
	struct isp_preset candidate, best;
	u32 area, best_area = 0;

	for (unsigned int i = 0; i < num; i++)
		if (presets[i].output_dim.x == size.x &&
		    presets[i].output_dim.y == size.y)
			return (num);

	/* Never use a generated size as evidence for another generated size. */
	for (unsigned int i = 0; i < sources; i++) {
		if (!isp_size_from_crop(&presets[i], size, &candidate))
			continue;
		area = candidate.crop_size.x * candidate.crop_size.y;
		if (area > best_area) {
			best = candidate;
			best_area = area;
		}
	}
	if (best_area)
		presets[num++] = best;
	return (num);
}

static int
isp_add_call_sizes(struct apple_isp *isp)
{
	unsigned int sources = isp->num_presets, num = sources;
	struct isp_preset *presets;
	struct isp_preset candidate;
	u32 square = 0;

	presets = malloc((sources + ARRAY_SIZE(isp_call_sizes) + 1) *
	    sizeof(*presets), M_APPLEISP, M_WAITOK | M_ZERO);
	memcpy(presets, isp->presets, sources * sizeof(*presets));

	for (unsigned int i = 0; i < ARRAY_SIZE(isp_call_sizes); i++)
		num = isp_append_size(presets, num, sources, isp_call_sizes[i]);

	/* A smaller sensor can still offer a square without enlarging pixels. */
	for (unsigned int i = 0; i < sources; i++) {
		u32 side = rounddown2(MIN(presets[i].crop_size.x,
		    presets[i].crop_size.y), 2);

		if (isp_size_from_crop(&presets[i], (struct coord){ side, side },
		    &candidate))
			square = MAX(square, side);
	}
	num = isp_append_size(presets, num, sources,
	    (struct coord){ square, square });
	free(isp->presets, M_APPLEISP);
	isp->presets = presets;
	isp->num_presets = num;
	return (0);
}

static int
isp_ch_cache_sensor_info(struct apple_isp *isp, u32 ch)
{
	struct isp_format *fmt = isp_get_format(isp, ch);
	struct cmd_ch_info *args; /* Too big to allocate on stack */
	int err;

	args = malloc(sizeof(*args), M_APPLEISP, M_WAITOK | M_ZERO);

	err = isp_cmd_ch_info_get(isp, ch, args);
	if (err)
		goto exit;

	device_printf(isp->dev, "found sensor %x on ch %d\n", args->version,
	    ch);

	/*
	 * The metadata buffers are allocated for the sizes the driver knows;
	 * without a capture metadata pool its size is not used.
	 */
	if (args->unk_68 != isp->hw->meta_size ||
	    (isp->hw->capture_meta_size &&
	    args->unk_78 != isp->hw->capture_meta_size)) {
		dev_err(isp->dev, "metadata sizes 0x%x/0x%x, expected "
		    "0x%x/0x%x\n", args->unk_68, args->unk_78,
		    isp->hw->meta_size, isp->hw->capture_meta_size);
		err = -ENODEV;
		goto exit;
	}

	fmt->version = args->version;

	err = isp_ch_get_sensor_id(isp, ch);
	if (err) {
		dev_err(isp->dev, "ch %d: unsupported sensor\n", ch);
		err = -ENODEV;
		goto exit;
	}

	/* Once, when the camera is detected: which one it is, and its setfile. */
	dev_info(isp->dev, "ch %u: sensor 0x%x, %u configs, platform 0x%x, "
	    "setfile %s\n", ch, fmt->version, args->num_presets,
	    isp->platform_id, isp_setfiles[fmt->id].path);

	err = isp_check_presets(isp, args->num_presets);
	if (!err)
		err = isp_add_call_sizes(isp);
	for (int i = 0; !err && i < isp->num_presets; i++)
		dev_info(isp->dev, "preset %d: config %u, %ux%u from %ux%u\n",
		    i, isp->presets[i].index, isp->presets[i].output_dim.x,
		    isp->presets[i].output_dim.y, isp->presets[i].crop_size.x,
		    isp->presets[i].crop_size.y);

exit:
	free(args, M_APPLEISP);

	return (err);
}

static int
isp_detect_camera(struct apple_isp *isp)
{
	struct cmd_config_get args;
	int err;

	memset(&args, 0, sizeof(args));

	err = isp_cmd_config_get(isp, &args);
	if (err)
		return (err);

	if (!args.num_channels) {
		dev_err(isp->dev, "did not detect any channels\n");
		return (-ENODEV);
	}

	if (args.num_channels > ISP_MAX_CHANNELS) {
		dev_warn(isp->dev, "found %d channels when maximum is %d\n",
		    args.num_channels, ISP_MAX_CHANNELS);
		args.num_channels = ISP_MAX_CHANNELS;
	}

	if (args.num_channels > 1)
		dev_warn(isp->dev,
		    "warning: driver doesn't support multiple channels\n");

	isp->num_channels = args.num_channels;
	isp->current_ch = 0;

	err = isp_ch_cache_sensor_info(isp, isp->current_ch);
	if (err) {
		dev_err(isp->dev, "failed to cache sensor info\n");
		return (err);
	}

	return (0);
}

/*
 * Boot the firmware, find the camera, and stop it again: it is off until the
 * first stream, as on macOS. Called with video_lock held.
 */
int
apple_isp_detect_camera(struct apple_isp *isp)
{
	int err;

	err = apple_isp_firmware_boot(isp);
	if (err) {
		dev_err(isp->dev, "failed to boot firmware for initial sensor "
		    "detection: %d\n", err);
		return (err);
	}

	err = isp_detect_camera(isp);

	apple_isp_firmware_halt(isp);
	if (!err)
		isp->camera_detected = true;
	return (err);
}

/* NV12 at the preset's output size: luma, then half-height chroma */
#define	ISP_STRIDE_ALIGNMENT	64

int
isp_set_preset(struct apple_isp *isp, struct isp_format *fmt,
    struct isp_preset *preset)
{
	fmt->preset = preset;
	fmt->num_planes = 2;
	fmt->strides[0] = roundup2(preset->output_dim.x, ISP_STRIDE_ALIGNMENT);
	fmt->strides[1] = fmt->strides[0];
	fmt->plane_size[0] = fmt->strides[0] * preset->output_dim.y;
	fmt->plane_size[1] = fmt->strides[1] * preset->output_dim.y / 2;
	fmt->total_size = fmt->plane_size[0] + fmt->plane_size[1];
	return (0);
}

/*
 * A setfile is a chain of { le32 size, le32 0, le32 tag, payload } sections
 * from 0x14 on, size counting what follows it. The lens shading sections
 * hold 18-byte table records after a 10-byte header: { ?, 0x20, 0x20,
 * offset lo, offset hi, 0, 0, width, height }. The xx24 setfiles keep the
 * row stride (2 * width) in "offset hi"; the H17 firmware reads it as the
 * offset's high half ("baseHi") and faults far past the file. Their xx25
 * twins are the same tables with it zeroed, as macOS hands them over.
 */
static void
isp_setfile_fixup_h17(u8 *data, size_t len)
{
	size_t off = 0x14;

	while (off + 12 <= len) {
		u32 size = le32dec(data + off);
		size_t end = off + 4 + size;
		size_t rec;
		int i;

		if (size < 8 || end > len)
			break;

		switch (le32dec(data + off + 8)) {
		case 0xc003:
		case 0xc01b:
		case 0xc01c:
		case 0xc01f:
		case 0xc021:
			rec = off + 12 + 10;
			for (i = 0; i < 16 && rec + 18 <= end; i++, rec += 18) {
				if (le16dec(data + rec + 2) != 0x20 ||
				    le16dec(data + rec + 4) != 0x20)
					break;
				if (le16dec(data + rec + 8) ==
				    2 * le16dec(data + rec + 14))
					le16enc(data + rec + 8, 0);
			}
			break;
		}
		off = end;
	}
}

/* The sensor tuning, /boot/firmware/apple/isp_2224_01XX.dat (from macOS) */
static int
isp_ch_load_setfile(struct apple_isp *isp, u32 ch)
{
	struct isp_format *fmt = isp_get_format(isp, ch);
	const struct isp_setfile *setfile = &isp_setfiles[fmt->id];
	const struct firmware *fw;
	u32 magic;
	int err;

	fw = firmware_get_flags(setfile->path, FIRMWARE_GET_NOWARN);
	if (fw == NULL) {
		static bool warned;

		if (!warned)
			dev_warn(isp->dev, "setfile '%s' missing, streaming "
			    "without sensor tuning\n", setfile->path);
		warned = true;
		return (-ENOENT);
	}
	if (fw->datasize < setfile->size) {
		dev_err(isp->dev, "setfile '%s' too small (0x%zx/0x%zx)\n",
		    setfile->path, fw->datasize, setfile->size);
		firmware_put(fw, FIRMWARE_UNLOAD);
		return (-EINVAL);
	}
	magic = be32dec(fw->data);
	if (magic != setfile->magic) {
		dev_err(isp->dev, "setfile '%s' has magic 0x%08x, expected "
		    "0x%08x\n", setfile->path, magic, setfile->magic);
		firmware_put(fw, FIRMWARE_UNLOAD);
		return (-EINVAL);
	}
	memcpy(isp->data_surf->virt, fw->data, setfile->size);
	firmware_put(fw, FIRMWARE_UNLOAD);
	if (isp->hw->setfile_stride_fixup)
		isp_setfile_fixup_h17(isp->data_surf->virt, setfile->size);

	err = isp_cmd_ch_set_file_load(isp, ch, isp->data_surf->iova,
	    setfile->size);
	if (err)
		dev_err(isp->dev, "ch %u: setfile '%s' load failed: %d\n", ch,
		    setfile->path, err);
	return (err);
}

static int
isp_ch_configure_frame_rate(struct apple_isp *isp, u32 ch)
{
	u32 rate = isp->frame_rate * ISP_FRAME_RATE_SCALE;
	int err;

	err = isp_cmd_ch_ae_frame_rate_max_set(isp, ch, rate);
	if (err)
		return (err);
	/* Auto exposure may lengthen frames in low light, down to 15 fps */
	return (isp_cmd_ch_ae_frame_rate_min_set(isp, ch,
	    ISP_FRAME_RATE_MIN * ISP_FRAME_RATE_SCALE));
}

/* The IMX958 (setfile xx24) runs the temporal filter with band 0 controls */
static int
isp_ch_configure_temporal_filter(struct apple_isp *isp, u32 ch)
{
	int err;

	err = isp_cmd_apple_ch_tnr_start_h17(isp, ch);
	if (err)
		return (err);
	err = isp_cmd_apple_ch_tnr_high_strength_set(isp, ch, 0);
	if (err)
		return (err);
	err = isp_cmd_apple_ch_tnr_band0_mod_set(isp, ch, 0);
	if (err)
		return (err);
	/* 120 and 100, in 1/256 steps */
	err = isp_cmd_apple_ch_tnr_band0_threshold_set(isp, ch, 120 << 8,
	    100 << 8);
	if (err)
		return (err);
	err = isp_cmd_apple_ch_motion_history_start(isp, ch);
	if (err)
		return (err);
	return (isp_cmd_apple_ch_temporal_filter_enable(isp, ch));
}

static int
isp_ch_configure_pools(struct apple_isp *isp, u32 ch)
{
	struct isp_format *fmt = isp_get_format(isp, ch);
	int err;

	err = isp_cmd_ch_buffer_pool_config_set(isp, ch, CISP_POOL_TYPE_META);
	if (err)
		return (err);
	err = isp_cmd_ch_buffer_pool_config_set(isp, ch,
	    CISP_POOL_TYPE_META_CAPTURE);
	if (err)
		return (err);
	return (isp_cmd_ch_buffer_pool_config_set_rendered(isp, ch,
	    isp_num_rendered(isp), fmt->plane_size[0], fmt->strides[0],
	    fmt->plane_size[1], fmt->strides[1]));
}

/* Linux's isp_ch_configure_capture(), its H17 path */
static int
isp_ch_configure_capture(struct apple_isp *isp, u32 ch)
{
	struct isp_format *fmt = isp_get_format(isp, ch);
	int err;

	isp_cmd_flicker_sensor_set(isp, 0);

	/* Missing tuning is fine; a bad setfile or a failed upload is not. */
	err = isp_ch_load_setfile(isp, ch);
	if (err && err != -ENOENT)
		return (err);

	err = isp_cmd_ch_lpdp_hs_receiver_tuning_set(isp, ch, 1, 15);
	if (err)
		return (err);
	/* Mains frequency for auto exposure, 16.16 Hz (50 Hz; 60 Hz in the US) */
	err = isp_cmd_apple_ch_ae_flicker_freq_update_current_set(isp, ch,
	    60 << 16);
	if (err)
		return (err);

	err = isp_cmd_ch_sbs_enable(isp, ch, 1);
	if (err)
		return (err);
	err = isp_cmd_ch_camera_config_select(isp, ch, fmt->preset->index);
	if (err)
		return (err);
	err = isp_cmd_ch_buffer_recycle_mode_set(isp, ch,
	    CISP_BUFFER_RECYCLE_MODE_EMPTY_ONLY);
	if (err)
		return (err);
	err = isp_cmd_ch_buffer_recycle_start(isp, ch);
	if (err)
		return (err);
	err = isp_cmd_ch_crop_set(isp, ch, fmt->preset->crop_offset.x,
	    fmt->preset->crop_offset.y, fmt->preset->crop_size.x,
	    fmt->preset->crop_size.y);
	if (err)
		return (err);
	err = isp_cmd_ch_output_config_set(isp, ch, fmt->preset->output_dim.x,
	    fmt->preset->output_dim.y, fmt->strides, CISP_COLORSPACE_REC709,
	    CISP_OUTPUT_FORMAT_YUV_2PLANE);
	if (err)
		return (err);

	/*
	 * The H17 firmware takes the frame rate limits and the buffer pools
	 * before the preview stream is enabled, and its local raw buffers:
	 * without them it waits for a raw pool from the host.
	 */
	err = isp_ch_configure_frame_rate(isp, ch);
	if (err)
		return (err);
	err = isp_ch_configure_pools(isp, ch);
	if (err)
		return (err);
	err = isp_cmd_ch_local_raw_buffer_enable(isp, ch, 1);
	if (err)
		return (err);

	err = isp_cmd_ch_preview_stream_set(isp, ch, 1);
	if (err)
		return (err);
	err = isp_cmd_ch_cnr_start(isp, ch);
	if (err)
		return (err);
	err = isp_cmd_ch_mbnr_enable(isp, ch, 0, ISP_MBNR_MODE_ENABLE, 1);
	if (err)
		return (err);
	err = isp_cmd_apple_ch_ae_metering_mode_set(isp, ch, 3);
	if (err)
		return (err);
	err = isp_cmd_ch_ae_stability_set(isp, ch, 32);
	if (err)
		return (err);
	err = isp_cmd_ch_ae_stability_to_stable_set(isp, ch, 20);
	if (err)
		return (err);
	err = isp_cmd_ch_sif_pixel_format_set(isp, ch);
	if (err)
		return (err);
	err = isp_ch_configure_temporal_filter(isp, ch);
	if (err)
		return (err);
	return (isp_cmd_ch_master_slave_sync_mode_set(isp, ch, 0));
}

/* Boot the firmware and set the channel up. Called with video_lock held. */
int
apple_isp_start_camera(struct apple_isp *isp)
{
	struct isp_format *fmt = isp_get_current_format(isp);
	int err;

	err = apple_isp_firmware_boot(isp);
	if (err < 0) {
		dev_err(isp->dev, "failed to boot firmware: %d\n", err);
		return (err);
	}
	if (isp->num_fw_presets && fmt->preset->index >= isp->num_fw_presets) {
		dev_err(isp->dev, "preset config %u: firmware has %u\n",
		    fmt->preset->index, isp->num_fw_presets);
		apple_isp_firmware_halt(isp);
		return (-EINVAL);
	}
	err = isp_ch_configure_capture(isp, isp->current_ch);
	if (err) {
		dev_err(isp->dev, "failed to configure capture: %d\n", err);
		apple_isp_stop_camera(isp);
		return (err);
	}
	return (0);
}

/* The firmware starts again for every stream: stop it after one. */
void
apple_isp_stop_camera(struct apple_isp *isp)
{
	apple_isp_firmware_halt(isp);
}

/*
 * Capture watchdog. While it is not serviced, the ISP firmware replaces
 * every frame with a flat fill: a privacy interlock with the camera-in-use
 * indicator, which secure firmware we don't run would draw. The driver
 * services it whenever it captures, at the rate observed on the hardware.
 */
#define	ISP_WDT_PERIOD_HZ	30

static void
apple_isp_wdt_kick(struct apple_isp *isp)
{
	bus_write_4(isp->wdt, ISP_WDT_CLEAR, 0);
	bus_write_4(isp->wdt, ISP_WDT_RELOAD, ISP_WDT_RELOAD_VAL);
	bus_write_4(isp->wdt, ISP_WDT_KICK, 1);
}

static void
apple_isp_wdt_callout(void *arg)
{
	struct apple_isp *isp = arg;

	apple_isp_wdt_kick(isp);
	callout_reset_sbt(&isp->wdt_callout, SBT_1S / ISP_WDT_PERIOD_HZ, 0,
	    apple_isp_wdt_callout, isp, 0);
}

static void
apple_isp_wdt_start(struct apple_isp *isp)
{
	if (isp->wdt == NULL || isp->wdt_running)
		return;
	isp->wdt_running = true;
	apple_isp_wdt_kick(isp);
	callout_reset_sbt(&isp->wdt_callout, SBT_1S / ISP_WDT_PERIOD_HZ, 0,
	    apple_isp_wdt_callout, isp, 0);
}

void
apple_isp_wdt_stop(struct apple_isp *isp)
{
	if (!isp->wdt_running)
		return;
	callout_drain(&isp->wdt_callout);
	isp->wdt_running = false;
}

int
apple_isp_start_capture(struct apple_isp *isp)
{
	int err;

	/* Serviced from before the channel starts... */
	apple_isp_wdt_start(isp);
	err = isp_cmd_ch_start(isp, 0);
	if (err)
		apple_isp_wdt_stop(isp);
	return (err);
}

void
apple_isp_stop_capture(struct apple_isp *isp)
{
	if (isp->ipc_failed || isp->fw_state != ISP_FW_RUNNING)
		goto out;
	if (!isp_cmd_ch_stop(isp, 0))
		isp_cmd_ch_buffer_return(isp, isp->current_ch);
out:
	/* ... until its buffers are back. */
	apple_isp_wdt_stop(isp);
}
