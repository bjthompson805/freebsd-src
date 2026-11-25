// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */
/*
 * Apple ISP camera detection: aurora-silicon's Linux isp-cam.c, the part
 * that runs when the driver attaches. Streaming is still to be ported.
 */

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

/* Buffer returns come with streaming (Linux's isp-v4l2.c). */
int
ipc_bt_handle(struct apple_isp *isp, struct isp_channel *chan)
{
	dev_err(isp->dev, "%s: buffer return without a stream\n", chan->name);
	return (-EIO);
}

void
ipc_bt_complete(struct apple_isp *isp)
{
}
