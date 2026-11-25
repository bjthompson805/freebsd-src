// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */
/*
 * Apple ISP video device: /dev/video0, the V4L2 capture interface FreeBSD
 * applications use through v4l_compat (FreeBSD's ioctl encoding, the Linux
 * UAPI structures). MMAP streaming of NV12 frames.
 *
 * The buffer exchange with the firmware is aurora-silicon's Linux
 * isp-v4l2.c (isp_submit_buffers(), ipc_bt_handle()); the device itself is
 * FreeBSD's own, in place of videobuf2 and the V4L2 core.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/malloc.h>
#include <sys/poll.h>
#include <sys/proc.h>
#include <sys/selinfo.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <arm64/apple/apple_isp.h>
#include <arm64/apple/apple_isp_cmd.h>
#include <arm64/apple/v4l2/videodev2.h>

#define	ISP_BUFFER_TIMEOUT	1500	/* ms */
#define	ISP_MAX_VBUFS		8
#define	ISP_MIN_VBUFS		2
/* mmap offsets: one window per buffer, room for the largest frame */
#define	ISP_VBUF_SPAN		(16ul << 20)
#define	ISP_IPC_BUFEXC_STAT_SIZE	0x280
#define	ISP_IPC_BUFEXC_FLAG_RENDER	0x10000000
#define	ISP_IPC_BUFEXC_FLAG_COMMAND	0x30000000
#define	ISP_IPC_BUFEXC_FLAG_ACK		0x80000000

/* Capture rates offered to applications, in frames per second */
static const unsigned int isp_frame_rates[] = { 30, 25, 24, 20, 15 };

struct isp_buflist_buffer {
	u64 iovas[4];
	u32 flags[4];
	u32 num_planes;
	u32 pool_type;
	u64 tag;
} __packed;
static_assert(sizeof(struct isp_buflist_buffer) == ISP_BUFLIST_DESC_SIZE);

struct isp_buflist {
	u64 type;
	u64 num_buffers;
	struct isp_buflist_buffer buffers[];
} __packed;
static_assert(sizeof(struct isp_buflist) == ISP_BUFLIST_HDR_SIZE);

/* Buffer list tag of capture metadata buffers (the pool: META_CAPTURE) */
#define	ISP_BUFLIST_POOL_CAPTURE_META	2

static void
isp_wake_buffers(struct apple_isp *isp)
{
	mtx_assert(&isp->buf_mtx, MA_OWNED);
	wakeup(&isp->bufs_done);
	selwakeup(&isp->sel);
}

void
ipc_bt_complete(struct apple_isp *isp)
{
}

/* Only buffers the firmware holds may come back. */
static struct isp_surf *
isp_returned_surface(struct apple_isp *isp,
    const struct isp_buflist_buffer *desc, struct isp_vbuf **rendered)
{
	u32 iova = desc->iovas[0];
	struct isp_vbuf *vb;

	*rendered = NULL;
	if (desc->pool_type == CISP_POOL_TYPE_META) {
		for (int i = 0; i < ISP_MAX_BUFFERS; i++) {
			struct isp_surf *meta = isp->meta_surfs[i];

			if (meta && meta->submitted && (u32)meta->iova == iova)
				return (meta);
		}
	} else if (desc->pool_type == ISP_BUFLIST_POOL_CAPTURE_META) {
		for (int i = 0; i < ISP_CAPMETA_BUFFERS; i++) {
			struct isp_surf *meta = isp->capmeta_surfs[i];

			if (meta && meta->submitted && (u32)meta->iova == iova)
				return (meta);
		}
	} else {
		TAILQ_FOREACH(vb, &isp->bufs_submitted, link)
			if ((u32)vb->surf->iova == iova) {
				*rendered = vb;
				return (vb->surf);
			}
	}
	return (NULL);
}

int
ipc_bt_handle(struct apple_isp *isp, struct isp_channel *chan)
{
	struct isp_message *req = &chan->req, *rsp = &chan->rsp;
	struct isp_surf *surfaces[ISP_MAX_BUFFERS * 2 + ISP_CAPMETA_BUFFERS];
	struct isp_vbuf *rendered[nitems(surfaces)];
	struct isp_buflist_buffer *buffers;
	struct isp_buflist *bl;
	u64 count;
	int err = 0;

	if (READ_ONCE(isp->ipc_error))
		return (READ_ONCE(isp->ipc_error));

	if (!isp->bt_surf || req->arg1 < sizeof(*bl) ||
	    req->arg1 > isp->bt_surf->size) {
		dev_err(isp->dev, "%s: Bad length 0x%llx\n", chan->name,
		    req->arg1);
		return (-EIO);
	}
	bl = apple_isp_translate(isp, isp->bt_surf,
	    isp_fw_iova(isp, req->arg0), req->arg1);
	if (!bl)
		return (-EIO);

	count = READ_ONCE(bl->num_buffers);
	if (count > (req->arg1 - sizeof(*bl)) / sizeof(*buffers) ||
	    count > nitems(surfaces)) {
		dev_err(isp->dev, "%s: Bad count 0x%llx\n", chan->name, count);
		return (-EIO);
	}

	/* Validate and consume one snapshot of the firmware-written records. */
	buffers = malloc(count * sizeof(*buffers) + 1, M_APPLEISP, M_WAITOK);
	memcpy(buffers, bl->buffers, count * sizeof(*buffers));

	mtx_lock(&isp->buf_mtx);
	for (u64 i = 0; i < count; i++) {
		surfaces[i] = isp_returned_surface(isp, &buffers[i],
		    &rendered[i]);
		if (!surfaces[i]) {
			err = -EIO;
			goto unlock;
		}
		for (u64 j = 0; j < i; j++)
			if (surfaces[i] == surfaces[j]) {
				err = -EIO;
				goto unlock;
			}
	}

	/* No completion or ownership change until the whole list is valid. */
	for (u64 i = 0; i < count; i++) {
		struct isp_vbuf *vb = rendered[i];

		if (vb) {
			TAILQ_REMOVE(&isp->bufs_submitted, vb, link);
			microtime(&vb->timestamp);
			vb->sequence = isp->sequence++;
			vb->error = req->arg2 != ISP_IPC_BUFEXC_FLAG_RENDER;
			vb->state = ISP_VBUF_DONE;
			TAILQ_INSERT_TAIL(&isp->bufs_done, vb, link);
		} else
			surfaces[i]->submitted = false;
	}
	if (count)
		isp_wake_buffers(isp);
unlock:
	mtx_unlock(&isp->buf_mtx);
	free(buffers, M_APPLEISP);
	if (err) {
		dev_err(isp->dev, "%s: invalid buffer return\n", chan->name);
		return (err);
	}

	rsp->arg0 = req->arg0 | ISP_IPC_FLAG_ACK;
	rsp->arg1 = 0x0;
	rsp->arg2 = ISP_IPC_BUFEXC_FLAG_ACK;
	return (0);
}

/*
 * The firmware takes the metadata buffers before CH_START and the capture
 * metadata and frame buffers after it, so the batch before the start
 * (@pre_start) has only the former. Called with video_lock held.
 */
static int
isp_submit_buffers(struct apple_isp *isp, bool pre_start)
{
	struct isp_format *fmt = isp_get_current_format(isp);
	struct isp_channel *chan = isp->chan_bh;
	struct isp_message *req = &chan->req;
	struct isp_buflist *bl = isp->cmd_virt;
	struct isp_buflist_buffer *bufd = &bl->buffers[0];
	unsigned int num_rendered = 0, num_buffers = 0;
	struct isp_vbuf *vb;
	size_t offset;
	int err;

	err = isp_ipc_resync(isp);
	if (err)
		return (err);
	if (READ_ONCE(isp->ipc_error))
		return (READ_ONCE(isp->ipc_error));

	/* Clear what earlier commands and batches left in the reserved fields. */
	memset(bl, 0, ISP_CMD_AREA_SIZE(isp_num_capmeta(isp)));
	bl->type = 1;

	mtx_lock(&isp->buf_mtx);
	for (int i = 0; i < isp_num_meta(isp); i++) {
		struct isp_surf *meta = isp->meta_surfs[i];

		if (meta->submitted)
			continue;
		bufd->num_planes = 1;
		bufd->pool_type = 0;
		bufd->iovas[0] = meta->iova;
		bufd->flags[0] = 0x40000000;
		bufd++;
		num_buffers++;
		meta->submitted = true;
	}
	if (pre_start)
		goto send;

	for (int i = 0; i < isp_num_capmeta(isp); i++) {
		struct isp_surf *meta = isp->capmeta_surfs[i];

		if (meta->submitted)
			continue;
		bufd->num_planes = 1;
		bufd->pool_type = ISP_BUFLIST_POOL_CAPTURE_META;
		bufd->iovas[0] = meta->iova;
		bufd->flags[0] = 0x40000000;
		bufd++;
		num_buffers++;
		meta->submitted = true;
	}

	while (num_rendered < isp_num_rendered(isp) &&
	    (vb = TAILQ_FIRST(&isp->bufs_pending)) != NULL) {
		bufd->num_planes = fmt->num_planes;
		bufd->pool_type = isp->hw->scl1 ? CISP_POOL_TYPE_RENDERED_SCL1 :
		    CISP_POOL_TYPE_RENDERED;
		offset = 0;
		for (unsigned int j = 0; j < fmt->num_planes; j++) {
			bufd->iovas[j] = vb->surf->iova + offset;
			bufd->flags[j] = 0x40000000;
			offset += fmt->plane_size[j];
		}
		bufd++;
		num_buffers++;
		num_rendered++;
		/* Submitted before it is sent: it may come back at once. */
		TAILQ_REMOVE(&isp->bufs_pending, vb, link);
		vb->state = ISP_VBUF_SUBMITTED;
		TAILQ_INSERT_TAIL(&isp->bufs_submitted, vb, link);
	}
send:
	mtx_unlock(&isp->buf_mtx);

	bl->num_buffers = num_buffers;
	req->arg0 = isp->cmd_iova;
	req->arg1 = max_t(u64, ISP_IPC_BUFEXC_STAT_SIZE,
	    (uintptr_t)bufd - (uintptr_t)bl);
	req->arg2 = ISP_IPC_BUFEXC_FLAG_COMMAND;

	err = ipc_chan_send(isp, chan, ISP_BUFFER_TIMEOUT);
	if (err)
		dev_err(isp->dev, "%s: failed to send buffers: %d\n",
		    chan->name, err);
	return (err);
}

/* Give every buffer back to the application, as an error if it was out. */
static void
isp_release_buffers(struct apple_isp *isp)
{
	struct isp_vbuf *vb;

	mtx_lock(&isp->buf_mtx);
	while ((vb = TAILQ_FIRST(&isp->bufs_submitted)) != NULL) {
		TAILQ_REMOVE(&isp->bufs_submitted, vb, link);
		vb->state = ISP_VBUF_USER;
	}
	while ((vb = TAILQ_FIRST(&isp->bufs_pending)) != NULL) {
		TAILQ_REMOVE(&isp->bufs_pending, vb, link);
		vb->state = ISP_VBUF_USER;
	}
	while ((vb = TAILQ_FIRST(&isp->bufs_done)) != NULL) {
		TAILQ_REMOVE(&isp->bufs_done, vb, link);
		vb->state = ISP_VBUF_USER;
	}
	isp_wake_buffers(isp);
	mtx_unlock(&isp->buf_mtx);
}

/* Linux's apple_isp_start_streaming(). Called with video_lock held. */
static int
isp_start_streaming(struct apple_isp *isp)
{
	int err;

	isp->sequence = 0;
	err = apple_isp_start_camera(isp);
	if (err)
		return (err);

	/* The pools are new for every stream. */
	mtx_lock(&isp->buf_mtx);
	for (int i = 0; i < ISP_MAX_BUFFERS; i++)
		isp->meta_surfs[i]->submitted = false;
	for (int i = 0; i < isp_num_capmeta(isp); i++)
		isp->capmeta_surfs[i]->submitted = false;
	mtx_unlock(&isp->buf_mtx);

	err = isp_submit_buffers(isp, true);
	if (err) {
		dev_err(isp->dev, "failed to send initial batch: %d\n", err);
		goto stop_camera;
	}
	err = apple_isp_start_capture(isp);
	if (err) {
		dev_err(isp->dev, "failed to start capture: %d\n", err);
		goto stop_camera;
	}
	err = isp_submit_buffers(isp, false);
	if (err) {
		dev_err(isp->dev, "failed to send the capture batch: %d\n", err);
		apple_isp_stop_capture(isp);
		goto stop_camera;
	}
	isp->streaming = true;
	return (0);

stop_camera:
	apple_isp_stop_camera(isp);
	return (err);
}

static void
isp_stop_streaming(struct apple_isp *isp)
{
	if (!isp->streaming)
		return;
	isp->streaming = false;
	apple_isp_stop_capture(isp);
	apple_isp_stop_camera(isp);
	isp_release_buffers(isp);
}

static void
isp_free_vbufs(struct apple_isp *isp)
{
	for (unsigned int i = 0; i < isp->nbufs; i++)
		isp_free_surface(isp, isp->vbufs[i].surf);
	free(isp->vbufs, M_APPLEISP);
	isp->vbufs = NULL;
	isp->nbufs = 0;
}

static struct isp_preset *
isp_select_preset(struct apple_isp *isp, u32 width, u32 height)
{
	struct isp_preset *preset, *best = &isp->presets[0];
	int score, best_score = INT_MAX;

	if (width == 0 || height == 0)
		return (best);
	for (int i = 0; i < isp->num_presets; i++) {
		preset = &isp->presets[i];
		score = abs((int)preset->output_dim.x - (int)width) +
		    abs((int)preset->output_dim.y - (int)height);
		if (score < best_score) {
			best = preset;
			best_score = score;
		}
	}
	return (best);
}

static void
isp_fill_pix_format(struct isp_format *fmt, struct v4l2_pix_format *pix)
{
	memset(pix, 0, sizeof(*pix));
	pix->width = fmt->preset->output_dim.x;
	pix->height = fmt->preset->output_dim.y;
	pix->pixelformat = V4L2_PIX_FMT_NV12;
	pix->field = V4L2_FIELD_NONE;
	pix->bytesperline = fmt->strides[0];
	pix->sizeimage = fmt->total_size;
	pix->colorspace = V4L2_COLORSPACE_REC709;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_709;
	pix->quantization = V4L2_QUANTIZATION_LIM_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_709;
}

static void
isp_fill_buffer(struct apple_isp *isp, struct isp_vbuf *vb,
    struct v4l2_buffer *b)
{
	memset(b, 0, sizeof(*b));
	b->index = vb->index;
	b->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	b->memory = V4L2_MEMORY_MMAP;
	b->m.offset = vb->index * ISP_VBUF_SPAN;
	b->length = isp_get_current_format(isp)->total_size;
	b->field = V4L2_FIELD_NONE;
	b->flags = V4L2_BUF_FLAG_MAPPED | V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC |
	    V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
	switch (vb->state) {
	case ISP_VBUF_PENDING:
	case ISP_VBUF_SUBMITTED:
		b->flags |= V4L2_BUF_FLAG_QUEUED;
		break;
	case ISP_VBUF_DONE:
		b->flags |= V4L2_BUF_FLAG_DONE;
		break;
	case ISP_VBUF_USER:
		break;
	}
	if (vb->error)
		b->flags |= V4L2_BUF_FLAG_ERROR;
	b->bytesused = b->length;
	b->sequence = vb->sequence;
	b->timestamp = vb->timestamp;
}

static int
isp_reqbufs(struct apple_isp *isp, struct v4l2_requestbuffers *rb)
{
	struct isp_format *fmt = isp_get_current_format(isp);
	unsigned int n;

	if (rb->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    rb->memory != V4L2_MEMORY_MMAP)
		return (EINVAL);
	if (isp->streaming)
		return (EBUSY);
	isp_free_vbufs(isp);
	if (rb->count == 0)
		return (0);
	n = MIN(MAX(rb->count, ISP_MIN_VBUFS), ISP_MAX_VBUFS);
	isp->vbufs = malloc(n * sizeof(*isp->vbufs), M_APPLEISP,
	    M_WAITOK | M_ZERO);
	for (unsigned int i = 0; i < n; i++) {
		isp->vbufs[i].surf = isp_alloc_surface(isp, fmt->total_size,
		    false);
		if (isp->vbufs[i].surf == NULL) {
			isp->nbufs = i;
			isp_free_vbufs(isp);
			return (ENOMEM);
		}
		isp->vbufs[i].index = i;
		isp->vbufs[i].state = ISP_VBUF_USER;
		isp->nbufs = i + 1;
	}
	rb->count = n;
	rb->capabilities = V4L2_BUF_CAP_SUPPORTS_MMAP;
	return (0);
}

static int
isp_qbuf(struct apple_isp *isp, struct v4l2_buffer *b)
{
	struct isp_vbuf *vb;
	int err;

	if (b->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    b->memory != V4L2_MEMORY_MMAP || b->index >= isp->nbufs)
		return (EINVAL);
	vb = &isp->vbufs[b->index];
	mtx_lock(&isp->buf_mtx);
	if (vb->state != ISP_VBUF_USER) {
		mtx_unlock(&isp->buf_mtx);
		return (EINVAL);
	}
	vb->state = ISP_VBUF_PENDING;
	vb->error = false;
	TAILQ_INSERT_TAIL(&isp->bufs_pending, vb, link);
	mtx_unlock(&isp->buf_mtx);
	isp_fill_buffer(isp, vb, b);

	if (isp->streaming) {
		err = isp_submit_buffers(isp, false);
		if (err)
			return (EIO);
	}
	return (0);
}

static int
isp_dqbuf(struct apple_isp *isp, struct v4l2_buffer *b, int fflag)
{
	struct isp_vbuf *vb;
	int err;

	if (b->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
	    b->memory != V4L2_MEMORY_MMAP)
		return (EINVAL);
	mtx_lock(&isp->buf_mtx);
	while ((vb = TAILQ_FIRST(&isp->bufs_done)) == NULL) {
		if (!isp->streaming || READ_ONCE(isp->ipc_error)) {
			mtx_unlock(&isp->buf_mtx);
			return (EIO);
		}
		if (fflag & O_NONBLOCK) {
			mtx_unlock(&isp->buf_mtx);
			return (EAGAIN);
		}
		/* The ioctl holds video_lock; frames arrive on the taskqueue. */
		err = mtx_sleep(&isp->bufs_done, &isp->buf_mtx, PCATCH,
		    "ispdq", hz * 2);
		if (err == EWOULDBLOCK) {
			mtx_unlock(&isp->buf_mtx);
			dev_err(isp->dev, "no frame in 2 s\n");
			return (EIO);
		}
		if (err) {
			mtx_unlock(&isp->buf_mtx);
			return (err);
		}
	}
	TAILQ_REMOVE(&isp->bufs_done, vb, link);
	vb->state = ISP_VBUF_USER;
	mtx_unlock(&isp->buf_mtx);
	isp_fill_buffer(isp, vb, b);
	b->flags &= ~V4L2_BUF_FLAG_DONE;
	return (0);
}

static int
isp_frame_size_ok(struct apple_isp *isp, u32 index, struct isp_preset **pp)
{
	if (index >= (u32)isp->num_presets)
		return (EINVAL);
	*pp = &isp->presets[index];
	return (0);
}

static int
isp_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
    struct thread *td)
{
	struct apple_isp *isp = dev->si_drv1;
	struct isp_format *fmt = isp_get_current_format(isp);
	struct isp_preset *preset;
	int err = 0;

	sx_xlock(&isp->video_lock);
	switch (cmd) {
	case VIDIOC_QUERYCAP: {
		struct v4l2_capability *cap = (void *)data;

		memset(cap, 0, sizeof(*cap));
		strlcpy(cap->driver, "apple-isp", sizeof(cap->driver));
		strlcpy(cap->card, APPLE_ISP_CARD_NAME, sizeof(cap->card));
		strlcpy(cap->bus_info, "platform:apple-isp",
		    sizeof(cap->bus_info));
		cap->version = 0x070300;
		cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
		cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
		break;
	}
	case VIDIOC_ENUM_FMT: {
		struct v4l2_fmtdesc *f = (void *)data;

		if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE || f->index != 0) {
			err = EINVAL;
			break;
		}
		memset(f->description, 0, sizeof(f->description));
		f->flags = 0;
		f->pixelformat = V4L2_PIX_FMT_NV12;
		strlcpy(f->description, "Y/UV 4:2:0", sizeof(f->description));
		break;
	}
	case VIDIOC_ENUM_FRAMESIZES: {
		struct v4l2_frmsizeenum *fs = (void *)data;

		if (fs->pixel_format != V4L2_PIX_FMT_NV12 ||
		    (err = isp_frame_size_ok(isp, fs->index, &preset)) != 0) {
			err = EINVAL;
			break;
		}
		fs->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fs->discrete.width = preset->output_dim.x;
		fs->discrete.height = preset->output_dim.y;
		break;
	}
	case VIDIOC_ENUM_FRAMEINTERVALS: {
		struct v4l2_frmivalenum *fi = (void *)data;
		bool found = false;

		for (int i = 0; i < isp->num_presets; i++)
			found |= isp->presets[i].output_dim.x == fi->width &&
			    isp->presets[i].output_dim.y == fi->height;
		if (fi->pixel_format != V4L2_PIX_FMT_NV12 || !found ||
		    fi->index >= nitems(isp_frame_rates)) {
			err = EINVAL;
			break;
		}
		fi->type = V4L2_FRMIVAL_TYPE_DISCRETE;
		fi->discrete.numerator = 1;
		fi->discrete.denominator = isp_frame_rates[fi->index];
		break;
	}
	case VIDIOC_G_FMT: {
		struct v4l2_format *f = (void *)data;

		if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			err = EINVAL;
			break;
		}
		isp_fill_pix_format(fmt, &f->fmt.pix);
		break;
	}
	case VIDIOC_TRY_FMT:
	case VIDIOC_S_FMT: {
		struct v4l2_format *f = (void *)data;
		struct isp_format tmp;

		if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			err = EINVAL;
			break;
		}
		preset = isp_select_preset(isp, f->fmt.pix.width,
		    f->fmt.pix.height);
		if (cmd == VIDIOC_S_FMT) {
			if (isp->streaming || isp->nbufs) {
				err = EBUSY;
				break;
			}
			isp_set_preset(isp, fmt, preset);
			isp_fill_pix_format(fmt, &f->fmt.pix);
		} else {
			tmp = *fmt;
			isp_set_preset(isp, &tmp, preset);
			isp_fill_pix_format(&tmp, &f->fmt.pix);
		}
		break;
	}
	case VIDIOC_G_PARM:
	case VIDIOC_S_PARM: {
		struct v4l2_streamparm *p = (void *)data;
		struct v4l2_fract *tpf = &p->parm.capture.timeperframe;

		if (p->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			err = EINVAL;
			break;
		}
		if (cmd == VIDIOC_S_PARM && tpf->numerator && tpf->denominator) {
			unsigned int want = tpf->denominator / tpf->numerator;
			unsigned int best = isp_frame_rates[0];

			/* The fastest rate at or below the request */
			for (unsigned int i = 0; i < nitems(isp_frame_rates); i++)
				if (isp_frame_rates[i] <= want) {
					best = isp_frame_rates[i];
					break;
				}
			if (!isp->streaming)
				isp->frame_rate = best;
		}
		memset(&p->parm.capture, 0, sizeof(p->parm.capture));
		p->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		p->parm.capture.timeperframe.numerator = 1;
		p->parm.capture.timeperframe.denominator = isp->frame_rate;
		p->parm.capture.readbuffers = ISP_MIN_VBUFS;
		break;
	}
	case VIDIOC_ENUMINPUT: {
		struct v4l2_input *in = (void *)data;

		if (in->index != 0) {
			err = EINVAL;
			break;
		}
		memset(in, 0, sizeof(*in));
		strlcpy(in->name, "Camera", sizeof(in->name));
		in->type = V4L2_INPUT_TYPE_CAMERA;
		break;
	}
	case VIDIOC_G_INPUT:
		*(int *)data = 0;
		break;
	case VIDIOC_S_INPUT:
		if (*(int *)data != 0)
			err = EINVAL;
		break;
	case VIDIOC_REQBUFS:
		err = isp_reqbufs(isp, (void *)data);
		break;
	case VIDIOC_QUERYBUF: {
		struct v4l2_buffer *b = (void *)data;

		if (b->type != V4L2_BUF_TYPE_VIDEO_CAPTURE ||
		    b->index >= isp->nbufs) {
			err = EINVAL;
			break;
		}
		isp_fill_buffer(isp, &isp->vbufs[b->index], b);
		break;
	}
	case VIDIOC_QBUF:
		err = isp_qbuf(isp, (void *)data);
		break;
	case VIDIOC_DQBUF:
		/* Waiting for a frame must not hold off the other ioctls. */
		sx_xunlock(&isp->video_lock);
		return (isp_dqbuf(isp, (void *)data, fflag));
	case VIDIOC_STREAMON:
		if (*(int *)data != V4L2_BUF_TYPE_VIDEO_CAPTURE || !isp->nbufs) {
			err = EINVAL;
			break;
		}
		if (!isp->streaming && (err = -isp_start_streaming(isp)) != 0)
			err = EIO;
		break;
	case VIDIOC_STREAMOFF:
		if (*(int *)data != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
			err = EINVAL;
			break;
		}
		isp_stop_streaming(isp);
		break;
	default:
		err = ENOTTY;
		break;
	}
	sx_xunlock(&isp->video_lock);
	return (err);
}

static int
isp_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct apple_isp *isp = dev->si_drv1;
	int err = 0;

	sx_xlock(&isp->video_lock);
	if (isp->opened)
		err = EBUSY;
	else
		isp->opened = true;
	sx_xunlock(&isp->video_lock);
	return (err);
}

static int
isp_close(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
	struct apple_isp *isp = dev->si_drv1;

	sx_xlock(&isp->video_lock);
	isp_stop_streaming(isp);
	isp_free_vbufs(isp);
	isp->opened = false;
	sx_xunlock(&isp->video_lock);
	return (0);
}

static int
isp_poll(struct cdev *dev, int events, struct thread *td)
{
	struct apple_isp *isp = dev->si_drv1;
	int revents = 0;

	mtx_lock(&isp->buf_mtx);
	if (events & (POLLIN | POLLRDNORM)) {
		if (!TAILQ_EMPTY(&isp->bufs_done))
			revents |= events & (POLLIN | POLLRDNORM);
		else if (!isp->streaming)
			revents |= POLLERR;
		else
			selrecord(td, &isp->sel);
	}
	mtx_unlock(&isp->buf_mtx);
	return (revents);
}

/* Buffer i is at mmap offset i * ISP_VBUF_SPAN, write-combining. */
static int
isp_mmap(struct cdev *dev, vm_ooffset_t offset, vm_paddr_t *paddr, int nprot,
    vm_memattr_t *memattr)
{
	struct apple_isp *isp = dev->si_drv1;
	unsigned int i = offset / ISP_VBUF_SPAN;
	vm_ooffset_t off = offset % ISP_VBUF_SPAN;

	if (i >= isp->nbufs || off >= isp->vbufs[i].surf->size)
		return (EINVAL);
	*paddr = isp->vbufs[i].surf->pa + off;
	*memattr = VM_MEMATTR_WRITE_COMBINING;
	return (0);
}

static struct cdevsw isp_cdevsw = {
	.d_version =	D_VERSION,
	.d_name =	"video",
	.d_open =	isp_open,
	.d_close =	isp_close,
	.d_ioctl =	isp_ioctl,
	.d_poll =	isp_poll,
	.d_mmap =	isp_mmap,
};

int
apple_isp_setup_video(struct apple_isp *isp)
{
	struct make_dev_args args;
	int err;

	mtx_init(&isp->buf_mtx, "apple_isp bufs", NULL, MTX_DEF);
	TAILQ_INIT(&isp->bufs_pending);
	TAILQ_INIT(&isp->bufs_submitted);
	TAILQ_INIT(&isp->bufs_done);
	callout_init(&isp->wdt_callout, 1);
	isp->frame_rate = ISP_FRAME_RATE_DEFAULT;
	isp_set_preset(isp, isp_get_current_format(isp), &isp->presets[0]);

	for (int i = 0; i < ISP_MAX_BUFFERS; i++) {
		isp->meta_surfs[i] = isp_alloc_surface(isp, isp->hw->meta_size,
		    false);
		if (!isp->meta_surfs[i])
			return (-ENOMEM);
	}
	for (int i = 0; i < isp_num_capmeta(isp); i++) {
		isp->capmeta_surfs[i] = isp_alloc_surface(isp,
		    isp->hw->capture_meta_size, false);
		if (!isp->capmeta_surfs[i])
			return (-ENOMEM);
	}

	make_dev_args_init(&args);
	args.mda_devsw = &isp_cdevsw;
	args.mda_uid = UID_ROOT;
	args.mda_gid = GID_VIDEO;
	args.mda_mode = 0660;
	args.mda_si_drv1 = isp;
	err = make_dev_s(&args, &isp->cdev, "video%d",
	    device_get_unit(isp->dev));
	if (err)
		return (-err);
	device_printf(isp->dev, "/dev/%s: %s\n", devtoname(isp->cdev),
	    APPLE_ISP_CARD_NAME);
	return (0);
}

void
apple_isp_remove_video(struct apple_isp *isp)
{
	if (isp->cdev != NULL) {
		destroy_dev(isp->cdev);
		isp->cdev = NULL;
	}
	if (mtx_initialized(&isp->buf_mtx)) {
		sx_xlock(&isp->video_lock);
		isp_stop_streaming(isp);
		isp_free_vbufs(isp);
		sx_xunlock(&isp->video_lock);
		for (int i = 0; i < ISP_MAX_BUFFERS; i++)
			if (isp->meta_surfs[i])
				isp_free_surface(isp, isp->meta_surfs[i]);
		for (int i = 0; i < ISP_CAPMETA_BUFFERS; i++)
			if (isp->capmeta_surfs[i])
				isp_free_surface(isp, isp->capmeta_surfs[i]);
		mtx_destroy(&isp->buf_mtx);
	}
}
