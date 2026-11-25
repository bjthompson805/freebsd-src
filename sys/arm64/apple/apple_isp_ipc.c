// SPDX-License-Identifier: GPL-2.0-only
/* Copyright 2023 Eileen Yoon <eyn@gmx.com> */
/*
 * Apple ISP IPC rings: aurora-silicon's Linux isp-ipc.c, without its P010
 * (T6040) paths.
 */

#include <arm64/apple/apple_isp.h>

#define	ISP_IPC_FLAG_TERMINAL_ACK	0x3
/* longer firmware log lines are cut */
#define	ISP_IPC_TERMINAL_MAX_LEN	512
/* how much longer a timed-out request may take to be answered, in ms */
#define	ISP_IPC_RESYNC_TIMEOUT		1000

static inline void *
chan_msg_virt(struct isp_channel *chan, u32 index)
{
	return ((char *)chan->virt + index * ISP_IPC_MESSAGE_SIZE);
}

static inline void
chan_read_msg(struct apple_isp *isp, struct isp_channel *chan,
    struct isp_message *msg)
{
	memcpy(msg, chan_msg_virt(chan, chan->cursor), sizeof(*msg));
}

static inline void
chan_write_msg(struct apple_isp *isp, struct isp_channel *chan,
    struct isp_message *msg)
{
	volatile u64 *p0 = chan_msg_virt(chan, chan->cursor);

	memcpy(__DEVOLATILE(u64 *, p0) + 1, &msg->arg1, sizeof(*msg) - 8);

	/* Make sure we write arg0 last, since that indicates message validity. */
	dma_wmb();
	*p0 = msg->arg0;
	dma_wmb();
}

static inline void
chan_update_cursor(struct isp_channel *chan)
{
	if (chan->cursor >= chan->num - 1)
		chan->cursor = 0;
	else
		chan->cursor += 1;
}

static int
chan_handle_once(struct apple_isp *isp, struct isp_channel *chan)
{
	int err;

	sx_assert(&chan->lock, SA_XLOCKED);

	err = chan->ops->handle(isp, chan);
	if (err < 0) {
		dev_err(isp->dev, "%s: handler failed: %d\n", chan->name, err);
		return (err);
	}

	chan_write_msg(isp, chan, &chan->rsp);

	isp_mbox2_write32(isp, ISP_MBOX2_IRQ_DOORBELL, chan->doorbell);

	chan_update_cursor(chan);
	if (chan->ops->complete)
		chan->ops->complete(isp);

	return (0);
}

static inline bool
chan_rx_done(struct apple_isp *isp, struct isp_channel *chan)
{
	return ((chan->req.arg0 & 0xf) == ISP_IPC_FLAG_ACK ||
	    (chan->req.arg0 & 0xf) == ISP_IPC_FLAG_TERMINAL_ACK);
}

int
ipc_chan_handle(struct apple_isp *isp, struct isp_channel *chan)
{
	int err = -EAGAIN;

	if (chan == NULL)
		return (0);
	sx_xlock(&chan->lock);
	/* One ring per turn, even if the producer refills acknowledged slots. */
	for (u32 i = 0; i < chan->num; i++) {
		if (READ_ONCE(isp->ipc_error)) {
			err = READ_ONCE(isp->ipc_error);
			break;
		}
		chan_read_msg(isp, chan, &chan->req);
		if (chan_rx_done(isp, chan)) {
			err = 0;
			break;
		}
		err = chan_handle_once(isp, chan);
		if (err < 0) {
			atomic_cmpset_int((volatile u_int *)&isp->ipc_error, 0,
			    err);
			isp_wake(isp);
			break;
		}
		err = -EAGAIN;
	}
	sx_xunlock(&chan->lock);

	return (err);
}

static inline bool
chan_tx_done(struct apple_isp *isp, struct isp_channel *chan)
{
	dma_rmb();

	chan_read_msg(isp, chan, &chan->rsp);
	if (isp_fw_iova(isp, chan->rsp.arg0) ==
	    isp_fw_iova(isp, chan->req.arg0 | ISP_IPC_FLAG_ACK)) {
		chan_update_cursor(chan);
		return (true);
	}
	return (false);
}

/*
 * wait_event_timeout(isp->wait, cond, ms): the interrupt task wakes &wait
 * after each turn over the rings. 0 on a timeout.
 */
static int
isp_wait_tx(struct apple_isp *isp, struct isp_channel *chan, bool on_error,
    unsigned long ms)
{
	sbintime_t end = getsbinuptime() + mstosbt(ms);

	mtx_lock(&isp->wait_mtx);
	for (;;) {
		if ((on_error && READ_ONCE(isp->ipc_error)) ||
		    chan_tx_done(isp, chan))
			break;
		if (getsbinuptime() >= end) {
			mtx_unlock(&isp->wait_mtx);
			return (0);
		}
		/*
		 * Also poll: a missed interrupt costs a tick, not the timeout.
		 * The firmware may be waiting on a SHAREDMALLOC request before
		 * it answers, so run the rings too.
		 */
		taskqueue_enqueue(isp->tq, &isp->irq_task);
		mtx_sleep(&isp->wait, &isp->wait_mtx, 0, "ispipc",
		    MAX(1, hz / 100));
	}
	mtx_unlock(&isp->wait_mtx);
	return (1);
}

/*
 * A request that timed out still owns its slot, and the firmware may answer
 * it late. On T8142 a timeout poisons the session (ipc_error), so nothing
 * is resent before the firmware is stopped; this only reports that state.
 */
int
isp_ipc_resync(struct apple_isp *isp)
{
	if (isp->ipc_failed)
		return (-EIO);
	return (0);
}

int
ipc_chan_send(struct apple_isp *isp, struct isp_channel *chan,
    unsigned long timeout)
{
	int err = READ_ONCE(isp->ipc_error);

	if (err)
		return (err);

	err = isp_ipc_resync(isp);
	if (err)
		return (err);
	chan_write_msg(isp, chan, &chan->req);
	dma_wmb();

	isp_mbox2_write32(isp, ISP_MBOX2_IRQ_DOORBELL, chan->doorbell);

	if (!timeout)
		return (0);

	if (!isp_wait_tx(isp, chan, true, timeout)) {
		dev_err(isp->dev,
		    "%s: timed out on request [0x%jx, 0x%jx, 0x%jx]\n",
		    chan->name, (uintmax_t)chan->req.arg0,
		    (uintmax_t)chan->req.arg1, (uintmax_t)chan->req.arg2);
		atomic_cmpset_int((volatile u_int *)&isp->ipc_error, 0,
		    -ETIMEDOUT);
		isp_wake(isp);
		return (READ_ONCE(isp->ipc_error));
	}

	if (READ_ONCE(isp->ipc_error))
		return (READ_ONCE(isp->ipc_error));

	isp_dbg(isp, "%s: request success\n", chan->name);

	return (0);
}

int
ipc_tm_handle(struct apple_isp *isp, struct isp_channel *chan)
{
	struct isp_message *req = &chan->req, *rsp = &chan->rsp;
	dma_addr_t iova =
	    isp_fw_iova(isp, req->arg0 & ~(u64)ISP_IPC_FLAG_TERMINAL_ACK);
	size_t size = min_t(u64, req->arg1, ISP_IPC_TERMINAL_MAX_LEN);
	const char *line;

	/* Firmware log output, with hw.apple_isp.debug */
	if (iova && size && isp->log_surf) {
		line = apple_isp_translate(isp, isp->log_surf, iova, size);
		if (line)
			dev_dbg(isp->dev, "ISPASC: %.*s", (int)size, line);
	}

	rsp->arg0 = ISP_IPC_FLAG_ACK;
	rsp->arg1 = 0x0;
	rsp->arg2 = 0x0;

	return (0);
}

/*
 * The firmware frees a SHAREDMALLOC surface by sending its iova. It does not
 * free every surface it allocated, and may still access a surface after
 * freeing it, so the surface is only moved to the released list here; it is
 * freed once the stream has stopped, see isp_free_released_surfaces(), and
 * what the firmware never frees is freed when the firmware stops.
 */
static void
ipc_sm_release(struct apple_isp *isp, dma_addr_t iova)
{
	struct isp_surf *surf, *found = NULL;

	sx_xlock(&isp->gc_lock);
	TAILQ_FOREACH(surf, &isp->gc, head) {
		if (surf->iova == iova) {
			found = surf;
			TAILQ_REMOVE(&isp->gc, surf, head);
			TAILQ_INSERT_TAIL(&isp->gc_released, surf, head);
			break;
		}
	}
	sx_xunlock(&isp->gc_lock);

	if (!found) {
		dev_warn(isp->dev,
		    "firmware freed unknown shared surface 0x%jx\n",
		    (uintmax_t)iova);
		return;
	}

	/* The terminal and buffer return handlers run in this task. */
	if (found == isp->log_surf)
		isp->log_surf = NULL;
	if (found == isp->bt_surf)
		isp->bt_surf = NULL;

	isp_dbg(isp, "shared surface released: iova 0x%jx size 0x%jx\n",
	    (uintmax_t)found->iova, (uintmax_t)found->size);
}

int
ipc_sm_handle(struct apple_isp *isp, struct isp_channel *chan)
{
	struct isp_message *req = &chan->req, *rsp = &chan->rsp;

	if (req->arg0 == 0x0) {
		struct isp_surf *surf;

		surf = isp_alloc_surface_gc(isp, req->arg1);
		if (!surf) {
			dev_err(isp->dev,
			    "failed to alloc requested size 0x%jx\n",
			    (uintmax_t)req->arg1);
			return (-ENOMEM);
		}
		surf->type = req->arg2;

		rsp->arg0 = surf->iova | ISP_IPC_FLAG_ACK;
		rsp->arg1 = 0x0;
		rsp->arg2 = 0x0; /* macOS uses this to index surfaces */

		switch (surf->type) {
		case 0x4c4f47: /* "LOG" */
			isp->log_surf = surf;
			break;
		case 0x4d495343: /* "MISC" */
			/* Hacky... maybe there's a better way to identify this surface? */
			if (surf->size == 0xc000)
				isp->bt_surf = surf;
			break;
		}
		/* Every surface is mapped on the host (isp_alloc_surface()). */
	} else {
		ipc_sm_release(isp, isp_fw_iova(isp, req->arg0));

		rsp->arg0 = req->arg0 | ISP_IPC_FLAG_ACK;
		rsp->arg1 = 0x0;
		rsp->arg2 = 0x0;
	}

	return (0);
}
