/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple RTKit coprocessor protocol over an ASC mailbox, polled.
 *
 * Boot: start the IOP if it is stopped, wake it (IOP power INIT), answer
 * HELLO and the endpoint map, start the system endpoints, then wait for
 * the IOP power ack. Afterwards apple_rtkit_poll() services the system
 * endpoints (log buffers, syslog acks, ioreport echoes) and hands
 * application endpoint messages to the client.
 *
 * Buffers the IOP asks for are allocated physically contiguous and passed
 * by physical address: the client's DART (identity) or SART (buf_map)
 * makes that address reachable.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/hash.h>
#include <sys/kernel.h>
#include <sys/malloc.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/atomic.h>
#include <machine/cpufunc.h>

#include <arm64/apple/apple_rtkit.h>

#define	ASC_CPU_CONTROL			0x44
#define	ASC_CPU_CONTROL_RUN		0x10

#define	MBOX_A2I_CONTROL		0x110
#define	MBOX_I2A_CONTROL		0x114
#define	MBOX_CONTROL_FULL		(1u << 16)
#define	MBOX_CONTROL_EMPTY		(1u << 17)
#define	MBOX_A2I_SEND0			0x800
#define	MBOX_A2I_SEND1			0x808
#define	MBOX_I2A_RECV0			0x830
#define	MBOX_I2A_RECV1			0x838

#define	EP_MGMT				0
#define	EP_CRASHLOG			1
#define	EP_SYSLOG			2
#define	EP_DEBUG			3
#define	EP_IOREPORT			4
#define	EP_OSLOG			8
#define	EP_TRACEKIT			0xa

#define	MSG_TYPE(m)			(((m) >> 52) & 0xff)
#define	MSG_TYPE_SET(t)			((uint64_t)(t) << 52)

#define	MGMT_HELLO			1
#define	MGMT_HELLO_ACK			2
#define	MGMT_START_EP			5
#define	MGMT_IOP_PWR			6
#define	MGMT_IOP_PWR_ACK		7
#define	MGMT_EPMAP			8
#define	MGMT_AP_PWR			0xb
#define	MGMT_EPMAP_DONE			(1ull << 51)
#define	MGMT_EPMAP_MORE			1ull

#define	PWR_SLEEP			0x01	/* can be restarted */
#define	PWR_QUIESCED			0x10	/* running, no messages */
#define	PWR_ON				0x20
#define	PWR_INIT			0x220

#define	MSG_BUFFER_REQUEST		1
#define	BUFREQ_SIZE(m)			(((m) >> 44) & 0xff)	/* 4K pages */
#define	BUFREQ_ADDR(m)			((m) & ((1ull << 42) - 1))
#define	MSG_SYSLOG_LOG			5
#define	MSG_SYSLOG_INIT			8

/* OSLog uses its own layout: type in [63:56], size in bytes, IOVA >> 12. */
#define	OSLOG_TYPE(m)			((m) >> 56)
#define	OSLOG_BUFFER_REQUEST		1
#define	OSLOG_SIZE(m)			(((m) >> 36) & 0xfffff)
#define	OSLOG_IOVA(m)			(((m) & ((1ull << 36) - 1)) << 12)

#define	RTKIT_MIN_VERSION		11
#define	RTKIT_MAX_VERSION		12

struct rtkit_buffer {
	void		*va;
	vm_paddr_t	pa;
	uint64_t	iova;		/* what the IOP was given */
	size_t		size;
	bool		iop_owned;	/* the IOP's buffer, mapped as device */
};

struct rtkit_syslog_entry {
	uint32_t	hdr;
	uint32_t	unk;
	char		context[24];
	char		msg[];
};

struct apple_rtkit {
	device_t		dev;
	vm_offset_t		asc;
	vm_offset_t		mbox;
	const struct apple_rtkit_ops *ops;
	void			*arg;
	u_int			flags;

	uint32_t		iop_power;
	uint32_t		ap_power;
	bool			crashed;
	bool			syslog_on;
	struct rtkit_buffer	crashlog, syslog, ioreport, oslog;
	uint32_t		syslog_count, syslog_size;
	uint32_t		syslog_last, syslog_repeats;
	uint32_t		epmap[8];	/* endpoint map, 32 per word */
};

#define	ASC_RD4(r, o)		(*(volatile uint32_t *)((r)->asc + (o)))
#define	ASC_WR4(r, o, v)	(*(volatile uint32_t *)((r)->asc + (o)) = (v))
#define	MBOX_RD4(r, o)		(*(volatile uint32_t *)((r)->mbox + (o)))
#define	MBOX_RD8(r, o)		(*(volatile uint64_t *)((r)->mbox + (o)))
#define	MBOX_WR8(r, o, v)	(*(volatile uint64_t *)((r)->mbox + (o)) = (v))

struct apple_rtkit *
apple_rtkit_create(device_t dev, vm_offset_t asc, vm_offset_t mbox,
    const struct apple_rtkit_ops *ops, void *arg, u_int flags)
{
	struct apple_rtkit *rtk;

	rtk = malloc(sizeof(*rtk), M_DEVBUF, M_WAITOK | M_ZERO);
	rtk->dev = dev;
	rtk->asc = asc;
	rtk->mbox = mbox;
	rtk->ops = ops;
	rtk->arg = arg;
	rtk->flags = flags;
	rtk->syslog_on = true;
	return (rtk);
}

bool
apple_rtkit_crashed(struct apple_rtkit *rtk)
{
	return (rtk->crashed);
}

void
apple_rtkit_set_syslog(struct apple_rtkit *rtk, bool on)
{
	rtk->syslog_on = on;
}

/* Whether the IOP's endpoint map (from apple_rtkit_boot) names ep. */
bool
apple_rtkit_has_endpoint(struct apple_rtkit *rtk, uint32_t ep)
{
	if (ep >= 32 * nitems(rtk->epmap))
		return (false);
	return ((rtk->epmap[ep / 32] & (1u << (ep % 32))) != 0);
}

int
apple_rtkit_send(struct apple_rtkit *rtk, uint32_t ep, uint64_t msg)
{
	int i;

	for (i = 0; MBOX_RD4(rtk, MBOX_A2I_CONTROL) & MBOX_CONTROL_FULL; i++) {
		if (i == 200000) {
			device_printf(rtk->dev, "mailbox full\n");
			return (ETIMEDOUT);
		}
		DELAY(1);
	}
	wmb();
	MBOX_WR8(rtk, MBOX_A2I_SEND0, msg);
	MBOX_WR8(rtk, MBOX_A2I_SEND1, ep);
	return (0);
}

static bool
rtkit_recv(struct apple_rtkit *rtk, uint32_t *ep, uint64_t *msg)
{
	if (MBOX_RD4(rtk, MBOX_I2A_CONTROL) & MBOX_CONTROL_EMPTY)
		return (false);
	*msg = MBOX_RD8(rtk, MBOX_I2A_RECV0);
	*ep = (uint32_t)MBOX_RD8(rtk, MBOX_I2A_RECV1) & 0xff;
	rmb();
	return (true);
}

static int
rtkit_recv_timeout(struct apple_rtkit *rtk, uint32_t *ep, uint64_t *msg,
    int usec)
{
	for (; usec > 0; usec -= 10) {
		if (rtkit_recv(rtk, ep, msg))
			return (0);
		DELAY(10);
	}
	return (ETIMEDOUT);
}

int
apple_rtkit_start_ep(struct apple_rtkit *rtk, uint32_t ep)
{
	return (apple_rtkit_send(rtk, EP_MGMT, MSG_TYPE_SET(MGMT_START_EP) |
	    (uint64_t)ep << 32 | 2));
}

/* Allocate (or map the IOP's own) buffer; returns its physical address. */
static int
rtkit_buffer(struct apple_rtkit *rtk, uint32_t ep, struct rtkit_buffer *buf,
    size_t size, uint64_t addr)
{
	vm_paddr_t pa;
	int error;

	if (addr != 0) {
		/*
		 * The IOP brought its own buffer (SRAM or filtered DRAM) at
		 * an address of its choosing. A repeat of the same request
		 * keeps it; a different one would retire a live buffer.
		 */
		if (buf->va != NULL) {
			if (buf->iova == addr && buf->size == size)
				return (0);
			device_printf(rtk->dev, "ep %u: buffer request for "
			    "%#zx at %#jx while %#zx at %#jx is live\n", ep,
			    size, (uintmax_t)addr, buf->size,
			    (uintmax_t)buf->iova);
			return (EBUSY);
		}
		pa = addr;
		if (rtk->ops->buf_lookup != NULL) {
			error = rtk->ops->buf_lookup(rtk->arg, addr, size, &pa);
			if (error != 0) {
				device_printf(rtk->dev, "ep %u: can't find "
				    "buffer at %#jx: %d\n", ep,
				    (uintmax_t)addr, error);
				return (error);
			}
		}
		buf->size = size;
		buf->iova = addr;
		buf->pa = pa;
		buf->va = pmap_mapdev(pa, size);
		buf->iop_owned = true;
		return (0);
	}
	if (buf->size != 0 && buf->size < roundup2(size, 16384)) {
		device_printf(rtk->dev, "ep %u: buffer request for %#zx while "
		    "%#zx is live\n", ep, size, buf->size);
		return (EBUSY);
	}
	if (buf->va == NULL) {
		buf->size = roundup2(size, 16384);
		buf->va = contigmalloc(buf->size, M_DEVBUF, M_ZERO | M_NOWAIT,
		    0, ~(vm_paddr_t)0, 16384, 0);
		if (buf->va == NULL)
			return (ENOMEM);
		buf->pa = buf->iova = pmap_kextract((vm_offset_t)buf->va);
		if (rtk->ops->buf_map != NULL) {
			error = rtk->ops->buf_map(rtk->arg, buf->pa, buf->size,
			    &buf->iova);
			if (error != 0)
				return (error);
		}
	}
	if (bootverbose)
		device_printf(rtk->dev, "ep %u buffer %#zx at %#jx iova %#jx\n",
		    ep, buf->size, (uintmax_t)buf->pa, (uintmax_t)buf->iova);
	return (0);
}

static int
rtkit_buffer_request(struct apple_rtkit *rtk, uint32_t ep, uint64_t msg,
    struct rtkit_buffer *buf)
{
	size_t npages = BUFREQ_SIZE(msg);
	uint64_t addr = BUFREQ_ADDR(msg);
	int error;

	error = rtkit_buffer(rtk, ep, buf, npages << 12, addr);
	if (error != 0 || addr != 0)
		return (error);
	return (apple_rtkit_send(rtk, ep, MSG_TYPE_SET(MSG_BUFFER_REQUEST) |
	    (uint64_t)npages << 44 | buf->iova));
}

static int
rtkit_oslog_request(struct apple_rtkit *rtk, uint64_t msg)
{
	size_t size = OSLOG_SIZE(msg);
	uint64_t addr = OSLOG_IOVA(msg);
	int error;

	error = rtkit_buffer(rtk, EP_OSLOG, &rtk->oslog, size, addr);
	if (error != 0 || addr != 0)
		return (error);
	return (apple_rtkit_send(rtk, EP_OSLOG,
	    (uint64_t)OSLOG_BUFFER_REQUEST << 56 | (uint64_t)size << 36 |
	    rtk->oslog.iova >> 12));
}

static void
rtkit_syslog(struct apple_rtkit *rtk, uint64_t msg)
{
	struct rtkit_syslog_entry *e;
	size_t idx = msg & 0xff, len, stride, i;
	uint32_t ebuf[(sizeof(*e) + 0xff + 3) / 4 + 1];
	const volatile uint32_t *src;
	uint32_t hash;

	stride = rtk->syslog_size + sizeof(*e);
	if (!rtk->syslog_on || rtk->syslog.va == NULL ||
	    idx >= rtk->syslog_count || (idx + 1) * stride > rtk->syslog.size)
		return;
	/*
	 * Copy the entry out in aligned words: the buffer may be the IOP's,
	 * mapped as device memory, where string routines can fault.
	 */
	src = (const volatile uint32_t *)((char *)rtk->syslog.va +
	    rounddown2(idx * stride, 4));
	for (i = 0; i < howmany(stride + 3, 4) && i < nitems(ebuf); i++)
		ebuf[i] = src[i];
	e = (struct rtkit_syslog_entry *)((char *)ebuf + (idx * stride) % 4);
	len = strnlen(e->msg, rtk->syslog_size);
	if (len > 0 && e->msg[len - 1] == '\n')
		len--;

	/* The firmware repeats some lines many times; print a line once. */
	hash = murmur3_32_hash(e->msg, len, 0);
	if (hash == rtk->syslog_last) {
		rtk->syslog_repeats++;
		return;
	}
	if (rtk->syslog_repeats > 0)
		device_printf(rtk->dev, "fw: (last line repeated %u times)\n",
		    rtk->syslog_repeats);
	rtk->syslog_last = hash;
	rtk->syslog_repeats = 0;
	device_printf(rtk->dev, "fw: [%.24s] %.*s\n", e->context, (int)len,
	    e->msg);
}

/* Print the readable text in a crash log (panic string, task names). */
static void
rtkit_dump_strings(struct apple_rtkit *rtk, struct rtkit_buffer *buf)
{
	const char *p = buf->va;
	size_t i, start;

	for (i = start = 0; i <= buf->size; i++) {
		if (i < buf->size && p[i] >= 0x20 && p[i] < 0x7f)
			continue;
		if (i - start >= 6)
			device_printf(rtk->dev, "crashlog +%#zx: %.*s\n",
			    start, (int)(i - start), p + start);
		start = i + 1;
	}
}

/* Handle one message. */
static int
rtkit_handle(struct apple_rtkit *rtk, uint32_t ep, uint64_t msg)
{
	uint32_t type = MSG_TYPE(msg);

	switch (ep) {
	case EP_MGMT:
		if (type == MGMT_IOP_PWR_ACK)
			rtk->iop_power = msg & 0xffff;
		else if (type == MGMT_AP_PWR)
			rtk->ap_power = msg & 0xffff;
		else
			device_printf(rtk->dev, "mgmt message %#jx\n",
			    (uintmax_t)msg);
		return (0);
	case EP_CRASHLOG:
		if (type != MSG_BUFFER_REQUEST)
			break;
		if (rtk->crashlog.va != NULL) {
			rtk->crashed = true;
			device_printf(rtk->dev, "RTKit firmware crashed\n");
			rtkit_dump_strings(rtk, &rtk->crashlog);
			/* XXX bring-up: raw dump for offline decoding */
			for (size_t o = 0; o < MIN(rtk->crashlog.size, 0x2000);
			    o += 32)
				printf("CRASHHEX %04zx %32D\n", o,
				    (uint8_t *)rtk->crashlog.va + o, "");
			return (EIO);
		}
		return (rtkit_buffer_request(rtk, ep, msg, &rtk->crashlog));
	case EP_SYSLOG:
		if (type == MSG_BUFFER_REQUEST)
			return (rtkit_buffer_request(rtk, ep, msg, &rtk->syslog));
		if (type == MSG_SYSLOG_INIT) {
			rtk->syslog_count = msg & 0xff;
			rtk->syslog_size = (msg >> 24) & 0xff;
			return (0);
		}
		if (type == MSG_SYSLOG_LOG) {
			rtkit_syslog(rtk, msg);
			return (apple_rtkit_send(rtk, ep, msg));
		}
		break;
	case EP_IOREPORT:
		if (type == MSG_BUFFER_REQUEST)
			return (rtkit_buffer_request(rtk, ep, msg,
			    &rtk->ioreport));
		if (type == 0x8 || type == 0xc)		/* must be echoed */
			return (apple_rtkit_send(rtk, ep, msg));
		break;
	case EP_OSLOG:
		if (OSLOG_TYPE(msg) == OSLOG_BUFFER_REQUEST)
			return (rtkit_oslog_request(rtk, msg));
		return (0);	/* log notifications: nothing to do */
	default:
		if (ep >= 0x20 && rtk->ops->recv != NULL) {
			rtk->ops->recv(rtk->arg, ep, msg);
			return (0);
		}
		break;
	}
	device_printf(rtk->dev, "unhandled message ep %u: %#jx\n", ep,
	    (uintmax_t)msg);
	return (0);
}

void
apple_rtkit_poll(struct apple_rtkit *rtk)
{
	uint64_t msg;
	uint32_t ep;

	while (!rtk->crashed && rtkit_recv(rtk, &ep, &msg))
		rtkit_handle(rtk, ep, msg);
}

/*
 * Take over a session a previous stage (U-Boot's keyboard) left running, as
 * aurora's rtkit-helper "mtp-live-v1" handoff: the firmware can't be
 * restarted once it has been quiesced, so its endpoints stay started and its
 * buffers stay where the stage listed them. No HELLO, no power change; the
 * syslog's geometry was announced to the stage, so its lines aren't printed
 * (they are still acknowledged).
 */
int
apple_rtkit_adopt(struct apple_rtkit *rtk,
    const struct apple_rtkit_inherited *bufs, int n)
{
	struct rtkit_buffer *buf;
	vm_paddr_t pa;
	int i;

	for (i = 0; i < n; i++) {
		switch (bufs[i].ep) {
		case EP_CRASHLOG:
			buf = &rtk->crashlog;
			break;
		case EP_SYSLOG:
			buf = &rtk->syslog;
			break;
		case EP_IOREPORT:
			buf = &rtk->ioreport;
			break;
		case EP_OSLOG:
			buf = &rtk->oslog;
			break;
		default:
			return (EINVAL);
		}
		pa = bufs[i].iova;
		if (rtk->ops->buf_lookup != NULL &&
		    rtk->ops->buf_lookup(rtk->arg, bufs[i].iova, bufs[i].size,
		    &pa) != 0)
			pa = bufs[i].iova;	/* DART bypass: IOVA == PA */
		buf->size = bufs[i].size;
		buf->iova = bufs[i].iova;
		buf->pa = pa;
		buf->va = pmap_mapdev(pa, buf->size);
		buf->iop_owned = true;
	}
	rtk->iop_power = PWR_ON;
	return (0);
}

int
apple_rtkit_boot(struct apple_rtkit *rtk)
{
	uint64_t msg, eps = 0;
	uint32_t ep, ver, base;
	int error, i;

	/*
	 * A previous stage's session (U-Boot's keyboard) can leave its last
	 * messages, such as its power-state ack, in the mailbox: drop them.
	 */
	for (i = 0; i < 64 && rtkit_recv(rtk, &ep, &msg); i++)
		;
	if (i != 0)
		device_printf(rtk->dev, "dropped %d stale mailbox messages\n", i);

	/* Start the IOP if a previous stage stopped it, then wake it. */
	if ((ASC_RD4(rtk, ASC_CPU_CONTROL) & ASC_CPU_CONTROL_RUN) == 0) {
		device_printf(rtk->dev, "starting stopped IOP\n");
		ASC_WR4(rtk, ASC_CPU_CONTROL, ASC_CPU_CONTROL_RUN);
	}
	error = apple_rtkit_send(rtk, EP_MGMT, MSG_TYPE_SET(MGMT_IOP_PWR) |
	    PWR_INIT);
	if (error != 0)
		return (error);

	/* Skip anything else (a late power ack) on the way to HELLO. */
	for (i = 0;; i++) {
		if (i == 16 || rtkit_recv_timeout(rtk, &ep, &msg, 1000000) != 0) {
			device_printf(rtk->dev, "no RTKit HELLO\n");
			return (ENXIO);
		}
		if (ep == EP_MGMT && MSG_TYPE(msg) == MGMT_HELLO)
			break;
		device_printf(rtk->dev, "before HELLO: ep %u msg %#jx\n", ep,
		    (uintmax_t)msg);
	}
	ver = MIN(RTKIT_MAX_VERSION, (msg >> 16) & 0xffff);
	if ((msg & 0xffff) > RTKIT_MAX_VERSION || ver < RTKIT_MIN_VERSION) {
		device_printf(rtk->dev, "unsupported RTKit versions %ju-%ju\n",
		    (uintmax_t)(msg & 0xffff), (uintmax_t)((msg >> 16) & 0xffff));
		return (ENXIO);
	}
	error = apple_rtkit_send(rtk, EP_MGMT, MSG_TYPE_SET(MGMT_HELLO_ACK) |
	    ver << 16 | ver);
	if (error != 0)
		return (error);

	do {
		if (rtkit_recv_timeout(rtk, &ep, &msg, 1000000) != 0 ||
		    ep != EP_MGMT || MSG_TYPE(msg) != MGMT_EPMAP) {
			device_printf(rtk->dev, "no RTKit endpoint map\n");
			return (ENXIO);
		}
		base = (msg >> 32) & 0x7;
		rtk->epmap[base] = msg & 0xffffffff;
		if (base == 0)
			eps = msg & 0xffffffff;	/* system endpoints */
		error = apple_rtkit_send(rtk, EP_MGMT,
		    MSG_TYPE_SET(MGMT_EPMAP) | (uint64_t)base << 32 |
		    ((msg & MGMT_EPMAP_DONE) ? MGMT_EPMAP_DONE :
		    MGMT_EPMAP_MORE));
		if (error != 0)
			return (error);
	} while ((msg & MGMT_EPMAP_DONE) == 0);

	const int sys_eps[] = { EP_DEBUG, EP_CRASHLOG, EP_SYSLOG, EP_IOREPORT,
	    EP_OSLOG, EP_TRACEKIT };
	for (i = 0; i < nitems(sys_eps); i++) {
		if ((eps & (1u << sys_eps[i])) == 0)
			continue;
		error = apple_rtkit_start_ep(rtk, sys_eps[i]);
		if (error != 0)
			return (error);
	}
	if (rtk->flags & APPLE_RTKIT_EARLY_AP) {
		error = apple_rtkit_send(rtk, EP_MGMT,
		    MSG_TYPE_SET(MGMT_AP_PWR) | PWR_ON);
		if (error != 0)
			return (error);
	}

	for (i = 0; (rtk->iop_power & 0xff) != PWR_ON; i++) {
		if (rtk->crashed)
			return (EIO);
		if (i == 100000) {
			device_printf(rtk->dev, "IOP power state %#x, not ON\n",
			    rtk->iop_power);
			return (ETIMEDOUT);
		}
		if (rtkit_recv(rtk, &ep, &msg))
			error = rtkit_handle(rtk, ep, msg);
		else
			DELAY(10);
		if (error != 0)
			return (error);
	}
	if (rtk->flags & APPLE_RTKIT_EARLY_AP)
		return (0);
	return (apple_rtkit_send(rtk, EP_MGMT, MSG_TYPE_SET(MGMT_AP_PWR) |
	    PWR_ON));
}

/* Sends a power state request and polls (up to 1 s) for its ack in *state. */
static int
rtkit_set_power(struct apple_rtkit *rtk, uint32_t type, uint32_t want,
    volatile uint32_t *state)
{
	uint64_t msg;
	uint32_t ep;
	int error, i;

	if ((error = apple_rtkit_send(rtk, EP_MGMT, MSG_TYPE_SET(type) |
	    want)) != 0)
		return (error);
	for (i = 0; (*state & 0xffff) != want; i++) {
		if (rtk->crashed)
			return (EIO);
		if (i == 100000) {
			device_printf(rtk->dev, "power state %#x, not %#x\n",
			    *state, want);
			return (ETIMEDOUT);
		}
		if (rtkit_recv(rtk, &ep, &msg))
			rtkit_handle(rtk, ep, msg);
		else
			DELAY(10);
	}
	return (0);
}

/*
 * Quiesces the IOP and puts it to sleep, as Linux's apple_rtkit_shutdown: a
 * storage IOP writes its caches back before acknowledging. Polled, for
 * shutdown handlers.
 */
int
apple_rtkit_shutdown(struct apple_rtkit *rtk)
{
	int error;

	error = rtkit_set_power(rtk, MGMT_AP_PWR, PWR_QUIESCED,
	    &rtk->ap_power);
	if (error == 0)
		error = rtkit_set_power(rtk, MGMT_IOP_PWR, PWR_SLEEP,
		    &rtk->iop_power);
	return (error);
}
