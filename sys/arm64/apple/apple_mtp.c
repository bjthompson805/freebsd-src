/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple MTP: the coprocessor behind the internal keyboard and trackpad of
 * M2+ laptops. iBoot loads its RTKit firmware into SRAM and leaves the CPU
 * stopped; we start it (apple_rtkit, polled) and then talk HID to it over a
 * DockChannel FIFO pair ("apple,dockchannel-hid" node, which names this
 * node in "apple,helper-cpu").
 *
 * Prerequisites set up elsewhere: the DART's DAPF allow-list (m1n1, from the
 * ADT) - without it the firmware hangs before HELLO. Its log buffers must
 * sit at IOVAs inside the DART's "apple,dma-range" window (it crashes on a
 * buffer at IOVA == PA), so they are mapped there with apple_dart_map().
 *
 * DockChannel packets: an 8-byte header (hdr_len 8, type 0x11 to the IOP /
 * 0x12 from it, payload length, sequence, interface), the payload padded to
 * 4 bytes, and a 32-bit checksum that makes the sum of all words ~0.
 * Payloads start with a message header (flags, length, retcode): flags 0
 * is a report/notification, 0x80 a device-control command or its ack.
 *
 * Interface 0 ("comm") announces each device with init messages carrying
 * its name and HID report descriptor; once the last one arrives we enable
 * the device (command 0xb4) and it starts sending input reports. Each HID
 * device we support gets a hidbus child; for now only "keyboard" (the
 * trackpad needs firmware and AFE setup first).
 *
 * Everything is polled from a callout; no DockChannel/mailbox interrupts.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/endian.h>
#include <sys/eventhandler.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/cpufunc.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <dev/evdev/input.h>
#include <dev/hid/hid.h>
#include <dev/hid/hidquirk.h>

#include <arm64/apple/apple_dart.h>
#include <arm64/apple/apple_rtkit.h>

#include "hid_if.h"

/* DockChannel FIFO data registers */
#define	DC_TX_8			0x04
#define	DC_TX_FREE		0x14
#define	DC_RX_8			0x1c
#define	DC_RX_COUNT		0x2c

#define	PKT_HDR_LEN		8
#define	PKT_TYPE_TX		0x11
#define	PKT_TYPE_RX		0x12
#define	PKT_MAX			0x800

#define	MSG_FLAGS_REPORT	0x00
#define	MSG_FLAGS_CONTROL	0x80

#define	CMD_ENABLE		0xb4
#define	CMD_SEND_FIRMWARE	0x95
#define	CMD_RESET_IFACE		0x40	/* power method, iface, state[, changed] */
#define	CMD_ACK_GPIO		0xa1
#define	POWER_METHOD_2		2
#define	POWER_OFF		0
#define	POWER_ON		2

/* comm (interface 0) notifications */
#define	NOTIFY_INIT		0xf0
#define	NOTIFY_READY		0xf1
#define	NOTIFY_GPIO		0xa0

/* Trackpad firmware file ("HIDF" header, then the blob the MTP takes). */
struct mtp_fw_header {
	uint32_t	magic;
	uint32_t	version;
	uint32_t	hdr_length;
	uint32_t	data_length;
	uint32_t	iface_offset;	/* byte to patch with the iface number */
	uint32_t	pad[3];
};
#define	MTP_FW_MAGIC		0x46444948	/* "HIDF" */
#define	INIT_BLK_DESCRIPTOR	0
#define	INIT_BLK_END		2

#define	MTP_MAX_IFACES		8

struct mtp_iface {
	struct apple_mtp_softc	*sc;
	uint8_t			id;
	char			name[17];
	uint8_t			*rdesc;
	size_t			rdesc_len;
	bool			want_enable;
	bool			ready;
	uint8_t			tx_seq;

	device_t		hidbus;
	struct hid_device_info	hw;
	hid_intr_t		*intr;
	void			*intr_ctx;
	bool			open;
};

struct mtp_report {
	struct mtp_iface	*ifc;
	size_t			len;
	uint8_t			data[PKT_MAX];
};

struct apple_mtp_softc {
	device_t		dev;
	vm_offset_t		asc;
	vm_offset_t		mbox;
	vm_offset_t		dc;		/* DockChannel "data" FIFO */
	phandle_t		hid_node;
	struct apple_rtkit	*rtk;
	uint64_t		iova_next, iova_end;	/* dma-range window */

	struct mtx		mtx;		/* rtkit, FIFO, iface state */
	struct callout		poll;
	bool			ctl_pending;	/* device control awaiting ack */
	uint32_t		ctl_retcode;
	uint8_t			gpio_ack[16];	/* GPIO request to ack, if len */
	size_t			gpio_ack_len;
	struct task		tp_task;	/* trackpad bring-up */
	eventhandler_tag	mountroot_tag;
	u_int			mt_logged;
	u_long			reports;	/* input reports delivered */
	struct mtp_iface	ifaces[MTP_MAX_IFACES];
	uint8_t			pkt[PKT_HDR_LEN + PKT_MAX + 4];
	struct mtp_report	rep;		/* handed to hidbus unlocked */
};

#define	DC_RD4(sc, o)		(*(volatile uint32_t *)((sc)->dc + (o)))
#define	DC_WR4(sc, o, v)	(*(volatile uint32_t *)((sc)->dc + (o)) = (v))

static int
mtp_buf_map(void *arg, vm_paddr_t pa, size_t size, uint64_t *iova)
{
	struct apple_mtp_softc *sc = arg;
	int error;

	size = roundup2(size, 16384);
	if (sc->iova_next + size > sc->iova_end)
		return (ENOSPC);
	error = apple_dart_map(sc->dev, sc->iova_next, pa, size);
	if (error != 0)
		return (error);
	*iova = sc->iova_next;
	sc->iova_next += size;
	return (0);
}

static const struct apple_rtkit_ops mtp_rtkit_ops = {
	.buf_map = mtp_buf_map,
};

/* The IOVA window the firmware accepts: <&dart>'s "apple,dma-range". */
static int
mtp_dma_window(struct apple_mtp_softc *sc, phandle_t node)
{
	pcell_t iommu[2], range[4];

	if (OF_getencprop(node, "iommus", iommu, sizeof(iommu)) !=
	    sizeof(iommu) ||
	    OF_getencprop(OF_node_from_xref(iommu[0]), "apple,dma-range",
	    range, sizeof(range)) != sizeof(range)) {
		device_printf(sc->dev, "no DART apple,dma-range\n");
		return (ENXIO);
	}
	sc->iova_next = (uint64_t)range[0] << 32 | range[1];
	sc->iova_end = sc->iova_next + ((uint64_t)range[2] << 32 | range[3]);
	return (0);
}

static const char *mtp_hid_ifaces[] = { "keyboard" };

static uint32_t
mtp_checksum(const uint8_t *p, size_t len)
{
	uint32_t sum = 0;
	size_t i;

	for (i = 0; i + 4 <= len; i += 4)
		sum += le32dec(p + i);
	return (~sum);
}

/* Read len bytes, waiting up to ~10 ms for each to arrive. */
static int
mtp_dc_read(struct apple_mtp_softc *sc, uint8_t *buf, size_t len)
{
	size_t i;
	int w;

	for (i = 0; i < len; i++) {
		for (w = 0; DC_RD4(sc, DC_RX_COUNT) == 0; w++) {
			if (w == 10000)
				return (ETIMEDOUT);
			DELAY(1);
		}
		buf[i] = DC_RD4(sc, DC_RX_8) >> 8;
	}
	return (0);
}

static int
mtp_dc_write(struct apple_mtp_softc *sc, const uint8_t *buf, size_t len)
{
	size_t i;
	int w;

	for (i = 0; i < len; i++) {
		for (w = 0; DC_RD4(sc, DC_TX_FREE) == 0; w++) {
			if (w == 10000)
				return (ETIMEDOUT);
			DELAY(1);
		}
		DC_WR4(sc, DC_TX_8, buf[i]);
	}
	return (0);
}

static int
mtp_send(struct apple_mtp_softc *sc, struct mtp_iface *ifc, uint16_t flags,
    const void *body, size_t len)
{
	uint8_t pkt[PKT_HDR_LEN + 8 + 64 + 4];
	size_t plen = roundup2(8 + len, 4);

	if (8 + len > 64 + 8)
		return (EINVAL);
	memset(pkt, 0, sizeof(pkt));
	pkt[0] = PKT_HDR_LEN;
	pkt[1] = PKT_TYPE_TX;
	le16enc(pkt + 2, plen);
	pkt[4] = ifc->tx_seq++;
	pkt[5] = ifc->id;
	le16enc(pkt + 8, flags);		/* message header */
	le16enc(pkt + 10, len);
	memcpy(pkt + 16, body, len);
	le32enc(pkt + PKT_HDR_LEN + plen,
	    mtp_checksum(pkt, PKT_HDR_LEN + plen));
	return (mtp_dc_write(sc, pkt, PKT_HDR_LEN + plen + 4));
}

static void
mtp_comm_init_msg(struct apple_mtp_softc *sc, const uint8_t *m, size_t len)
{
	struct mtp_iface *ifc;
	size_t off;
	uint16_t type, blen;

	/* f0 01 00 <id> <name[16]> <more u16> then {type, len, payload}... */
	if (len < 22 || m[3] >= MTP_MAX_IFACES)
		return;
	ifc = &sc->ifaces[m[3]];
	ifc->id = m[3];
	memcpy(ifc->name, m + 4, 16);
	ifc->name[16] = '\0';
	for (off = 22; off + 4 <= len; off += 4 + blen) {
		type = le16dec(m + off);
		blen = le16dec(m + off + 2);
		if (off + 4 + blen > len)
			break;
		if (type == INIT_BLK_DESCRIPTOR && ifc->rdesc == NULL) {
			ifc->rdesc = malloc(blen, M_DEVBUF, M_NOWAIT);
			if (ifc->rdesc != NULL) {
				memcpy(ifc->rdesc, m + off + 4, blen);
				ifc->rdesc_len = blen;
			}
		}
		if (type == INIT_BLK_END)
			break;
	}
	if (le16dec(m + 20) == 0)		/* no more init packets */
		ifc->want_enable = true;
}

/* Handle one packet; an input report is copied to sc->rep. */
static void
mtp_packet(struct apple_mtp_softc *sc, uint8_t iface, uint8_t *p, size_t len)
{
	struct mtp_iface *ifc;
	uint16_t flags, mlen;

	if (len < 8 || iface >= MTP_MAX_IFACES)
		return;
	flags = le16dec(p);
	mlen = le16dec(p + 2);
	if (8 + (size_t)mlen > len)
		return;
	p += 8;
	ifc = &sc->ifaces[iface];

	if (flags == MSG_FLAGS_CONTROL) {
		sc->ctl_retcode = le32dec(p - 4);
		sc->ctl_pending = false;
		return;
	}
	if (flags != MSG_FLAGS_REPORT || mlen == 0)
		return;
	if (iface == 0) {
		if (p[0] == NOTIFY_INIT && mlen >= 2 && p[1] == 0x01)
			mtp_comm_init_msg(sc, p, mlen);
		else if (p[0] == NOTIFY_READY && mlen >= 2 &&
		    p[1] < MTP_MAX_IFACES) {
			sc->ifaces[p[1]].ready = true;
			if (bootverbose)
				device_printf(sc->dev, "%s ready\n",
				    sc->ifaces[p[1]].name);
		} else if (p[0] == NOTIFY_GPIO && mlen <= 11) {
			/* XXX no SMC GPIO yet: log the request, ack success. */
			device_printf(sc->dev, "GPIO request %*D\n", (int)mlen,
			    p, " ");
			sc->gpio_ack[0] = CMD_ACK_GPIO;
			le32enc(sc->gpio_ack + 1, 0);
			memcpy(sc->gpio_ack + 5, p, mlen);
			sc->gpio_ack_len = 5 + mlen;
		}
		return;
	}
	/* XXX bring-up: show the first trackpad reports. */
	if (strcmp(ifc->name, "multi-touch") == 0 && sc->mt_logged < 4) {
		sc->mt_logged++;
		device_printf(sc->dev, "multi-touch report (%u): %*D\n", mlen,
		    (int)MIN(mlen, 48), p, " ");
	}
	if (ifc->open && ifc->intr != NULL && sc->rep.ifc == NULL) {
		sc->rep.ifc = ifc;
		sc->rep.len = mlen;
		memcpy(sc->rep.data, p, mlen);
	}
}

/* Read and handle at most one packet. Returns true if one was read. */
static bool
mtp_receive(struct apple_mtp_softc *sc)
{
	uint8_t *pkt = sc->pkt;
	uint16_t plen;

	mtx_assert(&sc->mtx, MA_OWNED);
	if (DC_RD4(sc, DC_RX_COUNT) < PKT_HDR_LEN)
		return (false);
	if (mtp_dc_read(sc, pkt, PKT_HDR_LEN) != 0)
		return (false);
	plen = le16dec(pkt + 2);
	if (pkt[0] != PKT_HDR_LEN || plen > PKT_MAX) {
		device_printf(sc->dev, "bad packet header %8D\n", pkt, " ");
		while (DC_RD4(sc, DC_RX_COUNT) != 0)	/* resync */
			(void)DC_RD4(sc, DC_RX_8);
		return (false);
	}
	if (mtp_dc_read(sc, pkt + PKT_HDR_LEN, plen + 4) != 0) {
		device_printf(sc->dev, "short packet\n");
		return (false);
	}
	if (mtp_checksum(pkt, PKT_HDR_LEN + plen) !=
	    le32dec(pkt + PKT_HDR_LEN + plen))
		device_printf(sc->dev, "packet checksum mismatch\n");
	mtp_packet(sc, pkt[5], pkt + PKT_HDR_LEN, plen);
	return (true);
}

/* Service RTKit, enable announced devices, read packets until a report. */
static void
mtp_work(struct apple_mtp_softc *sc)
{
	struct mtp_iface *ifc;
	uint8_t cmd[2];
	int i;

	mtx_assert(&sc->mtx, MA_OWNED);
	apple_rtkit_poll(sc->rtk);
	if (!sc->ctl_pending && sc->gpio_ack_len != 0) {
		if (mtp_send(sc, &sc->ifaces[0], MSG_FLAGS_CONTROL,
		    sc->gpio_ack, sc->gpio_ack_len) == 0)
			sc->ctl_pending = true;
		sc->gpio_ack_len = 0;
	}
	if (!sc->ctl_pending) {
		for (i = 1; i < MTP_MAX_IFACES; i++) {
			ifc = &sc->ifaces[i];
			if (!ifc->want_enable)
				continue;
			ifc->want_enable = false;
			cmd[0] = CMD_ENABLE;
			cmd[1] = ifc->id;
			if (mtp_send(sc, &sc->ifaces[0], MSG_FLAGS_CONTROL,
			    cmd, sizeof(cmd)) == 0)
				sc->ctl_pending = true;
			break;
		}
	}
	while (sc->rep.ifc == NULL && mtp_receive(sc))
		;
}

/* Deliver a report collected under the lock, with the lock dropped. */
static void
mtp_deliver(struct apple_mtp_softc *sc)
{
	struct mtp_iface *ifc;

	mtx_assert(&sc->mtx, MA_OWNED);
	while ((ifc = sc->rep.ifc) != NULL) {
		mtx_unlock(&sc->mtx);
		ifc->intr(ifc->intr_ctx, sc->rep.data, sc->rep.len);
		mtx_lock(&sc->mtx);
		sc->reports++;
		sc->rep.ifc = NULL;
		mtp_work(sc);
	}
}

static void
mtp_poll(void *arg)
{
	struct apple_mtp_softc *sc = arg;

	mtx_lock(&sc->mtx);
	mtp_work(sc);
	mtp_deliver(sc);
	if (!apple_rtkit_crashed(sc->rtk))
		callout_schedule(&sc->poll, hz / 100);
	mtx_unlock(&sc->mtx);
}

static int
mtp_map_reg(device_t dev, phandle_t node, const char *name, vm_offset_t *va)
{
	bus_addr_t pa;
	bus_size_t size;
	int idx;

	if (ofw_bus_find_string_index(node, "reg-names", name, &idx) != 0 ||
	    ofw_reg_to_paddr(node, idx, &pa, &size, NULL) != 0) {
		device_printf(dev, "no \"%s\" registers\n", name);
		return (ENXIO);
	}
	*va = (vm_offset_t)pmap_mapdev_attr(pa, size, VM_MEMATTR_DEVICE_NP);
	return (0);
}

/* Send a comm device-control command and wait for its ack (sleepable). */
static int
mtp_comm_cmd(struct apple_mtp_softc *sc, const void *cmd, size_t len)
{
	int i, error;

	mtx_lock(&sc->mtx);
	for (i = 0; sc->ctl_pending; i++) {
		if (i == 100) {
			mtx_unlock(&sc->mtx);
			return (EBUSY);
		}
		mtx_sleep(&sc->ctl_pending, &sc->mtx, 0, "mtpcmd", hz / 100);
	}
	error = mtp_send(sc, &sc->ifaces[0], MSG_FLAGS_CONTROL, cmd, len);
	if (error == 0)
		sc->ctl_pending = true;
	for (i = 0; error == 0 && sc->ctl_pending; i++) {
		if (i == 200) {
			error = ETIMEDOUT;
			break;
		}
		mtp_work(sc);
		mtp_deliver(sc);
		if (sc->ctl_pending)
			mtx_sleep(&sc->ctl_pending, &sc->mtx, 0, "mtpack",
			    hz / 100);
	}
	if (error == 0 && sc->ctl_retcode != 0) {
		device_printf(sc->dev, "command %#x failed: %#x\n",
		    *(const uint8_t *)cmd, sc->ctl_retcode);
		error = EIO;
	}
	mtx_unlock(&sc->mtx);
	return (error);
}

/* Power method 2 transition: announce, (AFE reset), wait, confirm. */
static int
mtp_pm2_set_power(struct apple_mtp_softc *sc, uint8_t iface, uint8_t state)
{
	uint8_t cmd[9] = { CMD_RESET_IFACE, POWER_METHOD_2, iface, state };
	int error;

	error = mtp_comm_cmd(sc, cmd, sizeof(cmd));
	if (error != 0)
		return (error);
	/* XXX the AFE reset line (SMC gP1c) should be driven here. */
	pause("mtpafe", hz / 20);
	cmd[4] = 1;				/* has changed */
	return (mtp_comm_cmd(sc, cmd, sizeof(cmd)));
}

static struct mtp_iface *
mtp_iface_by_name(struct apple_mtp_softc *sc, const char *name)
{
	int i;

	for (i = 1; i < MTP_MAX_IFACES; i++)
		if (strcmp(sc->ifaces[i].name, name) == 0)
			return (&sc->ifaces[i]);
	return (NULL);
}

/*
 * The trackpad (multi-touch) needs firmware from the filesystem: upload it,
 * then power-cycle the interface. Runs once root is mounted.
 */
static void
mtp_trackpad_task(void *arg, int pending)
{
	struct apple_mtp_softc *sc = arg;
	const struct firmware *fw;
	const struct mtp_fw_header *hdr;
	struct mtp_iface *ifc;
	phandle_t mt;
	char name[64];
	uint8_t *buf;
	uint64_t iova;
	size_t size;
	uint8_t cmd[16];
	int error;

	ifc = mtp_iface_by_name(sc, "multi-touch");
	mt = ofw_bus_find_child(sc->hid_node, "multi-touch");
	if (ifc == NULL || mt <= 0 ||
	    OF_getprop(mt, "firmware-name", name, sizeof(name)) <= 0)
		return;
	fw = firmware_get_flags(name, FIRMWARE_GET_NOWARN);
	if (fw == NULL) {
		device_printf(sc->dev, "no trackpad firmware /boot/firmware/%s\n",
		    name);
		return;
	}
	hdr = fw->data;
	if (fw->datasize < sizeof(*hdr) || hdr->magic != MTP_FW_MAGIC ||
	    hdr->version != 1 || hdr->hdr_length < sizeof(*hdr) ||
	    (size_t)hdr->hdr_length + hdr->data_length > fw->datasize ||
	    hdr->iface_offset >= hdr->data_length) {
		device_printf(sc->dev, "%s: bad firmware header\n", name);
		firmware_put(fw, FIRMWARE_UNLOAD);
		return;
	}
	size = roundup2(hdr->data_length, 16384);
	buf = contigmalloc(size, M_DEVBUF, M_WAITOK | M_ZERO, 0,
	    ~(vm_paddr_t)0, 16384, 0);
	memcpy(buf, (const uint8_t *)fw->data + hdr->hdr_length,
	    hdr->data_length);
	buf[hdr->iface_offset] = ifc->id;
	cpu_dcache_wb_range(buf, size);
	error = mtp_buf_map(sc, pmap_kextract((vm_offset_t)buf), size, &iova);
	if (error != 0)
		goto out;

	memset(cmd, 0, sizeof(cmd));
	cmd[0] = CMD_SEND_FIRMWARE;
	cmd[1] = 2;
	cmd[3] = ifc->id;
	le64enc(cmd + 4, iova);
	le32enc(cmd + 12, hdr->data_length);
	error = mtp_comm_cmd(sc, cmd, sizeof(cmd));
	device_printf(sc->dev, "trackpad firmware %s (%u bytes) sent: %d\n",
	    name, hdr->data_length, error);
	if (error == 0)
		error = mtp_pm2_set_power(sc, ifc->id, POWER_OFF);
	if (error == 0)
		error = mtp_pm2_set_power(sc, ifc->id, POWER_ON);
	device_printf(sc->dev, "trackpad power cycle: %d\n", error);
out:
	/* The MTP may keep using the buffer: never freed. */
	firmware_put(fw, FIRMWARE_UNLOAD);
}

static void
mtp_mountroot(void *arg)
{
	struct apple_mtp_softc *sc = arg;

	taskqueue_enqueue(taskqueue_thread, &sc->tp_task);
}

static int
apple_mtp_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    !ofw_bus_is_compatible(dev, "apple,mtp"))
		return (ENXIO);
	device_set_desc(dev, "Apple MTP keyboard/trackpad controller");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_mtp_add_hid(struct apple_mtp_softc *sc, struct mtp_iface *ifc)
{
	ifc->sc = sc;
	snprintf(ifc->hw.name, sizeof(ifc->hw.name), "Apple MTP %s",
	    ifc->name);
	ifc->hw.idBus = BUS_SPI;
	ifc->hw.idVendor = 0x05ac;
	ifc->hw.idProduct = ifc->id;
	ifc->hw.rdescsize = ifc->rdesc_len;
	/* Writes (keyboard LEDs) become set_report, which we ignore. */
	hid_add_dynamic_quirk(&ifc->hw, HQ_NOWRITE);

	ifc->hidbus = device_add_child(sc->dev, "hidbus", DEVICE_UNIT_ANY);
	if (ifc->hidbus == NULL)
		return (ENOMEM);
	device_set_ivars(ifc->hidbus, &ifc->hw);
	return (0);
}

static int
apple_mtp_attach(device_t dev)
{
	struct apple_mtp_softc *sc = device_get_softc(dev);
	phandle_t node, hid;
	pcell_t xref, helper;
	bus_addr_t pa;
	bus_size_t size;
	struct mtp_iface *ifc;
	int error, i, j, n;

	sc->dev = dev;
	node = ofw_bus_get_node(dev);
	mtx_init(&sc->mtx, "apple_mtp", NULL, MTX_DEF);
	callout_init(&sc->poll, 1);

	/* The HID FIFO pair is in the DockChannel node naming us as helper. */
	xref = OF_xref_from_node(node);
	hid = ofw_bus_find_compatible(OF_finddevice("/"),
	    "apple,dockchannel-hid");
	if (hid > 0 && (OF_getencprop(hid, "apple,helper-cpu", &helper,
	    sizeof(helper)) <= 0 || helper != xref))
		hid = 0;
	if (hid <= 0) {
		device_printf(dev, "no DockChannel HID node\n");
		return (ENXIO);
	}

	if (ofw_reg_to_paddr(node, 0, &pa, &size, NULL) != 0)
		return (ENXIO);
	sc->asc = (vm_offset_t)pmap_mapdev(pa, size);
	if (OF_getencprop(node, "mboxes", &xref, sizeof(xref)) <= 0 ||
	    ofw_reg_to_paddr(OF_node_from_xref(xref), 0, &pa, &size,
	    NULL) != 0) {
		device_printf(dev, "no mailbox\n");
		return (ENXIO);
	}
	sc->mbox = (vm_offset_t)pmap_mapdev(pa, size);
	sc->hid_node = hid;
	if (mtp_map_reg(dev, hid, "data", &sc->dc) != 0)
		return (ENXIO);

	if (mtp_dma_window(sc, node) != 0)
		return (ENXIO);
	error = apple_dart_identity(dev);
	if (error != 0) {
		device_printf(dev, "DART setup failed: %d\n", error);
		return (error);
	}

	/* Drop anything a previous stage left in the FIFO. */
	while (DC_RD4(sc, DC_RX_COUNT) != 0)
		(void)DC_RD4(sc, DC_RX_8);

	sc->rtk = apple_rtkit_create(dev, sc->asc, sc->mbox, &mtp_rtkit_ops,
	    sc, APPLE_RTKIT_EARLY_AP);
	mtx_lock(&sc->mtx);
	error = apple_rtkit_boot(sc->rtk);
	if (error != 0) {
		mtx_unlock(&sc->mtx);
		device_printf(dev, "RTKit boot failed: %d\n", error);
		return (error);
	}

	/* Let the devices announce themselves (~100 ms on J815). */
	for (i = 0; i < 2000; i++) {
		mtp_work(sc);
		sc->rep.ifc = NULL;		/* nobody is listening yet */
		n = 0;
		for (j = 0; j < nitems(mtp_hid_ifaces); j++)
			for (ifc = &sc->ifaces[1];
			    ifc < &sc->ifaces[MTP_MAX_IFACES]; ifc++)
				if (ifc->ready && ifc->rdesc != NULL &&
				    strcmp(ifc->name, mtp_hid_ifaces[j]) == 0)
					n++;
		if (n == nitems(mtp_hid_ifaces))
			break;
		DELAY(1000);
	}
	mtx_unlock(&sc->mtx);

	for (ifc = &sc->ifaces[1]; ifc < &sc->ifaces[MTP_MAX_IFACES]; ifc++) {
		if (ifc->name[0] == '\0')
			continue;
		if (bootverbose)
			device_printf(dev, "interface %u \"%s\": %s, %zu-byte "
			    "report descriptor\n", ifc->id, ifc->name,
			    ifc->ready ? "ready" : "not ready", ifc->rdesc_len);
		for (j = 0; j < nitems(mtp_hid_ifaces); j++)
			if (ifc->ready && ifc->rdesc != NULL &&
			    strcmp(ifc->name, mtp_hid_ifaces[j]) == 0)
				apple_mtp_add_hid(sc, ifc);
	}

	SYSCTL_ADD_ULONG(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "reports",
	    CTLFLAG_RD, &sc->reports, "Input reports delivered");
	callout_reset(&sc->poll, hz / 100, mtp_poll, sc);
	TASK_INIT(&sc->tp_task, 0, mtp_trackpad_task, sc);
	sc->mountroot_tag = EVENTHANDLER_REGISTER(mountroot, mtp_mountroot, sc,
	    EVENTHANDLER_PRI_ANY);
	bus_attach_children(dev);
	return (0);
}

static int
apple_mtp_detach(device_t dev)
{
	return (EBUSY);
}

static struct mtp_iface *
mtp_iface_by_hidbus(device_t dev, device_t child)
{
	struct apple_mtp_softc *sc = device_get_softc(dev);
	int i;

	for (i = 0; i < MTP_MAX_IFACES; i++)
		if (sc->ifaces[i].hidbus == child)
			return (&sc->ifaces[i]);
	panic("apple_mtp: unknown child");
}

static void
apple_mtp_intr_setup(device_t dev, device_t child, hid_intr_t intr,
    void *context, struct hid_rdesc_info *rdesc)
{
	struct apple_mtp_softc *sc = device_get_softc(dev);
	struct mtp_iface *ifc = mtp_iface_by_hidbus(dev, child);

	if (intr == NULL)
		return;
	rdesc->rdsize = PKT_MAX - 8;
	rdesc->grsize = 0;
	rdesc->srsize = 0;
	rdesc->wrsize = 0;
	mtx_lock(&sc->mtx);
	ifc->intr = intr;
	ifc->intr_ctx = context;
	mtx_unlock(&sc->mtx);
}

static void
apple_mtp_intr_unsetup(device_t dev, device_t child)
{
	struct apple_mtp_softc *sc = device_get_softc(dev);
	struct mtp_iface *ifc = mtp_iface_by_hidbus(dev, child);

	mtx_lock(&sc->mtx);
	ifc->intr = NULL;
	mtx_unlock(&sc->mtx);
}

static int
apple_mtp_intr_start(device_t dev, device_t child)
{
	struct apple_mtp_softc *sc = device_get_softc(dev);
	struct mtp_iface *ifc = mtp_iface_by_hidbus(dev, child);

	mtx_lock(&sc->mtx);
	ifc->open = true;
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
apple_mtp_intr_stop(device_t dev, device_t child)
{
	struct apple_mtp_softc *sc = device_get_softc(dev);
	struct mtp_iface *ifc = mtp_iface_by_hidbus(dev, child);

	mtx_lock(&sc->mtx);
	ifc->open = false;
	mtx_unlock(&sc->mtx);
	return (0);
}

/* Keyboard polling (kdb, early console): no locks may be taken. */
static void
apple_mtp_intr_poll(device_t dev, device_t child)
{
	struct apple_mtp_softc *sc = device_get_softc(dev);
	struct mtp_iface *ifc;

	while (sc->rep.ifc == NULL && mtp_receive(sc))
		;
	if ((ifc = sc->rep.ifc) != NULL) {
		ifc->intr(ifc->intr_ctx, sc->rep.data, sc->rep.len);
		sc->rep.ifc = NULL;
	}
}

static int
apple_mtp_get_rdesc(device_t dev, device_t child, void *buf, hid_size_t len)
{
	struct mtp_iface *ifc = mtp_iface_by_hidbus(dev, child);

	if (ifc->rdesc_len != len)
		return (ENXIO);
	memcpy(buf, ifc->rdesc, len);
	return (0);
}

static int
apple_mtp_set_report(device_t dev, device_t child, const void *buf,
    hid_size_t len, uint8_t type, uint8_t id)
{
	return (0);	/* XXX keyboard LEDs/backlight: not wired up yet */
}

static int
apple_mtp_get_report(device_t dev, device_t child, void *buf,
    hid_size_t maxlen, hid_size_t *actlen, uint8_t type, uint8_t id)
{
	return (ENOTSUP);
}

static int
apple_mtp_set_idle(device_t dev, device_t child, uint16_t duration,
    uint8_t id)
{
	return (0);
}

static int
apple_mtp_set_protocol(device_t dev, device_t child, uint16_t protocol)
{
	return (0);
}

static device_method_t apple_mtp_methods[] = {
	DEVMETHOD(device_probe,		apple_mtp_probe),
	DEVMETHOD(device_attach,	apple_mtp_attach),
	DEVMETHOD(device_detach,	apple_mtp_detach),

	DEVMETHOD(hid_intr_setup,	apple_mtp_intr_setup),
	DEVMETHOD(hid_intr_unsetup,	apple_mtp_intr_unsetup),
	DEVMETHOD(hid_intr_start,	apple_mtp_intr_start),
	DEVMETHOD(hid_intr_stop,	apple_mtp_intr_stop),
	DEVMETHOD(hid_intr_poll,	apple_mtp_intr_poll),
	DEVMETHOD(hid_get_rdesc,	apple_mtp_get_rdesc),
	DEVMETHOD(hid_set_report,	apple_mtp_set_report),
	DEVMETHOD(hid_get_report,	apple_mtp_get_report),
	DEVMETHOD(hid_set_idle,		apple_mtp_set_idle),
	DEVMETHOD(hid_set_protocol,	apple_mtp_set_protocol),

	DEVMETHOD_END
};

static driver_t apple_mtp_driver = {
	"apple_mtp",
	apple_mtp_methods,
	sizeof(struct apple_mtp_softc),
};

DRIVER_MODULE(apple_mtp, simplebus, apple_mtp_driver, NULL, NULL);
MODULE_DEPEND(apple_mtp, hid, 1, 1, 1);
MODULE_DEPEND(apple_mtp, hidbus, 1, 1, 1);
