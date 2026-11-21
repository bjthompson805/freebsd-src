/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple N1 ("Centauri"): Apple's own Wi-Fi/Bluetooth chip on the internal
 * PCIe port of M5 Macs. m1n1 trains the link (src/pcie.c, T8142), so the
 * generic ECAM host finds function 0 (106b:1900) still in its boot ROM.
 *
 * Stage 1: ROM boot. The whole restore-image ftab (rkosftab) is handed to
 * the ROM through a single scatter entry; the ROM reports through BAR0:
 *   0x8000 execution stage (0 ROM, 4 firmware running)
 *   0x803c 0, 0x8044/0x8048 descriptor address, 0x804c 0x20 (entry size)
 *   0x8040 image response, 0x807c board word ((board-id << 16) | chip-id)
 *   0x9000 doorbell (1 starts the boot)
 * The ROM signals through MSI; we point the MSI address at a DMA page and
 * poll instead (no MSI controller driver yet). Register facts come from the
 * aurora-linux centauri driver's measured J813 sequence.
 *
 * DMA goes through the PCIe DART stream named by the host's "iommu-map";
 * see n1_dma_alloc for the IOVA window.
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
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/cpufunc.h>
#include <machine/resource.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <sys/socket.h>
#include <sys/sockio.h>

#include <net/bpf.h>
#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_types.h>
#include <net/vnet.h>

#include <netgraph/ng_message.h>
#include <netgraph/netgraph.h>
#include <netgraph/bluetooth/include/ng_hci.h>

#include <arm64/apple/apple_dart.h>
#include <arm64/apple/apple_smc.h>

/* hw.apple_n1.debug=1: bring-up diagnostics (register/ring/context dumps, firmware log) */
static int n1_debug = 0;
TUNABLE_INT("hw.apple_n1.debug", &n1_debug);
#define	N1_DPRINTF(sc, ...) do {					\
	if (n1_debug)							\
		device_printf((sc)->dev, __VA_ARGS__);			\
} while (0)
#define	N1_DPRINTF_DEV(dev, ...) do {					\
	if (n1_debug)							\
		device_printf((dev), __VA_ARGS__);			\
} while (0)
#define	N1_PRINTF(...) do {						\
	if (n1_debug)							\
		printf(__VA_ARGS__);					\
} while (0)

#define	N1_VENDOR		0x106b
#define	N1_DEV_CONTROL		0x1900
#define	N1_DEV_CONTROL_FW	0x1901

#define	N1_STAGE		0x8000
#define	N1_STAGE_ROM		0
#define	N1_STAGE_RUNNING	4
#define	N1_ROM_CTRL		0x803c
#define	N1_ROM_RESPONSE		0x8040
#define	N1_ROM_DESC_LO		0x8044
#define	N1_ROM_DESC_HI		0x8048
#define	N1_ROM_DESC_SIZE	0x804c
#define	N1_ROM_BOARD		0x807c
#define	N1_ROM_DOORBELL		0x9000

#define	N1_MSI_DATA		0xa5a0
#define	N1_PAGE			16384
#define	N1_IOVA_BASE		0x200000ul	/* below 4 GiB, as macOS (0x250000) */
#define	N1_IOVA_END		0x10000000000ul	/* 1 TiB: 40 bits */
#define	N1_FW_MAX		(64 * 1024 * 1024)

#define	N1_DART_PARAMS3		0x008
#define	N1_DART_VERSION(v)	((v) & 0xffff)
#define	N1_DART_EXCEPTIONS	0x4000
#define	N1_DART_SID_FAULT(sid)	(0x8000 + 0x40 * (sid))

struct n1_board {
	const char	*compat;
	const char	*firmware;
	uint32_t	board_word;
	/* apcie port 0 config, its lane PHY and the PERST# pad (from the ADT) */
	bus_addr_t	port, lane, perst;
};

static const struct n1_board n1_boards[] = {
	{ "apple,j815", "apple/j815-ftab.bin", 0x00268142,
	    0x488028000, 0x48f010000, 0x39a000300 },
	{ "apple,j813", "apple/j813-ftab.bin", 0x00248142,
	    0x488028000, 0x48f010000, 0x39a000300 },
};

/* apcie port registers used by the post-ROM port cycle */
#define	PORT_LTSSM_EN		0x080
#define	PORT_PME		0x088
#define	PORT_MSICFG		0x124
#define	PORT_LINKDOWN		0x13c
#define	PORT_MSIADDR		0x16c
#define	N1_PORT_MSI		0
#define	PORT_PIPE		0x0a8
#define	PORT_LINKSTS		0x208
#define	PORT_APPCLK		0x800
#define	PORT_STATUS		0x804
#define	PORT_LANES		0x820
#define	PORT_PERST		0x82c
#define	PORT_RID2SID		0x3000
#define	PORT_RID2SID_VALID	(1u << 31)
#define	PORT_RID2SID_SID_SHIFT	16
#define	PORT_MSIMAP		0x3800
#define	RC_LTSSM		0x728	/* root port config: LTSSM state in [5:0] */
#define	LTSSM_L0		0x11
#define	LTSSM_L2_IDLE		0x14
#define	PAD_OUT_LOW		0x2
#define	PAD_OUT_HIGH		0x3

struct n1_softc {
	device_t		dev;
	struct resource		*bar;
	const struct n1_board	*board;
	phandle_t		dart;
	u_int			sid;
	uint64_t		iova_next;
	struct task		boot_task;
	eventhandler_tag	mountroot_tag;
	void			*fw, *desc, *mbox;
	uint64_t		fw_iova, desc_iova, mbox_iova, tkt_iova;
	size_t			tkt_size, fw_size;
	/* Firmware personality: functions 0 control, 1 alpha (Wi-Fi), 2 beta */
	device_t		fn[3];
	struct resource		*fnbar[3];
	void			*fnmsi[3];
	struct n1_ctx {
		uint8_t		*buf, *msgs;
		uint64_t	buf_iova, msgs_iova;
		uint16_t	tr, cr, index[4], mtr, mcr, head;
	}			control, wifi, beta;
	int			routed;
	uint8_t			*ring2, *rx;	/* XXX bring-up: control receive */
	int			rx_seen, rx_posted;
	uint64_t		rx_iova;	/* map DMA on the function streams too */
	void			*secondary;
	uint64_t		secondary_iova;
#define	N1_BT_RINGS		4
	struct {			/* beta (Bluetooth) rings */
		uint8_t		*mem;
		uint16_t	tail, seen;
	}			bt[N1_BT_RINGS];
	struct mtx		bt_mtx;
	struct callout		bt_callout;
	node_p			bt_node;
	hook_p			bt_hook;
	struct mbufq		bt_vq;		/* vendor commands to send */
	struct mbufq		bt_evq;		/* events made up for ng_hci */
	uint16_t		bt_vwait;	/* vendor opcode in flight */
	uint8_t			bt_raddr[6];	/* LE static random address */
	int			bt_vticks;
	u_int			bt_connects;	/* Connection Complete, status 0 */
	uint32_t		secondary_size;
	struct n1_wifi {		/* alpha (Wi-Fi) command rings */
		uint8_t		*win;
		uint64_t	win_iova;
		uint16_t	main_head, cmd_head, seq;
		uint16_t	rx_next[5], rx_head[5];
		uint16_t	reply_seq, reply_status;
		uint32_t	reply_cmd, channels, scans, beacons;
		int		reply_valid;
		uint8_t		mac[6];
		uint8_t		bss[32][6];
		int		nbss;
		/* join target, from hw.apple_n1.ssid / hw.apple_n1.pmk */
		char		ssid[33];
		uint8_t		pmk[32];
		int		have_pmk, found;
		uint8_t		bssid[6], chan, band;
		uint16_t	rsn_caps;
		int		connecting, connect_event, associated;
		uint16_t	connect_status;
		/* data path (aurora data_probe, drain_data_rx/tx) */
		uint32_t	data_done;
		int		connected;
		uint16_t	tx_len[128];
		uint8_t		tx_busy[128];
		uint32_t	rx_pkts, rx_bad, tx_pkts, tx_bad;
		if_t		ifp;
		struct mtx	mtx;
		struct callout	poll;
		struct mbufq	txq;
	}			w;
};

static struct n1_softc *n1_sc;	/* the control function, for n1fn */

#define	N1_RD4(sc, o)		bus_read_4((sc)->bar, (o))
#define	N1_WR4(sc, o, v)	bus_write_4((sc)->bar, (o), (v))

/*
 * Allocate page-aligned, physically contiguous memory and map it for the N1
 * at IOVAs just below 1 TiB (40 bits), where Linux's IOMMU layer would put
 * them for a 40-bit device. RAM itself sits above 1 TiB on these SoCs.
 */
/* hw.apple_n1.sid17=1: enable DART stream 17 as macOS does */
static int n1_sid17 = 0;
TUNABLE_INT("hw.apple_n1.sid17", &n1_sid17);

static void *
n1_dma_alloc(struct n1_softc *sc, size_t size, uint64_t *iova)
{
	void *buf;
	int error;

	size = roundup2(size, N1_PAGE);
	if (sc->iova_next + size > N1_IOVA_END)
		return (NULL);
	buf = contigmalloc(size, M_DEVBUF, M_WAITOK | M_ZERO, 0,
	    ~(vm_paddr_t)0, N1_PAGE, 0);
	if (buf == NULL)
		return (NULL);
	error = apple_dart_map_stream(sc->dev, sc->dart, sc->sid,
	    sc->iova_next, pmap_kextract((vm_offset_t)buf), size);
	if (error != 0) {
		device_printf(sc->dev, "DART map of %zu bytes: %d\n", size,
		    error);
		free(buf, M_DEVBUF);
		return (NULL);
	}
	for (u_int sid = 2; error == 0 && sc->routed && sid <= 3; sid++)
		error = apple_dart_map_stream(sc->dev, sc->dart, sid,
		    sc->iova_next, pmap_kextract((vm_offset_t)buf), size);
	if (error == 0 && sc->routed && n1_sid17)
		error = apple_dart_map_stream(sc->dev, sc->dart, 17,
		    sc->iova_next, pmap_kextract((vm_offset_t)buf), size);
	if (error != 0)
		device_printf(sc->dev, "DART function map: %d\n", error);
	*iova = sc->iova_next;
	sc->iova_next += size;
	return (buf);
}

/* The PCIe DART stream for this function, from the host bridge's iommu-map. */
static int
n1_find_stream(struct n1_softc *sc)
{
	phandle_t node;
	device_t d;

	node = -1;
	for (d = device_get_parent(sc->dev); d != NULL; d = device_get_parent(d)) {
		node = ofw_bus_get_node(d);
		if (node > 0 && OF_hasprop(node, "iommu-map"))
			break;
	}
	if (d == NULL)
		return (ENXIO);
	return (apple_dart_pci_stream(node, pci_get_rid(sc->dev), &sc->dart,
	    &sc->sid));
}

/* Inspect latched faults without acknowledging them. */
static void
n1_dart_errors(struct n1_softc *sc)
{
	bus_addr_t pa;
	bus_size_t size;
	vm_offset_t va;
	uint32_t pending, params;
	unsigned int offset, sid;

	if (ofw_reg_to_paddr(sc->dart, 0, &pa, &size, NULL) != 0)
		return;
	va = (vm_offset_t)pmap_mapdev(pa, size);
	params = *(volatile uint32_t *)(va + N1_DART_PARAMS3);
	if (N1_DART_VERSION(params) >= 0x0202 && size >= 0x8010) {
		/* The radio's streams are all in the first exception word. */
		pending = *(volatile uint32_t *)(va + N1_DART_EXCEPTIONS);
		N1_DPRINTF(sc, "DART exceptions %#x\n", pending);
		while (pending != 0) {
			sid = ffs(pending) - 1;
			pending &= pending - 1;
			offset = N1_DART_SID_FAULT(sid);
			if (offset + 0x10 > size)
				continue;
			device_printf(sc->dev, "DART SID %u status %#x "
			    "address %08x:%08x\n", sid,
			    *(volatile uint32_t *)(va + offset),
			    *(volatile uint32_t *)(va + offset + 0xc),
			    *(volatile uint32_t *)(va + offset + 8));
		}
	} else {
		device_printf(sc->dev, "DART error %#x addr %#x%08x\n",
		    *(volatile uint32_t *)(va + 0x100),
		    *(volatile uint32_t *)(va + 0x174),
		    *(volatile uint32_t *)(va + 0x170));
	}
	pmap_unmapdev((void *)va, size);

	/* Root port: interrupt status, link status (pcie-apple PORT_INTSTAT/LINKSTS). */
	va = (vm_offset_t)pmap_mapdev(0x488028000, 0x4000);
	N1_DPRINTF(sc, "port intstat %#x linksts %#x\n",
	    *(volatile uint32_t *)(va + 0x100), *(volatile uint32_t *)(va + 0x208));
	pmap_unmapdev((void *)va, 0x4000);
}

static int n1_corrupt;
TUNABLE_INT("hw.apple_n1.corrupt", &n1_corrupt);
/*
 * XXX bring-up: ticket placement. 0 none; 1 scatter entry 1 (flags 0); 2 entry 1, flags 1;
 * 3 ticket as entry 0, ftab as entry 1; 4 ticket IOVA/length in BAR0 0x8050/0x8054/0x8058.
 */
static int n1_tkt_mode;
TUNABLE_INT("hw.apple_n1.tkt_mode", &n1_tkt_mode);
static int n1_tkt_flags = 1;
TUNABLE_INT("hw.apple_n1.tkt_flags", &n1_tkt_flags);
static int n1_desc_len;		/* XXX bring-up: override the scatter length */
TUNABLE_INT("hw.apple_n1.desc_len", &n1_desc_len);

static int
n1_rom_boot(struct n1_softc *sc)
{
	const struct firmware *fw, *tkt;
	uint32_t stage, resp, desc_size;
	int cap, i;

	stage = N1_RD4(sc, N1_STAGE);
	if (stage != N1_STAGE_ROM) {
		device_printf(sc->dev, "stage %#x, not in ROM: no boot\n",
		    stage);
		return (EBUSY);
	}
	if (pci_find_cap(sc->dev, PCIY_MSI, &cap) != 0 ||
	    (pci_read_config(sc->dev, cap + PCIR_MSI_CTRL, 2) &
	    PCIM_MSICTRL_64BIT) == 0) {
		device_printf(sc->dev, "no 64-bit MSI capability\n");
		return (ENXIO);
	}

	fw = firmware_get_flags(sc->board->firmware, FIRMWARE_GET_NOWARN);
	if (fw == NULL) {
		device_printf(sc->dev, "no firmware /boot/firmware/%s\n",
		    sc->board->firmware);
		return (ENOENT);
	}
	if (fw->datasize < 0x30 || fw->datasize > N1_FW_MAX ||
	    memcmp((const char *)fw->data + 0x20, "rkosftab", 8) != 0) {
		device_printf(sc->dev, "%s: not an ftab\n",
		    sc->board->firmware);
		firmware_put(fw, FIRMWARE_UNLOAD);
		return (EINVAL);
	}
	sc->fw = n1_dma_alloc(sc, fw->datasize, &sc->fw_iova);
	sc->desc = n1_dma_alloc(sc, N1_PAGE, &sc->desc_iova);
	sc->mbox = n1_dma_alloc(sc, N1_PAGE, &sc->mbox_iova);
	if (sc->fw == NULL || sc->desc == NULL || sc->mbox == NULL) {
		firmware_put(fw, FIRMWARE_UNLOAD);
		return (ENOMEM);
	}
	memcpy(sc->fw, fw->data, fw->datasize);
	sc->fw_size = fw->datasize;
	if (n1_corrupt)		/* XXX bring-up: does the ROM parse it? */
		memset((uint8_t *)sc->fw + 0x20, 0, 16);
	/* Scatter entry: u64 address, u32 length (4K-aligned), u32 flags. */
	le64enc(sc->desc, sc->fw_iova);
	le32enc((uint8_t *)sc->desc + 8, n1_desc_len != 0 ? n1_desc_len :
	    roundup2(fw->datasize, 4096));
	desc_size = 0x20;
	/* XXX bring-up experiment: where the ROM takes the IMG4 ticket */
	tkt = n1_tkt_mode != 0 ? firmware_get_flags("apple/j815-centauri.im4m",
	    FIRMWARE_GET_NOWARN) : NULL;
	if (tkt != NULL) {
		void *tbuf;

		tbuf = n1_dma_alloc(sc, tkt->datasize, &sc->tkt_iova);
		if (tbuf != NULL) {
			memcpy(tbuf, tkt->data, tkt->datasize);
			cpu_dcache_wb_range(tbuf, roundup2(tkt->datasize, N1_PAGE));
			sc->tkt_size = tkt->datasize;
			switch (n1_tkt_mode) {
			case 1:
			case 2:
				le64enc((uint8_t *)sc->desc + 0x20, sc->tkt_iova);
				le32enc((uint8_t *)sc->desc + 0x28, tkt->datasize);
				le32enc((uint8_t *)sc->desc + 0x2c,
				    n1_tkt_mode == 2 ? n1_tkt_flags : 0);
				desc_size = 0x40;
				break;
			case 3:
				memmove((uint8_t *)sc->desc + 0x20, sc->desc, 0x20);
				memset(sc->desc, 0, 0x20);
				le64enc(sc->desc, sc->tkt_iova);
				le32enc((uint8_t *)sc->desc + 8, tkt->datasize);
				desc_size = 0x40;
				break;
			}
			N1_DPRINTF(sc, "ticket: %zu bytes, mode %d "
			    "flags %d\n", tkt->datasize, n1_tkt_mode, n1_tkt_flags);
		}
		firmware_put(tkt, FIRMWARE_UNLOAD);
	}
	cpu_dcache_wb_range(sc->fw, roundup2(fw->datasize, N1_PAGE));
	/* XXX bring-up: DART table vs. the buffer's physical pages */
	{
		size_t off, bad = 0;

		for (off = 0; off < fw->datasize; off += N1_PAGE)
			if (apple_dart_translate(sc->fw_iova + off) !=
			    pmap_kextract((vm_offset_t)sc->fw + off) && bad++ < 4)
				device_printf(sc->dev, "DART: iova %#lx -> %#lx, "
				    "want %#lx\n", sc->fw_iova + off,
				    apple_dart_translate(sc->fw_iova + off),
				    pmap_kextract((vm_offset_t)sc->fw + off));
		device_printf(sc->dev, "DART walk: %zu bad pages, fw pa %#lx, "
		    "pte[0] l1 %p\n", bad, pmap_kextract((vm_offset_t)sc->fw),
		    sc->fw);
	}
	cpu_dcache_wb_range(sc->desc, N1_PAGE);
	cpu_dcache_wbinv_range(sc->mbox, N1_PAGE);
	device_printf(sc->dev, "ROM boot: %s, %zu bytes at IOVA %#lx\n",
	    sc->board->firmware, fw->datasize, sc->fw_iova);
	firmware_put(fw, FIRMWARE_UNLOAD);

	pci_write_config(sc->dev, cap + PCIR_MSI_ADDR, (uint32_t)sc->mbox_iova,
	    4);
	pci_write_config(sc->dev, cap + PCIR_MSI_ADDR_HIGH,
	    (uint32_t)(sc->mbox_iova >> 32), 4);
	pci_write_config(sc->dev, cap + PCIR_MSI_DATA_64BIT, N1_MSI_DATA, 2);
	pci_write_config(sc->dev, cap + PCIR_MSI_CTRL,
	    (pci_read_config(sc->dev, cap + PCIR_MSI_CTRL, 2) &
	    ~PCIM_MSICTRL_MME_MASK) | PCIM_MSICTRL_MSI_ENABLE, 2);
	pci_enable_busmaster(sc->dev);

	N1_WR4(sc, N1_ROM_BOARD, sc->board->board_word);
	N1_WR4(sc, N1_ROM_CTRL, 0);
	N1_WR4(sc, N1_ROM_DESC_LO, (uint32_t)sc->desc_iova);
	N1_WR4(sc, N1_ROM_DESC_HI, (uint32_t)(sc->desc_iova >> 32));
	N1_WR4(sc, N1_ROM_DESC_SIZE, desc_size);
	if (n1_tkt_mode == 4 && sc->tkt_size != 0) {
		N1_WR4(sc, 0x8050, (uint32_t)sc->tkt_iova);
		N1_WR4(sc, 0x8054, (uint32_t)(sc->tkt_iova >> 32));
		N1_WR4(sc, 0x8058, sc->tkt_size);
	}
	dsb(sy);
	N1_WR4(sc, N1_ROM_DOORBELL, 1);

	resp = 0;
	for (i = 0; i < 1000 && (resp = N1_RD4(sc, N1_ROM_RESPONSE)) == 0; i++)
		pause("n1rom", hz / 50);
	for (; i < 2000 && (stage = N1_RD4(sc, N1_STAGE)) == N1_STAGE_ROM; i++)
		pause("n1rom", hz / 50);
	/* XXX bring-up: is a response "next image, please"? Ring again. */
	for (int r = 0; r < 3 && N1_RD4(sc, N1_STAGE) == N1_STAGE_ROM; r++) {
		uint32_t r2;

		N1_WR4(sc, N1_ROM_RESPONSE, 0);
		dsb(sy);
		N1_WR4(sc, N1_ROM_DOORBELL, 1);
		for (i = 0; i < 500 &&
		    (r2 = N1_RD4(sc, N1_ROM_RESPONSE)) == 0; i++)
			pause("n1rom", hz / 50);
		N1_DPRINTF(sc, "doorbell %d: response %#x stage %#x "
		    "(%d ms)\n", r + 2, r2, N1_RD4(sc, N1_STAGE), i * 20);
	}
	stage = N1_RD4(sc, N1_STAGE);
	cpu_dcache_inv_range(sc->mbox, N1_PAGE);
	N1_DPRINTF(sc, "ROM result: response %#x stage %#x mailbox %#x "
	    "(%d ms)\n", resp, stage, le32dec(sc->mbox), i * 20);
	n1_dart_errors(sc);
	for (i = 0; i < 0x100; i += 0x20)	/* XXX bring-up */
		N1_DPRINTF(sc, "%#x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
		    0x8000 + i, N1_RD4(sc, 0x8000 + i), N1_RD4(sc, 0x8004 + i),
		    N1_RD4(sc, 0x8008 + i), N1_RD4(sc, 0x800c + i),
		    N1_RD4(sc, 0x8010 + i), N1_RD4(sc, 0x8014 + i),
		    N1_RD4(sc, 0x8018 + i), N1_RD4(sc, 0x801c + i));
	return (stage == N1_STAGE_RUNNING ? 0 : EIO);
}

static void
n1_rmw(vm_offset_t va, bus_size_t off, uint32_t clear, uint32_t set)
{
	volatile uint32_t *r = (volatile uint32_t *)(va + off);

	*r = (*r & ~clear) | set;
}

static bool
n1_wait(vm_offset_t va, bus_size_t off, uint32_t mask, uint32_t want, int us)
{
	for (; us > 0; us -= 10) {
		if ((*(volatile uint32_t *)(va + off) & mask) == want)
			return (true);
		DELAY(10);
	}
	return ((*(volatile uint32_t *)(va + off) & mask) == want);
}

static bool
n1_wait_ltssm(device_t rp, uint32_t state, int ms)
{
	for (; ms > 0; ms--) {
		if ((pci_read_config(rp, RC_LTSSM, 4) & 0x3f) == state)
			return (true);
		DELAY(1000);
	}
	return (false);
}

/*
 * After the ROM has run the firmware (stage 4), cycle the port so the N1
 * re-enumerates in its firmware personality (functions 1901/1902/1903):
 * endpoint to D3hot, PME_Turn_Off and L2, PERST# asserted, the port reset
 * to its power-on state, then brought up and retrained as at boot. macOS
 * does the same (D3, disable, enable); the register order follows the
 * measured sequence.
 */
/*
 * XXX bring-up: N1 power off and on through SMC (gP13, gp0e; m1n1's
 * function-reset template), so the ROM boot follows power-on closely as
 * under macOS.
 */
static void
n1_power_cycle(struct n1_softc *sc)
{
	int error;

	error = apple_smc_write_u32(0x67503133, 0x800000);
	if (error == 0)
		error = apple_smc_write_u32(0x67703065, 0x800000);
	pause("n1off", hz / 4 + 1);
	if (error == 0)
		error = apple_smc_write_u32(0x67503133, 0x800001);
	if (error == 0)
		error = apple_smc_write_u32(0x67703065, 0x800001);
	pause("n1on", hz * 15 / 100 + 1);
	if (error != 0)
		device_printf(sc->dev, "N1 power cycle: %d\n", error);
}

static int
n1_port_cycle_power(struct n1_softc *sc, bool power)
{
	static const uint32_t reset[][2] = {
		{ 0x088, 0x110 }, { 0x100, ~0u }, { 0x148, ~0u },
		{ 0x210, ~0u }, { 0x080, 0 }, { 0x084, 0 },
		{ 0x104, 0xfffffff0 }, { 0x124, 0x100 }, { 0x16c, 0 },
		{ 0x13c, 0 }, { 0x800, 0x00100100 }, { 0x808, 0x001001ff },
		{ 0x82c, 0x00010000 }, { 0x130, 0x03020000 }, { 0x140, 0x10 },
		{ 0x144, 0x00253770 }, { 0x21c, 0 }, { 0x834, 0 },
		{ 0x83c, 0 },
	};
	device_t rp = device_get_parent(device_get_parent(sc->dev));
	vm_offset_t port, lane, pad;
	uint32_t *msimap, msicfg, msiaddr, padv, id, rpcfg[16];
	int error, i, pm;

	port = (vm_offset_t)pmap_mapdev(sc->board->port, 0x4000);
	lane = (vm_offset_t)pmap_mapdev(sc->board->lane, 0x4000);
	pad = (vm_offset_t)pmap_mapdev(trunc_page(sc->board->perst), PAGE_SIZE) +
	    (sc->board->perst & PAGE_MASK);
	msimap = malloc(256 * sizeof(*msimap), M_DEVBUF, M_WAITOK);
	error = ETIMEDOUT;

	/* pci_restore_state() leaves a bridge's windows out: keep the header */
	for (i = 1; i < 16; i++)
		rpcfg[i] = pci_read_config(rp, 4 * i, 4);
	pci_save_state(sc->dev);
	for (i = 0; i < 256; i++)
		msimap[i] = *(volatile uint32_t *)(port + PORT_MSIMAP + 4 * i);
	msicfg = *(volatile uint32_t *)(port + PORT_MSICFG);
	msiaddr = *(volatile uint32_t *)(port + PORT_MSIADDR);
	padv = *(volatile uint32_t *)pad;

	/* Endpoint to D3hot, then PME_Turn_Off and wait for L2 */
	if (pci_find_cap(sc->dev, PCIY_PMG, &pm) == 0) {
		pci_write_config(sc->dev, pm + PCIR_POWER_STATUS, 0x103, 2);
		pause("n1d3", hz / 100 + 1);
	}
	n1_rmw(port, PORT_APPCLK, 0, 0x100);
	*(volatile uint32_t *)(port + PORT_PME) = 0x11;
	if (!n1_wait(port, PORT_PME, 1, 0, 500000) ||
	    !n1_wait(port, PORT_LINKSTS, 1u << 6, 1u << 6, 500000))
		device_printf(sc->dev, "port cycle: no PME ack / L2\n");

	/* Link down: PERST# asserted, port quiesced and reset */
	pci_write_config(rp, PCIR_COMMAND, 0, 2);
	n1_rmw(lane, 0, 1u << 31, 0);
	*(volatile uint32_t *)pad = (padv & ~0xfu) | PAD_OUT_LOW;
	if (power)
		n1_power_cycle(sc);
	n1_wait(port, PORT_STATUS, 1u << 2, 0, 50000);
	n1_wait(port, PORT_LANES, 0x1ff, 0, 100000);
	n1_rmw(port, PORT_LINKDOWN, 0, 0x100);
	n1_wait_ltssm(rp, LTSSM_L2_IDLE, 200);
	n1_wait(port, PORT_LINKSTS, 1u << 12, 0, 200000);
	n1_wait(port, PORT_PIPE, 3, 0, 10000);
	n1_rmw(port, PORT_PERST, 0, 1u << 24);
	n1_wait(port, PORT_PIPE, 1u << 16, 1u << 16, 20000);
	n1_rmw(port, PORT_PERST, 0, 1u << 16);
	DELAY(10);
	n1_rmw(port, PORT_PERST, 1, 0);
	DELAY(10);
	n1_rmw(port, PORT_PERST, 1u << 24, 0);
	n1_rmw(port, PORT_LINKDOWN, 0x100, 0);
	n1_rmw(lane, 0, (1u << 31) | 0x600, 0x10);
	n1_rmw(lane, 0, 3, 0);
	*(volatile uint32_t *)(port + 0x100) = ~0u;
	n1_rmw(port, PORT_APPCLK, 1, 0);
	n1_wait(port, PORT_STATUS, 1, 0, 200000);
	for (i = 0; i < nitems(reset); i++)
		*(volatile uint32_t *)(port + reset[i][0]) = reset[i][1];
	for (i = 0; i < 16; i++)
		*(volatile uint32_t *)(port + PORT_RID2SID + 4 * i) = 0;
	for (i = 0; i < 256; i++)
		*(volatile uint32_t *)(port + PORT_MSIMAP + 4 * i) = 0;

	*(volatile uint32_t *)(port + 0x140) = 0x11;

	/*
	 * Bring-up as macOS does it: PERST bit 0 and the root port setup
	 * (lost in the port reset) before the lane clocks, PERST bit 16 kept.
	 */
	n1_rmw(port, PORT_APPCLK, 0, 1);
	n1_rmw(port, PORT_PERST, 0, 1);
	n1_rmw(port, PORT_APPCLK, 0x100, 0);
	{
		static const uint32_t rc[][3] = {
			{ 0x8bc, 0xbff49, 4 }, { 0x78, 0x100810, 4 },
			{ 0xb80, 0x1d1f3220, 4 }, { 0x890, 0x2000, 4 },
			{ 0x8a8, 0xc000061, 4 }, { 0x890, 0x1002000, 4 },
			{ 0x8a8, 0x4000021, 4 }, { 0xa0, 0x43, 2 },
			{ 0x710, 0x10120, 4 }, { 0x80c, 0x1301ff, 4 },
			{ 0x7c, 0x737814, 4 }, { 0x8bc, 0xbff48, 4 },
			{ 0x80, 0x10110400, 4 }, { 0x80c, 0x1301ff, 4 },
			{ 0x1a0, 0, 4 }, { 0x19c, 0x4055000f, 4 }, { 0x118, 0, 4 },
			{ 0x10c, 0x462030, 4 }, { 0x108, 0, 4 }, { 0x114, 0, 4 },
			{ 0x130, 0xff, 4 }, { 0x12c, 7, 4 }, { 0x78, 0x81f, 2 },
			{ 0x98, 0x400, 2 }, { 0x80, 0xc02, 2 }, { 0xa0, 0x43, 2 },
			{ 0xa8, 0, 2 }, { 0x2b8, 1, 4 },
		};
		for (i = 0; i < nitems(rc); i++)
			pci_write_config(rp, rc[i][0], rc[i][1], rc[i][2]);
	}
	n1_rmw(lane, 0, 0, 1);
	if (!n1_wait(lane, 0, 4, 4, 50000))
		goto out;
	n1_rmw(lane, 0, 0, 2);
	if (!n1_wait(lane, 0, 8, 8, 50000))
		goto out;
	n1_rmw(lane, 0, 0x10, 0);
	DELAY(100);
	n1_rmw(lane, 0, 0, 0x200);
	n1_rmw(lane, 0, 0, 0x400);
	DELAY(100);
	*(volatile uint32_t *)pad = (padv & ~0xfu) | PAD_OUT_HIGH;
	DELAY(100);
	n1_rmw(port, PORT_LTSSM_EN, 0, 1);
	pci_write_config(rp, 0x12c, 0, 4);
	n1_rmw(lane, 0, 0, 1u << 31);
	pci_write_config(rp, 0x2b8, 0, 4);
	if (!n1_wait_ltssm(rp, LTSSM_L0, 2000))
		goto out;
	pause("n1l0", hz / 10 + 1);
	for (i = 15; i > 0; i--)
		if (i != 2 && i != 3)
			pci_write_config(rp, 4 * i, rpcfg[i], 4);
	id = pci_read_config(sc->dev, PCIR_DEVVENDOR, 4);
	pci_restore_state(sc->dev);
	N1_DPRINTF(sc, "port cycle: link up, function 0 now %#x, "
	    "stage %#x\n", id, N1_RD4(sc, N1_STAGE));
	error = 0;
out:
	for (i = 0; i < 256; i++)
		*(volatile uint32_t *)(port + PORT_MSIMAP + 4 * i) = msimap[i];
	*(volatile uint32_t *)(port + PORT_MSIADDR) = msiaddr;
	*(volatile uint32_t *)(port + PORT_MSICFG) = msicfg;
	if (N1_PORT_MSI) {	/* XXX bring-up: port MSI doorbell as macOS */
		*(volatile uint32_t *)(port + PORT_MSIADDR) = 0xfffff000;
		*(volatile uint32_t *)(port + PORT_MSIADDR + 4) = 0;
		*(volatile uint32_t *)(port + PORT_MSICFG) = 1;
	}
	if (error != 0)
		device_printf(sc->dev, "port cycle timed out (status %#x "
		    "pipe %#x LTSSM %#x)\n",
		    *(volatile uint32_t *)(port + PORT_STATUS),
		    *(volatile uint32_t *)(port + PORT_PIPE),
		    pci_read_config(rp, RC_LTSSM, 4) & 0x3f);
	free(msimap, M_DEVBUF);
	pmap_unmapdev((void *)trunc_page(pad), PAGE_SIZE);
	pmap_unmapdev((void *)lane, 0x4000);
	pmap_unmapdev((void *)port, 0x4000);
	return (error);
}

static int
n1_port_cycle(struct n1_softc *sc)
{
	return (n1_port_cycle_power(sc, false));
}

/* An entry of an rkosftab image (header at 0x20 "rkosftab", count at 0x28) */
static int
n1_ftab_entry(const uint8_t *img, size_t size, const char tag[4],
    uint32_t *off, uint32_t *len)
{
	uint32_t i, n;

	if (size < 0x30 || memcmp(img + 0x24, "ftab", 4) != 0)
		return (EINVAL);
	n = le32dec(img + 0x28);
	if (n > 256 || 0x30 + 16 * n > size)
		return (EINVAL);
	for (i = 0; i < n; i++) {
		if (memcmp(img + 0x30 + 16 * i, tag, 4) != 0)
			continue;
		*off = le32dec(img + 0x34 + 16 * i);
		*len = le32dec(img + 0x38 + 16 * i);
		return (*off + (uint64_t)*len <= size ? 0 : EINVAL);
	}
	return (ENOENT);
}

/*
 * IPC context: a 32 KiB header + index arrays + rings block and a 16 KiB
 * message area (15 slots of 512 bytes after the first). Layout as the
 * firmware expects it: header version 768, size 0x68, the four index arrays
 * (cr heads, tr heads, cr tails, tr tails) from 0x78, the message transfer
 * and completion rings after them.
 */
static int
n1_ctx_init(struct n1_softc *sc, struct n1_ctx *c, uint16_t tr, uint16_t cr,
    uint64_t scratch, uint64_t scratch_size)
{
	uint8_t *h;
	int i;

	c->buf = n1_dma_alloc(sc, 32768, &c->buf_iova);
	c->msgs = n1_dma_alloc(sc, 16384, &c->msgs_iova);
	if (c->buf == NULL || c->msgs == NULL)
		return (ENOMEM);
	c->tr = tr;
	c->cr = cr;
	c->index[0] = 0x78;
	c->index[1] = c->index[0] + cr * 2;
	c->index[2] = c->index[1] + tr * 2;
	c->index[3] = c->index[2] + cr * 2;
	c->mtr = roundup2(c->index[3] + tr * 2, tr == 192 ? 16 : 2);
	c->mcr = roundup2(c->mtr + 256, 4);
	h = c->buf;
	le16enc(h, 768);
	le16enc(h + 2, 0x68);
	le64enc(h + 8, c->buf_iova + 0x68);
	for (i = 0; i < 4; i++)
		le64enc(h + 0x10 + 8 * i, c->buf_iova + c->index[i]);
	le16enc(h + 0x30, cr);
	le16enc(h + 0x32, tr);
	le64enc(h + 0x34, c->buf_iova + c->mcr);
	le64enc(h + 0x3c, c->buf_iova + c->mtr);
	le16enc(h + 0x44, 16);
	le16enc(h + 0x46, 16);
	le16enc(h + 0x4a, 0xffff);	/* as macOS (control and beta) */
	le64enc(h + 0x58, scratch != 0 ? scratch :
	    c->buf_iova + roundup2(c->mcr + 256, 8));
	le64enc(h + 0x60, scratch_size);
	cpu_dcache_wb_range(c->buf, 32768);
	return (0);
}

/* Context handshake: control 1 → status 1, context address, control 2 → status 2 */
static int
n1_handshake(struct resource *bar, struct n1_ctx *c, bus_size_t status,
    bus_size_t addr, bus_size_t control)
{
	int i;

	bus_write_4(bar, control, 1);
	for (i = 0; i < 200 && bus_read_4(bar, status) != 1; i++)
		pause("n1hs", hz / 100 + 1);
	if (bus_read_4(bar, status) != 1)
		return (ETIMEDOUT);
	bus_write_4(bar, addr, (uint32_t)c->buf_iova);
	bus_write_4(bar, addr + 4, (uint32_t)(c->buf_iova >> 32));
	dsb(sy);
	bus_write_4(bar, control, 2);
	for (i = 0; i < 200 && bus_read_4(bar, status) != 2; i++)
		pause("n1hs", hz / 100 + 1);
	return (bus_read_4(bar, status) == 2 ? 0 : ETIMEDOUT);
}

/*
 * After the port cycle macOS routes the three requester IDs to streams 1–3.
 * Keep all three streams on the private table with the already mapped image.
 */
static int
n1_route_functions(struct n1_softc *sc)
{
	volatile uint32_t *routes;
	unsigned int sid;
	int error, i;

	for (i = 0; i < nitems(sc->fn); i++) {
		sid = i + 1;
		error = apple_dart_private_stream(sc->fn[i], sc->dart, sid);
		if (error != 0)
			return (error);
	}
	routes = pmap_mapdev(sc->board->port + PORT_RID2SID, 16 * sizeof(*routes));
	for (i = 0; i < nitems(sc->fn); i++) {
		sid = i + 1;
		routes[sid] = PORT_RID2SID_VALID |
		    (sid << PORT_RID2SID_SID_SHIFT) | pci_get_rid(sc->fn[i]);
	}
	dsb(sy);
	pmap_unmapdev(__DEVOLATILE(void *, routes), 16 * sizeof(*routes));
	/* XXX bring-up: the port forwards MSIs to the AIC doorbell on SID 16 */
	if (N1_PORT_MSI) {
		error = apple_dart_private_stream(sc->dev, sc->dart, 16);
		if (error == 0)
			error = apple_dart_map_stream(sc->dev, sc->dart, 16,
			    0x381204000ul, 0x381204000ul, N1_PAGE);
		if (error != 0)
			device_printf(sc->dev, "SID 16 MSI doorbell map: %d\n", error);
	}
	/*
	 * XXX bring-up: macOS has stream 17 enabled with its own table before
	 * alpha starts (not in RID2SID); mirror the function mappings there.
	 */
	if (n1_sid17) {
		error = apple_dart_private_stream(sc->dev, sc->dart, 17);
		if (error == 0)
			error = apple_dart_map_stream(sc->dev, sc->dart, 17,
			    0x381204000ul, 0x381204000ul, N1_PAGE);
		if (error != 0)
			device_printf(sc->dev, "SID 17 enable: %d\n", error);
	}
	sc->sid = 1;
	sc->routed = 1;
	return (0);
}

/*
 * Firmware personality: MSI to a DMA mailbox per function (polled until
 * we have an MSI controller for the port) and bus mastering.
 */
static int
n1_functions(struct n1_softc *sc)
{
	/* 32 vectors to a DMA mailbox each, as aurora's enumerate_functions */
	static const uint16_t mme[] = {
		PCIM_MSICTRL_MME_32, PCIM_MSICTRL_MME_32, PCIM_MSICTRL_MME_32,
	};
	uint64_t iova;
	uint16_t command;
	int cap, error, i;

	sc->fn[0] = sc->dev;
	sc->fnbar[0] = sc->bar;
	for (i = 0; i < 3; i++) {
		if (sc->fn[i] == NULL || sc->fnbar[i] == NULL) {
			device_printf(sc->dev, "function %d missing\n", i);
			return (ENXIO);
		}
	}
	if ((error = n1_route_functions(sc)) != 0)
		return (error);
	for (i = 0; i < nitems(sc->fn); i++) {
		if (pci_find_cap(sc->fn[i], PCIY_MSI, &cap) != 0)
			return (ENXIO);
		if (i == 0) {
			iova = sc->mbox_iova;
		} else if (N1_PORT_MSI) {
			iova = 0xfffff000;	/* data 0x10 (alpha), 0 (beta) */
		} else {
			sc->fnmsi[i] = n1_dma_alloc(sc, N1_PAGE, &iova);
			if (sc->fnmsi[i] == NULL)
				return (ENOMEM);
		}
		pci_write_config(sc->fn[i], cap + PCIR_MSI_ADDR, (uint32_t)iova, 4);
		pci_write_config(sc->fn[i], cap + PCIR_MSI_ADDR_HIGH,
		    (uint32_t)(iova >> 32), 4);
		pci_write_config(sc->fn[i], cap + PCIR_MSI_DATA_64BIT,
		    i == 0 || !N1_PORT_MSI ? N1_MSI_DATA + 0x100 * i :
		    (i == 1 ? 0x10 : 0), 2);
		pci_write_config(sc->fn[i], cap + PCIR_MSI_CTRL,
		    (pci_read_config(sc->fn[i], cap + PCIR_MSI_CTRL, 2) &
		    ~PCIM_MSICTRL_MME_MASK) | mme[i] | PCIM_MSICTRL_MSI_ENABLE, 2);
		command = pci_read_config(sc->fn[i], PCIR_COMMAND, 2);
		pci_write_config(sc->fn[i], PCIR_COMMAND,
		    command | PCIM_CMD_BUSMASTEREN | PCIM_CMD_INTxDIS, 2);
	}
	return (0);
}

/*
 * PCIe Precision Time Measurement. The secondary loader runs a PTM exchange
 * before it validates the manifest (firmware "beforePTM"/"ptmDone" boot
 * timestamps in the macOS registry), so the root port has to answer.
 * Apple's sequence (macOS 26.4 kernelcache): AppleEmbeddedPCIE
 * _enableGTBToPTM sets common register 0x50 bit 0 (global timebase feeds
 * PTM); APCIECoreRCGen4Port::enablePortPTM turns the bridge PTM control
 * (config 0x2b8 bit 0) off, gates port config register 0x2b0 off and on,
 * loads the per-rate responder latencies (0x304 selects the rate, 0x2f4 and
 * 0x2f8 take AppleT8142PCIePort::_getLatency) and turns PTM back on.
 * IOPCIFamily then enables the requesters.
 */
#define	N1_COMMON		0x48c000000	/* apcie reg[1] */
#define	N1_PORT_CONFIG		0x488028000	/* apcie reg[8 + 6 * port] */
static const uint16_t n1_ptm_latency[4][2] = {
	{ 0xcb, 0x1e2 }, { 0x66, 0xf0 }, { 0x46, 0xc9 }, { 0x24, 0x65 },
};

static void
n1_enable_ptm(struct n1_softc *sc)
{
	device_t rp = device_get_parent(device_get_parent(sc->dev));
	vm_offset_t common, pcfg;
	uint32_t v;
	int cap, i;

	common = (vm_offset_t)pmap_mapdev(N1_COMMON, PAGE_SIZE);
	pcfg = (vm_offset_t)pmap_mapdev(N1_PORT_CONFIG, 0x4000);
	N1_DPRINTF(sc, "PTM: common 0x50 %#x, port cfg 0x2b0 %#x, "
	    "bridge 0x2b0 %08x %08x %08x, 0x2f4 %#x 0x2f8 %#x 0x304 %#x\n",
	    *(volatile uint32_t *)(common + 0x50),
	    *(volatile uint32_t *)(pcfg + 0x2b0),
	    pci_read_config(rp, 0x2b0, 4), pci_read_config(rp, 0x2b4, 4),
	    pci_read_config(rp, 0x2b8, 4), pci_read_config(rp, 0x2f4, 4),
	    pci_read_config(rp, 0x2f8, 4), pci_read_config(rp, 0x304, 4));

	*(volatile uint32_t *)(common + 0x50) |= 1;
	pci_write_config(rp, 0x2b8, pci_read_config(rp, 0x2b8, 4) & ~1u, 4);
	*(volatile uint32_t *)(pcfg + 0x2b0) = 0;
	*(volatile uint32_t *)(pcfg + 0x2b0) = 1;
	for (i = 0; i < nitems(n1_ptm_latency); i++) {
		v = pci_read_config(rp, 0x304, 4);
		pci_write_config(rp, 0x304, (v & ~0xfu) | i, 4);
		v = pci_read_config(rp, 0x2f4, 4);
		pci_write_config(rp, 0x2f4, (v & ~0xfffu) | n1_ptm_latency[i][0], 4);
		v = pci_read_config(rp, 0x2f8, 4);
		pci_write_config(rp, 0x2f8, (v & ~0xfffu) | n1_ptm_latency[i][1], 4);
	}
	/* Enable, plus Root Select as IOPCIFamily sets on a root-capable port */
	pci_write_config(rp, 0x2b8, pci_read_config(rp, 0x2b8, 4) | 3, 4);

	/* Requesters: PTM Enable in each function's PTM capability. */
	for (i = 0; i < nitems(sc->fn); i++) {
		if (pci_find_extcap(sc->fn[i], PCIZ_PTM, &cap) != 0)
			continue;
		pci_write_config(sc->fn[i], cap + 8,
		    pci_read_config(sc->fn[i], cap + 8, 4) | 1, 4);
		N1_DPRINTF(sc, "PTM: function %d cap %#x %08x ctl %08x\n",
		    i, cap, pci_read_config(sc->fn[i], cap + 4, 4),
		    pci_read_config(sc->fn[i], cap + 8, 4));
	}
	N1_DPRINTF(sc, "PTM: common 0x50 %#x, port cfg 0x2b0 %#x, "
	    "bridge ctl %#x\n", *(volatile uint32_t *)(common + 0x50),
	    *(volatile uint32_t *)(pcfg + 0x2b0), pci_read_config(rp, 0x2b8, 4));
	pmap_unmapdev((void *)pcfg, 0x4000);
	pmap_unmapdev((void *)common, PAGE_SIZE);
}

/* XXX bring-up: PCIe/AER state of the root port (and fn0 when safe) */
static void
n1_pcie_dump(struct n1_softc *sc, const char *when, int ep)
{
	device_t rp = device_get_parent(device_get_parent(sc->dev));
	device_t d[2] = { rp, sc->dev };
	int cap, i;

	for (i = 0; i < (ep ? 2 : 1); i++) {
		if (pci_find_cap(d[i], PCIY_EXPRESS, &cap) == 0)
			N1_DPRINTF(sc, "%s %s: devcap %08x devctl %04x "
			    "devsts %04x devcap2 %08x devctl2 %04x\n", when,
			    i ? "fn0" : "rp", pci_read_config(d[i], cap + 4, 4),
			    pci_read_config(d[i], cap + 8, 2),
			    pci_read_config(d[i], cap + 10, 2),
			    pci_read_config(d[i], cap + 0x24, 4),
			    pci_read_config(d[i], cap + 0x28, 2));
		if (pci_find_extcap(d[i], PCIZ_AER, &cap) == 0)
			N1_DPRINTF(sc, "%s %s AER: ue %08x mask %08x sev "
			    "%08x ce %08x hdr %08x %08x %08x %08x\n", when,
			    i ? "fn0" : "rp", pci_read_config(d[i], cap + 4, 4),
			    pci_read_config(d[i], cap + 8, 4),
			    pci_read_config(d[i], cap + 12, 4),
			    pci_read_config(d[i], cap + 16, 4),
			    pci_read_config(d[i], cap + 28, 4),
			    pci_read_config(d[i], cap + 32, 4),
			    pci_read_config(d[i], cap + 36, 4),
			    pci_read_config(d[i], cap + 40, 4));
	}
	if (pci_find_cap(rp, PCIY_EXPRESS, &cap) == 0)
		N1_DPRINTF(sc, "%s rp: secsts %04x rootsts %08x\n", when,
		    pci_read_config(rp, PCIR_SECSTAT_1, 2),
		    pci_read_config(rp, cap + 0x20, 4));
}

/*
 * Per-function memswap: after the control handshake each function gets its
 * own copy of its segment of the secondary ftab (mswc control, mswb beta,
 * msww alpha) through its config 0xf88/0xf8c/0xf90. Without msww alpha
 * stays at exec stage 0 (order and registers from aurora-silicon's
 * centauri_ipc.c memswap_handoff).
 */
static int
n1_memswap(struct n1_softc *sc, int fn, const char tag[4])
{
	uint32_t off, len;
	uint64_t iova;
	void *buf;
	int error;

	if ((error = n1_ftab_entry(sc->secondary, sc->secondary_size, tag,
	    &off, &len)) != 0)
		return (error);
	if ((buf = n1_dma_alloc(sc, len, &iova)) == NULL)
		return (ENOMEM);
	cpu_dcache_inv_range(sc->secondary, roundup2(sc->secondary_size,
	    N1_PAGE));
	memcpy(buf, (uint8_t *)sc->secondary + off, len);
	cpu_dcache_wb_range(buf, roundup2(len, N1_PAGE));
	pci_write_config(sc->fn[fn], 0xf88, (uint32_t)iova, 4);
	pci_write_config(sc->fn[fn], 0xf8c, (uint32_t)(iova >> 32), 4);
	pci_write_config(sc->fn[fn], 0xf90, len, 4);
	N1_DPRINTF(sc, "memswap fn%d: %.4s %u bytes at %#lx\n", fn, tag, len,
	    iova);
	return (0);
}

/*
 * Stage 1 → 2: hand the control function the secondary ftab ("2ftb" in the
 * image, seed of the memory it lays out) through config 0xf88/0xf8c/0xf90,
 * set up the control context, point the boot registers at the image again
 * and ring 0x9070 (AirshipDK t2026-CentauriControl.plist "boot" stage);
 * then the control IPC handshake.
 */
static const uint32_t n1_img_holes[][2] = {
	/* HOLES */
	{ 0, 0 },
};
/* XXX bring-up: unmapped holes in the secondary (memswap) buffer */
#define	N1_MSWB_PROT	0ul
#define	N1_SEC_IOVA	0ul
#define	N1_NO_RO	0
#define	N1_CRASH	0
#define	N1_AOP_RUN	0
#define	N1_POISON	0
#define	N1_LTR	0
static const uint32_t n1_sec_holes[][2] = {
	/* SECHOLES */
	{ 0, 0 },
};

static int n1_ctrl_rings(struct n1_softc *);
#define	N1_AER(sc, what)	/* XXX bring-up */				\
	N1_DPRINTF((sc), "AER %s: fn1 ue %#x devsts %#x, fn2 ue %#x\n",	\
	    what, pci_read_config((sc)->fn[1], 0x104, 4),			\
	    pci_read_config((sc)->fn[1], 0x7a, 2),				\
	    pci_read_config((sc)->fn[2], 0x104, 4))
static int n1_fn_start(struct n1_softc *, int, struct n1_ctx *);
static int n1w_start(struct n1_softc *);
static int n1_cchi_init(struct n1_softc *);
static int n1w_join(struct n1_softc *);
static int n1w_attach_if(struct n1_softc *);
static int n1w_data_ring(struct n1_softc *, unsigned int);
static void n1w_target(struct n1_softc *);
static int n1_alpha_start(struct n1_softc *);
static int n1w_scan(struct n1_softc *);
static int n1_bt_start(struct n1_softc *);
static struct ng_type n1bt_typestruct;

/* hw.apple_n1.bt_debug=1: print received Bluetooth packets */
static int n1_bt_debug = 0;
TUNABLE_INT("hw.apple_n1.bt_debug", &n1_bt_debug);

static int
n1_start_ipc(struct n1_softc *sc)
{
	uint32_t off, len, n, i, need, end, stage;
	const uint8_t *seed, *written;
	int error;

	if ((error = n1_functions(sc)) != 0)
		return (error);
	N1_AER(sc, "functions");
	n1_enable_ptm(sc);
	if ((error = n1_ftab_entry(sc->fw, sc->fw_size, "2ftb", &off,
	    &len)) != 0) {
		device_printf(sc->dev, "no 2ftb in the image: %d\n", error);
		return (error);
	}
	seed = (const uint8_t *)sc->fw + off;
	if (len < 0x30 || memcmp(seed + 0x24, "ftab", 4) != 0)
		return (EINVAL);
	n = le32dec(seed + 0x28);
	if (n == 0 || n > 256 || 0x30 + 16 * n != len)
		return (EINVAL);
	need = len;
	for (i = 0; i < n; i++) {
		end = le32dec(seed + 0x34 + 16 * i) + le32dec(seed + 0x38 + 16 * i);
		need = MAX(need, end);
	}
	{	/* XXX bring-up: choose the memswap IOVA */
		uint64_t save = sc->iova_next;

		if (N1_SEC_IOVA != 0)
			sc->iova_next = N1_SEC_IOVA;
		sc->secondary = n1_dma_alloc(sc, need, &sc->secondary_iova);
		if (N1_SEC_IOVA != 0)
			sc->iova_next = save;
	}
	if (sc->secondary == NULL)
		return (ENOMEM);
	sc->secondary_size = need;
	memcpy(sc->secondary, seed, len);
	/* XXX bring-up: poison mswb to detect device writes */
	if (N1_POISON)
		memset((uint8_t *)sc->secondary + 0x200300, 0xa5, 0x180000);
	cpu_dcache_wb_range(sc->secondary, roundup2(need, N1_PAGE));
	if (n1_sec_holes[0][1] != 0) {
		uint64_t base = sc->iova_next;
		uint32_t pg;
		int h, hole;

		for (pg = 0; pg < roundup2(need, N1_PAGE); pg += N1_PAGE) {
			for (h = 0, hole = 0; n1_sec_holes[h][1] != 0; h++)
				if (pg >= n1_sec_holes[h][0] && pg < n1_sec_holes[h][1])
					hole = 1;
			if (!hole)
				apple_dart_map_stream(sc->dev, sc->dart, sc->sid,
				    base + pg, pmap_kextract((vm_offset_t)
				    sc->secondary + pg), N1_PAGE);
		}
		sc->iova_next = base + roundup2(need, N1_PAGE);
		sc->secondary_iova = base;
		for (h = 0; n1_sec_holes[h][1] != 0; h++)
			N1_DPRINTF(sc, "secondary probe: memswap at %#lx, "
			    "hole %#x-%#x\n", base, n1_sec_holes[h][0],
			    n1_sec_holes[h][1]);
	}
	/* XXX bring-up: DART protection bits on the mswb page (7 no-write, 8 no-read) */
	if (N1_MSWB_PROT != 0)
		N1_DPRINTF(sc, "mswb page prot %#lx: %d\n",
		    (u_long)N1_MSWB_PROT, apple_dart_pte_bits(sc->dev, sc->dart,
		    sc->sid, sc->secondary_iova + 0x200000, N1_MSWB_PROT));
	pci_write_config(sc->dev, 0xf88, (uint32_t)sc->secondary_iova, 4);
	pci_write_config(sc->dev, 0xf8c, (uint32_t)(sc->secondary_iova >> 32), 4);
	pci_write_config(sc->dev, 0xf90, need, 4);

	N1_DPRINTF(sc, "secondary memory: seed %u, %u bytes\n", len,
	    need);

	if ((error = n1_ctx_init(sc, &sc->control, 13, 1, 0, 0)) != 0)
		return (error);
	/*
	 * XXX bring-up: resend the image page by page with unmapped holes; the
	 * DART fault then names the firmware's first access inside a hole.
	 */
	if (n1_img_holes[0][1] != 0) {
		uint64_t base, iova;
		uint32_t pg;
		void *p;
		int h, hole;

		base = sc->iova_next;
		for (pg = 0; pg < sc->fw_size; pg += N1_PAGE) {
			for (h = 0, hole = 0; n1_img_holes[h][1] != 0; h++)
				if (pg >= n1_img_holes[h][0] &&
				    pg < n1_img_holes[h][1])
					hole = 1;
			sc->iova_next = base + pg;
			if (hole)
				continue;
			if ((p = n1_dma_alloc(sc, N1_PAGE, &iova)) == NULL)
				break;
			memcpy(p, (uint8_t *)sc->fw + pg,
			    MIN(N1_PAGE, sc->fw_size - pg));
			cpu_dcache_wb_range(p, N1_PAGE);
		}
		sc->iova_next = base + roundup2(sc->fw_size, N1_PAGE);
		le64enc(sc->desc, base);
		cpu_dcache_wb_range(sc->desc, N1_PAGE);
		for (h = 0; n1_img_holes[h][1] != 0; h++)
			N1_DPRINTF(sc, "secondary probe: image at %#lx, "
			    "hole %#x-%#x\n", base, n1_img_holes[h][0],
			    n1_img_holes[h][1]);
	}
	{	/* XXX bring-up: relaxed ordering off on fn0 and the root port */
		device_t rp = device_get_parent(device_get_parent(sc->dev));
		int c;

		if (N1_NO_RO && pci_find_cap(sc->dev, PCIY_EXPRESS, &c) == 0)
			pci_write_config(sc->dev, c + 8,
			    pci_read_config(sc->dev, c + 8, 2) & ~0x10, 2);
		if (N1_NO_RO && pci_find_cap(rp, PCIY_EXPRESS, &c) == 0)
			pci_write_config(rp, c + 8,
			    pci_read_config(rp, c + 8, 2) & ~0x10, 2);
	}
	n1_pcie_dump(sc, "before", 1);
	{	/* XXX bring-up: VC capability (TC/VC map) on fn0 and the root port */
		device_t dv[2] = { device_get_parent(device_get_parent(sc->dev)),
		    sc->dev };
		int c, k, w;

		for (k = 0; k < 2; k++) {
			if (pci_find_extcap(dv[k], PCIZ_VC, &c) != 0) {
				N1_DPRINTF(sc, "VC %s: none\n", k ? "fn0" : "rp");
				continue;
			}
			for (w = 0; w < 0x30; w += 0x10)
				N1_DPRINTF(sc, "VC %s %#x+%#x: %08x %08x %08x %08x\n",
				    k ? "fn0" : "rp", c, w,
				    pci_read_config(dv[k], c + w, 4),
				    pci_read_config(dv[k], c + w + 4, 4),
				    pci_read_config(dv[k], c + w + 8, 4),
				    pci_read_config(dv[k], c + w + 12, 4));
		}
		for (k = 0x18c; k <= 0x298; k += 0x298 - 0x18c)	/* Apple VSECs */
			for (w = 0; w < 0x40; w += 0x10)
				N1_DPRINTF(sc, "VSEC %#x+%#x: %08x %08x %08x %08x\n",
				    k, w, pci_read_config(sc->dev, k + w, 4),
				    pci_read_config(sc->dev, k + w + 4, 4),
				    pci_read_config(sc->dev, k + w + 8, 4),
				    pci_read_config(sc->dev, k + w + 12, 4));
		if (N1_LTR) {	/* LTR as macOS: enable on rp + fn0, max latency from the ADT */
			for (k = 0; k < 2; k++)
				if (pci_find_cap(dv[k], PCIY_EXPRESS, &c) == 0)
					pci_write_config(dv[k], c + 0x28,
					    pci_read_config(dv[k], c + 0x28, 2) | 0x400, 2);
			pci_write_config(sc->dev, 0x174 + 4, 0x08100810, 4);
			N1_DPRINTF(sc, "LTR on: fn0 ltr %08x\n",
			    pci_read_config(sc->dev, 0x174 + 4, 4));
		}
		for (k = 0x100, c = 0; k && c < 20; c++) {	/* fn0 ext cap list */
			w = pci_read_config(sc->dev, k, 4);
			N1_DPRINTF(sc, "fn0 extcap %#x: id %#x ver %u\n", k,
			    w & 0xffff, (w >> 16) & 0xf);
			k = (w >> 20) & 0xffc;
		}
	}
	if (N1_AOP_RUN) {	/* XXX bring-up: start the (iBoot-preloaded) AOP */
		vm_offset_t a = (vm_offset_t)pmap_mapdev(0x390600000, PAGE_SIZE);

		N1_DPRINTF(sc, "AOP cpu_control %#x -> RUN\n",
		    *(volatile uint32_t *)(a + 0x44));
		*(volatile uint32_t *)(a + 0x44) |= 0x10;
		DELAY(500000);
		N1_DPRINTF(sc, "AOP cpu_control %#x mbox %#x\n",
		    *(volatile uint32_t *)(a + 0x44),
		    *(volatile uint32_t *)(a + 0x48));
		pmap_unmapdev((void *)a, PAGE_SIZE);
	}
	N1_WR4(sc, N1_ROM_DESC_LO, (uint32_t)sc->desc_iova);
	N1_WR4(sc, N1_ROM_DESC_HI, (uint32_t)(sc->desc_iova >> 32));
	N1_WR4(sc, N1_ROM_DESC_SIZE, 0x20);
	N1_WR4(sc, 0x8074, 0x62746632);	/* "2ftb" */
	dsb(sy);
	/*
	 * The firmware doesn't answer BAR reads while it brings up the
	 * secondary stage, and an unanswered read resets the SoC. Like macOS,
	 * wait for its exec-stage interrupt (MSI to the mailbox) first.
	 */
	le32enc(sc->mbox, 0);
	cpu_dcache_wbinv_range(sc->mbox, N1_PAGE);
	{
		/* XXX bring-up: timeline of safe state (no BAR access) */
		device_t rp = device_get_parent(device_get_parent(sc->dev));
		bus_addr_t dpa;
		bus_size_t dsz;
		vm_offset_t dva, pva, gva;
		uint32_t v[7], pv[7] = { ~0u, ~0u, ~0u, ~0u, ~0u, ~0u, ~0u };
		int k;

		ofw_reg_to_paddr(sc->dart, 0, &dpa, &dsz, NULL);
		dva = (vm_offset_t)pmap_mapdev(dpa, dsz);
		pva = (vm_offset_t)pmap_mapdev(0x488028000, 0x4000);
		/* centauri function-error_detect: aop-gpio pin 0x2d */
		gva = (vm_offset_t)pmap_mapdev(0x390824000, PAGE_SIZE);
		N1_DPRINTF(sc, "error_detect gpio before doorbell %#x\n",
		    *(volatile uint32_t *)(gva + 4 * 0x2d));
		N1_WR4(sc, 0x9070, 1);
		for (i = 0; i < 500; i++) {
			cpu_dcache_inv_range(sc->mbox, N1_PAGE);
			v[0] = pci_read_config(rp, RC_LTSSM, 4) & 0x3f;
			v[1] = pci_read_config(rp, 0x82, 2);
			v[2] = *(volatile uint32_t *)(dva + 0x100);
			v[3] = *(volatile uint32_t *)(dva + 0x170);
			v[4] = *(volatile uint32_t *)(pva + 0x100);
			v[5] = le32dec(sc->mbox);
			v[6] = *(volatile uint32_t *)(gva + 4 * 0x2d);
			for (k = 0; k < 7 && v[k] == pv[k]; k++)
				;
			if (k < 7) {
				N1_DPRINTF(sc, "XXX t=%d ltssm %#x lnk %#x "
				    "darterr %#x addr %#x intstat %#x mbox %#x "
				    "errgpio %#x\n", i * 10, v[0], v[1], v[2], v[3],
				    v[4], v[5], v[6]);
				memcpy(pv, v, sizeof(v));
				DELAY(50000);
			}
			if (v[5] != 0)
				break;
			DELAY(10000);
		}
		pmap_unmapdev((void *)gva, PAGE_SIZE);
		pmap_unmapdev((void *)pva, 0x4000);
		pmap_unmapdev((void *)dva, dsz);
	}
	N1_DPRINTF(sc, "exec-stage MSI %#x after ~%d ms\n",
	    le32dec(sc->mbox), i * 10);
	n1_dart_errors(sc);
	if (le32dec(sc->mbox) == 0) {
		/* Check for device writes without accessing the endpoint. */
		cpu_dcache_inv_range(sc->secondary, roundup2(need, N1_PAGE));
		written = memcchr((const uint8_t *)sc->secondary + 0x200300, 0xa5,
		    0x180000);
		n1_pcie_dump(sc, "after", 0);
		if (N1_CRASH) {	/* XXX bring-up: TriggerChipCrash, then look for a crashlog */
			const uint8_t *b = sc->secondary;
			uint32_t o, k;

			N1_DPRINTF(sc, "TriggerChipCrash (config 0xf84 = 1)\n");
			DELAY(100000);
			pci_write_config(sc->dev, 0xf84, 1, 4);
			N1_DPRINTF(sc, "crash written, waiting\n");
			DELAY(3000000);
			cpu_dcache_inv_range(sc->secondary, roundup2(need, N1_PAGE));
			for (o = len, k = 0; o < need && k < 48; o += 64) {
				uint32_t j, nz = 0;

				for (j = 0; j < 64; j++)
					if (b[o + j] != 0 && !(o + j >= 0x200300 &&
					    o + j < 0x380300 && b[o + j] == 0xa5))
						nz = 1;
				if (!nz)
					continue;
				k++;
				N1_DPRINTF(sc, "sec %#x: %32D\n", o, b + o, "");
				N1_DPRINTF(sc, "sec %#x: %32D\n", o + 32, b + o + 32, "");
			}
			N1_DPRINTF(sc, "crash scan: %u nonzero lines\n", k);
		}
		N1_DPRINTF(sc, "mswb poison %s (first %#tx)\n",
		    written ? "CHANGED" : "intact", written ? written -
		    (const uint8_t *)sc->secondary : 0);
		written = memcchr((const uint8_t *)sc->secondary + len, 0,
		    0x200300 - len);
		if (written != NULL)
			N1_DPRINTF(sc, "secondary memory populated "
			    "at offset %#tx\n",
			    written - (const uint8_t *)sc->secondary);
		else
			device_printf(sc->dev, "secondary memory unchanged beyond seed\n");
		return (ETIMEDOUT);
	}
	stage = N1_RD4(sc, N1_STAGE);
	device_printf(sc->dev, "secondary boot: stage %#x\n", stage);
	N1_AER(sc, "secondary");
	n1_dart_errors(sc);
	if (stage != 2)
		return (EIO);
	error = n1_handshake(sc->bar, &sc->control, 0x8050, 0x8054, 0x907c);
	device_printf(sc->dev, "control IPC handshake: %d\n", error);
	if (error != 0)
		return (error);
	N1_AER(sc, "handshake");
	/* From here as aurora's centauri_start_ipc */
	if ((error = n1_memswap(sc, 0, "mswc")) != 0 ||
	    (error = n1_memswap(sc, 2, "mswb")) != 0 ||
	    (error = n1_memswap(sc, 1, "msww")) != 0) {
		device_printf(sc->dev, "memswap: %d\n", error);
		return (error);
	}
	if ((error = n1_cchi_init(sc)) != 0) {
		device_printf(sc->dev, "CCHI: %d\n", error);
		return (error);
	}
	n1w_target(sc);
	if ((error = n1_alpha_start(sc)) == 0 && (error = n1w_start(sc)) == 0)
		error = n1w_scan(sc);
	if (error == 0 && sc->w.ssid[0] != '\0' && sc->w.have_pmk &&
	    (error = n1w_join(sc)) == 0)
		error = n1w_attach_if(sc);
	device_printf(sc->dev, "wifi: %d\n", error);
	/* Beta as before (btcentauri needs only control booted, beta at 2) */
	if ((error = n1_fn_start(sc, 2, &sc->beta)) != 0)
		return (error);
	device_printf(sc->dev, "bluetooth: %d\n", n1_bt_start(sc));
	return (0);
}

/*
 * Control transfer ring 0 carries 0x80-byte messages from the message area;
 * an entry is u32 (length << 8 | 1), u64 message address. The tail goes to
 * the tr tail array and to doorbell 0x9000 + 4 * ring; the firmware moves the
 * tr head when it has taken the message.
 */
static int
n1_msg_send(struct n1_ctx *c, struct resource *bar, bus_size_t db,
    const void *msg, uint32_t len)
{
	uint16_t slot = c->head % 16;
	uint8_t *ent = c->buf + c->mtr + 16 * slot;
	int i;

	memset(c->msgs + 0x80 * slot, 0, 0x80);
	memcpy(c->msgs + 0x80 * slot, msg, len);
	le32enc(ent, len << 8 | 1);
	le64enc(ent + 4, c->msgs_iova + 0x80 * slot);
	le32enc(ent + 12, db == 0x9000 ? 0 : slot);	/* beta: slot, as macOS */
	c->head++;
	le16enc(c->buf + c->index[3], c->head);
	cpu_dcache_wb_range(c->buf, 32768);
	cpu_dcache_wb_range(c->msgs, 16384);
	bus_write_4(bar, db, c->head);
	for (i = 0; i < 100; i++) {
		cpu_dcache_inv_range(c->buf, PAGE_SIZE);
		if (le16dec(c->buf + c->index[1]) == c->head) {
			/* consume the completion as macOS does: cr tail = cr head */
			le16enc(c->buf + c->index[2], le16dec(c->buf + c->index[0]));
			cpu_dcache_wb_range(c->buf, PAGE_SIZE);
			return (0);
		}
		pause("n1msg", hz / 100 + 1);
	}
	return (ETIMEDOUT);
}

static int
n1_ctrl_send(struct n1_softc *sc, const void *msg, uint32_t len)
{

	return (n1_msg_send(&sc->control, sc->bar, 0x9000, msg, len));
}

/*
 * Control receive (ring 2, 16 slots): print what the firmware returned and
 * give each buffer back at the tail, which the doorbell takes modulo 16.
 */
static void
n1_rx_scan(struct n1_softc *sc)
{
	struct n1_ctx *c = &sc->control;
	uint8_t *e, *b;
	uint32_t len, id, j, k;
	int posted = 0;

	if (sc->ring2 == NULL)
		return;
	cpu_dcache_inv_range(sc->ring2, N1_PAGE);
	while (sc->ring2[16 * (sc->rx_seen % 16) + 15] != 0) {
		e = sc->ring2 + 16 * (sc->rx_seen % 16);
		id = le32dec(e + 12) & 0xffffff;
		b = sc->rx + 0x8000 * (id % 15);
		len = MIN(le32dec(e) >> 8, 0x8000);
		cpu_dcache_inv_range(b, 0x8000);
		N1_DPRINTF(sc, "rx %d: type %u len %#x\n", sc->rx_seen,
		    e[0], len);
		if (len <= 0x40) {
			for (j = 0; j < len; j += 32)
				N1_DPRINTF(sc, "rx %d +%#04x: %32D\n",
				    sc->rx_seen, j, b + j, " ");
		} else {
			for (j = 0; j < len; j = k + 1) {
				for (k = j; k < len && b[k] >= 0x20 &&
				    b[k] < 0x7f; k++)
					;
				if (k - j >= 4)
					N1_PRINTF("n1log: %.*s\n", (int)(k - j),
					    b + j);
			}
			for (j = len > 0x200 ? len - 0x200 : 0; j < len; j += 32)
				N1_DPRINTF(sc, "rx %d +%#06x: %32D\n",
				    sc->rx_seen, j, b + j, " ");
		}
		memset(e, 0, 16);
		e = sc->ring2 + 16 * (sc->rx_posted % 16);
		le16enc(e, 1);
		le16enc(e + 2, 0x80);
		le64enc(e + 4, sc->rx_iova + 0x8000 * (id % 15));
		le32enc(e + 12, id % 15);
		sc->rx_posted++;
		sc->rx_seen++;
		posted = 1;
	}
	if (posted) {
		cpu_dcache_wb_range(sc->ring2, N1_PAGE);
		le16enc(c->buf + c->index[3] + 2 * 2, sc->rx_posted % 16);
		cpu_dcache_wb_range(c->buf, PAGE_SIZE);
		N1_WR4(sc, 0x9008, sc->rx_posted % 16);
	}
}

/* Rings 2, 4 and 3 as macOS creates them: 16 entries of 0x40 bytes */
static int __unused
n1_ctrl_rings(struct n1_softc *sc)
{
	static const struct { uint16_t id, flags; } rings[] = {
		{ 2, 0 }, { 4, 0x4010 }, { 3, 0x4010 },
	};
	uint8_t msg[0x34], *ring[5], *rx, *e;
	uint64_t iova, rx_iova;
	int error, i, j, n;

	for (i = 0; i < nitems(rings); i++) {
		if ((ring[rings[i].id] = n1_dma_alloc(sc, N1_PAGE, &iova)) == NULL)
			return (ENOMEM);
		memset(msg, 0, sizeof(msg));
		le16enc(msg, 1);
		le16enc(msg + 2, rings[i].flags);
		le16enc(msg + 4, rings[i].id);
		le16enc(msg + 6, rings[i].id);
		le64enc(msg + 8, iova);
		le64enc(msg + 0x10, ~0ul);
		le16enc(msg + 0x18, 16);
		le16enc(msg + 0x1a, 0xffff);
		le16enc(msg + 0x1c, rings[i].id);
		le16enc(msg + 0x1e, 0x40);
		error = n1_ctrl_send(sc, msg, sizeof(msg));
		N1_DPRINTF(sc, "create ring %u: %d (tr0 head %u)\n",
		    rings[i].id, error, le16dec(sc->control.buf +
		    sc->control.index[1]));
		if (error != 0)
			return (error);
		if (rings[i].id != 2)
			continue;

		/*
		 * Ring 2 receives: 16-byte entries u16 1, u16 size >> 8, u64
		 * buffer, u32 index; the firmware fills a buffer, writes the
		 * length into bits 8-31 of the first word and sets 4 in the
		 * last byte. macOS posts 15 of 32 KiB, and posts and recycles
		 * them before it creates rings 4 and 3.
		 */
		if ((rx = n1_dma_alloc(sc, 15 * 0x8000, &rx_iova)) == NULL)
			return (ENOMEM);
		for (j = 0; j < 15; j++) {
			e = ring[2] + 16 * j;
			le16enc(e, 1);
			le16enc(e + 2, 0x80);
			le64enc(e + 4, rx_iova + 0x8000 * j);
			le32enc(e + 12, j);
		}
		cpu_dcache_wb_range(ring[2], N1_PAGE);
		le16enc(sc->control.buf + sc->control.index[3] + 2 * 2, 15);
		cpu_dcache_wb_range(sc->control.buf, PAGE_SIZE);
		N1_WR4(sc, 0x9008, 15);
		sc->rx_iova = rx_iova;
		sc->rx_posted = 15;
		sc->rx = rx;
		sc->ring2 = ring[2];
		for (j = 0; j < 100; j++) {
			pause("n1rx", hz / 100 + 1);
			n1_rx_scan(sc);
		}
		n = sc->rx_seen;
		N1_DPRINTF(sc, "ring 2: %d buffers returned, tr heads %*D\n",
		    n, 10, sc->control.buf + sc->control.index[1], " ");
	}

	/*
	 * Ring 4 takes firmware messages in host-posted 0x40-byte slots; ring 3
	 * carries host messages: u8 type, u8 length, u16 0, u64 ~0, u32 flags,
	 * u32 length, u32 0, payload. Type 2 opens "CentauriTransport", after
	 * which macOS sees the Wi-Fi function at stage 2.
	 */
	memset(ring[4], 0, N1_PAGE);
	memset(ring[3], 0, N1_PAGE);
	le32enc(ring[4], 0x00040000);
	le64enc(ring[4] + 4, ~0ul);
	ring[3][0] = 2;
	ring[3][1] = 0x1a;
	le64enc(ring[3] + 4, ~0ul);
	le32enc(ring[3] + 0x10, 0x1a);
	memcpy(ring[3] + 0x18, "CentauriTransport", sizeof("CentauriTransport"));
	cpu_dcache_wb_range(ring[4], N1_PAGE);
	cpu_dcache_wb_range(ring[3], N1_PAGE);
	le16enc(sc->control.buf + sc->control.index[3] + 2 * 4, 7);
	le16enc(sc->control.buf + sc->control.index[3] + 2 * 3, 1);
	cpu_dcache_wb_range(sc->control.buf, PAGE_SIZE);
	N1_WR4(sc, 0x9010, 7);
	/* macOS sees alpha at stage 2 here, before its first ring 3 message */
	for (i = 0; i < 100 && bus_read_4(sc->fnbar[1], 0x8) != 2; i++) {
		pause("n1fn1", hz / 100 + 1);
		n1_rx_scan(sc);
	}
	N1_PRINTF("apple_n1: alpha stage %#x %d ms after ring 4 post\n",
	    bus_read_4(sc->fnbar[1], 0x8), i * 10);
	N1_WR4(sc, 0x900c, 1);
	pause("n1r34", hz);
	cpu_dcache_inv_range(sc->control.buf, PAGE_SIZE);
	cpu_dcache_inv_range(ring[3], N1_PAGE);
	cpu_dcache_inv_range(ring[4], N1_PAGE);
	N1_DPRINTF(sc, "after rings 3/4: heads %*D tails %*D\n",
	    10, sc->control.buf + sc->control.index[1], " ",
	    10, sc->control.buf + sc->control.index[3], " ");
	for (i = 3; i <= 4; i++)
		for (j = 0; j < 16 * 0x40; j += 32)
			for (n = 0; n < 32; n++)
				if (ring[i][j + n] != 0) {
					N1_DPRINTF(sc, "ring%d +%#05x: %32D\n",
					    i, j, ring[i] + j, " ");
					break;
				}
	for (i = 0; i < 500 && bus_read_4(sc->fnbar[1], 0x8) != 2; i++) {
		pause("n1fn1", hz / 100 + 1);
		n1_rx_scan(sc);
	}
	N1_DPRINTF(sc, "wifi function stage %#x after %d ms\n",
	    bus_read_4(sc->fnbar[1], 0x8), i * 10);
	for (i = 1; i < 3; i++) {	/* XXX bring-up: does the BAR answer? */
		uint32_t v, before, after;

		pci_write_config(sc->fn[i], 0x7a, 0xf, 2);
		before = pci_read_config(sc->fn[i], 0x7a, 2);
		v = bus_read_4(sc->fnbar[i], 0x8);
		after = pci_read_config(sc->fn[i], 0x7a, 2);
		N1_DPRINTF(sc, "fn%d BAR read %#x: devsts %#x -> %#x, "
		    "cmd %#x bar0 %#x rman %#jx-%#jx\n", i, v, before, after,
		    pci_read_config(sc->fn[i], 4, 2),
		    pci_read_config(sc->fn[i], 0x10, 4),
		    rman_get_start(sc->fnbar[i]), rman_get_end(sc->fnbar[i]));
	}
	for (i = 1; i < 3; i++)	/* XXX bring-up */
		N1_DPRINTF(sc, "fn%d %#x bar %#jx: %08x %08x %08x %08x "
		    "%08x %08x %08x %08x\n", i, pci_get_devid(sc->fn[i]),
		    rman_get_start(sc->fnbar[i]), bus_read_4(sc->fnbar[i], 0),
		    bus_read_4(sc->fnbar[i], 4), bus_read_4(sc->fnbar[i], 8),
		    bus_read_4(sc->fnbar[i], 0xc), bus_read_4(sc->fnbar[i], 0x10),
		    bus_read_4(sc->fnbar[i], 0x14), bus_read_4(sc->fnbar[i], 0x18),
		    bus_read_4(sc->fnbar[i], 0x1c));
	return (0);
}

/*
 * Data functions under macOS: alpha (Wi-Fi, function 1, BAR 0xb82010000)
 * has 192 transfer and 234 completion rings; beta (function 2, BAR
 * 0xb82000000) 22 and 1. Each takes a two-step handshake on its control
 * register; before the second step macOS passes the context and a DMA
 * aperture (base, then a 32-bit size running to the top of its IOVA window).
 * Alpha reaches stage 2 only after beta's handshake.
 */
#define	N1_APERTURE_SIZE	0x40000000u
#define	W_WINDOW		(16 * 1024 * 1024)	/* alpha: rings, scratch */
#define	W_SCRATCH		0x800000

static const struct n1_fn_regs {
	const char	*name;
	bus_size_t	stage, status, ctx, aperture, control;
	uint16_t	tr, cr;
	uint32_t	w50, scratch, scratch_size;
} n1_fn_regs[3] = {
	[1] = { "alpha", 0x8, 0xc, 0x10, 0x18, 0x1028, 192, 234,
	    0, 0x920, 0x500000 },
	[2] = { "beta", 0x18, 0x0, 0x4, 0xc, 0x107c, 22, 1,
	    0x04000000, 0x3d8, 0 },
};

/* XXX bring-up: dump a context like hv_trace_n1_ipc.py to diff with macOS */
static void
n1_ctx_dump(struct n1_softc *sc, const char *name, struct n1_ctx *c,
    size_t len)
{
	char line[3 * 32 + 1];
	size_t o;
	int i;

	cpu_dcache_inv_range(c->buf, len);
	for (o = 0; o < len; o += 32) {
		for (i = 0; i < 32; i++)
			snprintf(line + 3 * i, 4, " %02x", c->buf[o + i]);
		N1_PRINTF("n1ctx %s +0x%03zx:%s\n", name, o, line);
	}
}

/* XXX bring-up: print the nonzero words of a function BAR */
static void
n1_bar_scan(struct n1_softc *sc, int fn)
{
	struct resource *bar = sc->fnbar[fn];
	bus_size_t o, size = rman_get_size(bar);
	uint32_t v;
	int n = 0;

	for (o = 0; o < size && n < 200; o += 4) {
		v = bus_read_4(bar, o);
		if (v != 0 && v != 0xffffffff) {
			N1_PRINTF("n1bar fn%d +0x%04jx = %#x\n", fn, (uintmax_t)o, v);
			n++;
		}
	}
	N1_PRINTF("n1bar fn%d: %d nonzero words in %#jx\n", fn, n, (uintmax_t)size);
}

static int
n1_fn_start(struct n1_softc *sc, int fn, struct n1_ctx *c)
{
	const struct n1_fn_regs *r = &n1_fn_regs[fn];
	struct resource *bar = sc->fnbar[fn];
	uint64_t ap;
	int error, i;

	if (fn == 1) {
		n1_ctx_dump(sc, "control", &sc->control, 0x200);
		n1_ctx_dump(sc, "beta", &sc->beta, 0x100);
		n1_bar_scan(sc, 1);
		n1_bar_scan(sc, 2);
	}
	for (i = 0; i < 200 && bus_read_4(bar, r->stage) != 2; i++)
		pause("n1fn", hz / 100 + 1);
	if ((error = n1_ctx_init(sc, c, r->tr, r->cr, 0, r->scratch_size)) != 0)
		return (error);
	le32enc(c->buf + 0x50, r->w50);
	le64enc(c->buf + 0x58, c->buf_iova + r->scratch);
	le32enc(c->buf + 0x68, 2);
	le32enc(c->buf + 0x6c, 2);
	cpu_dcache_wb_range(c->buf, PAGE_SIZE);
	ap = N1_IOVA_BASE;	/* the whole DMA window, as macOS */
	n1_rx_scan(sc);
	N1_DPRINTF(sc, "%s: stage %#x after %d ms, context %#lx\n",
	    r->name, bus_read_4(bar, r->stage), i * 10, c->buf_iova);
	bus_write_4(bar, r->control, 1);
	for (i = 0; i < 200 && bus_read_4(bar, r->status) != 1; i++)
		pause("n1hs", hz / 100 + 1);
	if (bus_read_4(bar, r->status) != 1) {
		device_printf(sc->dev, "%s: no handshake step 1 (%#x)\n",
		    r->name, bus_read_4(bar, r->status));
		return (ETIMEDOUT);
	}
	bus_write_4(bar, r->aperture, (uint32_t)ap);
	bus_write_4(bar, r->aperture + 4, (uint32_t)(ap >> 32));
	bus_write_4(bar, r->aperture + 8, N1_APERTURE_SIZE);
	bus_write_4(bar, r->ctx, (uint32_t)c->buf_iova);
	bus_write_4(bar, r->ctx + 4, (uint32_t)(c->buf_iova >> 32));
	dsb(sy);
	bus_write_4(bar, r->control, 2);
	for (i = 0; i < 200 && bus_read_4(bar, r->status) != 2; i++)
		pause("n1hs", hz / 100 + 1);
	N1_DPRINTF(sc, "%s: handshake status %#x\n", r->name,
	    bus_read_4(bar, r->status));
	n1_dart_errors(sc);
	return (bus_read_4(bar, r->status) == 2 ? 0 : ETIMEDOUT);
}

/*
 * Control messages as aurora's centauri_ipc.c message(): 512-byte slots
 * after the first, entry u32 (length << 8 | 1), u64 address, u32 head + 1;
 * the doorbell takes head + 1 and the firmware moves the tr head to it.
 */
static int
n1_cmsg(struct n1_softc *sc, struct n1_ctx *c, struct resource *bar,
    bus_size_t db, const void *msg, uint32_t len)
{
	uint16_t head = c->head;
	uint8_t *ent;
	int i;

	if (head >= 15 || len == 0 || len > 512)
		return (ENOSPC);
	memcpy(c->msgs + 512 * (head + 1), msg, len);
	ent = c->buf + c->mtr + 16 * head;
	le32enc(ent, 1 | len << 8);
	le64enc(ent + 4, c->msgs_iova + 512 * (head + 1));
	le32enc(ent + 12, head + 1);
	cpu_dcache_wb_range(c->msgs, 16384);
	cpu_dcache_wb_range(c->buf, PAGE_SIZE);
	bus_write_4(bar, db, head + 1);
	c->head++;
	for (i = 0; i < 120; i++) {
		cpu_dcache_inv_range(c->buf, PAGE_SIZE);
		if (le16dec(c->buf + c->index[1]) == head + 1)
			return (0);
		pause("n1cm", hz / 100 + 1);
	}
	return (ETIMEDOUT);
}

/* Open a control transfer ring on a host buffer (aurora open_transfer) */
static int
n1_open_transfer(struct n1_softc *sc, uint16_t id, uint64_t iova,
    uint16_t footer)
{
	uint8_t msg[0x34] = { 1 };
	uint32_t words = footer >> 2, shift = MIN(ffs(words) - 1, 4);

	msg[2] = words >> shift;
	msg[3] = shift << 4;
	le16enc(msg + 4, id);
	le64enc(msg + 8, iova);
	le64enc(msg + 0x10, ~0ul);
	le16enc(msg + 0x18, 16);
	le16enc(msg + 0x1a, 0xffff);
	le16enc(msg + 0x1c, id);
	le16enc(msg + 0x1e, 0x40);
	return (n1_cmsg(sc, &sc->control, sc->bar, 0x9000, msg, sizeof(msg)));
}

/*
 * CCHI (aurora cchi_init): config message {9, 0, ff, ff}, rings 4 (CCHI
 * receive, 1 KiB footers), 3 (CCHI transmit) and 5 (debug receive, 256),
 * 15 receive buffers on 4 and 5, then a 64-byte hello on 3 and its answer.
 */
static int
n1_cchi_init(struct n1_softc *sc)
{
	static const uint8_t config[4] = { 9, 0, 0xff, 0xff };
	uint8_t *rx, *tx, *dbg, *e;
	uint64_t rx_iova, tx_iova, dbg_iova;
	int error, i;

	rx = n1_dma_alloc(sc, 32768, &rx_iova);
	tx = n1_dma_alloc(sc, 32768, &tx_iova);
	dbg = n1_dma_alloc(sc, 16384, &dbg_iova);
	if (rx == NULL || tx == NULL || dbg == NULL)
		return (ENOMEM);
	if ((error = n1_cmsg(sc, &sc->control, sc->bar, 0x9000, config,
	    sizeof(config))) != 0 ||
	    (error = n1_open_transfer(sc, 4, rx_iova, 1024)) != 0 ||
	    (error = n1_open_transfer(sc, 3, tx_iova, 1024)) != 0 ||
	    (error = n1_open_transfer(sc, 5, dbg_iova, 256)) != 0)
		return (error);
	for (i = 0; i < 15; i++) {
		e = rx + i * 1040;
		le32enc(e, 1 | 1024 << 8);
		le32enc(e + 12, i + 1);
		e = dbg + i * 272;
		le32enc(e, 1 | 256 << 8);
		le32enc(e + 12, i + 1);
	}
	cpu_dcache_wb_range(rx, 32768);
	cpu_dcache_wb_range(dbg, 16384);
	N1_WR4(sc, 0x9010, 15);
	N1_WR4(sc, 0x9014, 15);
	le32enc(tx, 1 | 64 << 8);
	le32enc(tx + 12, 1);
	le16enc(tx + 16, 64);		/* CCHI gid 0, oid 0, seq 0 hello */
	cpu_dcache_wb_range(tx, 32768);
	N1_WR4(sc, 0x900c, 1);
	for (i = 0; i < 200; i++) {
		cpu_dcache_inv_range(sc->control.buf, PAGE_SIZE);
		if (le16dec(sc->control.buf + sc->control.index[1] + 4 * 2) != 0)
			break;
		pause("n1cchi", hz / 100 + 1);
	}
	if (i == 200)
		return (ETIMEDOUT);
	cpu_dcache_inv_range(rx, 32768);
	if ((le32dec(rx) >> 8) < 8 || rx[18] != 0 || rx[19] != 0)
		return (EPROTO);
	device_printf(sc->dev, "control CCHI hello answered\n");
	return (0);
}

/*
 * Alpha (aurora centauri_start_ipc): wait for stage 2, a 16 MiB window with
 * the firmware's 5 MiB scratch at +8 MiB, the context (192 transfer, 234
 * completion rings), the window at +0x18/+0x1c/+0x20, then the handshake on
 * +0x1028 (status +0xc, context address +0x10).
 */
static int
n1_alpha_start(struct n1_softc *sc)
{
	struct resource *bar = sc->fnbar[1];
	struct n1_ctx *c = &sc->wifi;
	int error, i;

	for (i = 0; i < 200 && bus_read_4(bar, 0x8) != 2; i++)
		pause("n1fn", hz / 100 + 1);
	if (bus_read_4(bar, 0x8) != 2) {
		device_printf(sc->dev, "alpha: stage %#x\n", bus_read_4(bar, 0x8));
		return (EIO);
	}
	if ((sc->w.win = n1_dma_alloc(sc, W_WINDOW, &sc->w.win_iova)) == NULL)
		return (ENOMEM);
	if ((error = n1_ctx_init(sc, c, 192, 234, sc->w.win_iova + W_SCRATCH,
	    5 * 1024 * 1024)) != 0)
		return (error);
	bus_write_4(bar, 0x18, (uint32_t)sc->w.win_iova);
	bus_write_4(bar, 0x1c, (uint32_t)(sc->w.win_iova >> 32));
	bus_write_4(bar, 0x20, W_WINDOW);
	error = n1_handshake(bar, c, 0xc, 0x10, 0x1028);
	device_printf(sc->dev, "alpha: handshake %d\n", error);
	return (error);
}

/*
 * Wi-Fi on alpha, ported from aurora-silicon's centauri_wifi.c (J813). The
 * host owns a 16 MiB window: completion rings from +1 MiB and transfer rings
 * from +4 MiB (0x20000 apart, ring id - 1), command buffers at +6 MiB and
 * receive buffers for rings 2-4 after them; the firmware's scratch is at
 * +8 MiB (5 MiB). Ring pairs are opened with messages on alpha's message ring
 * (512-byte slots, doorbell +0x1004). Ring 1 carries commands (24-byte
 * header: u16 length at 0x10, u16 sequence 0x12, u32 id 0x14), rings 2 and 3
 * replies and events (bit 31 of the id), ring 4 beacons and probe responses.
 */
#define	W_CR_BASE	0x100000
#define	W_TR_BASE	0x400000
#define	W_SPAN		0x20000
#define	W_TX_BUF	0x600000
#define	W_BUFSZ		2048

static const uint32_t w_rxbuf[5] = { 0, 0, 0x620000, 0x680000, 0x700000 };
static const uint16_t w_rsize[5] = { 0, 64, 192, 96, 192 };

static uint16_t
n1w_idx(struct n1_softc *sc, int array, int i)
{
	struct n1_ctx *c = &sc->wifi;

	cpu_dcache_inv_range(c->buf, PAGE_SIZE);
	return (le16dec(c->buf + c->index[array] + 2 * i));
}

static void
n1w_idx_set(struct n1_softc *sc, int array, int i, uint16_t v)
{
	struct n1_ctx *c = &sc->wifi;

	le16enc(c->buf + c->index[array] + 2 * i, v);
	cpu_dcache_wb_range(c->buf, PAGE_SIZE);
}

static int
n1w_up(struct n1_softc *sc)
{
	return (bus_read_4(sc->fnbar[1], 0x8) == 2);
}

/* Completion tails follow the heads (rings 0-4) */
static void
n1w_completions(struct n1_softc *sc)
{
	int v;

	for (v = 0; v <= 4; v++)
		n1w_idx_set(sc, 2, v, n1w_idx(sc, 0, v));
}

static int
n1w_msg(struct n1_softc *sc, const void *m, size_t len)
{
	struct n1_ctx *c = &sc->wifi;
	uint16_t head = sc->w.main_head;
	uint8_t *d;
	int i;

	if (head >= 15 || len > 512)
		return (ENOSPC);
	memcpy(c->msgs + (head + 1) * 512, m, len);
	d = c->buf + c->mtr + 16 * head;
	le32enc(d, 1 | len << 8);
	le64enc(d + 4, c->msgs_iova + (head + 1) * 512);
	le32enc(d + 12, head + 1);
	cpu_dcache_wb_range(c->msgs, 16384);
	n1w_idx_set(sc, 3, 0, head + 1);
	if (head != 0)
		n1w_idx_set(sc, 3, 8, head + 1);
	sc->w.main_head++;
	bus_write_4(sc->fnbar[1], 0x1004, 1);
	for (i = 0; i < 200; i++) {
		if (!n1w_up(sc))
			return (EIO);
		if (n1w_idx(sc, 1, 0) >= head + 1) {
			n1w_completions(sc);
			return (0);
		}
		pause("n1wm", hz / 100 + 1);
	}
	return (ETIMEDOUT);
}

static int
n1w_open_pair(struct n1_softc *sc, uint16_t id)
{
	uint8_t cr[0x2c] = { 2 }, tr[0x34] = { 1 };
	int error;

	le32enc(cr + 4, id | id << 16);
	le64enc(cr + 8, sc->w.win_iova + W_CR_BASE + (id - 1) * W_SPAN);
	le16enc(cr + 0x10, 1024);
	le16enc(cr + 0x12, 0xffff);
	if ((error = n1w_msg(sc, cr, sizeof(cr))) != 0)
		return (error);
	le32enc(tr + 4, id | id << 16);
	le64enc(tr + 8, sc->w.win_iova + W_TR_BASE + (id - 1) * W_SPAN);
	le64enc(tr + 0x10, ~0ul);
	le16enc(tr + 0x18, w_rsize[id]);
	le16enc(tr + 0x1a, id);
	le16enc(tr + 0x1c, id);
	le16enc(tr + 0x1e, 0x40);
	le16enc(tr + 0x24, 2);
	return (n1w_msg(sc, tr, sizeof(tr)));
}

static void
n1w_post(struct n1_softc *sc, int ring, uint16_t slot)
{
	uint8_t *d = sc->w.win + W_TR_BASE + (ring - 1) * W_SPAN + 16 * slot;
	uint32_t off = w_rxbuf[ring] + slot * W_BUFSZ;

	memset(sc->w.win + off, 0, W_BUFSZ);
	cpu_dcache_wb_range(sc->w.win + off, W_BUFSZ);
	le32enc(d, 1 | W_BUFSZ << 8);
	le64enc(d + 4, sc->w.win_iova + off);
	le32enc(d + 12, slot + 1);
	cpu_dcache_wb_range(d, 16);
}

/* A beacon or probe response (ring 4): 0x60-byte descriptor, then the frame */
static void
n1w_beacon(struct n1_softc *sc, const uint8_t *p, size_t size)
{
	const uint8_t *f = p + 0x60, *ie, *end;
	char ssid[33];
	int chan = 0, rssi, i;

	if (size < 0x60 + 36 || (f[0] != 0x80 && f[0] != 0x50))
		return;
	sc->w.beacons++;
	for (i = 0; i < sc->w.nbss; i++)
		if (memcmp(sc->w.bss[i], f + 16, 6) == 0)
			return;
	if (sc->w.nbss < (int)nitems(sc->w.bss))
		memcpy(sc->w.bss[sc->w.nbss++], f + 16, 6);
	end = p + size;
	ssid[0] = '\0';
	for (ie = f + 36; ie + 2 <= end && ie + 2 + ie[1] <= end;
	    ie += 2 + ie[1]) {
		if (ie[0] == 0 && ie[1] <= 32) {
			memcpy(ssid, ie + 2, ie[1]);
			ssid[ie[1]] = '\0';
		}
		if ((ie[0] == 3 || ie[0] == 61) && ie[1] != 0)
			chan = ie[2];
	}
	rssi = MAX((int8_t)p[0x16], (int8_t)p[0x17]);
	if (sc->w.ssid[0] != '\0' && strcmp(ssid, sc->w.ssid) == 0 &&
	    !sc->w.found) {
		sc->w.found = 1;
		memcpy(sc->w.bssid, f + 16, 6);
		sc->w.chan = chan;
		sc->w.band = chan > 14;
		/* RSN element: version, group, pairwise and AKM lists, caps */
		for (ie = f + 36; ie + 2 <= end && ie + 2 + ie[1] <= end;
		    ie += 2 + ie[1]) {
			const uint8_t *q = ie + 2, *e = ie + 2 + ie[1];

			if (ie[0] != 48 || ie[1] < 8)
				continue;
			q += 2 + 4;
			if (q + 2 <= e)
				q += 2 + 4 * le16dec(q);
			if (q + 2 <= e)
				q += 2 + 4 * le16dec(q);
			if (q + 2 <= e)
				sc->w.rsn_caps = le16dec(q);
		}
	}
	device_printf(sc->dev, "wifi: bss %6D chan %3d rssi %4d \"%s\"\n",
	    f + 16, ":", chan, rssi, ssid);
}

static void
n1w_drain(struct n1_softc *sc)
{
	int ring, posted = 0;

	for (ring = 2; ring <= 4; ring++) {
		int budget = w_rsize[ring];

		while (budget--) {
			uint16_t slot = sc->w.rx_next[ring], size, status;
			uint8_t *d = sc->w.win + W_TR_BASE + (ring - 1) * W_SPAN +
			    16 * slot;
			uint8_t *p = sc->w.win + w_rxbuf[ring] + slot * W_BUFSZ;
			uint32_t cmd;

			cpu_dcache_inv_range(d, 16);
			if ((d[15] & 4) == 0)
				break;
			cpu_dcache_inv_range(p, W_BUFSZ);
			size = le16dec(p + 0x10);
			status = le16dec(p + 0xe);
			cmd = le32dec(p + 0x14) & ~(1u << 30);
			sc->w.rx_next[ring] = (slot + 1) % w_rsize[ring];
			if (ring != 4)
				N1_DPRINTF(sc, "wifi: rx%d slot %u desc %16D "
				    "pkt %32D\n", ring, slot, d, " ", p, " ");
			if (size < 24 || size > W_BUFSZ)
				;
			else if (ring == 4)
				n1w_beacon(sc, p + 24, size - 24);
			else if (cmd & (1u << 31)) {
				if (cmd == 0x80000030)
					sc->w.channels++;
				else if (cmd == 0x80000005)
					sc->w.scans++;
				else if (cmd == 0x80000000 && sc->w.connecting) {
					sc->w.connect_event = 1;
					sc->w.connect_status = status;
				} else if (cmd == 0x80000001) {
					sc->w.associated = status == 0;
					device_printf(sc->dev, "wifi: "
					    "association event status %u\n",
					    status);
				} else if (cmd == 0x80000050 && size >= 64)
					device_printf(sc->dev, "wifi: "
					    "supplicant result %u phase %u\n",
					    le32dec(p + 0x18), p[0x1c]);
				else
					N1_DPRINTF(sc, "wifi: event %#x status "
					    "%u size %u\n", cmd, status, size);
			} else {
				sc->w.reply_valid = 1;
				sc->w.reply_seq = le16dec(p + 0x12);
				sc->w.reply_status = status;
				sc->w.reply_cmd = cmd;
			}
			n1w_post(sc, ring, sc->w.rx_head[ring]);
			sc->w.rx_head[ring] = (sc->w.rx_head[ring] + 1) %
			    w_rsize[ring];
			n1w_idx_set(sc, 3, ring, sc->w.rx_head[ring]);
			posted = 1;
		}
	}
	n1w_completions(sc);
	if (posted)
		bus_write_4(sc->fnbar[1], 0x1004, 1);
}

/* XXX bring-up: alpha ring state */
static void
n1w_dump(struct n1_softc *sc)
{
	static const char *names[4] = { "cr heads", "tr heads", "cr tails",
	    "tr tails" };
	uint8_t *d;
	int a, ring;

	for (a = 0; a < 4; a++)
		device_printf(sc->dev, "wifi: %s %u %u %u %u %u (msg %u)\n",
		    names[a], n1w_idx(sc, a, 0), n1w_idx(sc, a, 1),
		    n1w_idx(sc, a, 2), n1w_idx(sc, a, 3), n1w_idx(sc, a, 4),
		    n1w_idx(sc, a, 8));
	for (ring = 1; ring <= 3; ring++) {
		d = sc->w.win + W_TR_BASE + (ring - 1) * W_SPAN;
		cpu_dcache_inv_range(d, 32);
		device_printf(sc->dev, "wifi: tr%d %16D\n", ring, d, " ");
		d = sc->w.win + W_CR_BASE + (ring - 1) * W_SPAN;
		cpu_dcache_inv_range(d, 32);
		device_printf(sc->dev, "wifi: cr%d %32D\n", ring, d, " ");
	}
	n1_ctx_dump(sc, "alpha", &sc->wifi, 0x80);
	device_printf(sc->dev, "wifi: stage %#x\n",
	    bus_read_4(sc->fnbar[1], 0x8));
}

static int
n1w_cmd(struct n1_softc *sc, uint32_t id, const void *payload, size_t size)
{
	uint16_t slot = sc->w.cmd_head, seq = ++sc->w.seq;
	uint8_t *p, *d;
	int i;

	if (size + 24 > W_BUFSZ)
		return (EMSGSIZE);
	if ((slot + 1) % w_rsize[1] == n1w_idx(sc, 1, 1))
		return (ENOSPC);
	p = sc->w.win + W_TX_BUF + slot * W_BUFSZ;
	memset(p, 0, 24 + size);
	le16enc(p + 0x10, 24 + size);
	le16enc(p + 0x12, seq);
	le32enc(p + 0x14, id);
	if (size != 0)
		memcpy(p + 24, payload, size);
	cpu_dcache_wb_range(p, 24 + size);
	d = sc->w.win + W_TR_BASE + 16 * slot;
	le32enc(d, 1 | (24 + size) << 8);
	le64enc(d + 4, sc->w.win_iova + W_TX_BUF + slot * W_BUFSZ);
	le32enc(d + 12, slot + 1);
	cpu_dcache_wb_range(d, 16);
	sc->w.cmd_head = (slot + 1) % w_rsize[1];
	n1w_idx_set(sc, 3, 1, sc->w.cmd_head);
	sc->w.reply_valid = 0;
	bus_write_4(sc->fnbar[1], 0x1004, 1);
	for (i = 0; i < 150; i++) {
		n1w_drain(sc);
		if (!n1w_up(sc))
			return (EIO);
		if (sc->w.reply_valid && sc->w.reply_seq == seq &&
		    sc->w.reply_cmd == id) {
			if (sc->w.reply_status != 0)
				device_printf(sc->dev, "wifi: command %#x "
				    "status %u\n", id, sc->w.reply_status);
			return (sc->w.reply_status != 0 ? EIO : 0);
		}
		pause("n1wc", hz / 100 + 1);
	}
	device_printf(sc->dev, "wifi: command %#x timed out\n", id);
	n1w_dump(sc);
	return (ETIMEDOUT);
}

/* Commands 0 (init), 1 (MAC address), 2 (startInterface) */
static int
n1w_start(struct n1_softc *sc)
{
	static const uint8_t start[5] = { 0, 0x0b, 0, 0, 0 };
	int error, ring, slot;

	for (ring = 1; ring <= 4; ring++)
		if ((error = n1w_open_pair(sc, ring)) != 0) {
			device_printf(sc->dev, "wifi: ring pair %d: %d\n",
			    ring, error);
			return (error);
		}
	for (ring = 2; ring <= 4; ring++) {
		for (slot = 0; slot < w_rsize[ring] - 1; slot++)
			n1w_post(sc, ring, slot);
		sc->w.rx_head[ring] = w_rsize[ring] - 1;
		n1w_idx_set(sc, 3, ring, w_rsize[ring] - 1);
	}
	bus_write_4(sc->fnbar[1], 0x1004, 1);
	pause("n1ws", hz * 4 / 10);
	arc4rand(sc->w.mac, sizeof(sc->w.mac), 0);
	sc->w.mac[0] = (sc->w.mac[0] & ~1) | 2;	/* local, unicast */
	if ((error = n1w_cmd(sc, 0, NULL, 0)) != 0 ||
	    (error = n1w_cmd(sc, 1, sc->w.mac, 6)) != 0 ||
	    (error = n1w_cmd(sc, 2, start, sizeof(start))) != 0)
		return (error);
	pause("n1ws", hz);
	device_printf(sc->dev, "wifi: interface started, MAC %6D\n",
	    sc->w.mac, ":");
	return (0);
}

/*
 * Scan (command 0x1000f, aurora scan_worker and build_scan) in chunks of 13
 * entries of the channel table (2.4 GHz 1-13, then 5 GHz). The request is
 * 14 fixed bytes (chunk number first, dwell times 110 ms at +8/+10/+12);
 * J815's macOS 27 adapter appends TLV 0x104 (count, then {channel, band, 0,
 * 0}) and TLV 0x165 (OCE, flags bit 0). The firmware reports each finished
 * channel (event 0x80000030) and the end (0x80000005). A chunk runs to its
 * end; it's stopped (command 0x10001) when all its channels reported without
 * an end event, or after a second without progress. A refused stop means
 * the chunk had already ended.
 */
static const uint8_t w_5ghz[] = {
	36, 40, 44, 48, 52, 56, 60, 64, 100, 104, 108, 112,
	116, 120, 124, 128, 132, 136, 140, 144, 149, 153, 157, 161, 165,
};
#define	W_NCHAN		(13 + nitems(w_5ghz))
#define	W_DWELL		110

static void
n1w_tlv(uint8_t *buf, size_t *size, uint16_t tag, const void *data,
    uint16_t len)
{
	le16enc(buf + *size, tag);
	le16enc(buf + *size + 2, len);
	memcpy(buf + *size + 4, data, len);
	*size += 4 + len;
}

static size_t
n1w_scan_req(int chunk, uint8_t *buf)
{
	static const uint8_t fixed[14] = { 1, 0, 0, 0x80, 0, 2, 0, 0x10 };
	static const uint8_t oce[5] = { 0, 0, 1, 0, 0 };
	uint8_t list[1 + 4 * 13];
	size_t size = sizeof(fixed);
	int first = chunk * 13, n, i, ch;

	memcpy(buf, fixed, sizeof(fixed));
	buf[0] = chunk + 1;
	le16enc(buf + 8, W_DWELL);
	le16enc(buf + 10, W_DWELL);
	le16enc(buf + 12, W_DWELL);
	n = MIN(13, (int)W_NCHAN - first);
	list[0] = n;
	for (i = 0; i < n; i++) {
		ch = first + i;
		list[1 + 4 * i] = ch < 13 ? ch + 1 : w_5ghz[ch - 13];
		list[2 + 4 * i] = ch >= 13;
		list[3 + 4 * i] = 0;
		list[4 + 4 * i] = 0;
	}
	n1w_tlv(buf, &size, 0x104, list, 1 + 4 * n);
	n1w_tlv(buf, &size, 0x165, oce, sizeof(oce));
	return (size);
}

static int
n1w_scan(struct n1_softc *sc)
{
	uint8_t req[14 + 4 + 1 + 4 * 13 + 4 + 5];
	uint32_t channels, scans, seen, ch0 = sc->w.channels;
	int chunk, count, error = 0, i, limit, progress, stopped, refused;

	for (chunk = 0; chunk * 13 < (int)W_NCHAN && error == 0; chunk++) {
		count = MIN(13, (int)W_NCHAN - chunk * 13);
		channels = seen = sc->w.channels;
		scans = sc->w.scans;
		stopped = refused = 0;
		if ((error = n1w_cmd(sc, 0x1000f, req,
		    n1w_scan_req(chunk, req))) != 0)
			break;
		/* ticks of 10 ms */
		limit = MAX(800, (count * 2 * W_DWELL + 2000) / 10);
		for (i = progress = 0; i < limit; i++) {
			n1w_drain(sc);
			if (!n1w_up(sc)) {
				error = EIO;
				break;
			}
			if (sc->w.scans != scans)
				break;
			if (sc->w.channels != seen) {
				seen = sc->w.channels;
				progress = i;
			}
			if (!stopped && (seen - channels >= (uint32_t)count ||
			    (seen != channels && i - progress > 100))) {
				stopped = 1;
				if (n1w_cmd(sc, 0x10001, NULL, 0) != 0) {
					refused = 1;
					break;
				}
				limit = i + 100;
			}
			pause("n1wscan", hz / 100 + 1);
		}
		if (error == 0 && !refused && sc->w.scans == scans) {
			device_printf(sc->dev, "wifi: scan chunk %d: %u of %d "
			    "channels, no end event\n", chunk,
			    sc->w.channels - channels, count);
			if (!stopped)
				n1w_cmd(sc, 0x10001, NULL, 0);
		}
	}
	device_printf(sc->dev, "wifi: scan %d: %u of %zu channels, %u beacons, "
	    "%d networks\n", error, sc->w.channels - ch0, W_NCHAN,
	    sc->w.beacons, sc->w.nbss);
	return (error);
}

/*
 * Join a WPA2-PSK network (aurora wifi_connect, host_setup_commands and
 * connect_worker). The firmware runs the 4-way handshake from the PMK in
 * the request: a 130-byte block (1, BSSID, channel, band at 8, security 4 at
 * 0x47 and 0x48, PMK length 32 at 0x4a, PMK at 0x4c), then TLVs 304 {0},
 * 0xf913 (connection timeouts), 342 (1, 4, SSID length, SSID), 48 (our RSN
 * element: CCMP/CCMP/PSK with the AP's capabilities), 316 (PMF) and 376 (MLD
 * defaults). Event 0x80000000 completes the join.
 */
static int
n1w_join(struct n1_softc *sc)
{
	static const uint8_t mld[14] = { 1, 0, 1, 0xff, 0xff, 0xff, 0xff,
	    0xff, 0xff, 0xff, 0xff, 3, 0, 0 };
	static const uint32_t timeouts[5] = { 260, 6000, 2000, 15000, 300 };
	static const uint8_t awake[3] = { 0 };
	static const uint8_t profile[14] = { 0x54, 1, 10, 0, 0, 8, 0, 0,
	    0xff, 0xff, 0xff, 0xff, 1, 1 };
	uint8_t rsn[20] = { 1, 0, 0x00, 0x0f, 0xac, 4, 1, 0, 0x00, 0x0f, 0xac,
	    4, 1, 0, 0x00, 0x0f, 0xac, 2 };
	uint8_t pay[512], lifetime[20], ssid[35], tmo[20], zero = 0, pmf;
	size_t size = 130, len = strlen(sc->w.ssid);
	int error, i;

	if (!sc->w.found)
		return (ENOENT);
	for (i = 0; i < 4; i++) {
		static const unsigned int ids[4] = { 76, 24, 15, 12 };

		if ((error = n1w_data_ring(sc, ids[i])) != 0)
			device_printf(sc->dev, "wifi: data ring %u: %d\n",
			    ids[i], error);
	}
	memset(lifetime, 0, sizeof(lifetime));
	for (i = 0; i < 4; i++)
		le32enc(lifetime + 4 + 4 * i, 2000000);
	device_printf(sc->dev, "wifi: host awake %d, lifetime %d, profile %d\n",
	    n1w_cmd(sc, 6, awake, sizeof(awake)),
	    n1w_cmd(sc, 43, lifetime, sizeof(lifetime)),
	    n1w_cmd(sc, 52, profile, sizeof(profile)));
	le16enc(rsn + 18, sc->w.rsn_caps);
	pmf = (sc->w.rsn_caps & 0x80) != 0;
	memset(pay, 0, sizeof(pay));
	pay[0] = 1;
	memcpy(pay + 1, sc->w.bssid, 6);
	pay[7] = sc->w.chan;
	pay[8] = sc->w.band;
	pay[0x47] = 4;
	le16enc(pay + 0x48, 4);
	le16enc(pay + 0x4a, 32);
	memcpy(pay + 0x4c, sc->w.pmk, 32);
	for (i = 0; i < 5; i++)
		le32enc(tmo + 4 * i, timeouts[i]);
	ssid[0] = 1;
	ssid[1] = 4;
	ssid[2] = len;
	memcpy(ssid + 3, sc->w.ssid, len);
	n1w_tlv(pay, &size, 304, &zero, 1);
	n1w_tlv(pay, &size, 0xf913, tmo, sizeof(tmo));
	n1w_tlv(pay, &size, 342, ssid, 3 + len);
	n1w_tlv(pay, &size, 48, rsn, sizeof(rsn));
	n1w_tlv(pay, &size, 316, &pmf, 1);
	n1w_tlv(pay, &size, 376, mld, sizeof(mld));
	sc->w.connecting = 1;
	sc->w.connect_event = sc->w.associated = 0;
	sc->w.connect_status = 0;
	error = n1w_cmd(sc, 0x20000, pay, size);
	explicit_bzero(pay, sizeof(pay));
	explicit_bzero(sc->w.win + W_TX_BUF, w_rsize[1] * W_BUFSZ);
	cpu_dcache_wb_range(sc->w.win + W_TX_BUF, w_rsize[1] * W_BUFSZ);
	for (i = 0; error == 0 && !sc->w.connect_event && i < 1500; i++) {
		n1w_drain(sc);
		if (!n1w_up(sc))
			error = EIO;
		pause("n1wj", hz / 100 + 1);
	}
	sc->w.connecting = 0;
	if (error == 0 && !sc->w.connect_event)
		error = ETIMEDOUT;
	device_printf(sc->dev, "wifi: join \"%s\" %6D chan %u: %d, event "
	    "status %u, associated %d\n", sc->w.ssid, sc->w.bssid, ":",
	    sc->w.chan, error, sc->w.connect_status, sc->w.associated);
	if (error == 0 && sc->w.connect_status == 0)
		device_printf(sc->dev, "wifi: roam scan off %d\n",
		    n1w_cmd(sc, 0x20016, NULL, 0));
	return (error != 0 ? error : sc->w.connect_status != 0 ? EIO : 0);
}

/* Join target from the loader: hw.apple_n1.ssid, hw.apple_n1.pmk (hex) */
static void
n1w_target(struct n1_softc *sc)
{
	char *v;
	int i;

	if ((v = kern_getenv("hw.apple_n1.ssid")) != NULL) {
		strlcpy(sc->w.ssid, v, sizeof(sc->w.ssid));
		freeenv(v);
	}
	if ((v = kern_getenv("hw.apple_n1.pmk")) != NULL) {
		if (strlen(v) == 64) {
			for (i = 0; i < 32; i++) {
				char b[3] = { v[2 * i], v[2 * i + 1], 0 };

				sc->w.pmk[i] = strtoul(b, NULL, 16);
			}
			sc->w.have_pmk = 1;
		}
		explicit_bzero(v, strlen(v));
		freeenv(v);
		kern_unsetenv("hw.apple_n1.pmk");
	}
}

/*
 * Wi-Fi data path (aurora data_probe_locked, drain_data_rx, drain_data_tx).
 * Rings are opened with alpha messages: completion rings 192 (id 76, 2 KiB
 * entries at window +0x200000, transmit completions) and 207 (id 15, at
 * +0x240000, receive completions), transfer rings 24, 12 (receive buffers,
 * descriptors at +0x520000, 4 KiB buffers at +0xe00000) and 25 (transmit,
 * 32-byte descriptors at +0x540000, 2 KiB buffers at +0xd00000). Frames
 * are Ethernet: the firmware does the 802.11 side.
 */
#define	W_DTX_BUF	0xd00000
#define	W_DRX_BUF	0xe00000
#define	W_DTX_SLOTS	128

static int
n1w_data_ring(struct n1_softc *sc, unsigned int id)
{
	uint8_t msg[0x34 + 10];
	unsigned int vector, offset, size, bit, db, irq;
	int transfer = 0, error;

	memset(msg, 0, sizeof(msg));
	switch (id) {
	case 76:
		vector = 192; offset = 0x200000; size = 2048; bit = 0;
		db = 18; irq = 3;
		break;
	case 15:
		vector = 207; offset = 0x240000; size = 128; bit = 2;
		db = 11; irq = 8;
		break;
	case 24: case 25: case 12:
		transfer = 1;
		vector = id;
		offset = id == 12 ? 0x520000 : id == 24 ? 0x500000 : 0x540000;
		size = 128;
		bit = id == 12 ? 4 : id == 24 ? 3 : 5;
		db = id == 12 ? 11 : 18;
		irq = id == 12 ? 8 : 3;
		break;
	default:
		return (EINVAL);
	}
	if (!n1w_up(sc))
		return (EIO);
	if (sc->w.data_done & (1u << bit))
		return (EBUSY);
	sc->w.data_done |= 1u << bit;
	memset(sc->w.win + offset, 0, size * 32);
	cpu_dcache_wb_range(sc->w.win + offset, size * 32);
	n1w_idx_set(sc, transfer ? 1 : 0, vector, 0);
	n1w_idx_set(sc, transfer ? 3 : 2, vector, 0);
	msg[0] = transfer ? 1 : 2;
	msg[2] = transfer && id == 12 ? 0 : 4;
	le16enc(msg + 4, id);
	le16enc(msg + 6, vector);
	le64enc(msg + 8, sc->w.win_iova + offset);
	if (transfer) {
		le64enc(msg + 0x10, ~0ul);
		le16enc(msg + 0x18, size);
		le16enc(msg + 0x1a, id == 12 ? 12 : 76);
		le16enc(msg + 0x1c, db);
		le16enc(msg + 0x1e, id == 12 ? 0x220 : 0);
		le32enc(msg + 0x20, 16);
		le16enc(msg + 0x24, irq);
	} else {
		le16enc(msg + 0x10, size);
		le16enc(msg + 0x12, id == 15 ? 12 : 0xffff);
		le16enc(msg + 0x14, db);
		le32enc(msg + 0x18, 1);
		le16enc(msg + 0x1c, irq);
	}
	error = n1w_msg(sc, msg, (transfer ? 0x34 : 0x2c) + 10);
	pause("n1wd", hz);
	if (error == 0 && !n1w_up(sc))
		error = EIO;
	if (error == 0 && id == 12) {
		unsigned int slot;

		for (slot = 0; slot < size - 1; slot++) {
			uint8_t *d = sc->w.win + offset + slot * 16;
			uint32_t buf = W_DRX_BUF + slot * 4096;

			memset(sc->w.win + buf, 0, 4096);
			cpu_dcache_wb_range(sc->w.win + buf, 4096);
			le32enc(d, 1 | 4096 << 8);
			le64enc(d + 4, sc->w.win_iova + buf);
			le32enc(d + 12, slot + 1);
		}
		cpu_dcache_wb_range(sc->w.win + offset, size * 16);
		n1w_idx_set(sc, 3, vector, size - 1);
		bus_write_4(sc->fnbar[1], 0x1050, 1);
		pause("n1wd", hz / 10);
	}
	N1_DPRINTF(sc, "wifi: data ring %u: %d\n", id, error);
	return (error);
}

/* Receive completions on 207 name a buffer by cookie; repost it on 12 */
static void
n1w_data_rx(struct n1_softc *sc, struct mbufq *in)
{
	uint16_t head, tail, post, next, cookie, length;
	uint32_t hdr, info, footer, buf, pad;
	uint8_t *c, *d;
	struct mbuf *m;
	int n = 0;

	if (!(sc->w.data_done & (1u << 4)) || !n1w_up(sc))
		return;
	head = n1w_idx(sc, 0, 207);
	tail = n1w_idx(sc, 2, 207);
	post = n1w_idx(sc, 3, 12);
	if (head >= 128 || tail >= 128 || post >= 128) {
		sc->w.rx_bad++;
		return;
	}
	while (tail != head && n < 127) {
		c = sc->w.win + 0x240000 + tail * 32;
		cpu_dcache_inv_range(c, 32);
		hdr = le32dec(c);
		info = le32dec(c + 4);
		cookie = info & 0xffff;
		length = info >> 16;
		next = (post + 1) % 128;
		if ((hdr >> 16) != 12 || cookie == 0 || cookie > 127 ||
		    length > 4096 || le32dec(c + 8) != 0) {
			sc->w.rx_bad++;
			break;
		}
		if (next == n1w_idx(sc, 1, 12))
			break;
		buf = W_DRX_BUF + (cookie - 1) * 4096;
		footer = le32dec(c + 12);
		pad = (footer >> 12) & 15;
		cpu_dcache_inv_range(sc->w.win + buf, 4096);
		sc->w.rx_pkts++;
		if (sc->w.ifp != NULL && (footer & (1u << 24)) == 0 &&
		    length >= pad + ETHER_HDR_LEN &&
		    (m = m_get2(length - pad + ETHER_ALIGN, M_NOWAIT, MT_DATA,
		    M_PKTHDR)) != NULL) {
			m->m_data += ETHER_ALIGN;
			memcpy(mtod(m, void *), sc->w.win + buf + pad,
			    length - pad);
			m->m_len = m->m_pkthdr.len = length - pad;
			m->m_pkthdr.rcvif = sc->w.ifp;
			if (mbufq_enqueue(in, m) != 0)
				m_freem(m);
		} else if (sc->w.ifp != NULL)
			if_inc_counter(sc->w.ifp, IFCOUNTER_IQDROPS, 1);
		d = sc->w.win + 0x520000 + post * 16;
		le32enc(d, 1 | 4096 << 8);
		le64enc(d + 4, sc->w.win_iova + buf);
		le32enc(d + 12, cookie);
		cpu_dcache_wb_range(d, 16);
		tail = (tail + 1) % 128;
		post = next;
		n++;
	}
	if (n != 0) {
		n1w_idx_set(sc, 2, 207, tail);
		n1w_idx_set(sc, 3, 12, post);
		bus_write_4(sc->fnbar[1], 0x1050, 1);
	}
}

/*
 * Transmit completions on 192 (ring 25, cookie of the last of count
 * transfers), then queued frames into ring 25: entry u32 (len << 8 | 3),
 * u64 buffer, u16 slot + 1 at 12, metadata 6 << 27 | 4 << 15 at 16 (TID,
 * native packet preparation), lifetime 2000000 at 24.
 */
static void
n1w_data_tx(struct n1_softc *sc)
{
	uint16_t head, tail, post, next, ring, cookie, count;
	uint8_t *e, *d, status;
	unsigned int first, i, slot;
	struct mbuf *m;
	int posted = 0, completed = 0, budget = 256;

	if (!(sc->w.data_done & (1u << 5)) || !n1w_up(sc))
		return;
	head = n1w_idx(sc, 0, 192);
	tail = n1w_idx(sc, 2, 192);
	post = n1w_idx(sc, 3, 25);
	if (head >= 2048 || tail >= 2048 || post >= W_DTX_SLOTS)
		return;
	while (tail != head && budget--) {
		e = sc->w.win + 0x200000 + tail * 32;
		cpu_dcache_inv_range(e, 32);
		ring = le32dec(e) >> 16;
		cookie = le16dec(e + 4);
		count = le16dec(e + 9);
		status = e[12];
		if (ring == 25) {
			if (cookie == 0 || cookie > W_DTX_SLOTS || count == 0 ||
			    count >= W_DTX_SLOTS || !(e[1] & 1)) {
				sc->w.tx_bad++;
				break;
			}
			first = (cookie + W_DTX_SLOTS - count) % W_DTX_SLOTS;
			for (i = 0; i < count; i++)
				if (!sc->w.tx_busy[(first + i) % W_DTX_SLOTS])
					break;
			if (i != count) {
				sc->w.tx_bad++;
				break;
			}
			for (i = 0; i < count; i++) {
				slot = (first + i) % W_DTX_SLOTS;
				if (status == 0 || status == 0x83) {
					sc->w.tx_pkts++;
					if_inc_counter(sc->w.ifp,
					    IFCOUNTER_OPACKETS, 1);
					if_inc_counter(sc->w.ifp,
					    IFCOUNTER_OBYTES, sc->w.tx_len[slot]);
				} else
					if_inc_counter(sc->w.ifp,
					    IFCOUNTER_OERRORS, 1);
				sc->w.tx_busy[slot] = 0;
			}
		}
		tail = (tail + 1) % 2048;
		completed = 1;
	}
	if (completed)
		n1w_idx_set(sc, 2, 192, tail);
	for (budget = W_DTX_SLOTS - 1; sc->w.connected && budget--; ) {
		uint32_t buf = W_DTX_BUF + post * W_BUFSZ;

		next = (post + 1) % W_DTX_SLOTS;
		if (sc->w.tx_busy[post] || next == n1w_idx(sc, 1, 25))
			break;
		if ((m = mbufq_dequeue(&sc->w.txq)) == NULL)
			break;
		if (m->m_pkthdr.len > W_BUFSZ) {
			if_inc_counter(sc->w.ifp, IFCOUNTER_OERRORS, 1);
			m_freem(m);
			continue;
		}
		m_copydata(m, 0, m->m_pkthdr.len, sc->w.win + buf);
		cpu_dcache_wb_range(sc->w.win + buf, m->m_pkthdr.len);
		d = sc->w.win + 0x540000 + post * 32;
		memset(d, 0, 32);
		le32enc(d, 3 | m->m_pkthdr.len << 8);
		le64enc(d + 4, sc->w.win_iova + buf);
		le16enc(d + 12, post + 1);
		le64enc(d + 16, 6ull << 27 | 4ull << 15);
		le64enc(d + 24, 2000000);
		cpu_dcache_wb_range(d, 32);
		sc->w.tx_len[post] = m->m_pkthdr.len;
		sc->w.tx_busy[post] = 1;
		m_freem(m);
		post = next;
		posted = 1;
	}
	if (posted)
		n1w_idx_set(sc, 3, 25, post);
	if (posted || completed)
		bus_write_4(sc->fnbar[1], 0x1030, 1);
}

static void
n1w_poll(void *arg)
{
	struct n1_softc *sc = arg;
	struct mbufq in;
	struct mbuf *m;

	mtx_assert(&sc->w.mtx, MA_OWNED);
	mbufq_init(&in, 256);
	n1w_drain(sc);
	n1w_data_rx(sc, &in);
	n1w_data_tx(sc);
	if (n1w_up(sc))
		callout_reset(&sc->w.poll, hz / 100 + 1, n1w_poll, sc);
	else
		device_printf(sc->dev, "wifi: alpha left stage 2 (%#x)\n",
		    bus_read_4(sc->fnbar[1], 0x8));
	if (mbufq_len(&in) == 0)
		return;
	mtx_unlock(&sc->w.mtx);
	while ((m = mbufq_dequeue(&in)) != NULL) {
		if_inc_counter(sc->w.ifp, IFCOUNTER_IPACKETS, 1);
		if_input(sc->w.ifp, m);
	}
	mtx_lock(&sc->w.mtx);
}

static int
n1w_transmit(if_t ifp, struct mbuf *m)
{
	struct n1_softc *sc = if_getsoftc(ifp);
	int error;

	BPF_MTAP(ifp, m);
	mtx_lock(&sc->w.mtx);
	if (!sc->w.connected) {
		mtx_unlock(&sc->w.mtx);
		m_freem(m);
		return (ENETDOWN);
	}
	if ((error = mbufq_enqueue(&sc->w.txq, m)) != 0) {
		mtx_unlock(&sc->w.mtx);
		if_inc_counter(ifp, IFCOUNTER_OQDROPS, 1);
		m_freem(m);
		return (error);
	}
	n1w_data_tx(sc);
	mtx_unlock(&sc->w.mtx);
	return (0);
}

static void
n1w_qflush(if_t ifp)
{
	struct n1_softc *sc = if_getsoftc(ifp);

	mtx_lock(&sc->w.mtx);
	mbufq_drain(&sc->w.txq);
	mtx_unlock(&sc->w.mtx);
}

static void
n1w_init(void *arg)
{
	struct n1_softc *sc = arg;

	if_setdrvflagbits(sc->w.ifp, IFF_DRV_RUNNING, 0);
}

static int
n1w_ioctl(if_t ifp, u_long cmd, caddr_t data)
{
	switch (cmd) {
	case SIOCSIFFLAGS:
		if (if_getflags(ifp) & IFF_UP)
			if_setdrvflagbits(ifp, IFF_DRV_RUNNING, 0);
		else
			if_setdrvflagbits(ifp, 0, IFF_DRV_RUNNING);
		return (0);
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		return (0);
	default:
		return (ether_ioctl(ifp, cmd, data));
	}
}

/* After a join: transmit ring 25, the n1w0 Ethernet interface, polling */
static int
n1w_attach_if(struct n1_softc *sc)
{
	if_t ifp;
	int error;

	if ((error = n1w_data_ring(sc, 25)) != 0)
		return (error);
	mtx_init(&sc->w.mtx, "n1w", NULL, MTX_DEF);
	callout_init_mtx(&sc->w.poll, &sc->w.mtx, 0);
	mbufq_init(&sc->w.txq, 64);
	sc->w.connected = 1;
	ifp = sc->w.ifp = if_alloc(IFT_ETHER);
	if_setsoftc(ifp, sc);
	if_initname(ifp, "n1w", 0);
	if_setflags(ifp, IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	if_setinitfn(ifp, n1w_init);
	if_setioctlfn(ifp, n1w_ioctl);
	if_settransmitfn(ifp, n1w_transmit);
	if_setqflushfn(ifp, n1w_qflush);
	CURVNET_SET(vnet0);	/* from the boot task */
	ether_ifattach(ifp, sc->w.mac);
	if_link_state_change(ifp, LINK_STATE_UP);
	CURVNET_RESTORE();
	{
		struct sysctl_ctx_list *ctx = device_get_sysctl_ctx(sc->dev);
		struct sysctl_oid_list *kids =
		    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

		SYSCTL_ADD_U32(ctx, kids, OID_AUTO, "wifi_rx", CTLFLAG_RD,
		    &sc->w.rx_pkts, 0, "frames received");
		SYSCTL_ADD_U32(ctx, kids, OID_AUTO, "wifi_rx_bad", CTLFLAG_RD,
		    &sc->w.rx_bad, 0, "bad receive completions");
		SYSCTL_ADD_U32(ctx, kids, OID_AUTO, "wifi_tx", CTLFLAG_RD,
		    &sc->w.tx_pkts, 0, "frames sent");
		SYSCTL_ADD_U32(ctx, kids, OID_AUTO, "wifi_tx_bad", CTLFLAG_RD,
		    &sc->w.tx_bad, 0, "bad transmit completions");
	}
	mtx_lock(&sc->w.mtx);
	callout_reset(&sc->w.poll, hz / 100 + 1, n1w_poll, sc);
	mtx_unlock(&sc->w.mtx);
	device_printf(sc->dev, "wifi: n1w0 up, %6D\n", sc->w.mac, ":");
	return (0);
}

/*
 * Bluetooth on beta (function 2), as AirshipDK's t2026-CentauriBeta.plist and
 * the macOS trace show it: rings are created with the control ring message
 * on beta's message ring (doorbell 0x1000), and each ring's doorbell is
 * 0x1000 + 4 * id. HCI rings carry the data in place: an entry is a 16-byte
 * header (u32 length << 8 | type, u64 ~0, u32 slot; the firmware sets 4 in the
 * last byte when done) followed by the footer with the packet, without the
 * H4 type byte. Transmit entries have type 2; receive buffers are posted
 * with the footer size as length.
 *
 * The node "n1bt" has one hook, "hook", for ng_hci's "drv" hook, and passes
 * H4 framed packets like ng_ubt. Receive rings are polled.
 */
static const struct n1_bt_ringdef {
	uint16_t	id, flags, n, footer;
	uint8_t		h4;		/* receive: packet type; 0 for transmit */
} n1_bt_rings[N1_BT_RINGS] = {
	{ 7, 0x41, 17, 260, NG_HCI_EVENT_PKT },		/* hci-rx */
	{ 1, 0x41, 5, 260, 0 },				/* hci-tx */
	{ 8, 0x3051, 9, 2592, NG_HCI_ACL_DATA_PKT },	/* acl-rx */
	{ 2, 0x3051, 9, 2592, 0 },			/* acl-tx */
};
#define	BT_HCI_RX	0
#define	BT_HCI_TX	1
#define	BT_ACL_RX	2
#define	BT_ACL_TX	3
#define	BT_ENT(d)	(16 + (d)->footer)

static int
n1_bt_ring(struct n1_softc *sc, int r)
{
	const struct n1_bt_ringdef *d = &n1_bt_rings[r];
	uint8_t msg[0x34];
	uint64_t iova;

	if ((sc->bt[r].mem = n1_dma_alloc(sc, d->n * BT_ENT(d), &iova)) == NULL)
		return (ENOMEM);
	memset(msg, 0, sizeof(msg));
	le16enc(msg, 1);
	le16enc(msg + 2, d->flags);
	le16enc(msg + 4, d->id);
	le16enc(msg + 6, d->id);
	le64enc(msg + 8, iova);
	le64enc(msg + 0x10, ~0ul);
	le16enc(msg + 0x18, d->n);
	le16enc(msg + 0x1a, 0xffff);
	le16enc(msg + 0x1c, d->id);
	le16enc(msg + 0x1e, 0x40);
	return (n1_msg_send(&sc->beta, sc->fnbar[2], 0x1000, msg, sizeof(msg)));
}

static void
n1_bt_doorbell(struct n1_softc *sc, int r)
{
	const struct n1_bt_ringdef *d = &n1_bt_rings[r];
	struct n1_ctx *c = &sc->beta;
	uint16_t tail = sc->bt[r].tail % d->n;

	cpu_dcache_wb_range(sc->bt[r].mem, d->n * BT_ENT(d));
	le16enc(c->buf + c->index[3] + 2 * d->id, tail);
	cpu_dcache_wb_range(c->buf, PAGE_SIZE);
	bus_write_4(sc->fnbar[2], 0x1000 + 4 * d->id, tail);
}

static uint8_t *
n1_bt_entry(struct n1_softc *sc, int r, uint16_t i)
{
	const struct n1_bt_ringdef *d = &n1_bt_rings[r];

	return (sc->bt[r].mem + BT_ENT(d) * (i % d->n));
}

static void
n1_bt_post(struct n1_softc *sc, int r)
{
	uint8_t *e = n1_bt_entry(sc, r, sc->bt[r].tail);

	memset(e, 0, 16);
	le32enc(e, n1_bt_rings[r].footer << 8);
	le64enc(e + 4, ~0ul);
	le32enc(e + 12, sc->bt[r].tail % n1_bt_rings[r].n);
	sc->bt[r].tail++;
}

/* Queue one packet (without the H4 byte) on a transmit ring. */
static int
n1_bt_tx(struct n1_softc *sc, int r, struct mbuf *m)
{
	const struct n1_bt_ringdef *d = &n1_bt_rings[r];
	struct n1_ctx *c = &sc->beta;
	uint16_t head;
	uint8_t *e;
	int len = m->m_pkthdr.len;

	mtx_assert(&sc->bt_mtx, MA_OWNED);
	cpu_dcache_inv_range(c->buf, PAGE_SIZE);
	head = le16dec(c->buf + c->index[1] + 2 * d->id);
	if (len > d->footer || (uint16_t)(sc->bt[r].tail + 1 - head) % d->n == 0)
		return (ENOBUFS);
	e = n1_bt_entry(sc, r, sc->bt[r].tail);
	memset(e, 0, 16);
	le32enc(e, len << 8 | 2);
	le64enc(e + 4, ~0ul);
	le32enc(e + 12, sc->bt[r].tail % d->n);
	m_copydata(m, 0, len, e + 16);
	if (n1_bt_debug)
		device_printf(sc->dev, "bt tx %u slot %u head %u: %*D\n", d->id,
		    sc->bt[r].tail % d->n, head, MIN(len, 32), e + 16, " ");
	sc->bt[r].tail++;
	n1_bt_doorbell(sc, r);
	return (0);
}

/*
 * The controller refuses Create Connection (0x0405) with Command Disallowed;
 * macOS connects with vendor 0xff7b instead, whose parameters are 0x0405's
 * plus three bytes it sends as zero.  Rewrite the command on the way out and
 * the opcode in its Command Status on the way in, so ng_hci sees 0x0405.
 */
#define	N1_BT_CREATE_CON	0x0405
#define	N1_BT_VSC_CREATE_CON	0xff7b

static struct mbuf *
n1_bt_fixcmd(struct mbuf *m)
{
	struct mbuf *n;
	uint8_t *p;

	if (m->m_pkthdr.len != 3 + 13)
		return (m);
	m = m_pullup(m, 3);
	if (m == NULL || le16dec(mtod(m, uint8_t *)) != N1_BT_CREATE_CON)
		return (m);
	n = m_get2(3 + 16, M_NOWAIT, MT_DATA, M_PKTHDR);
	if (n == NULL)
		return (m);
	p = mtod(n, uint8_t *);
	le16enc(p, N1_BT_VSC_CREATE_CON);
	p[2] = 16;
	m_copydata(m, 3, 13, p + 3);
	memset(p + 16, 0, 3);
	n->m_len = n->m_pkthdr.len = 3 + 16;
	m_freem(m);
	return (n);
}

/*
 * LE Set Scan Parameters and Enable are refused as well.  macOS scans with
 * vendor ff42 (one per scan type), ff3e (apply) and ff3f (commit), and the
 * results come as vendor event 0xff/0x0e: an LE Extended Advertising Report
 * after a 3-byte header, with a 12-byte trailer.  Answer 0x200b/0x200c here,
 * send the vendor sequence from the tick one command at a time, swallow its
 * completions, and give ng_hci legacy Advertising Reports.
 * Each entry: opcode (le16), parameter length, parameters.
 */
static const uint8_t n1_bt_scan_on[] = {
	0x3e, 0xff, 0x09, 0x00, 0x02, 0x01, 0x02, 0x09, 0x00, 0x00, 0x00, 0x00,
	0x41, 0xff, 0x04, 0x06, 0x00, 0x00, 0x84,
	0xe6, 0xfd, 0x0e, 0x02, 0x01, 0x02, 0x00, 0x00, 0x00,
	    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xbd, 0xfd, 0x10, 0x06, 0x00, 0x00, 0x80, 0x07, 0x00, 0x00, 0x80,
	    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0xb2, 0xfd, 0x08, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xc9, 0xfd, 0x00,
	0x42, 0xff, 0x0c, 0x01, 0x00, 0x01, 0x02, 0x01, 0x01, 0xe0, 0x01,
	    0x30, 0x00, 0x1e, 0x00,
	0x42, 0xff, 0x0c, 0x01, 0x00, 0x01, 0x02, 0x02, 0x01, 0xe0, 0x01,
	    0x2b, 0x00, 0x1e, 0x00,
	0x42, 0xff, 0x0c, 0x01, 0x00, 0x01, 0x02, 0x08, 0x01, 0xe0, 0x01,
	    0x10, 0x00, 0x1e, 0x00,
	0x3e, 0xff, 0x09, 0x01, 0x02, 0x00, 0x02, 0x0b, 0x00, 0x00, 0x00, 0x00,
	0x3f, 0xff, 0x01, 0x0b,
};
static const uint8_t n1_bt_scan_off[] = {
	0x3e, 0xff, 0x09, 0x00, 0x02, 0x00, 0x02, 0x0b, 0x00, 0x00, 0x00, 0x00,
	0x3f, 0xff, 0x01, 0x0b,
};

static void n1_bt_vqueue(struct n1_softc *, const uint8_t *, size_t);

/*
 * After every Reset: the controller state the vendor scan needs, as found by
 * bisecting the macOS init (2026-11-20).  Without the two event masks no scan
 * events arrive; without vendor 0xfda3 the scan starts but stays silent;
 * without a random address ff3e (scan start) fails with 0x12.
 */
static void
n1_bt_postreset(struct n1_softc *sc)
{
	static const uint8_t init[] = {
		0x01, 0x0c, 0x08, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f,
		0x01, 0x20, 0x08, 0x7f, 0xfa, 0x86, 0x3f, 0xff, 0xfb, 0x8f, 0x80,
		0xa3, 0xfd, 0x04, 0x01, 0x00, 0x00, 0x00,
	};
	uint8_t raddr[3 + 6];

	n1_bt_vqueue(sc, init, sizeof(init));
	le16enc(raddr, 0x2005);
	raddr[2] = 6;
	memcpy(raddr + 3, sc->bt_raddr, 6);
	n1_bt_vqueue(sc, raddr, sizeof(raddr));
}

static void
n1_bt_vqueue(struct n1_softc *sc, const uint8_t *p, size_t len)
{
	struct mbuf *m;
	size_t n;

	mtx_assert(&sc->bt_mtx, MA_OWNED);
	for (; len >= 3; p += n, len -= n) {
		n = 3 + p[2];
		m = m_get2(n, M_NOWAIT, MT_DATA, M_PKTHDR);
		if (m == NULL)
			return;
		memcpy(mtod(m, uint8_t *), p, n);
		m->m_len = m->m_pkthdr.len = n;
		if (mbufq_enqueue(&sc->bt_vq, m) != 0)
			m_freem(m);
	}
}

/* Queue a made-up Command Complete (status 0) for ng_hci. */
static void
n1_bt_fakecc(struct n1_softc *sc, uint16_t op)
{
	struct mbuf *m;
	uint8_t *p;

	mtx_assert(&sc->bt_mtx, MA_OWNED);
	m = m_get2(7, M_NOWAIT, MT_DATA, M_PKTHDR);
	if (m == NULL)
		return;
	p = mtod(m, uint8_t *);
	p[0] = NG_HCI_EVENT_PKT;
	p[1] = NG_HCI_EVENT_COMMAND_COMPL;
	p[2] = 4;
	p[3] = 1;
	le16enc(p + 4, op);
	p[6] = 0;
	m->m_len = m->m_pkthdr.len = 7;
	if (mbufq_enqueue(&sc->bt_evq, m) != 0)
		m_freem(m);
}

/* Host commands answered here; true if the command was consumed. */
static bool
n1_bt_hostcmd(struct n1_softc *sc, struct mbuf *m)
{
	uint8_t p[4];
	uint16_t op;

	if (m->m_pkthdr.len < 3)
		return (false);
	m_copydata(m, 0, MIN(m->m_pkthdr.len, 4), p);
	op = le16dec(p);
	if (op != 0x200b && op != 0x200c)
		return (false);
	mtx_lock(&sc->bt_mtx);
	if (op == 0x200c && m->m_pkthdr.len >= 4) {
		if (p[3])
			n1_bt_vqueue(sc, n1_bt_scan_on, sizeof(n1_bt_scan_on));
		else
			n1_bt_vqueue(sc, n1_bt_scan_off,
			    sizeof(n1_bt_scan_off));
	}
	n1_bt_fakecc(sc, op);
	mtx_unlock(&sc->bt_mtx);
	return (true);
}

/* Vendor event 0xff/0x0e (one legacy report) as an LE Advertising Report. */
static struct mbuf *
n1_bt_advrep(const uint8_t *p, uint32_t len)
{
	struct mbuf *m;
	uint8_t *q, t, dlen;

	if (len < 30 || p[2] != 0x0e || p[5] != 1 || (p[6] & 0x10) == 0)
		return (NULL);
	dlen = p[29];
	if (dlen > 31 || 30 + dlen > len)
		return (NULL);
	t = p[6];
	if (t & 0x08)
		t = 4;			/* SCAN_RSP */
	else if (t & 0x04)
		t = 1;			/* ADV_DIRECT_IND */
	else if ((t & 0x03) == 0x03)
		t = 0;			/* ADV_IND */
	else if (t & 0x02)
		t = 2;			/* ADV_SCAN_IND */
	else
		t = 3;			/* ADV_NONCONN_IND */
	m = m_get2(1 + 2 + 12 + dlen, M_NOWAIT, MT_DATA, M_PKTHDR);
	if (m == NULL)
		return (NULL);
	q = mtod(m, uint8_t *);
	q[0] = NG_HCI_EVENT_PKT;
	q[1] = NG_HCI_EVENT_LE;
	q[2] = 12 + dlen;
	q[3] = NG_HCI_LEEV_ADVREP;
	q[4] = 1;
	q[5] = t;
	q[6] = p[8] & 1;
	memcpy(q + 7, p + 9, 6);
	q[13] = dlen;
	memcpy(q + 14, p + 30, dlen);
	q[14 + dlen] = p[19];		/* RSSI */
	m->m_len = m->m_pkthdr.len = 1 + 2 + 12 + dlen;
	return (m);
}

/* Received HCI event (H4 framed): pass, rewrite, replace or drop it. */
static struct mbuf *
n1_bt_event(struct n1_softc *sc, struct mbuf *m)
{
	uint8_t *p = mtod(m, uint8_t *) + 1;
	uint32_t len = m->m_len - 1;
	struct mbuf *n;
	uint16_t op = 0;
	uint8_t st = 0;

	if (len >= 6 && p[0] == NG_HCI_EVENT_COMMAND_COMPL) {
		op = le16dec(p + 3);
		st = p[5];
		if (op == 0x0c03 && st == 0 && op != sc->bt_vwait)
			n1_bt_postreset(sc);
	} else if (len >= 6 && p[0] == NG_HCI_EVENT_COMMAND_STATUS) {
		op = le16dec(p + 4);
		st = p[2];
		if (op == N1_BT_VSC_CREATE_CON)
			le16enc(p + 4, N1_BT_CREATE_CON);
	}
	if (len >= 3 && p[0] == NG_HCI_EVENT_CON_COMPL && p[2] == 0)
		sc->bt_connects++;
	if (len >= 12 && (p[0] == NG_HCI_EVENT_CON_COMPL ||
	    p[0] == NG_HCI_EVENT_CON_REQ) && (p[11] & 0xfe) == 0xf0) {
		/*
		 * The firmware's native ACL link types 0xf0/0xf1 (aurora
		 * btcentauri): ng_hci matches and accepts only ACL (1).
		 */
		if (n1_bt_debug)
			device_printf(sc->dev, "bt event %#x: link type %#x "
			    "as ACL\n", p[0], p[11]);
		p[11] = NG_HCI_LINK_ACL;
	} else if (len >= 2 && p[0] == 0xff) {
		n = n1_bt_advrep(p, len);
		m_freem(m);
		return (n);
	}
	if (op != 0 && op == sc->bt_vwait) {
		if (st != 0)
			device_printf(sc->dev, "bt vendor %04x: status %#x\n",
			    op, st);
		sc->bt_vwait = 0;
		m_freem(m);
		return (NULL);
	}
	return (m);
}

/* Send the next queued vendor command once the previous one completed. */
static void
n1_bt_vsend(struct n1_softc *sc)
{
	struct mbuf *m;

	mtx_assert(&sc->bt_mtx, MA_OWNED);
	if (sc->bt_vwait != 0 && ++sc->bt_vticks < 100)
		return;
	sc->bt_vwait = 0;
	if ((m = mbufq_dequeue(&sc->bt_vq)) == NULL)
		return;
	if (n1_bt_tx(sc, BT_HCI_TX, m) != 0) {
		mbufq_prepend(&sc->bt_vq, m);
		return;
	}
	sc->bt_vwait = le16dec(mtod(m, uint8_t *));
	sc->bt_vticks = 0;
	m_freem(m);
}

/* Take the filled receive buffers off a ring, H4 framed, and repost them. */
static void
n1_bt_rx(struct n1_softc *sc, int r, struct mbufq *q)
{
	const struct n1_bt_ringdef *d = &n1_bt_rings[r];
	struct mbuf *m;
	uint32_t len;
	uint8_t *e;
	int n = 0;

	cpu_dcache_inv_range(sc->bt[r].mem, d->n * BT_ENT(d));
	for (;; n++) {
		e = n1_bt_entry(sc, r, sc->bt[r].seen);
		if (e[15] != 4)
			break;
		len = MIN(le32dec(e) >> 8, d->footer);
		if (n1_bt_debug)
			device_printf(sc->dev, "bt rx %u: %*D\n", d->id,
			    (int)MIN(len, 32), e + 16, " ");
		m = m_get2(len + 1, M_NOWAIT, MT_DATA, M_PKTHDR);
		if (m != NULL) {
			*mtod(m, uint8_t *) = d->h4;
			memcpy(mtod(m, uint8_t *) + 1, e + 16, len);
			m->m_len = m->m_pkthdr.len = len + 1;
			if (d->h4 == NG_HCI_EVENT_PKT)
				m = n1_bt_event(sc, m);
			if (m != NULL && mbufq_enqueue(q, m) != 0)
				m_freem(m);
		}
		sc->bt[r].seen++;
		n1_bt_post(sc, r);
	}
	if (n != 0)
		n1_bt_doorbell(sc, r);
}

static void
n1_bt_tick(void *arg)
{
	struct n1_softc *sc = arg;
	struct mbufq q;
	struct mbuf *m;
	hook_p hook;
	int error;

	mbufq_init(&q, 64);
	mtx_lock(&sc->bt_mtx);
	n1_bt_rx(sc, BT_HCI_RX, &q);
	n1_bt_rx(sc, BT_ACL_RX, &q);
	while ((m = mbufq_dequeue(&sc->bt_evq)) != NULL)
		if (mbufq_enqueue(&q, m) != 0)
			m_freem(m);
	n1_bt_vsend(sc);
	hook = sc->bt_hook;
	if (hook != NULL)
		NG_HOOK_REF(hook);
	mtx_unlock(&sc->bt_mtx);
	while ((m = mbufq_dequeue(&q)) != NULL) {
		if (hook != NULL)
			NG_SEND_DATA_ONLY(error, hook, m);
		else
			m_freem(m);
	}
	if (hook != NULL)
		NG_HOOK_UNREF(hook);
	callout_schedule(&sc->bt_callout, hz / 100 + 1);
}

/* Before the node exists: one command, waiting for its event (bring-up). */
static int
n1_bt_cmd(struct n1_softc *sc, uint16_t op, const void *param, uint8_t plen)
{
	struct mbufq q;
	struct mbuf *m;
	uint8_t pkt[3 + 255];
	int error, i;

	le16enc(pkt, op);
	pkt[2] = plen;
	memcpy(pkt + 3, param, plen);
	if ((m = m_get2(3 + plen, M_WAITOK, MT_DATA, M_PKTHDR)) == NULL)
		return (ENOMEM);
	m_copyback(m, 0, 3 + plen, pkt);
	mtx_lock(&sc->bt_mtx);
	error = n1_bt_tx(sc, BT_HCI_TX, m);
	mtx_unlock(&sc->bt_mtx);
	m_freem(m);
	if (error != 0)
		return (error);
	mbufq_init(&q, 16);
	for (i = 0; i < 100; i++) {
		pause("n1bt", hz / 100 + 1);
		mtx_lock(&sc->bt_mtx);
		n1_bt_rx(sc, BT_HCI_RX, &q);
		mtx_unlock(&sc->bt_mtx);
		if ((m = mbufq_dequeue(&q)) != NULL) {
			N1_DPRINTF(sc, "hci %#06x: event %*D\n", op,
			    (int)MIN(m->m_len, 16), mtod(m, uint8_t *), " ");
			mbufq_drain(&q);
			m_freem(m);
			return (0);
		}
	}
	device_printf(sc->dev, "hci command %#06x: no event\n", op);
	return (ETIMEDOUT);
}

static int
n1bt_ng_constructor(node_p node)
{

	return (EINVAL);
}

static int
n1bt_ng_newhook(node_p node, hook_p hook, const char *name)
{
	struct n1_softc *sc = NG_NODE_PRIVATE(node);
	int error = 0;

	if (strcmp(name, "hook") != 0)
		return (EINVAL);
	mtx_lock(&sc->bt_mtx);
	if (sc->bt_hook != NULL)
		error = EISCONN;
	else
		sc->bt_hook = hook;
	mtx_unlock(&sc->bt_mtx);
	return (error);
}

static int
n1bt_ng_connect(hook_p hook)
{

	NG_HOOK_FORCE_QUEUE(NG_HOOK_PEER(hook));
	return (0);
}

static int
n1bt_ng_disconnect(hook_p hook)
{
	struct n1_softc *sc = NG_NODE_PRIVATE(NG_HOOK_NODE(hook));

	mtx_lock(&sc->bt_mtx);
	if (sc->bt_hook == hook)
		sc->bt_hook = NULL;
	mtx_unlock(&sc->bt_mtx);
	return (0);
}

static int
n1bt_ng_rcvdata(hook_p hook, item_p item)
{
	struct n1_softc *sc = NG_NODE_PRIVATE(NG_HOOK_NODE(hook));
	struct mbuf *m;
	int error, r;

	NGI_GET_M(item, m);
	NG_FREE_ITEM(item);
	if (m->m_pkthdr.len < 2) {
		m_freem(m);
		return (EINVAL);
	}
	switch (*mtod(m, uint8_t *)) {
	case NG_HCI_CMD_PKT:
		r = BT_HCI_TX;
		break;
	case NG_HCI_ACL_DATA_PKT:
		r = BT_ACL_TX;
		break;
	default:
		m_freem(m);
		return (EINVAL);
	}
	m_adj(m, 1);
	if (r == BT_HCI_TX && n1_bt_hostcmd(sc, m)) {
		m_freem(m);
		return (0);
	}
	if (r == BT_HCI_TX && (m = n1_bt_fixcmd(m)) == NULL)
		return (ENOBUFS);
	mtx_lock(&sc->bt_mtx);
	error = n1_bt_tx(sc, r, m);
	mtx_unlock(&sc->bt_mtx);
	if (error != 0)
		device_printf(sc->dev, "bt tx ring %u: %d\n", n1_bt_rings[r].id,
		    error);
	m_freem(m);
	return (error);
}

static int
n1bt_ng_shutdown(node_p node)
{
	struct n1_softc *sc = NG_NODE_PRIVATE(node);

	/* The node lives as long as the device: make a new one. */
	NG_NODE_UNREF(node);
	if (ng_make_node_common(&n1bt_typestruct, &sc->bt_node) == 0) {
		ng_name_node(sc->bt_node, "n1bt0");
		NG_NODE_SET_PRIVATE(sc->bt_node, sc);
		NG_NODE_FORCE_WRITER(sc->bt_node);
	}
	return (0);
}

static struct ng_type n1bt_typestruct = {
	.version =	NG_ABI_VERSION,
	.name =		"n1bt",
	.constructor =	n1bt_ng_constructor,
	.newhook =	n1bt_ng_newhook,
	.connect =	n1bt_ng_connect,
	.rcvdata =	n1bt_ng_rcvdata,
	.disconnect =	n1bt_ng_disconnect,
	.shutdown =	n1bt_ng_shutdown,
};
NETGRAPH_INIT(n1bt, &n1bt_typestruct);

/*
 * XXX bring-up: dev.apple_n1.0.bt_cmd="03 0c 00" sends one raw HCI command
 * (opcode, length, parameters, in hex) next to ng_hci, to replay the macOS
 * init. The events go up the hook like any other.
 */
static int
n1_bt_cmd_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct n1_softc *sc = arg1;
	char buf[800], *p, *end;
	struct mbuf *m;
	u_long v;
	int error, i, len = 0;
	uint8_t pkt[3 + 255];

	buf[0] = '\0';
	error = sysctl_handle_string(oidp, buf, sizeof(buf), req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	for (p = buf; *p != '\0' && len < sizeof(pkt); p = end) {
		while (*p == ' ')
			p++;
		if (*p == '\0')
			break;
		v = strtoul(p, &end, 16);
		if (end == p || v > 0xff)
			return (EINVAL);
		pkt[len++] = v;
	}
	if (len < 3 || pkt[2] != len - 3)
		return (EINVAL);
	m = m_get2(len, M_WAITOK, MT_DATA, M_PKTHDR);
	m_copyback(m, 0, len, pkt);
	for (i = 0; i < 50; i++) {
		mtx_lock(&sc->bt_mtx);
		error = n1_bt_tx(sc, BT_HCI_TX, m);
		mtx_unlock(&sc->bt_mtx);
		if (error != ENOBUFS)
			break;
		pause("n1btc", hz / 100 + 1);
	}
	m_freem(m);
	return (error);
}

static int
n1_bt_start(struct n1_softc *sc)
{
	static const uint8_t fe44[24] = {
		0xbd, 0x19, 0x64, 0xa6, 0x07, 0xba, 0x5a, 0xd3, 0x60, 0xb2,
		0xbb, 0x94, 0xe8, 0xd4, 0x83, 0xa3, 0x7f, 0x9c, 0x59, 0xfe,
		0xb1, 0x5a, 0x91, 0x68,
	};
	uint8_t addr[6], ff0b[16];
	phandle_t node;
	int error, r;

	mtx_init(&sc->bt_mtx, "n1bt", NULL, MTX_DEF);
	mbufq_init(&sc->bt_vq, 64);
	mbufq_init(&sc->bt_evq, 64);
	callout_init(&sc->bt_callout, 1);
	for (r = 0; r < N1_BT_RINGS; r++) {
		if ((error = n1_bt_ring(sc, r)) != 0)
			return (error);
		if (n1_bt_rings[r].h4 == 0)
			continue;
		mtx_lock(&sc->bt_mtx);
		while (sc->bt[r].tail < n1_bt_rings[r].n - 1)
			n1_bt_post(sc, r);
		n1_bt_doorbell(sc, r);
		mtx_unlock(&sc->bt_mtx);
	}
	/*
	 * The factory BD_ADDR: local-bd-address is least significant byte
	 * first, WriteBdAddr (0xfc06) takes network byte order (aurora
	 * btcentauri cen_bt_address); sent as is, the address came out
	 * reversed.
	 */
	node = OF_finddevice("bluetooth0");
	if (node != -1 && OF_getprop(node, "local-bd-address", addr,
	    sizeof(addr)) == sizeof(addr)) {
		uint8_t be[6];
		int k;

		for (k = 0; k < 6; k++)
			be[k] = addr[5 - k];
		(void)n1_bt_cmd(sc, 0xfc06, be, sizeof(be));
	} else
		memset(addr, 0x5a, sizeof(addr));
	/* static random address: the public one, top two bits set */
	memcpy(sc->bt_raddr, addr, sizeof(addr));
	sc->bt_raddr[5] |= 0xc0;
	/*
	 * macOS then sends vendor 0xff0b (16 bytes 0xff) and 0xfe44 (24 bytes,
	 * from the trace) before Reset. Without them inquiry gets no Command
	 * Status. XXX what 0xfe44 carries is unknown; macOS also sees HCI
	 * version 0x11 where we see 0x0e.
	 */
	memset(ff0b, 0xff, sizeof(ff0b));
	(void)n1_bt_cmd(sc, 0xff0b, ff0b, sizeof(ff0b));
	(void)n1_bt_cmd(sc, 0xfe44, fe44, sizeof(fe44));
	CURVNET_SET(vnet0);	/* the boot task has no vnet */
	error = ng_make_node_common(&n1bt_typestruct, &sc->bt_node);
	if (error == 0)
		ng_name_node(sc->bt_node, "n1bt0");
	CURVNET_RESTORE();
	if (error != 0)
		return (error);
	NG_NODE_SET_PRIVATE(sc->bt_node, sc);
	NG_NODE_FORCE_WRITER(sc->bt_node);
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(sc->dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)), OID_AUTO,
	    "bt_cmd", CTLTYPE_STRING | CTLFLAG_WR | CTLFLAG_MPSAFE, sc, 0,
	    n1_bt_cmd_sysctl, "A", "send a raw HCI command (hex)");
	SYSCTL_ADD_INT(device_get_sysctl_ctx(sc->dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)), OID_AUTO,
	    "bt_debug", CTLFLAG_RW, &n1_bt_debug, 0, "print Bluetooth rx/tx");
	SYSCTL_ADD_UINT(device_get_sysctl_ctx(sc->dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)), OID_AUTO,
	    "bt_connects", CTLFLAG_RD, &sc->bt_connects, 0,
	    "successful Connection Complete events");
	mtx_lock(&sc->bt_mtx);
	n1_bt_postreset(sc);
	mtx_unlock(&sc->bt_mtx);
	callout_reset(&sc->bt_callout, hz / 100 + 1, n1_bt_tick, sc);
	return (0);
}

/* hw.apple_n1.boot=0: attach only (bring-up). */
static int n1_boot = 1;
TUNABLE_INT("hw.apple_n1.boot", &n1_boot);

/* hw.apple_n1.repower=1: power-cycle the N1 right before the ROM boot */
static int n1_repower = 0;
TUNABLE_INT("hw.apple_n1.repower", &n1_repower);

static void
n1_boot_task(void *arg, int pending __unused)
{
	struct n1_softc *sc = arg;

	if (n1_boot && n1_repower)
		(void)n1_port_cycle_power(sc, true);

	if (n1_boot && n1_rom_boot(sc) == 0 && n1_port_cycle(sc) == 0) {
		device_t *kids;
		int i, n;

		BUS_RESCAN(device_get_parent(sc->dev));
		n1_start_ipc(sc);
		if (device_get_children(device_get_parent(sc->dev), &kids,
		    &n) == 0) {
			for (i = 0; i < n; i++)
				N1_DPRINTF(sc, "function %d: %04x:%04x\n",
				    pci_get_function(kids[i]),
				    pci_get_vendor(kids[i]),
				    pci_get_device(kids[i]));
			free(kids, M_TEMP);
		}
	}
}

static void
n1_mountroot(void *arg)
{
	struct n1_softc *sc = arg;

	taskqueue_enqueue(taskqueue_thread, &sc->boot_task);
}

static int
n1_probe(device_t dev)
{
	if (pci_get_vendor(dev) != N1_VENDOR ||
	    (pci_get_device(dev) != N1_DEV_CONTROL &&
	    pci_get_device(dev) != N1_DEV_CONTROL_FW))
		return (ENXIO);
	device_set_desc(dev, "Apple N1 Wi-Fi/Bluetooth (control)");
	return (BUS_PROBE_DEFAULT);
}

static int
n1_attach(device_t dev)
{
	struct n1_softc *sc = device_get_softc(dev);
	phandle_t root;
	int error, i, rid;

	sc->dev = dev;
	n1_sc = sc;
	root = OF_finddevice("/");
	for (i = 0; i < nitems(n1_boards); i++)
		if (ofw_bus_node_is_compatible(root, n1_boards[i].compat))
			sc->board = &n1_boards[i];
	if (sc->board == NULL) {
		device_printf(dev, "unknown machine\n");
		return (ENXIO);
	}
	if ((error = n1_find_stream(sc)) != 0) {
		device_printf(dev, "no DART stream in iommu-map: %d\n", error);
		return (error);
	}
	rid = PCIR_BAR(0);
	sc->bar = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->bar == NULL) {
		device_printf(dev, "can't map BAR0\n");
		return (ENXIO);
	}
	error = apple_dart_private_stream(dev, sc->dart, sc->sid);
	if (error != 0) {
		device_printf(dev, "DART stream %u setup: %d\n", sc->sid,
		    error);
		goto fail;
	}
	N1_DPRINTF_DEV(dev, "BAR0 %#lx (%#lx bytes), stage %#x, DART sid %u\n",
	    rman_get_start(sc->bar), rman_get_size(sc->bar),
	    N1_RD4(sc, N1_STAGE), sc->sid);

	TASK_INIT(&sc->boot_task, 0, n1_boot_task, sc);
	sc->iova_next = N1_IOVA_BASE;
	sc->mountroot_tag = EVENTHANDLER_REGISTER(mountroot, n1_mountroot, sc,
	    EVENTHANDLER_PRI_ANY);
	return (0);
fail:
	bus_release_resource(dev, SYS_RES_MEMORY, PCIR_BAR(0), sc->bar);
	return (error);
}

static int
n1_detach(device_t dev)
{
	/* The firmware keeps DMAing into our buffers: never detach. */
	return (EBUSY);
}

static device_method_t n1_methods[] = {
	DEVMETHOD(device_probe,		n1_probe),
	DEVMETHOD(device_attach,	n1_attach),
	DEVMETHOD(device_detach,	n1_detach),
	DEVMETHOD_END
};

static driver_t n1_driver = {
	"apple_n1",
	n1_methods,
	sizeof(struct n1_softc)
};

DRIVER_MODULE(apple_n1, pci, n1_driver, NULL, NULL);

/*
 * Functions 1 (1902, Wi-Fi) and 2 (1903, Bluetooth) of the firmware
 * personality: claim BAR0 and hand it to the control function's driver.
 */
static int
n1fn_probe(device_t dev)
{
	if (pci_get_vendor(dev) != N1_VENDOR ||
	    (pci_get_device(dev) != 0x1902 && pci_get_device(dev) != 0x1903))
		return (ENXIO);
	device_set_desc(dev, pci_get_device(dev) == 0x1902 ?
	    "Apple N1 Wi-Fi" : "Apple N1 Bluetooth");
	return (BUS_PROBE_DEFAULT);
}

static int
n1fn_attach(device_t dev)
{
	struct resource *bar;
	int f, rid;

	f = pci_get_function(dev);
	rid = PCIR_BAR(0);
	bar = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (bar == NULL) {
		device_printf(dev, "can't map BAR0\n");
		return (ENXIO);
	}
	N1_DPRINTF_DEV(dev, "BAR0 %#lx (%#lx bytes)\n", rman_get_start(bar),
	    rman_get_size(bar));
	if (n1_sc != NULL && f > 0 && f < 3) {
		n1_sc->fn[f] = dev;
		n1_sc->fnbar[f] = bar;
	}
	return (0);
}

static device_method_t n1fn_methods[] = {
	DEVMETHOD(device_probe,		n1fn_probe),
	DEVMETHOD(device_attach,	n1fn_attach),
	DEVMETHOD_END
};

static driver_t n1fn_driver = { "apple_n1fn", n1fn_methods, 0 };

DRIVER_MODULE(apple_n1fn, pci, n1fn_driver, NULL, NULL);
