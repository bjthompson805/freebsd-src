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
#include <sys/endian.h>
#include <sys/eventhandler.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>
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

#include <arm64/apple/apple_dart.h>

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
#define	N1_IOVA_BASE		0xf000000000ul
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
	}			control;
	void			*secondary;
	uint64_t		secondary_iova;
	uint32_t		secondary_size;
};

static struct n1_softc *n1_sc;	/* the control function, for n1fn */

#define	N1_RD4(sc, o)		bus_read_4((sc)->bar, (o))
#define	N1_WR4(sc, o, v)	bus_write_4((sc)->bar, (o), (v))

/*
 * Allocate page-aligned, physically contiguous memory and map it for the N1
 * at IOVAs just below 1 TiB (40 bits), where Linux's IOMMU layer would put
 * them for a 40-bit device. RAM itself sits above 1 TiB on these SoCs.
 */
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
	*iova = sc->iova_next;
	sc->iova_next += size;
	return (buf);
}

/* The PCIe DART stream for this function, from the host bridge's iommu-map. */
static int
n1_find_stream(struct n1_softc *sc)
{
	pcell_t *map;
	phandle_t node;
	device_t d;
	u_int rid;
	int i, n;

	node = -1;
	for (d = device_get_parent(sc->dev); d != NULL; d = device_get_parent(d)) {
		node = ofw_bus_get_node(d);
		if (node > 0 && OF_hasprop(node, "iommu-map"))
			break;
	}
	if (d == NULL)
		return (ENXIO);
	rid = pci_get_rid(sc->dev);
	n = OF_getencprop_alloc_multi(node, "iommu-map", sizeof(*map),
	    (void **)&map);
	for (i = 0; i + 3 < n; i += 4) {
		if (rid >= map[i] && rid < map[i] + map[i + 3]) {
			sc->dart = OF_node_from_xref(map[i + 1]);
			sc->sid = map[i + 2] + rid - map[i];
			OF_prop_free(map);
			return (0);
		}
	}
	if (n > 0)
		OF_prop_free(map);
	return (ENOENT);
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
		device_printf(sc->dev, "DART exceptions %#x\n", pending);
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
	device_printf(sc->dev, "port intstat %#x linksts %#x\n",
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
			device_printf(sc->dev, "ticket: %zu bytes, mode %d "
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
		device_printf(sc->dev, "doorbell %d: response %#x stage %#x "
		    "(%d ms)\n", r + 2, r2, N1_RD4(sc, N1_STAGE), i * 20);
	}
	stage = N1_RD4(sc, N1_STAGE);
	cpu_dcache_inv_range(sc->mbox, N1_PAGE);
	device_printf(sc->dev, "ROM result: response %#x stage %#x mailbox %#x "
	    "(%d ms)\n", resp, stage, le32dec(sc->mbox), i * 20);
	n1_dart_errors(sc);
	for (i = 0; i < 0x100; i += 0x20)	/* XXX bring-up */
		device_printf(sc->dev, "%#x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
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
static int
n1_port_cycle(struct n1_softc *sc)
{
	static const uint32_t reset[][2] = {
		{ 0x088, 0x110 }, { 0x100, ~0u }, { 0x148, ~0u },
		{ 0x210, ~0u }, { 0x080, 0 }, { 0x084, 0 },
		{ 0x104, 0xfffffff0 }, { 0x124, 0x100 }, { 0x16c, 0 },
		{ 0x13c, 0 }, { 0x800, 0x00100100 }, { 0x808, 0x001001ff },
		{ 0x82c, 0x00010000 }, { 0x130, 0 }, { 0x140, 0x10 },
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

	/* Bring-up: lane clocks, PERST# released, retrain to L0 */
	n1_rmw(port, PORT_APPCLK, 0, 1);
	DELAY(10);
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
	n1_rmw(port, PORT_PERST, 0, 1);
	*(volatile uint32_t *)pad = (padv & ~0xfu) | PAD_OUT_HIGH;
	DELAY(100);
	n1_wait(port, PORT_PIPE, 3, 0, 20000);
	n1_rmw(port, PORT_PERST, 1u << 16, 0);
	if (!n1_wait(port, PORT_STATUS, 1, 1, 200000))
		goto out;
	n1_rmw(port, PORT_APPCLK, 0x100, 0);
	pci_write_config(rp, 0x8bc, pci_read_config(rp, 0x8bc, 4) ^ 1, 4);
	pci_write_config(rp, 0x80, pci_read_config(rp, 0x80, 4) | 0x400, 4);
	n1_rmw(lane, 0, 0, 1u << 31);
	n1_rmw(port, PORT_LTSSM_EN, 0, 1);
	if (!n1_wait_ltssm(rp, LTSSM_L0, 2000))
		goto out;
	pause("n1l0", hz / 10 + 1);
	for (i = 15; i > 0; i--)
		if (i != 2 && i != 3)
			pci_write_config(rp, 4 * i, rpcfg[i], 4);
	id = pci_read_config(sc->dev, PCIR_DEVVENDOR, 4);
	pci_restore_state(sc->dev);
	device_printf(sc->dev, "port cycle: link up, function 0 now %#x, "
	    "stage %#x\n", id, N1_RD4(sc, N1_STAGE));
	error = 0;
out:
	for (i = 0; i < 256; i++)
		*(volatile uint32_t *)(port + PORT_MSIMAP + 4 * i) = msimap[i];
	*(volatile uint32_t *)(port + PORT_MSIADDR) = msiaddr;
	*(volatile uint32_t *)(port + PORT_MSICFG) = msicfg;
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
	sc->sid = 1;
	return (0);
}

/*
 * Firmware personality: MSI to a DMA mailbox per function (polled until
 * we have an MSI controller for the port) and bus mastering.
 */
static int
n1_functions(struct n1_softc *sc)
{
	/* Control, Alpha, and Beta vector allocations in the macOS registry. */
	static const uint16_t mme[] = {
		PCIM_MSICTRL_MME_4, PCIM_MSICTRL_MME_16, PCIM_MSICTRL_MME_2,
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
		} else {
			sc->fnmsi[i] = n1_dma_alloc(sc, N1_PAGE, &iova);
			if (sc->fnmsi[i] == NULL)
				return (ENOMEM);
		}
		pci_write_config(sc->fn[i], cap + PCIR_MSI_ADDR, (uint32_t)iova, 4);
		pci_write_config(sc->fn[i], cap + PCIR_MSI_ADDR_HIGH,
		    (uint32_t)(iova >> 32), 4);
		pci_write_config(sc->fn[i], cap + PCIR_MSI_DATA_64BIT,
		    N1_MSI_DATA + 0x100 * i, 2);
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
	device_printf(sc->dev, "PTM: common 0x50 %#x, port cfg 0x2b0 %#x, "
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
		device_printf(sc->dev, "PTM: function %d cap %#x %08x ctl %08x\n",
		    i, cap, pci_read_config(sc->fn[i], cap + 4, 4),
		    pci_read_config(sc->fn[i], cap + 8, 4));
	}
	device_printf(sc->dev, "PTM: common 0x50 %#x, port cfg 0x2b0 %#x, "
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
			device_printf(sc->dev, "%s %s: devcap %08x devctl %04x "
			    "devsts %04x devcap2 %08x devctl2 %04x\n", when,
			    i ? "fn0" : "rp", pci_read_config(d[i], cap + 4, 4),
			    pci_read_config(d[i], cap + 8, 2),
			    pci_read_config(d[i], cap + 10, 2),
			    pci_read_config(d[i], cap + 0x24, 4),
			    pci_read_config(d[i], cap + 0x28, 2));
		if (pci_find_extcap(d[i], PCIZ_AER, &cap) == 0)
			device_printf(sc->dev, "%s %s AER: ue %08x mask %08x sev "
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
		device_printf(sc->dev, "%s rp: secsts %04x rootsts %08x\n", when,
		    pci_read_config(rp, PCIR_SECSTAT_1, 2),
		    pci_read_config(rp, cap + 0x20, 4));
}

/* XXX bring-up: aop-spmi0 controller state (the N1 is spmi-wifibt at 0x0e) */
static void
n1_spmi_dump(struct n1_softc *sc, const char *when)
{
	static const uint64_t base[] = { 0x390914000, 0x390904000, 0x390900000 };
	vm_offset_t va;
	int i, j;

	for (i = 0; i < nitems(base); i++) {
		va = (vm_offset_t)pmap_mapdev(base[i], PAGE_SIZE);
		for (j = 0; j < 0x40; j += 0x10)
			device_printf(sc->dev, "spmi %s %#lx+%#x: %08x %08x %08x %08x\n",
			    when, base[i], j, *(volatile uint32_t *)(va + j),
			    *(volatile uint32_t *)(va + j + 4),
			    *(volatile uint32_t *)(va + j + 8),
			    *(volatile uint32_t *)(va + j + 12));
		pmap_unmapdev((void *)va, PAGE_SIZE);
	}
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
static const uint32_t n1_sec_holes[][2] = {
	/* SECHOLES */
	{ 0, 0 },
};

static int
n1_start_ipc(struct n1_softc *sc)
{
	uint32_t off, len, n, i, need, end, stage;
	const uint8_t *seed, *written;
	int error;

	if ((error = n1_functions(sc)) != 0)
		return (error);
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
			device_printf(sc->dev, "secondary probe: memswap at %#lx, "
			    "hole %#x-%#x\n", base, n1_sec_holes[h][0],
			    n1_sec_holes[h][1]);
	}
	/* XXX bring-up: DART protection bits on the mswb page (7 no-write, 8 no-read) */
	if (N1_MSWB_PROT != 0)
		device_printf(sc->dev, "mswb page prot %#lx: %d\n",
		    (u_long)N1_MSWB_PROT, apple_dart_pte_bits(sc->dev, sc->dart,
		    sc->sid, sc->secondary_iova + 0x200000, N1_MSWB_PROT));
	pci_write_config(sc->dev, 0xf88, (uint32_t)sc->secondary_iova, 4);
	pci_write_config(sc->dev, 0xf8c, (uint32_t)(sc->secondary_iova >> 32), 4);
	pci_write_config(sc->dev, 0xf90, need, 4);

	device_printf(sc->dev, "secondary memory: seed %u, %u bytes\n", len,
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
			device_printf(sc->dev, "secondary probe: image at %#lx, "
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
	n1_spmi_dump(sc, "before");
	if (N1_AOP_RUN) {	/* XXX bring-up: start the (iBoot-preloaded) AOP */
		vm_offset_t a = (vm_offset_t)pmap_mapdev(0x390600000, PAGE_SIZE);

		device_printf(sc->dev, "AOP cpu_control %#x -> RUN\n",
		    *(volatile uint32_t *)(a + 0x44));
		*(volatile uint32_t *)(a + 0x44) |= 0x10;
		DELAY(500000);
		device_printf(sc->dev, "AOP cpu_control %#x mbox %#x\n",
		    *(volatile uint32_t *)(a + 0x44),
		    *(volatile uint32_t *)(a + 0x48));
		pmap_unmapdev((void *)a, PAGE_SIZE);
	}
	N1_WR4(sc, N1_ROM_DESC_LO, (uint32_t)sc->desc_iova);
	N1_WR4(sc, N1_ROM_DESC_HI, (uint32_t)(sc->desc_iova >> 32));
	N1_WR4(sc, N1_ROM_DESC_SIZE, 0x20);
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
		device_printf(sc->dev, "error_detect gpio before doorbell %#x\n",
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
				device_printf(sc->dev, "XXX t=%d ltssm %#x lnk %#x "
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
	device_printf(sc->dev, "exec-stage MSI %#x after ~%d ms\n",
	    le32dec(sc->mbox), i * 10);
	n1_dart_errors(sc);
	if (le32dec(sc->mbox) == 0) {
		/* Check for device writes without accessing the endpoint. */
		cpu_dcache_inv_range(sc->secondary, roundup2(need, N1_PAGE));
		written = memcchr((const uint8_t *)sc->secondary + 0x200300, 0xa5,
		    0x180000);
		n1_pcie_dump(sc, "after", 0);
		n1_spmi_dump(sc, "after");
		if (N1_CRASH) {	/* XXX bring-up: TriggerChipCrash, then look for a crashlog */
			const uint8_t *b = sc->secondary;
			uint32_t o, k;

			device_printf(sc->dev, "TriggerChipCrash (config 0xf84 = 1)\n");
			DELAY(100000);
			pci_write_config(sc->dev, 0xf84, 1, 4);
			device_printf(sc->dev, "crash written, waiting\n");
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
				device_printf(sc->dev, "sec %#x: %32D\n", o, b + o, "");
				device_printf(sc->dev, "sec %#x: %32D\n", o + 32, b + o + 32, "");
			}
			device_printf(sc->dev, "crash scan: %u nonzero lines\n", k);
		}
		device_printf(sc->dev, "mswb poison %s (first %#tx)\n",
		    written ? "CHANGED" : "intact", written ? written -
		    (const uint8_t *)sc->secondary : 0);
		written = memcchr((const uint8_t *)sc->secondary + len, 0,
		    0x200300 - len);
		if (written != NULL)
			device_printf(sc->dev, "secondary memory populated "
			    "at offset %#tx\n",
			    written - (const uint8_t *)sc->secondary);
		else
			device_printf(sc->dev, "secondary memory unchanged beyond seed\n");
		return (ETIMEDOUT);
	}
	stage = N1_RD4(sc, N1_STAGE);
	device_printf(sc->dev, "secondary boot: stage %#x\n", stage);
	n1_dart_errors(sc);
	if (stage != 2)
		return (EIO);
	error = n1_handshake(sc->bar, &sc->control, 0x8050, 0x8054, 0x907c);
	device_printf(sc->dev, "control IPC handshake: %d\n", error);
	return (error);
}

/* hw.apple_n1.boot=0: attach only (bring-up). */
static int n1_boot = 1;
TUNABLE_INT("hw.apple_n1.boot", &n1_boot);

static void
n1_boot_task(void *arg, int pending __unused)
{
	struct n1_softc *sc = arg;

	if (n1_boot && n1_rom_boot(sc) == 0 && n1_port_cycle(sc) == 0) {
		device_t *kids;
		int i, n;

		BUS_RESCAN(device_get_parent(sc->dev));
		n1_start_ipc(sc);
		if (device_get_children(device_get_parent(sc->dev), &kids,
		    &n) == 0) {
			for (i = 0; i < n; i++)
				device_printf(sc->dev, "function %d: %04x:%04x\n",
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
	device_printf(dev, "BAR0 %#lx (%#lx bytes), stage %#x, DART sid %u\n",
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
	device_printf(dev, "BAR0 %#lx (%#lx bytes)\n", rman_get_start(bar),
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
