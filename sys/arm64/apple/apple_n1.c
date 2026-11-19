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

struct n1_board {
	const char	*compat;
	const char	*firmware;
	uint32_t	board_word;
};

static const struct n1_board n1_boards[] = {
	{ "apple,j815", "apple/j815-ftab.bin", 0x00268142 },
	{ "apple,j813", "apple/j813-ftab.bin", 0x00248142 },
};

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
	uint64_t		fw_iova, desc_iova, mbox_iova;
};

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

/* XXX bring-up: DART T8110 error state (ERROR 0x100, ERROR_ADDR 0x170/0x174). */
static void
n1_dart_errors(struct n1_softc *sc)
{
	bus_addr_t pa;
	bus_size_t size;
	vm_offset_t va;

	if (ofw_reg_to_paddr(sc->dart, 0, &pa, &size, NULL) != 0)
		return;
	va = (vm_offset_t)pmap_mapdev(pa, size);
	device_printf(sc->dev, "DART error %#x addr %#x%08x\n",
	    *(volatile uint32_t *)(va + 0x100),
	    *(volatile uint32_t *)(va + 0x174),
	    *(volatile uint32_t *)(va + 0x170));
	pmap_unmapdev((void *)va, size);

	/* Root port: interrupt status, link status (pcie-apple PORT_INTSTAT/LINKSTS). */
	va = (vm_offset_t)pmap_mapdev(0x488028000, 0x4000);
	device_printf(sc->dev, "port intstat %#x linksts %#x\n",
	    *(volatile uint32_t *)(va + 0x100), *(volatile uint32_t *)(va + 0x208));
	pmap_unmapdev((void *)va, 0x4000);
}

static int n1_corrupt;
TUNABLE_INT("hw.apple_n1.corrupt", &n1_corrupt);

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
	if (n1_corrupt)		/* XXX bring-up: does the ROM parse it? */
		memset((uint8_t *)sc->fw + 0x20, 0, 16);
	/* Scatter entry: u64 address, u32 length (4K-aligned), u32 flags. */
	le64enc(sc->desc, sc->fw_iova);
	le32enc((uint8_t *)sc->desc + 8, roundup2(fw->datasize, 4096));
	desc_size = 0x20;
	/* XXX bring-up experiment: the IMG4 ticket as a second scatter entry. */
	tkt = firmware_get_flags("apple/j815-centauri.im4m", FIRMWARE_GET_NOWARN);
	if (tkt != NULL) {
		void *tbuf;
		uint64_t tiova;

		tbuf = n1_dma_alloc(sc, tkt->datasize, &tiova);
		if (tbuf != NULL) {
			memcpy(tbuf, tkt->data, tkt->datasize);
			cpu_dcache_wb_range(tbuf, roundup2(tkt->datasize, N1_PAGE));
			le64enc((uint8_t *)sc->desc + 0x20, tiova);
			le32enc((uint8_t *)sc->desc + 0x28, tkt->datasize);
			desc_size = 0x40;
			device_printf(sc->dev, "ticket: %zu bytes as entry 1\n",
			    tkt->datasize);
		}
		firmware_put(tkt, FIRMWARE_UNLOAD);
	}
	cpu_dcache_wb_range(sc->fw, roundup2(fw->datasize, N1_PAGE));
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
	dsb(sy);
	N1_WR4(sc, N1_ROM_DOORBELL, 1);

	resp = 0;
	for (i = 0; i < 1000 && (resp = N1_RD4(sc, N1_ROM_RESPONSE)) == 0; i++)
		pause("n1rom", hz / 50);
	for (; i < 2000 && (stage = N1_RD4(sc, N1_STAGE)) == N1_STAGE_ROM; i++)
		pause("n1rom", hz / 50);
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

/* hw.apple_n1.boot=0: attach only (bring-up). */
static int n1_boot = 1;
TUNABLE_INT("hw.apple_n1.boot", &n1_boot);

static void
n1_boot_task(void *arg, int pending __unused)
{
	struct n1_softc *sc = arg;

	if (n1_boot)
		n1_rom_boot(sc);
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
	error = apple_dart_identity_stream(dev, sc->dart, sc->sid);
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
