/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple M5 (T8142) GPU, bring-up stage 1: identify the hardware and the
 * firmware iBoot loaded, without starting either coprocessor.
 *
 * Follows Aurora's drm/asahi G17 probe (g17/mod.rs Gpu::new, regs.rs
 * get_g17_id, fwid.rs, hw/t8142.rs, g17/pmpdash.c): hold ps_gfx on, read the
 * ID and core-mask registers, and match the loaded TEXT (the m1n1 "fw-text"
 * region, both roles back to back) against the known builds.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>

#include <crypto/sha2/sha256.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>

#include <arm64/apple/apple_pmgr.h>

/* SGX ID registers (regs.rs). */
#define	AGX_ID_VERSION		0xd04000
#define	AGX_ID_UNK08		0xd04008
#define	AGX_ID_COUNTS_1		0xd04010
#define	AGX_ID_COUNTS_2		0xd04014
#define	AGX_ID_UNK18		0xd04018
#define	AGX_ID_CLUSTERS		0xd0401c
#define	AGX_G17_PERF_CONTROL	0xe0141c
#define	AGX_G17_PERF_MAP	0xe01480
#define	AGX_CORE_MASKS		0xe01500	/* three words */

/* J813 identity (hw/t8142.rs BASE). */
#define	AGX_T8142_FAMILY	0xb
#define	AGX_T8142_VARIANT	2

static MALLOC_DEFINE(M_AGX, "apple_agx", "Apple GPU");

struct agx_fw_build {
	const char	*name;
	uint64_t	text_size;
	uint64_t	data_size;
	/* Each role's GKTS and ECAP entropy record offsets in TEXT. */
	uint32_t	entropy[2][2];
	/* Other ranges iBoot fills, zeroed before hashing. */
	uint32_t	boot_filled[8][2];
	uint8_t		text_sha256[SHA256_DIGEST_LENGTH];
};

/* hw/t8142.rs BUILD_26A428 and BUILD_26B123. */
static const struct agx_fw_build agx_t8142_builds[] = {
	{
		.name = "27.0 (26A428), G17G",
		.text_size = 0x9c000,
		.data_size = 0x184000,
		.entropy = { { 0x49d10, 0x49df3 }, { 0x997d0, 0x998b3 } },
		.boot_filled = {
			{ 0x224, 4 }, { 0x23c, 8 }, { 0x495d0, 0x740 },
			{ 0x49d10, 0x231 }, { 0x50224, 4 }, { 0x5023c, 8 },
			{ 0x99090, 0x740 }, { 0x997d0, 0x231 },
		},
		.text_sha256 = {
			0xec, 0xff, 0x40, 0xe3, 0xad, 0xb8, 0x9c, 0x90, 0x62,
			0xe9, 0x1d, 0xb6, 0x3f, 0x31, 0xba, 0xaf, 0x75, 0xeb,
			0x5d, 0x3f, 0x97, 0x22, 0xe3, 0x06, 0x95, 0x45, 0x3f,
			0xad, 0x02, 0xfc, 0xf5, 0x0c,
		},
	},
	{
		.name = "27.2 (26B123), G17G",
		.text_size = 0x9c000,
		.data_size = 0x184000,
		.entropy = { { 0x49d54, 0x49e37 }, { 0x997e4, 0x998c7 } },
		.boot_filled = {
			{ 0x224, 4 }, { 0x23c, 8 }, { 0x49614, 0x740 },
			{ 0x49d54, 0x231 }, { 0x50224, 4 }, { 0x5023c, 8 },
			{ 0x990a4, 0x740 }, { 0x997e4, 0x231 },
		},
		.text_sha256 = {
			0xe3, 0xab, 0x32, 0x7b, 0x73, 0x1b, 0xf2, 0xf1, 0x74,
			0x9b, 0x04, 0x44, 0x1f, 0x5c, 0xb5, 0xab, 0xfc, 0x23,
			0xf3, 0x41, 0xc3, 0x99, 0x72, 0x8e, 0x2c, 0xb4, 0x6f,
			0x68, 0xb1, 0x7a, 0x39, 0xb9,
		},
	},
};

struct agx_id {
	uint32_t	version;
	uint32_t	family;
	uint32_t	variant;
	uint32_t	rev;
	uint32_t	num_dies;
	uint32_t	num_clusters;
	uint32_t	num_cores;
	uint32_t	core_masks[3];
	uint32_t	active_cores;
	uint32_t	perf_control;
	uint32_t	perf_map;
};

struct apple_agx_softc {
	device_t		dev;
	uint8_t			*asc;
	uint8_t			*sgx;
	bus_size_t		asc_size;
	bus_size_t		sgx_size;
	struct agx_id		id;
	const struct agx_fw_build *build;
};

static struct ofw_compat_data compat_data[] = {
	{ "apple,agx-t8142",	1 },
	{ NULL,			0 }
};

/* A memory-region of node by its memory-region-names entry. */
static int
agx_region(phandle_t node, const char *name, uint64_t *base, uint64_t *size)
{
	pcell_t *phs, reg[4];
	phandle_t rn;
	int idx, n;

	if (ofw_bus_find_string_index(node, "memory-region-names", name,
	    &idx) != 0)
		return (ENOENT);
	n = OF_getencprop_alloc_multi(node, "memory-region", sizeof(*phs),
	    (void **)&phs);
	if (n <= idx) {
		OF_prop_free(phs);
		return (ENOENT);
	}
	rn = OF_node_from_xref(phs[idx]);
	OF_prop_free(phs);
	if (OF_getencprop(rn, "reg", reg, sizeof(reg)) != sizeof(reg))
		return (EINVAL);
	*base = ((uint64_t)reg[0] << 32) | reg[1];
	*size = ((uint64_t)reg[2] << 32) | reg[3];
	return (*size != 0 ? 0 : EINVAL);
}

static int
agx_map(phandle_t node, int idx, uint8_t **va, bus_size_t *size)
{
	bus_addr_t pa;
	bus_size_t sz;
	int error;

	if ((error = ofw_reg_to_paddr(node, idx, &pa, &sz, NULL)) != 0)
		return (error);
	*va = pmap_mapdev(pa, sz);
	*size = sz;
	return (0);
}

static inline uint32_t
agx_sgx_read(struct apple_agx_softc *sc, bus_size_t off)
{

	return (*(volatile uint32_t *)(sc->sgx + off));
}

static int
agx_read_id(struct apple_agx_softc *sc)
{
	struct agx_id *id = &sc->id;
	uint32_t c1, max_mask, regs[3], mask;
	int i, j;

	id->version = agx_sgx_read(sc, AGX_ID_VERSION);
	c1 = agx_sgx_read(sc, AGX_ID_COUNTS_1);
	device_printf(sc->dev, "ID registers: %#x %#x %#x %#x %#x %#x\n",
	    id->version, agx_sgx_read(sc, AGX_ID_UNK08), c1,
	    agx_sgx_read(sc, AGX_ID_COUNTS_2),
	    agx_sgx_read(sc, AGX_ID_UNK18),
	    agx_sgx_read(sc, AGX_ID_CLUSTERS));

	id->perf_control = agx_sgx_read(sc, AGX_G17_PERF_CONTROL) & 1;
	id->perf_map = agx_sgx_read(sc, AGX_G17_PERF_MAP);
	for (i = 0; i < 3; i++)
		regs[i] = agx_sgx_read(sc, AGX_CORE_MASKS + 4 * i);
	device_printf(sc->dev, "core masks: %#x %#x %#x\n", regs[0], regs[1],
	    regs[2]);

	id->family = (id->version >> 24) & 0xff;
	id->variant = (id->version >> 16) & 0xff;
	id->rev = (id->version >> 8) & 0xff;
	id->num_dies = (c1 >> 16) & 0xf;
	id->num_clusters = ((c1 >> 8) & 0xff) * id->num_dies;
	id->num_cores = c1 & 0xff;
	if (id->num_cores == 0 || id->num_cores > 32 ||
	    id->num_cores * id->num_clusters > 3 * 32 ||
	    id->num_clusters > nitems(id->core_masks)) {
		device_printf(sc->dev, "invalid core counts (%u x %u)\n",
		    id->num_clusters, id->num_cores);
		return (ENXIO);
	}

	/* split_core_masks */
	max_mask = id->num_cores == 32 ? ~0u : (1u << id->num_cores) - 1;
	id->active_cores = 0;
	for (i = 0; i < (int)id->num_clusters; i++) {
		mask = regs[0] & max_mask;
		id->core_masks[i] = mask;
		id->active_cores += __builtin_popcount(mask);
		for (j = 0; j < 3; j++) {
			regs[j] = id->num_cores == 32 ? 0 : regs[j] >> id->num_cores;
			if (j < 2 && id->num_cores != 0)
				regs[j] |= regs[j + 1] << (32 - id->num_cores);
		}
	}
	if (regs[0] != 0 || regs[1] != 0 || regs[2] != 0) {
		device_printf(sc->dev, "leftover core mask %#x %#x %#x\n",
		    regs[0], regs[1], regs[2]);
		return (EIO);
	}

	device_printf(sc->dev,
	    "G17 family %#x variant %#x rev %#x: %u die(s), %u cluster(s) x "
	    "%u core slots, %u active; perf control %u map %#x\n",
	    id->family, id->variant, id->rev, id->num_dies, id->num_clusters,
	    id->num_cores, id->active_cores, id->perf_control, id->perf_map);
	if (id->family != AGX_T8142_FAMILY || id->variant != AGX_T8142_VARIANT ||
	    id->num_dies != 1) {
		device_printf(sc->dev, "unexpected GPU identity\n");
		return (ENXIO);
	}
	return (0);
}

static bool
agx_record_at(const uint8_t *text, size_t len, uint32_t off, const char *tag)
{
	uint32_t l;

	if (off > len || len - off < 16 || memcmp(text + off, tag, 4) != 0)
		return (false);
	memcpy(&l, text + off + 4, 4);
	return (le32toh(l) == 8);
}

/* fwid.rs digest_at with boot-filled ranges, on a private copy. */
static bool
agx_digest(const struct agx_fw_build *b, const uint8_t *text, uint8_t *copy,
    size_t len, uint8_t out[SHA256_DIGEST_LENGTH])
{
	static const char *const tags[2] = { "GKTS", "ECAP" };
	SHA256_CTX ctx;
	int r, t;

	for (r = 0; r < 2; r++)
		for (t = 0; t < 2; t++)
			if (!agx_record_at(text, len, b->entropy[r][t], tags[t]))
				return (false);
	memcpy(copy, text, len);
	for (r = 0; r < 2; r++)
		for (t = 0; t < 2; t++)
			memset(copy + b->entropy[r][t] + 8, 0, 8);
	for (r = 0; r < (int)nitems(b->boot_filled); r++) {
		uint32_t off = b->boot_filled[r][0], n = b->boot_filled[r][1];

		if (n == 0)
			continue;
		if (off > len || len - off < n)
			return (false);
		memset(copy + off, 0, n);
	}
	SHA256_Init(&ctx);
	SHA256_Update(&ctx, copy, len);
	SHA256_Final(out, &ctx);
	return (true);
}

static int
agx_identify_fw(struct apple_agx_softc *sc, phandle_t node)
{
	uint64_t tbase, tsize, dbase, dsize;
	uint8_t digest[SHA256_DIGEST_LENGTH];
	uint8_t *text, *copy;
	void *map;
	int error, i;

	if ((error = agx_region(node, "fw-text", &tbase, &tsize)) != 0 ||
	    (error = agx_region(node, "fw-data", &dbase, &dsize)) != 0) {
		device_printf(sc->dev,
		    "no fw-text/fw-data regions (m1n1 too old?)\n");
		return (error);
	}
	device_printf(sc->dev, "fw-text %#lx+%#lx, fw-data %#lx+%#lx\n",
	    tbase, tsize, dbase, dsize);
	if (tsize > 16 * 1024 * 1024)
		return (EINVAL);

	map = pmap_mapdev_attr(tbase, tsize, VM_MEMATTR_WRITE_COMBINING);
	text = malloc(tsize, M_AGX, M_WAITOK);
	copy = malloc(tsize, M_AGX, M_WAITOK);
	memcpy(text, map, tsize);
	pmap_unmapdev(map, tsize);

	error = ENXIO;
	for (i = 0; i < (int)nitems(agx_t8142_builds); i++) {
		const struct agx_fw_build *b = &agx_t8142_builds[i];

		if (b->text_size != tsize || b->data_size != dsize)
			continue;
		if (!agx_digest(b, text, copy, tsize, digest))
			continue;
		if (memcmp(digest, b->text_sha256, sizeof(digest)) == 0) {
			sc->build = b;
			error = 0;
			break;
		}
	}
	if (error == 0) {
		device_printf(sc->dev, "firmware %s admitted\n",
		    sc->build->name);
	} else {
		device_printf(sc->dev, "unknown GPU firmware; last digest ");
		for (i = 0; i < SHA256_DIGEST_LENGTH; i++)
			printf("%02x", digest[i]);
		printf("\n");
	}
	free(copy, M_AGX);
	free(text, M_AGX);
	return (error);
}

static int
apple_agx_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Apple M5 GPU");
	return (BUS_PROBE_DEFAULT);
}

static int
apple_agx_detach(device_t dev)
{
	struct apple_agx_softc *sc = device_get_softc(dev);

	if (sc->sgx_size != 0)
		pmap_unmapdev(sc->sgx, sc->sgx_size);
	if (sc->asc_size != 0)
		pmap_unmapdev(sc->asc, sc->asc_size);
	sc->sgx_size = sc->asc_size = 0;
	return (0);
}

static int
apple_agx_attach(device_t dev)
{
	struct apple_agx_softc *sc = device_get_softc(dev);
	phandle_t node = ofw_bus_get_node(dev);
	int error;

	sc->dev = dev;

	/* Check the firmware before touching any GPU register. */
	if ((error = agx_identify_fw(sc, node)) != 0)
		goto fail;

	/*
	 * The SGX window contains the ASC's, the mailboxes' and the GPU
	 * PMGR's, so map both directly rather than through rman.
	 */
	if ((error = agx_map(node, 0, &sc->asc, &sc->asc_size)) != 0 ||
	    (error = agx_map(node, 1, &sc->sgx, &sc->sgx_size)) != 0) {
		device_printf(dev, "cannot map registers: %d\n", error);
		goto fail;
	}

	/*
	 * The SGX registers fault while ps_gfx is gated; hold it on first
	 * (pmpdash.c asahi_j813_power_on_gfx: top state, auto off).
	 */
	if ((error = apple_pmgr_power_on(dev)) != 0) {
		device_printf(dev, "cannot power on GFX: %d\n", error);
		goto fail;
	}

	if ((error = agx_read_id(sc)) != 0)
		goto fail;

	device_printf(dev, "stage 1 done; coprocessors not started\n");
	return (0);
fail:
	apple_agx_detach(dev);
	return (error);
}

static device_method_t apple_agx_methods[] = {
	DEVMETHOD(device_probe,		apple_agx_probe),
	DEVMETHOD(device_attach,	apple_agx_attach),
	DEVMETHOD(device_detach,	apple_agx_detach),
	DEVMETHOD_END
};

static DEFINE_CLASS_0(apple_agx, apple_agx_driver, apple_agx_methods,
    sizeof(struct apple_agx_softc));

DRIVER_MODULE(apple_agx, simplebus, apple_agx_driver, NULL, NULL);
