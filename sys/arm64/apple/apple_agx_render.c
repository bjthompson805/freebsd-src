/*-
 * SPDX-License-Identifier: MIT
 *
 * Apple M5 GPU: render-pair command encoders, translated from Aurora's
 * drm/asahi g17 (see apple_agx_render.h). Each function names the Rust it
 * follows. Records are written little endian at the Rust offsets.
 */

#ifdef _KERNEL
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#else
#include <errno.h>
#include <string.h>
#endif

#include "apple_agx_render.h"

struct rw {
	uint32_t	reg;
	uint64_t	value;
};

static void
put8(uint8_t *p, size_t off, uint8_t v)
{

	p[off] = v;
}

static void
put16(uint8_t *p, size_t off, uint16_t v)
{

	p[off] = v;
	p[off + 1] = v >> 8;
}

static void
put32(uint8_t *p, size_t off, uint32_t v)
{
	int i;

	for (i = 0; i < 4; i++)
		p[off + i] = v >> (8 * i);
}

static void
put64(uint8_t *p, size_t off, uint64_t v)
{
	int i;

	for (i = 0; i < 8; i++)
		p[off + i] = v >> (8 * i);
}

static void
put128(uint8_t *p, size_t off, uint64_t a, uint64_t b)
{

	put64(p, off, a);
	put64(p, off + 8, b);
}

static uint32_t
read32(const uint8_t *p, size_t off)
{

	return (p[off] | p[off + 1] << 8 | p[off + 2] << 16 |
	    (uint32_t)p[off + 3] << 24);
}

static uint64_t
div_ceil(uint64_t a, uint64_t b)
{

	return ((a + b - 1) / b);
}

static uint64_t
next_multiple(uint64_t a, uint64_t b)
{

	return (div_ceil(a, b) * b);
}

static bool
utile_ok(unsigned w, unsigned h)
{

	return ((w == 32 && h == 32) || (w == 32 && h == 16) ||
	    (w == 16 && h == 16));
}

/* fw/mod.rs */
#define	UNK_0A599	0x0000006000400020ull
#define	UNK_0D411	0x0000000200000001ull
#define	UNK_101D9	0x1cull
#define	UNK_1C8F8	0x8860ull

static uint64_t
work_key(uint16_t context_id, uint32_t state_word)
{

	return ((uint64_t)context_id << 8 | (state_word & 0x3f));
}

static uint64_t
ids_word(struct agx_object_ids ids)
{

	return ((uint64_t)ids.predecessor << 32 | ids.id);
}

/*
 * queue/render/m5_render_submission.rs validate. The opt-in single-layer
 * MSAA compressed depth forms stay off (m5_depth_msaa_compression = 0).
 */
static bool
compressed_zls_form(const struct agx_render_pass *p)
{
	const uint64_t z_load = 1 << 15, s_load = 1 << 14;
	const uint64_t z_store = 1 << 19, s_store = 1 << 18;
	const uint64_t z_lc = 1 << 2, s_lc = 1 << 4, z_sc = 1 << 6, s_sc = 1 << 8;
	const uint64_t z_format = 3ull << 25;
	const uint64_t known = z_load | s_load | z_store | s_store | z_lc |
	    s_lc | z_sc | s_sc | z_format;
	uint64_t c = p->zls_control;
	bool zc = (c & (z_lc | z_sc)) != 0, sc = (c & (s_lc | s_sc)) != 0;

	return (c != 0 && (c & ~known) == 0 &&
	    ((c & z_load) == 0 || (c & z_lc) != 0) &&
	    ((c & s_load) == 0 || (c & s_lc) != 0) &&
	    ((c & z_store) == 0 || (c & z_sc) != 0) &&
	    ((c & s_store) == 0 || (c & s_sc) != 0) &&
	    (!zc || (p->depth.va != 0 && p->depth.compression_va != 0)) &&
	    (!sc || (p->stencil.va != 0 && p->stencil.compression_va != 0)) &&
	    (zc || p->depth.compression_va == 0) &&
	    (sc || p->stencil.compression_va == 0));
}

int
agx_m5_render_validate(const struct agx_render_pass *p)
{
	uint32_t ss = p->sample_size, uw = p->utile_width, uh = p->utile_height;
	bool single, plain, multi;

	single = p->samples_log2 == 0 && utile_ok(uw, uh) && ss % 8 == 0 &&
	    ss * uw * uh <= 65536 && (ss != 0 || p->layers == 1);
	plain = p->zls_control == 0 || ((p->zls_control & 0x154) == 0 &&
	    p->depth.compression_va == 0 && p->stencil.compression_va == 0);
	multi = (plain || (p->layers > 1 && compressed_zls_form(p))) &&
	    p->samples_log2 >= 1 && p->samples_log2 <= 3 && utile_ok(uw, uh) &&
	    ss >= 8 && ss % 8 == 0 &&
	    ((uw * uh) << p->samples_log2) <= 2048 &&
	    ((ss * uw * uh) << p->samples_log2) <= 65536;
	if (!(single || multi) ||
	    (p->zls_control != 0 && p->depth.va == 0 && p->stencil.va == 0))
		return (EOPNOTSUPP);
	return (0);
}

/* fw/m5/target.rs TargetSlices::new, cluster_control */
static int
target_slices(size_t tilemap, size_t tpc_used, uint16_t layers, uint32_t kind,
    uint32_t segments, size_t pb_bytes, struct agx_m5_target_layout *l)
{
	static const size_t align2[8] = { 0x40, 0x40, 0x100, 0, 0x40, 0x40,
	    0x80, 0x40 };
	size_t sizes[8] = { 0 }, align[8] = { 0x40 }, bytes = 0, pages, groups;
	int i;

	if (tilemap == 0 || tpc_used == 0 || layers == 0 || kind == 0)
		return (EINVAL);
	sizes[0] = tpc_used;
	if (kind >= 2) {
		if (segments == 0 || pb_bytes == 0)
			return (EINVAL);
		sizes[1] = tilemap * segments;
		sizes[2] = 0x100 * segments;
		sizes[4] = kind * 0x300 + 0x680;
		sizes[5] = kind * 0x5c0;
		pages = (pb_bytes + 0x7fff) / 0x8000;
		groups = (pages + 0x17ff) / 0x1800 + 1;
		sizes[6] = groups * 8 * segments;
		sizes[7] = 0x80;
		memcpy(align, align2, sizeof(align));
	}
	for (i = 0; i < 8; i++) {
		if (align[i] != 0)
			bytes = (bytes + align[i] - 1) & ~(align[i] - 1);
		l->slice_offsets[i] = bytes;
		l->slice_sizes[i] = sizes[i];
		bytes += sizes[i];
	}
	l->slice_bytes = bytes;
	return (0);
}

/*
 * buffer/target.rs Layout::new and with_m5_slices, as
 * m5_render_submission.rs layout calls them: one group per cluster unless
 * the pass asks for few primitives, 64 segments.
 */
int
agx_m5_target_layout(const struct agx_render_pass *p, uint32_t clusters,
    size_t parameter_bytes, struct agx_m5_target_layout *l)
{
	size_t uw = p->utile_width, uh = p->utile_height, layers = p->layers;
	size_t utiles, tiles_x, tiles_y, macro_x, macro_y, macro, tiles;
	uint32_t groups;
	int error;

	if ((error = agx_m5_render_validate(p)) != 0)
		return (error);
	if (p->width == 0 || p->height == 0 || layers == 0 || clusters == 0 ||
	    uw == 0 || uh == 0 || 32 % uw != 0 || 32 % uh != 0)
		return (EINVAL);
	memset(l, 0, sizeof(*l));
	utiles = (32 / uw) * (32 / uh);
	tiles_x = div_ceil(p->width, 32);
	tiles_y = div_ceil(p->height, 32);
	macro_x = next_multiple(div_ceil(tiles_x, 4), 4);
	macro_y = next_multiple(div_ceil(tiles_y, 4), 4);
	macro = macro_x * macro_y;
	l->tilemap_size = next_multiple(5 * macro * utiles, 4) * 16 * layers;
	l->tpc_size = (8 * utiles * macro) / 4 * 4 * 16 * layers * clusters;
	l->layer_offset = next_multiple(l->tilemap_size, 0x1000);
	l->heap_offset = l->layer_offset + (layers > 1 ? 0x100 : 0);
	l->used = l->heap_offset + 0x200;
	tiles = tiles_x * tiles_y;
	l->tvb_blocks = next_multiple(div_ceil(tiles, 128), 8);
	if (l->tvb_blocks < 8)
		l->tvb_blocks = 8;
	if (clusters > 1 && l->tvb_blocks < 7 + 2 * layers)
		l->tvb_blocks = 7 + 2 * layers;
	if (l->tvb_blocks < 32)		/* TVB_INITIAL_PAGES.len() */
		l->tvb_blocks = 32;
	groups = p->few_primitives ? 1 : clusters;
	if ((error = target_slices(l->tilemap_size, l->tpc_size, p->layers,
	    groups, 64, parameter_bytes, l)) != 0)
		return (error);
	/* cluster_control(groups, min_parameter_buffer_pages(clusters)) */
	l->cluster_control[2] = 32 + 16 * (uint64_t)clusters;
	switch (groups) {
	case 1:
		l->cluster_control[0] = 1;
		l->cluster_control[1] = 0;
		return (0);
	case 2:
		l->cluster_control[0] = 0x40 | 4 << 8 | 4 << 19;
		break;
	case 4:
		l->cluster_control[0] = 0x40 | 5 << 8 | 5 << 19;
		break;
	case 8:
		l->cluster_control[0] = 0x40 | 6 << 8 | 6 << 19;
		break;
	default:
		return (EINVAL);
	}
	l->cluster_control[1] = 4;
	return (0);
}

/*
 * fw/render.rs Geometry, PassWords and the A18 Pro arrays, in the forms
 * M5RegisterPrograms::write_m5 lowers them: G17P programs, the one-cluster
 * deflake layout, no ray-query stage and up to eight samples.
 */
#define	TA_REGISTERS		73
#define	FRAGMENT_REGISTERS	89
#define	PARTIAL_STORE_REGISTERS	16
#define	PARTIAL_RESUME_REGISTERS 23
#define	PARTIAL_LOAD_REGISTERS	10

#define	USER_BASE		0x1000000000ull
#define	VDM_OFFSET_LIMIT	(1ull << 39)
#define	TILE_CONFIG_BASE	0x280
#define	TILE_CONFIG_LAYERED	(1u << 0)
#define	TILE_CONFIG_PROCESS_EMPTY_TILES (1u << 16)
#define	AUX_FB_FLAGS_BASE	0xc000
#define	LAYER_MODE_BASE		(1u << 15)
#define	LAYER_MODE_LAYERED	(1u << 14)
#define	LAYER_MODE_PROCESS_EMPTY (1u << 13)
#define	LAYER_MODE_SKIP_EMPTY	(1u << 12)
#define	LAYER_MODE_PIPE_MASK	0x47ff
#define	TILE_STATE_BASE		(0x3717full | 1ull << 37)
#define	TILE_STATE_LAYERED	(1ull << 32)
#define	ZLS_STORE_DEPTH		(1ull << 19)
#define	ZLS_STORE_STENCIL	(1ull << 18)
#define	ZLS_LOAD_DEPTH		(1ull << 15)
#define	ZLS_LOAD_STENCIL	(1ull << 14)
#define	ZLS_COMPRESS_DEPTH	(1ull << 6)
#define	ZLS_COMPRESS_STENCIL	(1ull << 8)
#define	ZLS_PARTIAL_CLEAR	(ZLS_COMPRESS_DEPTH | ZLS_COMPRESS_STENCIL | 0xfull << 56)
#define	STENCIL_CLEAR_BASE	0x300
#define	ZLS_SWIZZLE		0x04040404
#define	UNK_15359_PARTIAL	0x20
#define	UNK_15049		0x100000
#define	DEFLAKE_ONE_CLUSTER_20	0x20
#define	DEFLAKE_ONE_CLUSTER_2A0	0x2a0
/* fw/compute.rs ScratchRequest::new(0) */
#define	G17P_WINDOW_DISABLED	0xff0000
#define	G17P_STACK_DISABLED	8

struct geometry {
	uint64_t	tiles_x, tiles_y, region_size, macro_tiles, utile_slots;
	uint64_t	x_blocks, y_blocks, screen, pixels, macro_size;
};

struct pass_words {
	uint64_t	utile_config, tile_config, layer_mode, tib_blocks;
	uint64_t	partial_zls_control, partial_zls_reload, aux_fb_flags;
	uint64_t	bg_rsrc_spec, bg_va, partial_bg_rsrc_spec, partial_bg_va;
	uint64_t	eot_va, eot_rsrc_spec, partial_eot_va, partial_eot_rsrc_spec;
};

static int
geometry_new(const struct agx_render_pass *p, struct geometry *g)
{
	uint64_t width = p->width, height = p->height, ux, uy, utiles, mx, my;

	if (p->width == 0 || p->height == 0 || p->layers == 0 ||
	    p->layers > 2048 || !utile_ok(p->utile_width, p->utile_height))
		return (EINVAL);
	g->tiles_x = div_ceil(width, 32);
	g->tiles_y = div_ceil(height, 32);
	ux = 32 / p->utile_width;
	uy = 32 / p->utile_height;
	utiles = ux * uy;
	mx = next_multiple(div_ceil(g->tiles_x, 4), 4);
	my = next_multiple(div_ceil(g->tiles_y, 4), 4);
	g->macro_tiles = mx * my;
	g->region_size = div_ceil(5 * g->macro_tiles * utiles, 4);
	g->utile_slots = 2 * g->macro_tiles * utiles;
	g->x_blocks = (3 * mx) | ((2 * mx) << 9) | (mx << 18);
	g->y_blocks = (3 * my) | ((2 * my) << 9) | (my << 18);
	g->screen = (g->tiles_y - 1) << 12 | (g->tiles_x - 1);
	g->pixels = (width - 1) | (height - 1) << 16;
	g->macro_size = (my * uy) | ((mx * ux) << 16);
	return (0);
}

static int
usc_va(const struct agx_render_pass *p, uint32_t offset, uint64_t *va)
{
	uint64_t off = offset & ~7u;

	if (p->usc_base + off < p->usc_base)
		return (EINVAL);
	*va = p->usc_base + off;
	return (0);
}

static uint64_t
rsrc_spec(const struct agx_render_pass *p, uint64_t spec)
{

	return (p->rsrc_spec_hi ? spec : spec & 0xffffffffu);
}

static int
pass_words_new(const struct agx_render_pass *p, struct pass_words *w)
{
	uint64_t samples = 1ull << p->samples_log2;
	int error;

	if (p->samples_log2 > 3)
		return (EINVAL);
	w->tile_config = TILE_CONFIG_BASE;
	w->layer_mode = LAYER_MODE_BASE | (p->layers - 1u);
	if (p->layers > 1) {
		w->tile_config |= TILE_CONFIG_LAYERED;
		w->layer_mode |= LAYER_MODE_LAYERED | (p->process_empty_tiles ?
		    LAYER_MODE_PROCESS_EMPTY : LAYER_MODE_SKIP_EMPTY);
	}
	if (p->process_empty_tiles)
		w->tile_config |= TILE_CONFIG_PROCESS_EMPTY_TILES;
	w->partial_zls_control = p->zls_control & ~ZLS_PARTIAL_CLEAR;
	if (p->depth.va != 0)
		w->partial_zls_control |= ZLS_STORE_DEPTH;
	if (p->depth.compression_va != 0)
		w->partial_zls_control |= ZLS_COMPRESS_DEPTH;
	if (p->stencil.va != 0)
		w->partial_zls_control |= ZLS_STORE_STENCIL;
	if (p->stencil.compression_va != 0)
		w->partial_zls_control |= ZLS_COMPRESS_STENCIL;
	w->partial_zls_reload = w->partial_zls_control;
	if (p->depth.va != 0)
		w->partial_zls_reload |= ZLS_LOAD_DEPTH;
	if (p->stencil.va != 0)
		w->partial_zls_reload |= ZLS_LOAD_STENCIL;
	w->aux_fb_flags = AUX_FB_FLAGS_BASE;
	if (p->depth_bias_is_int)
		w->aux_fb_flags |= 1u << 18;
	if (p->few_primitives)
		w->aux_fb_flags |= 1;
	w->utile_config = (uint64_t)(p->utile_width / 16) << 12 |
	    (uint64_t)(p->utile_height / 16) << 14 | p->samples_log2;
	w->tib_blocks = div_ceil((uint64_t)p->sample_size * p->utile_width *
	    p->utile_height * samples, 2048);
	w->bg_rsrc_spec = rsrc_spec(p, p->bg_rsrc_spec);
	w->partial_bg_rsrc_spec = rsrc_spec(p, p->partial_bg_rsrc_spec);
	w->eot_rsrc_spec = rsrc_spec(p, p->eot_rsrc_spec);
	w->partial_eot_rsrc_spec = rsrc_spec(p, p->partial_eot_rsrc_spec);
	if ((error = usc_va(p, p->bg_offset, &w->bg_va)) != 0 ||
	    (error = usc_va(p, p->partial_bg_offset, &w->partial_bg_va)) != 0 ||
	    (error = usc_va(p, p->eot_offset, &w->eot_va)) != 0 ||
	    (error = usc_va(p, p->partial_eot_offset, &w->partial_eot_va)) != 0)
		return (error);
	return (0);
}

static int
user_offset(uint64_t va, uint64_t *off)
{

	if (va < USER_BASE)
		return (EINVAL);
	*off = va - USER_BASE;
	return (0);
}

static int
user_offset32(uint64_t va, uint64_t *off)
{
	int error;

	if ((error = user_offset(va, off)) != 0)
		return (error);
	return (*off > 0xffffffffu ? EINVAL : 0);
}

#define	RW(r, v)	(struct rw){ (r), (v) }

/* fw/render.rs ta_registers, without its SCS enable (index 1). */
static int
ta_registers(const struct agx_render_args *a, const struct geometry *g,
    const struct pass_words *w, struct rw out[TA_REGISTERS])
{
	const struct agx_render_pass *p = a->pass;
	const struct agx_render_resources *r = a->resources;
	uint64_t tilemap, layermeta, heapmeta, tpc, deflake, d20, d2a0, vdm;
	uint64_t key, ids;
	int error, n = 0;

	if ((error = user_offset(r->tilemap_va, &tilemap)) != 0 ||
	    (error = user_offset(r->layermeta_va, &layermeta)) != 0 ||
	    (error = user_offset(r->heapmeta_va, &heapmeta)) != 0 ||
	    (error = user_offset(r->tpc_va, &tpc)) != 0 ||
	    (error = user_offset32(r->deflake_va, &deflake)) != 0 ||
	    (error = user_offset32(r->deflake_va + DEFLAKE_ONE_CLUSTER_20,
	    &d20)) != 0 ||
	    (error = user_offset32(r->deflake_va + DEFLAKE_ONE_CLUSTER_2A0,
	    &d2a0)) != 0 ||
	    (error = user_offset(p->vdm_va, &vdm)) != 0)
		return (error);
	if (vdm >= VDM_OFFSET_LIMIT)
		return (EINVAL);
	heapmeta |= 1ull << 63;
	key = work_key(r->context_id, a->ta.state_word);
	ids = ids_word(r->ta_ids);
#define	P(reg, v)	out[n++] = RW(reg, v)
	P(0x01748, 1);
	P(0x10141, 0x200);
	P(0x1c039, tilemap);
	P(0x1c9c8, tilemap);
	P(0x1c0a1, tpc);
	P(0x1c031, heapmeta);
	P(0x1c9c0, heapmeta);
	P(0x1c051, 0x003a0012006b0003ull);
	P(0x1c061, 1);
	P(0x10149, w->utile_config);
	P(0x10139, p->msaa_control);
	P(0x10111, d2a0);
	P(0x1c9b0, d2a0);
	P(0x10119, d20);
	P(0x1c9b8, d20);
	P(0x1c958, 1);
	P(0x1c950, deflake | 1ull << 50);
	P(0x1c930, 0);
	P(0x1c880, vdm);
	P(0x1c079, layermeta);
	P(0x1c9d8, layermeta);
	P(0x10151, 0);
	P(0x1c199, 0);
	P(0x1c1a1, 0);
	P(0x1c1a9, 0);
	P(0x1c1b1, 0);
	P(0x1c1b9, 0);
	P(0x1c8f8, UNK_1C8F8);
	P(0x1c0b1, g->region_size);
	P(0x1c850, g->region_size);
	P(0x10131, p->msaa_control);
	P(0x10121, p->ppp_control);
	P(0x10129, g->pixels);
	P(0x101b9, g->screen);
	P(0x1c069, g->x_blocks);
	P(0x1c071, g->y_blocks);
	P(0x1c081, g->macro_tiles);
	P(0x1c0a9, g->utile_slots);
	P(0x10171, 0x100);
	P(0x10169, w->layer_mode);
	P(0x0a309, 0);
	P(0x1c8e0, ~0ull);
	P(0x1c8e8, ~0ull);
	P(0x1c898, 0);
	P(0x101e1, UNK_101D9);
	P(0x1c9e8, w->layer_mode & LAYER_MODE_PIPE_MASK);
	P(0x1a099, 0);
	P(0x1a0a1, 0);
	P(0x1a069, 0);
	P(0x1a071, 0);
	P(0x1a0c9, 0);
	P(0x1a0d1, 0);
	P(0x101c9, 0);
	P(0x0d471, 0);
	P(0x1a0f1, G17P_STACK_DISABLED);
	P(0x10799, G17P_WINDOW_DISABLED);
	P(0x1c830, r->pb_slot);
	P(0x1ca30, r->pm_scratch);
	P(0x16c39, r->pm_scratch);
	P(0x1c910, r->pm_metric);
	P(0x0a5a1, UNK_0A599);
	P(0x0d419, UNK_0D411);
	P(0x1ca10, ids);
	P(0x014a1, ids);
	P(0x0a349, ids);
	P(0x10209, key);
	P(0x1c9f0, key);
	P(0x14320, key);
	P(0x14308, r->free_list_slot);
	P(0x14318, r->ta_status_va | 1);
	P(0x01740, 1);
	P(0x1c880, d2a0);
	P(0x1c898, 1);
#undef P
	return (n == TA_REGISTERS ? 0 : EINVAL);
}

/* fw/render.rs fragment_registers, G17P tile state, without its SCS enable. */
static int
fragment_registers(const struct agx_render_args *a, const struct geometry *g,
    const struct pass_words *w, struct rw out[FRAGMENT_REGISTERS])
{
	const struct agx_render_pass *p = a->pass;
	const struct agx_render_resources *r = a->resources;
	uint64_t tile_state, key, ids;
	int n = 0;

	tile_state = TILE_STATE_BASE | (w->utile_config & 0xf000) << 28 |
	    (g->tiles_x - 1) << 44 | (g->tiles_y - 1) << 53;
	if (p->layers > 1)
		tile_state |= TILE_STATE_LAYERED;
	key = work_key(r->context_id, a->fragment.state_word);
	ids = ids_word(r->fragment_ids);
#define	P(reg, v)	out[n++] = RW(reg, v)
	P(0x01739, 1);
	P(0x10009, w->utile_config);
	P(0x15379, w->eot_rsrc_spec);
	P(0x15381, w->eot_va);
	P(0x15369, w->bg_rsrc_spec);
	P(0x15371, w->bg_va);
	P(0x15131, p->merge_upper_x);
	P(0x15139, p->merge_upper_y);
	P(0x100a1, 0);
	P(0x15069, 0);
	P(0x15071, 0);
	P(0x16058, 0);
	P(0x10019, p->msaa_control);
	P(0x100b1, g->macro_size);
	P(0x16030, g->macro_size);
	P(0x100d9, g->screen);
	P(0x0a301, 0);
	P(0x10791, G17P_WINDOW_DISABLED | 1u << 9);
	P(0x16098, r->heapmeta_va);
	P(0x15109, p->scissor_va);
	P(0x15101, p->depth_bias_va);
	P(0x15021, w->aux_fb_flags);
	P(0x15211, (uint64_t)p->height << 32 | p->width);
	P(0x15049, UNK_15049);
	P(0x10051, w->tib_blocks);
	P(0x15321, p->depth_dimensions);
	P(0x15301, p->depth_clear);
	P(0x15309, p->stencil_clear | STENCIL_CLEAR_BASE);
	P(0x15311, p->occlusion_query_va);
	P(0x15319, p->zls_control);
	P(0x15349, ZLS_SWIZZLE);
	P(0x15351, 0);
	P(0x15329, p->depth.va);
	P(0x15331, p->depth.va);
	P(0x15339, p->stencil.va);
	P(0x15341, p->stencil.va);
	P(0x15231, 0);
	P(0x15221, 0);
	P(0x15239, 0);
	P(0x15229, 0);
	P(0x15401, p->depth.stride);
	P(0x15421, p->depth.stride);
	P(0x15409, p->stencil.stride);
	P(0x15429, p->stencil.stride);
	P(0x153c1, p->depth.compression_va);
	P(0x15411, p->depth.compression_stride);
	P(0x153c9, p->depth.compression_va);
	P(0x15431, p->depth.compression_stride);
	P(0x153d1, p->stencil.compression_va);
	P(0x15419, p->stencil.compression_stride);
	P(0x153d9, p->stencil.compression_va);
	P(0x15439, p->stencil.compression_stride);
	P(0x16429, r->tilemap_va);
	P(0x16060, r->layermeta_va);
	P(0x16431, (4 * g->region_size) << 24);
	P(0x10039, w->tile_config);
	P(0x16020, 0);
	P(0x16451, 0);
	P(0x15359, 0);
	P(0x100b8, UNK_1C8F8);
	P(0x16461, r->aux_fb_va);
	P(0x16090, r->aux_fb_va);
	P(0x101e9, UNK_101D9);
	P(0x160a8, 0);
	P(0x16068, tile_state);
	P(0x1a0a9, 0);
	P(0x1a0b1, 0);
	P(0x1a079, 0);
	P(0x1a081, 0);
	P(0x1a0d9, 0);
	P(0x1a0e1, 0);
	P(0x101c1, 0);
	P(0x0d469, 0);
	P(0x1a0f9, G17P_STACK_DISABLED);
	P(0x0a5a9, UNK_0A599);
	P(0x0d429, UNK_0D411);
	P(0x160e0, ids);
	P(0x01499, ids);
	P(0x0a341, ids);
	P(0x1c838, r->pb_slot);
	P(0x1ca28, r->pm_scratch);
	P(0x10211, key);
	P(0x10420, key);
	P(0x14048, r->free_list_slot);
	P(0x14080, r->fragment_status_va | 1);
	P(0x01731, 1);
	P(0x16020, 1);
	P(0x16020, 0);
	P(0x16068, 0x40000);
#undef P
	return (n == FRAGMENT_REGISTERS ? 0 : EINVAL);
}

static void
partial_store_registers(const struct agx_render_pass *p,
    const struct pass_words *w, struct rw out[PARTIAL_STORE_REGISTERS])
{
	int n = 0;

#define	P(reg, v)	out[n++] = RW(reg, v)
	P(0x15379, w->partial_eot_rsrc_spec);
	P(0x15381, w->partial_eot_va);
	P(0x10039, w->tile_config);
	P(0x15359, UNK_15359_PARTIAL);
	P(0x15331, p->depth.va);
	P(0x153c9, p->depth.compression_va);
	P(0x15341, p->stencil.va);
	P(0x153d9, p->stencil.compression_va);
	P(0x15421, p->depth.stride);
	P(0x15431, p->depth.compression_stride);
	P(0x15429, p->stencil.stride);
	P(0x15439, p->stencil.compression_stride);
	P(0x15221, 0);
	P(0x15229, 0);
	P(0x15319, w->partial_zls_control);
	P(0x15349, ZLS_SWIZZLE);
}

static void
partial_resume_registers(const struct agx_render_pass *p,
    const struct pass_words *w, struct rw out[PARTIAL_RESUME_REGISTERS])
{
	int n = 0;

	P(0x15379, w->partial_eot_rsrc_spec);
	P(0x15381, w->partial_eot_va);
	P(0x15369, w->partial_bg_rsrc_spec);
	P(0x15371, w->partial_bg_va);
	P(0x10039, w->tile_config & ~(uint64_t)TILE_CONFIG_PROCESS_EMPTY_TILES);
	P(0x15359, UNK_15359_PARTIAL);
	P(0x15331, p->depth.va);
	P(0x153c9, p->depth.compression_va);
	P(0x15341, p->stencil.va);
	P(0x153d9, p->stencil.compression_va);
	P(0x15421, p->depth.stride);
	P(0x15431, p->depth.compression_stride);
	P(0x15429, p->stencil.stride);
	P(0x15439, p->stencil.compression_stride);
	P(0x15221, 0);
	P(0x15229, 0);
	P(0x15309, p->stencil_clear | STENCIL_CLEAR_BASE);
	P(0x15329, p->depth.va);
	P(0x153c1, p->depth.compression_va);
	P(0x15339, p->stencil.va);
	P(0x153d1, p->stencil.compression_va);
	P(0x15319, w->partial_zls_reload);
	P(0x15349, ZLS_SWIZZLE);
}

static void
partial_load_registers(const struct agx_render_pass *p,
    const struct pass_words *w, struct rw out[PARTIAL_LOAD_REGISTERS])
{
	int n = 0;

	P(0x15369, w->partial_bg_rsrc_spec);
	P(0x15371, w->partial_bg_va);
	P(0x10039, w->tile_config | TILE_CONFIG_PROCESS_EMPTY_TILES);
	P(0x15309, p->stencil_clear | STENCIL_CLEAR_BASE);
	P(0x15329, p->depth.va);
	P(0x153c1, p->depth.compression_va);
	P(0x15339, p->stencil.va);
	P(0x153d1, p->stencil.compression_va);
	P(0x15319, w->partial_zls_reload);
	P(0x15349, ZLS_SWIZZLE);
#undef P
}

/* fw/render.rs M5RegisterPrograms */
struct common_programs {
	struct rw	ta[TA_REGISTERS];
	struct rw	fragment[FRAGMENT_REGISTERS];
	struct rw	store[PARTIAL_STORE_REGISTERS];
	struct rw	resume[PARTIAL_RESUME_REGISTERS];
	struct rw	load[PARTIAL_LOAD_REGISTERS];
};

static int
write_m5(const struct agx_render_args *a, struct common_programs *c)
{
	const struct agx_render_pass *p = a->pass;
	const uint64_t load = (1u << 14) | (1u << 15);
	const uint64_t store = (1u << 18) | (1u << 19);
	struct geometry g;
	struct pass_words w;
	uint64_t partial;
	struct rw *arrays[3] = { c->store, c->resume, c->load };
	const int lengths[3] = { PARTIAL_STORE_REGISTERS,
	    PARTIAL_RESUME_REGISTERS, PARTIAL_LOAD_REGISTERS };
	int error, i, j;

	if ((error = geometry_new(p, &g)) != 0 ||
	    (error = pass_words_new(p, &w)) != 0 ||
	    (error = ta_registers(a, &g, &w, c->ta)) != 0 ||
	    (error = fragment_registers(a, &g, &w, c->fragment)) != 0)
		return (error);
	partial_store_registers(p, &w, c->store);
	partial_resume_registers(p, &w, c->resume);
	partial_load_registers(p, &w, c->load);
	partial = p->zls_control;
	if (p->depth.va != 0)
		partial |= ZLS_LOAD_DEPTH | ZLS_STORE_DEPTH;
	if (p->stencil.va != 0)
		partial |= ZLS_LOAD_STENCIL | ZLS_STORE_STENCIL;
	for (i = 0; i < 3; i++)
		for (j = 0; j < lengths[i]; j++)
			if (arrays[i][j].reg == 0x15319)
				arrays[i][j].value = partial;
	for (j = 0; j < PARTIAL_STORE_REGISTERS; j++) {
		if (c->store[j].reg == 0x15319)
			c->store[j].value = (c->store[j].value & ~load) |
			    (p->zls_control & load);
		else if (c->store[j].reg == 0x10039 &&
		    (p->depth.va != 0 || p->stencil.va != 0))
			c->store[j].value |= TILE_CONFIG_PROCESS_EMPTY_TILES;
	}
	for (j = 0; j < PARTIAL_LOAD_REGISTERS; j++)
		if (c->load[j].reg == 0x15319)
			c->load[j].value = (c->load[j].value & ~store) |
			    (p->zls_control & store);
	return (0);
}

/* fw/m5/render_registers.rs M5RenderControls, with its stage controls. */
struct stage_controls {
	struct agx_scratch scratch;
	struct agx_intersection intersection;
	uint32_t	auxiliary_allocation;
	uint64_t	resource_flags;
	uint64_t	resource_control;
	uint32_t	ptc_max;
	uint64_t	work_key;
};

struct controls {
	const struct agx_board *board;
	uint64_t	fragment_event_control;
	struct stage_controls ta, fragment;
	uint64_t	ta_tile_control;
	uint64_t	ta_dispatch_control;
	uint64_t	fragment_raster_control;
	uint64_t	fragment_tile_flags;
	uint64_t	store_pipeline_2_bind;
	uint64_t	store_pipeline_2;
	uint64_t	partial_store_pipeline_2_bind;
	uint64_t	partial_store_pipeline_2;
	uint64_t	partial_tile_flags;
	uint64_t	aux_fb_flags;
	uint64_t	aux_fb_control;
};

/* m5_render_submission.rs controls, then the plan's stages (memory.rs). */
static int
controls_new(const struct agx_render_args *a, struct controls *c)
{
	const struct agx_render_pass *p = a->pass;
	int error, i;

	if ((error = agx_m5_render_validate(p)) != 0)
		return (error);
	memset(c, 0, sizeof(*c));
	c->board = a->board;
	c->fragment_event_control = 3;		/* fragment_event_control(true, 1) */
	c->ta.resource_flags = 0x4054b;
	c->fragment.resource_flags = 0x104054b;
	for (i = 0; i < 2; i++) {
		struct stage_controls *s = i == 0 ? &c->ta : &c->fragment;

		s->resource_control = 0x200000001ull;
		s->ptc_max = 96;
		s->scratch.layout = 0;		/* StageScratch::NONE */
		s->scratch.driver_end = 0;
		s->scratch.pages = 0;
		if (a->has_stages) {
			s->scratch = a->stages[i].scratch;
			s->intersection = a->stages[i].intersection;
			s->auxiliary_allocation =
			    a->stages[i].auxiliary_allocation;
			s->resource_flags = a->stages[i].resource_flags;
			s->ptc_max = a->stages[i].ptc_max;
		}
	}
	c->ta_tile_control = 0x10c08860;
	c->ta_dispatch_control = 0x700;
	c->fragment_raster_control = 0x200000ff0200ull;
	c->fragment_tile_flags = 0x200003717full;
	c->store_pipeline_2_bind = p->eot_rsrc_spec << 32;
	c->store_pipeline_2 = p->usc_base + (p->eot_offset & ~7u);
	c->partial_store_pipeline_2_bind = p->partial_eot_rsrc_spec << 32;
	c->partial_store_pipeline_2 = p->usc_base +
	    (p->partial_eot_offset & ~7u);
	c->partial_tile_flags = 0x200000;
	c->aux_fb_flags = 0x10c000 | (uint64_t)p->few_primitives |
	    (uint64_t)p->depth_bias_is_int << 18;
	c->aux_fb_control = 0x100000;
	return (0);
}

/* fw/m5/render.rs M5RenderCommandInputs, the fields this path sets. */
struct stage_completion {
	uint64_t	stamp_addresses[2];
	uint32_t	stamp;
	uint32_t	queue_index;
	uint32_t	ordinal;
	bool		has_wait;
	uint64_t	completion;
	uint64_t	key;
};

struct ta_fields {
	uint64_t	sequence;
	struct stage_completion completion;
	uint32_t	pb_slot;
	uint64_t	pb_state, pm_scene, scene_end_va;
	uint64_t	context_token, tail_descriptor, resolved_tail_value;
	uint64_t	tpc_va, tpc_size;
	uint32_t	linked_queue_index, linked_stamp;
	uint8_t		linked_flag;
	uint64_t	linked_completion;
	uint64_t	deflake_state_va;
	uint32_t	predecessor_id;
	uint32_t	stage_word_b;
	uint64_t	timestamp_va, diagnostic_ta_timestamp_upper;
	uint64_t	shifted_usage_hint;
	uint64_t	uma_state;
	uint8_t		tail_flag, pb_growth_request, shader_write_class;
};

struct fragment_fields {
	uint64_t	sequence;
	struct stage_completion completion;
	uint8_t		pass;
	uint64_t	context_token, tail_descriptor;
	bool		has_resolved;
	uint64_t	resolved_tail_value;
	uint8_t		short_partial_programs;
	uint8_t		tail_flag;
	uint64_t	pm_state, pm_scene, scene_end_va;
	uint64_t	tilemap_va, msaa_control;
	uint32_t	sample_count, utile_grid;
	uint64_t	merge_bounds[2];
	uint32_t	tile_count;
	uint64_t	depth_bias_va, scissor_va, occlusion_query_va;
	uint64_t	binding_pairs[23][2];
	uint64_t	depth_dimensions, depth_va, depth_va_pair[2];
	uint64_t	stencil_va, stencil_va_pair[2];
	uint64_t	depth_stride, stencil_stride;
	uint64_t	depth_compression_va, depth_compression_stride;
	uint64_t	stencil_compression_va, stencil_compression_stride;
	uint64_t	sample_aux_flags[2], size_aux_control[2];
	uint64_t	eot_program[2], partial_eot_program[2];
	uint64_t	zs_clear_values;
	uint32_t	sample_size;
	uint8_t		fragment_pass_flag;
	uint32_t	fragment_word_n;
	uint8_t		fragment_flag_c;
	uint32_t	predecessor_id;
	uint32_t	stage_word_b;
	uint64_t	uma_state;
	uint64_t	timestamp_va[2];
	uint8_t		empty_tile_flags[4];
};

struct inputs {
	uint32_t	context_id;
	uint64_t	scheduler_record;
	struct ta_fields ta;
	struct fragment_fields fragment;
};

/* m5_render_submission.rs inputs */
static int
inputs_new(const struct agx_render_args *a, struct inputs *in)
{
	const struct agx_render_resources *r = a->resources;
	uint64_t token;
	uint32_t stamp;
	int i;

	memset(in, 0, sizeof(*in));
	in->ta.stage_word_b = ~0u;
	in->fragment.stage_word_b = ~0u;
	/* render_tail.rs context_token */
	if (r->context_id >= 64 || r->context_generation == 0)
		return (EINVAL);
	token = (uint64_t)r->context_id << 58 | r->context_generation;
	in->ta.context_token = token;
	in->fragment.context_token = token;
	in->ta.resolved_tail_value = a->usage[0];
	in->fragment.has_resolved = true;
	in->fragment.resolved_tail_value = a->usage[1];
	in->scheduler_record = r->kick_record_va;
	in->ta.shader_write_class = a->pass->shader_writes_memory;
	in->ta.sequence = a->ordinal * 2 + 1;
	in->fragment.sequence = a->ordinal * 2;
	in->fragment.pass = 1;
	in->ta.predecessor_id = r->ta_ids.predecessor;
	in->fragment.predecessor_id = r->fragment_ids.predecessor;
	in->fragment.fragment_word_n = 1;
	in->fragment.fragment_flag_c = 1;
	in->fragment.binding_pairs[22][1] = 0x04040404;
	in->ta.timestamp_va = r->timestamp_va;
	in->ta.diagnostic_ta_timestamp_upper = r->timestamp_va + 8;
	in->fragment.timestamp_va[0] = r->timestamp_va + 16;
	in->fragment.timestamp_va[1] = r->timestamp_va + 24;
	stamp = ((uint32_t)a->ordinal + 1) << 8;
	in->ta.linked_queue_index = a->fragment.qid;
	in->ta.linked_stamp = stamp;
	for (i = 0; i < 2; i++) {
		const struct agx_render_stage *s = i == 0 ? &a->ta :
		    &a->fragment;
		struct stage_completion *c = i == 0 ? &in->ta.completion :
		    &in->fragment.completion;

		c->stamp_addresses[0] = s->stamp_va;
		c->stamp_addresses[1] = s->aux_stamp_va;
		c->stamp = stamp;
		c->queue_index = s->qid;
		c->ordinal = (uint32_t)a->ordinal;
		c->key = i == 0 ? r->ta_ids.id : r->fragment_ids.id;
	}
	in->ta.pb_slot = r->pb_slot;
	in->ta.pb_state = r->pb_va;
	in->fragment.pm_state = r->pb_va;
	in->ta.pm_scene = r->scene_va;
	in->fragment.pm_scene = r->scene_va;
	in->ta.scene_end_va = r->scene_end_va;
	in->fragment.scene_end_va = r->scene_end_va;
	in->ta.uma_state = r->free_list_control_va;
	in->fragment.uma_state = r->free_list_control_va;
	in->ta.tail_flag = 1;
	in->fragment.tail_flag = 1;
	/* memory.rs: render::mesh_class_usage_hint */
	in->ta.shifted_usage_hint = a->mesh_class == 0 ? 0xc0c0 :
	    a->mesh_class == 1 ? 0x10ac0 : 0x12b40;
	return (0);
}

/* render_registers.rs ContextStore::selected */
static void
context_store(const struct agx_board *b, uint64_t *auxiliary, uint64_t *stream)
{
	uint64_t clusters = b->g17d_forms || b->clusters == 4 ? b->clusters : 2;

	*auxiliary = (0x20 + 0x40 * clusters + 63) & ~63ull;
	*stream = *auxiliary + ((0x2e0 * (clusters + 1) + 63) & ~63ull);
}

/* render_registers.rs isp_sample_control */
static uint64_t
isp_sample_control(const struct agx_render_pass *p)
{
	uint32_t samples = 1u << p->samples_log2, bytes, groups, units, value;

	if (samples <= 1)
		return (0);
	bytes = ((p->sample_size > 8 ? p->sample_size : 8) + 7) & ~7u;
	groups = div_ceil(bytes, 32);
	units = (p->utile_width >> 3) * (p->utile_height >> 2) * groups;
	switch (samples) {
	case 2:
		value = (units > 8 ? units : 8) >> 3;
		break;
	case 4:
		value = units >> 1;
		break;
	case 8:
		value = units;
		break;
	default:
		value = 0;
	}
	if (!(samples == 2 && p->utile_width == 16 && p->utile_height == 16))
		value >>= 1;
	return ((uint64_t)(value & 0x1f) << 16);
}

/* render_registers.rs apply_pass */
static int
apply_pass(const struct agx_render_args *a, const struct controls *c,
    struct inputs *in)
{
	const struct agx_render_pass *p = a->pass;
	const struct agx_render_resources *r = a->resources;
	struct fragment_fields *f = &in->fragment;
	uint64_t bg, pbg, eot, peot, aux, stream;
	uint32_t tx, ty, mx, my;
	int error;

	if (p->width == 0 || p->width > 32768 || p->height == 0 ||
	    p->height > 32768 || p->layers == 0 || p->layers > 2048 ||
	    r->context_id == 0 || r->context_id > 63 || p->samples_log2 > 3 ||
	    !utile_ok(p->utile_width, p->utile_height))
		return (EINVAL);
	if ((error = usc_va(p, p->bg_offset, &bg)) != 0 ||
	    (error = usc_va(p, p->partial_bg_offset, &pbg)) != 0 ||
	    (error = usc_va(p, p->eot_offset, &eot)) != 0 ||
	    (error = usc_va(p, p->partial_eot_offset, &peot)) != 0)
		return (error);
	if (r->deflake_va < USER_BASE)
		return (EINVAL);
	context_store(c->board, &aux, &stream);
	in->context_id = r->context_id;
	in->ta.tpc_va = r->tpc_va;
	in->ta.tpc_size = r->tpc_size;
	in->ta.deflake_state_va = r->deflake_va - USER_BASE + stream;
	in->ta.pb_growth_request = c->board->g17d_forms;
	f->empty_tile_flags[0] = p->process_empty_tiles;
	f->fragment_pass_flag = c->board->g17d_forms;
	if (p->samples_log2 == 0) {
		f->empty_tile_flags[1] = p->process_empty_tiles;
		f->empty_tile_flags[2] = 0;
	} else {
		f->empty_tile_flags[1] = p->process_empty_tiles &&
		    (p->bg_rsrc_spec & (1u << 6)) != 0;
		f->empty_tile_flags[2] = p->process_empty_tiles;
	}
	f->depth_bias_va = p->depth_bias_va;
	f->scissor_va = p->scissor_va;
	f->occlusion_query_va = p->occlusion_query_va;
	f->binding_pairs[18][0] = rsrc_spec(p, p->bg_rsrc_spec);
	f->binding_pairs[18][1] = bg;
	f->binding_pairs[21][0] = rsrc_spec(p, p->partial_bg_rsrc_spec);
	f->binding_pairs[21][1] = pbg;
	f->binding_pairs[22][0] = p->zls_control;
	f->depth_dimensions = p->depth_dimensions;
	f->depth_va = p->depth.va;
	f->depth_va_pair[0] = f->depth_va_pair[1] = p->depth.va;
	f->stencil_va = p->stencil.va;
	f->stencil_va_pair[0] = f->stencil_va_pair[1] = p->stencil.va;
	f->depth_stride = p->depth.stride;
	f->stencil_stride = p->stencil.stride;
	f->depth_compression_va = p->depth.compression_va;
	f->depth_compression_stride = p->depth.compression_stride;
	f->stencil_compression_va = p->stencil.compression_va;
	f->stencil_compression_stride = p->stencil.compression_stride;
	f->sample_aux_flags[0] = isp_sample_control(p);
	f->sample_aux_flags[1] = c->aux_fb_flags;
	f->size_aux_control[0] = p->width | (uint64_t)p->height << 32;
	f->size_aux_control[1] = c->aux_fb_control;
	f->eot_program[0] = rsrc_spec(p, p->eot_rsrc_spec) << 32;
	f->eot_program[1] = eot;
	f->partial_eot_program[0] = rsrc_spec(p, p->partial_eot_rsrc_spec) << 32;
	f->partial_eot_program[1] = peot;
	f->zs_clear_values = p->depth_clear |
	    (uint64_t)(p->stencil_clear | 0x300) << 32;
	f->tilemap_va = r->tilemap_va;
	f->msaa_control = p->msaa_control;
	f->sample_count = 1u << p->samples_log2;
	tx = div_ceil(p->width, 32);
	ty = div_ceil(p->height, 32);
	mx = next_multiple(div_ceil(tx, 4), 4);
	my = next_multiple(div_ceil(ty, 4), 4);
	f->utile_grid = ((mx * (32 / p->utile_width)) << 16) |
	    (my * (32 / p->utile_height));
	f->merge_bounds[1] = p->merge_upper_x |
	    (uint64_t)p->merge_upper_y << 32;
	f->tile_count = tx * ty;
	f->sample_size = p->sample_size;
	return (0);
}

/* render_registers.rs RegisterWriter over a fixed-size array */
struct writer {
	struct rw	*values;
	int		count, capacity;
};

static int
wpush(struct writer *w, uint32_t reg, uint64_t value)
{

	if (w->count >= w->capacity)
		return (EINVAL);
	w->values[w->count++] = RW(reg, value);
	return (0);
}

static int
value_of(const struct rw *regs, int n, uint32_t reg, uint64_t *value)
{
	int i;

	for (i = 0; i < n; i++)
		if (regs[i].reg == reg) {
			*value = regs[i].value;
			return (0);
		}
	return (EINVAL);
}

static uint64_t
ptc_control(uint32_t maximum)
{

	return ((uint64_t)(maximum == 0 ? 0x60 : maximum) << 32 | 0x400020);
}

static int
tile_state(const struct agx_render_pass *p, uint64_t flags, uint64_t *out)
{
	uint64_t tiles_x = div_ceil(p->width, 32), tiles_y = div_ceil(p->height, 32);
	uint64_t utile = (uint64_t)(p->utile_width / 16) << 12 |
	    (uint64_t)(p->utile_height / 16) << 14;

	if (tiles_x < 1 || tiles_x > 1024 || tiles_y < 1 || tiles_y > 1024)
		return (EINVAL);
	if ((flags & ((0xfffffull << 42) | (0xfull << 38) | (1ull << 32))) != 0)
		return (EINVAL);
	*out = flags | (tiles_x - 1) << 42 | (tiles_y - 1) << 52 |
	    (utile & 0xf000) << 26 | (p->layers > 1 ? 1ull << 32 : 0);
	return (0);
}

static uint64_t
tile_blocks(uint32_t pixels)
{
	uint64_t tiles = div_ceil(pixels, 32);
	uint64_t first = ((tiles + 3) / 4 + 3) & ~3ull;

	return ((3 * first) | ((2 * first) << 10) | (first << 20));
}

#define	CHECK(e)	do { if ((error = (e)) != 0) return (error); } while (0)

/* render_registers.rs build_ta_registers_into */
static int
build_ta_registers(const struct agx_render_pass *p, const struct controls *c,
    uint32_t free_list_slot, struct rw out[82], const struct rw common[73])
{
	struct writer w = { out, 0, 82 };
	struct rw r;
	uint64_t v, ift[6], geometry;
	int error, i, j;
	static const uint32_t copies[3][2] = { { 0x1ca50, 0x101b9 },
	    { 0x1ca58, 0x10149 }, { 0x1ca60, 0x10141 } };

	if ((c->ta_dispatch_control & 1) != 0)
		return (EINVAL);
	CHECK(tile_state(p, c->fragment_tile_flags, &geometry));
	agx_intersection_words(&c->ta.intersection, ift);
	for (i = 0; i < 73; i++) {
		r = common[i];
		switch (r.reg) {
		case 0x0a309:
		case 0x0a5a1:
		case 0x0d419:
			continue;
		case 0x10131:
			r.value = 0x88;
			break;
		case 0x1c8f8:
			CHECK(wpush(&w, 0x0d499, c->ta.resource_flags));
			CHECK(wpush(&w, 0x0d419, c->ta.resource_control));
			CHECK(wpush(&w, 0x0a5a1, ptc_control(c->ta.ptc_max)));
			r.value = agx_m5_dispatch_control(c->board,
			    c->ta_tile_control);
			break;
		case 0x10171:
			for (j = 0; j < 3; j++) {
				CHECK(value_of(common, 73, copies[j][1], &v));
				CHECK(wpush(&w, copies[j][0], v));
			}
			CHECK(wpush(&w, 0x1ca70, tile_blocks(p->width)));
			CHECK(wpush(&w, 0x1ca78, tile_blocks(p->height)));
			CHECK(value_of(common, 73, 0x1c081, &v));
			CHECK(wpush(&w, 0x1ca68, v));
			break;
		case 0x1c069:
			r.value = tile_blocks(p->width);
			break;
		case 0x1c071:
			r.value = tile_blocks(p->height);
			break;
		case 0x1c898:
			if (r.value > 1)
				return (EINVAL);
			r.value |= c->ta_dispatch_control;
			break;
		case 0x101c9:
			r.value = agx_scratch_size_word(&c->ta.scratch);
			break;
		case 0xd471:
			r.value = c->ta.auxiliary_allocation;
			break;
		case 0x1a0f1:
			r.value = agx_scratch_stack_word(&c->ta.scratch);
			break;
		case 0x1a099:
			r.value = ift[0];
			break;
		case 0x1a0a1:
			r.value = ift[1];
			break;
		case 0x1a069:
			r.value = ift[2];
			break;
		case 0x1a071:
			r.value = ift[3];
			break;
		case 0x1a0c9:
			r.value = ift[4];
			break;
		case 0x1a0d1:
			r.value = ift[5];
			break;
		case 0x10799:
			r.value = agx_scratch_window_word(&c->ta.scratch) |
			    (r.value & (1u << 25));
			CHECK(wpush(&w, 0x1a149,
			    agx_scratch_driver_word(&c->ta.scratch)));
			break;
		case 0x10209:
			CHECK(wpush(&w, 0x0a739, 0));
			r.value = c->ta.work_key;
			break;
		case 0x1c9f0:
		case 0x14320:
			r.value = c->ta.work_key;
			break;
		case 0x14308:
			r.value = (uint64_t)(int64_t)(int32_t)free_list_slot;
			break;
		}
		CHECK(wpush(&w, r.reg, r.value));
		if (r.reg == 0x01740)
			CHECK(wpush(&w, 0x0a739, 0));
	}
	return (w.count == 82 ? 0 : EINVAL);
}

/* render_registers.rs build_fragment_registers_into */
static int
build_fragment_registers(const struct agx_render_pass *p,
    const struct controls *c, uint32_t free_list_slot, struct rw out[90],
    const struct rw common[89])
{
	struct writer w = { out, 0, 90 };
	const struct agx_scratch *s = &c->fragment.scratch;
	struct rw r;
	uint64_t geometry, ift[6];
	int error, i, dispatch_writes = 0, tile_writes = 0;

	CHECK(tile_state(p, c->fragment_tile_flags, &geometry));
	agx_intersection_words(&c->fragment.intersection, ift);
	for (i = 0; i < 89; i++) {
		r = common[i];
		switch (r.reg) {
		case 0x0a301:
		case 0x15109:
		case 0x15101:
		case 0x160a8:
		case 0x0a5a9:
		case 0x0d429:
			continue;
		case 0x100b1:
			r.reg = 0x16469;
			break;
		case 0x100d9:
			r.reg = 0x16471;
			break;
		case 0x100b8:
			r.reg = 0x16138;
			r.value = agx_m5_dispatch_control(c->board, r.value);
			break;
		case 0x10791:
			if (agx_scratch_enabled(s))
				r.value = ((c->fragment_raster_control |
				    (1u << 24)) & 0x200001000204ull) |
				    (agx_scratch_window_word(s) & 0xff0000) |
				    (c->fragment_raster_control & (1u << 25));
			else
				r.value = c->fragment_raster_control;
			break;
		case 0x101c1:
			r.value = agx_scratch_size_word(s);
			break;
		case 0xd469:
			r.value = c->fragment.auxiliary_allocation;
			break;
		case 0x1a0f9:
			r.value = agx_scratch_stack_word(s);
			break;
		case 0x1a0a9:
			r.value = ift[0];
			break;
		case 0x1a0b1:
			r.value = ift[1];
			break;
		case 0x1a079:
			r.value = ift[2];
			break;
		case 0x1a081:
			r.value = ift[3];
			break;
		case 0x1a0d9:
			r.value = ift[4];
			break;
		case 0x1a0e1:
			r.value = ift[5];
			break;
		case 0x15321:
			CHECK(wpush(&w, 0x0d491, c->fragment.resource_flags));
			CHECK(wpush(&w, 0x0d429, c->fragment.resource_control));
			CHECK(wpush(&w, 0x0a5a9,
			    ptc_control(c->fragment.ptc_max)));
			break;
		case 0x16068:
			if (++tile_writes == 1)
				r.value = geometry;
			break;
		case 0x16020:
			if (++dispatch_writes == 3) {
				if (r.value != 0)
					return (EINVAL);
				continue;
			}
			break;
		case 0x160e0:
			CHECK(wpush(&w, 0x1a151, agx_scratch_driver_word(s)));
			break;
		case 0x10211:
			CHECK(wpush(&w, 0x0a741, c->fragment_event_control));
			r.value = c->fragment.work_key;
			break;
		case 0x10420:
			r.value = c->fragment.work_key;
			break;
		case 0x14048:
			r.value = (uint64_t)(int64_t)(int32_t)free_list_slot;
			break;
		}
		CHECK(wpush(&w, r.reg, r.value));
		if (r.reg == 0x15381) {
			CHECK(wpush(&w, 0x153b1, c->store_pipeline_2_bind));
			CHECK(wpush(&w, 0x153b9, c->store_pipeline_2));
		}
		if (r.reg == 0x01731)
			CHECK(wpush(&w, 0x0a741, 0));
	}
	if (dispatch_writes != 3 || tile_writes != 2)
		return (EINVAL);
	return (w.count == 90 ? 0 : EINVAL);
}

/*
 * render_registers.rs tiling_with_scratch_into, for the J813 form only:
 * the common registers unchanged inside the program's own 0x1748/0x1740
 * bracket, with any SCS and matrix brackets.
 */
static int
tiling_program(const struct rw *common, int n, bool scs, bool matrix,
    bool j813, struct rw out[99], int *length)
{
	struct writer w = { out, 0, 99 };
	int error, i;

	if (!j813)
		return (EOPNOTSUPP);	/* the tiler sub-buffer form */
	if (scs)
		CHECK(wpush(&w, 0x17e1, 1));
	if (matrix)
		CHECK(wpush(&w, 0x17f1, 1));
	for (i = 0; i < n; i++) {
		if (common[i].reg == 0x1748) {
			CHECK(wpush(&w, 0x1748, 1));
			continue;
		}
		if (common[i].reg == 0x1740) {
			if (scs)
				CHECK(wpush(&w, 0x17d9, 1));
			if (matrix)
				CHECK(wpush(&w, 0x17e9, 1));
			CHECK(wpush(&w, 0x1740, 1));
			continue;
		}
		CHECK(wpush(&w, common[i].reg, common[i].value));
	}
	*length = w.count;
	return (0);
}

/* render_registers.rs fragment_main_forms_into */
static int
fragment_main(const struct agx_render_pass *p, const struct rw *common,
    int ncommon, bool scs, bool j813, struct rw *out, int count)
{
	struct rw r;
	int i, n = 0, tile = 0;

	if (!j813)
		return (EOPNOTSUPP);	/* FragmentDispatch::initial */
	for (i = 0; i < ncommon; i++) {
		r = common[i];
		if (r.reg == 0x10051)
			r.value = isp_sample_control(p);
		if (r.reg == 0x16068 && tile++ == 2)
			return (EINVAL);
		if (n >= count)
			return (EINVAL);
		out[n++] = r;
		if (scs && (r.reg == 0x1739 || r.reg == 0x1731)) {
			if (n >= count)
				return (EINVAL);
			out[n++] = RW(r.reg == 0x1739 ? 0x17e1 : 0x17d9, 1);
		}
	}
	return (n == count && tile == 2 ? 0 : EINVAL);
}

/* render_registers.rs partial */
static int
partial(const struct rw *common, int n, uint32_t insert_after,
    const struct controls *c, bool load, struct rw *out, int count)
{
	struct writer w = { out, 0, count };
	int error, i;

	for (i = 0; i < n; i++) {
		if (common[i].reg == 0x15359)
			continue;
		CHECK(wpush(&w, common[i].reg, common[i].value |
		    (common[i].reg == 0x10039 ? c->partial_tile_flags : 0)));
		if (common[i].reg == insert_after) {
			CHECK(wpush(&w, 0x153b1, load ?
			    c->store_pipeline_2_bind :
			    c->partial_store_pipeline_2_bind));
			CHECK(wpush(&w, 0x153b9, load ? c->store_pipeline_2 :
			    c->partial_store_pipeline_2));
		}
	}
	return (w.count == count ? 0 : EINVAL);
}

/* render_registers.rs Workspace */
struct workspace {
	struct common_programs common;
	struct rw	base[90];
	struct rw	ta[99];
	int		ta_count;
	struct rw	fragment[94];
	int		fragment_count;
	struct rw	store[17];
	struct rw	resume[24];
	struct rw	load[12];
};

static int
workspace_write(struct workspace *ws, const struct agx_render_args *a,
    const struct controls *controls, const struct inputs *in)
{
	const struct agx_render_resources *r = a->resources;
	const struct agx_render_pass *p = a->pass;
	struct common_programs *cp = &ws->common;
	struct controls c;
	uint64_t base, aux, stream, lifecycle, empty;
	bool vdm_seen = false, j813;
	int error, i;

	CHECK(write_m5(a, cp));
	if (r->deflake_va < USER_BASE)
		return (EINVAL);
	base = r->deflake_va - USER_BASE;
	context_store(controls->board, &aux, &stream);
	for (i = 0; i < TA_REGISTERS; i++) {
		struct rw *reg = &cp->ta[i];

		switch (reg->reg) {
		case 0x1c880:
			if (vdm_seen)
				reg->value = base + stream;
			break;
		case 0x10111:
		case 0x1c9b0:
			reg->value = base + stream;
			break;
		case 0x10119:
		case 0x1c9b8:
			reg->value = base + aux;
			break;
		case 0x1c079:
		case 0x1c9d8:
			if (p->layers == 1) {
				if (r->heapmeta_va < USER_BASE)
					return (EINVAL);
				reg->value = r->heapmeta_va - USER_BASE;
			}
			break;
		}
		if (reg->reg == 0x1c880)
			vdm_seen = true;
	}
	for (i = 0; i < FRAGMENT_REGISTERS; i++) {
		struct rw *reg = &cp->fragment[i];

		switch (reg->reg) {
		case 0x16060:
			if (p->layers == 1)
				reg->value = r->heapmeta_va;
			break;
		case 0x15021:
			reg->value = controls->aux_fb_flags;
			break;
		case 0x15049:
			reg->value = controls->aux_fb_control;
			break;
		case 0x10039:
			reg->value |=
			    (uint64_t)(!p->process_empty_tiles &&
			    (in->fragment.empty_tile_flags[3] & 1) == 0) << 17 |
			    (uint64_t)(p->depth.va != 0 &&
			    ((p->zls_control & (1u << 15)) != 0 ||
			    (p->zls_control & (1u << 19)) == 0)) << 18 |
			    (uint64_t)(p->stencil.va != 0 &&
			    ((p->zls_control & (1u << 14)) != 0 ||
			    (p->zls_control & (1u << 18)) == 0)) << 19;
			break;
		}
	}
	c = *controls;
	c.ta.work_key = work_key(r->context_id, a->ta.state_word);
	c.fragment.work_key = work_key(r->context_id, a->fragment.state_word);
	CHECK(build_ta_registers(p, &c, r->free_list_slot, ws->base, cp->ta));
	j813 = c.board->j813_forms;
	/* TaExtra: buffer/target.rs m5_extra has no brackets. */
	CHECK(tiling_program(ws->base, 82, agx_scratch_enabled(&c.ta.scratch),
	    false, j813, ws->ta, &ws->ta_count));
	lifecycle = (uint64_t)in->ta.predecessor_id << 32 |
	    (uint32_t)in->ta.completion.key;
	for (i = 0; i < ws->ta_count; i++)
		if (ws->ta[i].reg == 0x1ca10 || ws->ta[i].reg == 0x14a1 ||
		    ws->ta[i].reg == 0xa349)
			ws->ta[i].value = lifecycle;
	c.fragment_tile_flags = (c.fragment_tile_flags & ~0x7ffffull) | 0x36011 |
	    (uint64_t)(in->fragment.empty_tile_flags[3] & 1) << 18;
	if (j813)
		c.fragment_event_control = 0;
	CHECK(build_fragment_registers(p, &c, r->free_list_slot, ws->base,
	    cp->fragment));
	ws->fragment_count = (j813 ? 90 : 92) +
	    agx_scratch_enabled(&c.fragment.scratch) * 2;
	CHECK(fragment_main(p, ws->base, 90,
	    agx_scratch_enabled(&c.fragment.scratch), j813, ws->fragment,
	    ws->fragment_count));
	lifecycle = (uint64_t)in->fragment.predecessor_id << 32 |
	    (uint32_t)in->fragment.completion.key;
	for (i = 0; i < 94; i++)
		if (ws->fragment[i].reg == 0x160e0 ||
		    ws->fragment[i].reg == 0x1499 ||
		    ws->fragment[i].reg == 0xa341)
			ws->fragment[i].value = lifecycle;
	CHECK(partial(cp->store, PARTIAL_STORE_REGISTERS, 0x15381, &c, false,
	    ws->store, 17));
	empty = (in->fragment.empty_tile_flags[1] & 1) != 0 ||
	    p->depth.va != 0 || p->stencil.va != 0;
	for (i = 0; i < 17; i++)
		if (ws->store[i].reg == 0x10039)
			ws->store[i].value = (ws->store[i].value &
			    ~(1ull << 16)) | empty << 16;
	CHECK(partial(cp->resume, PARTIAL_RESUME_REGISTERS, 0x15381, &c, false,
	    ws->resume, 24));
	c.store_pipeline_2_bind = cp->fragment[2].value << 32;
	c.store_pipeline_2 = cp->fragment[3].value;
	CHECK(partial(cp->load, PARTIAL_LOAD_REGISTERS, 0x15309, &c, true,
	    ws->load, 12));
	for (i = 0; i < 12; i++)
		if (ws->load[i].reg == 0x10039)
			ws->load[i].value = (ws->load[i].value &
			    ~(1ull << 16)) |
			    (uint64_t)(in->fragment.empty_tile_flags[2] & 1) << 16;
	return (0);
}

/* fw/m5/render.rs */
#define	TA_PROGRAM_OFFSET	0x60
#define	PROGRAM_AREA		0x700
#define	REGISTER_BYTES		12
#define	PROGRAM_CAPACITY	(PROGRAM_AREA / REGISTER_BYTES)

struct program_ref {
	const struct rw	*registers;
	int		count;
	uint16_t	rce_count;
	uint32_t	command_offset;
};

static int
check_program(const struct program_ref *p, uint32_t offset)
{

	if (p->command_offset != offset || p->count > PROGRAM_CAPACITY ||
	    p->rce_count > p->count || p->rce_count > 0x7f)
		return (EINVAL);
	return (0);
}

static int
place_program(uint8_t *out, uint64_t low, const struct program_ref *p,
    uint64_t *address)
{
	size_t off = p->command_offset, entry;
	uint32_t header;
	int i;

	*address = low + p->command_offset;
	put16(out, off + PROGRAM_AREA + 8, 0);
	put64(out, off + PROGRAM_AREA, *address);
	for (i = 0; i < p->count; i++) {
		entry = off + i * REGISTER_BYTES;
		header = (read32(out, entry) & 0xfffc0006) |
		    (p->registers[i].reg & 0x3fff9);
		put32(out, entry, header);
		put64(out, entry + 4, p->registers[i].value);
	}
	put16(out, off + PROGRAM_AREA + 8, p->count);
	return (0);
}

static void
completion_tail(uint8_t *out, size_t base, const struct stage_completion *c)
{

	put64(out, base, c->stamp_addresses[0]);
	put64(out, base + 8, c->stamp_addresses[1]);
	put32(out, base + 0x10, c->stamp);
	put32(out, base + 0x14, c->queue_index);
	put32(out, base + 0x18, c->ordinal);
	put32(out, base + 0x1c, c->has_wait);
}

/* render_tail.rs encode */
static void
render_tail(uint8_t *out, size_t base, uint64_t descriptor, bool has_resolved,
    uint64_t resolved, uint64_t trailing)
{

	put64(out, base, descriptor);
	if (has_resolved)
		put64(out, base + 8, resolved);
	put64(out, base + 16, trailing);
}

/* render_tail.rs encode_extra with no addresses */
static void
extra_tail(uint8_t *out, size_t base, bool fragment)
{
	int i;

	out[base + 32] &= 0xf2;
	if (fragment)
		for (i = 33; i < 37; i++)
			out[base + i] = 0;
}

static int
encode_ta(uint8_t out[AGX_M5_TA_COMMAND_BYTES], uint64_t low,
    const struct inputs *in, const struct program_ref *ta, uint64_t rce[4])
{
	const struct ta_fields *t = &in->ta;
	uint64_t dispatch = 0, address;
	uint8_t pass;
	bool found = false;
	int error, i;

	CHECK(check_program(ta, TA_PROGRAM_OFFSET));
	for (i = 0; i < ta->count && !found; i++)
		if ((ta->registers[i].reg & ~1u) == 0x1c898) {
			dispatch = ta->registers[i].value;
			found = true;
		}
	if (!found || t->shifted_usage_hint > (~0ull >> 8))
		return (EINVAL);
	put32(out, 0, 0);			/* M5_TA_SELECTOR */
	put64(out, 4, t->sequence);
	put32(out, 0xc, in->context_id);
	render_tail(out, 0x937, (t->tail_descriptor & 0xff) |
	    t->shifted_usage_hint << 8, true, t->resolved_tail_value,
	    t->context_token);
	put32(out, 0x8d6, 0);			/* ta_word_e */
	put32(out, 0x48, 0);			/* ta_word_b */
	put32(out, 0x18, t->pb_slot);
	put32(out, 0x1c, 0);			/* ta_word_a */
	put64(out, 0x20, t->pb_state);
	put64(out, 0x28, t->pm_scene);
	put64(out, 0x30, t->scene_end_va);
	put64(out, 0x10, in->scheduler_record);
	put64(out, 0x780, t->tpc_va);
	put64(out, 0x788, t->tpc_size);
	put32(out, 0x79c, t->linked_queue_index);
	put32(out, 0x7a0, t->linked_stamp);
	put32(out, 0x7a4, t->linked_flag & 1);
	put64(out, 0x7a8, t->linked_completion & 0xffffffffffull);
	pass = t->linked_completion >> 40;
	put8(out, 0x7b1, pass);
	put8(out, 0x7b0, pass);
	put64(out, 0x7b6, 0);			/* ta_word_c */
	put64(out, 0x7be, 0);			/* ta_word_d */
	put64(out, 0x7d6, t->deflake_state_va);
	put128(out, 0x7de, 0, 0);
	put64(out, 0x7ee, 0);
	put32(out, 0x7b2, 0);
	put32(out, 0x86a, 0);			/* stage_flag_a */
	put32(out, 0x86e, t->predecessor_id);
	put32(out, 0x872, 0);			/* stage_word_a */
	put32(out, 0x876, t->stage_word_b);
	put128(out, 0x87a, 0, 0);
	put128(out, 0x90e, 0, 0);		/* ta_pair_a */
	put32(out, 0x91e, 0);			/* timestamp_counts */
	put32(out, 0x922, 0);
	put64(out, 0x926, 0);			/* ta_word_f */
	put64(out, 0x92e, t->uma_state);
	put8(out, 0x936, t->tail_flag & 1);
	put64(out, 0x8fe, t->timestamp_va);
	put64(out, 0x906, t->diagnostic_ta_timestamp_upper);
	put32(out, 0x892, t->pb_growth_request & 1);
	put32(out, 0x89a, t->shader_write_class & 1);
	put32(out, 0x89e, 0);			/* m5_ta_policy_bit: no policy */
	put32(out, 0x8a2, 0);
	put64(out, 0x8f6, 0);
	completion_tail(out, 0x8a6, &t->completion);
	put64(out, 0x8c6, t->completion.completion);
	put64(out, 0x8ce, t->completion.key);
	put32(out, 0x38, 0);
	put16(out, 0x3c, 0);
	put32(out, 0x40, 0);
	CHECK(place_program(out, low, ta, &address));
	put16(out, 0x38, ta->rce_count);
	put16(out, 0x3a, ta->count);
	put16(out, 0x3c, ta->count);
	put64(out, 0x85e, dispatch);
	extra_tail(out, 0x96f, false);
	memset(rce, 0, 4 * sizeof(*rce));
	return (agx_register_array_binding(address, ta->rce_count, &rce[0]));
}

static int
encode_3d(uint8_t out[AGX_M5_3D_COMMAND_BYTES], uint64_t low,
    const struct inputs *in, const struct program_ref programs[4],
    uint16_t main_mid_count, uint64_t rce[4])
{
	static const uint32_t offsets[4] = { 0xa0, 0x7c0, 0xee0, 0x1600 };
	const struct fragment_fields *t = &in->fragment;
	uint64_t tile = 0, address;
	bool found = false;
	int error, i;

	for (i = 0; i < 4; i++)
		CHECK(check_program(&programs[i], offsets[i]));
	for (i = 1; i < 4; i++)
		if (programs[i].rce_count != programs[i].count)
			return (EINVAL);
	if (main_mid_count < programs[0].rce_count ||
	    main_mid_count > programs[0].count)
		return (EINVAL);
	for (i = 0; i < programs[0].count && !found; i++)
		if ((programs[0].registers[i].reg & ~1u) == 0x10038) {
			tile = programs[0].registers[i].value;
			found = true;
		}
	if (!found)
		return (EINVAL);
	put32(out, 0, 1);			/* M5_3D_SELECTOR */
	put64(out, 4, t->sequence);
	put32(out, 0xc, in->context_id);
	render_tail(out, 0x21d1, t->tail_descriptor, t->has_resolved,
	    t->resolved_tail_value, t->context_token);
	put32(out, 0x2170, 0);			/* fragment_word_l */
	put32(out, 0x2174, 0);			/* fragment_flag_a */
	put32(out, 0x2100, t->short_partial_programs & 1);
	put32(out, 0x20f8, 0);
	put128(out, 0x2178, 0, 0);
	put64(out, 0x2188, 0);
	put8(out, 0x21d0, t->tail_flag & 1);
	put64(out, 0x20, in->scheduler_record);
	put64(out, 0x28, t->pm_state);
	put64(out, 0x30, t->pm_scene);
	put64(out, 0x38, t->scene_end_va);
	put32(out, 0x58, 0);			/* fragment_word_a */
	put64(out, 0x40, t->tilemap_va);
	put64(out, 0x48, t->msaa_control);
	put32(out, 0x50, t->sample_count);
	put32(out, 0x54, t->utile_grid);
	put128(out, 0x60, t->merge_bounds[0], t->merge_bounds[1]);
	put64(out, 0x70, 0);			/* fragment_word_b */
	put32(out, 0x78, t->tile_count);
	put64(out, 0x1d30, t->scissor_va);
	put64(out, 0x1d20, t->depth_bias_va);
	put64(out, 0x1d40, t->occlusion_query_va);
	for (i = 0; i < 23; i++)
		put128(out, 0x1d58 + i * 16, t->binding_pairs[i][0],
		    t->binding_pairs[i][1]);
	put64(out, 0x1ec8, t->depth_va);
	put64(out, 0x1ed0, t->depth_stride);
	put64(out, 0x1ed8, t->depth_compression_stride);
	put64(out, 0x1f28, 0);			/* fragment_word_d */
	put128(out, 0x1ee0, t->depth_va_pair[0], t->depth_va_pair[1]);
	put64(out, 0x1ef0, t->depth_compression_va);
	put64(out, 0x1ef8, t->stencil_va);
	put64(out, 0x1f00, t->stencil_stride);
	put64(out, 0x1f08, t->stencil_compression_stride);
	put64(out, 0x1f30, 0);			/* fragment_word_e */
	put128(out, 0x1f10, t->stencil_va_pair[0], t->stencil_va_pair[1]);
	put64(out, 0x1f20, t->stencil_compression_va);
	put64(out, 0x1f68, 0);			/* fragment_word_f */
	put32(out, 0x1f70, 0);			/* fragment_word_g */
	put128(out, 0x1f48, t->size_aux_control[0], t->size_aux_control[1]);
	put128(out, 0x1f74, t->eot_program[0], t->eot_program[1]);
	put64(out, 0x1f88, 0);			/* fragment_word_h */
	put32(out, 0x1f90, 0);			/* fragment_word_i */
	put128(out, 0x1f94, t->partial_eot_program[0],
	    t->partial_eot_program[1]);
	put64(out, 0x1fa8, t->zs_clear_values);
	put32(out, 0x1fb0, t->sample_size);
	put128(out, 0x1f38, t->sample_aux_flags[0], t->sample_aux_flags[1]);
	put64(out, 0x1d50, 0);			/* fragment_word_c */
	put64(out, 0x1fc0, 0);			/* fragment_word_j */
	put32(out, 0x1fc8, 0);			/* fragment_word_k */
	put64(out, 0x1fd0, t->depth_dimensions);
	memset(out + 0x1fdc, 0, 0x100);
	put32(out, 0x20dc, 0);
	put128(out, 0x20e0, 0, 0);
	put32(out, 0x1fd8, 0);
	put32(out, 0x20f0, 0);
	put8(out, 0x90, t->pass);
	put32(out, 0x20fc, t->fragment_pass_flag & 1);
	put32(out, 0x2209, t->fragment_word_n);
	put8(out, 0x220d, t->fragment_flag_c);
	put32(out, 0x2104, 0);			/* stage_flag_a */
	put32(out, 0x2108, t->predecessor_id);
	put32(out, 0x210c, 0);			/* stage_word_a */
	put32(out, 0x2110, t->stage_word_b);
	put128(out, 0x2114, 0, 0);
	put128(out, 0x21a8, 0, 0);		/* fragment_pair_a */
	put32(out, 0x21b8, 0);			/* timestamp_counts */
	put32(out, 0x21bc, 0);
	put64(out, 0x21c0, 0);			/* fragment_word_m */
	put64(out, 0x21c8, t->uma_state);
	put128(out, 0x2198, t->timestamp_va[0], t->timestamp_va[1]);
	completion_tail(out, 0x2140, &t->completion);
	put64(out, 0x2160, t->completion.completion);
	put64(out, 0x2190, 0);
	for (i = 0; i < 4; i++)
		put32(out, 0x2124 + i * 4, t->empty_tile_flags[i] & 1);
	put32(out, 0x2138, 0);			/* fragment_flag_b */
	put64(out, 0x2168, t->completion.key);
	extra_tail(out, 0x220e, true);
	put32(out, 0x80, 0);
	put16(out, 0x84, 0);
	put32(out, 0x88, 0);
	for (i = 0; i < 4; i++) {
		CHECK(place_program(out, low, &programs[i], &address));
		CHECK(agx_register_array_binding(address,
		    programs[i].rce_count, &rce[i]));
	}
	put64(out, 0x1f58, tile);
	put64(out, 0x1f60, 0);
	put16(out, 0x80, programs[0].rce_count);
	put16(out, 0x82, main_mid_count);
	put16(out, 0x84, main_mid_count);
	put16(out, 0x88, programs[0].count);
	return (0);
}

/* render_tail.rs common26_layout */
static void
common26_layout(uint8_t *out, size_t start, size_t end)
{

	memmove(out + start + 6, out + start, end - start);
	memset(out + start, 0, 6);
}

/* render_assembly.rs encode_into, from memory.rs write_m5_commands. */
int
agx_m5_render_encode(uint8_t storage[AGX_M5_RENDER_STORAGE_BYTES],
    uint64_t low, const struct agx_render_args *a,
    struct agx_m5_render_encoded *out)
{
	static struct workspace ws;	/* callers serialize encoding */
	static uint8_t ta[AGX_M5_TA_COMMAND_BYTES];
	static uint8_t fragment[AGX_M5_3D_COMMAND_BYTES];
	struct controls c;
	struct inputs in;
	struct program_ref tp, fp[4];
	int error;

	if (!a->board->j813_forms)
		return (EOPNOTSUPP);
	CHECK(controls_new(a, &c));
	CHECK(inputs_new(a, &in));
	CHECK(apply_pass(a, &c, &in));
	if (in.context_id != a->resources->context_id)
		return (EINVAL);
	memset(&ws, 0, sizeof(ws));
	CHECK(workspace_write(&ws, a, &c, &in));
	tp = (struct program_ref){ ws.ta, ws.ta_count, ws.ta_count - 3,
	    TA_PROGRAM_OFFSET };
	fp[0] = (struct program_ref){ ws.fragment, ws.fragment_count,
	    ws.fragment_count - 4, 0xa0 };
	fp[1] = (struct program_ref){ ws.store, 17, 17, 0x7c0 };
	fp[2] = (struct program_ref){ ws.resume, 24, 24, 0xee0 };
	fp[3] = (struct program_ref){ ws.load, 12, 12, 0x1600 };
	memset(ta, 0, sizeof(ta));
	memset(fragment, 0, sizeof(fragment));
	memset(out, 0, sizeof(*out));
	CHECK(encode_3d(fragment, low + AGX_M5_RENDER_FRAGMENT_OFFSET, &in, fp,
	    ws.fragment_count - 2, out->rce[1]));
	CHECK(encode_ta(ta, low, &in, &tp, out->rce[0]));
	if (a->board->common26) {
		common26_layout(ta, 0x92e, 0x990);
		common26_layout(fragment, 0x21c8, 0x2233);
	}
	memcpy(storage, ta, sizeof(ta));
	memcpy(storage + AGX_M5_RENDER_FRAGMENT_OFFSET, fragment,
	    sizeof(fragment));
	out->register_counts[0] = ws.ta_count;
	out->register_counts[1] = ws.fragment_count;
	out->register_counts[2] = 17;
	out->register_counts[3] = 24;
	out->register_counts[4] = 12;
	out->scratch[0] = agx_scratch_enabled(&c.ta.scratch);
	out->scratch[1] = agx_scratch_enabled(&c.fragment.scratch);
	return (0);
}

/* render.rs link_completions; KickTimestamp::completion_value */
void
agx_m5_render_link_completions(uint8_t storage[AGX_M5_RENDER_STORAGE_BYTES],
    const uint64_t kicks[2], uint8_t pass, bool fragment_busy)
{
	uint8_t *ta = storage, *fragment = storage +
	    AGX_M5_RENDER_FRAGMENT_OFFSET;

	put32(ta, 0x7a4, fragment_busy);
	put64(ta, 0x7a8, (kicks[1] | (uint64_t)pass << 40) & 0xffffffffffull);
	put8(ta, 0x7b0, pass);
	put8(ta, 0x7b1, pass);
	put64(ta, 0x8c6, kicks[0] | (uint64_t)pass << 40);
	put8(fragment, 0x90, pass);
	put64(fragment, 0x2160, kicks[1] | (uint64_t)pass << 40);
}

/* render.rs bind_work_slot */
int
agx_m5_render_bind_work_slot(uint8_t storage[AGX_M5_RENDER_STORAGE_BYTES],
    uint32_t slot)
{

	if (slot >= 4)
		return (EINVAL);
	put32(storage, 0x8be, slot);
	put32(storage + AGX_M5_RENDER_FRAGMENT_OFFSET, 0x2158, slot);
	return (0);
}

/*
 * fw/m5/render_demand.rs, the parts render_resources.rs M5Policy::plan uses
 * for vertex and fragment programs: RenderDemand::new, include_vertex,
 * include_fragment and resource_flags. Mesh and compute demand is not
 * ported (this driver refuses mesh programs). Overflow is EINVAL, where Rust
 * returns PoolError::Geometry.
 */
struct demand {
	uint32_t	counts[5];
	uint32_t	minimum_pages, ideal_pages, pressure;
	uint64_t	clusters, maximum, total, floor;
	uint64_t	limited_maximum, limited_total;
	bool		included;
};

struct fragment_params {
	uint32_t	pressure, page_units, large_units, auxiliary_units;
	uint32_t	primary_large, secondary_large;
};

struct tile_params {
	uint32_t	pressure, page_units, large_units;
	uint32_t	threadgroup_large, imageblock_large, multisample_large;
};

static uint64_t
pages_of(uint64_t value, uint64_t size)
{

	return ((value + size - 1) / size);
}

static uint64_t
sat_sub(uint64_t a, uint64_t b)
{

	return (a > b ? a - b : 0);
}

static int
fragment_params(uint32_t primary_bytes, uint32_t secondary_bytes,
    uint32_t linked_bytes, uint32_t minimum_primary_bytes,
    uint32_t auxiliary_count, struct fragment_params *f)
{
	uint64_t primary, secondary, small, large, auxiliary, reserved, pressure;
	uint64_t pl, sl;

	primary = (uint64_t)primary_bytes + linked_bytes;
	if (primary < minimum_primary_bytes)
		primary = minimum_primary_bytes;
	primary = primary * 32 + 0x4380;
	secondary = (uint64_t)secondary_bytes * 32;
	pl = pages_of(primary, 0x10000);
	sl = pages_of(secondary, 0x10000);
	small = pages_of(primary, 0x1000) + pages_of(secondary, 0x1000);
	large = pl + sl;
	auxiliary = (uint64_t)auxiliary_count * 128;
	if (auxiliary == 0)
		reserved = 0;
	else {
		reserved = (auxiliary > 0x4000 ? auxiliary : 0x4000) | 64;
		auxiliary |= 64;
	}
	pressure = small * 96 + large * 10 + pages_of(auxiliary, 0x1000) * 64 +
	    pages_of(auxiliary, 0x10000) * 5;
	if (pressure > UINT32_MAX)
		return (EINVAL);
	f->pressure = pressure;
	f->page_units = small + large;
	f->large_units = large;
	f->auxiliary_units = pages_of(reserved, 0x1000) +
	    pages_of(reserved, 0x10000);
	f->primary_large = pl;
	f->secondary_large = sl;
	return (0);
}

static void
tile_params(uint32_t threadgroup, uint32_t imageblock, uint32_t multisample,
    struct tile_params *t)
{
	uint64_t sizes[3] = { threadgroup, imageblock == 0 ? 0 :
	    (uint64_t)imageblock + 64, multisample };
	uint64_t large[3], small = 0, total = 0;
	int i;

	for (i = 0; i < 3; i++) {
		large[i] = pages_of(sizes[i], 0x10000);
		small += pages_of(sizes[i], 0x1000);
		total += large[i];
	}
	/* preempt_formula false: factor 6 */
	t->pressure = small * 6 + total * 6;
	t->page_units = small + total;
	t->large_units = total;
	t->threadgroup_large = large[0];
	t->imageblock_large = large[1];
	t->multisample_large = large[2];
}

static int
demand_flags(const uint32_t c[5], uint64_t *flags)
{

	if (c[0] > 0xff || c[1] > 0xf || c[2] > 3 || c[3] > 0xff || c[4] > 0xff)
		return (EINVAL);
	*flags = 0x14b | (uint64_t)c[0] << 10 | (uint64_t)c[1] << 18 |
	    (uint64_t)c[2] << 22 | (uint64_t)c[3] << 24 | (uint64_t)c[4] << 32;
	return (0);
}

static int
demand_merge(struct demand *d, const uint32_t parts[5], uint64_t minimum,
    uint64_t ideal, uint64_t pressure)
{
	struct demand next = *d;
	uint64_t flags, m, i;
	int k;

	m = (minimum * d->clusters + 7) & ~7ull;
	i = (ideal * d->clusters + 7) & ~7ull;
	if (m > INT32_MAX || i > INT32_MAX || pressure > UINT32_MAX)
		return (EINVAL);
	if (next.minimum_pages < m)
		next.minimum_pages = m;
	if (next.ideal_pages < i)
		next.ideal_pages = i;
	if (next.pressure < pressure)
		next.pressure = pressure;
	for (k = 0; k < 5; k++)
		if (next.counts[k] < parts[k])
			next.counts[k] = parts[k];
	if (demand_flags(next.counts, &flags) != 0)
		return (EINVAL);
	next.included = true;
	*d = next;
	return (0);
}

static int
demand_new(const uint32_t *masks, unsigned clusters, uint32_t nominal,
    uint32_t ptc_floor, struct demand *d)
{
	uint64_t limit, maximum = 0, total = 0, n;
	unsigned i;

	for (i = 0; i < clusters; i++) {
		n = __builtin_popcount(masks[i]);
		total += n;
		if (n > maximum)
			maximum = n;
	}
	if (clusters == 0 || total == 0 || ptc_floor == 0 || nominal < total)
		return (EINVAL);
	memset(d, 0, sizeof(*d));
	limit = (uint64_t)nominal * 2 / clusters;
	d->clusters = clusters;
	d->maximum = maximum;
	d->total = total;
	d->floor = ptc_floor;
	d->limited_maximum = maximum < limit ? maximum : limit;
	d->limited_total = total < limit * clusters ? total : limit * clusters;
	return (0);
}

static int
demand_fragment(struct demand *d, const struct fragment_params *f,
    const struct tile_params *t)
{
	uint64_t base, pressure, large, tails, ideal, minimum;
	uint32_t parts[5] = { f->primary_large, f->secondary_large,
	    t->threadgroup_large, t->imageblock_large, t->multisample_large };

	base = d->maximum * (d->floor + 11) + 58;
	pressure = (uint64_t)f->pressure + t->pressure;
	large = (uint64_t)f->large_units + f->auxiliary_units + t->large_units;
	tails = (uint64_t)parts[0] + parts[1] + parts[2] + parts[3];
	ideal = base + t->multisample_large + tails +
	    sat_sub(pressure, d->floor) * d->maximum + 64;
	minimum = base + t->multisample_large + tails +
	    (sat_sub((uint64_t)f->page_units + t->page_units, d->floor) +
	    t->page_units) * d->maximum + large * d->total + 64;
	return (demand_merge(d, parts, minimum, ideal, pressure));
}

static int
demand_vertex(struct demand *d, uint32_t primary, uint32_t secondary,
    uint32_t linked, uint32_t minimum_primary)
{
	struct fragment_params p;
	uint64_t large, base, minimum, ideal;
	uint32_t parts[5] = { 0 };
	int error;

	if ((error = fragment_params(primary, secondary, linked,
	    minimum_primary, 0, &p)) != 0)
		return (error);
	large = p.large_units;
	base = d->limited_maximum * (d->floor + 11) + 58;
	minimum = base + large + 64 + sat_sub(p.page_units, d->floor) *
	    d->limited_maximum + large * d->limited_total;
	ideal = base + large + 64 + sat_sub(p.pressure, d->floor) *
	    d->limited_maximum;
	parts[0] = p.primary_large;
	parts[1] = p.secondary_large;
	return (demand_merge(d, parts, minimum, ideal, p.pressure));
}

/*
 * render_resources.rs Programs::demands and the stage plans of
 * M5Policy::plan (CONTEXT_FLOOR 96; priority 0 counts three resident kicks,
 * the others four).
 */
int
agx_m5_render_plan(const uint32_t *core_masks, unsigned clusters,
    uint32_t nominal_cores, uint8_t priority,
    const struct agx_render_program *programs, unsigned n,
    struct agx_stage_plan stages[2])
{
	struct demand base, stage[2];
	struct fragment_params empty;
	struct tile_params no_tile, tile;
	struct fragment_params f;
	uint32_t active, pressure;
	unsigned i;
	int error;

	if (priority > 3 || n == 0)
		return (EINVAL);
	active = priority == 0 ? 3 : 4;
	if ((error = demand_new(core_masks, clusters, nominal_cores, 96,
	    &base)) != 0)
		return (error);
	stage[0] = stage[1] = base;
	if ((error = demand_vertex(&stage[0], 0, 0, 0, 0)) != 0 ||
	    (error = fragment_params(0, 0, 0, 0, 0, &empty)) != 0)
		return (error);
	tile_params(0, 0, 0, &no_tile);
	if ((error = demand_fragment(&stage[1], &empty, &no_tile)) != 0)
		return (error);
	for (i = 0; i < n; i++) {
		const struct agx_render_program *e = &programs[i];

		if (e->stage > 1 || (e->stage == 0 && (e->auxiliary_count != 0 ||
		    e->threadgroup_bytes != 0 || e->imageblock_bytes != 0 ||
		    e->multisample_bytes != 0)))
			return (EINVAL);
		if (e->stage == 0) {
			error = demand_vertex(&stage[0], e->primary_bytes,
			    e->secondary_bytes, e->linked_bytes,
			    e->minimum_primary_bytes);
		} else {
			error = fragment_params(e->primary_bytes,
			    e->secondary_bytes, e->linked_bytes,
			    e->minimum_primary_bytes, e->auxiliary_count, &f);
			tile_params(e->threadgroup_bytes, e->imageblock_bytes,
			    e->multisample_bytes, &tile);
			if (error == 0)
				error = demand_fragment(&stage[1], &f, &tile);
		}
		if (error != 0)
			return (error);
	}
	memset(stages, 0, 2 * sizeof(*stages));
	for (i = 0; i < 2; i++) {
		if ((error = demand_flags(stage[i].counts,
		    &stages[i].resource_flags)) != 0)
			return (error);
		pressure = stage[i].pressure / active;
		stages[i].ptc_max = pressure < 96 ? 96 : pressure > 512 ? 512 :
		    pressure;
	}
	return (0);
}
