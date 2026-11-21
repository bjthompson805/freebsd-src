/*-
 * SPDX-License-Identifier: GPL-2.0-only OR MIT
 *
 * Apple M5 GPU: firmware record encoders for compute kicks, translated from
 * Aurora's drm/asahi g17/fw (m5/compute.rs, kick.rs, queue.rs, channels.rs).
 * Every record is written into a zeroed buffer at the offsets of the Rust
 * packed layouts, little endian. Errors are EINVAL where Rust returns it.
 */

#ifdef _KERNEL
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/errno.h>
#else
#include <errno.h>
#include <string.h>
#endif

#include "apple_agx_fw.h"

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

/* fw/mod.rs offset_va */
static int
offset_va(uint64_t va, uint64_t off, uint64_t *out)
{

	if (va + off < va)
		return (EINVAL);
	*out = va + off;
	return (0);
}

/* fw/mod.rs work_key */
static uint64_t
work_key(uint16_t context_id, uint32_t state_word)
{

	return ((uint64_t)context_id << 8 | (state_word & 0x3f));
}

/* fw/m5/compute.rs */

#define	MAIN_COUNT		40
#define	ALTERNATE_COUNT		4
#define	PREEMPT_BYTES		0x4000
static const uint16_t checkpoints[4] = { 37, 39, 39, 40 };

int
agx_scratch_new(uint64_t layout, struct agx_scratch *s)
{
	uint64_t cliques, depth, levels, user, driver, threads, driver_end;
	uint64_t bytes, pages;

	memset(s, 0, sizeof(*s));
	cliques = (layout >> 24) & 0xff;
	if (cliques == 0)
		return (0);
	depth = layout & 0xffff;
	levels = (layout >> 16) & 0xff;
	user = (layout >> 32) & 0xffff;
	driver = layout >> 48;
	threads = cliques * 32;
	driver_end = 0x68000 + threads * (depth * 8 + driver);
	if (driver_end > 0xffffff)
		return (EINVAL);
	bytes = driver_end + threads * user +
	    cliques * 256 * (levels + (levels > 1));
	pages = (bytes + 0x1fff) >> 12;
	if (pages > UINT16_MAX)
		return (EINVAL);
	s->layout = layout;
	s->driver_end = driver_end;
	s->pages = pages;
	return (0);
}

static bool
scratch_enabled(const struct agx_scratch *s)
{

	return (s->pages != 0);
}

static uint64_t
scratch_size_word(const struct agx_scratch *s)
{
	uint64_t pages = s->pages;

	return (pages | (((pages + 255) / 256) << 16));
}

static uint64_t
scratch_window_word(const struct agx_scratch *s)
{

	return (scratch_enabled(s) ?
	    (1u << 24) | (((s->layout >> 24) & 0xff) << 16) : 0xff0000);
}

static uint64_t
scratch_stack_word(const struct agx_scratch *s)
{

	return (scratch_enabled(s) ? s->layout & 0xfff8 : 8);
}

static uint64_t
scratch_driver_word(const struct agx_scratch *s)
{
	uint64_t levels;

	if (!scratch_enabled(s))
		return (0);
	levels = (s->layout >> 16) & 0xff;
	return (s->driver_end | ((levels + (levels > 1)) << 24));
}

static uint16_t
scratch_checkpoint(const struct agx_scratch *s, int i)
{

	return (checkpoints[i] + (scratch_enabled(s) ? 2 : 0));
}

int
agx_intersection_new(uint64_t pool, uint64_t pool_bytes, uint32_t first,
    uint32_t count, uint64_t default_program, uint64_t default_control,
    struct agx_intersection *t)
{
	uint64_t end, blocks_end, code;

	memset(t, 0, sizeof(*t));
	if (pool == 0) {
		if (pool_bytes != 0 || first != 0 || count != 0 ||
		    default_program != 0 || default_control != 0)
			return (EINVAL);
		return (0);
	}
	end = pool + pool_bytes;
	if (end < pool)
		return (EINVAL);
	blocks_end = (uint64_t)first + count;
	code = default_program & 0x0000ffffffffffffull;
	if ((pool & 0x3fff) != 0 || pool_bytes == 0 ||
	    (pool_bytes & 0x3fff) != 0 || end > (1ull << 48) ||
	    (pool_bytes >> 14) >= (1ull << 25) || count == 0 ||
	    count >= (1u << 18) || blocks_end * 128 > pool_bytes ||
	    (code & 63) != 0 ||
	    (default_control & ~0x00007fff00003fffull) != 0 ||
	    (code == 0 && (default_program != 0 || default_control != 0)) ||
	    (code != 0 && (default_control & (1ull << 40)) == 0))
		return (EINVAL);
	t->pool = pool;
	t->pool_bytes = pool_bytes;
	t->first = first;
	t->count = count;
	t->default_program = default_program;
	t->default_control = default_control;
	return (0);
}

static void
intersection_words(const struct agx_intersection *t, uint64_t w[6])
{

	if (t->pool == 0) {
		memset(w, 0, 6 * sizeof(*w));
		return;
	}
	w[0] = (t->pool + (uint64_t)t->first * 128) | (1ull << 51);
	w[1] = 0xc0000000ull | ((uint64_t)t->count << 7);
	w[2] = t->pool;
	w[3] = t->pool_bytes >> 14;
	w[4] = t->default_program;
	w[5] = t->default_control;
}

uint64_t
agx_m5_dispatch_control(const struct agx_board *b, uint64_t base)
{

	return (b->clusters > 4 ? base | 0x9c : base);
}

static bool
counted_forms(const struct agx_board *b)
{

	return (b->g17d_forms || b->completion_inputs || b->j813_forms);
}

uint8_t
agx_m5_explicit_dependency(const struct agx_board *b)
{

	return (b->completion_inputs || b->j813_forms ?
	    AGX_COMPLETION_DEPENDENCY : AGX_COMPUTE_PIPELINED_DEPENDENCY);
}

uint8_t
agx_m5_parent_dependency(const struct agx_board *b, bool parent_retired)
{

	if (!counted_forms(b) || (parent_retired && b->completion_inputs))
		return (AGX_COMPUTE_PIPELINED_DEPENDENCY);
	return (AGX_COMPLETION_DEPENDENCY);
}

uint32_t
agx_m5_completion_ordinal(const struct agx_board *b, uint32_t published_before)
{

	return (counted_forms(b) ? published_before : 0);
}

void
agx_m5_completion_forms(const struct agx_board *b, uint64_t *identity,
    uint8_t metadata[16])
{
	static const uint8_t j813[16] = {
		0, 0, 0x55, 0x01, 0, 0, 0, 0, 0, 0, 0x85, 0, 0, 0, 0, 0,
	};

	if (b->j813_forms) {
		*identity = 0x0400000000000002ull;
		memcpy(metadata, j813, 16);
	} else {
		*identity = 1;
		memset(metadata, 0, 16);
	}
}

#define	POOL_EVENT_CONTEXTS	64
#define	POOL_EVENT_ENGINE	2
#define	ENTRY_GATE_EVENT	254

int
agx_m5_kick_event_mask(const struct agx_board *b, uint32_t context_id,
    uint64_t mask[4])
{
	uint32_t pool;

	memset(mask, 0, 4 * sizeof(*mask));
	if (b->j813_forms) {
		if (context_id >= POOL_EVENT_CONTEXTS)
			return (EINVAL);
		mask[1] = 1ull << context_id;
		return (0);
	}
	if (!counted_forms(b)) {
		mask[2] = 8;
		return (0);
	}
	if (context_id >= POOL_EVENT_CONTEXTS)
		return (EINVAL);
	pool = POOL_EVENT_ENGINE * POOL_EVENT_CONTEXTS + context_id;
	mask[pool / 64] |= 1ull << (pool % 64);
	if (b->g17d_forms)
		mask[ENTRY_GATE_EVENT / 64] |= 1ull << (ENTRY_GATE_EVENT % 64);
	return (0);
}

void
agx_m5_kick_tuple(const struct agx_board *b, uint8_t parent_class,
    struct agx_kick_tuple *t)
{

	if (b->g17d_forms) {			/* QUALIFIED_TUPLE */
		t->preempt_state_offset = 0x1480;
		t->dispatch_base = 0x10c08860;
		t->auxiliary_control = 0;
	} else if (b->j813_forms) {		/* J813_TUPLE */
		t->preempt_state_offset = 0x1480;
		t->dispatch_base = 0x14c08a60;
		t->auxiliary_control = 0;
	} else if (b->completion_inputs) {
		t->preempt_state_offset = 0x1b80;
		t->dispatch_base = 0x10c08860;
		t->auxiliary_control =
		    parent_class == AGX_COMPUTE_PIPELINED_DEPENDENCY;
	} else {				/* NORMAL_TUPLE */
		t->preempt_state_offset = 0x1b80;
		t->dispatch_base = 0x14c08a60;
		t->auxiliary_control = 1;
	}
}

/* fw/kick.rs packed_va and RegisterArrayBinding::new */
#define	PACKED_VA_SHIFT		5
#define	PACKED_VA_LIMIT		(1ull << (38 + PACKED_VA_SHIFT))

static int
packed_va(uint64_t va, uint64_t *out)
{

	if ((va & ((1u << PACKED_VA_SHIFT) - 1)) != 0 || va >= PACKED_VA_LIMIT)
		return (EINVAL);
	*out = va >> PACKED_VA_SHIFT;
	return (0);
}

static int
register_array_binding(uint64_t array_va, unsigned checkpoint, uint64_t *word)
{
	uint64_t packed;
	int error;

	if (checkpoint > 0x7f)
		return (EINVAL);
	if ((error = packed_va(array_va, &packed)) != 0)
		return (error);
	*word = packed | (uint64_t)checkpoint << 43;
	return (0);
}

int
agx_m5_bindings(uint64_t descriptor_va, const struct agx_scratch *s,
    bool matrix_ops, uint64_t words[4])
{
	uint64_t va;
	int error;

	words[2] = words[3] = 0;
	if ((error = offset_va(descriptor_va, 0x40, &va)) != 0 ||
	    (error = register_array_binding(va,
	    (uint8_t)(scratch_checkpoint(s, 0) + (matrix_ops ? 2 : 0)),
	    &words[0])) != 0 ||
	    (error = offset_va(descriptor_va, 0x760, &va)) != 0 ||
	    (error = register_array_binding(va, ALTERNATE_COUNT,
	    &words[1])) != 0)
		return (error);
	return (0);
}

struct regw {
	uint32_t	reg;
	uint64_t	value;
};

static int
validate(const struct agx_m5_compute_args *a)
{
	uint64_t va;

	if ((a->preempt_va & 0x3fff) != 0 || (a->operand_va & 0x3fff) != 0 ||
	    (a->cdm_va & 7) != 0 || (a->usage_va & 7) != 0 ||
	    a->context_id >= 64)
		return (EINVAL);
	if (offset_va(a->preempt_va, PREEMPT_BYTES, &va) != 0 ||
	    offset_va(a->operand_va, a->operand_bytes, &va) != 0)
		return (EINVAL);
	if ((a->preempt_state_offset & 7) != 0 ||
	    (uint64_t)a->preempt_state_offset + 32 > PREEMPT_BYTES ||
	    a->operand_region_bytes == 0 ||
	    (a->operand_region_bytes & 0xfff) != 0 ||
	    (uint64_t)a->operand_region_bytes * 2 + 0x808 > a->operand_bytes ||
	    (a->operand_aux_offset & 7) != 0 ||
	    (uint64_t)a->operand_aux_offset + 8 > a->operand_bytes)
		return (EINVAL);
	return (0);
}

/* main(): the 40 writes of the primary program. */
static int
main_program(const struct agx_m5_compute_args *a, struct regw w[MAIN_COUNT])
{
	uint64_t is[6], preempt = a->preempt_va, operand = a->operand_va;
	uint64_t ids = a->object_ids, state = a->preempt_state_offset;
	uint64_t region = a->operand_region_bytes;
	uint64_t key = work_key(a->context_id, a->work_state);
	int error, n = 0;

#define	W(r, v)		do { w[n].reg = (r); w[n].value = (v); n++; } while (0)
#define	WO(r, base, off) do {						\
	uint64_t _va;							\
	if ((error = offset_va(base, off, &_va)) != 0)			\
		return (error);						\
	W(r, _va);							\
} while (0)
	if ((error = validate(a)) != 0)
		return (error);
	intersection_words(&a->intersection, is);
	W(0x1a510, preempt);
	W(0x1a420, a->cdm_va);
	WO(0x1a4d0, preempt, state);
	WO(0x1a4d8, preempt, state + 8);
	WO(0x1a4e0, preempt, state + 16);
	WO(0x1a4e8, preempt, state + 24);
	W(0x1a440, 0x154024201ull);
	W(0x0d4a1, a->resource_control);
	W(0x0d411, 0x200000001ull);
	W(0x0a599, a->resource_policy);
	W(0x1a458, a->dispatch_control);
	W(0x101d9, 0x1c);
	W(0x1a089, is[0]);
	W(0x1a091, is[1]);
	W(0x1a059, is[2]);
	W(0x1a061, is[3]);
	W(0x1a0b9, is[4]);
	W(0x1a0c1, is[5]);
	W(0x101d1, scratch_size_word(&a->scratch));
	W(0x0d479, a->auxiliary_allocation);
	W(0x1a0e9, scratch_stack_word(&a->scratch));
	W(0x1a141, scratch_driver_word(&a->scratch));
	W(0x107a1, scratch_window_word(&a->scratch) |
	    ((uint64_t)a->matrix_ops << 25));
	W(0x1a540, ids);
	W(0x014a9, ids);
	W(0x0a351, ids);
	W(0x0a731, a->auxiliary_control);
	W(0x10201, key);
	W(0x10428, key);
	W(0x14028, a->free_list_slot);
	W(0x14070, a->usage_va | 1);
	WO(0x10229, operand, region * 2 + 0x800);
	WO(0x140a8, operand, a->operand_aux_offset);
	WO(0x10099, operand, region + 0x405);
	WO(0x10091, operand, region * 2 + 0x400);
	WO(0x0a5c1, operand, 5);
	WO(0x0a5c9, operand, region);
	W(0x0a731, 0);
	W(0x1a440, 0x154024209ull);
	W(0x0a599, 0x6000400020ull);
	return (n == MAIN_COUNT ? 0 : EINVAL);
}

/* program(): main with the scratch and matrix brackets. */
static int
program(const struct agx_m5_compute_args *a, struct regw w[MAIN_COUNT + 4],
    int *count)
{
	struct regw m[MAIN_COUNT];
	bool scs = scratch_enabled(&a->scratch);
	int error, i, n = 0;

	if ((error = main_program(a, m)) != 0)
		return (error);
	if (scs)
		w[n++] = (struct regw){ 0x17e1, 1 };
	if (a->matrix_ops)
		w[n++] = (struct regw){ 0x17f1, 1 };
	for (i = 0; i < MAIN_COUNT; i++) {
		if (i == 33) {
			if (scs)
				w[n++] = (struct regw){ 0x17d9, 1 };
			if (a->matrix_ops)
				w[n++] = (struct regw){ 0x17e9, 1 };
		}
		w[n++] = m[i];
	}
	*count = n;
	return (0);
}

static int
alternate_program(const struct agx_m5_compute_args *a,
    struct regw w[ALTERNATE_COUNT])
{
	uint64_t operand = a->operand_va, region = a->operand_region_bytes;
	int error, n = 0;

	if ((error = validate(a)) != 0)
		return (error);
	WO(0x10099, operand, 5);
	WO(0x10091, operand, region);
	WO(0x0a5c1, operand, region + 0x405);
	WO(0x0a5c9, operand, region * 2 + 0x400);
	return (0);
#undef W
#undef WO
}

/* fw/kick.rs RegisterArray::set, at offset off of the descriptor. */
#define	REGISTER_ARRAY_ENTRIES	128
#define	REGISTER_WRITE_SIZE	12

static int
register_array_set(uint8_t *p, size_t off, uint64_t self_va,
    const struct regw *w, int count)
{
	int i;

	if (count > REGISTER_ARRAY_ENTRIES)
		return (EINVAL);
	for (i = 0; i < count; i++) {
		put32(p, off + i * REGISTER_WRITE_SIZE, w[i].reg);
		put64(p, off + i * REGISTER_WRITE_SIZE + 4, w[i].value);
	}
	put64(p, off + 0x700, self_va);
	/* m5/mod.rs set_register_array: the size word is the count alone. */
	put32(p, off + 0x708, count);
	return (0);
}

#define	COMMON26_TAIL_INSERT	6

/* Descriptor::write: the full prefix, into a zeroed buffer. */
int
agx_m5_compute_descriptor(uint8_t out[AGX_M5_DESCRIPTOR_SIZE],
    const struct agx_m5_compute_args *a)
{
	struct regw prog[MAIN_COUNT + 4], alt[ALTERNATE_COUNT];
	uint64_t main_va, alternate_va, bind[4];
	size_t at;
	int count, error, i;

	if (a->cdm_end_va < a->cdm_va || a->qid >= 128)
		return (EINVAL);
	if ((error = program(a, prog, &count)) != 0 ||
	    (error = alternate_program(a, alt)) != 0 ||
	    (error = offset_va(a->descriptor_va, 0x40, &main_va)) != 0 ||
	    (error = offset_va(a->descriptor_va, 0x760, &alternate_va)) != 0 ||
	    (error = agx_m5_bindings(a->descriptor_va, &a->scratch,
	    a->matrix_ops, bind)) != 0)
		return (error);
	memset(out, 0, AGX_M5_DESCRIPTOR_SIZE);
	put32(out, 0x0, 3);
	put64(out, 0x4, a->submit_sequence);
	put32(out, 0xc, a->context_id);
	put64(out, 0x10, a->work_state_va);
	for (i = 0; i < 4; i++)
		put16(out, 0x18 + 2 * i, scratch_checkpoint(&a->scratch, i) +
		    (a->matrix_ops ? 2 : 0));
	if ((error = register_array_set(out, 0x40, main_va, prog, count)) != 0 ||
	    (error = register_array_set(out, 0x760, alternate_va, alt,
	    ALTERNATE_COUNT)) != 0)
		return (error);
	put64(out, 0xed8, a->preempt_va);
	put64(out, 0xee0, a->cdm_end_va);
	put64(out, 0xf08, 0x154024201ull);
	put32(out, 0xf20, (uint32_t)(a->object_ids >> 32));
	put32(out, 0xf28, UINT32_MAX);
	put64(out, 0xf40, a->stamp_va[0]);
	put64(out, 0xf48, a->stamp_va[1]);
	put32(out, 0xf50, a->stamp_value);
	put32(out, 0xf54, a->qid);
	put64(out, 0xf60, a->kick_timestamp);
	put64(out, 0xf68, (uint32_t)a->object_ids);
	put32(out, 0xf70, a->completion_ordinal);
	put64(out, 0xf7c, a->status_va[0]);
	put64(out, 0xf84, a->status_va[1]);
	put64(out, 0xf8c, a->timestamp_va[0]);
	put64(out, 0xf94, a->timestamp_va[1]);
	at = 0xfac + (a->common26_tail ? COMMON26_TAIL_INSERT : 0);
	put64(out, at, a->shared_control_va);
	out[at + 8] = a->shared_control_va != 0;
	memcpy(out + at + 9, a->event_metadata, 16);
	put64(out, at + 25, a->usage_fw_va);
	put64(out, at + 33, a->completion_identity);
	return (0);
}

/* fw/kick.rs KickDependency::word and KickEntry::new */
#define	KICK_TIMESTAMP_BITS	40

uint64_t
agx_kick_dependency(uint8_t qid, uint64_t timestamp)
{

	return (timestamp | (uint64_t)qid << KICK_TIMESTAMP_BITS);
}

int
agx_kick_entry(uint8_t out[AGX_KICK_ENTRY_SIZE], const struct agx_kick_args *a)
{
	uint64_t header, types = 0, mask;
	size_t count = a->nbarriers + 1, i;

	if (a->qid > AGX_QID_MAX || a->nbarriers > AGX_KICK_DEPENDENCIES - 1 ||
	    count > AGX_KICK_DEPENDENCIES || a->qos_class > 0x1f ||
	    a->priority >= AGX_PRIORITIES)
		return (EINVAL);
	header = a->timestamp << 2 | (uint64_t)a->qid << 42 |
	    (uint64_t)a->priority << 59;
	if (a->scs_enabled)
		header |= 1ull << 61;
	if (a->compute_matrix)
		header |= 1ull << 63;
	if (a->dependency_types != NULL) {
		for (i = 0; i < count; i++) {
			if (a->dependency_types[i] > 3)
				return (EINVAL);
			types |= (uint64_t)a->dependency_types[i] << (2 * i);
		}
	}
	mask = count == AGX_KICK_DEPENDENCIES ? UINT32_MAX :
	    (1u << count) - 1;
	memset(out, 0, AGX_KICK_ENTRY_SIZE);
	put64(out, 0x0, header);
	put64(out, 0x8, 0);		/* no MCache table */
	put64(out, 0x10, a->descriptor_va);
	put64(out, 0x18, a->queue_va);
	put64(out, 0x20, mask | (uint64_t)a->qos_slot << 32 |
	    (uint64_t)a->qos_class << 40 | 0xffffull << 48);
	for (i = 0; i < a->nbarriers; i++)
		put64(out, 0x28 + 8 * i, a->barriers[i]);
	put64(out, 0x28 + 8 * a->nbarriers,
	    agx_kick_dependency(a->qid, a->parent));
	for (i = 0; i < 4; i++)
		put64(out, 0x128 + 8 * i, a->event_mask[i]);
	for (i = 0; i < 4; i++)
		put64(out, 0x150 + 8 * i, a->register_arrays[i]);
	put64(out, 0x178, 0x003fffffffffffffull);
	put64(out, 0x180, types);
	return (0);
}

/* fw/queue.rs QueueConfig::new, QueueConfigTarget::Compute */
#define	TAG_KICK_ANNOUNCE	0x0e
#define	TAG_QUEUE_CONFIG	0x0f

int
agx_queue_config_compute(uint8_t out[AGX_QUEUE_CONFIG_SIZE],
    const struct agx_queue_config_args *a)
{

	if (a->qid > AGX_QID_MAX)
		return (EINVAL);
	memset(out, 0, AGX_QUEUE_CONFIG_SIZE);
	put32(out, 0x0, TAG_QUEUE_CONFIG);
	put64(out, 0x8, a->kick_ring_va);
	put64(out, 0x10, a->kick_ring_fw_va);
	put16(out, 0x18, a->qid);
	put16(out, 0x1a, a->install);
	put16(out, 0x1e, a->priority);
	put16(out, 0x22, AGX_DM_COMPUTE);
	put32(out, 0x26, a->has_completion_seed);
	put64(out, 0x2a, a->has_completion_seed ? a->completion_seed : 0);
	put16(out, 0x32, a->context_id);
	put64(out, 0x36, a->free_list_va);
	put64(out, 0x3e, a->free_list_generation);
	put32(out, 0x46, a->free_list_slot);
	put64(out, 0x4a, a->scheduler_va);
	put32(out, 0x52, a->qos_update);
	put16(out, 0x56, a->qos_slot);
	put32(out, 0x5a, a->owner_pid);
	put16(out, 0x5e, a->policy);
	put16(out, 0x62, a->context_update);
	put16(out, 0x66, 1);		/* UNK_66 */
	put64(out, 0x6e, 0);		/* pb_va */
	put64(out, 0x76, UINT64_MAX);	/* pb_token */
	put32(out, 0x7e, UINT32_MAX);	/* pb_slot */
	put32(out, 0x82, UINT32_MAX);	/* fragment_qid */
	return (0);
}

int
agx_kick_announce(uint8_t out[AGX_KICK_ANNOUNCE_SIZE], uint8_t qid,
    uint32_t data_master, uint64_t timestamp, uint8_t priority)
{

	if (qid > AGX_QID_MAX || priority >= AGX_PRIORITIES)
		return (EINVAL);
	memset(out, 0, AGX_KICK_ANNOUNCE_SIZE);
	put32(out, 0x0, TAG_KICK_ANNOUNCE);
	put16(out, 0x4, qid);
	put16(out, 0x6, 1);
	out[0x8] = priority;
	put64(out, 0x9, timestamp);
	put32(out, 0x11, data_master);
	return (0);
}

/* fw/channels.rs WorkSlot::new */
int
agx_work_slot(uint8_t out[AGX_WORK_SLOT_SIZE], uint32_t data_master,
    uint64_t queue_va, uint8_t qid, uint16_t items_end, bool first)
{

	if (qid > AGX_QID_MAX)
		return (EINVAL);
	memset(out, 0, AGX_WORK_SLOT_SIZE);
	put64(out, 0x8, queue_va);
	put32(out, 0x10, data_master);
	put16(out, 0x14, items_end);
	out[0x16] = qid;
	out[0x17] = first;
	return (0);
}
