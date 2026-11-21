/*-
 * SPDX-License-Identifier: MIT
 *
 * Apple M5 GPU: render-pair (tiling and fragment) command encoders,
 * translated from Aurora's drm/asahi g17 (fw/render.rs write_m5,
 * fw/m5/render_registers.rs, fw/m5/render.rs, fw/m5/render_tail.rs,
 * fw/m5/render_assembly.rs, queue/render/m5_render_submission.rs and
 * buffer/target.rs Layout). Like apple_agx_fw.c it builds in the kernel and
 * on the host, where tools/agx_fw_test checks it byte for byte against the
 * Rust encoders.
 */

#ifndef	_ARM64_APPLE_APPLE_AGX_RENDER_H_
#define	_ARM64_APPLE_APPLE_AGX_RENDER_H_

#include "apple_agx_fw.h"

#define	AGX_M5_TA_COMMAND_BYTES		0x9a0
#define	AGX_M5_3D_COMMAND_BYTES		0x2240
#define	AGX_M5_RENDER_FRAGMENT_OFFSET	0x4000
#define	AGX_M5_RENDER_STORAGE_BYTES	0x8000

#define	AGX_TA_QOS_CLASS		0x0c
#define	AGX_FRAGMENT_QOS_CLASS		0x18

/* fw/render.rs ZlsSurface */
struct agx_zls_surface {
	uint64_t	va;
	uint64_t	compression_va;
	uint64_t	stride;
	uint64_t	compression_stride;
};

/* fw/render.rs RenderPass; usc offsets are tagged, as userspace sends them. */
struct agx_render_pass {
	uint32_t	width;
	uint32_t	height;
	uint16_t	layers;
	uint8_t		utile_width;
	uint8_t		utile_height;
	uint8_t		samples_log2;
	uint8_t		sample_size;
	bool		process_empty_tiles;
	uint64_t	msaa_control;
	uint64_t	ppp_control;
	uint64_t	vdm_va;
	uint64_t	scissor_va;
	uint64_t	depth_bias_va;
	bool		depth_bias_is_int;
	bool		few_primitives;
	bool		mesh_output;
	bool		fragment_barriers;
	bool		stage_prefixes;
	bool		whole_attachments;
	bool		shader_writes_memory;
	uint64_t	occlusion_query_va;
	uint64_t	sampler_heap_va;
	uint32_t	sampler_count;
	uint64_t	zls_control;
	uint64_t	depth_dimensions;
	struct agx_zls_surface depth;
	struct agx_zls_surface stencil;
	uint32_t	depth_clear;
	uint32_t	stencil_clear;
	uint32_t	merge_upper_x;
	uint32_t	merge_upper_y;
	uint64_t	usc_base;
	uint32_t	bg_offset;
	uint64_t	bg_rsrc_spec;
	uint32_t	partial_bg_offset;
	uint64_t	partial_bg_rsrc_spec;
	uint32_t	eot_offset;
	uint64_t	eot_rsrc_spec;
	uint32_t	partial_eot_offset;
	uint64_t	partial_eot_rsrc_spec;
	bool		rsrc_spec_hi;
};

/* fw/mod.rs ObjectIds */
struct agx_object_ids {
	uint32_t	predecessor;
	uint32_t	current;
};

/* fw/render.rs RenderResources, the fields the M5 path reads. */
struct agx_render_resources {
	uint16_t	context_id;
	uint8_t		context_generation;
	uint32_t	pb_slot;
	uint32_t	free_list_slot;
	uint64_t	free_list_control_va;
	uint64_t	kick_record_va;
	uint64_t	pb_va;
	uint64_t	scene_va;
	uint64_t	scene_end_va;
	uint64_t	pm_scratch;
	uint64_t	pm_metric;
	uint64_t	tilemap_va;
	uint64_t	layermeta_va;
	uint64_t	heapmeta_va;
	uint64_t	tpc_va;
	uint64_t	tpc_size;
	uint64_t	deflake_va;
	uint64_t	ta_status_va;
	uint64_t	mesh_storage;
	uint64_t	fragment_status_va;
	uint64_t	aux_fb_va;
	uint64_t	timestamp_va;
	struct agx_object_ids ta_ids;
	struct agx_object_ids fragment_ids;
};

/* fw/render.rs RenderStage */
struct agx_render_stage {
	uint8_t		qid;
	uint64_t	descriptor_va;
	uint32_t	state_word;
	uint64_t	stamp_va;
	uint64_t	aux_stamp_va;
};

/* render_resources.rs StagePlan: one stage's SCS and resource budget. */
struct agx_stage_plan {
	struct agx_scratch scratch;
	struct agx_intersection intersection;
	uint32_t	auxiliary_allocation;
	uint64_t	resource_flags;
	uint32_t	ptc_max;
};

/* fw/render.rs RenderArgs with memory.rs write_m5_commands' inputs. */
struct agx_render_args {
	const struct agx_board *board;
	const struct agx_render_pass *pass;
	const struct agx_render_resources *resources;
	uint64_t	ordinal;
	struct agx_render_stage ta;
	struct agx_render_stage fragment;
	bool		has_stages;	/* the plan's stages */
	struct agx_stage_plan stages[2];
	uint8_t		mesh_class;
	uint64_t	usage[2];	/* graph m5_usage */
};

/* render_assembly.rs EncodedRender, with the kick flags. */
struct agx_m5_render_encoded {
	uint64_t	rce[2][4];	/* RegisterArrayBinding words, 0 if none */
	uint16_t	register_counts[5];
	bool		scratch[2];
	bool		matrix[2];
};

/* buffer/target.rs Layout for an M5 pass (with_m5_slices). */
struct agx_m5_target_layout {
	size_t		tilemap_size;
	size_t		layer_offset;
	size_t		heap_offset;
	size_t		used;
	size_t		tpc_size;
	size_t		tvb_blocks;
	size_t		slice_sizes[8];
	size_t		slice_offsets[8];
	size_t		slice_bytes;
	uint64_t	cluster_control[3];
};

int	agx_m5_render_validate(const struct agx_render_pass *p);
int	agx_m5_target_layout(const struct agx_render_pass *p, uint32_t clusters,
	    size_t parameter_bytes, struct agx_m5_target_layout *l);
int	agx_m5_render_encode(uint8_t storage[AGX_M5_RENDER_STORAGE_BYTES],
	    uint64_t low, const struct agx_render_args *a,
	    struct agx_m5_render_encoded *out);
void	agx_m5_render_link_completions(
	    uint8_t storage[AGX_M5_RENDER_STORAGE_BYTES], const uint64_t kicks[2],
	    uint8_t pass, bool fragment_busy);
int	agx_m5_render_bind_work_slot(
	    uint8_t storage[AGX_M5_RENDER_STORAGE_BYTES], uint32_t slot);

#endif /* _ARM64_APPLE_APPLE_AGX_RENDER_H_ */
