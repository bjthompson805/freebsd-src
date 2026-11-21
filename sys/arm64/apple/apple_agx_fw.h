/*-
 * SPDX-License-Identifier: MIT
 *
 * Apple M5 GPU: firmware record encoders for compute kicks, translated from
 * Aurora's drm/asahi g17/fw (m5/compute.rs, kick.rs, queue.rs, channels.rs).
 * Plain C over byte buffers, so that the same file builds in the kernel and
 * on the host, where it is checked byte for byte against the Rust encoders
 * (m1n1-env tools/agx_graph).
 */

#ifndef	_ARM64_APPLE_APPLE_AGX_FW_H_
#define	_ARM64_APPLE_APPLE_AGX_FW_H_

#ifdef _KERNEL
#include <sys/types.h>
#else
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#endif

/* The parts of fw/m5/initdata.rs Board the compute encoders read. */
struct agx_board {
	uint32_t	clusters;
	bool		j813_forms;
	bool		completion_inputs;
	bool		g17d_forms;
	bool		common26;
};

#define	AGX_KICK_ENTRY_SIZE		0x200
#define	AGX_KICK_DEPENDENCIES		32
#define	AGX_M5_DESCRIPTOR_SIZE		0x1000	/* DESCRIPTOR_PREFIX_BYTES */
#define	AGX_QUEUE_CONFIG_SIZE		0xc0
#define	AGX_KICK_ANNOUNCE_SIZE		0x18
#define	AGX_WORK_SLOT_SIZE		0x18
#define	AGX_QID_MAX			0x7f
#define	AGX_PRIORITIES			4

#define	AGX_DM_TILING			0
#define	AGX_DM_FRAGMENT			1
#define	AGX_DM_COMPUTE			2

#define	AGX_COMPUTE_PIPELINED_DEPENDENCY 2
#define	AGX_COMPLETION_DEPENDENCY	1
#define	AGX_COMPUTE_QOS_CLASS		0x08

/* fw/m5/compute.rs ScratchRequest. */
struct agx_scratch {
	uint64_t	layout;
	uint32_t	driver_end;
	uint16_t	pages;
};

/* fw/m5/compute.rs IntersectionTables. */
struct agx_intersection {
	uint64_t	pool;
	uint64_t	pool_bytes;
	uint32_t	first;
	uint32_t	count;
	uint64_t	default_program;
	uint64_t	default_control;
};

/* fw/m5/compute.rs KickTuple. */
struct agx_kick_tuple {
	uint32_t	preempt_state_offset;
	uint64_t	dispatch_base;
	uint64_t	auxiliary_control;
};

/* fw/m5/compute.rs RegisterArgs and DescriptorArgs, in one. */
struct agx_m5_compute_args {
	bool		matrix_ops;
	struct agx_scratch scratch;
	struct agx_intersection intersection;
	uint32_t	auxiliary_allocation;
	uint64_t	preempt_va;
	uint64_t	cdm_va;
	uint64_t	operand_va;
	uint64_t	usage_va;
	uint32_t	free_list_slot;
	uint16_t	context_id;
	uint32_t	work_state;
	uint64_t	object_ids;
	uint64_t	resource_policy;
	uint32_t	preempt_state_offset;
	uint32_t	operand_region_bytes;
	uint32_t	operand_aux_offset;
	size_t		operand_bytes;
	uint64_t	resource_control;
	uint64_t	dispatch_control;
	uint64_t	auxiliary_control;

	uint64_t	descriptor_va;
	uint64_t	submit_sequence;
	uint64_t	work_state_va;
	uint64_t	cdm_end_va;
	uint64_t	stamp_va[2];
	uint32_t	stamp_value;
	uint32_t	qid;
	uint64_t	kick_timestamp;
	uint32_t	completion_ordinal;
	uint64_t	status_va[2];
	uint64_t	timestamp_va[2];
	uint64_t	shared_control_va;
	uint8_t		event_metadata[16];
	uint64_t	usage_fw_va;
	uint64_t	completion_identity;
	bool		common26_tail;
};

/* fw/kick.rs KickArgs, for compute (no MCache table, no ancestor). */
struct agx_kick_args {
	uint8_t		qid;
	uint64_t	timestamp;
	uint64_t	parent;
	const uint64_t	*barriers;	/* KickDependency words */
	size_t		nbarriers;
	const uint8_t	*dependency_types;	/* nbarriers + 1, or NULL */
	uint64_t	descriptor_va;
	uint64_t	queue_va;
	uint8_t		qos_slot;
	uint8_t		qos_class;
	uint64_t	event_mask[4];
	uint64_t	register_arrays[4];	/* RegisterArrayBinding words */
	bool		scs_enabled;
	bool		compute_matrix;
	uint8_t		priority;
};

/* fw/queue.rs QueueConfigArgs for a compute queue. */
struct agx_queue_config_args {
	uint64_t	kick_ring_va;
	uint64_t	kick_ring_fw_va;
	uint8_t		qid;
	bool		install;
	bool		has_completion_seed;
	uint64_t	completion_seed;
	uint16_t	context_id;
	uint64_t	free_list_va;
	uint64_t	free_list_generation;
	uint32_t	free_list_slot;
	uint64_t	scheduler_va;
	uint8_t		priority;	/* Policy::priority */
	uint16_t	policy;		/* Policy::policy */
	uint8_t		qos_slot;
	bool		qos_update;
	uint32_t	owner_pid;
	bool		context_update;
};

int	agx_scratch_new(uint64_t layout, struct agx_scratch *s);
int	agx_intersection_new(uint64_t pool, uint64_t pool_bytes, uint32_t first,
	    uint32_t count, uint64_t default_program, uint64_t default_control,
	    struct agx_intersection *t);

bool	agx_scratch_enabled(const struct agx_scratch *s);
uint64_t agx_scratch_size_word(const struct agx_scratch *s);
uint64_t agx_scratch_window_word(const struct agx_scratch *s);
uint64_t agx_scratch_stack_word(const struct agx_scratch *s);
uint64_t agx_scratch_driver_word(const struct agx_scratch *s);
void	agx_intersection_words(const struct agx_intersection *t, uint64_t w[6]);
int	agx_register_array_binding(uint64_t array_va, unsigned checkpoint,
	    uint64_t *word);

uint64_t agx_m5_dispatch_control(const struct agx_board *b, uint64_t base);
uint8_t	agx_m5_explicit_dependency(const struct agx_board *b);
uint8_t	agx_m5_parent_dependency(const struct agx_board *b, bool parent_retired);
uint32_t agx_m5_completion_ordinal(const struct agx_board *b,
	    uint32_t published_before);
void	agx_m5_completion_forms(const struct agx_board *b, uint64_t *identity,
	    uint8_t metadata[16]);
int	agx_m5_kick_event_mask(const struct agx_board *b, uint32_t context_id,
	    uint64_t mask[4]);
void	agx_m5_kick_tuple(const struct agx_board *b, uint8_t parent_class,
	    struct agx_kick_tuple *t);
int	agx_m5_bindings(uint64_t descriptor_va, const struct agx_scratch *s,
	    bool matrix_ops, uint64_t words[4]);

int	agx_m5_compute_descriptor(uint8_t out[AGX_M5_DESCRIPTOR_SIZE],
	    const struct agx_m5_compute_args *a);
int	agx_kick_entry(uint8_t out[AGX_KICK_ENTRY_SIZE],
	    const struct agx_kick_args *a);
uint64_t agx_kick_dependency(uint8_t qid, uint64_t timestamp);
int	agx_queue_config_compute(uint8_t out[AGX_QUEUE_CONFIG_SIZE],
	    const struct agx_queue_config_args *a);
int	agx_queue_config_render(uint8_t out[AGX_QUEUE_CONFIG_SIZE],
	    const struct agx_queue_config_args *a, bool tiling, uint64_t pb_va,
	    uint64_t pb_token, uint32_t pb_slot, uint8_t fragment_qid);
int	agx_kick_announce(uint8_t out[AGX_KICK_ANNOUNCE_SIZE], uint8_t qid,
	    uint32_t data_master, uint64_t timestamp, uint8_t priority);
int	agx_work_slot(uint8_t out[AGX_WORK_SLOT_SIZE], uint32_t data_master,
	    uint64_t queue_va, uint8_t qid, uint16_t items_end, bool first);

static inline uint32_t
agx_stamp_value(uint32_t sequence)
{

	return (sequence << 8);
}

#endif /* _ARM64_APPLE_APPLE_AGX_FW_H_ */
