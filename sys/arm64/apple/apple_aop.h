/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple AOP (Always-On Processor): the interface its service drivers (the
 * children of apple_aop, such as apple_aop_audio) use. Follows aurora's
 * rust/kernel/soc/apple/aop.rs (the AOP trait and EPICService).
 */

#ifndef	_ARM64_APPLE_APPLE_AOP_H_
#define	_ARM64_APPLE_APPLE_AOP_H_

/*
 * An EPIC service the firmware announced: the RTKit (AFK) endpoint it runs
 * on and the channel it answers on.
 */
struct apple_aop_service {
	uint32_t	channel;
	uint8_t		endpoint;
};

/* The default bound on an EPIC call (aurora's EPIC_CALL_TIMEOUT_MS). */
#define	APPLE_AOP_CALL_TIMEOUT_MS	5000

/*
 * Find the service the firmware announced as name (e.g. "aop-audio"),
 * waiting up to timeout_ms for the announcement. ENOENT if it did not come,
 * ENXIO if the AOP is not running. Sleeps.
 */
int	apple_aop_service_find(device_t aop, const char *name, int timeout_ms,
	    struct apple_aop_service *svc);

/*
 * A synchronous EPIC call (aurora's epic_call / epic_call_ret): send req
 * (reqlen bytes) as a call of subtype on svc and wait up to timeout_ms
 * (0: APPLE_AOP_CALL_TIMEOUT_MS) for the reply.
 *
 * *retcode gets the reply's return code (the first word of its payload).
 * With reply != NULL, *replylen is the room in reply on entry (advertised to
 * the firmware as the reply capacity) and the number of bytes the firmware
 * returned after the return code on exit. With reply == NULL the call is
 * sent without a reply buffer, as epic_call does.
 *
 * Errors: ENODEV (AOP going away or crashed), EAGAIN (AOP asleep),
 * ETIMEDOUT (no reply; on this firmware the endpoint then takes no call
 * until the late reply arrives, and later calls fail with EIO), EIO.
 * Calls on one endpoint are serialized. Sleeps; never call it from a report
 * callback.
 */
int	apple_aop_epic_call(device_t aop, const struct apple_aop_service *svc,
	    uint16_t subtype, const void *req, size_t reqlen, void *reply,
	    size_t *replylen, uint32_t *retcode, int timeout_ms);

/*
 * Reports (EPIC category 0) the firmware sends on svc with this subtype.
 * The callback runs on the AOP's poll thread without AOP locks held; data is
 * the payload after the EPIC header and is only valid during the call. It
 * must not sleep for long and must not make EPIC calls (defer them to a
 * taskqueue). EBUSY if the (svc, subtype) pair already has a listener.
 */
typedef void apple_aop_report_cb_t(void *arg, uint16_t subtype,
	    const void *data, size_t len);
int	apple_aop_report_listen(device_t aop, const struct apple_aop_service *svc,
	    uint16_t subtype, apple_aop_report_cb_t *cb, void *arg);
/* Remove a listener; on return its callback is not running and won't run. */
void	apple_aop_report_unlisten(device_t aop,
	    const struct apple_aop_service *svc, uint16_t subtype);

#endif /* _ARM64_APPLE_APPLE_AOP_H_ */
