/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple audio DMA controller (ADMAC), the kernel API for sound(4) glue.
 * The driver follows aurora-silicon's drivers/dma/apple-admac.c.
 *
 * A channel is hardwired to one direction: even channels are MEM_TO_DEV
 * (playback, TX), odd ones DEV_TO_MEM (capture, RX). Each transfer is a
 * cyclic buffer split into equal periods; the controller holds up to four
 * period descriptors and the driver refills them on each period interrupt.
 *
 * Power: the controller is in the audio fabric the AOP firmware powers. The
 * driver touches no register at attach. apple_admac_get_channel() touches
 * none either; the first register access is apple_admac_config(), which the
 * caller must only make once the audio power state (for admac-leap-s: the
 * LEAP leaves) is up. From then on until apple_admac_release() the caller
 * keeps it up. If the fabric loses power in between, call
 * apple_admac_config() again before the next start.
 *
 * Usage, for one stream:
 *	apple_admac_get_channel(xref, cell, &ch);	(from "dmas")
 *	apple_admac_alloc_buffer(ch, len, &kva, &iova);	(AOP running)
 *	apple_admac_config(ch, 4, 2);			(audio powered)
 *	apple_admac_start_cyclic(ch, iova, len, period, cb, arg);
 *	... apple_admac_position(ch) from the pcm getptr ...
 *	apple_admac_stop(ch);
 *	apple_admac_release(ch);			(sleepable)
 */

#ifndef	_ARM64_APPLE_APPLE_ADMAC_H_
#define	_ARM64_APPLE_APPLE_ADMAC_H_

struct apple_admac_chan;

/*
 * Resolve one "dmas" entry, <&admac cell>: xref is the controller's phandle
 * as it appears in the property, cell the channel number. The channel is
 * claimed until apple_admac_release(). No register access. ENXIO if the
 * controller has not attached (yet), EINVAL for a bad cell, EBUSY if the
 * channel is claimed.
 */
int	apple_admac_get_channel(phandle_t xref, uint32_t cell,
	    struct apple_admac_chan **chp);

/* The channel's direction: true for TX (even channel, memory to device). */
bool	apple_admac_is_tx(struct apple_admac_chan *ch);

/*
 * Configure the channel: bus_width_bytes is the sample word on the
 * peripheral side (1, 2 or 4), frame_words the words per frame (0 or 1 for
 * no framing, 2 or 4), as Linux's addr_width and port_window_size. Takes
 * the channel's SRAM carveout on first use and (re)writes it. First
 * register access: the audio fabric must be powered. Not while running.
 * Sleeps (may allocate); EINVAL, EBUSY (no SRAM left).
 */
int	apple_admac_config(struct apple_admac_chan *ch, int bus_width_bytes,
	    int frame_words);

/*
 * A DMA buffer for the channel: physically contiguous, 16 KiB aligned and
 * rounded, non-cacheable (write-combining) in the kernel, mapped through the
 * controller's DART stream ("iommus") at an IOVA in its "apple,dma-range".
 * Requires the DART to be powered (AOP running). The buffer and its DART
 * mapping are kept for the channel's next use, since DART mappings are never
 * torn down; a later call with len at most the kept size returns the same
 * buffer. Sleeps.
 */
int	apple_admac_alloc_buffer(struct apple_admac_chan *ch, size_t len,
	    void **kva, bus_addr_t *iova);
/* Gives the buffer back to the channel (kept, see above). */
void	apple_admac_free_buffer(struct apple_admac_chan *ch, void *kva);

/*
 * Start a cyclic transfer over [iova, iova + buf_len) in periods of
 * period_len bytes (buf_len a multiple of period_len, both non-zero, buf_len
 * < 4 GiB). period_cb(arg) is called once per completed period from the
 * controller's interrupt thread (ithread, no ADMAC locks held, must not
 * sleep), so it can be a sound(4) chn_intr() call. Call apple_admac_config()
 * first. EBUSY if running, EINVAL.
 */
int	apple_admac_start_cyclic(struct apple_admac_chan *ch, bus_addr_t iova,
	    size_t buf_len, size_t period_len, void (*period_cb)(void *),
	    void *arg);

/*
 * Stop the transfer: clears the channel's enable bit, resets its rings and
 * waits (spinning, up to 100 ms) for the channel to wind down, so it no
 * longer touches the buffer on return. No sleep: callable with a pcm
 * channel lock held. A period callback that was already running may still
 * finish after this returns; none starts afterwards.
 */
void	apple_admac_stop(struct apple_admac_chan *ch);

/*
 * The transfer position: bytes into the buffer, [0, buf_len), for a pcm
 * pointer. Never ahead of the hardware and never behind the last position
 * returned. 0 when not running. No sleep.
 */
size_t	apple_admac_position(struct apple_admac_chan *ch);

/* Whether the transfer stopped on a controller error (then: stop, restart). */
bool	apple_admac_error(struct apple_admac_chan *ch);

/*
 * Stop, wait for period callbacks to drain, give back the SRAM carveout and
 * unclaim the channel. The kept buffer stays with the channel. Sleeps.
 */
void	apple_admac_release(struct apple_admac_chan *ch);

#endif /* _ARM64_APPLE_APPLE_ADMAC_H_ */
