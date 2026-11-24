/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Apple T8142 AOP audio: the firmware-control interface for the sound glue.
 * Follows the non-ASoC part of aurora's sound/soc/apple/t8152_aop_audio.rs.
 *
 * The AOP firmware owns the audio fabric and runs the MCA serializers; the
 * host only moves samples with the ADMAC. A stream's life, as the Rust
 * back-end DAI's:
 *
 *   apple_aop_audio_stream_open()   startup: attach the firmware device,
 *                                   request "pw0 " (raises the fabric), then
 *                                   power the MCA leaves. Only after this
 *                                   may the ADMAC channel be touched.
 *   apple_aop_audio_stream_power(s, true)
 *                                   prepare: "pwrd", the firmware starts the
 *                                   serializer and its clocks (cout/cin go
 *                                   back through "pw0 " first if running, so
 *                                   a restart begins on a frame boundary).
 *   apple_aop_audio_stream_power(s, false)
 *                                   back to "pw0 " (clocks stop).
 *   apple_aop_audio_stream_close()  shutdown: "pw0 " if running, MCA leaves
 *                                   released.
 *
 * "sense" (the speaker's V/I capture) has no service of its own: open puts
 * "spkr" in "pw0 " unless it runs, power and close do nothing; it runs with
 * the speaker's playback. If the sense capture is running when "spkr" goes
 * to "pwrd", the glue should restart it at a frame boundary (aurora's
 * realign_sense; info.realign).
 *
 * All calls sleep (firmware calls of up to 5 s); none may be made from
 * interrupt context.
 */

#ifndef	_ARM64_APPLE_APPLE_AOP_AUDIO_H_
#define	_ARM64_APPLE_APPLE_AOP_AUDIO_H_

struct apple_aop_audio_stream;

/* The stream's fixed shape: the firmware runs it, the PCM must match. */
struct apple_aop_audio_info {
	uint32_t	rate;		/* 48000 */
	uint32_t	channels;	/* cout 2, cin 1, spkr 4|6, sense 8|12 */
	uint32_t	sample_bits;	/* 24 (S24_LE in 32-bit words), sense 16 */
	uint32_t	word_bytes;	/* 4, sense 2 */
	uint32_t	port_window;	/* ADMAC window: min(channels, 4) */
	bool		playback;
	bool		realign;	/* sense: see above */
};

/* name: "cout", "cin", "spkr" or "sense". dev: the apple_aop_audio device. */
int	apple_aop_audio_stream_open(device_t dev, const char *name,
	    struct apple_aop_audio_stream **sp);
int	apple_aop_audio_stream_power(struct apple_aop_audio_stream *s,
	    bool running);
void	apple_aop_audio_stream_close(struct apple_aop_audio_stream *s);

void	apple_aop_audio_stream_info(struct apple_aop_audio_stream *s,
	    struct apple_aop_audio_info *info);

/*
 * The stream's ADMAC channel from the node's dmas/dma-names: the provider's
 * xref and its first specifier cell (the channel; odd = device to memory).
 * Works without an open stream too (by name, on the audio device).
 */
int	apple_aop_audio_dma(device_t dev, const char *name, phandle_t *xref,
	    uint32_t *cell);

#endif /* _ARM64_APPLE_APPLE_AOP_AUDIO_H_ */
