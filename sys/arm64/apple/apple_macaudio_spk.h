/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Brandon Thompson
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * The macaudio card's speakers (apple_macaudio_spk.c): the speaker and
 * speaker-sense sound(4) devices, children of the card's jack device, and
 * the speaker protection interlock.  See ~/m1n1-env/docs/speaker-interface.md
 * for the userland interface.
 */

#ifndef	_ARM64_APPLE_APPLE_MACAUDIO_SPK_H_
#define	_ARM64_APPLE_APPLE_MACAUDIO_SPK_H_

struct ma_spk;

/*
 * card: the apple,macaudio device (the jack's pcm device); aop_audio: the
 * AOP audio device of its links.  Returns 0 with *spp NULL when the card has
 * no admissible speakers.  Sleeps.
 */
int	ma_spk_attach(device_t card, device_t aop_audio, struct ma_spk **spp);
/* Detaches both children; sp may be NULL.  Sleeps. */
int	ma_spk_detach(struct ma_spk *sp);

#endif /* _ARM64_APPLE_APPLE_MACAUDIO_SPK_H_ */
