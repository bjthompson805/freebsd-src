/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * The Linux types and macros the vendored V4L2 headers (v4l_compat 1.23.0,
 * webcamd's Linux UAPI) use, for the kernel. Userland gets them from
 * v4l_compat's linux/types.h; the layouts are the same.
 */
#ifndef _ARM64_APPLE_V4L2_LINUX_TYPES_H_
#define	_ARM64_APPLE_V4L2_LINUX_TYPES_H_

#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioccom.h>

typedef uint64_t __u64;
typedef uint32_t __u32;
typedef uint16_t __u16;
typedef uint8_t __u8;
typedef int64_t __s64;
typedef int32_t __s32;
typedef int16_t __s16;
typedef int8_t __s8;
typedef uint32_t __le32;

#define	_BITUL(x)	(1UL << (x))
#define	_BITULL(x)	(1ULL << (x))
#define	__user

#endif
