/****************************************************************************
 *
 * Copyright 2026 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * os/include/tinyara/mm/kasan.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 * Derived from Apache NuttX include/nuttx/mm/kasan.h at commit b4aa94d.
 * The software tag, hardware tag and global instrumentation interfaces of
 * the original are not present; TizenRT supports generic mode only.
 *
 ****************************************************************************/

#ifndef __INCLUDE_MM_KASAN_H
#define __INCLUDE_MM_KASAN_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdbool.h>
#include <stddef.h>

#include <tinyara/compiler.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Cases the self test can run. Declared unconditionally so that user space
 * can name them without having to know whether the kernel it is talking to
 * was built with KASan; the kernel rejects every case when it was not.
 *
 * Every case but the first is expected to be reported and, unless the
 * corresponding panic has been disabled, to assert. That is the pass
 * condition, not a failure of the test.
 */

enum kasan_selftest_e {
	KASAN_SELFTEST_INBOUNDS = 0,	/* Access inside the block. No report. */
	KASAN_SELFTEST_OVERFLOW_WRITE,	/* Write one byte past the block */
	KASAN_SELFTEST_OVERFLOW_READ,	/* Read one byte past the block */
	KASAN_SELFTEST_UNDERFLOW,	/* Write one byte before the block */
	KASAN_SELFTEST_USE_AFTER_FREE,	/* Write to a block already freed */
	KASAN_SELFTEST_STRADDLE,	/* Word write across the block end */
	KASAN_SELFTEST_MAX
};

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The entry points are real in two cases only: a flat build, where there is
 * one image and one heap, and the kernel pass of a split build, which is the
 * pass that compiles with __KERNEL__ defined.
 *
 * Everywhere else they collapse to nothing. The user pass of a split build
 * compiles the same memory manager sources, but allocates from a heap that is
 * never registered, and os/mm/kasan/Make.defs deliberately keeps the runtime
 * out of libumm. Leaving the call sites as real calls there would put an
 * undefined reference to kasan_poison into every user image, for a call that
 * could only ever find no region and return.
 *
 * This is the same test mm_initialize.c applies before registering a heap,
 * written the other way round, so registration and the poison calls agree on
 * which pass owns KASan.
 */

#if !defined(CONFIG_MM_KASAN) || \
	(!defined(CONFIG_BUILD_FLAT) && !defined(__KERNEL__))

/* With KASan disabled, or in the user pass, every entry point collapses to
 * nothing, so the call sites in the memory manager need no conditional
 * compilation of their own.
 *
 * Each stub still consumes its arguments. A call site typically keeps a size
 * in a local just to pass it here, and discarding the argument would leave
 * that local set but unused in every build with KASan off.
 */

#define kasan_poison(addr, size)	do { (void)(addr); (void)(size); } while (0)
#define kasan_unpoison(addr, size)	((void)(size), (FAR void *)(addr))
#define kasan_register(addr, size)	do { (void)(addr); (void)(size); } while (0)
#define kasan_unregister(addr)		do { (void)(addr); } while (0)
#define kasan_start()
#define kasan_stop()
#define kasan_init_early()

#else

/* Called from the earliest point of the boot sequence. Checking must stay
 * disabled until the first region is registered, because instrumented code
 * runs long before any heap exists, and on some platforms before .bss has
 * been cleared.
 */

#define kasan_init_early()	kasan_stop()

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Name: kasan_poison
 *
 * Description:
 *   Mark the memory range as inaccessible. Any instrumented access to a
 *   poisoned address is reported.
 *
 * Input Parameters:
 *   addr - range start address
 *   size - range size in bytes
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void kasan_poison(FAR const void *addr, size_t size);

/****************************************************************************
 * Name: kasan_unpoison
 *
 * Description:
 *   Mark the memory range as accessible.
 *
 * Input Parameters:
 *   addr - range start address
 *   size - range size in bytes
 *
 * Returned Value:
 *   The address that was passed in, so the call can wrap an allocator
 *   return value directly.
 *
 ****************************************************************************/

FAR void *kasan_unpoison(FAR const void *addr, size_t size);

/****************************************************************************
 * Name: kasan_register
 *
 * Description:
 *   Monitor the memory range for invalid access checking.
 *
 *   The shadow map for the region is carved out of the tail of the region
 *   itself, so no linker script support is needed. On return *size has been
 *   reduced by the number of bytes taken for the shadow, and the caller must
 *   build its heap over the reduced range only.
 *
 *   The whole region is left poisoned, so an allocator must call
 *   kasan_unpoison() for each block it hands out.
 *
 *   Checking becomes active from the first successful registration.
 *
 * Input Parameters:
 *   addr - range start address
 *   size - on entry the region size, on return the size left for the caller
 *
 * Returned Value:
 *   None. If the region cannot be registered *size is left unchanged and
 *   the region is simply not covered.
 *
 ****************************************************************************/

void kasan_register(FAR void *addr, FAR size_t *size);

/****************************************************************************
 * Name: kasan_unregister
 *
 * Description:
 *   Stop monitoring the memory range beginning at addr.
 *
 * Input Parameters:
 *   addr - range start address, as passed to kasan_register()
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

void kasan_unregister(FAR void *addr);

/****************************************************************************
 * Name: kasan_start
 *
 * Description:
 *   Allow KASan to report. Called automatically by kasan_register().
 *
 ****************************************************************************/

/****************************************************************************
 * Name: kasan_stop
 *
 * Description:
 *   Stop KASan reporting. Used before the boot sequence has a heap, and by
 *   the assertion path so that the crash dump can walk the heap without
 *   tripping a second report.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KASAN_INSTRUMENT
void kasan_start(void);
void kasan_stop(void);
#else
#define kasan_start()
#define kasan_stop()
#endif

/****************************************************************************
 * Name: kasan_selftest
 *
 * Description:
 *   Perform one deliberate memory error against the kernel heap so that the
 *   instrumentation can be shown to work. Without this there is no way to
 *   tell a system with no memory errors from a system where the
 *   instrumentation was never emitted; both are silent.
 *
 *   Lives outside mm/ because everything under mm/ is built with
 *   -fno-sanitize=kernel-address and so cannot raise a report.
 *
 * Input Parameters:
 *   testcase - one of enum kasan_selftest_e
 *
 * Returned Value:
 *   Zero when KASAN_SELFTEST_INBOUNDS completed with nothing reported, or a
 *   negated errno. The remaining cases are not expected to return at all.
 *
 ****************************************************************************/

#ifdef CONFIG_MM_KASAN_SELFTEST
int kasan_selftest(int testcase);
#endif

#ifdef __cplusplus
}
#endif

#endif							/* KASan real in this pass */
#endif							/* __INCLUDE_MM_KASAN_H */
