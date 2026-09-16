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
 * os/mm/kasan/hook.c
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
 * Derived from Apache NuttX mm/kasan/hook.c at commit b4aa94d.
 *
 * Differences from the original, all deliberate:
 *
 *  1. Reports go to lowsyslog() rather than to _alert(). TizenRT's lldbg()
 *     expands to nothing unless CONFIG_DEBUG, CONFIG_DEBUG_ERROR and
 *     CONFIG_ARCH_LOWPUTC are all set, which would leave KASan panicking
 *     with no output at all in a plain build.
 *  2. The shadow dump formats its own hex rather than calling sprintf() on
 *     a 256 byte stack buffer, and prints ".." for any byte outside a
 *     registered region rather than dereferencing it. Upstream reads 80
 *     bytes either side of the faulting address unconditionally, which can
 *     fault a second time while reporting on a target without an MMU.
 *  3. The ANSI colour escapes are dropped; TizenRT consoles are raw UART.
 *  4. predict_false() is not a TizenRT macro, so a local one is used.
 *  5. The watchpoint support, kasan_debugpoint(), the dynamic init hooks
 *     and __sanitizer_annotate_contiguous_container() are omitted. Nothing
 *     in TizenRT drives them, and the compiler does not emit references to
 *     them with the flag set this port uses.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdint.h>
#include <syslog.h>
#include <assert.h>
#include <debug.h>

#include <tinyara/mm/kasan.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Reporting primitive.
 *
 * lowsyslog() is used directly rather than lldbg(), because lldbg() is
 * compiled away unless the debug options happen to be enabled, and a report
 * that prints nothing before panicking is worse than useless. lowsyslog()
 * is safe from interrupt context and falls back to syslog() on a target
 * without CONFIG_ARCH_LOWPUTC.
 *
 * generic.c is included below and uses this macro, so it must be defined
 * first.
 */

#define kasan_alert(format, ...)	\
	lowsyslog(LOG_ERR, "KASAN: " format, ##__VA_ARGS__)

/* TizenRT has no predict_false(). Note that inline_function in TizenRT is a
 * bare attribute rather than a full storage class as it is upstream, so it
 * cannot be used as a prefix here.
 */

#define kasan_predict_false(x)		__builtin_expect(!!(x), 0)

#ifdef CONFIG_MM_KASAN_GENERIC
#include "generic.c"
#else
/* MM_KASAN_NONE: the entry points are still emitted and called, so that the
 * cost of instrumentation can be measured on its own, but nothing is ever
 * reported and no shadow exists to dump.
 */

#define kasan_is_poisoned(addr, size)		false

/* The bit argument is consumed so that a caller which declares a variable
 * solely to pass its address here does not trip -Wunused-variable.
 */

#define kasan_mem_to_shadow(addr, size, bit)	((void)(bit), NULL)
#endif

#define KASAN_INIT_VALUE		0xcafe

/* Number of bytes shown either side of the faulting address, and the number
 * of bytes per dump line.
 */

#define KASAN_DUMP_BYTES		64
#define KASAN_DUMP_PER_LINE		16

/* Enough for the address, the separator and KASAN_DUMP_PER_LINE two digit
 * bytes with a space and a bracket allowance each.
 */

#define KASAN_DUMP_LINE_MAX		(KASAN_DUMP_PER_LINE * 4 + 8)

#ifdef CONFIG_MM_KASAN_DISABLE_READ_PANIC
#define MM_KASAN_DISABLE_READ_PANIC	1
#else
#define MM_KASAN_DISABLE_READ_PANIC	0
#endif

#ifdef CONFIG_MM_KASAN_DISABLE_WRITE_PANIC
#define MM_KASAN_DISABLE_WRITE_PANIC	1
#else
#define MM_KASAN_DISABLE_WRITE_PANIC	0
#endif

#ifdef CONFIG_MM_KASAN_INSTRUMENT

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The compiler emits a call to one of these for every instrumented access.
 * The _noabort forms are used when the sanitizer is built in recovering
 * mode, which is the default for kernel-address; the plain forms are used
 * with -fno-sanitize-recover=kernel-address. Both are defined so that the
 * build does not depend on which of the two the toolchain selects.
 */

#define DEFINE_ASAN_LOAD_STORE(size)					\
	void __asan_report_load##size##_noabort(FAR void *addr)		\
	{								\
		kasan_report(addr, size, false, __builtin_return_address(0)); \
	}								\
	void __asan_report_store##size##_noabort(FAR void *addr)	\
	{								\
		kasan_report(addr, size, true, __builtin_return_address(0)); \
	}								\
	void __asan_load##size##_noabort(FAR void *addr)		\
	{								\
		kasan_check_report(addr, size, false, __builtin_return_address(0)); \
	}								\
	void __asan_store##size##_noabort(FAR void *addr)		\
	{								\
		kasan_check_report(addr, size, true, __builtin_return_address(0)); \
	}								\
	void __asan_load##size(FAR void *addr)				\
	{								\
		kasan_check_report(addr, size, false, __builtin_return_address(0)); \
	}								\
	void __asan_store##size(FAR void *addr)				\
	{								\
		kasan_check_report(addr, size, true, __builtin_return_address(0)); \
	}

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Checking is off until this holds KASAN_INIT_VALUE. Instrumented code runs
 * from the first instruction of the image, long before any heap exists, so
 * this gate is what makes an image wide -fsanitize=kernel-address safe
 * without excluding the whole boot path from instrumentation.
 */

static uint32_t g_region_init;

static const char g_kasan_hex[] = "0123456789abcdef";

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kasan_show_memory
 *
 * Description:
 *   Dump the bytes around the faulting address, marking the poisoned ones.
 *
 *   Bytes which fall outside every registered region are shown as "..".
 *   They are never dereferenced: on a target without an MMU a read of an
 *   unmapped address would fault again from inside the report.
 *
 ****************************************************************************/

static void kasan_show_memory(FAR const uint8_t *addr, size_t size)
{
	FAR const uint8_t *start;
	FAR const uint8_t *p;
	char buffer[KASAN_DUMP_LINE_MAX];
	unsigned int bit;
	int i;
	int n;

	/* Round down to a line boundary and step back, without letting the
	 * pointer wrap when the faulting address is near zero.
	 */

	start = (FAR const uint8_t *)(((uintptr_t)addr) & ~(uintptr_t)(KASAN_DUMP_PER_LINE - 1));
	if ((uintptr_t)start >= KASAN_DUMP_BYTES) {
		start -= KASAN_DUMP_BYTES;
	} else {
		start = NULL;
	}

	kasan_alert("memory around the faulty address:\n");

	for (p = start; p < start + 2 * KASAN_DUMP_BYTES; p += KASAN_DUMP_PER_LINE) {
		n = 0;

		for (i = 0; i < KASAN_DUMP_PER_LINE; i++) {
			FAR const uint8_t *q = p + i;

			/* Mark the faulting range with brackets so it can be picked
			 * out of the line without counting columns.
			 */

			buffer[n++] = (q == addr) ? '[' : ((q == addr + size) ? ']' : ' ');

			if (kasan_mem_to_shadow(q, 1, &bit) == NULL) {
				/* Not covered by any region: do not touch it. */

				buffer[n++] = '.';
				buffer[n++] = '.';
			} else {
				buffer[n++] = g_kasan_hex[(*q >> 4) & 0xf];
				buffer[n++] = g_kasan_hex[*q & 0xf];
			}
		}

		buffer[n] = '\0';
		kasan_alert("%p:%s\n", p, buffer);
	}
}

/****************************************************************************
 * Name: kasan_report
 *
 * Description:
 *   Print the report and, unless the corresponding panic has been disabled,
 *   assert so that the crash dump is taken at the faulting instruction.
 *
 ****************************************************************************/

static void kasan_report(FAR const void *addr, size_t size, bool is_write, FAR void *return_address)
{
	bool dump_only = (is_write && MM_KASAN_DISABLE_WRITE_PANIC) || (!is_write && MM_KASAN_DISABLE_READ_PANIC);

	/* Stop checking before printing. The report path itself walks memory,
	 * and a second report from inside the first would recurse.
	 */

	kasan_stop();

	kasan_alert("invalid %s of size %zu at address %p\n", is_write ? "write" : "read", size, addr);
	kasan_alert("detected from %p\n", return_address);

	kasan_show_memory(addr, size);

	if (dump_only) {
#ifdef CONFIG_FRAME_POINTER
		/* dump_stack() is only declared when frame pointers are kept.
		 * Without them there is no backtrace to give, and the caller
		 * address printed above is all the context available.
		 */

		dump_stack();
#endif

		/* Resume checking: the caller has chosen to carry on running. */

		kasan_start();
	} else {
		PANIC();
	}
}

/****************************************************************************
 * Name: kasan_check_report
 *
 * Description:
 *   The hot path. Called from every instrumented load and store, so the
 *   ordering of the tests matters: the gate is checked first and is false
 *   for the whole of early boot.
 *
 ****************************************************************************/

static inline void kasan_check_report(FAR const void *addr, size_t size, bool is_write, FAR void *return_address)
{
	if (kasan_predict_false(size == 0 || g_region_init != KASAN_INIT_VALUE)) {
		return;
	}
#ifndef CONFIG_MM_KASAN_DISABLE_NULL_POINTER_CHECK
	if (kasan_predict_false(addr == NULL)) {
		kasan_report(addr, size, is_write, return_address);
		return;
	}
#endif

	if (kasan_predict_false(kasan_is_poisoned(addr, size))) {
		kasan_report(addr, size, is_write, return_address);
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kasan_start
 ****************************************************************************/

void kasan_start(void)
{
	g_region_init = KASAN_INIT_VALUE;
}

/****************************************************************************
 * Name: kasan_stop
 ****************************************************************************/

void kasan_stop(void)
{
	g_region_init = 0;
}

/****************************************************************************
 * Name: __asan_handle_no_return
 *
 * Description:
 *   Emitted before a call to a noreturn function. With stack instrumentation
 *   disabled there is no fake stack to unpoison, so there is nothing to do.
 *
 ****************************************************************************/

void __asan_handle_no_return(void)
{
}

/****************************************************************************
 * Name: __asan_register_globals / __asan_unregister_globals
 *
 * Description:
 *   Only referenced when the image is built with asan-globals=1, which this
 *   port does not do. Defined so that turning the parameter on for an
 *   experiment links rather than failing, with globals simply unchecked.
 *
 ****************************************************************************/

void __asan_register_globals(FAR void *ptr, size_t size)
{
	(void)ptr;
	(void)size;
}

void __asan_unregister_globals(FAR void *ptr, size_t size)
{
	(void)ptr;
	(void)size;
}

/****************************************************************************
 * Name: __asan_loadN / __asan_storeN and the fixed size variants
 ****************************************************************************/

void __asan_report_load_n_noabort(FAR void *addr, size_t size)
{
	kasan_report(addr, size, false, __builtin_return_address(0));
}

void __asan_report_store_n_noabort(FAR void *addr, size_t size)
{
	kasan_report(addr, size, true, __builtin_return_address(0));
}

void __asan_loadN_noabort(FAR void *addr, size_t size)
{
	kasan_check_report(addr, size, false, __builtin_return_address(0));
}

void __asan_storeN_noabort(FAR void *addr, size_t size)
{
	kasan_check_report(addr, size, true, __builtin_return_address(0));
}

void __asan_loadN(FAR void *addr, size_t size)
{
	kasan_check_report(addr, size, false, __builtin_return_address(0));
}

void __asan_storeN(FAR void *addr, size_t size)
{
	kasan_check_report(addr, size, true, __builtin_return_address(0));
}

DEFINE_ASAN_LOAD_STORE(1)
DEFINE_ASAN_LOAD_STORE(2)
DEFINE_ASAN_LOAD_STORE(4)
DEFINE_ASAN_LOAD_STORE(8)
DEFINE_ASAN_LOAD_STORE(16)

#endif							/* CONFIG_MM_KASAN_INSTRUMENT */
