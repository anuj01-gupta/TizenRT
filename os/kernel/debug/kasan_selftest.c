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
 * kernel/debug/kasan_selftest.c
 *
 * Deliberate memory errors, used to demonstrate that the instrumentation is
 * live. A system with no memory errors and a system where instrumentation
 * was never emitted look exactly the same from the console, so a clean boot
 * proves nothing on its own. These cases are the other half of that check.
 *
 * This file is not under mm/. Everything there is compiled with
 * -fno-sanitize=kernel-address so that the sanitizer does not instrument
 * itself, which also means an access made from there is never checked. The
 * faulting accesses have to be made from code that is instrumented, and
 * kernel/ is.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/mm/mm.h>
#include <tinyara/mm/kasan.h>
#include <tinyara/kmalloc.h>

/* kmm_malloc() is the kernel allocator whenever CONFIG_MM_KERNEL_HEAP is
 * set, and the kernel heap is the one the allocator hooks register with
 * KASan. In a protected build without that option it becomes the user
 * allocator, whose heap is deliberately never registered, and every case
 * here would pass silently while proving nothing.
 */

#if defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_MM_KERNEL_HEAP)
#error "the KASan self test needs CONFIG_MM_KERNEL_HEAP in a protected build"
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define selftest_msg(format, ...) \
	lowsyslog(LOG_ERR, "KASAN-SELFTEST: " format, ##__VA_ARGS__)

/* How much of a block is actually accessible.
 *
 * mm_malloc() rounds the request up with MM_ALIGN_UP(size +
 * SIZEOF_MM_ALLOCNODE) and hands back the whole chunk; the KASan hook then
 * unpoisons everything past the node header. So the accessible payload is
 * normally larger than what was asked for, and this port reports an overflow
 * past the end of the chunk rather than past the end of the request.
 *
 * An overflow case therefore has to step past the payload. Writing at
 * request[0] + the requested size would usually land on a byte the allocator
 * legitimately made available, nothing would be reported, and the test would
 * read as a failure of the instrumentation rather than of the test.
 */

#define SELFTEST_PAYLOAD(req) \
	(MM_ALIGN_UP((req) + SIZEOF_MM_ALLOCNODE) - SIZEOF_MM_ALLOCNODE)

/* Large enough that the block does not come from the smallest bin, small
 * enough to be satisfied on a nearly full heap.
 */

#define SELFTEST_REQUEST	64

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Every access below goes through one of these.
 *
 * The volatile qualifier is doing real work in three separate places, and
 * removing any of them silently weakens the test rather than breaking it:
 *
 *  - it stops the optimiser discarding a store whose result is never read,
 *    and a read whose value is never used, which would remove the very
 *    access being tested;
 *  - it stops constant propagation reaching the offset, which would let the
 *    compiler prove the access is out of bounds and either warn under
 *    -Warray-bounds or fold it to a trap of its own;
 *  - reloading the pointer from g_selftest_ptr after the free is what keeps
 *    -Wuse-after-free quiet on that one case, without having to turn the
 *    warning off for the whole file. The compiler loses track of where the
 *    pointer came from, which is exactly the situation the runtime check
 *    exists to catch.
 */

static FAR uint8_t *volatile g_selftest_ptr;
static volatile size_t g_selftest_off;
static volatile uint8_t g_selftest_sink;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kasan_selftest
 *
 * Description:
 *   Run one case of the self test against the kernel heap.
 *
 ****************************************************************************/

int kasan_selftest(int testcase)
{
	FAR uint8_t *volatile p;
	size_t payload;
	bool freed = false;

	if (testcase < 0 || testcase >= KASAN_SELFTEST_MAX) {
		selftest_msg("no such case %d, expected 0 to %d\n", testcase, KASAN_SELFTEST_MAX - 1);
		return -EINVAL;
	}

	payload = SELFTEST_PAYLOAD(SELFTEST_REQUEST);

	p = (FAR uint8_t *)kmm_malloc(SELFTEST_REQUEST);
	if (p == NULL) {
		selftest_msg("no kernel heap for a %d byte block\n", SELFTEST_REQUEST);
		return -ENOMEM;
	}

	g_selftest_ptr = p;

	selftest_msg("block %p, requested %d bytes, %u accessible\n", (FAR void *)p, SELFTEST_REQUEST, (unsigned int)payload);

	switch (testcase) {
	case KASAN_SELFTEST_INBOUNDS:

		/* The negative control. The last accessible byte of the block must
		 * not be reported. If this case reports, the allocator hook is
		 * unpoisoning less than it hands out and every other result here is
		 * meaningless.
		 */

		selftest_msg("case %d: write at offset %u, inside the block\n", testcase, (unsigned int)(payload - 1));

		g_selftest_off = payload - 1;
		p[g_selftest_off] = 0x41;
		g_selftest_sink = p[g_selftest_off];

		kmm_free((FAR void *)p);
		g_selftest_ptr = NULL;

		selftest_msg("case %d: no report, as expected\n", testcase);
		return 0;

	case KASAN_SELFTEST_OVERFLOW_WRITE:
		selftest_msg("case %d: write at offset %u, one past the block\n", testcase, (unsigned int)payload);

		g_selftest_off = payload;
		p[g_selftest_off] = 0x41;
		break;

	case KASAN_SELFTEST_OVERFLOW_READ:

		/* Only reported when reads are instrumented. With
		 * CONFIG_MM_KASAN_DISABLE_READS_CHECK set this case is expected to
		 * pass silently, and that is the correct result for that
		 * configuration rather than a missed detection.
		 */

		selftest_msg("case %d: read at offset %u, one past the block\n", testcase, (unsigned int)payload);

		g_selftest_off = payload;
		g_selftest_sink = p[g_selftest_off];
		break;

	case KASAN_SELFTEST_UNDERFLOW:

		/* The bytes in front of the payload are the node header. The whole
		 * region is poisoned at registration and only the payload of a live
		 * block is unpoisoned, so the header is poisoned and a step back
		 * into it is reported.
		 */

		selftest_msg("case %d: write one byte in front of the block\n", testcase);

		g_selftest_off = 0;
		p[g_selftest_off - 1] = 0x41;
		break;

	case KASAN_SELFTEST_USE_AFTER_FREE:
		selftest_msg("case %d: write to the block after freeing it\n", testcase);

		kmm_free((FAR void *)p);
		freed = true;

		/* Reloaded, not reused. See the note on g_selftest_ptr above. */

		p = g_selftest_ptr;
		g_selftest_off = 0;
		p[g_selftest_off] = 0x41;
		break;

	case KASAN_SELFTEST_STRADDLE:

		/* A four byte store starting two bytes before the end. The shadow
		 * holds one bit per machine word, so this access begins in the last
		 * accessible word and ends in the first poisoned one.
		 *
		 * This is the case a shadow test that looks only at the size of an
		 * access misses, because the access still fits within one word when
		 * its offset inside that word is ignored. It is also the shape of a
		 * real overflow: the last write of a copy that ran two bytes long.
		 */

		selftest_msg("case %d: word write at offset %u, across the block end\n", testcase, (unsigned int)(payload - 2));

		g_selftest_off = payload - 2;
		*(FAR volatile uint32_t *)(p + g_selftest_off) = 0x41424344;
		break;

	default:

		/* Unreachable: the range was checked on entry. Present so that a
		 * case added to the enum without a case added here is a compiler
		 * diagnostic rather than a block leaked on the kernel heap.
		 */

		kmm_free((FAR void *)p);
		g_selftest_ptr = NULL;
		return -EINVAL;
	}

	/* Only reached when the access above was not reported.
	 *
	 * That is expected for the read case with reads not instrumented, and
	 * for any case when a panic has been disabled and execution carries on
	 * past the report. Anywhere else it means the access was not checked.
	 *
	 * The use after free case has already returned the block, so freeing it
	 * here as well would be a double free raised by this test rather than
	 * found by it.
	 */

	if (!freed) {
		kmm_free((FAR void *)g_selftest_ptr);
	}

	g_selftest_ptr = NULL;

	selftest_msg("case %d: returned without a panic\n", testcase);
	return 0;
}
