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
 * os/mm/kasan/generic.c
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
 * Derived from Apache NuttX mm/kasan/generic.c at commit b4aa94d.
 *
 * Differences from the original, all deliberate:
 *
 *  1. The address tag helpers and kasan_bypass() are dropped. They exist
 *     upstream only to serve the software and hardware tag modes, which
 *     need ARMv8 top byte ignore or ARMv8.5 MTE. No TizenRT target has
 *     either.
 *  2. kasan_mem_to_shadow() refuses a range which starts inside a region
 *     but ends past it, instead of only asserting. TizenRT's DEBUGASSERT
 *     compiles away without CONFIG_DEBUG, and without the check the shadow
 *     index would run past the shadow array and corrupt the heap.
 *  3. kasan_register() validates the region table bound and the minimum
 *     region size unconditionally, for the same reason, and declines to
 *     register rather than overflowing g_region[]. It also requires the
 *     region base to be granule aligned, which point 4 depends on.
 *  4. kasan_is_poisoned() accounts for the offset of an access within its
 *     first granule. The original derives the granule count from the size
 *     alone, so any unaligned access that straddles a granule boundary is
 *     checked against the first granule only and an out of bounds access
 *     lands in the blind spot. A sweep of every offset and length across a
 *     block boundary missed 28 of them before this change and none after.
 *
 * The last two are corrections rather than adaptations, and should be
 * offered back upstream.
 *
 * This file is included by hook.c rather than compiled on its own, so that
 * kasan_is_poisoned() inlines into the instrumentation entry points. It
 * therefore relies on hook.c having already defined kasan_alert(), and is
 * not built directly by Make.defs.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdint.h>
#include <string.h>
#include <assert.h>

#include <tinyara/mm/kasan.h>
#include <tinyara/spinlock.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The shadow map holds one bit per machine word: set means poisoned. On a
 * 32-bit target that is one shadow byte per 32 bytes of memory, an overhead
 * of 1/32, which is four times cheaper than the one-byte-per-eight-bytes
 * encoding used by Linux and by user space AddressSanitizer.
 *
 * The cost of the compact encoding is that a poisoned granule cannot record
 * why it is poisoned, and that detection is at word rather than byte
 * granularity.
 */

#define KASAN_BYTES_PER_WORD		(sizeof(uintptr_t))
#define KASAN_BITS_PER_WORD		(KASAN_BYTES_PER_WORD * 8)

#define KASAN_FIRST_WORD_MASK(start)	\
	(UINTPTR_MAX << ((start) & (KASAN_BITS_PER_WORD - 1)))
#define KASAN_LAST_WORD_MASK(end)	\
	(UINTPTR_MAX >> (-(end) & (KASAN_BITS_PER_WORD - 1)))

#define KASAN_SHADOW_SCALE		(sizeof(uintptr_t))

#define KASAN_SHADOW_SIZE(size)		\
	(KASAN_BYTES_PER_WORD * ((size) / KASAN_SHADOW_SCALE / KASAN_BITS_PER_WORD))
#define KASAN_REGION_SIZE(size)		\
	(sizeof(struct kasan_region_s) + KASAN_SHADOW_SIZE(size))

/* KASAN_SHADOW_SIZE() truncates, so a region whose size is not a multiple of
 * KASAN_SHADOW_SCALE * KASAN_BITS_PER_WORD needs one shadow word more than
 * the macro yields. That word is the shadow[1] member already counted by
 * sizeof(struct kasan_region_s), which covers a remainder of up to
 * KASAN_SHADOW_SCALE * KASAN_BITS_PER_WORD - 1 bytes. Do not "simplify" the
 * struct to a zero length array without also rounding the shadow size up.
 */

/* A region smaller than this cannot usefully carry both its shadow and a
 * heap, so it is left unregistered rather than being carved to nothing.
 */

#define KASAN_MIN_REGION_SIZE		(4 * KASAN_SHADOW_SCALE * KASAN_BITS_PER_WORD)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct kasan_region_s {
	uintptr_t begin;
	uintptr_t end;
	uintptr_t shadow[1];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct kasan_region_s *g_region[CONFIG_MM_KASAN_REGIONS];
static size_t g_region_count;
static spinlock_t g_lock = SP_UNLOCKED;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kasan_mem_to_shadow
 *
 * Description:
 *   Map an address range to the shadow word and bit which describe its
 *   first granule.
 *
 * Returned Value:
 *   A pointer to the shadow word, with *bit set to the bit index of the
 *   first granule, or NULL when the range is not covered by any registered
 *   region. A range which begins inside a region but extends past its end
 *   is treated as not covered, because the shadow cannot describe it.
 *
 ****************************************************************************/

static inline FAR uintptr_t *kasan_mem_to_shadow(FAR const void *ptr, size_t size, FAR unsigned int *bit)
{
	uintptr_t addr = (uintptr_t)ptr;
	size_t i;

	for (i = 0; i < g_region_count; i++) {
		if (addr >= g_region[i]->begin && addr < g_region[i]->end) {
			if (addr + size > g_region[i]->end) {
				/* Straddles the end of the region. Indexing the shadow
				 * here would run off the end of the shadow array, so
				 * refuse the lookup. The access is out of bounds of the
				 * whole region, which is a wild pointer rather than a
				 * heap overflow, and is left to the MPU to catch.
				 */

				DEBUGASSERT(addr + size <= g_region[i]->end);
				return NULL;
			}

			addr -= g_region[i]->begin;
			addr /= KASAN_SHADOW_SCALE;
			*bit = addr % KASAN_BITS_PER_WORD;
			return &g_region[i]->shadow[addr / KASAN_BITS_PER_WORD];
		}
	}

	return NULL;
}

/****************************************************************************
 * Name: kasan_is_poisoned
 *
 * Description:
 *   Test whether any granule of the range is marked inaccessible.
 *
 ****************************************************************************/

static inline bool kasan_is_poisoned(FAR const void *addr, size_t size)
{
	FAR uintptr_t *p;
	unsigned int bit;
	unsigned int nbit;
	uintptr_t mask;
	size_t off;

	p = kasan_mem_to_shadow(addr, size, &bit);
	if (p == NULL) {
		return false;
	}

	/* Offset of the access within its first granule. Region begins are
	 * granule aligned (see kasan_register), so this can be taken from the
	 * address alone.
	 *
	 * The original tests only the size here, which silently misses every
	 * unaligned access that straddles a granule boundary: a 4 byte store
	 * at offset 2 spans two granules but is checked against the bit of the
	 * first one only. Those are precisely the accesses that run off the
	 * end of a block, so the offset has to be part of both the fast path
	 * test and the granule count below.
	 */

	off = (uintptr_t)addr & (KASAN_SHADOW_SCALE - 1);

	/* An access contained within one granule touches a single bit. This is
	 * the overwhelmingly common case, so keep it branch free.
	 */

	if (off + size <= KASAN_SHADOW_SCALE) {
		return ((*p >> bit) & 1);
	}

	nbit = KASAN_BITS_PER_WORD - bit % KASAN_BITS_PER_WORD;
	mask = KASAN_FIRST_WORD_MASK(bit);

	size = (off + size + KASAN_SHADOW_SCALE - 1) / KASAN_SHADOW_SCALE;

	while (size >= nbit) {
		if ((*p++ & mask) != 0) {
			return true;
		}

		bit += nbit;
		size -= nbit;

		nbit = KASAN_BITS_PER_WORD;
		mask = UINTPTR_MAX;
	}

	if (size) {
		mask &= KASAN_LAST_WORD_MASK(bit + size);
		if ((*p & mask) != 0) {
			return true;
		}
	}

	return false;
}

/****************************************************************************
 * Name: kasan_set_poison
 *
 * Description:
 *   Set or clear the shadow bits describing a range.
 *
 *   The size is truncated to whole granules. Every caller in the memory
 *   manager passes a granule aligned size, because MM_ALIGN_UP() rounds to
 *   MM_MIN_CHUNK, which is a multiple of KASAN_SHADOW_SCALE.
 *
 ****************************************************************************/

static void kasan_set_poison(FAR const void *addr, size_t size, bool poisoned)
{
	FAR uintptr_t *p;
	irqstate_t flags;
	unsigned int bit;
	unsigned int nbit;
	uintptr_t mask;

	p = kasan_mem_to_shadow(addr, size, &bit);
	if (p == NULL) {
		return;
	}

	nbit = KASAN_BITS_PER_WORD - bit % KASAN_BITS_PER_WORD;
	mask = KASAN_FIRST_WORD_MASK(bit);
	size /= KASAN_SHADOW_SCALE;

	flags = spin_lock_irqsave(&g_lock);

	while (size >= nbit) {
		if (poisoned) {
			*p++ |= mask;
		} else {
			*p++ &= ~mask;
		}

		bit += nbit;
		size -= nbit;

		nbit = KASAN_BITS_PER_WORD;
		mask = UINTPTR_MAX;
	}

	if (size) {
		mask &= KASAN_LAST_WORD_MASK(bit + size);
		if (poisoned) {
			*p |= mask;
		} else {
			*p &= ~mask;
		}
	}

	spin_unlock_irqrestore(&g_lock, flags);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: kasan_poison
 ****************************************************************************/

void kasan_poison(FAR const void *addr, size_t size)
{
	kasan_set_poison(addr, size, true);
}

/****************************************************************************
 * Name: kasan_unpoison
 ****************************************************************************/

FAR void *kasan_unpoison(FAR const void *addr, size_t size)
{
	kasan_set_poison(addr, size, false);
	return (FAR void *)addr;
}

/****************************************************************************
 * Name: kasan_register
 ****************************************************************************/

void kasan_register(FAR void *addr, FAR size_t *size)
{
	FAR struct kasan_region_s *region;
	irqstate_t flags;
	size_t i;

	/* A region which cannot hold a shadow and a worthwhile heap is better
	 * left uncovered than carved down to nothing.
	 */

	if (addr == NULL || *size < KASAN_MIN_REGION_SIZE) {
		kasan_alert("region %p size %zu too small to register\n", addr, *size);
		return;
	}

	/* kasan_is_poisoned() derives an access's offset within its granule
	 * from the address alone, which is only correct while every region
	 * begins on a granule boundary. Every caller passes a base that has
	 * been through MM_ALIGN_UP(), so this holds; refuse rather than
	 * silently mis-check if a new caller ever does not.
	 */

	if (((uintptr_t)addr & (KASAN_SHADOW_SCALE - 1)) != 0) {
		kasan_alert("region %p is not %zu byte aligned\n", addr, (size_t)KASAN_SHADOW_SCALE);
		return;
	}

	flags = spin_lock_irqsave(&g_lock);

	/* Checked unconditionally: DEBUGASSERT() is compiled out when
	 * CONFIG_DEBUG is off, and overflowing g_region[] would corrupt memory
	 * from inside the tool meant to detect corruption.
	 */

	if (g_region_count >= CONFIG_MM_KASAN_REGIONS) {
		spin_unlock_irqrestore(&g_lock, flags);
		kasan_alert("no free region slot for %p, raise CONFIG_MM_KASAN_REGIONS above %d\n", addr, CONFIG_MM_KASAN_REGIONS);
		return;
	}

	/* Two regions describing the same memory would each carve a shadow and
	 * the lookup would always find the first, leaving the second stale. That
	 * is far harder to diagnose than a refusal, so reject the overlap.
	 */

	for (i = 0; i < g_region_count; i++) {
		if ((uintptr_t)addr < g_region[i]->end && g_region[i]->begin < (uintptr_t)addr + *size) {
			spin_unlock_irqrestore(&g_lock, flags);
			kasan_alert("region %p size %zu overlaps the one at %p\n", addr, *size, (FAR void *)g_region[i]->begin);
			return;
		}
	}

	/* Place the descriptor and its shadow in the tail of the region. This
	 * is what keeps KASan free of any linker script requirement.
	 */

	region = (FAR struct kasan_region_s *)((FAR char *)addr + *size - KASAN_REGION_SIZE(*size));

	region->begin = (uintptr_t)addr;
	region->end = region->begin + *size;

	g_region[g_region_count++] = region;

	spin_unlock_irqrestore(&g_lock, flags);

	/* Enable checking, then poison the whole region. The allocator is
	 * responsible for unpoisoning each block as it is handed out.
	 */

	kasan_start();
	kasan_poison(addr, *size);

	/* Hand back the size that is left for the heap. region->end still spans
	 * the shadow, so a stray access into it is reported as poisoned.
	 */

	*size -= KASAN_REGION_SIZE(*size);
}

/****************************************************************************
 * Name: kasan_unregister
 ****************************************************************************/

void kasan_unregister(FAR void *addr)
{
	irqstate_t flags;
	size_t i;

	flags = spin_lock_irqsave(&g_lock);

	for (i = 0; i < g_region_count; i++) {
		if (g_region[i]->begin == (uintptr_t)addr) {
			g_region_count--;
			memmove(&g_region[i], &g_region[i + 1], (g_region_count - i) * sizeof(g_region[0]));
			spin_unlock_irqrestore(&g_lock, flags);

			/* No shadow is cleared here. The original clears the shadow
			 * after dropping the entry from the table, by which point the
			 * lookup can no longer resolve the address, so the call has no
			 * effect. It is omitted rather than reordered because the
			 * shadow lives inside the region being released and goes away
			 * with it.
			 */

			return;
		}
	}

	spin_unlock_irqrestore(&g_lock, flags);
}
