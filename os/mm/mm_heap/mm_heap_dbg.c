/****************************************************************************
 *
 * Copyright 2023 Samsung Electronics All Rights Reserved.
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
 * Included Files
 ****************************************************************************/
#include <tinyara/config.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>
#include <tinyara/sched.h>
#include <tinyara/mm/mm.h>
#include <tinyara/security_level.h>

/* mm_heap is compiled twice: into the kernel, and into libumm for the loadable
 * binaries.  Only the kernel copy is linked against the armv7-a MMU code, and
 * the user heap is never MMU-protected (same split as the R4 hooks in
 * tinyara/mm/mm.h).  Where the arch call is not linkable, nothing can carry an
 * AP=0 descriptor, so compile the check out instead of emitting a reference
 * nothing can resolve. */
#if defined(CONFIG_MMU_PAGE_PROTECT) && \
	(defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__))
#include <tinyara/mmu.h>
#else
#define mmu_range_is_no_access(a) (false)
#endif

/************************************************************************
 * Name: heap_dbg
 *
 * Description: Dump heap info debug logs without adding function name to start of log.
 ************************************************************************/
int heap_dbg(const char *fmt, ...)
{
	va_list ap;
	int ret;

	if (IS_SECURE_STATE()) {
		return 0;
	}

	va_start(ap, fmt);
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
	extern bool abort_mode;

	if (abort_mode) {
		ret = lowvsyslog(LOG_ERR, fmt, ap);
	} else {
		ret = vsyslog(LOG_ERR, fmt, ap);
	}
#else
	ret = vsyslog(LOG_ERR, fmt, ap);
#endif
	va_end(ap);

	return ret;
}

/************************************************************************
 * Name: mm_dump_heap_region
 *
 * Description: Print the hex value contents of heap from start to end address.
 ************************************************************************/
void mm_dump_heap_region(uint32_t start, uint32_t end)
{
	heap_dbg("#########################################################################################\n");
	heap_dbg("Dump heap: 0x%08x - 0x%08x\n", start, end);
	heap_dbg("#########################################################################################\n");
	while (start < end) {
		uint32_t *ptr = (uint32_t *)start;

		/* Plan risk R5: the data area of a freed chunk can be
		 * No-Access (TC-R4 locks it through the MMU).  Reading it
		 * here would raise a second Data Abort inside the reporter
		 * whose only job is to explain the first one, so ask the
		 * page table and say what was not read. */
		if (mmu_range_is_no_access(start)) {
			heap_dbg("%08x: <no-access: protected, contents not read>\n",
				 start);
			start = (start & ~0xfffU) + 0x1000;
			continue;
		}

		heap_dbg("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
			   start, ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5], ptr[6], ptr[7]);
		start += 32;
	}
	heap_dbg("#########################################################################################\n");
}

/************************************************************************
 * Name: mm_dump_heap_free_node_list
 *
 * Description: Print the hex value contents of heap free nodes.
 ************************************************************************/
void mm_dump_heap_free_node_list(struct mm_heap_s *heap)
{
	struct mm_freenode_s *node;
	heap_dbg("#########################################################################################\n");
	heap_dbg("Dump heap free node list\n");
	heap_dbg("[ndx], [HEAD]: [FREE NODES(SIZE)]\n");
	heap_dbg("#########################################################################################\n");
	for (uint8_t ndx = 0; ndx < MM_NNODES; ndx++) {
		heap_dbg("%3d, %08x:", ndx, &heap->mm_nodelist[ndx]);
		for (node = heap->mm_nodelist[ndx].flink; node; node = node->flink) {
			heap_dbg(" %08x(%d)", node, node->size);
		}
		heap_dbg("\n");

		if (heap->mm_nodelist[ndx].size != 0) {
			mfdbg("    Corrupted HEAD of free node link %08x\n", &heap->mm_nodelist[ndx]);
		}
	}
	heap_dbg("#########################################################################################\n");
}

/************************************************************************
 * Name: mm_dump_node
 *
 * Description: Print the information of heap node.
 ************************************************************************/
void mm_dump_node(struct mm_allocnode_s *node, char *node_type)
{
#if defined(CONFIG_DEBUG_MM_HEAPINFO)  && (CONFIG_TASK_NAME_SIZE > 0)
	char myname[CONFIG_TASK_NAME_SIZE + 1];
	char is_stack[9] = "'s stack";
#endif

	if (!node) {
		mfdbg("Unable to dump NULL node\n");
		return;
	}

	mfdbg("%s: addr = 0x%08x, type = %c, size = %u, preceding size = %u\n", node_type, node, node->preceding & MM_ALLOC_BIT ? 'A' : 'F', node->size, node->preceding & ~MM_ALLOC_BIT);

#ifdef CONFIG_DEBUG_MM_HEAPINFO
	pid_t pid = node->pid;
	if (pid < 0) {
		/* For stack allocated node, pid is negative value.
		 * To use the pid, change it to original positive value.
		 */
		pid = (-1) * pid;
	} else {
		is_stack[0] = '\0';
	}
#if CONFIG_TASK_NAME_SIZE > 0
	if (prctl(PR_GET_NAME_BYPID, myname, pid) == OK) {
		mfdbg("%s owner pid = %d (%s%s), allocated by code at addr = 0x%08x\n", node_type, node->pid, myname, is_stack, node->alloc_call_addr);
	} else {
		mfdbg("%s owner pid = %d (Exited Task%s), allocated by code at addr = 0x%08x\n", node_type, node->pid, is_stack, node->alloc_call_addr);
	}
#else
	mfdbg("%s owner pid = %d%s, allocated by code at addr = 0x%08x\n", node_type, node->pid, is_stack, node->alloc_call_addr);
#endif
#endif
}
