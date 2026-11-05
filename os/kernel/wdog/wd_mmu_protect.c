/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
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
/************************************************************************
 * kernel/wdog/wd_mmu_protect.c
 *
 *   Write access to the watchdog data page is normally denied by the access
 *   permission bits of the single second-level page-table descriptor that
 *   covers it (g_wdog_data: the watchdog pool, both list heads and the free
 *   counter - see struct wdog_data_s).  Code that must modify watchdog data
 *   brackets the write with wd_mmu_write_begin() / wd_mmu_write_end():
 *
 *     - the first caller in the system rewrites that one descriptor so that
 *       writing is permitted; nested callers only bump a counter;
 *     - the last caller puts the descriptor back to read-only and has the
 *       address invalidated in the TLB of both cores, so a core that is not
 *       inside a write window faults on its next write instead of silently
 *       modifying watchdog data.
 *
 *   Every caller holds a critical section across the bracket, and the
 *   descriptor is shared by both cores, so one nesting counter describes the
 *   state of the whole system.  (The per-CPU counter the old DACR version
 *   needed was correct only because DACR itself is per-CPU: with page-table
 *   permissions, a per-CPU counter lets the core that finishes first close
 *   the window underneath a core that still has it open.)
 *
 *   Failures are never silent: a descriptor that cannot be flipped is
 *   reported and counted, and the protection is left as it was.  If opening
 *   the window fails the page stays read-only, so the write faults loudly
 *   instead of going through untracked.

 *
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <tinyara/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/irq.h>
#include <tinyara/mmu.h>

#include "wdog/wdog.h"

#ifdef CONFIG_WDOG_MMU_PROTECT

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

#define WDOG_PAGE_SHIFT  (12)
#define WDOG_PAGE_SIZE   (1 << WDOG_PAGE_SHIFT)

/* Linker-provided boundaries of .bss, used to check that the whole watchdog
 * data page really sits inside the zeroed image section (design §7, F-02). */

extern char _sbss[];
extern char _ebss[];

/************************************************************************
 * Private Data
 ************************************************************************/

/* Page base of the watchdog data; set once by wd_mmu_protect_init(). */

static uintptr_t g_wdog_page;

/* The two descriptors of that page: the one the kernel mapped the region
 * with (writable) and the one it is kept at rest behind (read-only).  Both
 * are read back from the live page table instead of being assembled here, so
 * the memory type, the shareability and the execute-never attribute of the
 * page are never touched by the protection - only its permissions change. */

static uint32_t g_wdog_pte_rw;
static uint32_t g_wdog_pte_ro;

static volatile bool g_wdog_ready;		/* protection is installed */
static volatile int g_wdog_nest;	/* depth of the write window, 0 = closed */
static volatile uint8_t g_wdog_owner = 0xff;	/* CPU that opened the window */
static uint32_t g_wdog_failures;	/* descriptor flips that failed */
static uint32_t g_wdog_flips;		/* descriptor flips that succeeded */

/************************************************************************
 * Private Functions
 ************************************************************************/

/* Effective access permission of the page: is it writable?  In the short
 * descriptor format a page is writable only when AP[2] is clear and AP[0] is
 * set (AP = 1 or 3); every value with AP[2] set is read-only, and AP = 0
 * faults on any access. */

static bool wd_ap_writable(uint8_t ap)
{
	return ((ap & 2) == 0) && ((ap & 1) != 0);
}

/* Install one of the two saved descriptors of the watchdog page. */

static int wd_mmu_install(uint32_t pte, const char *what)
{
	int status;

	if (pte == 0) {
		g_wdog_failures++;
		lldbg("WDOG_MMU: %s - no descriptor saved\n", what);
		return -EINVAL;
	}

	status = mmu_restore_page_pte((uint32_t)g_wdog_page, pte);
	if (status != OK) {
		g_wdog_failures++;
		lldbg("WDOG_MMU: %s failed (%d)\n", what, status);
	} else {
		g_wdog_flips++;
	}

	return status;
}

static inline uint8_t wd_this_cpu(void)
{
#ifdef CONFIG_SMP
	return (uint8_t)up_cpu_index();
#else
	return 0;
#endif
}

/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: wd_mmu_protect_init
 *
 * Description:
 *   Protect the watchdog data page.  Called once, from wd_initialize(),
 *   before any watchdog is created or started.
 *
 *   The page holding g_wdog_data is mapped through the second level (its 1MB
 *   section is split, and the table is claimed permanently - this protection
 *   is never undone), the writable descriptor is saved, and the page is left
 *   read-only.  If any step fails the page is left exactly as the kernel
 *   mapped it and the error is returned: a subsystem that cannot be
 *   protected cleanly must not run half protected, because a window opened
 *   and never closed is worse than no protection at all.
 *
 * Returned Value:
 *   OK on success, otherwise a negated errno.
 *
 ************************************************************************/

int wd_mmu_protect_init(void)
{
	uintptr_t start = (uintptr_t)&g_wdog_data;
	uintptr_t end = start + sizeof(g_wdog_data);
	uint32_t pte = 0;
	uint8_t ap = 0xff;
	irqstate_t flags;
	int status;

	if (g_wdog_ready) {
		return OK;
	}

	/* Everything the subsystem keeps in RAM has to sit in one page: the
	 * protection works on that page and on nothing else.  The linker
	 * script 4KB-aligns the section and ASSERTs its size, and the header
	 * bounds it at compile time - check again, because silently losing
	 * that exclusivity would put unrelated .bss data behind the same
	 * read-only permission. */

	if (((start & (WDOG_PAGE_SIZE - 1)) != 0) || ((end - start) > WDOG_PAGE_SIZE)) {
		lldbg("WDOG_MMU: data 0x%08x..0x%08x does not fit one 4KB page\n",
		      (unsigned int)start, (unsigned int)end);
		return -EINVAL;
	}

	/* Both ends must sit inside .bss: the guarantee this protection rests
	 * on is that the page holds nothing but watchdog data, which is only
	 * true while it is part of the zeroed image section. */

	if (start < (uintptr_t)_sbss || end > (uintptr_t)_ebss) {
		lldbg("WDOG_MMU: data 0x%08x..0x%08x outside .bss"
		      " 0x%08x..0x%08x\n", (unsigned int)start,
		      (unsigned int)end, (unsigned int)_sbss,
		      (unsigned int)_ebss);
		return -EINVAL;
	}

	/* Hardening: a MANAGER domain makes the hardware ignore the AP
	 * bits, so the page would look protected while every write sailed
	 * through untouched.  Refuse instead of installing protection that
	 * cannot ever fault. */

	if (mmu_domain_manager_seen()) {
		lldbg("WDOG_MMU: domain is MANAGER, refusing to install vacuous protection\n");
		return -EPERM;
	}

	/* Map the region through the second level and keep that table: this
	 * protection lasts as long the system does. */

	status = mmu_protect_page_reserve((uint32_t)start, "wdog_data");
	if (status != OK) {
		lldbg("WDOG_MMU: cannot page-map 0x%08x (%d)\n",
		      (unsigned int)start, status);
		return status;
	}

	g_wdog_page = start;

	flags = enter_critical_section();

	/* What the kernel mapped the page with, and whether that really was
	 * writable: a page that is not writable to begin with cannot be put
	 * back later, so the protection is not installed on top of it. */

	g_wdog_pte_rw = mmu_save_page_pte((uint32_t)start);

	if ((g_wdog_pte_rw == 0) ||
			(mmu_get_page_pte_raw((uint32_t)start, &pte, &ap) != OK) ||
			!wd_ap_writable(ap)) {
		g_wdog_pte_rw = 0;
		leave_critical_section(flags);
		lldbg("WDOG_MMU: page 0x%08x is not writable (AP=%d)\n",
		      (unsigned int)start, (int)ap);
		return -EIO;
	}

	/* Close it */

	status = mmu_set_page_ro((uint32_t)start);

	if (status == OK) {
		if ((mmu_get_page_pte_raw((uint32_t)start, &pte, &ap) != OK) ||
				wd_ap_writable(ap)) {
			status = -EIO;
		}
	}

	if (status != OK) {
		(void)mmu_restore_page_pte((uint32_t)start, g_wdog_pte_rw);
		g_wdog_pte_ro = 0;
		g_wdog_pte_rw = 0;
		g_wdog_failures++;
		leave_critical_section(flags);
		lldbg("WDOG_MMU: protecting 0x%08x failed (%d)\n",
		      (unsigned int)start, status);
		return status;
	}

	g_wdog_pte_ro = mmu_save_page_pte((uint32_t)start);
	g_wdog_nest = 0;
	g_wdog_owner = 0xff;
	g_wdog_ready = true;

	leave_critical_section(flags);

	lldbg("WDOG_MMU: page 0x%08x is read-only (pte 0x%08x AP=%d, writable pte 0x%08x)\n",
	      (unsigned int)start, (unsigned int)g_wdog_pte_ro, (int)ap,
	      (unsigned int)g_wdog_pte_rw);

	/* Publish the protected range so the generic test command
	 * MMU_PROTECT_GET_INFO lists it next to the pool-owned regions.  The
	 * page stays owned by the watchdog subsystem: the registry entry only
	 * advertises it, no other owner may map or restore it. */

#ifdef CONFIG_MMU_FAULT_REPORT
	mmu_protect_region_register("wdog_data", start, WDOG_PAGE_SIZE);
#endif

	return OK;
}

/************************************************************************
 * Name: wd_mmu_write_begin
 *
 * Description:
 *   Open the write window on the watchdog data page.  The first caller
 *   rewrites the descriptor of that one page so writes are permitted; a
 *   nested call (a watchdog callback that calls wd_start(), for instance)
 *   only makes the window deeper.
 *
 *   Callers must already hold a critical section, as every caller in the
 *   subsystem does: it is what keeps the two cores out of the window at the
 *   same time, and it is what makes the single nesting counter sound.
 *
 * Returned Value:
 *   OK, or a negated errno if the page could not be made writable.  The
 *   protection is still in force in that case, so the write the caller wants
 *   to do will fault: loud, and better than a write that goes through with
 *   the bookkeeping saying otherwise.
 *
 ************************************************************************/

int wd_mmu_write_begin(void)
{
	int status;

	if (!g_wdog_ready) {
		return OK;
	}

	if (g_wdog_nest > 0) {
		/* The window is already open, and only the core that opened it
		 * may nest inside it.  The descriptor is one piece of shared
		 * memory: a second core nesting here would be counted against a
		 * window it does not own, and the owner would then find the
		 * counter above zero when it closes and would leave the page
		 * writable behind it.  Every caller holds a critical section
		 * from here on, so this is a tripwire and not a case to handle:
		 * the answer is a refusal the caller can act on, never a
		 * silently shared window.
		 */

		if (g_wdog_owner != wd_this_cpu()) {
			g_wdog_failures++;
			lldbg("WDOG_MMU: cpu%u nested into a window owned by cpu%u\n",
			      (unsigned int)wd_this_cpu(), (unsigned int)g_wdog_owner);
			return -EBUSY;
		}

		g_wdog_nest++;
		return OK;
	}

	status = wd_mmu_install(g_wdog_pte_rw, "write window open");
	if (status != OK) {
		return status;
	}

	g_wdog_nest = 1;
	g_wdog_owner = wd_this_cpu();

	return OK;
}

/************************************************************************
 * Name: wd_mmu_write_end
 *
 * Description:
 *   Close one level of the write window.  Only the last level actually
 *   rewrites the descriptor, and mmu_restore_page_pte() invalidates the
 *   address in the TLB of both cores as it does so - without that second
 *   step the core that did not perform the write would keep a writable
 *   translation of the page and no fault would ever be reported.
 *
 * Returned Value:
 *   OK, or a negated errno if the page could not be made read-only again
 *   (which is counted, and left to the fault it will cause).
 *
 ************************************************************************/

int wd_mmu_write_end(void)
{
	if (!g_wdog_ready) {
		return OK;
	}

	if (g_wdog_nest == 0) {
		/* Closing a window that was never opened: the caller is out of
		 * step with itself.  Say so instead of silently "re-protecting"
		 * something that is already protected. */

		g_wdog_failures++;
		lldbg("WDOG_MMU: write window closed while already closed (cpu%u)\n",
		      (unsigned int)wd_this_cpu());
		return -EINVAL;
	}

	/* Only the core that opened the window may close it, for the same reason
	 * it is the only core that may nest inside it: the counter and the
	 * descriptor are one piece of shared memory, so letting a core spend
	 * another core's reference would put the page behind its read-only
	 * descriptor while that core is still writing.  Nothing is rewritten
	 * here; the mismatch is counted and refused.
	 */

	if (g_wdog_owner != wd_this_cpu()) {
		g_wdog_failures++;
		lldbg("WDOG_MMU: cpu%u closed a window owned by cpu%u\n",
		      (unsigned int)wd_this_cpu(), (unsigned int)g_wdog_owner);
		return -EPERM;
	}

	if (g_wdog_nest > 1) {
		g_wdog_nest--;
		return OK;
	}

	g_wdog_nest = 0;
	g_wdog_owner = 0xff;

	return wd_mmu_install(g_wdog_pte_ro, "write window close");
}

/************************************************************************
 * Name: wd_mmu_is_protected
 *
 * Description:
 *   True when the watchdog data page currently carries its read-only
 *   descriptor: the protection is installed and no write window is open.
 *   The answer comes from the live descriptor and not from a cached flag, so
 *   a descriptor flip that failed can never report "protected".
 *
 ****************************************************************************/

bool wd_mmu_is_protected(void)
{
	uint32_t pte;
	uint8_t ap;

	if (!g_wdog_ready) {
		return false;
	}

	if (mmu_get_page_pte_raw((uint32_t)g_wdog_page, &pte, &ap) != OK) {
		return false;
	}

	return !wd_ap_writable(ap);
}

/************************************************************************
 * Name: wd_mmu_get_window_state
 *
 * Description:
 *   Snapshot of the write window, for the data abort report and for the
 *   protection tests:
 *
 *     open  - a write to the watchdog data page is currently allowed;
 *     cpu   - which core opened it (0xff when closed);
 *     depth - how many brackets are nested.
 *
 *   Read without locking on purpose: the data abort path ends up here, and
 *   the faulting core may itself be holding the critical section lock, so
 *   taking one would deadlock the report that exists to explain the fault.
 *   The values are read once each; they only ever have to be consistent
 *   enough to tell "window open" from "window closed".
 *
 ****************************************************************************/

void wd_mmu_get_window_state(FAR bool *open, FAR uint8_t *cpu, FAR uint8_t *depth)
{
	int nest = g_wdog_nest;

	if (open != NULL) {
		*open = (nest > 0);
	}

	if (cpu != NULL) {
		*cpu = g_wdog_owner;
	}

	if (depth != NULL) {
		*depth = (nest > 0xff) ? 0xff : (uint8_t)((nest < 0) ? 0 : nest);
	}
}

/************************************************************************
 * Name: wd_mmu_get_failures
 *
 * Description:
 *   How many times the protection refused or failed to change the descriptor
 *   of the watchdog data page: a flip that the MMU did not accept, a window
 *   closed while already closed, and - the one that counts for SMP - a window
 *   that a core tried to share with the core that owns it.  Every one of
 *   those is silent by design on the write path, because a watchdog callback
 *   must not be lost to a bookkeeping error, so this is the only way to see
 *   that any of them happened.
 *
 *   It is a running total since boot and it never decreases, which is what
 *   makes it usable as a test oracle: a scenario that leaves the count where
 *   it found it proved that no core ever stepped on another core's window.
 *
 ****************************************************************************/

uint32_t wd_mmu_get_failures(void)
{
	return g_wdog_failures;
}

/*****************************************************************************
 * Name: wd_mmu_get_flips
 *
 * Description:
 *   How many times the descriptor of the watchdog data page was successfully
 *   written since boot.  A nesting pair is supposed to cost exactly two
 *   flips - one to make the page writable, one to put it back - and nothing
 *   in between, so this is what lets a test say "the 998 inner calls never
 *   touched the page table" instead of "nothing crashed".
 *
 *****************************************************************************/

uint32_t wd_mmu_get_flips(void)
{
	return g_wdog_flips;
}

#endif							/* CONFIG_WDOG_MMU_PROTECT */
