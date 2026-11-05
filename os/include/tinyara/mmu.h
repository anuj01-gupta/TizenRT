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

///@file tinyara/mmu.h

#ifndef __INCLUDE_TINYARA_MMU_H
#define __INCLUDE_TINYARA_MMU_H

/********************************************************************************
 * Included Files
 ********************************************************************************/

#include <tinyara/config.h>

#include <sys/types.h>
#include <stdint.h>

/********************************************************************************
 * Pre-processor Definitions
 ********************************************************************************/

#define SMALL_PAGE_SZ		(4096)	/* 4KB small page */
#define MMU_ALIGNMENT_BYTES	SMALL_PAGE_SZ
#define MMU_ALIGN_UP(a)		(((a) + MMU_ALIGNMENT_BYTES - 1) & ~(MMU_ALIGNMENT_BYTES - 1))

#ifdef CONFIG_OPTIMIZE_APP_RELOAD_TIME
/* Separate three MPU regions (text, ro and rw) to optimize reloading time */
#define NUM_APP_REGIONS     3
#else
/* Just a MPU region for all of section data */
#define NUM_APP_REGIONS     1
#endif


/********************************************************************************
 * Public Function Prototypes
 ********************************************************************************/
#ifdef CONFIG_APP_BINARY_SEPARATION
uint32_t *mmu_get_os_l1_pgtbl(void);
uint32_t *mmu_allocate_app_l1_pgtbl(int app_id);
uint32_t *mmu_allocate_app_l2_pgtbl(int app_id, int l2_idx);
void mmu_update_app_l1_pgtbl_ospgtbl(uint32_t *app_l1_pgtbl);
void mmu_map_app_region(int app_id, uint32_t *l1_pgtbl, uint32_t start, uint32_t size, bool ro, bool exec);
void mmu_dump_app_pgtbl(void);
void mmu_clear_app_pgtbl(uint32_t app_id);
#endif // CONFIG_APP_BINARY_SEPARATION

/********************************************************************************
 * MMU Page Protection Functions for Use-After-Free Detection
 * ********************************************************************************/

#ifdef CONFIG_MMU_PAGE_PROTECT

/****************************************************************************
 * Name: mmu_split_section_to_l2
 *
 * Description:
 *   Split a 1MB L1 section mapping into 256 × 4KB L2 small page mappings.
 *   This is necessary before using mmu_set_page_ro() or mmu_set_page_no_access()
 *   on a specific 4KB page within a 1MB section, otherwise the entire 1MB
 *   section would be affected.
 *
 * Input Parameters:
 *   vaddr - A virtual address within the 1MB section to split.
 *
 ****************************************************************************/

int mmu_split_section_to_l2(uint32_t vaddr);

/****************************************************************************
 * Name: mmu_save_page_pte

 *
 * Description:
 *   Save the current page table entry (L2 PTE or L1 section entry) for a
 *   given virtual address so it can be restored later.
 *
 * Input Parameters:
 *   vaddr - A virtual address within the page whose PTE should be saved.
 *
 ****************************************************************************/

uint32_t mmu_save_page_pte(uint32_t vaddr);

/****************************************************************************
 * Name: mmu_set_page_no_access
 *
 * Description:
 *   Set a 4KB page (or 1MB section) to No-Access at any privilege level.
 *   Any subsequent access triggers a Data Abort.  Used for use-after-free
 *   detection on page-aligned allocations.
 *
 * Input Parameters:
 *   vaddr - A virtual address within the page to protect.
 *
 ****************************************************************************/

int mmu_set_page_no_access(uint32_t vaddr);

/****************************************************************************
 * Name: mmu_set_page_ro
 *
 * Description:
 *   Set a 4KB page (or 1MB section) to Read-Only at privileged level.
 *   Any write triggers a Data Abort.  Used to protect read-only data
 *   structures from accidental corruption.
 *
 * Input Parameters:
 *   vaddr - A virtual address within the page to protect.
 *
 ****************************************************************************/

int mmu_set_page_ro(uint32_t vaddr);

/****************************************************************************
 * Name: mmu_restore_page_pte
 *
 * Description:
 *   Restore a previously saved PTE for a given virtual address, reverting
 *   the page to its original access permissions.
 *
 * Input Parameters:
 *   vaddr     - A virtual address within the page to restore.
 *   saved_pte - The PTE value previously returned by mmu_save_page_pte().
 *
 ****************************************************************************/

int mmu_restore_page_pte(uint32_t vaddr, uint32_t saved_pte);

/* Split the section that contains vaddr and keep the resulting second-level
 * table for the caller: the table is never released and is tagged with
 * 'owner' so a later mmu_split_section_to_l2() on the same 1MB region is
 * accepted instead of refused.  Returns OK, -EINVAL (not a section) or
 * -ENOMEM (L2 pool exhausted). */

int mmu_protect_page_reserve(uint32_t vaddr, const char *owner);

/* Read the live small-page descriptor for vaddr: *pte receives the raw
 * descriptor and *ap the access permission folded to
 * AP[1:0] | (AP2 << 1), so 0..3 - 0 = no access, 1 = privileged
 * read/write, 3 = privileged read-only ("is it protected?" is a test of
 * bit 1).  Returns -EINVAL when the address is not covered by a page
 * table, i.e. it is still a 1MB section and has no 4KB permission at all. */

int mmu_get_page_pte_raw(uint32_t vaddr, uint32_t *pte, uint8_t *ap);

/* Claimed second-level tables of the static L2 pool: returns the number in
 * use and, through 'total', the number CONFIG_MMU_L2_POOL_TABLES allows.
 * Tables are never released, so the returned count only grows. */

uint32_t mmu_l2_pool_usage(uint32_t *total);

/* Of the claimed tables, how many are tagged as held for a region that is
 * protected for the life of the system (mmu_protect_page_reserve()).  The
 * difference against mmu_l2_pool_usage() is what ad-hoc splitting has used;
 * tables are never released, so neither number goes down. */

uint32_t mmu_l2_pool_held(void);

/* Cumulative split / protection counters for soak runs and post-mortem
 * inspection: ok counts the pages currently protected, skipped the map
 * attempts that found a mapping they do not own, and failed_budget the
 * attempts refused because the L2 pool was exhausted. */

#ifdef CONFIG_MMU_PAGE_PROTECT
void mmu_protect_get_counters(uint32_t *ok, uint32_t *skipped,
                              uint32_t *failed_budget);
#else
#define mmu_protect_get_counters(a, b, c)    do { } while (0)
#endif

/* Largest range mmu_protect_range_no_access() accepts, and how many pages the
 * save table currently holds.  The allocator hook asks for the first instead
 * of keeping its own copy of the limit (two definitions of one number drift
 * apart with nothing but a coverage counter to show for it); the second is
 * the ceiling on how much memory can be No-Access at once - the L2 pool only
 * bounds how many 1MB sections can ever be split. */

size_t mmu_protect_range_max(void);
uint32_t mmu_protect_saved_usage(uint32_t *max);

/* Is the page containing vaddr currently an AP=0 descriptor?  Read only, no
 * side effect.  Any dump path that walks freed heap memory must ask this
 * before dereferencing (plan risk R5: the reporter must not fault inside the
 * fault it reports). */

bool mmu_range_is_no_access(uintptr_t vaddr);

/* Hardening: non-zero once a MANAGER-domain mapping has been observed.
 * AP-bit protection is vacuous while this holds, so a subsystem must
 * refuse to install it (the watchdog init does).  Implemented over the
 * live DACR as well as the latched flag. */

bool mmu_domain_manager_seen(void);
#endif /* CONFIG_MMU_PAGE_PROTECT */

/****************************************************************************
 * Protected Region Registry (MMU fault report, R2 mitigation)
 *
 * Named registry of protected regions. The data abort handler looks up
 * the faulting address so the fault report can identify which protected
 * region was hit (design item M0-6). Producers of protected regions
 * (test driver, watchdog pool, heap sanitizer) register here.
 *
 * Abort-context safe: lookup performs no allocation and takes no locks.
 ****************************************************************************/

#ifdef CONFIG_MMU_FAULT_REPORT

/****************************************************************************
 * Name: mmu_protect_region_register
 *
 * Description:
 *   Register a named region so the MMU fault report can name it when a
 *   faulting address falls inside it. Re-registering the identical
 *   region (same name, address and size) is idempotent.
 *
 * Input Parameters:
 *   name  - Region name; the string must have static storage duration.
 *   vaddr - Region start virtual address.
 *   size  - Region size in bytes.
 *
 * Returned Value:
 *   OK (0) on success, or a negated errno value on failure.
 ****************************************************************************/

int mmu_protect_region_register(const char *name, uintptr_t vaddr, size_t size);

/****************************************************************************
 * Name: mmu_protect_region_lookup
 *
 * Description:
 *   Look up the registered region containing the given virtual address.
 *   Safe to call from abort context.
 *
 * Input Parameters:
 *   vaddr - Virtual address to look up.
 *   psize - Optional (may be NULL) location to store the region size.
 *
 * Returned Value:
 *   The region name, or NULL if no registered region contains the address.
 ****************************************************************************/

const char *mmu_protect_region_lookup(uintptr_t vaddr, size_t *psize);

#endif /* CONFIG_MMU_FAULT_REPORT */

/****************************************************************************
 * ARM CP15 and Performance Counter Inline Functions
 *
 * These are static inline to avoid linker conflicts when included from
 * multiple .c files (kernel and drivers).
 ****************************************************************************/

/* NOTE: cp15_clean_dcache_bymva is defined in arch/arm/src/armv7-a/cp15_cacheops.h for arch code */

/* For kernel/drivers that can't include arch headers, provide __cp15_clean_dcache_bymva__ */
static inline void __cp15_clean_dcache_bymva__(uint32_t va)
{
	__asm__ __volatile__(
		"mcr p15, 0, %0, c7, c10, 1"  /* DCCMVAC */
		:
		: "r" (va)
		: "memory"
	);
}

#ifndef __ASM_ARM_BARRIERS_H
#define ARM_DSB() __asm__ __volatile__("dsb sy" ::: "memory")
#define ARM_ISB() __asm__ __volatile__("isb" ::: "memory")
#else
/* Use arch definitions if available */
#define ARM_DSB() dsb_sy()
#define ARM_ISB() isb()
#endif

#ifndef _TINYARA_CLOCK_H
static inline uint32_t up_get_cycle_count(void)
{
	uint32_t cycles;
	__asm__ __volatile__(
		"mrc p15, 0, %0, c9, c13, 0"  /* Read PMCCNTR */
		: "=r" (cycles)
		:
		: "memory"
	);
	return cycles;
}
#endif

#endif
