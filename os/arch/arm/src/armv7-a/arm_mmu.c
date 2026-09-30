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
 * arch/arm/src/armv7-a/arm_mmu.c
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
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdint.h>

#include "cp15_cacheops.h"
#include "barriers.h"
#include "mmu.h"


#ifdef CONFIG_APP_BINARY_SEPARATION
#include <tinyara/mm/mm.h>
#endif

#ifdef CONFIG_MMU_PAGE_PROTECT
#ifdef CONFIG_SMP
#include "gic.h"
#include "smp.h"
#include "sched/sched.h"
#endif
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
#ifdef CONFIG_APP_BINARY_SEPARATION
static int l2_idx = 0;
#endif

#ifdef CONFIG_MMU_PAGE_PROTECT
#ifdef CONFIG_SMP

/* Global variable to pass PTE address to SGI4 handler on other cores.
 * Written by Core 0 before sending SGI4, read by Core 1 in SGI handler.
 * The PTE address is needed for DCIMVAC (invalidate dcache line) to
 * remove the stale PTE from Core 1's L1 data cache before TLBIALL.
 */
static volatile uint32_t g_tlb_flush_pte_addr;
static volatile uint32_t g_tlb_flush_done;  /* ACK flag: Core 1 sets after TLB flush */

/* Safe wrapper: cross-checks mmu_l1_pgtable() against TTBR0.
 * If they match, returns mmu_l1_pgtable() (no change).
 * If they differ, prints warning and returns TTBR0-based address.
 */
uint32_t *mmu_l1_pgtable_safe(void)
{
	uint32_t *pgtable = mmu_l1_pgtable();
	uint32_t raw_ttbr0;

	__asm__ volatile("mrc p15, 0, %0, c2, c0, 0" : "=r"(raw_ttbr0));
	uint32_t ttbr0_base = raw_ttbr0 & 0xFFFFC000;

	if ((uint32_t)pgtable != ttbr0_base) {
		lldbg("WARN: mmu_l1_pgtable()=0x%08x but TTBR0 base=0x%08x — using TTBR0\n",
		      (uint32_t)pgtable, ttbr0_base);
		return (uint32_t *)ttbr0_base;
	}
	return pgtable;
}

/* SGI4 handler — runs on target CPU, flushes entire local TLB.
 * Uses TLBIALL (not TLBIMVA) to:
 *   1. Avoid needing to pass address via shared variable (no cache coherency issue)
 *   2. Flush stale 1MB section TLB entries after section-to-L2 split
 *
 * CRITICAL: Also does DCIMVAC to invalidate the stale PTE from this CPU's
 * L1 data cache. Without this, the MMU page table walker reads the old
 * PTE from the data cache after TLBIALL, and the protection has no effect.
 *
 * This is non-blocking: the requesting CPU sends SGI4 and returns immediately.
 * The target CPU flushes its TLB asynchronously when the SGI is handled.
 */
int mmu_tlb_flush_sgi_handler(int irq, void *context, void *arg)
{
	uint32_t pte_addr = g_tlb_flush_pte_addr;
	uint32_t pte_before;
	uint32_t sctlr, ttbr0, dacr;

	/* Read system control registers to verify MMU state */
	__asm__ volatile("mrc p15, 0, %0, c1, c0, 0" : "=r"(sctlr));  /* SCTLR */
	__asm__ volatile("mrc p15, 0, %0, c2, c0, 0" : "=r"(ttbr0));  /* TTBR0 */
	__asm__ volatile("mrc p15, 0, %0, c3, c0, 0" : "=r"(dacr));   /* DACR */

	pte_before = *(volatile uint32_t *)pte_addr;

	lldbg("SGI4: === ENTER on CPU%d ===\n", this_cpu());
	lldbg("SGI4: SCTLR=0x%08x (MMU=%d DC=%d IC=%d)\n",
	      sctlr, sctlr & 1, (sctlr >> 2) & 1, (sctlr >> 12) & 1);
	lldbg("SGI4: TTBR0=0x%08x (base=0x%08x)\n", ttbr0, ttbr0 & 0xFFFFC000);
	lldbg("SGI4: DACR=0x%08x (dom0=%s dom1=%s)\n",
	      dacr,
	      (dacr & 0x3) == 0 ? "NO_ACCESS" :
	      (dacr & 0x3) == 1 ? "CLIENT" :
	      (dacr & 0x3) == 3 ? "MANAGER" : "RESERVED",
	      ((dacr >> 2) & 0x3) == 0 ? "NO_ACCESS" :
	      ((dacr >> 2) & 0x3) == 1 ? "CLIENT" :
	      ((dacr >> 2) & 0x3) == 3 ? "MANAGER" : "RESERVED");
	lldbg("SGI4: pte_addr=0x%08x pte_before=0x%08x (AP=0x%x type=0x%x)\n",
	      pte_addr, pte_before, (pte_before >> 4) & 3, pte_before & 3);

	/* CRITICAL CHECK: If domain 0 is MANAGER, AP bits are IGNORED! */
	if ((dacr & 0x3) == 3) {
		lldbg("SGI4: !!! DOMAIN 0 IS MANAGER — AP BITS IGNORED! THIS IS THE BUG !!!\n");
	}

	/* Invalidate dcache line for pte_addr — removes stale PTE from this CPU's L1 dcache */
	cp15_invalidate_dcacheline_bymva(pte_addr);
	ARM_DSB();

	/* Flush TLB — TLBIALL (entire local TLB) */
	__asm__ volatile("mcr p15, 0, r0, c8, c7, 0");  /* TLBIALL */
	ARM_DSB();
	ARM_ISB();

	lldbg("SGI4: TLBIALL done, signaling completion\n");

	/* ACK: set g_tlb_flush_done so Core 0 can stop spin-waiting */
	g_tlb_flush_done = 1;
	cp15_clean_dcache_bymva((uint32_t)&g_tlb_flush_done);
	ARM_DSB();

	return OK;
}
#endif /* CONFIG_SMP */

/* SMP TLB invalidate by VA.
 * On SMP systems, TLBIMVAAIS (hardware broadcast) may not work if the
 * SMP coherency bit is not enabled. Instead, we use SGI (Software
 * Generated Interrupt) to notify other cores to flush their local TLB.
 * This is guaranteed to work because GIC SGI is already used for
 * CPU start/pause/hotplug on this hardware.
 *
 * Non-blocking: sends SGI4 to all other cores and returns immediately.
 */
static inline void mmu_invalidate_tlb_bymva_is(uint32_t vaddr)
{
#ifdef CONFIG_SMP
	int cpu;
	int me = this_cpu();
	uint32_t l1_entry;

	/* 1. Compute PTE address using mmu_l1_pgtable_safe() */
	{
		uint32_t *l1table = mmu_l1_pgtable_safe();
		uint32_t l1_index = vaddr >> 20;
		l1_entry = l1table[l1_index];
	}

	if ((l1_entry & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
		uint32_t *l2table = (uint32_t *)(l1_entry & PMD_PTE_PADDR_MASK);
		uint32_t l2_index = (vaddr & 0x000FF000) >> 12;
		g_tlb_flush_pte_addr = (uint32_t)&l2table[l2_index];
	} else {
		uint32_t *l1table = mmu_l1_pgtable_safe();
		g_tlb_flush_pte_addr = (uint32_t)&l1table[vaddr >> 20];
	}

	/* 2. Clean dcache for g_tlb_flush_pte_addr so Core 1 sees the correct value */
	cp15_clean_dcache_bymva((uint32_t)&g_tlb_flush_pte_addr);
	ARM_DSB();

	/* 3. Clear completion flag */
	g_tlb_flush_done = 0;
	cp15_clean_dcache_bymva((uint32_t)&g_tlb_flush_done);
	ARM_DSB();

	/* 4. Flush local TLB for this address (TLBIMVA — local only) */
	__asm__ volatile("mcr p15, 0, %0, c8, c7, 1" :: "r"(vaddr)); /* TLBIMVA — local */
	ARM_DSB();
	ARM_ISB();

	lldbg("TLB_IS: CPU%d local TLB flushed for vaddr=0x%08x, sending SGI4\n", me, vaddr);

	/* 5. Send SGI4 to all other cores */
	for (cpu = 0; cpu < CONFIG_SMP_NCPUS; cpu++) {
		if (cpu == me) {
			continue;
		}
		{
			uint32_t pte_val = *(volatile uint32_t *)g_tlb_flush_pte_addr;
			lldbg("TLB_IS: CPU%d sending SGI4 to CPU%d (vaddr=0x%08x pte_addr=0x%08x pte_val=0x%08x AP=0x%x type=0x%x)\n",
			       me, cpu, vaddr, g_tlb_flush_pte_addr, pte_val,
			       (pte_val >> 4) & 3, pte_val & 3);
		}
		arm_cpu_sgi(GIC_IRQ_SGI4, (1 << cpu));

		/* 6. Spin-wait for this core to acknowledge TLB flush */
		{
			uint32_t timeout = 1000000;
			while (g_tlb_flush_done == 0 && timeout-- > 0) {
				/* Invalidate dcache line to force re-read from memory */
				cp15_invalidate_dcacheline_bymva((uint32_t)&g_tlb_flush_done);
			}
			if (g_tlb_flush_done) {
				lldbg("TLB_IS: CPU%d acknowledged TLB flush\n", cpu);
			} else {
				lldbg("TLB_IS: TIMEOUT waiting for CPU%d!\n", cpu);
			}
		}
		/* Reset flag for next CPU (if multiple) */
		g_tlb_flush_done = 0;
		cp15_clean_dcache_bymva((uint32_t)&g_tlb_flush_done);
		ARM_DSB();
	}
#else
	/* Non-SMP: just flush local TLB */
	__asm__ volatile("mcr p15, 0, %0, c8, c7, 1" :: "r"(vaddr)); /* TLBIMVA — local */
	ARM_DSB();
	ARM_ISB();
#endif
}
#endif /* CONFIG_MMU_PAGE_PROTECT */


/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void mmu_set_flags(uint32_t *val, bool ro, bool exec, uint8_t isL1)
{
	if (isL1) {
		if (ro && exec) {
			*val |= MMU_APP_L1_ROX;
		} else if (ro) {
			*val |= MMU_APP_L1_RO;
		} else if (exec) {
			*val |= MMU_APP_L1_RWX;
		} else {
			*val |= MMU_APP_L1_RW;
		}
	} else {
		if (ro && exec) {
			*val |= MMU_APP_L2_ROX;
		} else if (ro) {
			*val |= MMU_APP_L2_RO;
		} else if (exec) {
			*val |= MMU_APP_L2_RWX;
		} else {
			*val |= MMU_APP_L2_RW;
		}
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmu_l1_setentry
 *
 * Description:
 *   Set a one level 1 translation table entry.  Only a single L1 page table
 *   is supported.
 *
 * Input Parameters:
 *   paddr - The physical address to be mapped.  Must be aligned to a 1MB
 *     address boundary
 *   vaddr - The virtual address to be mapped.  Must be aligned to a 1MB
 *     address boundary
 *   mmuflags - The MMU flags to use in the mapping.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l1_setentry(uint32_t paddr, uint32_t vaddr, uint32_t mmuflags)
{
	uint32_t *l1table = mmu_l1_pgtable();
	uint32_t index = vaddr >> 20;

	/* Save the page table entry */

	l1table[index] = (paddr | mmuflags);

	/* Flush the data cache entry.  Make sure that the modified contents
	 * of the page table are flushed into physical memory.
	 */

	cp15_clean_dcache_bymva((uint32_t)&l1table[index]);

	/* Invalidate the TLB cache associated with virtual address range */

	mmu_invalidate_region(vaddr, SECTION_SIZE);
}
#endif

/****************************************************************************
 * Name: mmu_l1_restore
 *
 * Description:
 *   Restore one L1 table entry previously returned by mmu_l1_getentry() (or
 *   any other encoded L1 page table value).
 *
 * Input Parameters:
 *   vaddr - A virtual address to be mapped
 *   l1entry - The value to write into the page table entry
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_ADDRENV)
void mmu_l1_restore(uintptr_t vaddr, uint32_t l1entry)
{
	uint32_t *l1table = mmu_l1_pgtable();
	uint32_t index = vaddr >> 20;

	/* Set the encoded page table entry */

	l1table[index] = l1entry;

	/* Flush the data cache entry.  Make sure that the modified contents
	 * of the page table are flushed into physical memory.
	 */

	cp15_clean_dcache_bymva((uint32_t)&l1table[index]);

	/* Invalidate the TLB cache associated with virtual address range */

	mmu_invalidate_region(vaddr & PMD_PTE_PADDR_MASK, SECTION_SIZE);
}
#endif

/****************************************************************************
 * Name: mmu_l2_setentry
 *
 * Description:
 *   Set one small (4096B) entry in a level2 translation table.
 *
 * Input Parameters:
 *   l2vaddr - the virtual address of the beginning of the L2 translation
 *     table.
 *   paddr - The physical address to be mapped.  Must be aligned to a 4KB
 *     address boundary
 *   vaddr - The virtual address to be mapped.  Must be aligned to a 4KB
 *     address boundary
 *   mmuflags - The MMU flags to use in the mapping.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l2_setentry(uint32_t l2vaddr, uint32_t paddr, uint32_t vaddr, uint32_t mmuflags)
{
	uint32_t *l2table = (uint32_t *)l2vaddr;
	uint32_t index;

	/* The table divides a 1Mb address space up into 256 entries, each
	 * corresponding to 4Kb of address space.  The page table index is
	 * related to the offset from the beginning of 1Mb region.
	 */

	index = (vaddr & 0x000ff000) >> 12;

	/* Save the table entry */

	l2table[index] = (paddr | mmuflags);

	/* Flush the data cache entry.  Make sure that the modified contents
	 * of the page table are flushed into physical memory.
	 */

	cp15_clean_dcache_bymva((uint32_t)&l2table[index]);

	/* Invalidate the TLB cache associated with virtual address range */

	cp15_invalidate_tlb_bymva(vaddr);
}
#endif

/****************************************************************************
 * Name: mmu_l1_map_region
 *
 * Description:
 *   Set multiple level 1 translation table entries in order to map a
 *   region of memory.
 *
 * Input Parameters:
 *   mapping - Describes the mapping to be performed.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l1_map_region(const struct section_mapping_s *mapping)
{
	uint32_t paddr = mapping->physbase;
	uint32_t vaddr = mapping->virtbase;
	uint32_t mmuflags = mapping->mmuflags;
	int i;

	/* Loop, writing each mapping into the L1 page table */

	for (i = 0; i < mapping->nsections; i++) {
		mmu_l1_setentry(paddr, vaddr, mmuflags);
		paddr += SECTION_SIZE;
		vaddr += SECTION_SIZE;
	}
}
#endif

/****************************************************************************
 * Name: mmu_l1_map_regions
 *
 * Description:
 *   Set multiple level 1 translation table entries in order to map a region
 *   array of memory.
 *
 * Input Parameters:
 *   mappings - Describes the array of mappings to be performed.
 *   count    - The number of mappings to be performed.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l1_map_regions(const struct section_mapping_s *mappings, size_t count)
{
	size_t i;

	for (i = 0; i < count; i++) {
		mmu_l1_map_region(&mappings[i]);
	}
}
#endif

/****************************************************************************
 * Name: mmu_invalidate_region
 *
 * Description:
 *   Invalidate TLBs for a range of addresses (all 4KB aligned).
 *
 * Input Parameters:
 *   vaddr - The beginning of the region to invalidate.
 *   size  - The size of the region in bytes to be invalidated.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_invalidate_region(uint32_t vstart, size_t size)
{
	uint32_t vaddr = vstart & 0xfffff000;
	uint32_t vend = vstart + size;

	/* Loop, invalidating regions */

	while (vaddr < vend) {
		cp15_invalidate_tlb_bymva(vaddr);
		vaddr += 4096;
	}
}
#endif

#ifdef CONFIG_APP_BINARY_SEPARATION
/****************************************************************************
 * Name: mmu_get_os_l1_pgtbl
 *
 * Description:
 *   Returns the virtual address of the kernel L1 page table.
 *
 * Input Parameters:
 *
 * Returned Value:
 * Page table address
 ****************************************************************************/
uint32_t *mmu_get_os_l1_pgtbl(void)
{
	return (uint32_t *)PGTABLE_BASE_VADDR;
}

/****************************************************************************
 * Name: mmu_allocate_app_l1_pgtbl
 *
 * Description:
 *   Allocate space for L1 page table of application, in accordance with
 *   the requirements of the arch specific mmu.
 *
 * Input Parameters:
 *
 * Returned Value:
 * L1 Page table address
 ****************************************************************************/
uint32_t *mmu_allocate_app_l1_pgtbl(int app_id)
{
	uint32_t *addr = (uint32_t *)(PGTABLE_BASE_VADDR + (app_id * L1_PGTBL_SIZE));
	l2_idx = 0;
	return addr;
}

/****************************************************************************
 * Name: mmu_allocate_app_l2_pgtbl
 *
 * Description:
 *   Allocate space for L2 page table of application, in accordance with
 *   the requirements of the arch specific mmu.
 *
 * Input Parameters:
 *
 * Returned Value:
 * L2 Page table address
 ****************************************************************************/
uint32_t *mmu_allocate_app_l2_pgtbl(int app_id, int l2idx)
{
	/* === DIAGNOSTIC LOGS START === */
	lldbg("L2_ALLOC: >>> ENTER <<< app_id=%d l2idx=%d\n", app_id, l2idx);
	lldbg("L2_ALLOC: CONFIG_NUM_L2_PER_APP = %d\n", CONFIG_NUM_L2_PER_APP);
	lldbg("L2_ALLOC: PGTABLE_BASE_VADDR = 0x%08X\n", PGTABLE_BASE_VADDR);
	lldbg("L2_ALLOC: L1_PGTBL_SIZE = %d bytes\n", L1_PGTBL_SIZE);
	lldbg("L2_ALLOC: L2_PGTBL_SIZE = %d bytes\n", L2_PGTBL_SIZE);
	/* === DIAGNOSTIC LOGS END === */

	dbg("app_id : %d, l2idx : %d\n", app_id, l2idx);
	/* Page table structure as follows:
	 *
	 * | Kernel L1 | App1 L1 | App2 L1 | Common L2 1 ... CONFIG_NUM_L2_PER_APP | App1 L2 1 ... CONFIG_NUM_L2_PER_APP | App2 L2 1 ... CONFIG_NUM_L2_PER_APP |
	 *
	 * In the below calculation, we take (CONFIG_NUM_APPS + 1) because the first
	 * page table is for kernel. Also, we reserve CONFIG_NUM_L2_PER_APP L2 page tables for each app.
	 */
	uint32_t *addr = (uint32_t *)(PGTABLE_BASE_VADDR + ((CONFIG_NUM_APPS + 1) * L1_PGTBL_SIZE) + 
			((CONFIG_NUM_APPS - app_id) * CONFIG_NUM_L2_PER_APP * L2_PGTBL_SIZE) + (l2idx * L2_PGTBL_SIZE));

	/* === DIAGNOSTIC LOGS START === */
	lldbg("L2_ALLOC: Calculated addr = 0x%08X\n", (uint32_t)addr);
	lldbg("L2_ALLOC: L2 table range: 0x%08X - 0x%08X\n", 
	      (uint32_t)addr, (uint32_t)addr + L2_PGTBL_SIZE - 1);
	lldbg("L2_ALLOC: <<< EXIT >>>\n");
	/* === DIAGNOSTIC LOGS END === */

	return addr;
}

/****************************************************************************
 * Name: mmu_update_app_l1_pgtbl_ospgtbl
 *
 * Description:
 * Loop through the L1 page table.
 * Copy kernel L1 page table to app page table.
 * If the entry is pointing to a L2 page table
 * Allocate L2 page table for app.
 * Copy entries from kernel to app L2 table.
 * Update the L2 page table address in L1 table.
 *
 * Input Parameters:
 * app_pgtbl: Pointer to L1 page table of app
 *
 ****************************************************************************/
void mmu_update_app_l1_pgtbl_ospgtbl(uint32_t *app_l1_pgtbl)
{
	uint32_t *os_l1_pgtbl = (uint32_t *)PGTABLE_BASE_VADDR;
	memcpy((void *)app_l1_pgtbl, (void *)os_l1_pgtbl, L1_PGTBL_SIZE);
}

/****************************************************************************
 * Name: mmu_clear_app_pgtbl
 *
 * Description:
 * Clear the app page tables. If required reset the entries in L1 page table.
 *
 * Input Parameters:
 * app_id : Application id for clearing page tables.
 *
 * Returned Value:
 *
 ****************************************************************************/
void mmu_clear_app_pgtbl(uint32_t app_id)
{
	uint32_t *addr;

	// Clear L2 page tables
	addr = (uint32_t *)(PGTABLE_BASE_VADDR + ((CONFIG_NUM_APPS + 1) * L1_PGTBL_SIZE) + (app_id * CONFIG_NUM_L2_PER_APP * L2_PGTBL_SIZE));
	memset(addr, 0, CONFIG_NUM_L2_PER_APP * L2_PGTBL_SIZE);

	if (app_id == 0) {
		// Reset the L2 page entries in L1 page table
		addr = (uint32_t *) PGTABLE_BASE_VADDR;
		for (int i = 0; i < L1_PGTBL_NENTRIES; i++) {
			if ((addr[i] & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
				addr[i] = addr[i - 1] & PMD_SECT_PADDR_MASK;
				addr[i] += SECTION_SIZE;
				addr[i] |= MMU_MEMFLAGS;
			}
		}
	} else {
		// Clear L1 page table
		addr = (uint32_t *)(PGTABLE_BASE_VADDR + (app_id * L1_PGTBL_SIZE));
		memset(addr, 0, L1_PGTBL_SIZE);
	}

	cp15_wrttb((uint32_t)(mmu_get_os_l1_pgtbl()) | TTBR0_RGN_WBWA | TTBR0_IRGN0);
	cp15_invalidate_tlbs();
}

/****************************************************************************
 * Name: mmu_map_app_region
 *
 * Description:
 * The api will create page tables for the given section of app and update them with the
 * access permissions as mentioned in the input parameters. While updating the page table,
 * following is done:
 * - Check if the given region can exactly fit into a 1MB section.
 * - In this case, update the corresponding section in the l1 page table.
 * - Otherwise, create an L2 page table and update it with default entries.
 * - Later, update the access permissions in the L2 page table for the given memory region.
 * - Update the L2 page table address in L1 page table.
 *
 * Input Parameters:
 * app_id :	Id of the app being updated.
 * l1_pgtbl:	Pointer to the L1 page table allocated for the app.
 * start:	Start address of memory region to be mapped.
 * size:	Size of the memory region to be mapped.
 * ro:		Whether this region is read-only?
 * exec:	Whether this region has execute permission?
 *
 * Returned Value:
 ****************************************************************************/
void mmu_map_app_region(int app_id, uint32_t *l1_pgtbl, uint32_t start, uint32_t size, bool ro, bool exec)
{
	uint32_t idx;
	uint32_t val;
	uint32_t end = start + size;

	// Run a loop until the entire region is mapped.
	while (start < end) {
		// Check if this address can be mapped to a section. Below are the conditions,
		// 1. start should be aligned to section size
		// 2. remaining size should be greater than or equal to section size
		if (!(start & SECTION_MASK) && size >= SECTION_SIZE) {
			// Yes. Update the section entry in the the L1 page table.
			idx = start >> 20;
			val = start & PMD_PTE_PADDR_MASK;
			mmu_set_flags(&val, ro, exec, true);
			l1_pgtbl[idx] = val;

			// Advance the memory region address.
			start += SECTION_SIZE;
			size -= SECTION_SIZE;
		} else {				// Check if this address can be mapped to a small page.

			// Check if L2 page table is not created.
			idx = (start & 0xfff00000) >> 20;
			uint32_t *l2_pgtbl = (uint32_t *)(l1_pgtbl[idx] & PMD_PTE_PADDR_MASK);

			if ((l1_pgtbl[idx] & PMD_TYPE_MASK) != PMD_TYPE_PTE) {
				// Yes. Allocate L2 page table for app.
				l2_pgtbl = mmu_allocate_app_l2_pgtbl(app_id, l2_idx++);

				// Fill the newly allocated l2 page table with default kernel page entries
				uint32_t l2_start = start & PMD_SECT_PADDR_MASK;
				for (int i = 0; i < L2_PGTBL_NENTRIES; i++) {
					l2_pgtbl[i] = l2_start | MMU_L2_MEMFLAGS;
					l2_start += SMALL_PAGE_SIZE;
				}

				// Update L2 page table address in L1 page table.
				val = (uint32_t)l2_pgtbl & PMD_PTE_PADDR_MASK;
				val |= MMU_L1_PGTABFLAGS;
				l1_pgtbl[idx] = val;
			}
			// Update the L2 page table entry.
			idx = (start & 0x000ff000) >> 12;
			val = start & PTE_SMALL_PADDR_MASK;
			mmu_set_flags(&val, ro, exec, false);
			l2_pgtbl[idx] = val;

			// Advance the memory region address.
			start += SMALL_PAGE_SIZE;
			size -= SMALL_PAGE_SIZE;
		}
	}
	cp15_invalidate_tlbs();
}

/****************************************************************************
 * Name: mmu_dump_app_pgtbl
 *
 * Description:
 * Print the L1 and L2 page table entries corresponding to the application.
 *
 * Input Parameters:
 *
 * Returned Value:
 ****************************************************************************/
void mmu_dump_app_pgtbl(void)
{
	struct tcb_s *rtcb = sched_self();
	uint32_t *l1tbl = mmu_l1_pgtable();

	lldbg_noarg("L1 page table base addr = 0x%08x appid = %d\n", l1tbl, rtcb->app_id);

	if (rtcb->app_id < 1) {
		return;
	}

	lldbg_noarg("=====================================================================\n");
	lldbg_noarg("ENTRY      TYPE    OUT             ACCESS\n");
	lldbg_noarg("ADDR               ADDR                  \n");
	lldbg_noarg("=====================================================================\n");
	for (int i = 0; i < L1_PGTBL_NENTRIES; i++) {
		if ((l1tbl[i] & PMD_TYPE_MASK) == PMD_TYPE_SECT && (l1tbl[i] & PMD_SECT_AP1)) {	// Only print user areas.
			lldbg_noarg("0x%08x SECT    0x%08x      %s-%s\n", &l1tbl[i], l1tbl[i] & PMD_SECT_PADDR_MASK, (l1tbl[i] & PMD_SECT_AP2) ? "RO" : "RW", (l1tbl[i] & PMD_SECT_XN) ? "XN" : "X");
		} else if ((l1tbl[i] & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
			lldbg_noarg("0x%08x L1PTE   0x%08x\n", &l1tbl[i], l1tbl[i] & PMD_PTE_PADDR_MASK);
			uint32_t *l2tbl = (uint32_t *)(l1tbl[i] & PMD_PTE_PADDR_MASK);
			for (int j = 0; j < L2_PGTBL_NENTRIES; j++) {
				if ((l2tbl[j] & PTE_AP1) &&	// Only print user areas.
					((l2tbl[j] & PTE_TYPE_MASK) != PTE_TYPE_FAULT)) {
					lldbg_noarg("0x%08x PAGE    0x%08x      %s-%s\n", &l2tbl[j], l2tbl[j] & PTE_SMALL_PADDR_MASK, (l2tbl[j] & PTE_AP2) ? "RO" : "RW", (l2tbl[j] & PTE_SMALL_XN) ? "XN" : "X");
				}
			}
		}
	}
	lldbg_noarg("=====================================================================\n");
}
#endif							// CONFIG_APP_BINARY_SEPARATION

/****************************************************************************
 * Name: mmu_split_section_to_l2
 *
 * Description:
 *   Split a 1MB section mapping into 256 × 4KB L2 small page mappings.
 *
 *   On ARMv7-A, kernel RAM is typically mapped with 1MB L1 section entries.
 *   This means mmu_set_page_ro() would make the entire 1MB section read-only,
 *   not just the 4KB page we want to protect.
 *
 *   This function:
 *     1. Allocates a static L2 page table (1KB, 256 entries)
 *     2. Fills all 256 entries with 4KB small page mappings covering the
 *        same 1MB physical region, preserving the original access permissions
 *        and cache attributes from the L1 section entry
 *     3. Replaces the L1 section entry with an L1 PTE entry pointing to
 *        the new L2 page table
 *     4. Flushes data cache and invalidates TLB for the region
 *
 *   After this call, mmu_set_page_ro() / mmu_save_page_pte() will find
 *   PMD_TYPE_PTE and operate on individual 4KB L2 entries.
 *
 * Input Parameters:
 *   vaddr - A virtual address within the 1MB section to split.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_split_section_to_l2(uint32_t vaddr)
{
	uint32_t section_addr = vaddr & ~((1 << SECTION_SHIFT) - 1);
	uint32_t l1_entry = mmu_l1_getentry(section_addr);
	uint32_t *l1table;
	uint32_t l1_index;
	uint32_t phys_base;
	static uint32_t s_l2_table[256] __attribute__((aligned(1024)));
	uint32_t *l2table;
	int i;

	lldbg("SPLIT_RUNTIME: vaddr=0x%08x section=0x%08x l1_entry=0x%08x\n",
	      vaddr, section_addr, l1_entry);
	lldbg("SPLIT_RUNTIME: s_l2_table at 0x%08x\n", (uint32_t)s_l2_table);

	/* Only split if currently mapped as a 1MB section */
	if ((l1_entry & PMD_TYPE_MASK) != PMD_TYPE_SECT) {
		/* Already an L2 page table or fault — nothing to do */
		lldbg("SPLIT: not a section (type=0x%x), skipping\n",
		      l1_entry & PMD_TYPE_MASK);
		return;
	}

	l1table = mmu_l1_pgtable_safe();
	l1_index = section_addr >> 20;

	/* Extract the physical base address from the section entry */
	phys_base = l1_entry & PMD_SECT_PADDR_MASK;

	lldbg("SPLIT: l1table=0x%08x l1_index=%d phys_base=0x%08x\n",
	      (uint32_t)l1table, l1_index, phys_base);

	l2table = s_l2_table;


	/* Build L2 small page entries for all 256 pages in the 1MB section.
	 * Translate the section flags to L2 small page flags:
	 *
	 * Section:  C=bit3, B=bit2, AP=bits[11:10], TEX=bits[14:12], S=bit16, XN=bit4, AP2=bit15
	 * L2 small: C=bit3, B=bit2, AP=bits[5:4],  TEX=bits[8:6],  S=bit10, XN=bit0, AP2=bit9
	 */
	for (i = 0; i < L2_PGTBL_NENTRIES; i++) {
		uint32_t page_phys = phys_base + (i * SMALL_PAGE_SIZE);
		uint32_t l2_entry = PTE_TYPE_SMALL;	/* 4KB small page */

		/* Copy bufferable/cacheable bits (same bit position) */
		if (l1_entry & PMD_SECT_B) {
			l2_entry |= PTE_B;
		}
		if (l1_entry & PMD_SECT_C) {
			l2_entry |= PTE_C;
		}

		/* Translate AP bits: section [11:10] -> L2 [5:4] */
		{
			uint32_t sect_ap = (l1_entry >> PMD_SECT_AP_SHIFT) & 3;
			l2_entry |= (sect_ap << PTE_AP_SHIFT);
		}

		/* Translate TEX bits: section [14:12] -> L2 [8:6] */
		{
			uint32_t tex = (l1_entry & PMD_SECT_TEX_MASK) >> PMD_SECT_TEX_SHIFT;
			l2_entry |= (tex << 6);
		}

		/* Copy AP[2] bit: section bit15 -> L2 bit9 */
		if (l1_entry & PMD_SECT_AP2) {
			l2_entry |= PTE_AP2;
		}

		/* Copy shareable bit: section bit16 -> L2 bit10 */
		if (l1_entry & PMD_SECT_S) {
			l2_entry |= PTE_S;
		}

		/* Translate XN bit: section bit4 -> L2 small page bit0.
		 * For small pages, XN is at bit 0. When XN is set, the type
		 * field becomes 0b11 (small page XN) instead of 0b10 (small page). */
		if (l1_entry & PMD_SECT_XN) {
			l2_entry |= PTE_SMALL_XN;
		}

		/* Set physical address */
		l2_entry |= (page_phys & PTE_SMALL_PADDR_MASK);

		l2table[i] = l2_entry;
	}


	/* Debug: dump first 3 L2 entries and the wdog page entry */
	lldbg("SPLIT: L2[0]=0x%08x L2[1]=0x%08x L2[2]=0x%08x\n",
	      l2table[0], l2table[1], l2table[2]);
	lldbg("SPLIT: L2[0x13]=0x%08x (wdog page entry, index=0x13)\n",
	      l2table[0x13]);
	lldbg("SPLIT: l1_entry flags: AP=0x%x TEX=0x%x C=%d B=%d S=%d\n",

	      (l1_entry >> PMD_SECT_AP_SHIFT) & 3,
	      (l1_entry & PMD_SECT_TEX_MASK) >> PMD_SECT_TEX_SHIFT,
	      (l1_entry & PMD_SECT_C) ? 1 : 0,
	      (l1_entry & PMD_SECT_B) ? 1 : 0,
	      (l1_entry & PMD_SECT_S) ? 1 : 0);
	lldbg("SPLIT: L2[0] flags:   AP=0x%x TEX=0x%x C=%d B=%d S=%d\n",
	      (l2table[0] >> PTE_AP_SHIFT) & 3,
	      (l2table[0] & 0x1C0) >> 6,
	      (l2table[0] & PTE_C) ? 1 : 0,
	      (l2table[0] & PTE_B) ? 1 : 0,
	      (l2table[0] & PTE_S) ? 1 : 0);

	/* CRITICAL: Clean data cache for the entire L2 table before
	 * changing the L1 entry.  The MMU page table walker reads from
	 * RAM, not from the data cache.  If the L2 entries are still
	 * only in cache when the L1 switch happens, the MMU will read
	 * stale/zero data and fault.
	 */
	for (i = 0; i < L2_PGTBL_NENTRIES; i++) {
		cp15_clean_dcache_bymva((uint32_t)&l2table[i]);
	}

	/* Ensure all L2 table writes are visible in physical RAM before
	 * we switch the L1 entry.  Without this barrier, the MMU page
	 * table walker may read stale/zero data from the L2 table.
	 */
	ARM_DSB();

	/* Replace the L1 section entry with an L1 PTE pointing to our L2 table.
	 * On this platform VA==PA for kernel RAM, so the virtual address of
	 * s_l2_table is also its physical address.
	 *
	 * Build the L1 PTE entry from scratch using only the type, domain, and
	 * page table address.  Do NOT use MMU_L1_PGTABFLAGS because it includes
	 * PMD_PTE_NS (Non-Secure bit) which is wrong for kernel RAM — the
	 * original section entry does not have NS set, and setting it on the
	 * PTE entry causes the L2 page table walks to use non-secure access,
	 * which can break inter-core shared memory (IPC).
	 */
	{
		uint32_t new_l1 = (uint32_t)l2table & PMD_PTE_PADDR_MASK;
		new_l1 |= PMD_TYPE_PTE;

		/* Preserve domain from original section entry */
		new_l1 |= (l1_entry & PMD_SECT_DOM_MASK);

		l1table[l1_index] = new_l1;
	}


	/* Flush: clean L1 entry in data cache, then DSB to ensure the L1
	 * write is visible to the MMU page table walker.
	 */
	cp15_clean_dcache_bymva((uint32_t)&l1table[l1_index]);
	ARM_DSB();

	/* Also split in kernel L1 table so Core 1 (kernel thread) sees the L2 table.
	 * Core 0 uses App1's L1 table (TTBR0=0x63ff4000), but Core 1 uses the
	 * kernel L1 table (TTBR0=0x63ff0000). If we only split in App1's table,
	 * Core 1 still sees the 1MB section with RW and bypasses the protection.
	 *
	 * Fix: replace the kernel L1 section with an L1 PTE pointing to the
	 * SAME s_l2_table. Then both cores walk the same L2 table and see
	 * the same AP bits.
	 */
	{
		uint32_t *kern_l1 = (uint32_t *)PGTABLE_BASE_VADDR;
		uint32_t kern_entry = kern_l1[l1_index];

		if ((kern_entry & PMD_TYPE_MASK) == PMD_TYPE_SECT) {
			uint32_t new_kern_l1;

			new_kern_l1 = (uint32_t)l2table & PMD_PTE_PADDR_MASK;
			new_kern_l1 |= PMD_TYPE_PTE;
			new_kern_l1 |= (kern_entry & PMD_SECT_DOM_MASK);  /* preserve domain */

			kern_l1[l1_index] = new_kern_l1;
			cp15_clean_dcache_bymva((uint32_t)&kern_l1[l1_index]);
			ARM_DSB();

			lldbg("SPLIT: kernel L1[0x%x] = 0x%08x (was section 0x%08x)\n",
			      l1_index, new_kern_l1, kern_entry);
		}
	}

	/* Invalidate the entire TLB (not just by VA) to ensure no stale
	 * section-mapping TLB entries remain.  The old L1 section entry
	 * may have been cached in the TLB as a 1MB section mapping; after
	 * replacing it with an L2 PTE, any stale TLB entry would cause the
	 * MMU to use the old section attributes instead of the new L2
	 * small page entries, leading to incorrect memory attributes.
	 */
	cp15_invalidate_tlbs();
	ARM_ISB();
}
#endif


/****************************************************************************
 * Name: mmu_save_page_pte
 *
 * Description:
 *   Save the current L2 (or L1 section) page table entry for a given
 *   virtual address so that it can be restored later by
 *   mmu_restore_page_pte().

 *
 *   This is useful when temporarily changing page permissions (e.g. to
 *   No-Access for use-after-free detection) and later restoring the
 *   original access rights.
 *
 * Input Parameters:
 *   vaddr - A virtual address within the page whose PTE should be saved.
 *           The address is internally aligned to a 4KB page boundary.
 *
 * Returned Value:
 *   The current PTE value (L2 small-page entry or L1 section entry).
 *   Returns 0 if the address is not mapped via an L2 page table or a
 *   1MB section.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
uint32_t mmu_save_page_pte(uint32_t vaddr)
{
	uint32_t page_addr = vaddr & ~SMALL_PAGE_MASK;
	uint32_t l1_entry = mmu_l1_getentry(page_addr);

	if ((l1_entry & PMD_TYPE_MASK) == PMD_TYPE_SECT) {
		/* 1MB section - permission is in the L1 entry directly */
		uint32_t *l1table = mmu_l1_pgtable_safe();
		uint32_t index = page_addr >> 20;
		return l1table[index];
	}

	if ((l1_entry & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
		/* L1 points to an L2 page table */
		uint32_t *l2table = (uint32_t *)(l1_entry & PMD_PTE_PADDR_MASK);
		uint32_t l2_index = (page_addr & 0x000FF000) >> 12;
		return l2table[l2_index];
	}

	return 0;					/* Not mapped */
}
#endif

/****************************************************************************
 * Name: mmu_set_page_no_access
 *
 * Description:
 *   Set a 4KB page (or 1MB section) to No-Access at any privilege level
 *   by clearing the AP (Access Permission) bits to 00.
 *
 *   When any code subsequently reads from or writes to this page, the MMU
 *   triggers a Data Abort.  This is the core mechanism for detecting
 *   use-after-free on page-aligned allocations.
 *
 *   The original PTE should be saved beforehand via mmu_save_page_pte()
 *   so that access can be restored later with mmu_restore_page_pte().
 *
 * Input Parameters:
 *   vaddr - A virtual address within the page to protect.  The address
 *           is internally aligned to a 4KB page boundary.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_set_page_no_access(uint32_t vaddr)
{
	uint32_t page_addr = vaddr & ~SMALL_PAGE_MASK;
	uint32_t l1_entry = mmu_l1_getentry(page_addr);

	if ((l1_entry & PMD_TYPE_MASK) == PMD_TYPE_SECT) {
		/* 1MB section - modify the L1 entry directly */
		uint32_t *l1table = mmu_l1_pgtable_safe();
		uint32_t index = page_addr >> 20;

		/* Clear ALL AP bits to 00 = No access at any privilege level.
		 * Must clear AP[2] (bit 15, PMD_SECT_AP2) in addition to
		 * AP[1:0] (bits 10-11, PMD_SECT_AP_MASK).  If AP[2] is left
		 * set, the section becomes privileged read-only instead of
		 * no-access, and read accesses would NOT fault. */
		l1table[index] = (l1table[index] & ~PMD_SECT_AP_MASK & ~PMD_SECT_AP2) | PMD_SECT_AP_NONE;


		cp15_clean_dcache_bymva((uint32_t)&l1table[index]);
		mmu_invalidate_region(page_addr, SECTION_SIZE);
		return;
	}

	if ((l1_entry & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
		/* L1 points to an L2 page table */
		uint32_t *l2table = (uint32_t *)(l1_entry & PMD_PTE_PADDR_MASK);
		uint32_t l2_index = (page_addr & 0x000FF000) >> 12;
		uint32_t dacr;

		__asm__ volatile("mrc p15, 0, %0, c3, c0, 0" : "=r"(dacr));
		lldbg("SET_NO_ACCESS: CPU%d DACR=0x%08x (dom0=%s) l1_entry=0x%08x (dom=%d)\n",
		      this_cpu(), dacr,
		      (dacr & 0x3) == 0 ? "NO_ACCESS" :
		      (dacr & 0x3) == 1 ? "CLIENT" :
		      (dacr & 0x3) == 3 ? "MANAGER" : "RESERVED",
		      l1_entry, (l1_entry >> 5) & 0xF);
		lldbg("SET_NO_ACCESS: vaddr=0x%08x l2table=0x%08x l2_index=%d pte_addr=0x%08x\n",
		      page_addr, (uint32_t)l2table, l2_index, (uint32_t)&l2table[l2_index]);
		lldbg("SET_NO_ACCESS: PTE before=0x%08x (AP=0x%x AP2=%d type=0x%x)\n",
		      l2table[l2_index], (l2table[l2_index] >> 4) & 3,
		      (l2table[l2_index] >> 9) & 1, l2table[l2_index] & 3);


		/* Set AP=00 (No Access at any privilege level).
		 * Must clear both AP[1:0] (bits 4-5, PTE_AP_MASK) and
		 * AP[2] (bit 9, PTE_AP2).  If AP[2] is left set, the page
		 * becomes privileged read-only instead of no-access, and
		 * read accesses would NOT fault. */
		l2table[l2_index] = (l2table[l2_index] & ~PTE_AP_MASK & ~PTE_AP2) | PTE_AP_NONE;

		lldbg("SET_NO_ACCESS: PTE after write=0x%08x (AP=0x%x type=0x%x)\n",
		      l2table[l2_index], (l2table[l2_index] >> 4) & 3, l2table[l2_index] & 3);

		cp15_clean_dcache_bymva((uint32_t)&l2table[l2_index]);

		lldbg("SET_NO_ACCESS: PTE after clean=0x%08x (AP=0x%x type=0x%x)\n",
		      l2table[l2_index], (l2table[l2_index] >> 4) & 3, l2table[l2_index] & 3);

		mmu_invalidate_tlb_bymva_is(page_addr);
	}
}
#endif

/****************************************************************************
 * Name: mmu_set_page_ro
 *
 * Description:
 *   Set a 4KB page (or 1MB section) to Read-Only at privileged level
 *   (AP = privileged read-only, user no access).
 *
 *   Any write to this page will trigger a Data Abort.  This can be used
 *   to protect read-only data structures (e.g. watchdog timer registers)
 *   from accidental corruption.
 *
 *   The original PTE should be saved beforehand via mmu_save_page_pte()
 *   so that write access can be restored later with
 *   mmu_restore_page_pte().
 *
 * Input Parameters:
 *   vaddr - A virtual address within the page to protect.  The address
 *           is internally aligned to a 4KB page boundary.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_set_page_ro(uint32_t vaddr)
{
	uint32_t page_addr = vaddr & ~SMALL_PAGE_MASK;
	uint32_t l1_entry = mmu_l1_getentry(page_addr);

	if ((l1_entry & PMD_TYPE_MASK) == PMD_TYPE_SECT) {
		/* 1MB section - modify the L1 entry directly */
		uint32_t *l1table = mmu_l1_pgtable_safe();
		uint32_t index = page_addr >> 20;

		/* Set AP to privileged read-only */
		l1table[index] = (l1table[index] & ~PMD_SECT_AP_MASK) | PMD_SECT_AP_R1;

		cp15_clean_dcache_bymva((uint32_t)&l1table[index]);
		mmu_invalidate_region(page_addr, SECTION_SIZE);
		return;
	}

	if ((l1_entry & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
		/* L1 points to an L2 page table */
		uint32_t *l2table = (uint32_t *)(l1_entry & PMD_PTE_PADDR_MASK);
		uint32_t l2_index = (page_addr & 0x000FF000) >> 12;
		uint32_t dacr;

		__asm__ volatile("mrc p15, 0, %0, c3, c0, 0" : "=r"(dacr));
		lldbg("SET_RO: CPU%d DACR=0x%08x (dom0=%s) l1_entry=0x%08x (dom=%d)\n",
		      this_cpu(), dacr,
		      (dacr & 0x3) == 0 ? "NO_ACCESS" :
		      (dacr & 0x3) == 1 ? "CLIENT" :
		      (dacr & 0x3) == 3 ? "MANAGER" : "RESERVED",
		      l1_entry, (l1_entry >> 5) & 0xF);
		lldbg("SET_RO: vaddr=0x%08x l2table=0x%08x l2_index=%d pte_addr=0x%08x\n",
		      page_addr, (uint32_t)l2table, l2_index, (uint32_t)&l2table[l2_index]);
		lldbg("SET_RO: PTE before=0x%08x (AP=0x%x AP2=%d type=0x%x)\n",
		      l2table[l2_index], (l2table[l2_index] >> 4) & 3,
		      (l2table[l2_index] >> 9) & 1, l2table[l2_index] & 3);


		/* Set AP to privileged read-only (PTE_AP_R1).
		 * Must clear both AP[1:0] (bits 4-5, PTE_AP_MASK) and
		 * AP[2] (bit 9, PTE_AP2) before setting the new value,
		 * because PTE_AP_R1 = PTE_AP_R12 = PTE_AP0 | PTE_AP2. */
		l2table[l2_index] = (l2table[l2_index] & ~PTE_AP_MASK & ~PTE_AP2) | PTE_AP_R1;

		lldbg("SET_RO: PTE after write=0x%08x (AP=0x%x type=0x%x)\n",
		      l2table[l2_index], (l2table[l2_index] >> 4) & 3, l2table[l2_index] & 3);

		cp15_clean_dcache_bymva((uint32_t)&l2table[l2_index]);

		lldbg("SET_RO: PTE after clean=0x%08x (AP=0x%x type=0x%x)\n",
		      l2table[l2_index], (l2table[l2_index] >> 4) & 3, l2table[l2_index] & 3);

		mmu_invalidate_tlb_bymva_is(page_addr);
	}
}
#endif

/****************************************************************************
 * Name: mmu_restore_page_pte

 *
 * Description:
 *   Restore a previously saved PTE for a given virtual address, reverting
 *   the page to its original access permissions.
 *
 *   This is the counterpart to mmu_save_page_pte().  After temporarily
 *   protecting a page with mmu_set_page_no_access() or mmu_set_page_ro(),
 *   call this function with the saved PTE to re-enable read/write access.
 *
 * Input Parameters:
 *   vaddr     - A virtual address within the page to restore.  The address
 *               is internally aligned to a 4KB page boundary.
 *   saved_pte - The PTE value previously returned by mmu_save_page_pte().
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_restore_page_pte(uint32_t vaddr, uint32_t saved_pte)
{
	uint32_t page_addr = vaddr & ~SMALL_PAGE_MASK;
	uint32_t l1_entry = mmu_l1_getentry(page_addr);

	if ((l1_entry & PMD_TYPE_MASK) == PMD_TYPE_SECT) {
		/* 1MB section - restore the L1 entry directly */
		uint32_t *l1table = mmu_l1_pgtable_safe();
		uint32_t index = page_addr >> 20;

		l1table[index] = saved_pte;

		cp15_clean_dcache_bymva((uint32_t)&l1table[index]);
		mmu_invalidate_region(page_addr, SECTION_SIZE);
		return;
	}

	if ((l1_entry & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
		/* L1 points to an L2 page table */
		uint32_t *l2table = (uint32_t *)(l1_entry & PMD_PTE_PADDR_MASK);
		uint32_t l2_index = (page_addr & 0x000FF000) >> 12;

		/* Restore the original PTE (with original R/W permissions) */
		l2table[l2_index] = saved_pte;

		cp15_clean_dcache_bymva((uint32_t)&l2table[l2_index]);
		mmu_invalidate_tlb_bymva_is(page_addr);
	}
}
#endif
