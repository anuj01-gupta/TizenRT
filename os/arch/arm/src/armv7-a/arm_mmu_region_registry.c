/****************************************************************************
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
 * arch/arm/src/armv7-a/arm_mmu_region_registry.c
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
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#ifdef CONFIG_MMU_FAULT_REPORT

#include <stdint.h>
#include <stddef.h>
#include <errno.h>

#include <tinyara/irq.h>
#include <tinyara/mmu.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MMU_REGION_MAX   16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mmu_protected_region_s {
	const char *name;	/* Region name shown in the fault report */
	uintptr_t vaddr;	/* Region start (virtual address) */
	size_t size;		/* Region size in bytes */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mmu_protected_region_s g_mmu_regions[MMU_REGION_MAX];
static int g_mmu_region_count;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmu_protect_region_register
 *
 * Description:
 *   Register a named protected region so the MMU fault report can identify
 *   which protected region a faulting address belongs to (R2 mitigation,
 *   design item M0-6).  Re-registering the identical region (same name,
 *   address and size) is idempotent.
 *
 *   Runs in task context with interrupts disabled for the table update;
 *   safe against concurrent SMP registration.
 *
 * Input Parameters:
 *   name  - Region name; the string must have static storage duration.
 *   vaddr - Region start virtual address.
 *   size  - Region size in bytes.
 *
 * Returned Value:
 *   OK (0) on success, -EINVAL on bad arguments, -ENOSPC when the table
 *   is full, -EEXIST when a different region with the same name exists.
 *
 ****************************************************************************/

int mmu_protect_region_register(const char *name, uintptr_t vaddr, size_t size)
{
	irqstate_t flags;
	int i;

	if (name == NULL || size == 0) {
		return -EINVAL;
	}

	flags = enter_critical_section();

	for (i = 0; i < g_mmu_region_count; i++) {
		if (g_mmu_regions[i].name == name) {
			if (g_mmu_regions[i].vaddr == vaddr &&
			    g_mmu_regions[i].size == size) {
				/* Idempotent re-registration of the same region */
				leave_critical_section(flags);
				return OK;
			}

			leave_critical_section(flags);
			return -EEXIST;
		}
	}

	if (g_mmu_region_count >= MMU_REGION_MAX) {
		leave_critical_section(flags);
		return -ENOSPC;
	}

	g_mmu_regions[g_mmu_region_count].name = name;
	g_mmu_regions[g_mmu_region_count].vaddr = vaddr;
	g_mmu_regions[g_mmu_region_count].size = size;
	g_mmu_region_count++;

	leave_critical_section(flags);
	return OK;
}

/****************************************************************************
 * Name: mmu_protect_region_lookup
 *
 * Description:
 *   Look up the registered region containing a virtual address.  Safe to
 *   call from the data abort handler: no allocation, no locks, scan over
 *   a fixed-size table only.  A concurrent registration can only append;
 *   existing entries are never modified.
 *
 * Input Parameters:
 *   vaddr - Virtual address to look up.
 *   psize - Optional (may be NULL) location to store the region size.
 *
 * Returned Value:
 *   The region name, or NULL if no registered region contains vaddr.
 *
 ****************************************************************************/

const char *mmu_protect_region_lookup(uintptr_t vaddr, size_t *psize)
{
	int i;

	for (i = 0; i < g_mmu_region_count; i++) {
		if (vaddr >= g_mmu_regions[i].vaddr &&
		    vaddr - g_mmu_regions[i].vaddr < g_mmu_regions[i].size) {
			if (psize != NULL) {
				*psize = g_mmu_regions[i].size;
			}

			return g_mmu_regions[i].name;
		}
	}

	return NULL;
}

#endif /* CONFIG_MMU_FAULT_REPORT */