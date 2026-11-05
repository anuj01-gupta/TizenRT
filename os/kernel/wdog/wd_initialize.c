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
 * kernel/wdog/wd_initialize.c
 *
 *   Copyright (C) 2007, 2009, 2014 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************/

/************************************************************************
 * Included Files
 ************************************************************************/

#include <tinyara/config.h>

#include <stdint.h>
#include <queue.h>
#include <stdlib.h>
#include <string.h>

#include "wdog/wdog.h"

#ifdef CONFIG_WDOG_MMU_PROTECT
#include <tinyara/irq.h>
#endif

/* The permission mechanism itself lives in wd_mmu_protect.c: it flips the
 * AP bits of the one L2 descriptor that covers the watchdog data page.
 */





/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Private Type Declarations
 ************************************************************************/

/************************************************************************
 * Public Variables
 ************************************************************************/

/* All watchdog globals live in one object so that, when
 * CONFIG_WDOG_MMU_PROTECT is enabled, a single L2 descriptor covers every
 * one of them: the pool, both list heads and the free count share one 4KB
 * page of the .wdog_data section (the linker script ASSERTs that it fits).
 *
 * wdog/wdog.h aliases g_wdpool / g_wdfreelist / g_wdactivelist / g_wdnfree
 * to the members below, so the rest of the subsystem is unchanged.
 */

#ifdef CONFIG_WDOG_MMU_PROTECT
struct wdog_data_s g_wdog_data
	__attribute__((aligned(4096), section(".wdog_data")));
#else
struct wdog_data_s g_wdog_data;
#endif

/************************************************************************
 * Private Functions
 ************************************************************************/

/* The watchdog MMU permission mechanism - the write window, the descriptor
 * flip and the TLB broadcast - lives in wd_mmu_protect.c.
 */






/* the end of the watchdog-protected region marker is not needed here */





/************************************************************************
 * Public Functions
 ************************************************************************/

/************************************************************************
 * Name: wd_is_prealloc
 *
 * Description:
 * This function checks if the wdog is pre- allocated or not
 *
 * Parameters:
 *   wdog - the address of wdog (WDOG_ID)
 *
 * Return Value:
 *   true  - if wdog is preallocated
 *   false - otherwise
 *
 ************************************************************************/

bool wd_is_prealloc(WDOG_ID wdog)
{
	uintptr_t wdog_ptr = (uintptr_t)wdog;
	uintptr_t start = (uintptr_t)(&g_wdpool[0]);
	uintptr_t end = (uintptr_t)(&g_wdpool[CONFIG_PREALLOC_WDOGS - 1]);

	if (end < start) {
		start = start ^ end;
		end = start ^ end;
		start = start ^ end;
	}

	return (wdog_ptr >= start) && (wdog_ptr <= end) && (((wdog_ptr - start) % sizeof(struct wdog_s)) == 0);
}

/************************************************************************
 * Name: wd_activelist
 *
 * Description:
 * This function returns the list of running watchdogs, read-only, for
 * diagnostics outside the watchdog subsystem.
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   The head of the active watchdog list
 *
 ************************************************************************/

FAR struct sq_queue_s *wd_activelist(void)
{
	return &g_wdog_data.core.activelist;
}

/************************************************************************
 * Name: wd_initialize
 *
 * Description:
 * This function initializes the watchdog data structures
 *
 * Parameters:
 *   None
 *
 * Return Value:
 *   None
 *
 * Assumptions:
 *   This function must be called early in the initialization sequence
 *   before the timer interrupt is attached and before any watchdog
 *   services are used.
 *
 ************************************************************************/

void wd_initialize(void)
{
	FAR struct wdog_s *wdog;
	int i;

	wdog = g_wdpool;

	/* Initialize watchdog lists */
	sq_init(&g_wdfreelist);
	sq_init(&g_wdactivelist);

	/* The g_wdfreelist must be loaded at initialization time to hold the
	 * configured number of watchdogs.
	 */
	for (i = 0; i < CONFIG_PREALLOC_WDOGS; i++) {
		sq_addlast((FAR sq_entry_t *)wdog++, &g_wdfreelist);
	}

	/* All watchdogs are free */
	g_wdnfree = CONFIG_PREALLOC_WDOGS;

	/* Protect the watchdog data page.  After this call the page holding
	 * g_wdog_data is read-only and every write has to be bracketed by
	 * wd_mmu_write_begin() / wd_mmu_write_end().  Nothing has touched the
	 * watchdog data since the loop above, so this is the last moment at
	 * which the page can be closed without opening a window.
	 *
	 * Failing to protect the page is reported and is not fatal: the
	 * subsystem then runs unprotected, which is better than running
	 * half-protected with a window that is never closed.
	 */

	if (wd_mmu_protect_init() != OK) {
		lldbg("WDOG_MMU: watchdog data page NOT protected\n");
	}
}
