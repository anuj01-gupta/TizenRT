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
 * kernel/wdog/wdog.h
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

#ifndef __SCHED_WDOG_WDOG_H
#define __SCHED_WDOG_WDOG_H

/************************************************************************
 * Included Files
 ************************************************************************/

#include <tinyara/config.h>

#include <stdint.h>
#include <stdbool.h>

#include <tinyara/compiler.h>
#include <tinyara/wdog.h>

/************************************************************************
 * Pre-processor Definitions
 ************************************************************************/

/************************************************************************
 * Public Type Declarations
 ************************************************************************/

/************************************************************************
 * Public Variables
 ************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif

/* All watchdog globals live in one object.
 *
 * With CONFIG_WDOG_MMU_PROTECT this object is 4KB-aligned, occupies exactly
 * one page and is placed in the .wdog_data section, so a single second-level
 * page-table descriptor covers the pool, both list heads and the free
 * counter.  Protecting the watchdog data then costs one descriptor write, and
 * because the object fills its page no other data shares the permissions
 * being flipped.
 */

struct wdog_data_core_s {
	sq_queue_t freelist;			/* Head of the free wdog list */
	sq_queue_t activelist;			/* Head of the active wdog list */
	uint16_t nfree;			/* Number of wdogs on the free list */
	uint8_t pad0[2];
	struct wdog_s pool[CONFIG_PREALLOC_WDOGS];
};

/* The design rests on the data fitting in one page.  Check it at compile
 * time (a negative array size) so a wrong CONFIG_PREALLOC_WDOGS fails the
 * build instead of quietly making a neighbour page read-only; the linker
 * ASSERT and wd_mmu_protect_init() repeat the check. */

/* Upper bound on the queue this design supports, checked against the page
 * size at compile time: 96 slots is a generous worst case, and any value
 * above it would push the consolidated object past one page and make a
 * neighbour read-only.  The linker ASSERT and wd_mmu_protect_init() repeat
 * the check at link and at boot. */

#define WDOG_POOL_MAX  96

typedef char wdog_data_fits_one_page[(sizeof(struct wdog_data_core_s) <= 4096) ? 1 : -1];

#if CONFIG_PREALLOC_WDOGS > WDOG_POOL_MAX
#  error "CONFIG_PREALLOC_WDOGS exceeds WDOG_POOL_MAX: watchdog data would not fit one 4KB page"
#endif

struct wdog_data_s {
	struct wdog_data_core_s core;
	uint8_t pad_page[4096 - sizeof(struct wdog_data_core_s)];
};

extern struct wdog_data_s g_wdog_data;

/* The names the subsystem uses.  The globals are members of the consolidated
 * object, so no other file needs to change. */

#define g_wdfreelist			g_wdog_data.core.freelist
#define g_wdactivelist		g_wdog_data.core.activelist
#define g_wdnfree				g_wdog_data.core.nfree
#define g_wdpool				g_wdog_data.core.pool





/************************************************************************
 * Public Function Prototypes
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

bool wd_is_prealloc(WDOG_ID wdog);

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

void weak_function wd_initialize(void);

/****************************************************************************
 * Name: wd_timer
 *
 * Description:
 *   This function is called from the timer interrupt handler to determine
 *   if it is time to execute a watchdog function.  If so, the watchdog
 *   function will be executed in the context of the timer interrupt
 *   handler.
 *
 * Parameters:
 *   ticks - If CONFIG_SCHED_TICKLESS is defined then the number of ticks
 *     in the interval that just expired is provided.  Otherwise,
 *     this function is called on each timer interrupt and a value of one
 *     is implicit.
 *
 * Return Value:
 *   If CONFIG_SCHED_TICKLESS is defined then the number of ticks for the
 *   next delay is provided (zero if no delay).  Otherwise, this function
 *   has no returned value.
 *
 * Assumptions:
 *   Called from interrupt handler logic with interrupts disabled.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
unsigned int wd_timer(int ticks);
#else
void wd_timer(void);
#endif
#ifdef CONFIG_SCHED_TICKSUPPRESS
void wd_timer_nohz(clock_t ticks);
#endif
#ifdef CONFIG_DEBUG
void wd_corruption_dbg(struct wdog_s *wdog);
#else
#define wd_corruption_dbg(wdog) (0)
#endif
/****************************************************************************
 * Name: wd_recover
 *
 * Description:
 *   This function is called from task_recover() when a task is deleted via
 *   task_delete() or via pthread_cancel(). It checks if the deleted task
 *   is waiting for a timed event and if so cancels the timeout
 *
 * Inputs:
 *   tcb - The TCB of the terminated task or thread
 *
 * Return Value:
 *   None.
 *
 * Assumptions:
 *   This function is called from task deletion logic in a safe context.
 *
 ****************************************************************************/

struct tcb_s;
void wd_recover(FAR struct tcb_s *tcb);

/****************************************************************************
 * MMU Page Protection for the Watchdog Data Page
 *
 * When CONFIG_WDOG_MMU_PROTECT is enabled, the second-level descriptor of
 * the page that holds g_wdog_data is kept Read-Only.  Wdog code must call
 * wd_mmu_write_begin() before modifying any wdog data, and
 * wd_mmu_write_end() afterwards; the implementation is wd_mmu_protect.c.
 *
 * All of it lives in wd_mmu_protect.c - the page is mapped through the
 * second level at boot and its table is claimed for good, so no split and
 * no descriptor work happens on the write path.
 *
 * When disabled, these are no-ops.
 ****************************************************************************/


#ifdef CONFIG_WDOG_MMU_PROTECT
int  wd_mmu_protect_init(void);
int  wd_mmu_write_begin(void);
int  wd_mmu_write_end(void);
bool wd_mmu_is_protected(void);
void wd_mmu_get_window_state(FAR bool *open, FAR uint8_t *cpu, FAR uint8_t *depth);
uint32_t wd_mmu_get_failures(void);
uint32_t wd_mmu_get_flips(void);
#else
#define wd_mmu_protect_init()                OK
#define wd_mmu_write_begin()                 OK
#define wd_mmu_write_end()                   OK
#define wd_mmu_is_protected()                (false)
#define wd_mmu_get_failures()                (0)
#define wd_mmu_get_flips()                   (0)
#define wd_mmu_get_window_state(a, b, c)     do { *(a) = false; *(b) = 0; *(c) = 0; } \
                                               while (0)
#endif

/* Read-only view of the active watchdog list, for diagnostics outside the
 * subsystem (test drivers).  Available in every configuration. */

FAR struct sq_queue_s *wd_activelist(void);

#undef EXTERN

#ifdef __cplusplus
}
#endif

#endif							/* __SCHED_WDOG_WDOG_H */
