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

/// @file stc_sem_monitor.h
/// @brief SCN-MON-01, the boost-leak invariant monitor.
///
/// Phase 2 of the Priority Inheritance and Semaphore Recovery Scenario Test
/// Catalogue.  This is the generic detector: it catches any inheritance defect
/// that leaves a task above its base priority, without needing to anticipate
/// which branch of the restore path caused it.
///
/// Invariant
///   No task runs above its base priority unless some semaphore it holds has
///   a higher-priority waiter.
///
/// Implementation note - a correction to catalogue section 8.2
///   Section 8.2 states that this monitor reuses the existing
///   TESTIOC_SCHED_FOREACH command.  It cannot: that command takes a kernel
///   function pointer and calls sched_foreach() with it, so it enumerates
///   tasks only for code already running in the kernel.  Handing it a
///   user-space callback is not possible under CONFIG_APP_BINARY_SEPARATION,
///   and the command returns nothing to the caller in any case.
///
///   The monitor therefore works from a registry instead.  The harness already
///   creates every actor, so it already knows each actor's true base priority
///   and every semaphore in play - which is strictly better information than
///   a TCB walk would give, because it needs no kernel access at all.
///   stc_spawn() registers each actor automatically, so any scenario built on
///   the harness is monitored without doing anything.
///
///   The cost of this choice is scope: the monitor watches the scenarios'
///   actors, not every task in the system.  A leak in an unrelated kernel
///   thread is out of range.  That is an acceptable limit here, since every
///   defect class the catalogue attributes to this monitor manifests on a task
///   the harness created.  Widening it to all tasks needs a new read-only
///   snapshot command, which belongs with the Phase 4 driver work.
///
/// Justification for the "no waiter anywhere" test
///   A boost is justified when some registered semaphore has a waiter, which
///   is exactly the condition sem_getvalue() < 0 detects.  The monitor treats
///   a waiter on *any* registered semaphore as justifying *any* boost.  That
///   is deliberately conservative: it can miss a leak that overlaps an
///   unrelated wait, but it can never invent one.  For a monitor intended to
///   run alongside every other scenario, a false positive would be far more
///   expensive than a missed detection, which the targeted Family B scenarios
///   cover anyway.

#ifndef __EXAMPLES_TESTCASE_KERNEL_STC_SEM_MONITOR_H
#define __EXAMPLES_TESTCASE_KERNEL_STC_SEM_MONITOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdbool.h>
#include <stdint.h>
#include <semaphore.h>
#include <sys/types.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Above the harness, so a sample is never delayed by scenario work. */

#define STC_PRIO_MONITOR        250

/* Sampling period. */

#define STC_MON_PERIOD_US       10000

/* Consecutive boosted samples required before an episode counts as a
 * violation.
 *
 * K = 3, i.e. 30 ms.  A single sample can legitimately catch the window
 * between up_unblock_task() and the restore pass, during which a holder is
 * boosted with no justifying waiter still linked.  That window is bounded by
 * the sched_lock() region in sem_post(), a few hundred instructions.  Three
 * 10 ms samples is four orders of magnitude above it, and still two orders
 * below the smallest leak that matters: a boost that clears within 30 ms is
 * not a leak, and one that persists is, by construction, permanent.
 */

#define STC_MON_K               3

#define STC_MON_MAX_ACTORS      8
#define STC_MON_MAX_SEMS        4

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Forget every registered actor and semaphore, and reset the violation
 * count.  Called from stc_reset(), so each scenario starts clean.
 */

void stc_mon_clear(void);

/* Register an actor and the base priority it was created with.  Called
 * automatically by stc_spawn().
 */

void stc_mon_register_actor(pid_t pid, int base_prio);

/* Register a semaphore that scenarios use, so the monitor can tell whether a
 * boost is currently justified.
 */

void stc_mon_register_sem(sem_t *sem);

/* Start and stop the monitor task.  Both are idempotent. */

int stc_mon_start(void);
void stc_mon_stop(void);

/* Violations observed since the last stc_mon_clear(). */

uint32_t stc_mon_violations(void);

/* SCN-MON-01 entry point. */

int stc_sem_monitor_main(void);

#endif							/* __EXAMPLES_TESTCASE_KERNEL_STC_SEM_MONITOR_H */
