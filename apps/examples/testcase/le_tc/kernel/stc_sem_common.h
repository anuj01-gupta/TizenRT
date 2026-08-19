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

/// @file stc_sem_common.h
/// @brief Shared harness for the semaphore scenario test tier (stc_).
///
/// Phase 1 of the Priority Inheritance and Semaphore Recovery Scenario Test
/// Catalogue.  The harness exists so that each scenario reduces to a short,
/// readable sequence of "make this state true, then assert a priority".
///
/// Design rules enforced here (catalogue section 6):
///
///  Rule 1  The harness outranks every actor.  stc_harness_begin() raises the
///          calling task to STC_PRIO_HARNESS, so the harness is scheduled the
///          instant the state it samples becomes true.
///  Rule 2  Sequencing uses handshakes and exact predicates, never sleep().
///          An actor announces progress with stc_stage(); the harness waits
///          for it with stc_wait_stage().  The one state an actor cannot
///          announce - "I am now blocked" - is observed through the semaphore
///          count by stc_wait_count().
///  Rule 3  No printf inside a measurement window.  Actors never print.
///  Rule 9  Actors are pinned to one CPU on SMP builds, so that an exact
///          priority at an exact moment is well defined.  Family I covers the
///          unpinned case deliberately.
///  Rule 11 Every wait is bounded.  A scenario that would hang fails instead.
///
/// Two properties of this design are load-bearing rather than incidental:
///
///  1. Progress is reported per actor, not through one shared semaphore.  A
///     single post can make two actors runnable at once, so any shared
///     acknowledgement would arrive in an order the harness cannot predict.
///     Per-actor stages have no such ambiguity.
///
///  2. The handshake semaphores are created with sem_init(..., 0), which sets
///     FLAGS_SIGSEM.  A signalling semaphore never records a holder, so the
///     harness machinery cannot boost anybody and cannot perturb the very
///     priorities the scenarios measure.

#ifndef __EXAMPLES_TESTCASE_KERNEL_STC_SEM_COMMON_H
#define __EXAMPLES_TESTCASE_KERNEL_STC_SEM_COMMON_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdbool.h>
#include <stdint.h>
#include <sched.h>
#include <semaphore.h>
#include <sys/types.h>
#include <tinyara/os_api_test_drv.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Actor priorities.  Every band is separated by more than the largest boost
 * any Family A scenario applies, and the harness sits above all of them.
 */

#define STC_PRIO_HARNESS        200
#define STC_PRIO_EXTRA          160	/* T_X : highest waiter              */
#define STC_PRIO_HIGH           140	/* T_H : the blocking waiter         */
#define STC_PRIO_MID            130	/* second waiter                     */
#define STC_PRIO_INVERTER       120	/* T_M : CPU burner, touches no sem  */
#define STC_PRIO_LOW3           110	/* third holder of a counting sem    */
#define STC_PRIO_LOW2           105	/* second holder                     */
#define STC_PRIO_LOW            100	/* T_L : the initial holder          */

#define STC_STACKSIZE           2048

/* Maximum number of actors one scenario may spawn */

#define STC_MAX_ACTORS          8

/* Bound on every harness wait.  This is a failure timeout, not a tolerance:
 * no oracle depends on its value, and a scenario that reaches it has already
 * failed.  It only guarantees the failure is reported rather than hanging the
 * whole run (rule 11).
 */

#define STC_TIMEOUT_MS          3000

/* Retry cadence for the exact-predicate waits. */

#define STC_POLL_US             1000

/* Grace period stc_teardown() allows an actor to return before it is treated
 * as leaked.  An actor that has just posted its last semaphore is only a few
 * instructions from its exit point, but it has not necessarily executed them
 * by the time the harness observes the count.  Without this grace the leak
 * report would be a race rather than a signal.  It is short enough that a
 * genuinely stuck actor is still reported promptly.
 */

#define STC_GRACE_MS            200

/* Order trace: one character per event, no locks, no allocation, no printf. */

#define STC_TRACE_MAX           64

/* Stage value an actor reaches when it returns. */

#define STC_STAGE_DONE          0xff

/****************************************************************************
 * Public Types
 ****************************************************************************/

struct stc_actor_s {
	pid_t pid;					/* 0 when the slot is unused             */
	sem_t go;					/* harness -> actor, signalling flavour   */
	volatile uint8_t stage;		/* monotonically increasing progress mark */
	volatile bool spawned;		/* slot holds a live task                 */
	volatile bool finished;		/* actor reached its exit point           */
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern struct stc_actor_s g_stc_actor[STC_MAX_ACTORS];
extern volatile uint16_t g_stc_trace_idx;
extern char g_stc_trace[STC_TRACE_MAX];

/* Set by the harness, polled by the CPU-burner actor of SCN-PI-04. */

extern volatile bool g_stc_spin_stop;
extern volatile uint32_t g_stc_spin_count;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Record one ordering event. */

void stc_trace(char ev);

/* Harness lifecycle ------------------------------------------------------- */

/* Raise the calling task to STC_PRIO_HARNESS and pin it, remembering the
 * previous settings.  Returns OK or ERROR.
 */

int stc_harness_begin(void);

/* Restore the calling task's original priority and affinity. */

void stc_harness_end(void);

/* Prepare the actor table for one scenario.  Call at the start of each. */

int stc_reset(void);

/* Delete any actor still alive, release the harness semaphores, and return
 * how many actors had to be force-deleted.  Non-zero means the scenario did
 * not complete its own sequence, which is itself a failure (rule 7).
 */

int stc_teardown(void);

/* Actor control ----------------------------------------------------------- */

/* Create an actor in the given slot at the given priority, pinned on SMP.
 * Returns the pid, or ERROR.
 */

pid_t stc_spawn(const char *name, int prio, main_t entry, int slot);

/* Release the actor waiting in slot. */

void stc_go(int slot);

/* Called by an actor: block until the harness releases this slot. */

void stc_wait_go(int slot);

/* Called by an actor: announce that it has reached the given stage. */

void stc_stage(int slot, uint8_t stage);

/* Called by an actor as its last statement. */

void stc_actor_done(int slot);

/* Drop a slot from the actor table without deleting anything.  Used by the
 * scenarios that delete a blocked actor on purpose: that actor never reaches
 * its exit point, and without this it would be reported as a leaked actor by
 * the very teardown check that exists to catch accidental leaks.
 */

void stc_actor_forget(int slot);

/* Harness observation ------------------------------------------------------ */

/* Bounded wait until the actor in slot reaches at least the given stage.
 * Returns OK, or ERROR on timeout.
 */

int stc_wait_stage(int slot, uint8_t stage);

/* Bounded wait until sem's count equals expected.  This is how the harness
 * learns that a waiter has actually blocked - an actor cannot announce that
 * itself.  It is also how the harness learns that a post has fully completed:
 * sem_post() increments the count, hands it to a waiter and runs the priority
 * restore all inside one critical section, so a count observed from outside
 * that section implies the restore has already run.  Returns OK or ERROR.
 */

int stc_wait_count(sem_t *sem, int expected);

/* Bounded wait for the actor in slot to reach its exit point. */

int stc_wait_finished(int slot);

/* Effective priority of pid, or ERROR. */

int stc_getprio(pid_t pid);

/* Current count of sem, or INT32_MIN on failure. */

int stc_getcount(sem_t *sem);

/* Read the kernel's holder book-keeping for sem through TESTIOC_SEM_SNAPSHOT.
 * Returns OK, or ERROR when the driver is unavailable.
 *
 * Two rules on its use, from catalogue section 3.2.  It must never be called
 * inside a measurement window, because the ioctl takes a critical section.
 * And no scenario may rest its *hard* oracle on it where a priority based
 * oracle exists: priorities are observable on any configuration, this is not.
 */

int stc_snapshot(sem_t *sem, struct sem_snapshot_s *snap);

/* Counts owned by pid in the snapshot, or 0 when it holds no record. */

int stc_snapshot_counts(struct sem_snapshot_s *snap, pid_t pid);

#endif							/* __EXAMPLES_TESTCASE_KERNEL_STC_SEM_COMMON_H */
