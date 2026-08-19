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

/// @file stc_sem_inherit.c
/// @brief Families A and B of the Priority Inheritance and Semaphore Recovery
///        Scenario Test Catalogue - boost and restore, and the cross
///        semaphore and multi holder restore paths.
///
/// Family A - boost and restore
///   SCN-PI-01  boost on block, restore on post
///   SCN-PI-02  no boost when the waiter does not outrank the holder
///   SCN-PI-03  the holder tracks the maximum waiter priority
///   SCN-PI-04  a classic inversion is bounded by the critical section
///   SCN-PI-05  every holder of a counting semaphore is boosted
///   SCN-PI-06  every holder is restored, not only the one that posted
///
/// Family B - cross semaphore and multi holder restore
///   SCN-PIX-01 a holder that still owes another semaphore stays boosted
///   SCN-PIX-02 a spent holder entry stops boosting its holder
///   SCN-PIX-03 the just awakened waiter does not re-boost its own holder
///   SCN-PIX-04 an ambiguous release leaves every holder record intact
///   SCN-PIX-05 an unambiguous release frees the holder record
///
/// Family B is the subtle half.  Each scenario isolates one branch of
/// sem_restoreholderprio(), and each has a defect signature that no API level
/// test can express, because the only symptom is a priority.
///
/// These are the base cases.  Nothing in the tree asserts today that a boost
/// happens at all: the only priority inheritance test that exists,
/// tc_semaphore_sem_setprotocol(), checks a return code and never reads a
/// priority, so a kernel with sem_boostpriority() stubbed out passes it
/// unchanged.  Every oracle below is an exact integer equality on a priority
/// or on a count; none of them carries a timing tolerance.
///
/// Note on style: every value asserted is first stored in a local.  The
/// TC_ASSERT macros evaluate their argument a second time when building the
/// failure message, and several of the expressions here either spawn a task or
/// wait up to STC_TIMEOUT_MS.

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>
#include <semaphore.h>
#include <sys/types.h>

#include "tc_common.h"
#include "tc_internal.h"
#include "stc_sem_common.h"
#include "stc_sem_monitor.h"

#ifdef CONFIG_PRIORITY_INHERITANCE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Actor stages.  Monotonic per actor, so the harness never has to guess which
 * of two runnable actors reported first.
 */

#define STAGE_STARTED           1
#define STAGE_HELD              2	/* holder acquired a count             */
#define STAGE_POSTED            3	/* holder released it                  */

#define STAGE_ACQUIRED          2	/* waiter obtained the count           */

/* Slot assignment, uniform across the scenarios in this file. */

#define SLOT_HOLDER             0
#define SLOT_HOLDER2            1
#define SLOT_HOLDER3            2
#define SLOT_WAITER             3
#define SLOT_WAITER2            4
#define SLOT_SPINNER            5
#define SLOT_POSTER             6
#define SLOT_TAKER              7

/* Length of the SCN-PI-04 critical section, in loop iterations.  Work, not a
 * sleep: a sleeping holder would release the CPU, and the scenario would then
 * pass even with inheritance removed.
 */

#define CRITICAL_WORK           200000

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* The semaphore under test.  Re-initialised by every scenario. */

static sem_t g_target;

/* Second semaphore, used by the Family B scenarios that need a holder to owe
 * two semaphores at once.
 */

static sem_t g_target2;
static bool g_target2_valid;

/* Sink for the critical-section work loop, so it cannot be optimised away. */

static volatile uint32_t g_work_sink;

/****************************************************************************
 * Private Functions - actors
 ****************************************************************************/

/****************************************************************************
 * Name: holder_actor
 *
 * Description:
 *   Take one count, announce it, wait for the harness, release it.  While
 *   waiting for the harness this actor is blocked on its own signalling
 *   semaphore, which records no holder and so cannot itself boost anybody:
 *   the only boost it can receive is the one under test.
 *
 ****************************************************************************/

static int holder_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	if (sem_wait(&g_target) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_trace('L');
	stc_stage(slot, STAGE_HELD);

	stc_wait_go(slot);

	(void)sem_post(&g_target);
	stc_trace('l');
	stc_stage(slot, STAGE_POSTED);

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Name: holder_work_actor
 *
 * Description:
 *   As holder_actor(), but with a bounded work loop inside the critical
 *   section.  Used by SCN-PI-04, where the point is that the holder needs CPU
 *   time it can only obtain by inheriting the waiter's priority.
 *
 ****************************************************************************/

static int holder_work_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);
	uint32_t i;

	(void)argc;

	if (sem_wait(&g_target) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_trace('L');
	stc_stage(slot, STAGE_HELD);

	stc_wait_go(slot);

	for (i = 0; i < CRITICAL_WORK; i++) {
		g_work_sink++;
	}

	(void)sem_post(&g_target);
	stc_trace('l');
	stc_stage(slot, STAGE_POSTED);

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Name: waiter_actor
 *
 * Description:
 *   Block on the semaphore, announce the acquisition, wait for the harness,
 *   release it.  The actor cannot announce "I am now blocked", so the harness
 *   observes that transition through the semaphore count instead.
 *
 ****************************************************************************/

static int waiter_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	stc_stage(slot, STAGE_STARTED);
	stc_trace('h');

	if (sem_wait(&g_target) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_trace('H');
	stc_stage(slot, STAGE_ACQUIRED);

	stc_wait_go(slot);

	(void)sem_post(&g_target);

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Name: spinner_actor
 *
 * Description:
 *   The medium-priority inverter of SCN-PI-04.  It never touches the
 *   semaphore; its only job is to occupy the CPU the holder needs.
 *
 ****************************************************************************/

static int spinner_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	stc_stage(slot, STAGE_STARTED);
	stc_trace('M');

	while (!g_stc_spin_stop) {
		g_stc_spin_count++;
	}

	stc_actor_done(slot);
	return OK;
}


/****************************************************************************
 * Name: holder_two_sems_actor
 *
 * Description:
 *   Take one count of each semaphore, then release them one at a time, each
 *   release gated by its own stc_go().  Used by SCN-PIX-01 and SCN-PIX-02,
 *   where the whole point is what the holder's priority is between the two
 *   releases.
 *
 ****************************************************************************/

static int holder_two_sems_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	if (sem_wait(&g_target) != OK || sem_wait(&g_target2) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_HELD);

	stc_wait_go(slot);
	(void)sem_post(&g_target);
	stc_stage(slot, STAGE_POSTED);

	stc_wait_go(slot);
	(void)sem_post(&g_target2);

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Name: holder_two_counts_actor
 *
 * Description:
 *   Take two counts of the same semaphore and release them one at a time.
 *   Used by SCN-PIX-03: after the first release the holder still owns a count,
 *   so its holder entry is still examined by the restore pass - which is what
 *   makes the exclusion of the awakened waiter observable.
 *
 ****************************************************************************/

static int holder_two_counts_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	if (sem_wait(&g_target) != OK || sem_wait(&g_target) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_HELD);

	stc_wait_go(slot);
	(void)sem_post(&g_target);
	stc_stage(slot, STAGE_POSTED);

	stc_wait_go(slot);
	(void)sem_post(&g_target);

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Name: holder_no_post_actor
 *
 * Description:
 *   Take one count and never release it.  Used by SCN-PIX-05, where a
 *   non-holder returns the count on this actor's behalf; posting again here
 *   would inject a second count and corrupt the arithmetic.
 *
 ****************************************************************************/

static int holder_no_post_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	if (sem_wait(&g_target) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_HELD);
	stc_wait_go(slot);

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Name: poster_actor
 *
 * Description:
 *   Posts a semaphore it never waited on.  POSIX allows this for counting
 *   semaphores, and it is the case sem_releaseholder() has to disambiguate.
 *
 ****************************************************************************/

static int poster_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	stc_stage(slot, STAGE_STARTED);
	stc_wait_go(slot);

	(void)sem_post(&g_target);
	stc_stage(slot, STAGE_POSTED);

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Name: waiter2_actor
 *
 * Description:
 *   waiter_actor() for the second semaphore.
 *
 ****************************************************************************/

static int waiter2_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	stc_stage(slot, STAGE_STARTED);

	if (sem_wait(&g_target2) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_ACQUIRED);
	stc_wait_go(slot);

	(void)sem_post(&g_target2);

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Private Functions - helpers
 *
 * These return int rather than using TC_ASSERT, because the TC_ASSERT macros
 * expand to a bare return and may only be used from the void scenario
 * functions themselves.
 ****************************************************************************/

/****************************************************************************
 * Name: scenario_begin
 ****************************************************************************/

static int scenario_begin(int initial_count)
{
	if (stc_reset() != OK) {
		return ERROR;
	}

	if (sem_init(&g_target, 0, initial_count) != OK) {
		return ERROR;
	}

	/* Let the boost-leak monitor tell a justified boost from a leaked one. */

	stc_mon_register_sem(&g_target);

	return OK;
}

/****************************************************************************
 * Name: scenario_end
 *
 * Description:
 *   Rule 7: deterministic teardown.  Returns the number of actors that had to
 *   be force-deleted, so the scenario can assert that it is zero.
 *
 ****************************************************************************/

static int scenario_end(void)
{
	int leaked = stc_teardown();

	(void)sem_destroy(&g_target);

	if (g_target2_valid) {
		(void)sem_destroy(&g_target2);
		g_target2_valid = false;
	}

	return leaked;
}

/****************************************************************************
 * Name: scenario_begin2
 *
 * Description:
 *   As scenario_begin(), plus a second semaphore.  Both are registered with
 *   the boost-leak monitor, so a waiter on either one justifies a boost.
 *
 ****************************************************************************/

static int scenario_begin2(int count1, int count2)
{
	if (scenario_begin(count1) != OK) {
		return ERROR;
	}

	if (sem_init(&g_target2, 0, count2) != OK) {
		return ERROR;
	}

	g_target2_valid = true;
	stc_mon_register_sem(&g_target2);

	return OK;
}

/****************************************************************************
 * Name: hold_and_block
 *
 * Description:
 *   Common opening: one holder at priority 100 takes the semaphore, then one
 *   waiter of the given priority blocks on it.  On return the holder is parked
 *   on its go semaphore and the waiter is in TSTATE_WAIT_SEM, so nothing can
 *   move until the harness releases someone and the sample that follows is
 *   unambiguous.
 *
 ****************************************************************************/

static int hold_and_block(int waiter_prio, int waiter_slot)
{
	if (stc_spawn("stc_hold", STC_PRIO_LOW, holder_actor, SLOT_HOLDER) == (pid_t)ERROR) {
		return ERROR;
	}

	if (stc_wait_stage(SLOT_HOLDER, STAGE_HELD) != OK) {
		return ERROR;
	}

	if (stc_spawn("stc_wait", waiter_prio, waiter_actor, waiter_slot) == (pid_t)ERROR) {
		return ERROR;
	}

	return stc_wait_count(&g_target, -1);
}

/****************************************************************************
 * Name: release_waiter
 *
 * Description:
 *   Wait for the waiter in the given slot to acquire the count, then release
 *   it and wait for the semaphore to reach its final value.
 *
 ****************************************************************************/

static int release_waiter_on(int slot, sem_t *sem, int final_count)
{
	if (stc_wait_stage(slot, STAGE_ACQUIRED) != OK) {
		return ERROR;
	}

	stc_go(slot);

	return stc_wait_count(sem, final_count);
}

static int release_waiter(int slot, int final_count)
{
	return release_waiter_on(slot, &g_target, final_count);
}

/****************************************************************************
 * Private Functions - scenarios
 ****************************************************************************/

/****************************************************************************
 * Name: stc_sem_pi01_boost_and_restore
 *
 * Scenario: SCN-PI-01
 *   A priority 100 task holds a binary semaphore; a priority 140 task blocks
 *   on it.
 *
 * Oracle:
 *   (1) while the waiter is blocked, the holder reports 140       <- HARD
 *   (2) after the post, the holder reports 100 again              <- HARD
 *   (3) the count returns to its initial value                    <- HARD
 *
 * Defect signature:
 *   Oracle (1) fails if the boost never happens.  Oracle (2) fails if the
 *   restore never happens, which leaves a worker thread permanently elevated.
 *
 ****************************************************************************/

static void stc_sem_pi01_boost_and_restore(void)
{
	uint32_t violations;
	int ret;
	int prio;

	ret = scenario_begin(1);
	TC_ASSERT_EQ("pi01_begin", ret, OK);

	ret = hold_and_block(STC_PRIO_HIGH, SLOT_WAITER);
	TC_ASSERT_EQ_CLEANUP("pi01_setup", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pi01_boosted", prio, STC_PRIO_HIGH, scenario_end());

	/* Release the holder.  Observing the count return to 0 proves the post
	 * completed: sem_post() increments the count, hands it to the waiter and
	 * runs the priority restore inside a single critical section, so a count
	 * observed from outside that section implies the restore has already run.
	 */

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("pi01_post", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pi01_restored", prio, STC_PRIO_LOW, scenario_end());

	ret = release_waiter(SLOT_WAITER, 1);
	TC_ASSERT_EQ_CLEANUP("pi01_drained", ret, OK, scenario_end());

	ret = stc_wait_finished(SLOT_HOLDER);
	TC_ASSERT_EQ_CLEANUP("pi01_holder_done", ret, OK, scenario_end());

	ret = stc_wait_finished(SLOT_WAITER);
	TC_ASSERT_EQ_CLEANUP("pi01_waiter_done", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pi01_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pi01_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pi02_no_boost_for_equal_or_lower
 *
 * Scenario: SCN-PI-02
 *   The same shape, with the waiter at the holder's own priority, and then
 *   below it.
 *
 * Oracle:
 *   the holder's priority is unchanged in both runs                <- HARD
 *
 * Defect signature:
 *   A non-strict comparison in sem_boostholderprio() would call
 *   sched_setpriority() with the value the holder already has, requeueing it
 *   behind its equals - a yield the holder never asked for.
 *
 ****************************************************************************/

static void stc_sem_pi02_no_boost_for_equal_or_lower(void)
{
	uint32_t violations;
	int ret;
	int prio;

	/* Run 1: waiter at exactly the holder's priority. */

	ret = scenario_begin(1);
	TC_ASSERT_EQ("pi02_begin1", ret, OK);

	ret = hold_and_block(STC_PRIO_LOW, SLOT_WAITER);
	TC_ASSERT_EQ_CLEANUP("pi02_setup1", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pi02_equal_no_boost", prio, STC_PRIO_LOW, scenario_end());

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("pi02_post1", ret, OK, scenario_end());

	ret = release_waiter(SLOT_WAITER, 1);
	TC_ASSERT_EQ_CLEANUP("pi02_drained1", ret, OK, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pi02_no_leaked_actor1", ret, 0);

	/* Run 2: waiter below the holder. */

	ret = scenario_begin(1);
	TC_ASSERT_EQ("pi02_begin2", ret, OK);

	ret = hold_and_block(STC_PRIO_LOW - 10, SLOT_WAITER);
	TC_ASSERT_EQ_CLEANUP("pi02_setup2", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pi02_lower_no_boost", prio, STC_PRIO_LOW, scenario_end());

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("pi02_post2", ret, OK, scenario_end());

	ret = release_waiter(SLOT_WAITER, 1);
	TC_ASSERT_EQ_CLEANUP("pi02_drained2", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pi02_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pi02_no_leaked_actor2", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pi03_tracks_highest_waiter
 *
 * Scenario: SCN-PI-03
 *   Waiters arrive at 130 and then at 160.
 *
 * Oracle:
 *   the holder reports 130, then 160                               <- HARD
 *
 ****************************************************************************/

static void stc_sem_pi03_tracks_highest_waiter(void)
{
	uint32_t violations;
	int ret;
	int prio;
	pid_t pid;

	ret = scenario_begin(1);
	TC_ASSERT_EQ("pi03_begin", ret, OK);

	ret = hold_and_block(STC_PRIO_MID, SLOT_WAITER);
	TC_ASSERT_EQ_CLEANUP("pi03_setup", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pi03_first_boost", prio, STC_PRIO_MID, scenario_end());

	pid = stc_spawn("stc_wait2", STC_PRIO_EXTRA, waiter_actor, SLOT_WAITER2);
	TC_ASSERT_NEQ_CLEANUP("pi03_spawn_waiter2", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -2);
	TC_ASSERT_EQ_CLEANUP("pi03_two_waiters", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pi03_second_boost", prio, STC_PRIO_EXTRA, scenario_end());

	/* Drain.  The highest-priority waiter takes the count first, so the count
	 * walks -2 -> -1 -> 0 -> 1 as each participant posts in turn.
	 */

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("pi03_post", ret, OK, scenario_end());

	ret = release_waiter(SLOT_WAITER2, 0);
	TC_ASSERT_EQ_CLEANUP("pi03_x_drained", ret, OK, scenario_end());

	ret = release_waiter(SLOT_WAITER, 1);
	TC_ASSERT_EQ_CLEANUP("pi03_h_drained", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pi03_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pi03_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pi04_inversion_is_bounded
 *
 * Scenario: SCN-PI-04
 *   A priority 100 holder, a priority 120 CPU burner that never touches the
 *   semaphore, and a priority 140 waiter.  All actors are pinned to one CPU,
 *   so the holder can only make progress by inheriting.
 *
 * Oracle:
 *   (1) the holder reports 140 while the waiter is blocked         <- HARD
 *   (2) the holder completes its critical section and posts        <- HARD
 *   (3) the holder is back at 100 afterwards                       <- HARD
 *
 * Defect signature:
 *   Without inheritance the holder sits at 100, the burner at 120 owns the CPU
 *   indefinitely, and oracle (2) times out.  That timeout is the classic
 *   unbounded priority inversion, reported as a failure rather than as a hang.
 *
 ****************************************************************************/

static void stc_sem_pi04_inversion_is_bounded(void)
{
	uint32_t violations;
	int ret;
	int prio;
	pid_t pid;

	ret = scenario_begin(1);
	TC_ASSERT_EQ("pi04_begin", ret, OK);

	pid = stc_spawn("stc_hwork", STC_PRIO_LOW, holder_work_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("pi04_spawn_holder", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("pi04_held", ret, OK, scenario_end());

	pid = stc_spawn("stc_spin", STC_PRIO_INVERTER, spinner_actor, SLOT_SPINNER);
	TC_ASSERT_NEQ_CLEANUP("pi04_spawn_spinner", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_SPINNER, STAGE_STARTED);
	TC_ASSERT_EQ_CLEANUP("pi04_spinning", ret, OK, scenario_end());

	pid = stc_spawn("stc_wait", STC_PRIO_HIGH, waiter_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("pi04_spawn_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("pi04_blocked", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pi04_boosted", prio, STC_PRIO_HIGH, scenario_end());

	/* The holder now needs CPU time that only the boost can give it: the
	 * burner outranks its base priority and never blocks.
	 */

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("pi04_progress", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pi04_restored", prio, STC_PRIO_LOW, scenario_end());

	g_stc_spin_stop = true;

	ret = release_waiter(SLOT_WAITER, 1);
	TC_ASSERT_EQ_CLEANUP("pi04_drained", ret, OK, scenario_end());

	/* The burner must be confirmed gone before teardown, otherwise it would be
	 * force-deleted and reported as a leaked actor.
	 */

	ret = stc_wait_finished(SLOT_SPINNER);
	TC_ASSERT_EQ_CLEANUP("pi04_spinner_stopped", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pi04_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pi04_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: three_holders_and_waiter
 *
 * Description:
 *   Shared opening for SCN-PI-05 and SCN-PI-06: a counting semaphore with
 *   three counts, three holders at 100, 105 and 110, and one waiter at 160.
 *
 ****************************************************************************/

static int three_holders_and_waiter(void)
{
	static const int prio[3] = { STC_PRIO_LOW, STC_PRIO_LOW2, STC_PRIO_LOW3 };
	static const int slot[3] = { SLOT_HOLDER, SLOT_HOLDER2, SLOT_HOLDER3 };
	int i;

	for (i = 0; i < 3; i++) {
		if (stc_spawn("stc_hold", prio[i], holder_actor, slot[i]) == (pid_t)ERROR) {
			return ERROR;
		}

		if (stc_wait_stage(slot[i], STAGE_HELD) != OK) {
			return ERROR;
		}
	}

	if (stc_getcount(&g_target) != 0) {
		return ERROR;
	}

	if (stc_spawn("stc_wait", STC_PRIO_EXTRA, waiter_actor, SLOT_WAITER) == (pid_t)ERROR) {
		return ERROR;
	}

	return stc_wait_count(&g_target, -1);
}

/****************************************************************************
 * Name: stc_sem_pi05_all_holders_boosted
 *
 * Scenario: SCN-PI-05
 *   Three holders of a counting semaphore, one waiter at 160.
 *
 * Oracle:
 *   all three holders report 160                                   <- HARD
 *
 * Defect signature:
 *   A boost pass that stops at the first holder leaves the other two below the
 *   waiter, and the inversion persists for as long as they hold counts.
 *
 ****************************************************************************/

static void stc_sem_pi05_all_holders_boosted(void)
{
	uint32_t violations;
	int ret;
	int prio;

	ret = scenario_begin(3);
	TC_ASSERT_EQ("pi05_begin", ret, OK);

	ret = three_holders_and_waiter();
	TC_ASSERT_EQ_CLEANUP("pi05_setup", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pi05_holder1_boosted", prio, STC_PRIO_EXTRA, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER2].pid);
	TC_ASSERT_EQ_CLEANUP("pi05_holder2_boosted", prio, STC_PRIO_EXTRA, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER3].pid);
	TC_ASSERT_EQ_CLEANUP("pi05_holder3_boosted", prio, STC_PRIO_EXTRA, scenario_end());

	/* Drain: -1 -> 0 -> 1 -> 2 -> 3 as each participant posts. */

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("pi05_post1", ret, OK, scenario_end());

	stc_go(SLOT_HOLDER2);
	ret = stc_wait_count(&g_target, 1);
	TC_ASSERT_EQ_CLEANUP("pi05_post2", ret, OK, scenario_end());

	stc_go(SLOT_HOLDER3);
	ret = stc_wait_count(&g_target, 2);
	TC_ASSERT_EQ_CLEANUP("pi05_post3", ret, OK, scenario_end());

	ret = release_waiter(SLOT_WAITER, 3);
	TC_ASSERT_EQ_CLEANUP("pi05_drained", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pi05_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pi05_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pi06_all_holders_restored
 *
 * Scenario: SCN-PI-06
 *   The same set-up, observed after the first holder posts.  The waiter has
 *   been granted the count, so no waiter remains that could justify a boost.
 *
 * Oracle:
 *   every holder is back at its own base priority, not only the one that
 *   posted                                                         <- HARD
 *
 * Defect signature:
 *   A restore pass that only reprioritises the posting task leaves the other
 *   two holders at 160 until they happen to post something of their own.  It
 *   is invisible to every existing test, because no existing test reads a
 *   priority.
 *
 ****************************************************************************/

static void stc_sem_pi06_all_holders_restored(void)
{
	uint32_t violations;
	int ret;
	int prio;

	ret = scenario_begin(3);
	TC_ASSERT_EQ("pi06_begin", ret, OK);

	ret = three_holders_and_waiter();
	TC_ASSERT_EQ_CLEANUP("pi06_setup", ret, OK, scenario_end());

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("pi06_post1", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pi06_poster_restored", prio, STC_PRIO_LOW, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER2].pid);
	TC_ASSERT_EQ_CLEANUP("pi06_peer2_restored", prio, STC_PRIO_LOW2, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER3].pid);
	TC_ASSERT_EQ_CLEANUP("pi06_peer3_restored", prio, STC_PRIO_LOW3, scenario_end());

	stc_go(SLOT_HOLDER2);
	ret = stc_wait_count(&g_target, 1);
	TC_ASSERT_EQ_CLEANUP("pi06_post2", ret, OK, scenario_end());

	stc_go(SLOT_HOLDER3);
	ret = stc_wait_count(&g_target, 2);
	TC_ASSERT_EQ_CLEANUP("pi06_post3", ret, OK, scenario_end());

	ret = release_waiter(SLOT_WAITER, 3);
	TC_ASSERT_EQ_CLEANUP("pi06_drained", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pi06_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pi06_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}


/****************************************************************************
 * Private Functions - Family B scenarios
 ****************************************************************************/

/****************************************************************************
 * Name: stc_sem_pix01_still_owes_other_semaphore
 *
 * Scenario: SCN-PIX-01
 *   One holder owns S1 and S2.  A 140 task waits on S1, a 160 task waits on
 *   S2.  S1 is posted first.
 *
 * Oracle:
 *   (1) the holder reaches 140, then 160, as the waiters arrive     <- HARD
 *   (2) after posting S1 the holder is still at 160                 <- HARD
 *   (3) after posting S2 the holder is back at 100                  <- HARD
 *
 * Defect signature:
 *   Oracle (2) fails if the restore pass looks only at the semaphore being
 *   posted instead of walking the holder's whole holdsem list.  Dropping the
 *   holder to base there re-opens the inversion on S2 for its 160 waiter,
 *   which is the exact situation inheritance exists to prevent.
 *
 ****************************************************************************/

static void stc_sem_pix01_still_owes_other_semaphore(void)
{
	uint32_t violations;
	int ret;
	int prio;
	pid_t pid;

	ret = scenario_begin2(1, 1);
	TC_ASSERT_EQ("pix01_begin", ret, OK);

	pid = stc_spawn("stc_h2", STC_PRIO_LOW, holder_two_sems_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("pix01_spawn_holder", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("pix01_held", ret, OK, scenario_end());

	pid = stc_spawn("stc_w1", STC_PRIO_HIGH, waiter_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("pix01_spawn_w1", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("pix01_w1_blocked", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pix01_boost_140", prio, STC_PRIO_HIGH, scenario_end());

	pid = stc_spawn("stc_w2", STC_PRIO_EXTRA, waiter2_actor, SLOT_WAITER2);
	TC_ASSERT_NEQ_CLEANUP("pix01_spawn_w2", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target2, -1);
	TC_ASSERT_EQ_CLEANUP("pix01_w2_blocked", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pix01_boost_160", prio, STC_PRIO_EXTRA, scenario_end());

	/* Release S1 only.  S2 is still held and its waiter still outranks the
	 * holder, so the holder must stay where it is.
	 */

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("pix01_post_s1", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pix01_still_boosted", prio, STC_PRIO_EXTRA, scenario_end());

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target2, 0);
	TC_ASSERT_EQ_CLEANUP("pix01_post_s2", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pix01_restored", prio, STC_PRIO_LOW, scenario_end());

	ret = release_waiter(SLOT_WAITER, 1);
	TC_ASSERT_EQ_CLEANUP("pix01_drain_s1", ret, OK, scenario_end());

	ret = release_waiter_on(SLOT_WAITER2, &g_target2, 1);
	TC_ASSERT_EQ_CLEANUP("pix01_drain_s2", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pix01_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pix01_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pix02_spent_entry_stops_boosting
 *
 * Scenario: SCN-PIX-02
 *   One holder owns S1 and S2.  Two tasks, 130 and 140, wait on S1.  Nobody
 *   waits on S2.  S1 is posted once, so the 140 task takes the count and the
 *   130 task keeps waiting.
 *
 * Oracle:
 *   after the post the holder is back at 100, not at 130            <- HARD
 *
 * Defect signature:
 *   The holder's S1 entry is spent - its count reached zero in
 *   sem_releaseholder() - so the restore pass must skip it.  If it does not,
 *   it finds the still-waiting 130 task and pins the holder there.  The task
 *   is then permanently one boost above its base until it happens to post
 *   something else, and a worker looping on the same two semaphores never
 *   unwinds at all.  This is the single most valuable scenario in Family B.
 *
 ****************************************************************************/

static void stc_sem_pix02_spent_entry_stops_boosting(void)
{
	uint32_t violations;
	int ret;
	int prio;
	pid_t pid;

	ret = scenario_begin2(1, 1);
	TC_ASSERT_EQ("pix02_begin", ret, OK);

	pid = stc_spawn("stc_h2", STC_PRIO_LOW, holder_two_sems_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("pix02_spawn_holder", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("pix02_held", ret, OK, scenario_end());

	pid = stc_spawn("stc_w130", STC_PRIO_MID, waiter_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("pix02_spawn_w130", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("pix02_w130_blocked", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pix02_boost_130", prio, STC_PRIO_MID, scenario_end());

	pid = stc_spawn("stc_w140", STC_PRIO_HIGH, waiter_actor, SLOT_WAITER2);
	TC_ASSERT_NEQ_CLEANUP("pix02_spawn_w140", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -2);
	TC_ASSERT_EQ_CLEANUP("pix02_w140_blocked", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pix02_boost_140", prio, STC_PRIO_HIGH, scenario_end());

	/* Post S1 once.  The 140 task takes the count; the 130 task is still
	 * linked as a waiter on S1, but the holder no longer holds a count there.
	 */

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("pix02_post_s1", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pix02_fully_restored", prio, STC_PRIO_LOW, scenario_end());

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target2, 1);
	TC_ASSERT_EQ_CLEANUP("pix02_post_s2", ret, OK, scenario_end());

	ret = release_waiter(SLOT_WAITER2, 0);
	TC_ASSERT_EQ_CLEANUP("pix02_drain_w140", ret, OK, scenario_end());

	ret = release_waiter(SLOT_WAITER, 1);
	TC_ASSERT_EQ_CLEANUP("pix02_drain_w130", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pix02_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pix02_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pix03_awakened_waiter_excluded
 *
 * Scenario: SCN-PIX-03
 *   One holder owns two counts of the same semaphore.  A 160 task blocks.  One
 *   count is posted, so the waiter is granted it while the holder still owns
 *   the other count.
 *
 * Oracle:
 *   after the post the holder is back at 100                        <- HARD
 *
 * Defect signature:
 *   The holder still owns a count, so its entry is examined rather than
 *   skipped.  The task that has just been granted the count can still be
 *   linked on the waiting list while priorities are recomputed, so it must be
 *   excluded from that recomputation.  Without the exclusion it is found as
 *   the highest waiter of a semaphore it has already been given, and the boost
 *   never ends.
 *
 ****************************************************************************/

static void stc_sem_pix03_awakened_waiter_excluded(void)
{
	uint32_t violations;
	int ret;
	int prio;
	pid_t pid;

	ret = scenario_begin(2);
	TC_ASSERT_EQ("pix03_begin", ret, OK);

	pid = stc_spawn("stc_h2c", STC_PRIO_LOW, holder_two_counts_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("pix03_spawn_holder", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("pix03_held", ret, OK, scenario_end());

	ret = stc_getcount(&g_target);
	TC_ASSERT_EQ_CLEANUP("pix03_both_counts_taken", ret, 0, scenario_end());

	pid = stc_spawn("stc_wx", STC_PRIO_EXTRA, waiter_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("pix03_spawn_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("pix03_blocked", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pix03_boosted", prio, STC_PRIO_EXTRA, scenario_end());

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("pix03_post_one", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pix03_restored", prio, STC_PRIO_LOW, scenario_end());

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 1);
	TC_ASSERT_EQ_CLEANUP("pix03_post_two", ret, OK, scenario_end());

	ret = release_waiter(SLOT_WAITER, 2);
	TC_ASSERT_EQ_CLEANUP("pix03_drained", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pix03_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pix03_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pix04_ambiguous_release_keeps_records
 *
 * Scenario: SCN-PIX-04
 *   A counting semaphore with two holders, a 140 waiter, and a third task that
 *   never waited posting the semaphore.  With two holders there is no way to
 *   tell whose count was released.
 *
 * Oracle:
 *   (1) both holders are boosted to 140 while the waiter blocks     <- HARD
 *   (2) after the non-holder post, both holders are back at their own
 *       base priorities                                             <- HARD
 *   (3) both can still release their own counts, and the final count
 *       reflects the extra count the non-holder injected             <- HARD
 *
 * Defect signature:
 *   Attributing an ambiguous release to the posting task would drop a holder
 *   record that is still live.  The holder then keeps being boosted by later
 *   waiters, and its count is released a second time if the task is deleted -
 *   which on a mutex flagged semaphore trips the semcount < 2 assertion.
 *
 ****************************************************************************/

static void stc_sem_pix04_ambiguous_release_keeps_records(void)
{
	uint32_t violations;
	int ret;
	int prio;
	pid_t pid;

	ret = scenario_begin(2);
	TC_ASSERT_EQ("pix04_begin", ret, OK);

	pid = stc_spawn("stc_ha", STC_PRIO_LOW, holder_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("pix04_spawn_h1", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("pix04_h1_held", ret, OK, scenario_end());

	pid = stc_spawn("stc_hb", STC_PRIO_LOW2, holder_actor, SLOT_HOLDER2);
	TC_ASSERT_NEQ_CLEANUP("pix04_spawn_h2", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER2, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("pix04_h2_held", ret, OK, scenario_end());

	pid = stc_spawn("stc_w", STC_PRIO_HIGH, waiter_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("pix04_spawn_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("pix04_blocked", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pix04_h1_boosted", prio, STC_PRIO_HIGH, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER2].pid);
	TC_ASSERT_EQ_CLEANUP("pix04_h2_boosted", prio, STC_PRIO_HIGH, scenario_end());

	/* A task that never waited posts the semaphore. */

	pid = stc_spawn("stc_post", STC_PRIO_INVERTER, poster_actor, SLOT_POSTER);
	TC_ASSERT_NEQ_CLEANUP("pix04_spawn_poster", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_POSTER, STAGE_STARTED);
	TC_ASSERT_EQ_CLEANUP("pix04_poster_ready", ret, OK, scenario_end());

	stc_go(SLOT_POSTER);
	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("pix04_nonholder_post", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pix04_h1_restored", prio, STC_PRIO_LOW, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER2].pid);
	TC_ASSERT_EQ_CLEANUP("pix04_h2_restored", prio, STC_PRIO_LOW2, scenario_end());

	/* Both holders must still be able to release their own counts. */

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 1);
	TC_ASSERT_EQ_CLEANUP("pix04_h1_post", ret, OK, scenario_end());

	stc_go(SLOT_HOLDER2);
	ret = stc_wait_count(&g_target, 2);
	TC_ASSERT_EQ_CLEANUP("pix04_h2_post", ret, OK, scenario_end());

	ret = release_waiter(SLOT_WAITER, 3);
	TC_ASSERT_EQ_CLEANUP("pix04_drained", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pix04_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pix04_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pix05_unambiguous_release_frees_record
 *
 * Scenario: SCN-PIX-05
 *   A counting semaphore with exactly one holder, posted by a task that never
 *   waited.  With a single holder the release is unambiguous, so that holder's
 *   record must be released.  A different task then takes the count and a 140
 *   task blocks behind it.
 *
 * Oracle:
 *   (1) the new holder is boosted to 140                            <- HARD
 *   (2) the original holder stays at 100                            <- HARD
 *
 * Defect signature:
 *   If the original holder's record survives the non-holder post, it is a
 *   record of a count the task no longer owns.  Oracle (2) then fails: the
 *   stale entry boosts a task that holds nothing at all.
 *
 ****************************************************************************/

static void stc_sem_pix05_unambiguous_release_frees_record(void)
{
	uint32_t violations;
	int ret;
	int prio;
	pid_t pid;

	ret = scenario_begin(1);
	TC_ASSERT_EQ("pix05_begin", ret, OK);

	pid = stc_spawn("stc_hnp", STC_PRIO_LOW, holder_no_post_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("pix05_spawn_holder", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("pix05_held", ret, OK, scenario_end());

	pid = stc_spawn("stc_post", STC_PRIO_INVERTER, poster_actor, SLOT_POSTER);
	TC_ASSERT_NEQ_CLEANUP("pix05_spawn_poster", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_POSTER, STAGE_STARTED);
	TC_ASSERT_EQ_CLEANUP("pix05_poster_ready", ret, OK, scenario_end());

	/* The count is returned on the holder's behalf.  The holder owns nothing
	 * from this point on.
	 */

	stc_go(SLOT_POSTER);
	ret = stc_wait_count(&g_target, 1);
	TC_ASSERT_EQ_CLEANUP("pix05_nonholder_post", ret, OK, scenario_end());

	pid = stc_spawn("stc_take", STC_PRIO_LOW2, holder_actor, SLOT_TAKER);
	TC_ASSERT_NEQ_CLEANUP("pix05_spawn_taker", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_TAKER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("pix05_taker_held", ret, OK, scenario_end());

	pid = stc_spawn("stc_w", STC_PRIO_HIGH, waiter_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("pix05_spawn_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("pix05_blocked", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_TAKER].pid);
	TC_ASSERT_EQ_CLEANUP("pix05_taker_boosted", prio, STC_PRIO_HIGH, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pix05_exholder_not_boosted", prio, STC_PRIO_LOW, scenario_end());

	stc_go(SLOT_TAKER);
	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("pix05_taker_post", ret, OK, scenario_end());

	ret = release_waiter(SLOT_WAITER, 1);
	TC_ASSERT_EQ_CLEANUP("pix05_drained", ret, OK, scenario_end());

	/* The original holder never posts: its count was already returned. */

	stc_go(SLOT_HOLDER);
	ret = stc_wait_finished(SLOT_HOLDER);
	TC_ASSERT_EQ_CLEANUP("pix05_exholder_done", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pix05_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pix05_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stc_sem_inherit_main
 ****************************************************************************/

int stc_sem_inherit_main(void)
{
	if (stc_harness_begin() != OK) {
		printf("\n[stc_sem_inherit] FAIL : cannot raise the harness priority\n");
		total_fail++;
		return ERROR;
	}

	/* Catalogue section 8.2: the monitor runs alongside every other scenario,
	 * so each one below reports both its own oracle and any priority leak the
	 * monitor observed while it ran.
	 */

	if (stc_mon_start() != OK) {
		printf("\n[stc_sem_inherit] FAIL : cannot start the boost-leak monitor\n");
		total_fail++;
		stc_harness_end();
		return ERROR;
	}

	stc_sem_pi01_boost_and_restore();
	stc_sem_pi02_no_boost_for_equal_or_lower();
	stc_sem_pi03_tracks_highest_waiter();
	stc_sem_pi04_inversion_is_bounded();
	stc_sem_pi05_all_holders_boosted();
	stc_sem_pi06_all_holders_restored();

	stc_sem_pix01_still_owes_other_semaphore();
	stc_sem_pix02_spent_entry_stops_boosting();
	stc_sem_pix03_awakened_waiter_excluded();
	stc_sem_pix04_ambiguous_release_keeps_records();
	stc_sem_pix05_unambiguous_release_frees_record();

	stc_mon_stop();
	stc_harness_end();

	return OK;
}

#else							/* CONFIG_PRIORITY_INHERITANCE */

int stc_sem_inherit_main(void)
{
	printf("\n[stc_sem_inherit] SKIP : CONFIG_PRIORITY_INHERITANCE is not enabled\n");
	return OK;
}

#endif							/* CONFIG_PRIORITY_INHERITANCE */
