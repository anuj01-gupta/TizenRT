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

/// @file stc_sem_cancel.c
/// @brief Family C of the Priority Inheritance and Semaphore Recovery
///        Scenario Test Catalogue - wait cancellation.
///
/// SCN-PIC-01  a signal interrupts a blocked waiter
/// SCN-PIC-02  sem_timedwait() expires
/// SCN-PIC-03  sem_tickwait() expires                    (deferred, see below)
/// SCN-PIC-04  the blocked waiter is deleted
/// SCN-PIC-05  one of two waiters is deleted
///
/// Every path that ends a wait without a post has to do the same three things:
/// undo the boost it caused, return the count it took, and leave the ex-waiter
/// out of the holder set.  Each scenario below checks all three, because a
/// defect in any one of them is invisible in the others.
///
/// SCN-PIC-03 is deferred rather than implemented.  sem_tickwait() is not
/// declared in the application facing semaphore.h and is not exported through
/// the syscall table, so it cannot be reached from a user space test at all -
/// which is why the existing tc_semaphore_sem_tickwait() drives it through
/// TESTIOC_SEM_TICK_WAIT_TEST instead of calling it.  Covering it needs a
/// kernel side handler in os/drivers/os_api_test/kernel/test_sem.c, which
/// belongs with the phase 4 driver work rather than here.  SCN-PIC-02 already
/// covers the shared sem_waitirq() timeout path that sem_tickwait() reaches.
///
/// Note on cancellation points: CONFIG_CANCELLATION_POINTS is not set on the
/// reference platform, so enter_cancellation_point() is a constant false and
/// sem_wait() never returns ECANCELED.  Deleting a blocked waiter therefore
/// goes straight through task_recover() to sem_recover(), which is the path
/// SCN-PIC-04 and SCN-PIC-05 are written for.  Nothing here asserts ECANCELED.

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
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

#define STAGE_STARTED           1
#define STAGE_HELD              2
#define STAGE_CANCELLED         3	/* waiter returned without the count   */

#define SLOT_HOLDER             0
#define SLOT_WAITER             1
#define SLOT_WAITER2            2

/* Timeout used by SCN-PIC-02, comfortably longer than the harness needs to
 * observe the blocked state and sample the boost, and far shorter than
 * STC_TIMEOUT_MS.
 */

#define TIMEDWAIT_MS            300

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_target;

/* errno reported by the cancelled wait, recorded by the actor for the harness
 * to assert on.  Written once, by one actor, before it announces the stage.
 */

static volatile int g_cancel_errno;

/****************************************************************************
 * Private Functions - actors
 ****************************************************************************/

static int cancel_holder_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	if (sem_wait(&g_target) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_HELD);
	stc_wait_go(slot);

	(void)sem_post(&g_target);

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Name: signal_handler
 *
 * Description:
 *   Installed only so the signal has an action.  The work is done by the
 *   kernel's sem_waitirq() path, not here.
 *
 ****************************************************************************/

static void signal_handler(int signo)
{
	(void)signo;
}

/****************************************************************************
 * Name: signalled_waiter_actor
 *
 * Description:
 *   Blocks on the semaphore and expects to be woken by a signal rather than by
 *   a post.  It must not post afterwards: it never received a count.
 *
 ****************************************************************************/

static int signalled_waiter_actor(int argc, char *argv[])
{
	struct sigaction act;
	int slot = atoi(argv[1]);

	(void)argc;

	act.sa_handler = signal_handler;
	act.sa_flags = 0;
	(void)sigemptyset(&act.sa_mask);

	if (sigaction(SIGUSR1, &act, NULL) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_STARTED);

	if (sem_wait(&g_target) == OK) {
		/* Unexpected: the wait was supposed to be interrupted.  Give the
		 * count straight back so the arithmetic still adds up, and leave the
		 * stage unset so the harness reports the failure.
		 */

		(void)sem_post(&g_target);
		stc_actor_done(slot);
		return ERROR;
	}

	g_cancel_errno = get_errno();
	stc_stage(slot, STAGE_CANCELLED);

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Name: timedwait_waiter_actor
 ****************************************************************************/

static int timedwait_waiter_actor(int argc, char *argv[])
{
	struct timespec abstime;
	int slot = atoi(argv[1]);

	(void)argc;

	if (clock_gettime(CLOCK_REALTIME, &abstime) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	abstime.tv_nsec += (TIMEDWAIT_MS % 1000) * 1000000;
	if (abstime.tv_nsec >= 1000000000) {
		abstime.tv_sec += 1;
		abstime.tv_nsec -= 1000000000;
	}
	abstime.tv_sec += TIMEDWAIT_MS / 1000;

	stc_stage(slot, STAGE_STARTED);

	if (sem_timedwait(&g_target, &abstime) == OK) {
		(void)sem_post(&g_target);
		stc_actor_done(slot);
		return ERROR;
	}

	g_cancel_errno = get_errno();
	stc_stage(slot, STAGE_CANCELLED);

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Name: doomed_waiter_actor
 *
 * Description:
 *   Blocks and is expected never to return: the harness deletes it.  It is
 *   removed from the actor table with stc_actor_forget() so teardown does not
 *   report the deletion as a leaked actor.
 *
 ****************************************************************************/

static int doomed_waiter_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	stc_stage(slot, STAGE_STARTED);

	if (sem_wait(&g_target) == OK) {
		(void)sem_post(&g_target);
	}

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Private Functions - helpers
 ****************************************************************************/

static int scenario_begin(int initial_count)
{
	if (stc_reset() != OK) {
		return ERROR;
	}

	if (sem_init(&g_target, 0, initial_count) != OK) {
		return ERROR;
	}

	stc_mon_register_sem(&g_target);
	g_cancel_errno = 0;

	return OK;
}

static int scenario_end(void)
{
	int leaked = stc_teardown();

	(void)sem_destroy(&g_target);

	return leaked;
}

/****************************************************************************
 * Name: hold_and_block
 *
 * Description:
 *   A priority 100 holder takes the semaphore, then a waiter of the given kind
 *   and priority blocks on it.
 *
 ****************************************************************************/

static int hold_and_block(main_t waiter_entry, int waiter_prio, int waiter_slot, int expect_count)
{
	if (stc_spawn("stc_chold", STC_PRIO_LOW, cancel_holder_actor, SLOT_HOLDER) == (pid_t)ERROR) {
		return ERROR;
	}

	if (stc_wait_stage(SLOT_HOLDER, STAGE_HELD) != OK) {
		return ERROR;
	}

	if (stc_spawn("stc_cwait", waiter_prio, waiter_entry, waiter_slot) == (pid_t)ERROR) {
		return ERROR;
	}

	if (stc_wait_stage(waiter_slot, STAGE_STARTED) != OK) {
		return ERROR;
	}

	return stc_wait_count(&g_target, expect_count);
}

/****************************************************************************
 * Private Functions - scenarios
 ****************************************************************************/

/****************************************************************************
 * Name: stc_sem_pic01_signal_cancels_wait
 *
 * Scenario: SCN-PIC-01
 *   A 140 task blocks on a semaphore held by a 100 task, and is then sent
 *   SIGUSR1.
 *
 * Oracle:
 *   (1) the holder is boosted to 140 while the waiter blocks       <- HARD
 *   (2) the wait returns ERROR with errno EINTR                    <- HARD
 *   (3) the holder is back at 100                                  <- HARD
 *   (4) the count is exactly one higher, and exactly one higher again
 *       once the holder posts                                      <- HARD
 *
 * Defect signature:
 *   Oracle (4) is the interesting one.  The clean-up belongs entirely to
 *   sem_waitirq(): it restores the holder priorities, increments the count and
 *   clears waitsem.  If sem_wait()'s resume path repeated any of that, the
 *   count would end one too high - and on a mutex flagged semaphore that is an
 *   immediate assertion failure rather than a quiet corruption.
 *
 ****************************************************************************/

static void stc_sem_pic01_signal_cancels_wait(void)
{
	uint32_t violations;
	int ret;
	int prio;

	ret = scenario_begin(1);
	TC_ASSERT_EQ("pic01_begin", ret, OK);

	ret = hold_and_block(signalled_waiter_actor, STC_PRIO_HIGH, SLOT_WAITER, -1);
	TC_ASSERT_EQ_CLEANUP("pic01_setup", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pic01_boosted", prio, STC_PRIO_HIGH, scenario_end());

	ret = kill(g_stc_actor[SLOT_WAITER].pid, SIGUSR1);
	TC_ASSERT_EQ_CLEANUP("pic01_signal_sent", ret, OK, scenario_end());

	ret = stc_wait_stage(SLOT_WAITER, STAGE_CANCELLED);
	TC_ASSERT_EQ_CLEANUP("pic01_wait_interrupted", ret, OK, scenario_end());

	TC_ASSERT_EQ_CLEANUP("pic01_errno_eintr", g_cancel_errno, EINTR, scenario_end());

	ret = stc_getcount(&g_target);
	TC_ASSERT_EQ_CLEANUP("pic01_count_returned", ret, 0, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pic01_restored", prio, STC_PRIO_LOW, scenario_end());

	/* The ex-waiter must not have become a holder: with no waiter left, the
	 * holder's post has to leave the count at exactly 1.
	 */

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 1);
	TC_ASSERT_EQ_CLEANUP("pic01_no_double_count", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pic01_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pic01_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pic02_timeout_cancels_wait
 *
 * Scenario: SCN-PIC-02
 *   The same shape, ended by a sem_timedwait() timeout instead of a signal.
 *
 * Oracle:
 *   as SCN-PIC-01, with errno ETIMEDOUT                            <- HARD
 *
 ****************************************************************************/

static void stc_sem_pic02_timeout_cancels_wait(void)
{
	uint32_t violations;
	int ret;
	int prio;

	ret = scenario_begin(1);
	TC_ASSERT_EQ("pic02_begin", ret, OK);

	ret = hold_and_block(timedwait_waiter_actor, STC_PRIO_HIGH, SLOT_WAITER, -1);
	TC_ASSERT_EQ_CLEANUP("pic02_setup", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pic02_boosted", prio, STC_PRIO_HIGH, scenario_end());

	/* Wait for the timeout to fire on its own. */

	ret = stc_wait_stage(SLOT_WAITER, STAGE_CANCELLED);
	TC_ASSERT_EQ_CLEANUP("pic02_wait_timed_out", ret, OK, scenario_end());

	TC_ASSERT_EQ_CLEANUP("pic02_errno_etimedout", g_cancel_errno, ETIMEDOUT, scenario_end());

	ret = stc_getcount(&g_target);
	TC_ASSERT_EQ_CLEANUP("pic02_count_returned", ret, 0, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pic02_restored", prio, STC_PRIO_LOW, scenario_end());

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 1);
	TC_ASSERT_EQ_CLEANUP("pic02_no_double_count", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pic02_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pic02_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pic04_deleted_waiter
 *
 * Scenario: SCN-PIC-04
 *   The blocked waiter is deleted outright, reaching sem_recover() rather than
 *   sem_waitirq().
 *
 * Oracle:
 *   (1) the holder is back at 100                                  <- HARD
 *   (2) the count is exactly one higher                            <- HARD
 *   (3) the holder's own post still leaves it at exactly 1         <- HARD
 *
 ****************************************************************************/

static void stc_sem_pic04_deleted_waiter(void)
{
	uint32_t violations;
	int ret;
	int prio;

	ret = scenario_begin(1);
	TC_ASSERT_EQ("pic04_begin", ret, OK);

	ret = hold_and_block(doomed_waiter_actor, STC_PRIO_HIGH, SLOT_WAITER, -1);
	TC_ASSERT_EQ_CLEANUP("pic04_setup", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pic04_boosted", prio, STC_PRIO_HIGH, scenario_end());

	ret = task_delete(g_stc_actor[SLOT_WAITER].pid);
	TC_ASSERT_EQ_CLEANUP("pic04_delete_waiter", ret, OK, scenario_end());

	/* The actor will never reach its exit point, by design. */

	stc_actor_forget(SLOT_WAITER);

	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("pic04_count_returned", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pic04_restored", prio, STC_PRIO_LOW, scenario_end());

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 1);
	TC_ASSERT_EQ_CLEANUP("pic04_no_double_count", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pic04_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pic04_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pic05_delete_highest_of_two_waiters
 *
 * Scenario: SCN-PIC-05
 *   Two waiters, at 140 and 160.  The 160 one is deleted while blocked.
 *
 * Oracle:
 *   the holder drops from 160 to 140, not to 100                   <- HARD
 *
 * Defect signature:
 *   This is the discriminating case for the cancellation path.  Restoring the
 *   holder all the way to base would re-open the inversion for the 140 task
 *   that is still waiting, and no scenario with a single waiter can tell the
 *   difference: with one waiter, "drop to base" and "drop to the highest
 *   remaining waiter" give the same answer.
 *
 ****************************************************************************/

static void stc_sem_pic05_delete_highest_of_two_waiters(void)
{
	uint32_t violations;
	int ret;
	int prio;
	pid_t pid;

	ret = scenario_begin(1);
	TC_ASSERT_EQ("pic05_begin", ret, OK);

	ret = hold_and_block(doomed_waiter_actor, STC_PRIO_HIGH, SLOT_WAITER, -1);
	TC_ASSERT_EQ_CLEANUP("pic05_setup", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pic05_boost_140", prio, STC_PRIO_HIGH, scenario_end());

	pid = stc_spawn("stc_cw2", STC_PRIO_EXTRA, doomed_waiter_actor, SLOT_WAITER2);
	TC_ASSERT_NEQ_CLEANUP("pic05_spawn_w2", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -2);
	TC_ASSERT_EQ_CLEANUP("pic05_two_waiters", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pic05_boost_160", prio, STC_PRIO_EXTRA, scenario_end());

	ret = task_delete(g_stc_actor[SLOT_WAITER2].pid);
	TC_ASSERT_EQ_CLEANUP("pic05_delete_w2", ret, OK, scenario_end());

	stc_actor_forget(SLOT_WAITER2);

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("pic05_count_returned", ret, OK, scenario_end());

	/* The 140 task is still waiting, so the holder must stay at 140. */

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pic05_partial_restore", prio, STC_PRIO_HIGH, scenario_end());

	/* Release the holder.  The surviving waiter takes the count and posts it
	 * straight back on its way out, so the settled value is 1.  Waiting for
	 * the intermediate 0 would be a race against that post rather than an
	 * observation, so the wait targets the settled value; the holder's
	 * priority is stable from its own post onwards and can be sampled after.
	 */

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 1);
	TC_ASSERT_EQ_CLEANUP("pic05_drained", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pic05_full_restore", prio, STC_PRIO_LOW, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pic05_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pic05_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stc_sem_cancel_main(void)
{
	if (stc_harness_begin() != OK) {
		printf("\n[stc_sem_cancel] FAIL : cannot raise the harness priority\n");
		total_fail++;
		return ERROR;
	}

	if (stc_mon_start() != OK) {
		printf("\n[stc_sem_cancel] FAIL : cannot start the boost-leak monitor\n");
		total_fail++;
		stc_harness_end();
		return ERROR;
	}

	stc_sem_pic01_signal_cancels_wait();
	stc_sem_pic02_timeout_cancels_wait();
	stc_sem_pic04_deleted_waiter();
	stc_sem_pic05_delete_highest_of_two_waiters();

	stc_mon_stop();
	stc_harness_end();

	return OK;
}

#else							/* CONFIG_PRIORITY_INHERITANCE */

int stc_sem_cancel_main(void)
{
	printf("\n[stc_sem_cancel] SKIP : CONFIG_PRIORITY_INHERITANCE is not enabled\n");
	return OK;
}

#endif							/* CONFIG_PRIORITY_INHERITANCE */
