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

/// @file stc_sem_recovery.c
/// @brief Families E and F of the Priority Inheritance and Semaphore Recovery
///        Scenario Test Catalogue - recovery.
///
/// Family E - mutex usage
///   SCN-REC-01  delete a task blocked on a mutex flagged semaphore
///   SCN-REC-02  delete the owner while another task is blocked
///   SCN-REC-03  delete a task holding several semaphores
///   SCN-REC-05  delete a task holding two counts of one semaphore  (measured)
///   SCN-REC-06  signal and delete racing on the same waiter
///
/// Family F - signalling usage and holder accounting
///   SCN-SIG-01  delete a task blocked on a signalling semaphore
///   SCN-SIG-02  delete the signaller, waiter recovers by its own timeout
///   SCN-SIG-03  the same with a plain wait                    (characterisation)
///   SCN-SIG-04  delete one of two holders                          (measured)
///   SCN-SIG-05  repeated create, acquire and delete cycles
///
/// SCN-REC-04, the robust mutex EOWNERDEAD path, is compiled only when
/// CONFIG_PTHREAD_MUTEX_UNSAFE is off.  The reference platform sets it on,
/// which leaves pthread_mutex.c, pthread_mutexconsistent.c and
/// pthread_mutexinconsistent.c out of the build entirely - so on that platform
/// the scenario reports a skip rather than a pass, and the run log says which
/// configuration is needed to cover it.
///
/// Recovery has no coverage at all in the tree today.  Its failure mode is a
/// task blocked forever on a semaphore whose holder died - the most expensive
/// kind of defect to diagnose from a crash dump, because the evidence is the
/// absence of something.  SCN-REC-02 alone justifies the family.

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
#include <pthread.h>
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
#define STAGE_ACQUIRED          3
#define STAGE_CANCELLED         4

#define SLOT_HOLDER             0
#define SLOT_HOLDER2            1
#define SLOT_WAITER             2
#define SLOT_WAITER2            3
#define SLOT_SIGNALLER          4

/* SCN-REC-06 iteration count.  The race is between sem_waitirq() and
 * sem_recover(), both of which guard on task_state == TSTATE_WAIT_SEM; a
 * missing guard shows up as a count one too high, so the scenario needs enough
 * repetitions to hit the window.
 */

#define RACE_ITERATIONS         200

/* SCN-SIG-05 cycle count.  Lower than the catalogue's 1000 so the tier stays
 * usable in a nightly job; the leak it looks for is per cycle, so it is
 * visible long before then.
 */

#define CHURN_CYCLES            100

#define SIG_TIMEDWAIT_MS        300

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_target;			/* generic semaphore under test           */
static sem_t g_extra1;			/* SCN-REC-03 second semaphore            */
static sem_t g_extra2;			/* SCN-REC-03 third semaphore             */
static bool g_extra_valid;

static pthread_mutex_t g_mutex;	/* FLAGS_SEM_MUTEX semaphore              */
static bool g_mutex_valid;

static volatile int g_cancel_errno;

/****************************************************************************
 * Private Functions - actors
 ****************************************************************************/

/* Locks the mutex, announces it, waits, unlocks. */

static int mutex_owner_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	if (pthread_mutex_lock(&g_mutex) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_HELD);
	stc_wait_go(slot);

	(void)pthread_mutex_unlock(&g_mutex);

	stc_actor_done(slot);
	return OK;
}

/* Blocks on the mutex.  Used both as the survivor that must be woken by
 * recovery, and as the victim that is deleted while blocked.
 */

static int mutex_blocker_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	stc_stage(slot, STAGE_STARTED);

	if (pthread_mutex_lock(&g_mutex) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_ACQUIRED);
	stc_wait_go(slot);

	(void)pthread_mutex_unlock(&g_mutex);

	stc_actor_done(slot);
	return OK;
}

/* Holds g_target, g_extra1 and g_extra2 at once, and is then deleted. */

static int three_sem_holder_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	if (sem_wait(&g_target) != OK || sem_wait(&g_extra1) != OK || sem_wait(&g_extra2) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_HELD);
	stc_wait_go(slot);

	(void)sem_post(&g_extra2);
	(void)sem_post(&g_extra1);
	(void)sem_post(&g_target);

	stc_actor_done(slot);
	return OK;
}

/* Takes two counts of g_target and is then deleted (SCN-REC-05). */

static int two_count_holder_actor(int argc, char *argv[])
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
	(void)sem_post(&g_target);

	stc_actor_done(slot);
	return OK;
}

/* Takes one count and parks; deleted while holding (SCN-SIG-04). */

static int plain_holder_actor(int argc, char *argv[])
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

/* Blocks on g_target and is expected to acquire it when recovery hands the
 * count over.
 */

static int survivor_waiter_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	stc_stage(slot, STAGE_STARTED);

	if (sem_wait(&g_target) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_ACQUIRED);
	stc_wait_go(slot);

	(void)sem_post(&g_target);

	stc_actor_done(slot);
	return OK;
}

/* Blocks on g_target and is deleted there. */

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

/* Waits on a signalling semaphore with a timeout (SCN-SIG-02). */

static int sig_timedwait_actor(int argc, char *argv[])
{
	struct timespec abstime;
	int slot = atoi(argv[1]);

	(void)argc;

	if (clock_gettime(CLOCK_REALTIME, &abstime) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	abstime.tv_nsec += (SIG_TIMEDWAIT_MS % 1000) * 1000000;
	if (abstime.tv_nsec >= 1000000000) {
		abstime.tv_sec += 1;
		abstime.tv_nsec -= 1000000000;
	}
	abstime.tv_sec += SIG_TIMEDWAIT_MS / 1000;

	stc_stage(slot, STAGE_STARTED);

	if (sem_timedwait(&g_target, &abstime) == OK) {
		stc_stage(slot, STAGE_ACQUIRED);
		stc_actor_done(slot);
		return OK;
	}

	g_cancel_errno = get_errno();
	stc_stage(slot, STAGE_CANCELLED);

	stc_actor_done(slot);
	return OK;
}

/* Intends to post the signalling semaphore, but is deleted first. */

static int never_signals_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	stc_stage(slot, STAGE_STARTED);
	stc_wait_go(slot);

	(void)sem_post(&g_target);

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

static int scenario_begin_mutex(void)
{
	if (stc_reset() != OK) {
		return ERROR;
	}

	if (pthread_mutex_init(&g_mutex, NULL) != OK) {
		return ERROR;
	}

	g_mutex_valid = true;
	stc_mon_register_sem((sem_t *)&g_mutex.sem);

	return OK;
}

static int scenario_end(void)
{
	int leaked = stc_teardown();

	(void)sem_destroy(&g_target);

	if (g_extra_valid) {
		(void)sem_destroy(&g_extra1);
		(void)sem_destroy(&g_extra2);
		g_extra_valid = false;
	}

	if (g_mutex_valid) {
		(void)pthread_mutex_destroy(&g_mutex);
		g_mutex_valid = false;
	}

	return leaked;
}

/****************************************************************************
 * Name: mutex_count
 *
 * Description:
 *   The count of the semaphore underlying the mutex.  Catalogue rule G-6: a
 *   FLAGS_SEM_MUTEX semaphore must never be observed at 2 or more, and the
 *   kernel asserts on it in sem_wait(), sem_post(), sem_waitirq() and
 *   sem_recover(), so an over-release is a crash rather than a silent
 *   corruption.
 *
 ****************************************************************************/

static int mutex_count(void)
{
	return stc_getcount((sem_t *)&g_mutex.sem);
}

/****************************************************************************
 * Private Functions - Family E scenarios
 ****************************************************************************/

/****************************************************************************
 * Name: stc_sem_rec01_delete_blocked_on_mutex
 *
 * Scenario: SCN-REC-01
 *   A task blocked on a mutex flagged semaphore is deleted.
 *
 * Oracle:
 *   (1) the owner is restored to its base priority                 <- HARD
 *   (2) the count is exactly one higher, and never reaches 2       <- HARD
 *   (3) the owner can still unlock cleanly                         <- HARD
 *
 ****************************************************************************/

static void stc_sem_rec01_delete_blocked_on_mutex(void)
{
	uint32_t violations;
	int ret;
	int prio;
	pid_t pid;

	ret = scenario_begin_mutex();
	TC_ASSERT_EQ("rec01_begin", ret, OK);

	pid = stc_spawn("stc_mown", STC_PRIO_LOW, mutex_owner_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("rec01_spawn_owner", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("rec01_owned", ret, OK, scenario_end());

	pid = stc_spawn("stc_mblk", STC_PRIO_HIGH, mutex_blocker_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("rec01_spawn_blocker", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count((sem_t *)&g_mutex.sem, -1);
	TC_ASSERT_EQ_CLEANUP("rec01_blocked", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("rec01_boosted", prio, STC_PRIO_HIGH, scenario_end());

	ret = task_delete(g_stc_actor[SLOT_WAITER].pid);
	TC_ASSERT_EQ_CLEANUP("rec01_delete_blocker", ret, OK, scenario_end());

	stc_actor_forget(SLOT_WAITER);

	ret = stc_wait_count((sem_t *)&g_mutex.sem, 0);
	TC_ASSERT_EQ_CLEANUP("rec01_count_returned", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("rec01_restored", prio, STC_PRIO_LOW, scenario_end());

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count((sem_t *)&g_mutex.sem, 1);
	TC_ASSERT_EQ_CLEANUP("rec01_unlocked", ret, OK, scenario_end());

	ret = mutex_count();
	TC_ASSERT_LT_CLEANUP("rec01_mutex_invariant", ret, 2, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("rec01_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("rec01_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_rec02_delete_owner_wakes_blocker
 *
 * Scenario: SCN-REC-02
 *   The mutex owner is deleted while another task is blocked on it.
 *
 * Oracle:
 *   the blocked task resumes and owns the mutex                    <- HARD
 *
 * Defect signature:
 *   This is the highest value oracle in the family.  A recovery path that only
 *   increments the count without waking the waiter leaves that task blocked
 *   forever behind a semaphore whose count now looks available.  Nothing in
 *   the tree tests it today, and in the field it presents as a thread that
 *   simply stopped, with no fault and no log.
 *
 ****************************************************************************/

static void stc_sem_rec02_delete_owner_wakes_blocker(void)
{
	uint32_t violations;
	int ret;
	pid_t pid;

	ret = scenario_begin_mutex();
	TC_ASSERT_EQ("rec02_begin", ret, OK);

	pid = stc_spawn("stc_mown", STC_PRIO_LOW, mutex_owner_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("rec02_spawn_owner", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("rec02_owned", ret, OK, scenario_end());

	pid = stc_spawn("stc_mblk", STC_PRIO_HIGH, mutex_blocker_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("rec02_spawn_blocker", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count((sem_t *)&g_mutex.sem, -1);
	TC_ASSERT_EQ_CLEANUP("rec02_blocked", ret, OK, scenario_end());

	ret = task_delete(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("rec02_delete_owner", ret, OK, scenario_end());

	stc_actor_forget(SLOT_HOLDER);

	/* The whole point: the survivor must actually run again. */

	ret = stc_wait_stage(SLOT_WAITER, STAGE_ACQUIRED);
	TC_ASSERT_EQ_CLEANUP("rec02_blocker_resumed", ret, OK, scenario_end());

	ret = mutex_count();
	TC_ASSERT_EQ_CLEANUP("rec02_count_after_handover", ret, 0, scenario_end());

	stc_go(SLOT_WAITER);
	ret = stc_wait_count((sem_t *)&g_mutex.sem, 1);
	TC_ASSERT_EQ_CLEANUP("rec02_unlocked", ret, OK, scenario_end());

	ret = mutex_count();
	TC_ASSERT_LT_CLEANUP("rec02_mutex_invariant", ret, 2, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("rec02_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("rec02_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_rec03_delete_multi_holder
 *
 * Scenario: SCN-REC-03
 *   A task holding three semaphores is deleted.  Two of them have waiters.
 *
 * Oracle:
 *   (1) both waiters resume                                        <- HARD
 *   (2) the third semaphore returns to its free count              <- HARD
 *   (3) the delete completes rather than hanging                   <- HARD
 *
 * Defect signature:
 *   sem_release_all() walks the victim's holdsem list until it is empty.  A
 *   free that failed to unlink the entry from that list would spin forever,
 *   which this scenario detects as a bounded-wait failure instead of a wedged
 *   system.
 *
 ****************************************************************************/

static void stc_sem_rec03_delete_multi_holder(void)
{
	uint32_t violations;
	int ret;
	pid_t pid;

	ret = scenario_begin(1);
	TC_ASSERT_EQ("rec03_begin", ret, OK);

	ret = sem_init(&g_extra1, 0, 1);
	TC_ASSERT_EQ_CLEANUP("rec03_init_extra1", ret, OK, scenario_end());

	ret = sem_init(&g_extra2, 0, 1);
	TC_ASSERT_EQ_CLEANUP("rec03_init_extra2", ret, OK, scenario_end());

	g_extra_valid = true;
	stc_mon_register_sem(&g_extra1);
	stc_mon_register_sem(&g_extra2);

	pid = stc_spawn("stc_h3", STC_PRIO_LOW, three_sem_holder_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("rec03_spawn_holder", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("rec03_held_all", ret, OK, scenario_end());

	pid = stc_spawn("stc_w1", STC_PRIO_HIGH, survivor_waiter_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("rec03_spawn_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("rec03_waiter_blocked", ret, OK, scenario_end());

	ret = task_delete(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("rec03_delete_holder", ret, OK, scenario_end());

	stc_actor_forget(SLOT_HOLDER);

	ret = stc_wait_stage(SLOT_WAITER, STAGE_ACQUIRED);
	TC_ASSERT_EQ_CLEANUP("rec03_waiter_resumed", ret, OK, scenario_end());

	/* The semaphores with no waiters must simply return to their free count. */

	ret = stc_wait_count(&g_extra1, 1);
	TC_ASSERT_EQ_CLEANUP("rec03_extra1_released", ret, OK, scenario_end());

	ret = stc_wait_count(&g_extra2, 1);
	TC_ASSERT_EQ_CLEANUP("rec03_extra2_released", ret, OK, scenario_end());

	stc_go(SLOT_WAITER);
	ret = stc_wait_count(&g_target, 1);
	TC_ASSERT_EQ_CLEANUP("rec03_drained", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("rec03_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("rec03_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_rec05_delete_multi_count_holder
 *
 * Scenario: SCN-REC-05, a measurement scenario
 *   A task owning two counts of one counting semaphore is deleted while a
 *   waiter is blocked.
 *
 * Oracle:
 *   both counts are returned                                       <- HARD
 *
 * What this is really testing:
 *   sem_release_all() walks the victim's holdsem list and increments semcount
 *   once per holder *entry*.  A holder that took two counts has one entry
 *   carrying counts == 2, so a per-entry increment returns one count and
 *   silently drops the other.  The catalogue lists this as a measured
 *   scenario; the assertion here states the leak free expectation, so a
 *   failure is the detection, not a flaky test.  Whoever sees it fail should
 *   compare the reported counts against the drain arithmetic in this comment
 *   before assuming the test is wrong.
 *
 *   Arithmetic: the semaphore starts at 3.  The holder takes 2, leaving 1.
 *   The waiter takes the last one and then blocks on a second wait, so the
 *   count is -1.  Recovery must return the holder's 2 counts: -1 + 2 = 1,
 *   one of which is immediately handed to the blocked waiter, leaving 0 with
 *   the waiter holding 2.
 *
 ****************************************************************************/

static void stc_sem_rec05_delete_multi_count_holder(void)
{
	uint32_t violations;
	int before;
	int after;
	int ret;
	pid_t pid;

	ret = scenario_begin(3);
	TC_ASSERT_EQ("rec05_begin", ret, OK);

	pid = stc_spawn("stc_h2c", STC_PRIO_LOW, two_count_holder_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("rec05_spawn_holder", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("rec05_held_two", ret, OK, scenario_end());

	ret = stc_getcount(&g_target);
	TC_ASSERT_EQ_CLEANUP("rec05_one_left", ret, 1, scenario_end());

	pid = stc_spawn("stc_w", STC_PRIO_HIGH, survivor_waiter_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("rec05_spawn_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_WAITER, STAGE_ACQUIRED);
	TC_ASSERT_EQ_CLEANUP("rec05_waiter_took_free", ret, OK, scenario_end());

	before = stc_getcount(&g_target);

	ret = task_delete(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("rec05_delete_holder", ret, OK, scenario_end());

	stc_actor_forget(SLOT_HOLDER);

	/* Give recovery a bounded moment to complete before measuring. */

	(void)stc_wait_count(&g_target, before + 2);
	after = stc_getcount(&g_target);

	printf("[rec05] count before delete %d, after recovery %d, returned %d of 2\n",
		   before, after, after - before);

	TC_ASSERT_EQ_CLEANUP("rec05_both_counts_returned", after - before, 2, scenario_end());

	stc_go(SLOT_WAITER);

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("rec05_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("rec05_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_rec06_signal_delete_race
 *
 * Scenario: SCN-REC-06
 *   A signal and a delete are aimed at the same blocked waiter, in both
 *   orders, repeatedly.
 *
 * Oracle:
 *   the count is exactly one higher after every iteration          <- HARD
 *
 * Defect signature:
 *   sem_waitirq() and sem_recover() both return the waiter's count, and both
 *   guard on task_state == TSTATE_WAIT_SEM to make sure only one of them does.
 *   A missing guard shows up as a count one too high - and on the mutex
 *   flagged semaphore of SCN-REC-01 that is an immediate assertion failure
 *   rather than a quiet corruption.
 *
 ****************************************************************************/

static void stc_sem_rec06_signal_delete_race(void)
{
	uint32_t violations;
	int ret;
	int i;

	for (i = 0; i < RACE_ITERATIONS; i++) {
		ret = scenario_begin(1);
		TC_ASSERT_EQ("rec06_begin", ret, OK);

		if (stc_spawn("stc_hold", STC_PRIO_LOW, plain_holder_actor, SLOT_HOLDER) == (pid_t)ERROR) {
			TC_ASSERT_EQ_CLEANUP("rec06_spawn_holder", ERROR, OK, scenario_end());
		}

		ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
		TC_ASSERT_EQ_CLEANUP("rec06_held", ret, OK, scenario_end());

		if (stc_spawn("stc_doom", STC_PRIO_HIGH, doomed_waiter_actor, SLOT_WAITER) == (pid_t)ERROR) {
			TC_ASSERT_EQ_CLEANUP("rec06_spawn_waiter", ERROR, OK, scenario_end());
		}

		ret = stc_wait_count(&g_target, -1);
		TC_ASSERT_EQ_CLEANUP("rec06_blocked", ret, OK, scenario_end());

		/* Alternate the order so both interleavings are exercised. */

		if ((i & 1) == 0) {
			(void)kill(g_stc_actor[SLOT_WAITER].pid, SIGUSR1);
			(void)task_delete(g_stc_actor[SLOT_WAITER].pid);
		} else {
			(void)task_delete(g_stc_actor[SLOT_WAITER].pid);
			(void)kill(g_stc_actor[SLOT_WAITER].pid, SIGUSR1);
		}

		stc_actor_forget(SLOT_WAITER);

		ret = stc_wait_count(&g_target, 0);
		TC_ASSERT_EQ_CLEANUP("rec06_single_return", ret, OK, scenario_end());

		stc_go(SLOT_HOLDER);
		ret = stc_wait_count(&g_target, 1);
		TC_ASSERT_EQ_CLEANUP("rec06_final_count", ret, OK, scenario_end());

		ret = scenario_end();
		TC_ASSERT_EQ("rec06_no_leaked_actor", ret, 0);
	}

	violations = stc_mon_violations();
	TC_ASSERT_EQ("rec06_no_boost_leak", violations, 0);

	TC_SUCCESS_RESULT();
}


#ifndef CONFIG_PTHREAD_MUTEX_UNSAFE
/****************************************************************************
 * Name: robust_owner_actor
 *
 * Description:
 *   Locks the robust mutex and is then cancelled without unlocking, so that
 *   pthread_mutex_inconsistent() marks the mutex and wakes the blocker.
 *
 ****************************************************************************/

static int robust_owner_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	if (pthread_mutex_lock(&g_mutex) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_HELD);
	stc_wait_go(slot);

	/* Deliberately exits still holding the mutex. */

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Name: robust_blocker_actor
 *
 * Description:
 *   Blocks on the robust mutex and records the return code, which must be
 *   EOWNERDEAD once the owner has died holding it.
 *
 ****************************************************************************/

static int robust_blocker_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);
	int ret;

	(void)argc;

	stc_stage(slot, STAGE_STARTED);

	ret = pthread_mutex_lock(&g_mutex);
	g_cancel_errno = ret;
	stc_stage(slot, STAGE_ACQUIRED);

	stc_wait_go(slot);

	if (ret == EOWNERDEAD) {
		(void)pthread_mutex_consistent(&g_mutex);
	}

	if (ret == OK || ret == EOWNERDEAD) {
		(void)pthread_mutex_unlock(&g_mutex);
	}

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Name: stc_sem_rec04_robust_mutex_owner_dies
 *
 * Scenario: SCN-REC-04
 *   A robust mutex owner is deleted while another task blocks on it.
 *
 * Oracle:
 *   (1) the blocker acquires the mutex and is told EOWNERDEAD      <- HARD
 *   (2) after pthread_mutex_consistent() the mutex is usable again <- HARD
 *   (3) the underlying semaphore never reaches a count of 2        <- HARD
 *
 * Defect signature:
 *   pthread_mutex_inconsistent() walks the dying thread's mhead list, marks
 *   each mutex inconsistent and posts it.  If the mutex were also released a
 *   second time by sem_release_all(), the count would end one too high, which
 *   on a FLAGS_SEM_MUTEX semaphore is an assertion rather than a silent
 *   corruption.  Oracle (3) is what catches that.
 *
 ****************************************************************************/

static void stc_sem_rec04_robust_mutex_owner_dies(void)
{
	pthread_mutexattr_t attr;
	uint32_t violations;
	int ret;
	pid_t pid;

	ret = stc_reset();
	TC_ASSERT_EQ("rec04_reset", ret, OK);

	ret = pthread_mutexattr_init(&attr);
	TC_ASSERT_EQ("rec04_attr_init", ret, OK);

	ret = pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
	TC_ASSERT_EQ("rec04_attr_robust", ret, OK);

	ret = pthread_mutex_init(&g_mutex, &attr);
	TC_ASSERT_EQ("rec04_mutex_init", ret, OK);

	g_mutex_valid = true;
	stc_mon_register_sem((sem_t *)&g_mutex.sem);
	g_cancel_errno = 0;

	pid = stc_spawn("stc_rown", STC_PRIO_LOW, robust_owner_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("rec04_spawn_owner", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("rec04_owned", ret, OK, scenario_end());

	pid = stc_spawn("stc_rblk", STC_PRIO_HIGH, robust_blocker_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("rec04_spawn_blocker", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count((sem_t *)&g_mutex.sem, -1);
	TC_ASSERT_EQ_CLEANUP("rec04_blocked", ret, OK, scenario_end());

	ret = task_delete(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("rec04_delete_owner", ret, OK, scenario_end());

	stc_actor_forget(SLOT_HOLDER);

	ret = stc_wait_stage(SLOT_WAITER, STAGE_ACQUIRED);
	TC_ASSERT_EQ_CLEANUP("rec04_blocker_resumed", ret, OK, scenario_end());

	TC_ASSERT_EQ_CLEANUP("rec04_eownerdead", g_cancel_errno, EOWNERDEAD, scenario_end());

	ret = mutex_count();
	TC_ASSERT_LT_CLEANUP("rec04_mutex_invariant", ret, 2, scenario_end());

	stc_go(SLOT_WAITER);
	ret = stc_wait_finished(SLOT_WAITER);
	TC_ASSERT_EQ_CLEANUP("rec04_blocker_done", ret, OK, scenario_end());

	/* The mutex was made consistent, so a fresh lock must simply succeed. */

	ret = pthread_mutex_lock(&g_mutex);
	TC_ASSERT_EQ_CLEANUP("rec04_usable_again", ret, OK, scenario_end());

	ret = pthread_mutex_unlock(&g_mutex);
	TC_ASSERT_EQ_CLEANUP("rec04_unlock", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("rec04_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("rec04_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}
#else
static void stc_sem_rec04_robust_mutex_owner_dies(void)
{
	printf("\n[rec04] SKIP : needs CONFIG_PTHREAD_MUTEX_ROBUST "
		   "(CONFIG_PTHREAD_MUTEX_UNSAFE must be off)\n");
}
#endif							/* !CONFIG_PTHREAD_MUTEX_UNSAFE */

/****************************************************************************
 * Private Functions - Family F scenarios
 ****************************************************************************/

/****************************************************************************
 * Name: stc_sem_sig01_delete_blocked_on_signalling_sem
 *
 * Scenario: SCN-SIG-01
 *   A task blocked on a signalling semaphore - sem_init(..., 0), which sets
 *   FLAGS_SIGSEM - is deleted.
 *
 * Oracle:
 *   (1) the count is exactly one higher                            <- HARD
 *   (2) no holder record exists at any point                       <- SOFT
 *   (3) a later waiter still works normally                        <- HARD
 *
 * Defect signature:
 *   sem_canceled() asserts semcount <= 0 on entry.  Recovery on a signalling
 *   semaphore walks an empty holder list, so the assertion is the only thing
 *   standing between a correct count and a silent underflow.
 *
 ****************************************************************************/

static void stc_sem_sig01_delete_blocked_on_signalling_sem(void)
{
	struct sem_snapshot_s snap;
	uint32_t violations;
	int ret;
	pid_t pid;

	ret = scenario_begin(0);
	TC_ASSERT_EQ("sig01_begin", ret, OK);

	pid = stc_spawn("stc_doom", STC_PRIO_HIGH, doomed_waiter_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("sig01_spawn_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("sig01_blocked", ret, OK, scenario_end());

	if (stc_snapshot(&g_target, &snap) == OK) {
		TC_ASSERT_EQ_CLEANUP("sig01_is_sigsem", (snap.flags & FLAGS_SIGSEM) != 0, true, scenario_end());
		TC_ASSERT_EQ_CLEANUP("sig01_no_holder", snap.nholders, 0, scenario_end());
	}

	ret = task_delete(g_stc_actor[SLOT_WAITER].pid);
	TC_ASSERT_EQ_CLEANUP("sig01_delete_waiter", ret, OK, scenario_end());

	stc_actor_forget(SLOT_WAITER);

	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("sig01_count_returned", ret, OK, scenario_end());

	/* A fresh post and wait must still behave. */

	ret = sem_post(&g_target);
	TC_ASSERT_EQ_CLEANUP("sig01_post", ret, OK, scenario_end());

	ret = sem_wait(&g_target);
	TC_ASSERT_EQ_CLEANUP("sig01_wait", ret, OK, scenario_end());

	ret = stc_getcount(&g_target);
	TC_ASSERT_EQ_CLEANUP("sig01_final_count", ret, 0, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("sig01_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("sig01_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_sig02_delete_signaller_timeout_recovers
 *
 * Scenario: SCN-SIG-02
 *   The task that was going to post a signalling semaphore is deleted before
 *   it does.  The waiter used a timeout.
 *
 * Oracle:
 *   the waiter times out cleanly with ETIMEDOUT                    <- HARD
 *
 * What it establishes:
 *   Recovery cannot help here, and is not supposed to.  A signalling semaphore
 *   records no holder, so the dead task owns nothing that sem_release_all()
 *   could give back.  The waiter's own timeout is the only recovery mechanism
 *   that exists - which is the argument for requiring one in any signalling
 *   protocol that has to survive the death of its peer.
 *
 ****************************************************************************/

static void stc_sem_sig02_delete_signaller_timeout_recovers(void)
{
	uint32_t violations;
	int ret;
	pid_t pid;

	ret = scenario_begin(0);
	TC_ASSERT_EQ("sig02_begin", ret, OK);

	pid = stc_spawn("stc_sigr", STC_PRIO_LOW, never_signals_actor, SLOT_SIGNALLER);
	TC_ASSERT_NEQ_CLEANUP("sig02_spawn_signaller", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_SIGNALLER, STAGE_STARTED);
	TC_ASSERT_EQ_CLEANUP("sig02_signaller_ready", ret, OK, scenario_end());

	pid = stc_spawn("stc_tw", STC_PRIO_HIGH, sig_timedwait_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("sig02_spawn_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("sig02_blocked", ret, OK, scenario_end());

	ret = task_delete(g_stc_actor[SLOT_SIGNALLER].pid);
	TC_ASSERT_EQ_CLEANUP("sig02_delete_signaller", ret, OK, scenario_end());

	stc_actor_forget(SLOT_SIGNALLER);

	/* Nothing was owned, so nothing is returned: the count stays negative
	 * until the waiter's own timeout fires.
	 */

	ret = stc_getcount(&g_target);
	TC_ASSERT_EQ_CLEANUP("sig02_no_recovery_possible", ret, -1, scenario_end());

	ret = stc_wait_stage(SLOT_WAITER, STAGE_CANCELLED);
	TC_ASSERT_EQ_CLEANUP("sig02_timed_out", ret, OK, scenario_end());

	TC_ASSERT_EQ_CLEANUP("sig02_errno", g_cancel_errno, ETIMEDOUT, scenario_end());

	ret = stc_getcount(&g_target);
	TC_ASSERT_EQ_CLEANUP("sig02_count_restored", ret, 0, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("sig02_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("sig02_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_sig03_plain_wait_never_recovers
 *
 * Scenario: SCN-SIG-03, a characterisation scenario
 *   The same as SCN-SIG-02, but the waiter used a plain sem_wait().
 *
 * Oracle:
 *   the waiter is still blocked after the signaller is deleted     <- HARD
 *
 * Note:
 *   Catalogue rule G-7 - no task left blocked - is deliberately waived here.
 *   The scenario terminates by deleting the waiter itself, and asserts that
 *   this is clean.  The blocked-forever state is the documented consequence of
 *   signalling usage, not a defect, and pinning it down is the point: if a
 *   future change made recovery cover signalling semaphores, this scenario
 *   must be updated together with that change rather than silently passing.
 *
 ****************************************************************************/

static void stc_sem_sig03_plain_wait_never_recovers(void)
{
	uint32_t violations;
	int ret;
	pid_t pid;

	ret = scenario_begin(0);
	TC_ASSERT_EQ("sig03_begin", ret, OK);

	pid = stc_spawn("stc_sigr", STC_PRIO_LOW, never_signals_actor, SLOT_SIGNALLER);
	TC_ASSERT_NEQ_CLEANUP("sig03_spawn_signaller", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_SIGNALLER, STAGE_STARTED);
	TC_ASSERT_EQ_CLEANUP("sig03_signaller_ready", ret, OK, scenario_end());

	pid = stc_spawn("stc_doom", STC_PRIO_HIGH, doomed_waiter_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("sig03_spawn_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("sig03_blocked", ret, OK, scenario_end());

	ret = task_delete(g_stc_actor[SLOT_SIGNALLER].pid);
	TC_ASSERT_EQ_CLEANUP("sig03_delete_signaller", ret, OK, scenario_end());

	stc_actor_forget(SLOT_SIGNALLER);

	/* Characterisation: the waiter stays blocked, because nothing owns a count
	 * that recovery could hand back.
	 */

	usleep(STC_GRACE_MS * 1000);

	ret = stc_getcount(&g_target);
	TC_ASSERT_EQ_CLEANUP("sig03_still_blocked", ret, -1, scenario_end());

	TC_ASSERT_EQ_CLEANUP("sig03_waiter_not_finished", g_stc_actor[SLOT_WAITER].finished, false, scenario_end());

	/* Terminate the scenario by deleting the waiter, and check that is clean. */

	ret = task_delete(g_stc_actor[SLOT_WAITER].pid);
	TC_ASSERT_EQ_CLEANUP("sig03_delete_waiter", ret, OK, scenario_end());

	stc_actor_forget(SLOT_WAITER);

	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("sig03_waiter_recovered", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("sig03_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("sig03_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_sig04_surviving_holder_boost
 *
 * Scenario: SCN-SIG-04, a measurement scenario
 *   Two holders of a counting semaphore are boosted by a 160 waiter.  One
 *   holder is deleted.
 *
 * Oracle:
 *   (1) the waiter receives the released count                     <- HARD
 *   (2) the surviving holder is back at its base priority once it posts
 *                                                                  <- HARD
 *   Measured: the surviving holder's priority immediately after the delete.
 *
 * What this is really testing:
 *   sem_release_all() does not run the priority restore pass, so a co-holder
 *   that was boosted on behalf of the deleted task's waiter can stay elevated
 *   until it posts something of its own.  The measurement is printed rather
 *   than asserted, because the catalogue records this as a characterisation:
 *   what must never happen is that it is still elevated after it posts, and
 *   that is what oracle (2) asserts.
 *
 ****************************************************************************/

static void stc_sem_sig04_surviving_holder_boost(void)
{
	uint32_t violations;
	int after_delete;
	int ret;
	pid_t pid;

	ret = scenario_begin(2);
	TC_ASSERT_EQ("sig04_begin", ret, OK);

	pid = stc_spawn("stc_ha", STC_PRIO_LOW, plain_holder_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("sig04_spawn_h1", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("sig04_h1_held", ret, OK, scenario_end());

	pid = stc_spawn("stc_hb", STC_PRIO_LOW2, plain_holder_actor, SLOT_HOLDER2);
	TC_ASSERT_NEQ_CLEANUP("sig04_spawn_h2", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER2, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("sig04_h2_held", ret, OK, scenario_end());

	pid = stc_spawn("stc_w", STC_PRIO_EXTRA, survivor_waiter_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("sig04_spawn_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("sig04_blocked", ret, OK, scenario_end());

	ret = stc_getprio(g_stc_actor[SLOT_HOLDER2].pid);
	TC_ASSERT_EQ_CLEANUP("sig04_h2_boosted", ret, STC_PRIO_EXTRA, scenario_end());

	ret = task_delete(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("sig04_delete_h1", ret, OK, scenario_end());

	stc_actor_forget(SLOT_HOLDER);

	ret = stc_wait_stage(SLOT_WAITER, STAGE_ACQUIRED);
	TC_ASSERT_EQ_CLEANUP("sig04_waiter_resumed", ret, OK, scenario_end());

	after_delete = stc_getprio(g_stc_actor[SLOT_HOLDER2].pid);
	printf("[sig04] surviving holder priority after recovery %d, base %d\n",
		   after_delete, STC_PRIO_LOW2);

	/* Whatever the transient, the survivor must unwind when it posts. */

	stc_go(SLOT_HOLDER2);
	ret = stc_wait_stage(SLOT_HOLDER2, STC_STAGE_DONE);
	TC_ASSERT_EQ_CLEANUP("sig04_h2_posted", ret, OK, scenario_end());

	ret = stc_getprio(g_stc_actor[SLOT_HOLDER2].pid);
	if (ret != ERROR) {
		TC_ASSERT_EQ_CLEANUP("sig04_h2_restored", ret, STC_PRIO_LOW2, scenario_end());
	}

	stc_go(SLOT_WAITER);

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("sig04_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("sig04_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_sig05_holder_record_churn
 *
 * Scenario: SCN-SIG-05
 *   Repeated create, acquire and delete cycles.
 *
 * Oracle:
 *   the holder record count returns to zero every cycle            <- SOFT
 *   nothing asserts, nothing is left blocked                       <- HARD
 *
 * Defect signature:
 *   A holder container that is not returned to the free list on recovery is
 *   invisible until the pool is exhausted, at which point priority inheritance
 *   silently stops working for every semaphore in the system.  Counting the
 *   records per cycle turns that into an immediate failure.
 *
 ****************************************************************************/

static void stc_sem_sig05_holder_record_churn(void)
{
	struct sem_snapshot_s snap;
	bool snapshot_ok;
	int ret;
	int i;

	for (i = 0; i < CHURN_CYCLES; i++) {
		ret = scenario_begin(1);
		TC_ASSERT_EQ("sig05_begin", ret, OK);

		if (stc_spawn("stc_hold", STC_PRIO_LOW, plain_holder_actor, SLOT_HOLDER) == (pid_t)ERROR) {
			TC_ASSERT_EQ_CLEANUP("sig05_spawn", ERROR, OK, scenario_end());
		}

		ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
		TC_ASSERT_EQ_CLEANUP("sig05_held", ret, OK, scenario_end());

		snapshot_ok = (stc_snapshot(&g_target, &snap) == OK);
		if (snapshot_ok) {
			TC_ASSERT_EQ_CLEANUP("sig05_record_created", snap.nholders, 1, scenario_end());
		}

		ret = task_delete(g_stc_actor[SLOT_HOLDER].pid);
		TC_ASSERT_EQ_CLEANUP("sig05_delete", ret, OK, scenario_end());

		stc_actor_forget(SLOT_HOLDER);

		ret = stc_wait_count(&g_target, 1);
		TC_ASSERT_EQ_CLEANUP("sig05_count_returned", ret, OK, scenario_end());

		if (snapshot_ok && stc_snapshot(&g_target, &snap) == OK) {
			TC_ASSERT_EQ_CLEANUP("sig05_record_freed", snap.nholders, 0, scenario_end());
		}

		ret = scenario_end();
		TC_ASSERT_EQ("sig05_no_leaked_actor", ret, 0);
	}

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stc_sem_recovery_main(void)
{
	if (stc_harness_begin() != OK) {
		printf("\n[stc_sem_recovery] FAIL : cannot raise the harness priority\n");
		total_fail++;
		return ERROR;
	}

	if (stc_mon_start() != OK) {
		printf("\n[stc_sem_recovery] FAIL : cannot start the boost-leak monitor\n");
		total_fail++;
		stc_harness_end();
		return ERROR;
	}

	stc_sem_rec01_delete_blocked_on_mutex();
	stc_sem_rec02_delete_owner_wakes_blocker();
	stc_sem_rec03_delete_multi_holder();
	stc_sem_rec04_robust_mutex_owner_dies();
	stc_sem_rec05_delete_multi_count_holder();
	stc_sem_rec06_signal_delete_race();

	stc_sem_sig01_delete_blocked_on_signalling_sem();
	stc_sem_sig02_delete_signaller_timeout_recovers();
	stc_sem_sig03_plain_wait_never_recovers();
	stc_sem_sig04_surviving_holder_boost();
	stc_sem_sig05_holder_record_churn();

	stc_mon_stop();
	stc_harness_end();

	return OK;
}

#else							/* CONFIG_PRIORITY_INHERITANCE */

int stc_sem_recovery_main(void)
{
	printf("\n[stc_sem_recovery] SKIP : CONFIG_PRIORITY_INHERITANCE is not enabled\n");
	return OK;
}

#endif							/* CONFIG_PRIORITY_INHERITANCE */
