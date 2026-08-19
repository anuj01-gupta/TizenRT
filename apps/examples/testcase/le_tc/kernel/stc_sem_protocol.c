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

/// @file stc_sem_protocol.c
/// @brief Family D of the Priority Inheritance and Semaphore Recovery
///        Scenario Test Catalogue - protocol and flavour.
///
/// SCN-PIP-01  SEM_PRIO_NONE suppresses the boost
/// SCN-PIP-02  SEM_PRIO_INHERIT restores it
/// SCN-PIP-03  a PI disabled semaphore's waiters never influence a restore
/// SCN-PIP-04  a signalling semaphore never creates a permanent holder
/// SCN-PIP-05  pthread mutex protocol attribute, and the mutex invariant
///
/// SCN-PIP-01 is the scenario that section 1.2 of the catalogue says is
/// missing.  tc_semaphore_sem_setprotocol() asks the kernel to disable
/// inheritance on a semaphore and asserts that the call returned zero; it
/// never creates a second task and never reads a priority, so a kernel in
/// which SEM_PRIO_NONE sets no flag at all passes it unchanged.  This file
/// asserts the behaviour instead of the return code.

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

#define SLOT_HOLDER             0
#define SLOT_WAITER             1
#define SLOT_WAITER2            2

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_target;			/* the semaphore whose protocol varies    */
static sem_t g_normal;			/* always PI enabled, for SCN-PIP-03      */
static bool g_normal_valid;

static pthread_mutex_t g_mutex;
static bool g_mutex_valid;

/****************************************************************************
 * Private Functions - actors
 ****************************************************************************/

static int holder_actor(int argc, char *argv[])
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

/* Holds the PI disabled semaphore and a normal one (SCN-PIP-03). */

static int dual_holder_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	if (sem_wait(&g_target) != OK || sem_wait(&g_normal) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_HELD);

	stc_wait_go(slot);
	(void)sem_post(&g_normal);

	stc_wait_go(slot);
	(void)sem_post(&g_target);

	stc_actor_done(slot);
	return OK;
}

static int waiter_actor(int argc, char *argv[])
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

static int normal_waiter_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	stc_stage(slot, STAGE_STARTED);

	if (sem_wait(&g_normal) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_ACQUIRED);
	stc_wait_go(slot);

	(void)sem_post(&g_normal);

	stc_actor_done(slot);
	return OK;
}

/* Waits on the signalling semaphore, then parks (SCN-PIP-04). */

static int sig_waiter_actor(int argc, char *argv[])
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

	stc_actor_done(slot);
	return OK;
}

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

/****************************************************************************
 * Private Functions - helpers
 ****************************************************************************/

static int scenario_begin(int initial_count, int protocol)
{
	if (stc_reset() != OK) {
		return ERROR;
	}

	if (sem_init(&g_target, 0, initial_count) != OK) {
		return ERROR;
	}

	if (protocol >= 0 && sem_setprotocol(&g_target, protocol) != OK) {
		return ERROR;
	}

	stc_mon_register_sem(&g_target);

	return OK;
}

static int scenario_end(void)
{
	int leaked = stc_teardown();

	(void)sem_destroy(&g_target);

	if (g_normal_valid) {
		(void)sem_destroy(&g_normal);
		g_normal_valid = false;
	}

	if (g_mutex_valid) {
		(void)pthread_mutex_destroy(&g_mutex);
		g_mutex_valid = false;
	}

	return leaked;
}

/****************************************************************************
 * Name: hold_and_block
 ****************************************************************************/

static int hold_and_block(int waiter_prio)
{
	if (stc_spawn("stc_phold", STC_PRIO_LOW, holder_actor, SLOT_HOLDER) == (pid_t)ERROR) {
		return ERROR;
	}

	if (stc_wait_stage(SLOT_HOLDER, STAGE_HELD) != OK) {
		return ERROR;
	}

	if (stc_spawn("stc_pwait", waiter_prio, waiter_actor, SLOT_WAITER) == (pid_t)ERROR) {
		return ERROR;
	}

	return stc_wait_count(&g_target, -1);
}

static int drain_holder_and_waiter(void)
{
	stc_go(SLOT_HOLDER);

	if (stc_wait_count(&g_target, 0) != OK) {
		return ERROR;
	}

	if (stc_wait_stage(SLOT_WAITER, STAGE_ACQUIRED) != OK) {
		return ERROR;
	}

	stc_go(SLOT_WAITER);

	return stc_wait_count(&g_target, 1);
}

/****************************************************************************
 * Private Functions - scenarios
 ****************************************************************************/

/****************************************************************************
 * Name: stc_sem_pip01_prio_none_suppresses_boost
 *
 * Scenario: SCN-PIP-01
 *   sem_setprotocol(SEM_PRIO_NONE), then the SCN-PI-01 shape.
 *
 * Oracle:
 *   the holder's priority never changes                            <- HARD
 *
 ****************************************************************************/

static void stc_sem_pip01_prio_none_suppresses_boost(void)
{
	uint32_t violations;
	int ret;
	int prio;

	ret = scenario_begin(1, SEM_PRIO_NONE);
	TC_ASSERT_EQ("pip01_begin", ret, OK);

	ret = hold_and_block(STC_PRIO_HIGH);
	TC_ASSERT_EQ_CLEANUP("pip01_setup", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pip01_no_boost", prio, STC_PRIO_LOW, scenario_end());

	ret = drain_holder_and_waiter();
	TC_ASSERT_EQ_CLEANUP("pip01_drained", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pip01_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pip01_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pip02_prio_inherit_restores_boost
 *
 * Scenario: SCN-PIP-02
 *   SEM_PRIO_INHERIT set explicitly after SEM_PRIO_NONE.
 *
 * Oracle:
 *   boosting resumes                                               <- HARD
 *
 * Why both directions matter:
 *   SCN-PIP-01 alone would pass on a kernel where inheritance never works at
 *   all.  Running the same shape with the protocol flipped back is what
 *   distinguishes "the protocol was honoured" from "nothing ever boosts".
 *
 ****************************************************************************/

static void stc_sem_pip02_prio_inherit_restores_boost(void)
{
	uint32_t violations;
	int ret;
	int prio;

	ret = scenario_begin(1, SEM_PRIO_NONE);
	TC_ASSERT_EQ("pip02_begin", ret, OK);

	ret = sem_setprotocol(&g_target, SEM_PRIO_INHERIT);
	TC_ASSERT_EQ_CLEANUP("pip02_reenable", ret, OK, scenario_end());

	ret = hold_and_block(STC_PRIO_HIGH);
	TC_ASSERT_EQ_CLEANUP("pip02_setup", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pip02_boost_resumed", prio, STC_PRIO_HIGH, scenario_end());

	ret = drain_holder_and_waiter();
	TC_ASSERT_EQ_CLEANUP("pip02_drained", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	if (prio != ERROR) {
		TC_ASSERT_EQ_CLEANUP("pip02_restored", prio, STC_PRIO_LOW, scenario_end());
	}

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pip02_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pip02_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pip03_disabled_sem_never_contributes
 *
 * Scenario: SCN-PIP-03
 *   One task holds a SEM_PRIO_NONE semaphore and a normal one.  A 150 task
 *   waits on the PI disabled semaphore; a 130 task waits on the normal one.
 *   The normal one is posted.
 *
 * Oracle:
 *   the holder returns to base, not to 150                         <- HARD
 *
 * Defect signature:
 *   This is the regression for the explicit PRIOINHERIT_FLAGS_DISABLE check
 *   inside sem_findhighestwaiter().  A PI disabled semaphore can still appear
 *   on a task's holdsem list - holders are retained for fault recovery when
 *   CONFIG_BINMGR_RECOVERY is enabled - so the restore walk will visit it.  If
 *   it does not skip it, the waiters of a semaphore that is not supposed to
 *   participate in inheritance end up setting the holder's priority anyway.
 *
 * Note:
 *   Without CONFIG_BINMGR_RECOVERY, sem_setprotocol(SEM_PRIO_NONE) destroys
 *   the holder records outright, so the disabled semaphore never reaches the
 *   restore walk and the scenario passes trivially.  It is still run on both
 *   configurations: on the reference platform it documents the other branch,
 *   and on a BINMGR_RECOVERY build it is a real test.
 *
 ****************************************************************************/

static void stc_sem_pip03_disabled_sem_never_contributes(void)
{
	uint32_t violations;
	int ret;
	int prio;
	pid_t pid;

	ret = scenario_begin(1, SEM_PRIO_NONE);
	TC_ASSERT_EQ("pip03_begin", ret, OK);

	ret = sem_init(&g_normal, 0, 1);
	TC_ASSERT_EQ_CLEANUP("pip03_init_normal", ret, OK, scenario_end());

	g_normal_valid = true;
	stc_mon_register_sem(&g_normal);

	pid = stc_spawn("stc_dual", STC_PRIO_LOW, dual_holder_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("pip03_spawn_holder", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("pip03_held_both", ret, OK, scenario_end());

	/* A high priority waiter on the PI disabled semaphore.  It must not boost
	 * anybody, now or during any later restore.
	 */

	pid = stc_spawn("stc_wdis", STC_PRIO_EXTRA - 10, waiter_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("pip03_spawn_disabled_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("pip03_disabled_blocked", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pip03_no_boost_from_disabled", prio, STC_PRIO_LOW, scenario_end());

	/* A normal waiter, which legitimately boosts. */

	pid = stc_spawn("stc_wnorm", STC_PRIO_MID, normal_waiter_actor, SLOT_WAITER2);
	TC_ASSERT_NEQ_CLEANUP("pip03_spawn_normal_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_normal, -1);
	TC_ASSERT_EQ_CLEANUP("pip03_normal_blocked", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pip03_boost_from_normal", prio, STC_PRIO_MID, scenario_end());

	/* Release the normal semaphore.  The restore walk visits the holder's
	 * remaining entry, which is the PI disabled semaphore with a 150 waiter.
	 */

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_normal, 0);
	TC_ASSERT_EQ_CLEANUP("pip03_post_normal", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("pip03_restored_to_base", prio, STC_PRIO_LOW, scenario_end());

	/* Drain everything. */

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("pip03_post_disabled", ret, OK, scenario_end());

	ret = stc_wait_stage(SLOT_WAITER, STAGE_ACQUIRED);
	TC_ASSERT_EQ_CLEANUP("pip03_disabled_acquired", ret, OK, scenario_end());

	stc_go(SLOT_WAITER);
	ret = stc_wait_count(&g_target, 1);
	TC_ASSERT_EQ_CLEANUP("pip03_disabled_drained", ret, OK, scenario_end());

	ret = stc_wait_stage(SLOT_WAITER2, STAGE_ACQUIRED);
	TC_ASSERT_EQ_CLEANUP("pip03_normal_acquired", ret, OK, scenario_end());

	stc_go(SLOT_WAITER2);
	ret = stc_wait_count(&g_normal, 1);
	TC_ASSERT_EQ_CLEANUP("pip03_normal_drained", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pip03_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pip03_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pip04_signalling_sem_has_no_holder
 *
 * Scenario: SCN-PIP-04
 *   sem_init(&s, 0, 0) sets FLAGS_SIGSEM.  A task waits, another posts, and a
 *   third task then blocks on the same semaphore.
 *
 * Oracle:
 *   (1) no holder record exists at any point                       <- SOFT
 *   (2) the task that was granted the count is never boosted       <- HARD
 *
 * What it establishes:
 *   The classic failure mode of a semaphore used for signalling is that the
 *   awakened waiter becomes a permanent holder and is boosted by every later
 *   waiter forever - which is exactly what sem_setprotocol()'s own
 *   documentation describes as the reason SEM_PRIO_NONE exists.  The
 *   FLAGS_SIGSEM auto-detection in sem_init() is supposed to make that manual
 *   fix unnecessary for zero initialised semaphores.  This asserts that it
 *   does.
 *
 ****************************************************************************/

static void stc_sem_pip04_signalling_sem_has_no_holder(void)
{
	struct sem_snapshot_s snap;
	uint32_t violations;
	int ret;
	int prio;
	pid_t pid;

	ret = scenario_begin(0, -1);
	TC_ASSERT_EQ("pip04_begin", ret, OK);

	if (stc_snapshot(&g_target, &snap) == OK) {
		TC_ASSERT_EQ_CLEANUP("pip04_is_sigsem", (snap.flags & FLAGS_SIGSEM) != 0, true, scenario_end());
	}

	pid = stc_spawn("stc_sigw", STC_PRIO_LOW, sig_waiter_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("pip04_spawn_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("pip04_blocked", ret, OK, scenario_end());

	ret = sem_post(&g_target);
	TC_ASSERT_EQ_CLEANUP("pip04_post", ret, OK, scenario_end());

	ret = stc_wait_stage(SLOT_WAITER, STAGE_ACQUIRED);
	TC_ASSERT_EQ_CLEANUP("pip04_acquired", ret, OK, scenario_end());

	if (stc_snapshot(&g_target, &snap) == OK) {
		TC_ASSERT_EQ_CLEANUP("pip04_no_holder_recorded", snap.nholders, 0, scenario_end());
	}

	/* A second, higher priority waiter.  The task that took the count is not a
	 * holder, so it must not be boosted.
	 */

	pid = stc_spawn("stc_sigw2", STC_PRIO_EXTRA, sig_waiter_actor, SLOT_WAITER2);
	TC_ASSERT_NEQ_CLEANUP("pip04_spawn_waiter2", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("pip04_second_blocked", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_WAITER].pid);
	TC_ASSERT_EQ_CLEANUP("pip04_first_not_boosted", prio, STC_PRIO_LOW, scenario_end());

	ret = sem_post(&g_target);
	TC_ASSERT_EQ_CLEANUP("pip04_post2", ret, OK, scenario_end());

	ret = stc_wait_stage(SLOT_WAITER2, STAGE_ACQUIRED);
	TC_ASSERT_EQ_CLEANUP("pip04_second_acquired", ret, OK, scenario_end());

	stc_go(SLOT_WAITER);
	stc_go(SLOT_WAITER2);

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("pip04_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("pip04_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_pip05_mutex_protocol_attribute
 *
 * Scenario: SCN-PIP-05
 *   A mutex initialised with PTHREAD_PRIO_INHERIT and then with
 *   PTHREAD_PRIO_NONE.
 *
 * Oracle:
 *   (1) the boost is present in the first and absent in the second <- HARD
 *   (2) the underlying semaphore never reaches a count of 2        <- HARD
 *
 ****************************************************************************/

static void stc_sem_pip05_mutex_protocol_attribute(void)
{
	pthread_mutexattr_t attr;
	uint32_t violations;
	int ret;
	int prio;
	int proto;
	int run;

	for (run = 0; run < 2; run++) {
		proto = (run == 0) ? PTHREAD_PRIO_INHERIT : PTHREAD_PRIO_NONE;

		ret = stc_reset();
		TC_ASSERT_EQ("pip05_reset", ret, OK);

		ret = pthread_mutexattr_init(&attr);
		TC_ASSERT_EQ("pip05_attr_init", ret, OK);

		ret = pthread_mutexattr_setprotocol(&attr, proto);
		TC_ASSERT_EQ("pip05_attr_protocol", ret, OK);

		ret = pthread_mutex_init(&g_mutex, &attr);
		TC_ASSERT_EQ("pip05_mutex_init", ret, OK);

		g_mutex_valid = true;
		stc_mon_register_sem((sem_t *)&g_mutex.sem);

		if (stc_spawn("stc_mown", STC_PRIO_LOW, mutex_owner_actor, SLOT_HOLDER) == (pid_t)ERROR) {
			TC_ASSERT_EQ_CLEANUP("pip05_spawn_owner", ERROR, OK, scenario_end());
		}

		ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
		TC_ASSERT_EQ_CLEANUP("pip05_owned", ret, OK, scenario_end());

		ret = stc_getcount((sem_t *)&g_mutex.sem);
		TC_ASSERT_LT_CLEANUP("pip05_invariant_locked", ret, 2, scenario_end());

		if (stc_spawn("stc_mblk", STC_PRIO_HIGH, mutex_blocker_actor, SLOT_WAITER) == (pid_t)ERROR) {
			TC_ASSERT_EQ_CLEANUP("pip05_spawn_blocker", ERROR, OK, scenario_end());
		}

		ret = stc_wait_count((sem_t *)&g_mutex.sem, -1);
		TC_ASSERT_EQ_CLEANUP("pip05_blocked", ret, OK, scenario_end());

		prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
		if (proto == PTHREAD_PRIO_INHERIT) {
			TC_ASSERT_EQ_CLEANUP("pip05_inherit_boosts", prio, STC_PRIO_HIGH, scenario_end());
		} else {
			TC_ASSERT_EQ_CLEANUP("pip05_none_does_not_boost", prio, STC_PRIO_LOW, scenario_end());
		}

		stc_go(SLOT_HOLDER);
		ret = stc_wait_count((sem_t *)&g_mutex.sem, 0);
		TC_ASSERT_EQ_CLEANUP("pip05_unlocked", ret, OK, scenario_end());

		ret = stc_wait_stage(SLOT_WAITER, STAGE_ACQUIRED);
		TC_ASSERT_EQ_CLEANUP("pip05_blocker_acquired", ret, OK, scenario_end());

		stc_go(SLOT_WAITER);
		ret = stc_wait_count((sem_t *)&g_mutex.sem, 1);
		TC_ASSERT_EQ_CLEANUP("pip05_drained", ret, OK, scenario_end());

		ret = stc_getcount((sem_t *)&g_mutex.sem);
		TC_ASSERT_LT_CLEANUP("pip05_invariant_final", ret, 2, scenario_end());

		violations = stc_mon_violations();
		TC_ASSERT_EQ_CLEANUP("pip05_no_boost_leak", violations, 0, scenario_end());

		ret = scenario_end();
		TC_ASSERT_EQ("pip05_no_leaked_actor", ret, 0);
	}

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stc_sem_protocol_main(void)
{
	if (stc_harness_begin() != OK) {
		printf("\n[stc_sem_protocol] FAIL : cannot raise the harness priority\n");
		total_fail++;
		return ERROR;
	}

	if (stc_mon_start() != OK) {
		printf("\n[stc_sem_protocol] FAIL : cannot start the boost-leak monitor\n");
		total_fail++;
		stc_harness_end();
		return ERROR;
	}

	stc_sem_pip01_prio_none_suppresses_boost();
	stc_sem_pip02_prio_inherit_restores_boost();
	stc_sem_pip03_disabled_sem_never_contributes();
	stc_sem_pip04_signalling_sem_has_no_holder();
	stc_sem_pip05_mutex_protocol_attribute();

	stc_mon_stop();
	stc_harness_end();

	return OK;
}

#else							/* CONFIG_PRIORITY_INHERITANCE */

int stc_sem_protocol_main(void)
{
	printf("\n[stc_sem_protocol] SKIP : CONFIG_PRIORITY_INHERITANCE is not enabled\n");
	return OK;
}

#endif							/* CONFIG_PRIORITY_INHERITANCE */
