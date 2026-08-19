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

/// @file stc_sem_monitor.c
/// @brief SCN-MON-01 - the boost-leak invariant monitor.

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
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

/* Slots used by the SCN-MON-01 scenarios themselves. */

#define MON_SLOT_HOLDER         0
#define MON_SLOT_WAITER         1
#define MON_SLOT_PARKED         2

#define MON_STAGE_HELD          2
#define MON_STAGE_ACQUIRED      2
#define MON_STAGE_PARKED        1

/* Boost/restore cycles executed by the negative control. */

#define MON_WORKLOAD_CYCLES     10

/* How long the positive control holds its fabricated boost.  Comfortably
 * longer than STC_MON_K sampling periods, so the detection is not itself a
 * race.
 */

#define MON_FABRICATED_MS       200

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct mon_actor_s {
	volatile pid_t pid;			/* written last; 0 means the slot is free */
	int base_prio;
	uint8_t consecutive;		/* consecutive boosted samples            */
	bool reported;				/* one violation per episode              */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct mon_actor_s g_mon_actor[STC_MON_MAX_ACTORS];
static sem_t *g_mon_sem[STC_MON_MAX_SEMS];
static volatile uint32_t g_mon_violations;

static volatile bool g_mon_run;
static volatile bool g_mon_exited;
static pid_t g_mon_pid;

/* The semaphore under test in the SCN-MON-01 scenarios. */

static sem_t g_mon_target;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mon_boost_justified
 *
 * Description:
 *   True when any registered semaphore currently has at least one waiter.  A
 *   negative count is exactly "this many tasks are blocked here", so this is
 *   an exact test, not an estimate.
 *
 ****************************************************************************/

static bool mon_boost_justified(void)
{
	int value;
	int i;

	for (i = 0; i < STC_MON_MAX_SEMS; i++) {
		if (g_mon_sem[i] == NULL) {
			continue;
		}

		if (sem_getvalue(g_mon_sem[i], &value) == OK && value < 0) {
			return true;
		}
	}

	return false;
}

/****************************************************************************
 * Name: mon_sample
 *
 * Description:
 *   One pass over the registry.  An actor whose task has exited makes
 *   sched_getparam() fail; its slot is released rather than reported.
 *
 ****************************************************************************/

static void mon_sample(void)
{
	bool justified = mon_boost_justified();
	struct sched_param param;
	int i;

	for (i = 0; i < STC_MON_MAX_ACTORS; i++) {
		pid_t pid = g_mon_actor[i].pid;

		if (pid == 0) {
			continue;
		}

		if (sched_getparam(pid, &param) != OK) {
			/* The actor is gone.  Free the slot so a later scenario can
			 * reuse it, and do not count anything against it.
			 */

			g_mon_actor[i].pid = 0;
			continue;
		}

		if (param.sched_priority <= g_mon_actor[i].base_prio) {
			/* Back at or below base: the episode, if any, is over. */

			g_mon_actor[i].consecutive = 0;
			g_mon_actor[i].reported = false;
			continue;
		}

		if (justified) {
			/* Somebody is waiting somewhere, so this boost is legitimate as
			 * far as this monitor can tell.
			 */

			g_mon_actor[i].consecutive = 0;
			continue;
		}

		if (g_mon_actor[i].consecutive < STC_MON_K) {
			g_mon_actor[i].consecutive++;
		}

		if (g_mon_actor[i].consecutive >= STC_MON_K && !g_mon_actor[i].reported) {
			g_mon_actor[i].reported = true;
			g_mon_violations++;
		}
	}
}

/****************************************************************************
 * Name: mon_task
 ****************************************************************************/

static int mon_task(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	g_mon_exited = false;

	while (g_mon_run) {
		usleep(STC_MON_PERIOD_US);
		mon_sample();
	}

	g_mon_exited = true;
	return OK;
}

/****************************************************************************
 * Public Functions - registry
 ****************************************************************************/

void stc_mon_clear(void)
{
	memset(g_mon_actor, 0, sizeof(g_mon_actor));
	memset(g_mon_sem, 0, sizeof(g_mon_sem));
	g_mon_violations = 0;
}

/****************************************************************************
 * Name: stc_mon_register_actor
 *
 * Description:
 *   The pid field is written last, and the monitor skips entries whose pid is
 *   zero, so a sample that lands in the middle of a registration sees the slot
 *   as free rather than half-initialised.
 *
 ****************************************************************************/

void stc_mon_register_actor(pid_t pid, int base_prio)
{
	int i;

	for (i = 0; i < STC_MON_MAX_ACTORS; i++) {
		if (g_mon_actor[i].pid == 0) {
			g_mon_actor[i].base_prio = base_prio;
			g_mon_actor[i].consecutive = 0;
			g_mon_actor[i].reported = false;
			g_mon_actor[i].pid = pid;
			return;
		}
	}
}

void stc_mon_register_sem(sem_t *sem)
{
	int i;

	for (i = 0; i < STC_MON_MAX_SEMS; i++) {
		if (g_mon_sem[i] == sem) {
			return;
		}
	}

	for (i = 0; i < STC_MON_MAX_SEMS; i++) {
		if (g_mon_sem[i] == NULL) {
			g_mon_sem[i] = sem;
			return;
		}
	}
}

uint32_t stc_mon_violations(void)
{
	return g_mon_violations;
}

/****************************************************************************
 * Public Functions - lifecycle
 ****************************************************************************/

/****************************************************************************
 * Name: stc_mon_start
 *
 * Description:
 *   The monitor is deliberately left unpinned.  The actors are pinned to CPU0,
 *   so on an SMP build the monitor tends to sample from the other CPU and does
 *   not compete with the workload it is measuring.  The cost is that a sample
 *   can land concurrently with a post, which is exactly what STC_MON_K
 *   absorbs: a transient has to survive three consecutive samples to count.
 *
 ****************************************************************************/

int stc_mon_start(void)
{
	char *argv[1];

	if (g_mon_run) {
		return OK;
	}

	g_mon_run = true;
	g_mon_exited = false;

	argv[0] = NULL;

	g_mon_pid = task_create("stc_mon", STC_PRIO_MONITOR, STC_STACKSIZE, mon_task, (char *const *)argv);
	if (g_mon_pid <= 0) {
		g_mon_run = false;
		return ERROR;
	}

	return OK;
}

void stc_mon_stop(void)
{
	int retries = (STC_TIMEOUT_MS * 1000) / STC_POLL_US;

	if (!g_mon_run) {
		return;
	}

	g_mon_run = false;

	while (retries-- > 0 && !g_mon_exited) {
		usleep(STC_POLL_US);
	}

	if (!g_mon_exited && g_mon_pid > 0) {
		(void)task_delete(g_mon_pid);
	}

	g_mon_pid = 0;
}

/****************************************************************************
 * Private Functions - SCN-MON-01 actors
 ****************************************************************************/

static int mon_holder_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	if (sem_wait(&g_mon_target) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, MON_STAGE_HELD);
	stc_wait_go(slot);

	(void)sem_post(&g_mon_target);

	stc_actor_done(slot);
	return OK;
}

static int mon_waiter_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	if (sem_wait(&g_mon_target) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, MON_STAGE_ACQUIRED);
	stc_wait_go(slot);

	(void)sem_post(&g_mon_target);

	stc_actor_done(slot);
	return OK;
}

/* Touches no semaphore at all.  Used by the positive control, whose whole
 * point is a boost that no semaphore can justify.
 */

static int mon_parked_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	stc_stage(slot, MON_STAGE_PARKED);
	stc_wait_go(slot);

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Private Functions - helpers
 ****************************************************************************/

static int mon_scenario_begin(int initial_count)
{
	if (stc_reset() != OK) {
		return ERROR;
	}

	if (sem_init(&g_mon_target, 0, initial_count) != OK) {
		return ERROR;
	}

	stc_mon_register_sem(&g_mon_target);

	return OK;
}

static int mon_scenario_end(void)
{
	int leaked = stc_teardown();

	(void)sem_destroy(&g_mon_target);

	return leaked;
}

/****************************************************************************
 * Name: mon_one_boost_cycle
 *
 * Description:
 *   One complete boost-and-restore cycle: a low holder, a high waiter, the
 *   post, and the drain.  Exactly the traffic the monitor must not flag.
 *
 ****************************************************************************/

static int mon_one_boost_cycle(void)
{
	if (stc_spawn("stc_mhold", STC_PRIO_LOW, mon_holder_actor, MON_SLOT_HOLDER) == (pid_t)ERROR) {
		return ERROR;
	}

	if (stc_wait_stage(MON_SLOT_HOLDER, MON_STAGE_HELD) != OK) {
		return ERROR;
	}

	if (stc_spawn("stc_mwait", STC_PRIO_HIGH, mon_waiter_actor, MON_SLOT_WAITER) == (pid_t)ERROR) {
		return ERROR;
	}

	if (stc_wait_count(&g_mon_target, -1) != OK) {
		return ERROR;
	}

	stc_go(MON_SLOT_HOLDER);
	if (stc_wait_count(&g_mon_target, 0) != OK) {
		return ERROR;
	}

	if (stc_wait_stage(MON_SLOT_WAITER, MON_STAGE_ACQUIRED) != OK) {
		return ERROR;
	}

	stc_go(MON_SLOT_WAITER);
	if (stc_wait_count(&g_mon_target, 1) != OK) {
		return ERROR;
	}

	if (stc_wait_finished(MON_SLOT_HOLDER) != OK) {
		return ERROR;
	}

	return stc_wait_finished(MON_SLOT_WAITER);
}

/****************************************************************************
 * Private Functions - scenarios
 ****************************************************************************/

/****************************************************************************
 * Name: stc_sem_mon01_clean_workload
 *
 * Scenario: SCN-MON-01, negative control
 *   Run ten complete boost-and-restore cycles with the monitor active.
 *
 * Oracle:
 *   zero violations                                                <- HARD
 *
 * What it proves:
 *   The monitor does not fire on correct inheritance traffic.  Without this
 *   half, a monitor that reported a violation on every boost would look like
 *   a working detector.
 *
 ****************************************************************************/

static void stc_sem_mon01_clean_workload(void)
{
	uint32_t violations;
	int ret;
	int i;

	ret = mon_scenario_begin(1);
	TC_ASSERT_EQ("mon01_clean_begin", ret, OK);

	ret = stc_mon_start();
	TC_ASSERT_EQ_CLEANUP("mon01_clean_monitor", ret, OK, mon_scenario_end());

	for (i = 0; i < MON_WORKLOAD_CYCLES; i++) {
		ret = mon_one_boost_cycle();
		if (ret != OK) {
			break;
		}
	}

	stc_mon_stop();

	TC_ASSERT_EQ_CLEANUP("mon01_clean_workload", ret, OK, mon_scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("mon01_no_false_positive", violations, 0, mon_scenario_end());

	ret = mon_scenario_end();
	TC_ASSERT_EQ("mon01_clean_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_mon01_positive_control
 *
 * Scenario: SCN-MON-01, positive control
 *   Fabricate the exact condition the monitor exists to catch: a task above
 *   the priority it was created with, while no registered semaphore has any
 *   waiter.  The actor holds nothing and touches nothing.
 *
 * Oracle:
 *   at least one violation                                         <- HARD
 *
 * What it proves:
 *   The detector can fail.  This is the Phase 2 exit criterion from catalogue
 *   section 8.2: a monitor that has never been observed to fire is not
 *   evidence of anything.
 *
 * Note:
 *   sched_setparam() also rewrites the task's own base_priority, but the
 *   monitor compares against the priority the harness recorded at creation,
 *   so the fabricated boost is still visible to it.  That is precisely the
 *   shape of a real leak: a task running above where it started, with nothing
 *   waiting to justify it.
 *
 ****************************************************************************/

static void stc_sem_mon01_positive_control(void)
{
	struct sched_param param;
	uint32_t violations;
	int elapsed;
	int ret;
	pid_t pid;

	ret = mon_scenario_begin(1);
	TC_ASSERT_EQ("mon01_pos_begin", ret, OK);

	pid = stc_spawn("stc_mpark", STC_PRIO_LOW, mon_parked_actor, MON_SLOT_PARKED);
	TC_ASSERT_NEQ_CLEANUP("mon01_pos_spawn", pid, (pid_t)ERROR, mon_scenario_end());

	ret = stc_wait_stage(MON_SLOT_PARKED, MON_STAGE_PARKED);
	TC_ASSERT_EQ_CLEANUP("mon01_pos_parked", ret, OK, mon_scenario_end());

	ret = stc_mon_start();
	TC_ASSERT_EQ_CLEANUP("mon01_pos_monitor", ret, OK, mon_scenario_end());

	/* Fabricate the leak.  No semaphore has a waiter, so nothing justifies
	 * this actor sitting above the priority it was created with.
	 */

	param.sched_priority = STC_PRIO_HIGH;
	ret = sched_setparam(pid, &param);
	TC_ASSERT_EQ_CLEANUP("mon01_pos_fabricate", ret, OK, mon_scenario_end());

	/* Hold it there long enough for STC_MON_K consecutive samples, but stop
	 * as soon as the monitor reacts so a healthy run stays quick.
	 */

	for (elapsed = 0; elapsed < MON_FABRICATED_MS; elapsed += (STC_MON_PERIOD_US / 1000)) {
		if (stc_mon_violations() > 0) {
			break;
		}

		usleep(STC_MON_PERIOD_US);
	}

	violations = stc_mon_violations();

	/* Put it back before tearing down, so the actor exits normally. */

	param.sched_priority = STC_PRIO_LOW;
	(void)sched_setparam(pid, &param);

	stc_mon_stop();

	stc_go(MON_SLOT_PARKED);
	ret = stc_wait_finished(MON_SLOT_PARKED);
	TC_ASSERT_EQ_CLEANUP("mon01_pos_actor_done", ret, OK, mon_scenario_end());

	TC_ASSERT_GEQ_CLEANUP("mon01_detects_leak", violations, 1, mon_scenario_end());

	ret = mon_scenario_end();
	TC_ASSERT_EQ("mon01_pos_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stc_sem_monitor_main(void)
{
	if (stc_harness_begin() != OK) {
		printf("\n[stc_sem_monitor] FAIL : cannot raise the harness priority\n");
		total_fail++;
		return ERROR;
	}

	stc_sem_mon01_clean_workload();
	stc_sem_mon01_positive_control();

	stc_harness_end();

	return OK;
}

#else							/* CONFIG_PRIORITY_INHERITANCE */

void stc_mon_clear(void)
{
}

void stc_mon_register_actor(pid_t pid, int base_prio)
{
	(void)pid;
	(void)base_prio;
}

void stc_mon_register_sem(sem_t *sem)
{
	(void)sem;
}

int stc_mon_start(void)
{
	return OK;
}

void stc_mon_stop(void)
{
}

uint32_t stc_mon_violations(void)
{
	return 0;
}

int stc_sem_monitor_main(void)
{
	printf("\n[stc_sem_monitor] SKIP : CONFIG_PRIORITY_INHERITANCE is not enabled\n");
	return OK;
}

#endif							/* CONFIG_PRIORITY_INHERITANCE */
