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

/// @file stc_sem_smp.c
/// @brief Families I and J of the Priority Inheritance and Semaphore Recovery
///        Scenario Test Catalogue - SMP interaction and soak.
///
/// Family I - SMP interaction
///   SCN-SMP-01  holder and waiter on different CPUs
///   SCN-SMP-02  a task on the other CPU posts on the holder's behalf
///   SCN-SMP-04  sustained contention across both CPUs
///
/// Family J - soak
///   SCN-SOAK-01 mixed workload with randomised parameters
///
/// Every other family pins its actors to one CPU, because an exact priority at
/// an exact moment is only well defined when nothing else can be running.
/// This file exists to cover the case those scenarios deliberately exclude:
/// holder and waiter genuinely concurrent, with the boost applied from one CPU
/// and observed from another.
///
/// The oracles here are therefore weaker by construction, and that is stated
/// rather than hidden.  A sample taken while a post is in flight on the other
/// CPU can legitimately catch either the boosted or the restored value, so the
/// scenarios assert settled states and bounded transitions rather than exact
/// instantaneous priorities.  The boost-leak monitor of phase 2 does the
/// sharper work here: it is what turns "eventually correct" into "never leaks",
/// and it runs throughout.
///
/// SCN-SMP-03, the interrupt context restore path, is not implemented.  It
/// needs a semaphore posted from a real interrupt handler, which from the
/// application side means a watchdog or timer callback executing in interrupt
/// context.  Reaching sem_restorebaseprio_irq() reliably is driver work in
/// os/drivers/os_api_test rather than a user space scenario, and doing it
/// badly would produce a test that silently exercises the task path instead.
/// The gap is reported in the run log.

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

#define STAGE_STARTED           1
#define STAGE_HELD              2
#define STAGE_ACQUIRED          3

#define SLOT_HOLDER             0
#define SLOT_WAITER             1
#define SLOT_POSTER             2

/* SCN-SMP-04 and SCN-SOAK-01.  The soak length is a compromise: the catalogue
 * asks for thirty minutes in a nightly job, which is far too long for a tier
 * that also runs interactively, so the loop count is expressed in cycles and
 * the nightly job is expected to raise it.
 */

#define CONTENTION_CYCLES       200
#define SOAK_CYCLES             300

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_target;
static volatile uint32_t g_acquisitions;

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

/* Holds without posting: SCN-SMP-02 has a third task return the count. */

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

static int waiter_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	stc_stage(slot, STAGE_STARTED);

	if (sem_wait(&g_target) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	g_acquisitions++;
	stc_stage(slot, STAGE_ACQUIRED);
	stc_wait_go(slot);

	(void)sem_post(&g_target);

	stc_actor_done(slot);
	return OK;
}

static int poster_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	stc_stage(slot, STAGE_STARTED);
	stc_wait_go(slot);

	(void)sem_post(&g_target);

	stc_actor_done(slot);
	return OK;
}

/* Lock, brief work, unlock, repeat.  Used by SCN-SMP-04 and the soak. */

static int contender_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);
	int i;

	(void)argc;

	stc_stage(slot, STAGE_STARTED);

	for (i = 0; i < CONTENTION_CYCLES; i++) {
		if (sem_wait(&g_target) != OK) {
			break;
		}

		g_acquisitions++;
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
	g_acquisitions = 0;

	return OK;
}

static int scenario_end(void)
{
	int leaked = stc_teardown();

	(void)sem_destroy(&g_target);

	return leaked;
}

/****************************************************************************
 * Name: spawn_on_cpu
 *
 * Description:
 *   Create an actor and move it to the given CPU.  stc_spawn() pins to CPU0,
 *   so this overrides that for the scenarios whose whole point is that the
 *   actors are on different CPUs.
 *
 ****************************************************************************/

static pid_t spawn_on_cpu(const char *name, int prio, main_t entry, int slot, int cpu)
{
	pid_t pid = stc_spawn(name, prio, entry, slot);

#ifdef CONFIG_SMP
	cpu_set_t mask;

	if (pid != (pid_t)ERROR) {
		mask = (cpu_set_t)(1 << cpu);
		(void)sched_setaffinity(pid, sizeof(cpu_set_t), &mask);
	}
#else
	(void)cpu;
#endif

	return pid;
}

/****************************************************************************
 * Name: other_cpu
 *
 * Description:
 *   CPU 1 on an SMP build with more than one CPU, CPU 0 otherwise, so the
 *   scenarios degrade to the single core case instead of failing on it.
 *
 ****************************************************************************/

static int other_cpu(void)
{
	return (sched_getcpucount() > 1) ? 1 : 0;
}

/****************************************************************************
 * Private Functions - scenarios
 ****************************************************************************/

/****************************************************************************
 * Name: stc_sem_smp01_cross_cpu_boost
 *
 * Scenario: SCN-SMP-01
 *   The holder runs on CPU0, the waiter on CPU1.
 *
 * Oracle:
 *   (1) once the waiter has blocked, the holder is at the waiter's
 *       priority                                                   <- HARD
 *   (2) once the post has settled, the holder is back at base       <- HARD
 *
 * Why these are settled-state oracles:
 *   The blocked state is established by the semaphore count, which is only
 *   reached after sem_wait() has completed its boost inside the critical
 *   section, so oracle (1) is not racing the boost.  Likewise the count
 *   returning to 0 implies the post and its restore have both finished.  What
 *   this scenario cannot do is assert anything about the instant in between,
 *   because on two CPUs there is no such shared instant.
 *
 ****************************************************************************/

static void stc_sem_smp01_cross_cpu_boost(void)
{
	uint32_t violations;
	int ret;
	int prio;
	pid_t pid;

	ret = scenario_begin(1);
	TC_ASSERT_EQ("smp01_begin", ret, OK);

	pid = spawn_on_cpu("stc_shold", STC_PRIO_LOW, holder_actor, SLOT_HOLDER, 0);
	TC_ASSERT_NEQ_CLEANUP("smp01_spawn_holder", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("smp01_held", ret, OK, scenario_end());

	pid = spawn_on_cpu("stc_swait", STC_PRIO_HIGH, waiter_actor, SLOT_WAITER, other_cpu());
	TC_ASSERT_NEQ_CLEANUP("smp01_spawn_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("smp01_blocked", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("smp01_boosted_across_cpus", prio, STC_PRIO_HIGH, scenario_end());

	stc_go(SLOT_HOLDER);
	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("smp01_post", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("smp01_restored", prio, STC_PRIO_LOW, scenario_end());

	ret = stc_wait_stage(SLOT_WAITER, STAGE_ACQUIRED);
	TC_ASSERT_EQ_CLEANUP("smp01_acquired", ret, OK, scenario_end());

	stc_go(SLOT_WAITER);
	ret = stc_wait_count(&g_target, 1);
	TC_ASSERT_EQ_CLEANUP("smp01_drained", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("smp01_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("smp01_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_smp02_remote_post_restores
 *
 * Scenario: SCN-SMP-02
 *   The holder is on CPU0 and never posts; a task on CPU1 posts on its behalf.
 *
 * Oracle:
 *   (1) the waiter receives the count                              <- HARD
 *   (2) the holder ends at its base priority                       <- HARD
 *
 * What it exercises:
 *   The restore runs on the posting CPU, against a holder that is not the
 *   running task and is not even on the same CPU.  On a single core build this
 *   degenerates to the SCN-PIX-05 shape, which is why the count arithmetic is
 *   identical to it.
 *
 ****************************************************************************/

static void stc_sem_smp02_remote_post_restores(void)
{
	uint32_t violations;
	int ret;
	int prio;
	pid_t pid;

	ret = scenario_begin(1);
	TC_ASSERT_EQ("smp02_begin", ret, OK);

	pid = spawn_on_cpu("stc_snp", STC_PRIO_LOW, holder_no_post_actor, SLOT_HOLDER, 0);
	TC_ASSERT_NEQ_CLEANUP("smp02_spawn_holder", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("smp02_held", ret, OK, scenario_end());

	pid = spawn_on_cpu("stc_swait", STC_PRIO_HIGH, waiter_actor, SLOT_WAITER, other_cpu());
	TC_ASSERT_NEQ_CLEANUP("smp02_spawn_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_target, -1);
	TC_ASSERT_EQ_CLEANUP("smp02_blocked", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("smp02_boosted", prio, STC_PRIO_HIGH, scenario_end());

	pid = spawn_on_cpu("stc_spost", STC_PRIO_INVERTER, poster_actor, SLOT_POSTER, other_cpu());
	TC_ASSERT_NEQ_CLEANUP("smp02_spawn_poster", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_POSTER, STAGE_STARTED);
	TC_ASSERT_EQ_CLEANUP("smp02_poster_ready", ret, OK, scenario_end());

	stc_go(SLOT_POSTER);
	ret = stc_wait_count(&g_target, 0);
	TC_ASSERT_EQ_CLEANUP("smp02_remote_post", ret, OK, scenario_end());

	ret = stc_wait_stage(SLOT_WAITER, STAGE_ACQUIRED);
	TC_ASSERT_EQ_CLEANUP("smp02_waiter_got_count", ret, OK, scenario_end());

	prio = stc_getprio(g_stc_actor[SLOT_HOLDER].pid);
	TC_ASSERT_EQ_CLEANUP("smp02_holder_restored", prio, STC_PRIO_LOW, scenario_end());

	stc_go(SLOT_WAITER);
	ret = stc_wait_count(&g_target, 1);
	TC_ASSERT_EQ_CLEANUP("smp02_drained", ret, OK, scenario_end());

	stc_go(SLOT_HOLDER);
	ret = stc_wait_finished(SLOT_HOLDER);
	TC_ASSERT_EQ_CLEANUP("smp02_holder_done", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("smp02_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("smp02_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_smp04_sustained_contention
 *
 * Scenario: SCN-SMP-04
 *   Four tasks of four priorities contend on one semaphore, spread across the
 *   available CPUs, each looping lock, touch, unlock.
 *
 * Oracle:
 *   (1) every contender completes its whole loop - none starves     <- HARD
 *   (2) the total acquisition count matches the work issued         <- HARD
 *   (3) the count returns to its initial value                      <- HARD
 *   (4) the monitor reports no boost leak throughout                <- HARD
 *
 * Why oracle (4) carries the weight:
 *   Under real contention there is no instant at which an exact priority can
 *   be asserted from another task.  What can be asserted is that no contender
 *   is ever left above its base priority with nothing waiting, and that is
 *   precisely what the phase 2 monitor samples.  This scenario is the reason
 *   the monitor exists.
 *
 ****************************************************************************/

static void stc_sem_smp04_sustained_contention(void)
{
	static const int prio[4] = { STC_PRIO_LOW, STC_PRIO_LOW2, STC_PRIO_LOW3, STC_PRIO_MID };
	uint32_t violations;
	uint32_t expected;
	int ncpus;
	int ret;
	int i;

	ret = scenario_begin(1);
	TC_ASSERT_EQ("smp04_begin", ret, OK);

	ncpus = sched_getcpucount();

	for (i = 0; i < 4; i++) {
		if (spawn_on_cpu("stc_cont", prio[i], contender_actor, i, i % ((ncpus > 0) ? ncpus : 1))
			== (pid_t)ERROR) {
			TC_ASSERT_EQ_CLEANUP("smp04_spawn", ERROR, OK, scenario_end());
		}
	}

	for (i = 0; i < 4; i++) {
		ret = stc_wait_finished(i);
		TC_ASSERT_EQ_CLEANUP("smp04_contender_finished", ret, OK, scenario_end());
	}

	expected = (uint32_t)(4 * CONTENTION_CYCLES);
	TC_ASSERT_EQ_CLEANUP("smp04_all_work_done", g_acquisitions, expected, scenario_end());

	ret = stc_getcount(&g_target);
	TC_ASSERT_EQ_CLEANUP("smp04_count_restored", ret, 1, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("smp04_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("smp04_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_soak01
 *
 * Scenario: SCN-SOAK-01
 *   A mixed workload: repeated rounds of contention with the participants'
 *   priorities and CPU placement varied from round to round.
 *
 * Oracle:
 *   (1) no round loses work                                        <- HARD
 *   (2) the count returns to its initial value every round         <- HARD
 *   (3) zero monitor violations across the whole run               <- HARD
 *   (4) nothing asserts and no task is left blocked                <- HARD
 *
 * Note on duration:
 *   The catalogue specifies thirty minutes in a nightly job.  SOAK_CYCLES is
 *   set so the scenario is usable in an ordinary run; a nightly job should
 *   raise it rather than run this value and call it a soak.  The value is
 *   named here so that raising it is a one line change and so nobody mistakes
 *   the default for the intended coverage.
 *
 ****************************************************************************/

static void stc_sem_soak01(void)
{
	static const int prio[4] = { STC_PRIO_LOW, STC_PRIO_LOW2, STC_PRIO_LOW3, STC_PRIO_MID };
	uint32_t violations;
	int ncpus;
	int round;
	int ret;
	int i;

	ncpus = sched_getcpucount();
	if (ncpus <= 0) {
		ncpus = 1;
	}

	for (round = 0; round < (SOAK_CYCLES / CONTENTION_CYCLES) + 1; round++) {
		ret = scenario_begin(1);
		TC_ASSERT_EQ("soak01_begin", ret, OK);

		for (i = 0; i < 4; i++) {
			/* Rotate priorities and CPU placement between rounds so the same
			 * interleaving is not repeated.
			 */

			if (spawn_on_cpu("stc_soak", prio[(i + round) & 3], contender_actor, i,
							 (i + round) % ncpus) == (pid_t)ERROR) {
				TC_ASSERT_EQ_CLEANUP("soak01_spawn", ERROR, OK, scenario_end());
			}
		}

		for (i = 0; i < 4; i++) {
			ret = stc_wait_finished(i);
			TC_ASSERT_EQ_CLEANUP("soak01_finished", ret, OK, scenario_end());
		}

		TC_ASSERT_EQ_CLEANUP("soak01_work_done", g_acquisitions,
							 (uint32_t)(4 * CONTENTION_CYCLES), scenario_end());

		ret = stc_getcount(&g_target);
		TC_ASSERT_EQ_CLEANUP("soak01_count_restored", ret, 1, scenario_end());

		violations = stc_mon_violations();
		TC_ASSERT_EQ_CLEANUP("soak01_no_boost_leak", violations, 0, scenario_end());

		ret = scenario_end();
		TC_ASSERT_EQ("soak01_no_leaked_actor", ret, 0);
	}

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stc_sem_smp_main(void)
{
	if (stc_harness_begin() != OK) {
		printf("\n[stc_sem_smp] FAIL : cannot raise the harness priority\n");
		total_fail++;
		return ERROR;
	}

	if (stc_mon_start() != OK) {
		printf("\n[stc_sem_smp] FAIL : cannot start the boost-leak monitor\n");
		total_fail++;
		stc_harness_end();
		return ERROR;
	}

	if (sched_getcpucount() <= 1) {
		printf("\n[stc_sem_smp] NOTE : single CPU build, the cross CPU scenarios "
			   "degrade to the single core case\n");
	}

	stc_sem_smp01_cross_cpu_boost();
	stc_sem_smp02_remote_post_restores();
	stc_sem_smp04_sustained_contention();
	stc_sem_soak01();

	printf("[stc_sem_smp] NOTE: SCN-SMP-03, the interrupt context restore path, "
		   "needs a driver side post and is not covered by this tier\n");

	stc_mon_stop();
	stc_harness_end();

	return OK;
}

#else							/* CONFIG_PRIORITY_INHERITANCE */

int stc_sem_smp_main(void)
{
	printf("\n[stc_sem_smp] SKIP : CONFIG_PRIORITY_INHERITANCE is not enabled\n");
	return OK;
}

#endif							/* CONFIG_PRIORITY_INHERITANCE */
