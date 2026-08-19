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

/// @file stc_sem_destroy.c
/// @brief Families G and H of the Priority Inheritance and Semaphore Recovery
///        Scenario Test Catalogue.
///
/// Family G - destruction, reset and list integrity
///   SCN-DES-01  destroy a semaphore that still has live holders
///   SCN-DES-02  destroy a semaphore that has a blocked waiter
///   SCN-DES-03  sem_reset() count arithmetic          (driver side)
///   SCN-DES-04  sem_reset() argument validation       (driver side)
///   SCN-DES-05  repeated init and destroy cycles
///
/// Family H - binary manager fault recovery
///   SCN-BIN-01 to SCN-BIN-03 are NOT implemented here, and cannot be: they
///   require faulting a loadable user binary, which needs the binary manager
///   test fixture rather than a task inside the kernel test app.  Writing
///   something that merely looked like fault injection would be worse than
///   leaving the gap visible, so what ships here is SCN-BIN-04 - the scope
///   limit that is testable in process - plus an explicit notice naming what
///   is missing.
///
///   The manual procedure for SCN-BIN-01 to SCN-BIN-03, for whoever builds the
///   fixture:
///     1. Build with CONFIG_BINMGR_RECOVERY=y and at least one loadable user
///        binary that takes a kernel semaphore and then faults on demand.
///     2. Have a kernel task block on the same semaphore.
///     3. Trigger the fault.  binary_manager_deactivate_binary() runs
///        binary_manager_recover_tcb() per thread, then
///        binary_manager_release_binary_sem() walks g_sem_list.
///     4. Assert that the kernel task resumes and owns the semaphore, that a
///        second binary's semaphore is untouched, and that the mutex
///        invariant semcount < 2 held throughout.

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sched.h>
#include <semaphore.h>
#include <sys/types.h>
#include <tinyara/os_api_test_drv.h>

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
#define STAGE_SECOND_POSTED     3

#define SLOT_HOLDER             0
#define SLOT_HOLDER2            1
#define SLOT_WAITER             2

#define DESTROY_CYCLES          100

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_doomed;			/* the semaphore that gets destroyed      */
static sem_t g_survivor;		/* held by the same tasks, must stay sane */
static bool g_survivor_valid;

/****************************************************************************
 * Private Functions - actors
 ****************************************************************************/

/****************************************************************************
 * Name: dual_holder_actor
 *
 * Description:
 *   Takes a count of both semaphores, then - after the harness has destroyed
 *   the first one - posts only the survivor.  Posting the destroyed semaphore
 *   would be undefined behaviour and is not what the scenario is checking:
 *   the question is whether the destroy left this task's holdsem list in a
 *   state that makes the *survivor's* post fault.
 *
 ****************************************************************************/

static int dual_holder_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	if (sem_wait(&g_doomed) != OK || sem_wait(&g_survivor) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_HELD);
	stc_wait_go(slot);

	(void)sem_post(&g_survivor);
	stc_stage(slot, STAGE_SECOND_POSTED);

	stc_actor_done(slot);
	return OK;
}

static int doomed_waiter_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	stc_stage(slot, STAGE_STARTED);

	if (sem_wait(&g_doomed) == OK) {
		(void)sem_post(&g_doomed);
	}

	stc_actor_done(slot);
	return OK;
}

static int simple_holder_actor(int argc, char *argv[])
{
	int slot = atoi(argv[1]);

	(void)argc;

	if (sem_wait(&g_doomed) != OK) {
		stc_actor_done(slot);
		return ERROR;
	}

	stc_stage(slot, STAGE_HELD);
	stc_wait_go(slot);

	(void)sem_post(&g_doomed);

	stc_actor_done(slot);
	return OK;
}

/****************************************************************************
 * Private Functions - helpers
 ****************************************************************************/

static int scenario_begin(int doomed_count, bool with_survivor)
{
	if (stc_reset() != OK) {
		return ERROR;
	}

	if (sem_init(&g_doomed, 0, doomed_count) != OK) {
		return ERROR;
	}

	stc_mon_register_sem(&g_doomed);

	if (with_survivor) {
		if (sem_init(&g_survivor, 0, 2) != OK) {
			return ERROR;
		}

		g_survivor_valid = true;
		stc_mon_register_sem(&g_survivor);
	}

	return OK;
}

static int scenario_end(void)
{
	int leaked = stc_teardown();

	(void)sem_destroy(&g_doomed);

	if (g_survivor_valid) {
		(void)sem_destroy(&g_survivor);
		g_survivor_valid = false;
	}

	return leaked;
}

/****************************************************************************
 * Private Functions - scenarios
 ****************************************************************************/

/****************************************************************************
 * Name: stc_sem_des01_destroy_with_live_holders
 *
 * Scenario: SCN-DES-01
 *   A semaphore is destroyed while two live tasks hold counts on it.  Both
 *   tasks also hold a count of a second semaphore.
 *
 * Oracle:
 *   both tasks can still post the second semaphore without faulting <- HARD
 *
 * Defect signature:
 *   sem_destroyholder() has to unlink each holder record from two lists: the
 *   semaphore's own, and the holder task's holdsem list.  A free that skipped
 *   the second leaves a dangling entry whose sem pointer refers to a destroyed
 *   semaphore - and the very next priority restore walks holdsem and
 *   dereferences it.  Posting the survivor is what triggers that walk.
 *
 ****************************************************************************/

static void stc_sem_des01_destroy_with_live_holders(void)
{
	uint32_t violations;
	int ret;
	pid_t pid;

	ret = scenario_begin(2, true);
	TC_ASSERT_EQ("des01_begin", ret, OK);

	pid = stc_spawn("stc_da", STC_PRIO_LOW, dual_holder_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("des01_spawn_h1", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("des01_h1_held", ret, OK, scenario_end());

	pid = stc_spawn("stc_db", STC_PRIO_LOW2, dual_holder_actor, SLOT_HOLDER2);
	TC_ASSERT_NEQ_CLEANUP("des01_spawn_h2", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER2, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("des01_h2_held", ret, OK, scenario_end());

	/* Destroy out from under both of them. */

	ret = sem_destroy(&g_doomed);
	TC_ASSERT_EQ_CLEANUP("des01_destroy", ret, OK, scenario_end());

	/* The survivor's post walks each task's holdsem list during the priority
	 * restore.  A stale entry pointing at the destroyed semaphore faults here.
	 */

	stc_go(SLOT_HOLDER);
	ret = stc_wait_stage(SLOT_HOLDER, STAGE_SECOND_POSTED);
	TC_ASSERT_EQ_CLEANUP("des01_h1_survivor_post", ret, OK, scenario_end());

	stc_go(SLOT_HOLDER2);
	ret = stc_wait_stage(SLOT_HOLDER2, STAGE_SECOND_POSTED);
	TC_ASSERT_EQ_CLEANUP("des01_h2_survivor_post", ret, OK, scenario_end());

	ret = stc_wait_count(&g_survivor, 2);
	TC_ASSERT_EQ_CLEANUP("des01_survivor_drained", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("des01_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("des01_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_des02_destroy_with_blocked_waiter
 *
 * Scenario: SCN-DES-02
 *   A semaphore is destroyed while a task is blocked on it.
 *
 * Oracle:
 *   (1) nothing asserts and nothing faults                         <- HARD
 *   (2) the blocked task can still be deleted cleanly              <- HARD
 *
 * Note:
 *   POSIX leaves this undefined, and sem_destroy() says so: it leaves a
 *   negative count untouched and returns OK.  The scenario pins down that the
 *   undefined case is survivable rather than fatal, which is what a caller
 *   doing error path cleanup actually needs to know.  It does not assert any
 *   particular count, because there is no defined answer to assert.
 *
 ****************************************************************************/

static void stc_sem_des02_destroy_with_blocked_waiter(void)
{
	uint32_t violations;
	int ret;
	pid_t pid;

	ret = scenario_begin(1, false);
	TC_ASSERT_EQ("des02_begin", ret, OK);

	pid = stc_spawn("stc_hold", STC_PRIO_LOW, simple_holder_actor, SLOT_HOLDER);
	TC_ASSERT_NEQ_CLEANUP("des02_spawn_holder", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_stage(SLOT_HOLDER, STAGE_HELD);
	TC_ASSERT_EQ_CLEANUP("des02_held", ret, OK, scenario_end());

	pid = stc_spawn("stc_doom", STC_PRIO_HIGH, doomed_waiter_actor, SLOT_WAITER);
	TC_ASSERT_NEQ_CLEANUP("des02_spawn_waiter", pid, (pid_t)ERROR, scenario_end());

	ret = stc_wait_count(&g_doomed, -1);
	TC_ASSERT_EQ_CLEANUP("des02_blocked", ret, OK, scenario_end());

	ret = sem_destroy(&g_doomed);
	TC_ASSERT_EQ_CLEANUP("des02_destroy_returns_ok", ret, OK, scenario_end());

	/* The blocked task is still blocked, on a semaphore that no longer exists.
	 * Deleting it must be clean: sem_recover() runs against the same memory.
	 */

	ret = task_delete(g_stc_actor[SLOT_WAITER].pid);
	TC_ASSERT_EQ_CLEANUP("des02_delete_waiter", ret, OK, scenario_end());

	stc_actor_forget(SLOT_WAITER);

	/* And the holder can still be released. */

	stc_go(SLOT_HOLDER);
	ret = stc_wait_finished(SLOT_HOLDER);
	TC_ASSERT_EQ_CLEANUP("des02_holder_done", ret, OK, scenario_end());

	violations = stc_mon_violations();
	TC_ASSERT_EQ_CLEANUP("des02_no_boost_leak", violations, 0, scenario_end());

	ret = scenario_end();
	TC_ASSERT_EQ("des02_no_leaked_actor", ret, 0);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_des0304_reset
 *
 * Scenario: SCN-DES-03 and SCN-DES-04
 *   sem_reset() count arithmetic and argument validation.
 *
 * Oracle:
 *   the driver side sequence returns OK                            <- HARD
 *
 * Note:
 *   sem_reset() is declared in tinyara/semaphore.h and is not exported
 *   through the syscall table, so it cannot be called from a user space
 *   scenario at all.  The sequence runs inside the OS API test driver, in the
 *   same style as the existing tc_semaphore_sem_tickwait() test, and this
 *   scenario is the user space trigger for it.
 *
 ****************************************************************************/

static void stc_sem_des0304_reset(void)
{
	int fd = tc_get_drvfd();
	int ret;

	TC_ASSERT_GEQ("des0304_drvfd", fd, 0);

	ret = ioctl(fd, TESTIOC_SEM_RESET_TEST, 0);
	TC_ASSERT_EQ("des0304_reset_sequence", ret, OK);

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_des05_init_destroy_churn
 *
 * Scenario: SCN-DES-05
 *   Repeated init and destroy cycles of both flavours.
 *
 * Oracle:
 *   (1) every cycle leaves no holder record behind                 <- SOFT
 *   (2) a signalling semaphore destroys cleanly                    <- HARD
 *
 * Note on the registration rule:
 *   sem_init() registers a semaphore into g_sem_list only when its initial
 *   count is non zero and it lives in the kernel region, so a signalling
 *   semaphore is never registered - and sem_destroy() correspondingly skips
 *   the unregister for FLAGS_SIGSEM.  The list itself is kernel state with no
 *   accessor, so this scenario cannot inspect it directly; what it can do is
 *   churn both flavours hard enough that a mismatched register and unregister
 *   pair would corrupt the list and take the next traversal with it.
 *
 ****************************************************************************/

static void stc_sem_des05_init_destroy_churn(void)
{
	struct sem_snapshot_s snap;
	sem_t counting;
	sem_t signalling;
	int ret;
	int i;

	for (i = 0; i < DESTROY_CYCLES; i++) {
		ret = sem_init(&counting, 0, 1);
		TC_ASSERT_EQ("des05_init_counting", ret, OK);

		ret = sem_init(&signalling, 0, 0);
		TC_ASSERT_EQ("des05_init_signalling", ret, OK);

		if (stc_snapshot(&signalling, &snap) == OK) {
			TC_ASSERT_EQ_CLEANUP("des05_sigsem_flag", (snap.flags & FLAGS_SIGSEM) != 0, true,
								 (void)sem_destroy(&counting); (void)sem_destroy(&signalling));
			TC_ASSERT_EQ_CLEANUP("des05_sigsem_no_holder", snap.nholders, 0,
								 (void)sem_destroy(&counting); (void)sem_destroy(&signalling));
		}

		ret = sem_wait(&counting);
		TC_ASSERT_EQ_CLEANUP("des05_take", ret, OK,
							 (void)sem_destroy(&counting); (void)sem_destroy(&signalling));

		ret = sem_post(&counting);
		TC_ASSERT_EQ_CLEANUP("des05_give", ret, OK,
							 (void)sem_destroy(&counting); (void)sem_destroy(&signalling));

		if (stc_snapshot(&counting, &snap) == OK) {
			TC_ASSERT_EQ_CLEANUP("des05_record_freed", snap.nholders, 0,
								 (void)sem_destroy(&counting); (void)sem_destroy(&signalling));
		}

		ret = sem_destroy(&counting);
		TC_ASSERT_EQ_CLEANUP("des05_destroy_counting", ret, OK, (void)sem_destroy(&signalling));

		ret = sem_destroy(&signalling);
		TC_ASSERT_EQ("des05_destroy_signalling", ret, OK);
	}

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Name: stc_sem_bin04_scope_limit
 *
 * Scenario: SCN-BIN-04, the testable part of family H
 *   A signalling semaphore is never registered for binary manager recovery,
 *   because sem_init() registers only semaphores whose initial count is non
 *   zero.
 *
 * Oracle:
 *   a signalling semaphore carries FLAGS_SIGSEM and holds no records <- SOFT
 *
 * Note:
 *   SCN-BIN-01 to SCN-BIN-03 need a loadable user binary that can be faulted
 *   on demand.  That is fixture work, not test-case work, and it is described
 *   in the header of this file.  This scenario covers the one part of family H
 *   that is observable from inside the kernel test app, and prints a notice
 *   naming what is still missing so the gap is visible in the run log rather
 *   than only in the catalogue.
 *
 ****************************************************************************/

static void stc_sem_bin04_scope_limit(void)
{
	struct sem_snapshot_s snap;
	sem_t signalling;
	int ret;

	ret = sem_init(&signalling, 0, 0);
	TC_ASSERT_EQ("bin04_init", ret, OK);

	if (stc_snapshot(&signalling, &snap) == OK) {
		TC_ASSERT_EQ_CLEANUP("bin04_sigsem", (snap.flags & FLAGS_SIGSEM) != 0, true,
							 (void)sem_destroy(&signalling));
		TC_ASSERT_EQ_CLEANUP("bin04_no_holders", snap.nholders, 0, (void)sem_destroy(&signalling));
	}

	ret = sem_destroy(&signalling);
	TC_ASSERT_EQ("bin04_destroy", ret, OK);

	printf("[bin04] NOTE: SCN-BIN-01..03 require the binary manager fault "
		   "fixture and are not covered by this tier\n");

	TC_SUCCESS_RESULT();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stc_sem_destroy_main(void)
{
	if (stc_harness_begin() != OK) {
		printf("\n[stc_sem_destroy] FAIL : cannot raise the harness priority\n");
		total_fail++;
		return ERROR;
	}

	if (stc_mon_start() != OK) {
		printf("\n[stc_sem_destroy] FAIL : cannot start the boost-leak monitor\n");
		total_fail++;
		stc_harness_end();
		return ERROR;
	}

	stc_sem_des01_destroy_with_live_holders();
	stc_sem_des02_destroy_with_blocked_waiter();
	stc_sem_des0304_reset();
	stc_sem_des05_init_destroy_churn();
	stc_sem_bin04_scope_limit();

	stc_mon_stop();
	stc_harness_end();

	return OK;
}

#else							/* CONFIG_PRIORITY_INHERITANCE */

int stc_sem_destroy_main(void)
{
	printf("\n[stc_sem_destroy] SKIP : CONFIG_PRIORITY_INHERITANCE is not enabled\n");
	return OK;
}

#endif							/* CONFIG_PRIORITY_INHERITANCE */
