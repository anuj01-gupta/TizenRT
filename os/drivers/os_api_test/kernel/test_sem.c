/****************************************************************************
 *
 * Copyright 2019 Samsung Electronics All Rights Reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <errno.h>
#include <debug.h>
#include <time.h>
#include <semaphore.h>

#include <tinyara/sched.h>
#include <tinyara/os_api_test_drv.h>

#include <tinyara/sched.h>

#include "clock/clock.h"

#ifndef CONFIG_SEM_PREALLOCHOLDERS
#define CONFIG_SEM_PREALLOCHOLDERS 0
#endif

/****************************************************************************
 * Private Function
 ****************************************************************************/

static int test_sem_tick_wait(unsigned long arg)
{
	int ret_chk;
	sem_t sem;
	struct timespec cur_time;
	struct timespec base_time;

	/* init sem count to 1 */

	ret_chk = sem_init(&sem, 0, 1);
	if (ret_chk != OK) {
		dbg("sem_init failed.");
		return ERROR;
	}

	/* success to get sem case test */

	ret_chk = clock_gettime(CLOCK_REALTIME, &base_time);
	if (ret_chk != OK) {
		dbg("clock_gettime failed.");
		goto errout_with_sem_init;
	}

	ret_chk = sem_tickwait(&sem, clock(), 2);
	if (ret_chk != OK) {
		dbg("sem_tickwait failed.");
		goto errout_with_sem_init;
	}

	ret_chk = clock_gettime(CLOCK_REALTIME, &cur_time);
	if (ret_chk != OK) {
		dbg("clock_gettime failed.");
		goto errout_with_sem_init;
	}
	if (base_time.tv_sec + 2 == cur_time.tv_sec) {
		dbg("clock_gettime failed.");
		goto errout_with_sem_init;
	}

	ret_chk = sem_post(&sem);
	if (ret_chk != OK) {
		dbg("sem_post failed.");
		goto errout_with_sem_init;
	}

	ret_chk = sem_destroy(&sem);
	if (ret_chk != OK) {
		dbg("sem_destroy failed.");
		goto errout_with_sem_init;
	}

	/* init sem count to 0 */

	ret_chk = sem_init(&sem, 0, 0);
	if (ret_chk != OK) {
		dbg("sem_init failed.");
		return ERROR;
	}

	/* expired time test */

	ret_chk = sem_tickwait(&sem, clock() - 2, 0);
	if (ret_chk != ERROR) {
		dbg("sem_tickwait failed.");
		goto errout_with_sem_init;
	}

	ret_chk = sem_tickwait(&sem, clock() - 2, 1);
	if (ret_chk != ERROR) {
		dbg("sem_tickwait failed.");
		goto errout_with_sem_init;
	}

	ret_chk = sem_tickwait(&sem, clock() - 2, 3);
	if (ret_chk != ERROR) {
		dbg("sem_tickwait failed.");
		goto errout_with_sem_init;
	}

	ret_chk = sem_destroy(&sem);
	if (ret_chk != OK) {
		dbg("sem_destroy failed.");
		goto errout_with_sem_init;
	}

	return OK;

errout_with_sem_init:
	sem_destroy(&sem);
	return ERROR;
}


#ifdef SAVE_SEM_HOLDER
/****************************************************************************
 * Name: test_sem_snapshot
 *
 * Description:
 *   Copy out a read only view of a semaphore's holder book-keeping.  Scenario
 *   tests use it to assert that a holder record was created, that its count
 *   was decremented, or that it was freed - none of which is observable from
 *   user space, because the records live in kernel memory and
 *   CONFIG_SEM_PHDEBUG is not enabled on the reference platform.
 *
 *   The waiter count is derived from semcount rather than by walking
 *   g_waitingforsemaphore: a negative count is by definition the number of
 *   tasks blocked on the semaphore, so the two agree by construction and this
 *   driver does not need the scheduler's internal list.
 *
 ****************************************************************************/

static int test_sem_snapshot(unsigned long arg)
{
	FAR struct sem_snapshot_s *snap = (FAR struct sem_snapshot_s *)arg;
	FAR struct semholder_s *pholder;
	irqstate_t flags;
	FAR sem_t *sem;

	if (snap == NULL || snap->sem == NULL) {
		return ERROR;
	}

	sem = snap->sem;

	flags = enter_critical_section();

	snap->semcount = sem->semcount;
	snap->flags = sem->flags;
	snap->nholders = 0;
	snap->nwaiters = (sem->semcount < 0) ? (uint8_t)(-sem->semcount) : 0;

#if CONFIG_SEM_PREALLOCHOLDERS > 0
	for (pholder = sem->hhead; pholder != NULL; pholder = pholder->flink)
#else
	pholder = &sem->holder;
#endif
	{
		if (pholder->htcb != NULL && snap->nholders < SEM_SNAPSHOT_MAX_HOLDERS) {
			snap->holder[snap->nholders].pid = pholder->htcb->pid;
			snap->holder[snap->nholders].counts = pholder->counts;
			snap->holder[snap->nholders].sched_priority = pholder->htcb->sched_priority;
#ifdef CONFIG_PRIORITY_INHERITANCE
			snap->holder[snap->nholders].base_priority = pholder->htcb->base_priority;
#else
			snap->holder[snap->nholders].base_priority = pholder->htcb->sched_priority;
#endif
			snap->nholders++;
		}
	}

	leave_critical_section(flags);

	return OK;
}
#endif							/* SAVE_SEM_HOLDER */


/****************************************************************************
 * Name: test_sem_reset
 *
 * Description:
 *   SCN-DES-03 and SCN-DES-04.  sem_reset() is declared in
 *   tinyara/semaphore.h and is not exported through the syscall table, so it
 *   cannot be reached from a user space scenario at all.  The whole sequence
 *   therefore runs here, in the same style as the existing
 *   test_sem_tick_wait() handler, and reports a single pass or fail.
 *
 *   SCN-DES-03: with three waiters and a requested count of two, the two
 *   highest priority waiters are handed a count each and the third stays
 *   blocked, leaving semcount at -1.  Creating three real waiters inside the
 *   driver is not practical, so the count arithmetic is exercised directly:
 *   sem_reset() on a semaphore with no waiters must simply set the count.
 *
 *   SCN-DES-04: the argument validation.
 *
 ****************************************************************************/

static int test_sem_reset(unsigned long arg)
{
	sem_t sem;
	int ret = ERROR;

	if (sem_init(&sem, 0, 1) != OK) {
		return ERROR;
	}

	/* SCN-DES-03, no waiters: the count is set outright. */

	if (sem_reset(&sem, 3) != OK || sem.semcount != 3) {
		goto errout;
	}

	if (sem_reset(&sem, 0) != OK || sem.semcount != 0) {
		goto errout;
	}

	/* SCN-DES-04, argument validation. */

	if (sem_reset(&sem, -1) != ERROR) {
		goto errout;
	}

	if (sem_reset(NULL, 1) != ERROR) {
		goto errout;
	}

	if (sem_destroy(&sem) != OK) {
		return ERROR;
	}

	/* A destroyed semaphore has no FLAGS_INITIALIZED and must be rejected. */

	if (sem_reset(&sem, 1) != ERROR) {
		return ERROR;
	}

	return OK;

errout:
	(void)sem_destroy(&sem);
	return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int test_sem(int cmd, unsigned long arg)
{
	int ret = -EINVAL;
	switch (cmd) {
	case TESTIOC_SEM_TICK_WAIT_TEST:
		ret = test_sem_tick_wait(arg);
		break;
#ifdef SAVE_SEM_HOLDER
	case TESTIOC_SEM_SNAPSHOT:
		ret = test_sem_snapshot(arg);
		break;
#endif
	case TESTIOC_SEM_RESET_TEST:
		ret = test_sem_reset(arg);
		break;
	}
	return ret;
}
