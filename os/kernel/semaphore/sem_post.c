/****************************************************************************
 *
 * Copyright 2016-2017 Samsung Electronics All Rights Reserved.
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
 * kernel/semaphore/sem_post.c
 *
 *   Copyright (C) 2007-2009, 2012-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <limits.h>
#include <semaphore.h>
#include <errno.h>
#include <sched.h>
#include <tinyara/arch.h>
#include <tinyara/sched.h>
#include <tinyara/mm/mm.h>
#include <tinyara/spinlock.h>

#include "sched/sched.h"
#include "semaphore/semaphore.h"

#ifdef CONFIG_SEMAPHORE_HISTORY
#include <tinyara/debug/sysdbg.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

#ifdef CONFIG_SMP
spinlock_t g_sem_smp_lock = SP_UNLOCKED;
#endif

/* Test variable: when set to 1, sem_post() skips the waiter
 * scan and forces stcb=NULL. With semcount <= 0, this hits the
 * DEBUGASSERT at sem_holder.c:926. */
volatile int g_test_force_null_stcb = 0;

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Phase 1: Inside spinlock. Scans waiter, changes task_state. No sched_unlock. */
struct tcb_s *sem_unblock_task_locked(sem_t *sem, struct tcb_s *htcb)
{
	struct tcb_s *stcb = NULL;

#ifdef CONFIG_SEMAPHORE_HISTORY
	save_semaphore_history(sem, (void *)this_task(), SEM_RELEASE);
#endif
#ifdef CONFIG_PRIORITY_INHERITANCE
	/* Don't let any unblocked tasks run until we complete any priority
	 * restoration steps.  Interrupts are disabled, but we do not want
	 * the head of the read-to-run list to be modified yet.
	 *
	 * NOTE: If this sched_lock is called from an interrupt handler, it
	 * will do nothing.
	 */

	sched_lock();
#endif
	/* If the result of of semaphore unlock is non-positive, then
	 * there must be some task waiting for the semaphore.
	 */

	if (sem->semcount <= 0) {
		/* Check if there are any tasks in the waiting for semaphore
		 * task list that are waiting for this semaphore. This is a
		 * prioritized list so the first one we encounter is the one
		 * that we want.
		 */

		for (stcb = (FAR struct tcb_s *)g_waitingforsemaphore.head; (stcb && stcb->waitsem != sem); stcb = stcb->flink) ;

		if (stcb) {
			sem_addholder_tcb(stcb, sem);

			/* It is, let the task take the semaphore */

			stcb->waitsem = NULL;

#ifdef CONFIG_SEMAPHORE_HISTORY
			save_semaphore_history(sem, (void *)stcb, SEM_ACQUIRE);
#endif
			/* Restart the waiting task. */

			up_unblock_task(stcb);
		}
	}

	return stcb;
}

/* Phase 2: Outside spinlock. Priority restore and sched_unlock. Safe to switch. */
void sem_unblock_task_finish(sem_t *sem, struct tcb_s *htcb, struct tcb_s *stcb)
{
#ifdef SAVE_SEM_HOLDER
	struct semholder_s *pholder = NULL;

	/* Check if we need to drop the priority of any threads holding
	 * this semaphore.  The priority could have been boosted while they
	 * held the semaphore.
	 */

#ifdef CONFIG_PRIORITY_INHERITANCE
	if ((sem->flags & PRIOINHERIT_FLAGS_DISABLE) == 0) {
		sem_restorebaseprio(stcb, htcb, sem);
	} else {
#endif
		/* Free a semaphore holder directly. */
		pholder = sem_findholder(sem, htcb);
		if (pholder) {
			sem_freeholder(sem, pholder);
		}
#ifdef CONFIG_PRIORITY_INHERITANCE
	}

	sched_unlock();
#endif
#endif /* SAVE_SEM_HOLDER */
}

/* Wrapper: both phases sequentially. For non-spinlock callers. */
void sem_unblock_task(sem_t *sem, struct tcb_s *htcb)
{
	struct tcb_s *stcb = sem_unblock_task_locked(sem, htcb);

	sem_unblock_task_finish(sem, htcb, stcb);
}

/****************************************************************************
 * Name: sem_post
 *
 * Description:
 *   When a task has finished with a semaphore, it will call sem_post().
 *   This function unlocks the semaphore referenced by sem by performing the
 *   semaphore unlock operation on that semaphore.
 *
 *   If the semaphore value resulting from this operation is positive, then
 *   no tasks were blocked waiting for the semaphore to become unlocked; the
 *   semaphore is simply incremented.
 *
 *   If the value of the semaphore resulting from this operation is zero,
 *   then one of the tasks blocked waiting for the semaphore shall be
 *   allowed to return successfully from its call to sem_wait().
 *
 * Parameters:
 *   sem - Semaphore descriptor
 *
 * Return Value:
 *   0 (OK) or -1 (ERROR) if unsuccessful
 *
 * Assumptions:
 *   This function may be called from an interrupt handler.
 *
 ****************************************************************************/

int sem_post(FAR sem_t *sem)
{
	FAR struct tcb_s *htcb;
	irqstate_t saved_state;
	int ret = ERROR;
	size_t caller_retaddr = (size_t)GET_RETURN_ADDRESS();

	/* Make sure we were supplied with a valid semaphore. */
	saved_state = enter_critical_section();

	if (sem && ((sem->flags & FLAGS_INITIALIZED) != 0)) {
		/* The following operations must be performed with interrupts
		 * disabled because sem_post() may be called from an interrupt
		 * handler.
		 */

		/* Perform the semaphore unlock operation. */
		ASSERT_INFO(sem->semcount < SEM_VALUE_MAX, "sem = 0x%x, semcount = %d, flags = 0x%x, caller address = 0x%x\n", sem, sem->semcount, sem->flags, caller_retaddr);
		htcb = this_task();
		htcb = sem_releaseholder(sem, htcb);
		sem->semcount++;

		if ((sem->flags & FLAGS_SEM_MUTEX) != 0) {
			ASSERT_INFO(sem->semcount < 2, "sem = 0x%x, semcount = %d, flags = 0x%x, caller address = 0x%x\n", sem, sem->semcount, sem->flags, caller_retaddr);
		}

#ifdef CONFIG_SMP
		{
			irqstate_t spin_flags;
			if ((sem->flags & FLAGS_SIGSEM) == 0) {
				/* Prevent SMP race with sem_waitirq() timeout on other CPU. */
				spin_flags = spin_lock_irqsave(&g_sem_smp_lock);
			}

#endif

#ifdef CONFIG_TC_SEM_IRQ_RACE
			if (g_test_force_null_stcb) {
				/* Force stcb=NULL to simulate SMP race: waiter was removed
				 * from g_waitingforsemaphore by another CPU's timeout before
				 * we could scan the list. With semcount <= 0 and stcb == NULL,
				 * the assert at sem_holder.c:926 will fire. */
				lldbg("FORCE_NULL: semcount=%d, skipping scan → stcb=NULL → ASSERT!\n",
				      sem->semcount);
				sem_restorebaseprio(NULL, htcb, sem);
			} else
#endif
			{
				/* Phase 1: inside spinlock. Changes task_state, fixes SMP race. */
				struct tcb_s *stcb = sem_unblock_task_locked(sem, htcb);

#ifdef CONFIG_SMP
				if ((sem->flags & FLAGS_SIGSEM) == 0) {
					spin_unlock_irqrestore(&g_sem_smp_lock, spin_flags);
				}
#endif

				/* Phase 2: outside spinlock. Priority restore and sched_unlock. */
				sem_unblock_task_finish(sem, htcb, stcb);
			}

#ifdef CONFIG_SMP
		}
#endif

		ret = OK;

		/* Interrupts may now be enabled. */

	} else {
		set_errno(EINVAL);
	}

	leave_critical_section(saved_state);

	return ret;
}
