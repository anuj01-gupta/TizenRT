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

/// @file stc_sem_common.c
/// @brief Shared harness for the semaphore scenario test tier.

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sched.h>
#include <semaphore.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <tinyara/os_api_test_drv.h>

#include "tc_internal.h"
#include "stc_sem_common.h"
#include "stc_sem_monitor.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Saved harness task attributes, restored by stc_harness_end(). */

static int g_stc_saved_prio;
static bool g_stc_harness_active;
#ifdef CONFIG_SMP
static cpu_set_t g_stc_saved_affinity;
static bool g_stc_saved_affinity_valid;
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct stc_actor_s g_stc_actor[STC_MAX_ACTORS];
volatile uint16_t g_stc_trace_idx;
char g_stc_trace[STC_TRACE_MAX];

volatile bool g_stc_spin_stop;
volatile uint32_t g_stc_spin_count;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stc_finished_within
 *
 * Description:
 *   Bounded wait for one actor to reach its exit point, used by the teardown
 *   grace period.
 *
 ****************************************************************************/

static bool stc_finished_within(int slot, int timeout_ms)
{
	int retries = (timeout_ms * 1000) / STC_POLL_US;

	while (retries-- > 0) {
		if (g_stc_actor[slot].finished) {
			return true;
		}

		usleep(STC_POLL_US);
	}

	return g_stc_actor[slot].finished;
}

/****************************************************************************
 * Name: stc_pin
 *
 * Description:
 *   Pin a task to CPU0 on SMP builds.  Catalogue rule 9: a scenario whose
 *   oracle is an exact priority at an exact moment needs its actors on one
 *   CPU, otherwise holder and waiter progress concurrently and the boost may
 *   legitimately be observed mid-flight.
 *
 ****************************************************************************/

static void stc_pin(pid_t pid)
{
#ifdef CONFIG_SMP
	cpu_set_t mask = (cpu_set_t)(1 << 0);

	(void)sched_setaffinity(pid, sizeof(cpu_set_t), &mask);
#else
	(void)pid;
#endif
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stc_trace(char ev)
{
	uint16_t idx = g_stc_trace_idx++;

	if (idx < STC_TRACE_MAX) {
		g_stc_trace[idx] = ev;
	}
}

/****************************************************************************
 * Name: stc_harness_begin
 ****************************************************************************/

int stc_harness_begin(void)
{
	struct sched_param param;
	pid_t self = getpid();

	if (g_stc_harness_active) {
		return OK;
	}

	if (sched_getparam(self, &param) != OK) {
		return ERROR;
	}

	g_stc_saved_prio = param.sched_priority;

#ifdef CONFIG_SMP
	g_stc_saved_affinity_valid =
		(sched_getaffinity(self, sizeof(cpu_set_t), &g_stc_saved_affinity) == OK);
#endif

	param.sched_priority = STC_PRIO_HARNESS;
	if (sched_setparam(self, &param) != OK) {
		return ERROR;
	}

	stc_pin(self);
	g_stc_harness_active = true;

	return OK;
}

/****************************************************************************
 * Name: stc_harness_end
 ****************************************************************************/

void stc_harness_end(void)
{
	struct sched_param param;
	pid_t self = getpid();

	if (!g_stc_harness_active) {
		return;
	}

#ifdef CONFIG_SMP
	if (g_stc_saved_affinity_valid) {
		(void)sched_setaffinity(self, sizeof(cpu_set_t), &g_stc_saved_affinity);
		g_stc_saved_affinity_valid = false;
	}
#endif

	param.sched_priority = g_stc_saved_prio;
	(void)sched_setparam(self, &param);

	g_stc_harness_active = false;
}

/****************************************************************************
 * Name: stc_reset
 ****************************************************************************/

int stc_reset(void)
{
	int i;

	memset(g_stc_actor, 0, sizeof(g_stc_actor));
	memset(g_stc_trace, 0, sizeof(g_stc_trace));
	g_stc_trace_idx = 0;
	g_stc_spin_stop = false;
	g_stc_spin_count = 0;

	/* Every scenario starts with an empty monitor registry and a zero
	 * violation count, so a violation is always attributable to the scenario
	 * that reports it.
	 */

	stc_mon_clear();

	/* Initial value 0 makes each of these a signalling semaphore, so the
	 * handshake machinery records no holders and cannot boost anybody.
	 */

	for (i = 0; i < STC_MAX_ACTORS; i++) {
		if (sem_init(&g_stc_actor[i].go, 0, 0) != OK) {
			return ERROR;
		}
	}

	return OK;
}

/****************************************************************************
 * Name: stc_teardown
 *
 * Description:
 *   Catalogue rule 7.  Delete anything still alive, release the handshake
 *   semaphores, and report how many actors had to be force-deleted.  A leaked
 *   actor poisons every later scenario, and the resulting failure surfaces
 *   somewhere else entirely, so it is reported per scenario.
 *
 ****************************************************************************/

int stc_teardown(void)
{
	int leaked = 0;
	int i;

	/* Stop the CPU burner first: a spinner left running starves everything. */

	g_stc_spin_stop = true;

	for (i = 0; i < STC_MAX_ACTORS; i++) {
		/* Allow an actor that has already done its last observable action a
		 * bounded moment to execute the handful of instructions between that
		 * action and its exit point.  Anything still unfinished after the
		 * grace period is genuinely stuck, and is reported.
		 */

		if (g_stc_actor[i].spawned && !stc_finished_within(i, STC_GRACE_MS)) {
			(void)task_delete(g_stc_actor[i].pid);
			leaked++;
		}

		g_stc_actor[i].spawned = false;
		g_stc_actor[i].pid = 0;

		(void)sem_destroy(&g_stc_actor[i].go);
	}

	return leaked;
}

/****************************************************************************
 * Name: stc_spawn
 ****************************************************************************/

pid_t stc_spawn(const char *name, int prio, main_t entry, int slot)
{
	static char slotarg[STC_MAX_ACTORS][4];
	char *argv[2];
	pid_t pid;

	if (slot < 0 || slot >= STC_MAX_ACTORS) {
		return (pid_t)ERROR;
	}

	/* task_create() copies the argument strings into the new task, so a
	 * per-slot static buffer is safe and avoids handing the actor a pointer
	 * into the harness stack.  The kernel places the task name in argv[0],
	 * so the actor reads its slot from argv[1].
	 */

	(void)snprintf(slotarg[slot], sizeof(slotarg[slot]), "%d", slot);
	argv[0] = slotarg[slot];
	argv[1] = NULL;

	pid = task_create(name, prio, STC_STACKSIZE, entry, (char *const *)argv);
	if (pid <= 0) {
		return (pid_t)ERROR;
	}

	g_stc_actor[slot].pid = pid;
	g_stc_actor[slot].stage = 0;
	g_stc_actor[slot].spawned = true;
	g_stc_actor[slot].finished = false;

	stc_pin(pid);

	/* Register with the boost-leak monitor.  Doing it here means every
	 * scenario built on this harness is monitored without writing a line for
	 * it, and the recorded base priority is the true creation priority rather
	 * than anything read back from the kernel.
	 */

	stc_mon_register_actor(pid, prio);

	return pid;
}

/****************************************************************************
 * Name: stc_go
 ****************************************************************************/

void stc_go(int slot)
{
	if (slot >= 0 && slot < STC_MAX_ACTORS) {
		(void)sem_post(&g_stc_actor[slot].go);
	}
}

/****************************************************************************
 * Name: stc_wait_go
 ****************************************************************************/

void stc_wait_go(int slot)
{
	if (slot < 0 || slot >= STC_MAX_ACTORS) {
		return;
	}

	while (sem_wait(&g_stc_actor[slot].go) != OK) {
		if (get_errno() != EINTR) {
			break;
		}
	}
}

/****************************************************************************
 * Name: stc_stage
 ****************************************************************************/

void stc_stage(int slot, uint8_t stage)
{
	if (slot >= 0 && slot < STC_MAX_ACTORS) {
		g_stc_actor[slot].stage = stage;
	}
}

/****************************************************************************
 * Name: stc_actor_done
 ****************************************************************************/

void stc_actor_done(int slot)
{
	if (slot >= 0 && slot < STC_MAX_ACTORS) {
		g_stc_actor[slot].stage = STC_STAGE_DONE;
		g_stc_actor[slot].finished = true;
	}
}

/****************************************************************************
 * Name: stc_actor_forget
 ****************************************************************************/

void stc_actor_forget(int slot)
{
	if (slot >= 0 && slot < STC_MAX_ACTORS) {
		g_stc_actor[slot].spawned = false;
		g_stc_actor[slot].finished = true;
		g_stc_actor[slot].pid = 0;
	}
}

/****************************************************************************
 * Name: stc_wait_stage
 ****************************************************************************/

int stc_wait_stage(int slot, uint8_t stage)
{
	int retries = (STC_TIMEOUT_MS * 1000) / STC_POLL_US;

	if (slot < 0 || slot >= STC_MAX_ACTORS) {
		return ERROR;
	}

	while (retries-- > 0) {
		if (g_stc_actor[slot].stage >= stage) {
			return OK;
		}

		usleep(STC_POLL_US);
	}

	return ERROR;
}

/****************************************************************************
 * Name: stc_wait_count
 *
 * Description:
 *   Wait until sem's count reaches the expected value.  The predicate is an
 *   exact integer; only the retry cadence and the give-up bound are timed, and
 *   no oracle depends on either.
 *
 ****************************************************************************/

int stc_wait_count(sem_t *sem, int expected)
{
	int retries = (STC_TIMEOUT_MS * 1000) / STC_POLL_US;
	int value;

	while (retries-- > 0) {
		if (sem_getvalue(sem, &value) != OK) {
			return ERROR;
		}

		if (value == expected) {
			return OK;
		}

		usleep(STC_POLL_US);
	}

	return ERROR;
}

/****************************************************************************
 * Name: stc_wait_finished
 ****************************************************************************/

int stc_wait_finished(int slot)
{
	int retries = (STC_TIMEOUT_MS * 1000) / STC_POLL_US;

	if (slot < 0 || slot >= STC_MAX_ACTORS) {
		return ERROR;
	}

	while (retries-- > 0) {
		if (g_stc_actor[slot].finished) {
			return OK;
		}

		usleep(STC_POLL_US);
	}

	return ERROR;
}

/****************************************************************************
 * Name: stc_getprio
 ****************************************************************************/

int stc_getprio(pid_t pid)
{
	struct sched_param param;

	if (sched_getparam(pid, &param) != OK) {
		return ERROR;
	}

	return param.sched_priority;
}

/****************************************************************************
 * Name: stc_getcount
 ****************************************************************************/

int stc_getcount(sem_t *sem)
{
	int value;

	if (sem_getvalue(sem, &value) != OK) {
		return INT32_MIN;
	}

	return value;
}

/****************************************************************************
 * Name: stc_snapshot
 ****************************************************************************/

int stc_snapshot(sem_t *sem, struct sem_snapshot_s *snap)
{
	int fd = tc_get_drvfd();

	if (sem == NULL || snap == NULL || fd < 0) {
		return ERROR;
	}

	memset(snap, 0, sizeof(struct sem_snapshot_s));
	snap->sem = sem;

	if (ioctl(fd, TESTIOC_SEM_SNAPSHOT, (unsigned long)snap) != OK) {
		return ERROR;
	}

	return OK;
}

/****************************************************************************
 * Name: stc_snapshot_counts
 ****************************************************************************/

int stc_snapshot_counts(struct sem_snapshot_s *snap, pid_t pid)
{
	int i;

	if (snap == NULL) {
		return 0;
	}

	for (i = 0; i < snap->nholders; i++) {
		if (snap->holder[i].pid == pid) {
			return snap->holder[i].counts;
		}
	}

	return 0;
}
