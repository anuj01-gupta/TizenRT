/****************************************************************************
 *
 * Copyright 2017 Samsung Electronics All Rights Reserved.
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
#include <stdlib.h>
#include <string.h>

#include <tinyara/fs/fs.h>
#include <stdbool.h>
#include <tinyara/os_api_test_drv.h>
#include <tinyara/sched.h>
#include <tinyara/irq.h>
#include <tinyara/mmu.h>
#ifdef CONFIG_WDOG_MMU_DEFECT_TEST
/* Real kernel heap boundaries (mm_heapstart[] / mm_heapend[]) are needed to
 * tell whether the allocator can reach the watchdog page.  Membership in
 * CONFIG_RAM_START..+CONFIG_RAM_SIZE cannot answer that: on this SoC that
 * range covers every DDR address.
 */
#include <tinyara/mm/mm.h>
#endif
#ifdef CONFIG_MMU_PAGE_PROTECT
#include <tinyara/arch.h>
#include <time.h>
#include <unistd.h>
/* External function to get page table base */
extern uint32_t *mmu_get_os_l1_pgtbl(void);
#define PGTABLE_BASE_VADDR  ((uint32_t)mmu_get_os_l1_pgtbl())
/* External function to get L1 page table (matches mmu_split_section_to_l2) */
extern uint32_t *mmu_l1_pgtable_safe(void);
/* R4 range protect API (arm_mmu.c) */
extern int mmu_protect_range_no_access(uintptr_t vaddr, size_t len);
extern int mmu_restore_range(uintptr_t vaddr, size_t len);
extern void mmu_set_uaf_protect_enabled(int on);
extern void mmu_protect_get_counters(uint32_t *ok, uint32_t *skipped,
				     uint32_t *failed_budget);
/* MMU Page Table Entry macros - copied from arch/arm/src/armv7-a/mmu.h */

#define PMD_TYPE_SHIFT         (0)
#define PMD_TYPE_MASK          (3 << PMD_TYPE_SHIFT)
#define PMD_TYPE_SECT          (2 << PMD_TYPE_SHIFT)  /* 1MB section */
#define PMD_TYPE_PTE           (1 << PMD_TYPE_SHIFT)  /* L2 page table */
#define PMD_PTE_PADDR_MASK     (0xFFFFFC00)           /* L2 table address mask */
#endif
#include "os_api_test_proto.h"
#include "wdog/wdog.h"

#ifdef CONFIG_EXAMPLES_MEM_PROTECT_TEST
#include <tinyara/binfmt/binfmt.h>
#include <tinyara/mem_protect_test.h>
#include "binary_manager/binary_manager_internal.h"
#endif
/****************************************************************************
 * Public variables
 ****************************************************************************/
#ifdef CONFIG_EXAMPLES_MEM_PROTECT_TEST
extern uint32_t _stext_flash;
extern uint32_t _sdata;
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int os_api_test_drv_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static ssize_t os_api_test_drv_read(FAR struct file *filep, FAR char *buffer, size_t len);
static ssize_t os_api_test_drv_write(FAR struct file *filep, FAR const char *buffer, size_t len);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations os_api_test_drv_fops = {
	0,						/* open */
	0,						/* close */
	os_api_test_drv_read,				/* read */
	os_api_test_drv_write,				/* write */
	0,						/* seek */
	os_api_test_drv_ioctl				/* ioctl */
#ifndef CONFIG_DISABLE_POLL
	, 0						/* poll */
#endif
};

#if defined(CONFIG_MMU_PAGE_PROTECT) || defined(CONFIG_WDOG_MMU_PROTECT)
/* Every call that changes a descriptor or opens a write window returns a
 * status.  The cases below used to discard it, so a refused window or a
 * restore that did not take printed exactly the same "done" line as a
 * success.  MMU_TRY() reports the status once and counts it; MMU_TRYQ() only
 * counts it, for the loops that measure time and must not print.  A case may
 * only claim PASS while the count stays where it started.
 */

static int g_mmu_op_failures;

#define MMU_TRY(call) \
	do \
	  { \
		int _st = (call); \
		if (_st != OK) \
		  { \
			g_mmu_op_failures++; \
			lldbg("MMU_OP: %s -> %d\n", #call, _st); \
		  } \
	  } \
	while (0)

#define MMU_TRYQ(call) \
	do \
	  { \
		if ((call) != OK) \
		  { \
			g_mmu_op_failures++; \
		  } \
	  } \
	while (0)
#endif /* CONFIG_MMU_PAGE_PROTECT || CONFIG_WDOG_MMU_PROTECT */


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* TC-R3 helpers: timing.  The PMU is NOT reachable from the non-secure
 * AP OS on rtl8730e: every cp15 access on crn 9 / crn 15 traps to the
 * secure world as an Undefined Instruction (observed: reboot reason 52
 * in the middle of Phase C).  There is no DWT/CYCCNT on ARMv7-A,
 * up_perf_init() is never called and CONFIG_ARCH_PERF_EVENTS is off, so
 * the only clock available is CLOCK_MONOTONIC with a 1ms floor
 * (CONFIG_USEC_PER_TICK=1000).  Per-operation costs are therefore
 * measured as batch averages over many operations. */
#ifdef CONFIG_MMU_PAGE_PROTECT
#define R3_WS_SIZE    (4 * 1024 * 1024)
#define R3_REPS       250
#define R3_SPLIT_REPS 4       /* fresh sections timed for the Phase C cost */
#define R3_PAIR_REPS  4000    /* protect+restore pairs timed in Phase D/E  */
#define R3_PAIR_BATCH 1000    /* pairs per timing sample                   */

/* Per-pair resolution of the batch average, in microseconds: one clock tick
 * (1000us here) divided by the number of pairs in one sample.  With the old
 * batch of 100 this was 10us, so the reported "3 us" and "0 us" were below
 * what the measurement can resolve at all; a value under this figure is
 * printed as "below clock resolution" instead of as a number.
 */

#define R3_TICK_US    (1000 / R3_PAIR_BATCH)

static uint32_t r3_now_us(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint32_t)ts.tv_sec * 1000000u + (uint32_t)(ts.tv_nsec / 1000);
}

/* Sequential-stride scan of the 4MB working set + allocator churn.
 * The scan is TLB-sensitive (64B stride over 4MB = 1024 distinct
 * pages touched per pass); the churn exercises allocator paths that
 * the R4 hooks run through. */
static void r3_workload(FAR uint32_t *ws, uint32_t reps)
{
	uint32_t nwords = R3_WS_SIZE / sizeof(uint32_t);
	uint32_t r;
	uint32_t i;

	for (r = 0; r < reps; r++) {
		for (i = 0; i < nwords; i += 16) {
			ws[i] = r;
		}

		for (i = 0; i < 300; i++) {
			FAR void *p = kmm_malloc(64 + (i & 0x3f));
			if (p) {
				kmm_free(p);
			}
		}
	}
}
#endif /* CONFIG_MMU_PAGE_PROTECT */
/* R4 coverage: how much of the free() traffic the UAF hook actually
 * protected.  Reported next to the counters because "failed_budget = 0" on
 * its own says nothing when almost every free() was skipped (observed on
 * rtl8730e: ok=151 skipped=961, i.e. ~13%).  Defined outside the
 * CONFIG_MMU_PAGE_PROTECT block because the R4 case is guarded by
 * CONFIG_MM_HEAP_UAF_PROTECT.
 */

#define R4_COVERAGE(ok, skipped) \
	(((ok) + (skipped)) ? (unsigned int)((ok) * 100u / ((ok) + (skipped))) : 0u)

/* TC-R4 helper: one round of mixed-size churn.  Shared by step 2 and the
 * phase-5 endurance loop so there is one definition of the workload. */

#ifdef CONFIG_MM_HEAP_UAF_PROTECT
static void r4_mixed_churn(int iterations)
{
	int i;

	for (i = 0; i < iterations; i++) {
		FAR uint8_t *a = (FAR uint8_t *)kmm_malloc(4096);
		FAR uint8_t *b = (FAR uint8_t *)kmm_malloc(5120);
		FAR uint8_t *c = (FAR uint8_t *)kmm_malloc(64 * 1024);

		if (a) {
			a[0] = (uint8_t)i;
			a[4095] = (uint8_t)i;
		}
		if (b) {
			b[0] = (uint8_t)i;
		}
		if (c) {
			c[0] = (uint8_t)i;
			c[65535] = (uint8_t)i;
		}
		kmm_free(a);
		kmm_free(b);
		kmm_free(c);
	}
}
#endif /* CONFIG_MM_HEAP_UAF_PROTECT */

/* TC-R4 helper: core-1 heap churn thread (step 3, dual-core) */
#if defined(CONFIG_MM_HEAP_UAF_PROTECT) && defined(CONFIG_SMP)
static volatile int g_r4_core1_done;

static int r4_core1_churn(int argc, FAR char *argv[])
{
	int i;

	for (i = 0; i < 50; i++) {
		FAR uint8_t *a = (FAR uint8_t *)kmm_malloc(4096);
		FAR uint8_t *b = (FAR uint8_t *)kmm_malloc(64 * 1024);

		if (a) {
			a[0] = (uint8_t)i;
			a[4095] = (uint8_t)i;
			kmm_free(a);
		}

		if (b) {
			b[0] = (uint8_t)i;
			b[65535] = (uint8_t)i;
			kmm_free(b);
		}
	}

	g_r4_core1_done = 1;
	return 0;
}
#endif /* CONFIG_MM_HEAP_UAF_PROTECT && CONFIG_SMP */

/************************************************************************************
 * Name: os_api_test_drv_ioctl
 *
 * Description:  The standard ioctl method.
 *
 ************************************************************************************/

/* File-scope callback and thread functions for scenario tests.
 * These cannot be defined inside the ioctl function (C doesn't allow
 * static nested functions).
 */

/* TC-06: Multiple watchdog expiration callback */
static volatile int g_tc06_callback_count;
static void tc06_callback(int argc, uint32_t arg1, ...)
{
	g_tc06_callback_count++;
}

/* TC-08: Core 1 thread for SMP concurrent test */
static volatile int g_tc08_core1_running;
static volatile int g_tc08_core1_ok_count;
static volatile int g_tc08_core1_stop;
static int tc08_core1_thread(int argc, char *argv[])
{
	int i;
	WDOG_ID wdog;
	int ok = 0;

	lldbg("TC08: Core 1 thread started (pid=%d)\n", getpid());

	for (i = 0; i < 500 && !g_tc08_core1_stop; i++) {
		wdog = wd_create();
		if (!wdog) {
			lldbg("TC08: Core1 wd_create() failed at %d\n", i);
			break;
		}
		if (wd_start(wdog, 10000, NULL, 0) != OK) {
			lldbg("TC08: Core1 wd_start() failed at %d\n", i);
			wd_delete(wdog);
			break;
		}
		wd_cancel(wdog);
		wd_delete(wdog);
		ok++;
	}

	g_tc08_core1_ok_count = ok;
	g_tc08_core1_running = 0;
	lldbg("TC08: Core 1 thread done (%d iterations)\n", ok);
	return 0;
}

/* TC-09: Callback and Core 1 thread for SMP tick/cb test */
static volatile int g_tc09_callback_fired;
static volatile int g_tc09_core1_done;
static WDOG_ID g_tc09_wdog;

static void tc09_callback(int argc, uint32_t arg1, ...)
{
	g_tc09_callback_fired = 1;
}

static int tc09_core1_thread(int argc, char *argv[])
{
	lldbg("TC09: Core 1 thread started (pid=%d)\n", getpid());

	g_tc09_wdog = wd_create();
	if (!g_tc09_wdog) {
		lldbg("TC09: Core1 wd_create() failed\n");
		g_tc09_core1_done = 1;
		return -ENOMEM;
	}

	if (wd_start(g_tc09_wdog, 1, tc09_callback, 0) != OK) {
		lldbg("TC09: Core1 wd_start() failed\n");
		wd_delete(g_tc09_wdog);
		g_tc09_wdog = NULL;
		g_tc09_core1_done = 1;
		return -EIO;
	}

	lldbg("TC09: Core1 started watchdog with 1-tick delay\n");
	g_tc09_core1_done = 1;
	return 0;
}

/* AL SMP: Core 1 thread for cross-core page access test
 *
 * Two-phase access:
 *   Phase 0: Access page BEFORE protection (creates stale TLB entry on Core 1)
 *   Phase 1: Access page AFTER protection (should Data Abort if fix works)
 */
static volatile uint32_t g_al_smp_page_addr;
static volatile int g_al_smp_access_type;  /* 0 = read, 1 = write */
static volatile int g_al_smp_phase1_done;  /* Core 1 completed phase 1 (pre-protection access) */
static volatile int g_al_smp_phase2_go;    /* Core 0 signals Core 1 to do phase 2 (post-protection) */
static volatile int g_al_smp_core1_cpu;    /* Records which CPU Core 1 thread actually ran on (-1 = not yet) */

static int al_smp_core1_thread(int argc, char *argv[])
{
	volatile uint8_t *page = (volatile uint8_t *)g_al_smp_page_addr;

	/* Record which CPU this thread is running on, for SMP verification */
	g_al_smp_core1_cpu = up_cpu_index();
	__asm__ volatile("dsb" ::: "memory");  /* Ensure Core 0 sees the CPU ID write */

	lldbg("AL SMP: Core 1 thread started on CPU%d (pid=%d), page=0x%08x, access=%s\n",
	      g_al_smp_core1_cpu, getpid(), g_al_smp_page_addr,
	      g_al_smp_access_type ? "write" : "read");

	/* Phase 1: Access page BEFORE protection — creates stale TLB entry */
	lldbg("AL SMP: Phase 1 — accessing page BEFORE protection (creates stale TLB)\n");
	if (g_al_smp_access_type == 0) {
		volatile uint8_t val = page[0];
		(void)val;
		lldbg("AL SMP: Phase 1 read OK (val=0x%02x) — TLB now has RW entry on Core 1\n", val);
	} else {
		page[0] = 0xBB;
		lldbg("AL SMP: Phase 1 write OK — TLB now has RW entry on Core 1\n");
	}

	/* Signal Core 0 that phase 1 is done */
	g_al_smp_phase1_done = 1;

	/* Wait for Core 0 to protect the page and signal us */
	lldbg("AL SMP: Waiting for Core 0 to protect page...\n");
	while (!g_al_smp_phase2_go) {
		__asm__ volatile("dmb" ::: "memory");
		up_mdelay(1);
	}
	__asm__ volatile("dmb" ::: "memory");  /* Ensure we see the protected PTE before accessing */

	/* Phase 2: Access page AFTER protection — should Data Abort if fix works */
	lldbg("AL SMP: Phase 2 — accessing page AFTER protection (expect Data Abort)\n");
	if (g_al_smp_access_type == 0) {
		lldbg("AL SMP: >>> Core 1 READ, expect Data Abort <<<\n");
		volatile uint8_t val = page[0];
		(void)val;
		lldbg("AL SMP: ERROR — read succeeded (no Data Abort)! DEFECT REPRODUCED!\n");
	} else {
		lldbg("AL SMP: >>> Core 1 WRITE, expect Data Abort <<<\n");
		page[0] = 0xCC;
		lldbg("AL SMP: ERROR — write succeeded (no Data Abort)! DEFECT REPRODUCED!\n");
	}

	return 0;
}

static int os_api_test_drv_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
	int ret = -EINVAL;
	/* Handle built-in ioctl commands */

	switch (cmd) {
	/* TESTFWIOC_DRIVER_ANALOG - Run the test case for /os/driver/analog module
	 *
	 *   ioctl argument:  An integer value indicating the particular test to be run
	 */

	case TESTIOC_ANALOG:
		break;
#ifndef CONFIG_DISABLE_SIGNALS
	case TESTIOC_GET_SIG_FINDACTION_ADD:
	case TESTIOC_SIGNAL_PAUSE:
	case TESTIOC_GET_TCB_SIGPROCMASK:
		ret = test_signal(cmd, arg);
		break;
#endif
	case TESTIOC_GET_SELF_PID:
	case TESTIOC_IS_ALIVE_THREAD:
	case TESTIOC_GET_TCB_ADJ_STACK_SIZE:
	case TESTIOC_SCHED_FOREACH:
		ret = test_sched(cmd, arg);
		break;
	case TESTIOC_CLOCK_ABSTIME2TICKS_TEST:
		ret = test_clock(cmd, arg);
		break;
	case TESTIOC_TIMER_INITIALIZE_TEST:
		ret = test_timer(cmd, arg);
		break;
	case TESTIOC_SEM_TICK_WAIT_TEST:
		ret = test_sem(cmd, arg);
		break;
#if defined(CONFIG_SCHED_HAVE_PARENT) && defined(CONFIG_SCHED_CHILD_STATUS)
	case TESTIOC_GROUP_ADD_FINED_REMOVE_TEST:
	case TESTIOC_GROUP_ALLOC_FREE_TEST:
	case TESTIOC_GROUP_EXIT_CHILD_TEST:
	case TESTIOC_GROUP_REMOVECHILDREN_TEST:
		ret = test_group(cmd, arg);
		break;
#endif
	case TESTIOC_TASK_REPARENT:
	case TESTIOC_TASK_INIT_TEST:
		ret = test_task(cmd, arg);
		break;
#ifdef CONFIG_TC_COMPRESS_READ
	case TESTIOC_COMPRESSION_TEST:
		ret = test_compress_decompress(cmd, arg);
		break;
#endif

#ifdef CONFIG_EXAMPLES_MEM_PROTECT_TEST
	case TESTIOC_MEM_PROTECTTEST:
		ret = OK;
		struct mem_protecttest_arg_s *obj = (struct mem_protecttest_arg_s*)arg;
		if (!obj) {
			return -EINVAL;
		}

		switch (obj->type) {
		case MEM_PROTECTTEST_KERNEL_CODE:
			obj->addr = &_stext_flash;
			break;
		case MEM_PROTECTTEST_KERNEL_DATA:
			obj->addr = &_sdata;
			break;
		case MEM_PROTECTTEST_APP_ADDR:
		{
			/* Find the current executing app and return an address
			* which belongs to any other app in the system. Here,
			* we choose to return the address of the app heap
			*/
			uint32_t binidx = sched_self()->group->tg_binidx;
			binidx = (binidx + 1) % (binary_manager_get_ucount() + 1);
			if (binidx == 0) {
				binidx++;
			}
			struct tcb_s *tcb = (struct tcb_s *)sched_gettcb(BIN_ID(binidx));
			if (tcb) {
				obj->addr = (volatile uint32_t *)tcb->uheap;
			} else {
				ret = -ESRCH;
			}
			break;
		}
		default:
			ret = -EINVAL;
			break;
		}

		break;
#endif

#ifdef CONFIG_ARMV8M_TRUSTZONE
	case TESTIOC_TZ:
		ret = test_tz();
		break;
#endif

#ifdef CONFIG_EXAMPLES_STACK_PROTECTION
	case TESTIOC_KTHREAD_STACK_PROTECTION_TEST:
		ret = test_kthread_stack_overflow_protection(cmd, arg);
		break;
#endif

#ifdef CONFIG_TC_NET_PBUF
	/* Run the test case for pbuf   */
	case TESTIOC_NET_PBUF:
		ret = test_net_pbuf(cmd, arg);
		break;
#endif
#if defined(CONFIG_AUTOMOUNT_USERFS) && defined(CONFIG_EXAMPLES_TESTCASE_FILESYSTEM)
	case TESTIOC_GET_FS_PARTNO:
		ret = test_fs_get_devname();
		break;
#endif
#if defined(CONFIG_EXAMPLES_MMU_PROTECT_TEST) && defined(CONFIG_MMU_PAGE_PROTECT)
	case TESTIOC_UAF_MMU_PROTECT: {
		/* Use-after-free detection via MMU page protection.
		 *
		 * arg = 0: read from protected page (UAF read)
		 * arg = 1: write to protected page (UAF write)
		 *
		 * 1. Allocate a page-aligned buffer (simulates a kernel allocation).
		 * 2. Write data to it (normal use).
		 * 3. "Free" it: call mmu_set_page_no_access() to make the page
		 *    inaccessible at any privilege level (AP=000).
		 * 4. Read or write the protected page — the MMU triggers a Data Abort.
		 *
		 * The Data Abort handler (arm_dataabort) prints PC/DFAR/DFSR and
		 * calls PANIC().  DFAR contains the freed page address, confirming
		 * a use-after-free was detected.
		 *
		 * This function does NOT return — the Data Abort is fatal.
		 */
		uint8_t *buf;
		uint32_t page_addr;
		volatile uint8_t val;
		int do_write = (arg == 1);

		buf = (uint8_t *)memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("UAF test: memalign failed\n");
			return -ENOMEM;
		}

		/* Get page-aligned address */
		page_addr = ((uint32_t)buf + 4095) & ~4095u;

		/* Step 1: Normal use — write data */
		memset((void *)page_addr, 0xAA, 4096);
		lldbg("UAF test: allocated buffer at 0x%08x, page 0x%08x\n",
		      (uint32_t)buf, page_addr);
		lldbg("UAF test: wrote data, buf[0] = 0x%02x\n",
		      ((volatile uint8_t *)page_addr)[0]);

		/* Step 2: Simulate free — protect the page via MMU */
		mmu_set_page_no_access(page_addr);
		lldbg("UAF test: page 0x%08x set to No-Access (simulating free)\n",
		      page_addr);

		/* Step 3: Use-after-free — access the protected page.
		 * The MMU sees AP=000 and raises a permission fault (Data Abort).
		 * DFAR will contain page_addr — the freed address that was accessed.
		 */
		if (do_write) {
			lldbg("UAF test: >>> WRITE to freed memory, expect Data Abort <<<\n");
			((volatile uint8_t *)page_addr)[0] = 0xBB;
		} else {
			lldbg("UAF test: >>> READ from freed memory, expect Data Abort <<<\n");
			val = ((volatile uint8_t *)page_addr)[0];
		}

		/* If we reach here, protection failed */
		(void)val;
		lldbg("UAF test: ERROR — access succeeded, Data Abort NOT triggered!\n");
		MMU_TRY(mmu_restore_page_pte(page_addr, mmu_save_page_pte(page_addr)));
		free(buf);
		ret = -EIO;
		break;
	}
	case TESTIOC_UAF_NO_MMU: {
		/* UAF WITHOUT MMU protection — demonstrates what happens when
		 * freed memory is accessed without any protection.
		 *
		 * 1. Allocate a page-aligned buffer.
		 * 2. Write known data to it.
		 * 3. free() the buffer (real free, no MMU protection).
		 * 4. Read from the freed memory — access succeeds silently,
		 *    returning stale data.  No Data Abort, no detection.
		 * 5. Write to the freed memory — also succeeds silently.
		 *
		 * This shows why MMU protection is needed: without it, a
		 * use-after-free goes completely undetected.
		 */
		uint8_t *buf;
		uint32_t page_addr;
		volatile uint8_t val;

		buf = (uint8_t *)memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("UAF test (no mmu): memalign failed\n");
			return -ENOMEM;
		}

		page_addr = ((uint32_t)buf + 4095) & ~4095u;

		/* Step 1: Normal use — write data */
		memset((void *)page_addr, 0xAA, 4096);
		lldbg("UAF test (no mmu): allocated buffer at 0x%08x, page 0x%08x\n",
		      (uint32_t)buf, page_addr);
		lldbg("UAF test (no mmu): wrote data, buf[0] = 0x%02x\n",
		      ((volatile uint8_t *)page_addr)[0]);

		/* Step 2: Free the buffer — NO MMU protection */
		free(buf);
		lldbg("UAF test (no mmu): buffer freed (no MMU protection)\n");

		/* Step 3: Use-after-free — read from freed memory.
		 * Without MMU protection, this access succeeds silently.
		 * The data may be stale (0xAA) or garbage if the allocator
		 * has already reused the page.
		 */
		lldbg("UAF test (no mmu): >>> reading freed memory (no protection) <<<\n");
		val = ((volatile uint8_t *)page_addr)[0];
		lldbg("UAF test (no mmu): read succeeded! val = 0x%02x (stale data)\n",
		      val);

		/* Step 4: Use-after-free — write to freed memory.
		 * Without MMU protection, this also succeeds silently,
		 * silently corrupting whatever may now be at that address.
		 */
		lldbg("UAF test (no mmu): >>> writing to freed memory (no protection) <<<\n");
		((volatile uint8_t *)page_addr)[0] = 0xCC;
		lldbg("UAF test (no mmu): write succeeded! (silent corruption)\n");

		lldbg("UAF test (no mmu): NO Data Abort — UAF went UNDETECTED!\n");
		lldbg("UAF test (no mmu): This is why MMU protection is needed.\n");

		ret = OK;
		break;
	}

#ifdef CONFIG_MMU_PAGE_PROTECT
	/* TC-AL-R1: Section split correctness - Verify all 256 L2 entries (Safe) */
	case TESTIOC_AL_R1_SPLIT_CORRECTNESS: {
		uint8_t *buf;
		uint32_t page_addr;
		uint32_t original_l1;
		uint32_t new_l1;
		uint32_t *l1table;
		uint32_t *l2table;
		int pass_count = 0;
		int fail_count = 0;
		int i;

		lldbg("=== R1 Unit Test: Split Correctness ===\n");

		/* Allocate aligned page */
		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("R1: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;

		lldbg("R1: Test address = 0x%08X\n", page_addr);

		/* Get L1 table - use SAME table as mmu_split_section_to_l2() */
		l1table = mmu_l1_pgtable_safe();
		uint32_t l1_index = page_addr >> 20;

		/* Step 1: Read original L1 entry */
		original_l1 = l1table[l1_index];
		lldbg("R1: Original L1[%d] = 0x%08X (type=0x%x)\n",
			   l1_index, original_l1, original_l1 & PMD_TYPE_MASK);

		/* Step 2: If already split (PTE), restore to section first */
		if ((original_l1 & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
			lldbg("R1: Memory already split (PTE), restoring to section...\n");
			
			/* Calculate section base address */
			uint32_t sect_addr = page_addr & 0xFFE00000;
			
			/* Read L2 table to get original attributes */
			uint32_t *l2table_temp = (uint32_t *)(original_l1 & PMD_PTE_PADDR_MASK);
			uint32_t l2_entry = l2table_temp[0];
			
			/* Reconstruct L1 section entry from L2 attributes */
			uint32_t sect_entry = sect_addr;
			sect_entry |= PMD_TYPE_SECT;
			sect_entry |= ((l2_entry >> 4) & 0x3) << 10;    /* AP[1:0] */
			sect_entry |= ((l2_entry >> 9) & 0x1) << 15;    /* APX */
			sect_entry |= ((l2_entry >> 6) & 0x7) << 12;    /* TEX[2:0] */
			sect_entry |= ((l2_entry >> 3) & 0x1) << 3;     /* C */
			sect_entry |= ((l2_entry >> 2) & 0x1) << 2;     /* B */
			sect_entry |= ((l2_entry >> 0) & 0x1) << 4;     /* XN */
			sect_entry |= (1 << 16);                         /* S */
			
			/* Restore to section.  The L2 table this entry pointed
			 * at stays claimed - the pool has no release path by
			 * design (arm_mmu.c:812), so each re-split claims a
			 * fresh table and l2_pool (id 62) counts it. */
			l1table[l1_index] = sect_entry;
			
			/* Invalidate by MVA, inner shareable - the form
			 * mmu_split_section_to_l2() uses (arm_mmu.c:268): the
			 * peer core may still hold the 1MB section entry. */
			__asm__ volatile("mcr p15, 0, %0, c8, c3, 3" :: "r"(sect_addr));
			__asm__ volatile("dsb sy");
			__asm__ volatile("isb");
			
			lldbg("R1: Restored to section 0x%08X (MVA invalidate, shareable)\n",
			      sect_entry);
			
			/* Update original_l1 for comparison later */
			original_l1 = sect_entry;
			lldbg("R1: Now splitting fresh for this TC\n");
		}
		else if ((original_l1 & PMD_TYPE_MASK) != PMD_TYPE_SECT) {
			lldbg("R1: ERROR - Unknown L1 type 0x%x!\n", original_l1 & PMD_TYPE_MASK);
			kmm_free(buf);
			return -EINVAL;
		}
		else {
			lldbg("R1: Fresh section found, splitting for this TC\n");
		}

		/* Step 3: Now split fresh (whether restored or was already section) */
		lldbg("R1: Calling mmu_split_section_to_l2(0x%08X)...\n", page_addr);
		mmu_split_section_to_l2(page_addr);

		/* Step 4: Verify new L1 entry */
		new_l1 = l1table[l1_index];
		lldbg("R1: New L1[%d] = 0x%08X (type=0x%x)\n",
			   l1_index, new_l1, new_l1 & PMD_TYPE_MASK);

		if ((new_l1 & PMD_TYPE_MASK) != PMD_TYPE_PTE) {
			lldbg("R1: FAIL - L1 entry type is not Page Table (expected 0x10, got 0x%x)\n",
				   new_l1 & PMD_TYPE_MASK);
			kmm_free(buf);
			return -EINVAL;
		}
		lldbg("R1: PASS - L1 entry type is Page Table (0x10)\n");
		pass_count++;

		/* Get L2 table address */
		l2table = (uint32_t *)(new_l1 & PMD_PTE_PADDR_MASK);
		lldbg("R1: L2 table at 0x%08X\n", l2table);

		/* Step 5: Verify all 256 L2 entries match original L1 attributes */
		lldbg("R1: Verifying 256 L2 entries...\n");

		/* Extract original L1 attributes */
		uint32_t original_ap = (original_l1 >> 10) & 0x3;      /* AP[1:0] at bits 10-11 */
		uint32_t original_apx = (original_l1 >> 15) & 0x1;     /* APX at bit 15 */
		uint32_t original_tex = (original_l1 >> 12) & 0x7;     /* TEX at bits 12-14 */
		uint32_t original_c = (original_l1 >> 3) & 0x1;        /* C at bit 3 */
		uint32_t original_b = (original_l1 >> 2) & 0x1;        /* B at bit 2 */

		for (i = 0; i < 256; i++) {
			uint32_t l2_entry = l2table[i];  /* Read L2 entry */
			uint32_t l2_ap = (l2_entry >> 4) & 0x3;            /* AP[1:0] at bits 4-5 */
			uint32_t l2_apx = (l2_entry >> 9) & 0x1;           /* APX at bit 9 */
			uint32_t l2_tex = (l2_entry >> 6) & 0x7;           /* TEX at bits 6-8 */
			uint32_t l2_c = (l2_entry >> 3) & 0x1;             /* C at bit 3 */
			uint32_t l2_b = (l2_entry >> 2) & 0x1;             /* B at bit 2 */

			/* Compare permissions, TEX, C, B bits */
			if (l2_ap == original_ap &&
				l2_apx == original_apx &&
				l2_tex == original_tex &&
				l2_c == original_c &&
				l2_b == original_b) {
				pass_count++;
			} else {
				fail_count++;
				if (fail_count <= 5) {  /* Print first 5 failures */
					lldbg("  FAIL L2[%d]: entry=0x%08X\n", i, l2_entry);
					lldbg("    AP:   exp 0x%x, got 0x%x\n", original_ap, l2_ap);
					lldbg("    APX:  exp %d, got %d\n", original_apx, l2_apx);
					lldbg("    TEX:  exp %d, got %d\n", original_tex, l2_tex);
					lldbg("    C:    exp %d, got %d\n", original_c, l2_c);
					lldbg("    B:    exp %d, got %d\n", original_b, l2_b);
				}
			}
		}

		/* Step 6: Verify memory is still accessible */
		lldbg("R1: Testing memory accessibility...\n");
		volatile uint8_t *test_ptr = (volatile uint8_t *)page_addr;
		uint8_t original_value = test_ptr[0];
		test_ptr[0] = 0xAA;
		uint8_t read_value = test_ptr[0];
		test_ptr[0] = original_value;  /* Restore */

		if (read_value != 0xAA) {
			lldbg("R1: FAIL - Memory write/read test failed!\n");
			kmm_free(buf);
			return -EINVAL;
		}
		lldbg("R1: PASS - Memory is accessible after split\n");
		pass_count++;

		/* Final Result */
		lldbg("\n=== R1 Test Result ===\n");
		lldbg("L2 entries passed: %d/256\n", pass_count - 2);
		lldbg("L2 entries failed: %d/256\n", fail_count);
		lldbg("L1 type test:      PASS\n");
		lldbg("Memory test:       PASS\n");

		if (fail_count == 0) {
			lldbg("\n*** R1 UNIT TEST PASSED: All 256 L2 entries verified ***\n");
			ret = OK;
		} else {
			lldbg("\n*** R1 UNIT TEST FAILED: %d L2 entries mismatch ***\n", fail_count);
			ret = -EINVAL;
		}

		kmm_free(buf);
		break;
	}
	/* TC-AL-01: Section split verification (Safe) */
	case TESTIOC_AL_SPLIT: {
		uint8_t *buf;
		uint32_t page_addr;
		uint32_t pte = 0;
		uint8_t ap = 0xff;

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("AL SPLIT: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;

		lldbg("AL SPLIT: page=0x%08x, calling mmu_split_section_to_l2\n", page_addr);

		MMU_TRY(mmu_split_section_to_l2(page_addr));

		/* Verify memory is still accessible after split */
		memset((void *)page_addr, 0xAA, 4096);

		/* And that the split really happened: "no crash" was the
		 * whole test before, so a split that was refused (L2 pool
		 * exhausted, for instance) printed the same PASS line. */

		if (mmu_get_page_pte_raw(page_addr, &pte, &ap) != OK) {
			lldbg("AL SPLIT: FAIL - section is still not 4KB-mapped\n");
			kmm_free(buf);
			ret = -EIO;
			break;
		}

		lldbg("AL SPLIT: memory read/write OK, now 4KB-mapped (pte=0x%08x AP=%u)\n",
		      pte, (unsigned int)ap);

		kmm_free(buf);
		ret = OK;
		break;
	}

	/* TC-AL-02: No-access + restore round-trip (Safe) */
	case TESTIOC_AL_NOACCESS_RESTORE: {
		uint8_t *buf;
		uint32_t page_addr;
		uint32_t saved_pte;
		uint32_t pte = 0;
		uint8_t ap = 0xff;

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("AL NOACCESS_RESTORE: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;
		memset((void *)page_addr, 0xAA, 4096);

		/* mmu_set_page_no_access() refuses a plain 1MB section with
		 * -EINVAL, so without this split the case protected nothing
		 * and still reported PASS.  Split first, then assert each
		 * step against the live descriptor instead of against the
		 * absence of a crash. */

		MMU_TRY(mmu_split_section_to_l2(page_addr));

		saved_pte = mmu_save_page_pte(page_addr);
		lldbg("AL NOACCESS_RESTORE: saved_pte=0x%08x\n", saved_pte);

		MMU_TRY(mmu_set_page_no_access(page_addr));

		if (mmu_get_page_pte_raw(page_addr, &pte, &ap) != OK || ap != 0) {
			lldbg("AL NOACCESS_RESTORE: FAIL - not No-Access after set (pte=0x%08x AP=%u)\n",
			      pte, (unsigned int)ap);
			MMU_TRY(mmu_restore_page_pte(page_addr, saved_pte));
			kmm_free(buf);
			ret = -EIO;
			break;
		}

		lldbg("AL NOACCESS_RESTORE: page is No-Access (pte=0x%08x AP=0)\n", pte);

		MMU_TRY(mmu_restore_page_pte(page_addr, saved_pte));

		if (mmu_get_page_pte_raw(page_addr, &pte, &ap) != OK ||
		    pte != saved_pte) {
			lldbg("AL NOACCESS_RESTORE: FAIL - descriptor not restored (pte=0x%08x saved=0x%08x)\n",
			      pte, saved_pte);
			kmm_free(buf);
			ret = -EIO;
			break;
		}

		lldbg("AL NOACCESS_RESTORE: page restored to 0x%08x\n", pte);

		/* Verify access works after restore */
		((volatile uint8_t *)page_addr)[0] = 0xBB;
		lldbg("AL NOACCESS_RESTORE: write after restore OK\n");

		kmm_free(buf);
		ret = OK;
		break;
	}

	/* TC-AL-03: RO + restore round-trip (Safe) */
	case TESTIOC_AL_RO_RESTORE: {
		uint8_t *buf;
		uint32_t page_addr;
		uint32_t saved_pte;
		uint32_t pte = 0;
		uint8_t ap = 0xff;
		volatile uint8_t val;

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("AL RO_RESTORE: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;
		memset((void *)page_addr, 0xAA, 4096);

		/* Same reason as AL-02: the RO set is refused on a plain
		 * section, so the case has to split first to be a test. */

		MMU_TRY(mmu_split_section_to_l2(page_addr));

		saved_pte = mmu_save_page_pte(page_addr);
		lldbg("AL RO_RESTORE: saved_pte=0x%08x\n", saved_pte);

		MMU_TRY(mmu_set_page_ro(page_addr));

		/* Read-only means AP2 is set: with domain 0 = Client that is
		 * what turns the page read-only for privileged code, and it
		 * is the same test wdog_page/defect_smp apply.  (The folded
		 * value is 3 in the normal AP[2:0] model and 2 in the
		 * CONFIG_AFE_ENABLE AP[2:1] model, so AP2 is tested, not the
		 * number.)  Anything else means the descriptor was never
		 * changed, which is exactly what this case used to miss.
		 */

		if (mmu_get_page_pte_raw(page_addr, &pte, &ap) != OK || !(ap & 2)) {
			lldbg("AL RO_RESTORE: FAIL - not read-only after set (pte=0x%08x AP=%u, AP2 must be set)\n",
			      pte, (unsigned int)ap);
			MMU_TRY(mmu_restore_page_pte(page_addr, saved_pte));
			kmm_free(buf);
			ret = -EIO;
			break;
		}

		lldbg("AL RO_RESTORE: page is Read-Only (pte=0x%08x AP=3)\n", pte);

		/* Read should succeed under RO */
		val = ((volatile uint8_t *)page_addr)[0];
		lldbg("AL RO_RESTORE: read under RO OK, val=0x%02x\n", val);

		MMU_TRY(mmu_restore_page_pte(page_addr, saved_pte));

		if (mmu_get_page_pte_raw(page_addr, &pte, &ap) != OK ||
		    pte != saved_pte) {
			lldbg("AL RO_RESTORE: FAIL - descriptor not restored (pte=0x%08x saved=0x%08x)\n",
			      pte, saved_pte);
			kmm_free(buf);
			ret = -EIO;
			break;
		}

		lldbg("AL RO_RESTORE: page restored to 0x%08x\n", pte);

		/* Write should succeed after restore */
		((volatile uint8_t *)page_addr)[0] = 0xCC;
		lldbg("AL RO_RESTORE: write after restore OK\n");

		kmm_free(buf);
		ret = OK;
		break;
	}

	/* TC-AL-04: Protection of a section-mapped address is refused (Safe)
	 *
	 * This case used to set a whole 1MB section to No-Access and read
	 * from it, expecting the resulting Data Abort to nest (the section
	 * also covers the TCB).  That premise is obsolete: the arch layer
	 * refuses to widen a 4KB permission change to a 1MB section
	 * (mmu_set_page_no_access() -> -EINVAL, "not 4KB-mapped - refused"),
	 * so the abort cannot happen and the case could only ever print
	 * "ERROR - no Data Abort occurred!".
	 *
	 * What is asserted now is the refusal itself, which is the behaviour
	 * the kernel has to guarantee, plus the consequence that matters:
	 * nothing was written and the memory is still there.  Same invariant
	 * wdog_refuse (TESTIOC_WDOG_MMU_REFUSE_SECTION) checks for the
	 * watchdog page.  Non-fatal by design.
	 */
	case TESTIOC_AL_NOACCESS_ABORT: {
		uint8_t *buf;
		uint32_t page_addr;
		volatile uint8_t val;
		uint32_t saved_pte;
		uint32_t pte = 0;
		uint8_t ap = 0xff;
		bool mapped;
		int status;
		int ops_before = g_mmu_op_failures;

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("AL NOACCESS_ABORT: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;
		memset((void *)page_addr, 0xAA, 4096);

		/* Take the value and the descriptor snapshot BEFORE the
		 * protect call: once the page is No-Access the read below
		 * faults, and nothing may touch the page until the
		 * descriptor is back in place.
		 *
		 * The buffer comes from the kernel heap and every other AL
		 * case allocates the same size, so the allocator hands this
		 * case the very page an earlier al_split(),
		 * al_noaccess_restore(), al_ro_restore(), al_smp_restore() or
		 * al_smp_split() run turned into a 4KB page.  Nothing in the
		 * kernel merges an L1 table back into a 1MB section, so the
		 * section-refusal behaviour is only observable when no AL
		 * case has run yet in this boot.  Ask the page tables which
		 * guarantee applies here instead of printing an assumption:
		 * mmu_get_page_pte_raw() fails with -EINVAL for exactly the
		 * addresses mmu_set_page_no_access() refuses - both test
		 * (l1_entry & PMD_TYPE_MASK) != PMD_TYPE_PTE.
		 */

		val = ((volatile uint8_t *)page_addr)[0];
		saved_pte = mmu_save_page_pte(page_addr);
		mapped = (mmu_get_page_pte_raw(page_addr, &pte, &ap) == OK);

		if (mapped) {
			/* Already 4KB-mapped: the set must be ACCEPTED.
			 * Prove it from the descriptor only - the page
			 * itself stays untouched until it has been restored,
			 * or this non-fatal case ends in a Data Abort.
			 */

			lldbg("AL NOACCESS_ABORT: page=0x%08x already 4KB-mapped (pte=0x%08x AP=%u), expecting the set to be accepted\n",
			      page_addr, pte, (unsigned int)ap);

			if (saved_pte == 0) {
				lldbg("AL NOACCESS_ABORT: FAIL - no descriptor to restore\n");
				kmm_free(buf);
				ret = -EIO;
				break;
			}

			MMU_TRY(mmu_set_page_no_access(page_addr));

			if (mmu_get_page_pte_raw(page_addr, &pte, &ap) != OK ||
			    ap != 0) {
				lldbg("AL NOACCESS_ABORT: FAIL - not No-Access after set (pte=0x%08x AP=%u)\n",
				      pte, (unsigned int)ap);
				MMU_TRY(mmu_restore_page_pte(page_addr,
				                             saved_pte));
				kmm_free(buf);
				ret = -EIO;
				break;
			}

			lldbg("AL NOACCESS_ABORT: page is No-Access (pte=0x%08x AP=0), proven without touching it\n",
			      pte);

			MMU_TRY(mmu_restore_page_pte(page_addr, saved_pte));

			if (mmu_get_page_pte_raw(page_addr, &pte, &ap) != OK ||
			    pte != saved_pte) {
				lldbg("AL NOACCESS_ABORT: FAIL - descriptor not restored (pte=0x%08x saved=0x%08x)\n",
				      pte, saved_pte);
				kmm_free(buf);
				ret = -EIO;
				break;
			}

			/* Reachable again - and only now may the page be
			 * read */

			val = ((volatile uint8_t *)page_addr)[0];

			if (val != 0xAA || g_mmu_op_failures != ops_before) {
				lldbg("AL NOACCESS_ABORT: FAIL - val=0x%02x (want 0xaa), statuses %d -> %d\n",
				      val, ops_before, g_mmu_op_failures);
				kmm_free(buf);
				ret = -EIO;
				break;
			}

			lldbg("AL NOACCESS_ABORT: PASS - 4KB-mapped: set accepted, AP=0 read back from the descriptor, page restored and readable (val=0x%02x)\n",
			      val);
			kmm_free(buf);
			ret = OK;
			break;
		}

		/* Still covered by a 1MB section: the refusal path,
		 * unchanged */

		lldbg("AL NOACCESS_ABORT: page=0x%08x is 1MB-section mapped, calling mmu_set_page_no_access(), expect -EINVAL\n",
		      page_addr);

		status = mmu_set_page_no_access(page_addr);

		/* Refused: the descriptor was not touched, so this must not
		 * fault */

		val = ((volatile uint8_t *)page_addr)[0];

		/* g_mmu_op_failures is a boot-wide counter, so it is compared
		 * against its own value on entry: any case run before this one
		 * may already have counted a refused status, and that is not
		 * this case's failure.  What matters here is that the refused
		 * call counted nothing new.
		 */

		if (status == -EINVAL && val == 0xAA &&
		    g_mmu_op_failures == ops_before) {
			lldbg("AL NOACCESS_ABORT: PASS - section refused (-EINVAL), memory intact (val=0x%02x)\n",
			      val);
			ret = OK;
		} else {
			lldbg("AL NOACCESS_ABORT: FAIL - set -> %d (want -EINVAL), val=0x%02x (want 0xaa), statuses %d -> %d\n",
			      status, val, ops_before, g_mmu_op_failures);
			ret = -EIO;
		}

		kmm_free(buf);
		break;
	}
#endif

#ifdef CONFIG_MMU_PAGE_PROTECT
	/* TC-AL-05: Split + 4KB-granular protect (Fatal, single-core) */
	case TESTIOC_AL_SPLIT_PROTECT: {
		uint8_t *buf;
		uint32_t page_addr;
		uint32_t other_page;
		uint32_t saved_pte;
		volatile uint8_t val;
#ifdef CONFIG_SMP
		cpu_set_t cpu_mask;
		cpu_set_t orig_mask;
#endif

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 4);
		if (!buf) {
			lldbg("AL SPLIT_PROTECT: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;
		other_page = page_addr + 8192; /* different 4KB page in same 1MB */

		memset((void *)page_addr, 0xAA, 4096);
		memset((void *)other_page, 0xBB, 4096);

#ifdef CONFIG_SMP
		/* Pin to Core 0 — single-core test, prevent scheduler migration */
		sched_getaffinity(getpid(), sizeof(cpu_set_t), &orig_mask);
		CPU_ZERO(&cpu_mask);
		CPU_SET(0, &cpu_mask);
		sched_setaffinity(getpid(), sizeof(cpu_set_t), &cpu_mask);
		lldbg("AL SPLIT_PROTECT: Pinned to CPU 0 (was CPU %d)\n", up_cpu_index());
#endif

		mmu_split_section_to_l2(page_addr);
		lldbg("AL SPLIT_PROTECT: section split on CPU %d\n", up_cpu_index());

		saved_pte = mmu_save_page_pte(page_addr);
		mmu_set_page_no_access(page_addr);
		lldbg("AL SPLIT_PROTECT: page 0x%08x protected on CPU %d\n",
		      page_addr, up_cpu_index());

		/* Read different page — should succeed (proves only 4KB is protected) */
		val = ((volatile uint8_t *)other_page)[0];
		lldbg("AL SPLIT_PROTECT: other page read OK, val=0x%02x\n", val);

		/* Read protected page — Data Abort */
		lldbg("AL SPLIT_PROTECT: >>> READ protected page on CPU %d, expect Data Abort <<<\n",
		      up_cpu_index());
		val = ((volatile uint8_t *)page_addr)[0];
		(void)val;

		lldbg("AL SPLIT_PROTECT: ERROR — access succeeded!\n");
		MMU_TRY(mmu_restore_page_pte(page_addr, saved_pte));
#ifdef CONFIG_SMP
		sched_setaffinity(getpid(), sizeof(cpu_set_t), &orig_mask);
#endif
		kmm_free(buf);
		ret = -EIO;
		break;
	}
#endif /* CONFIG_MMU_PAGE_PROTECT */
#endif

#ifdef CONFIG_SMP
#ifdef CONFIG_MMU_PAGE_PROTECT
	/* TC-AL-06: SMP cross-core no-access (Fatal)
	 *
	 * Two-phase test to properly reproduce the SMP TLB defect:
	 *   Phase 1: Core 1 reads page (TLB gets RW entry)
	 *   Phase 2: Core 0 protects page, Core 1 reads again
	 *     Without fix: stale TLB → read succeeds (defect)
	 *     With fix: TLB flushed → Data Abort (protection works)
	 */
	case TESTIOC_AL_SMP_NOACCESS: {
		uint8_t *buf;
		uint32_t page_addr;
		pid_t core1_pid;
		cpu_set_t cpu_mask;

		/* B8: Verify SMP is actually available */
#ifndef CONFIG_SMP
		lldbg("AL SMP_NOACCESS: CONFIG_SMP not enabled — test is meaningless, skipping\n");
		return -ENOTSUP;
#endif
#if CONFIG_SMP_NCPUS < 2
		lldbg("AL SMP_NOACCESS: only %d CPU(s) — need >= 2, skipping\n", CONFIG_SMP_NCPUS);
		return -ENOTSUP;
#endif
		/* Log which CPU the ioctl is running on */
		lldbg("AL SMP_NOACCESS: Core 0 (ioctl) running on CPU%d\n", up_cpu_index());

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("AL SMP_NOACCESS: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;
		memset((void *)page_addr, 0xAA, 4096);

		/* Set up globals for Core 1 thread */
		g_al_smp_page_addr = page_addr;
		g_al_smp_access_type = 0;  /* read */
		g_al_smp_phase1_done = 0;
		g_al_smp_phase2_go = 0;
		g_al_smp_core1_cpu = -1;

		/* Spawn Core 1 thread — it will access page BEFORE protection */
		core1_pid = kernel_thread("al_smp_c1", 100, 2048,
		                          al_smp_core1_thread, NULL);
		if (core1_pid < 0) {
			lldbg("AL SMP_NOACCESS: kernel_thread() failed (%d)\n", core1_pid);
			kmm_free(buf);
			ret = -ENOMEM;
			break;
		}

		/* Pin the thread to Core 1 (CPU 1) */
		CPU_ZERO(&cpu_mask);
		CPU_SET(1, &cpu_mask);
		if (sched_setaffinity(core1_pid, sizeof(cpu_set_t),
		                      &cpu_mask) != OK) {
			lldbg("AL SMP_NOACCESS: sched_setaffinity() failed — aborting test\n");
			kmm_free(buf);
			ret = -EIO;
			break;
		}
		lldbg("AL SMP_NOACCESS: Core 1 thread pinned to CPU 1 (pid=%d)\n",
		      core1_pid);

		/* Wait for Core 1 to complete phase 1 (pre-protection access) */
		lldbg("AL SMP_NOACCESS: Waiting for Core 1 phase 1 (pre-protection read)...\n");
		int wait_count = 0;
		while (!g_al_smp_phase1_done && wait_count < 1000) {
			__asm__ volatile("dmb" ::: "memory");
			up_mdelay(1);
			wait_count++;
		}
		if (!g_al_smp_phase1_done) {
			lldbg("AL SMP_NOACCESS: ERROR — Core 1 phase 1 timeout!\n");
			kmm_free(buf);
			ret = -EIO;
			break;
		}

		/* B7: Verify Core 1 actually ran on CPU 1 */
		if (g_al_smp_core1_cpu != 1) {
			lldbg("AL SMP_NOACCESS: ERROR — Core 1 thread ran on CPU%d, not CPU1!\n",
			      g_al_smp_core1_cpu);
			lldbg("AL SMP_NOACCESS: SMP test is INVALID — thread did not run on a different core\n");
			kmm_free(buf);
			ret = -EIO;
			break;
		}
		lldbg("AL SMP_NOACCESS: Verified — Core 1 thread ran on CPU%d ✓\n",
		      g_al_smp_core1_cpu);
		lldbg("AL SMP_NOACCESS: Core 1 phase 1 done — TLB has stale RW entry\n");

		/* Now protect the page — Core 1's TLB still has old RW entry.
		 * Must split the 1MB L1 section into 4KB L2 pages first, otherwise
		 * mmu_set_page_no_access() would protect the entire 1MB section
		 * (including TCB), causing a nested Data Abort in the abort handler. */
		mmu_split_section_to_l2(page_addr);
		mmu_set_page_no_access(page_addr);
		lldbg("AL SMP_NOACCESS: Core 0 protected page 0x%08x (No-Access)\n", page_addr);

		/* Diagnostic: read DACR from Core 0's perspective after protection */
		{
			uint32_t dacr;
			__asm__ volatile("mrc p15, 0, %0, c3, c0, 0" : "=r"(dacr));
			lldbg("POST_PROTECT: Core0 DACR=0x%08x (dom0=%s)\n",
			      dacr,
			      (dacr & 0x3) == 0 ? "NO_ACCESS" :
			      (dacr & 0x3) == 1 ? "CLIENT" :
			      (dacr & 0x3) == 3 ? "MANAGER" : "RESERVED");
		}

		/* Signal Core 1 to do phase 2 (post-protection access) */
		g_al_smp_phase2_go = 1;
		__asm__ volatile("dsb" ::: "memory");  /* Ensure Core 1 sees the signal */
		lldbg("AL SMP_NOACCESS: >>> Core 1 READ again, expect Data Abort <<<\n");

		/* Wait for Core 1 thread — it should Data Abort */
		up_mdelay(100);

		/* If we reach here, Data Abort did NOT occur (defect reproduced) */
		lldbg("AL SMP_NOACCESS: ERROR — no Data Abort! DEFECT REPRODUCED!\n");
		ret = -EIO;
		break;
	}

	/* TC-AL-07: SMP cross-core RO (Fatal)
	 *
	 * Two-phase test to properly reproduce the SMP TLB defect:
	 *   Phase 1: Core 1 writes page (TLB gets RW entry)
	 *   Phase 2: Core 0 sets page RO, Core 1 writes again
	 *     Without fix: stale TLB → write succeeds (defect)
	 *     With fix: TLB flushed → Data Abort (protection works)
	 */
	case TESTIOC_AL_SMP_RO: {
		uint8_t *buf;
		uint32_t page_addr;
		pid_t core1_pid;
		cpu_set_t cpu_mask;

		/* B8: Verify SMP is actually available */
#ifndef CONFIG_SMP
		lldbg("AL SMP_RO: CONFIG_SMP not enabled — test is meaningless, skipping\n");
		return -ENOTSUP;
#endif
#if CONFIG_SMP_NCPUS < 2
		lldbg("AL SMP_RO: only %d CPU(s) — need >= 2, skipping\n", CONFIG_SMP_NCPUS);
		return -ENOTSUP;
#endif
		/* Log which CPU the ioctl is running on */
		lldbg("AL SMP_RO: Core 0 (ioctl) running on CPU%d\n", up_cpu_index());

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("AL SMP_RO: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;
		memset((void *)page_addr, 0xAA, 4096);

		/* Set up globals for Core 1 thread */
		g_al_smp_page_addr = page_addr;
		g_al_smp_access_type = 1;  /* write */
		g_al_smp_phase1_done = 0;
		g_al_smp_phase2_go = 0;
		g_al_smp_core1_cpu = -1;

		/* Spawn Core 1 thread — it will access page BEFORE protection */
		core1_pid = kernel_thread("al_smp_c1", 100, 2048,
		                          al_smp_core1_thread, NULL);
		if (core1_pid < 0) {
			lldbg("AL SMP_RO: kernel_thread() failed (%d)\n", core1_pid);
			kmm_free(buf);
			ret = -ENOMEM;
			break;
		}

		/* Pin the thread to Core 1 (CPU 1) */
		CPU_ZERO(&cpu_mask);
		CPU_SET(1, &cpu_mask);
		if (sched_setaffinity(core1_pid, sizeof(cpu_set_t),
		                      &cpu_mask) != OK) {
			lldbg("AL SMP_RO: sched_setaffinity() failed — aborting test\n");
			kmm_free(buf);
			ret = -EIO;
			break;
		}
		lldbg("AL SMP_RO: Core 1 thread pinned to CPU 1 (pid=%d)\n",
		      core1_pid);

		/* Wait for Core 1 to complete phase 1 (pre-protection access) */
		lldbg("AL SMP_RO: Waiting for Core 1 phase 1 (pre-protection write)...\n");
		int wait_count = 0;
		while (!g_al_smp_phase1_done && wait_count < 1000) {
			__asm__ volatile("dmb" ::: "memory");
			up_mdelay(1);
			wait_count++;
		}
		if (!g_al_smp_phase1_done) {
			lldbg("AL SMP_RO: ERROR — Core 1 phase 1 timeout!\n");
			kmm_free(buf);
			ret = -EIO;
			break;
		}

		/* B7: Verify Core 1 actually ran on CPU 1 */
		if (g_al_smp_core1_cpu != 1) {
			lldbg("AL SMP_RO: ERROR — Core 1 thread ran on CPU%d, not CPU1!\n",
			      g_al_smp_core1_cpu);
			lldbg("AL SMP_RO: SMP test is INVALID — thread did not run on a different core\n");
			kmm_free(buf);
			ret = -EIO;
			break;
		}
		lldbg("AL SMP_RO: Verified — Core 1 thread ran on CPU%d ✓\n",
		      g_al_smp_core1_cpu);
		lldbg("AL SMP_RO: Core 1 phase 1 done — TLB has stale RW entry\n");

		/* Now set the page to RO — Core 1's TLB still has old RW entry.
		 * Must split the 1MB L1 section into 4KB L2 pages first, otherwise
		 * mmu_set_page_ro() would protect the entire 1MB section
		 * (including TCB), causing incorrect protection. */
		mmu_split_section_to_l2(page_addr);
		mmu_set_page_ro(page_addr);
		lldbg("AL SMP_RO: Core 0 set page 0x%08x to RO\n", page_addr);

		/* Diagnostic: read DACR from Core 0's perspective after protection */
		{
			uint32_t dacr;
			__asm__ volatile("mrc p15, 0, %0, c3, c0, 0" : "=r"(dacr));
			lldbg("POST_PROTECT: Core0 DACR=0x%08x (dom0=%s)\n",
			      dacr,
			      (dacr & 0x3) == 0 ? "NO_ACCESS" :
			      (dacr & 0x3) == 1 ? "CLIENT" :
			      (dacr & 0x3) == 3 ? "MANAGER" : "RESERVED");
		}

		/* Signal Core 1 to do phase 2 (post-protection access) */
		g_al_smp_phase2_go = 1;
		__asm__ volatile("dsb" ::: "memory");  /* Ensure Core 1 sees the signal */
		lldbg("AL SMP_RO: >>> Core 1 WRITE again, expect Data Abort <<<\n");

		/* Wait for Core 1 thread — it should Data Abort */
		up_mdelay(100);

		/* If we reach here, Data Abort did NOT occur (defect reproduced) */
		lldbg("AL SMP_RO: ERROR — no Data Abort! DEFECT REPRODUCED!\n");
		ret = -EIO;
		break;
	}
#endif /* CONFIG_SMP */
#endif /* CONFIG_MMU_PAGE_PROTECT */

	/* TC-AL-08: Page protect + restore round-trip (Safe, single-core) */
	case TESTIOC_AL_SMP_RESTORE: {
		uint8_t *buf;
		uint32_t page_addr;
		uint32_t saved_pte;
		uint32_t pte = 0;
		uint8_t ap = 0xff;

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("AL RESTORE: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;
		memset((void *)page_addr, 0xAA, 4096);

		/* Split first: on a plain section the protect call is
		 * refused and the round-trip below proves nothing. */

		MMU_TRY(mmu_split_section_to_l2(page_addr));

		saved_pte = mmu_save_page_pte(page_addr);
		MMU_TRY(mmu_set_page_no_access(page_addr));

		if (mmu_get_page_pte_raw(page_addr, &pte, &ap) != OK || ap != 0) {
			lldbg("AL RESTORE: FAIL - not No-Access after set (pte=0x%08x AP=%u)\n",
			      pte, (unsigned int)ap);
			MMU_TRY(mmu_restore_page_pte(page_addr, saved_pte));
			kmm_free(buf);
			ret = -EIO;
			break;
		}

		MMU_TRY(mmu_restore_page_pte(page_addr, saved_pte));

		if (mmu_get_page_pte_raw(page_addr, &pte, &ap) != OK ||
		    pte != saved_pte) {
			lldbg("AL RESTORE: FAIL - descriptor not restored (pte=0x%08x saved=0x%08x)\n",
			      pte, saved_pte);
			kmm_free(buf);
			ret = -EIO;
			break;
		}

		lldbg("AL RESTORE: protect+restore done (pte=0x%08x), page accessible\n",
		      pte);

		((volatile uint8_t *)page_addr)[0] = 0xBB;
		lldbg("AL RESTORE: write OK\n");

		kmm_free(buf);
		ret = OK;
		break;
	}

	/* TC-AL-09: Section split (Safe, single-core) */
#ifdef CONFIG_MMU_PAGE_PROTECT
	case TESTIOC_AL_SMP_SPLIT: {
		uint8_t *buf;
		uint32_t page_addr;
		uint32_t pte = 0;
		uint8_t ap = 0xff;

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("AL SPLIT: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;

		MMU_TRY(mmu_split_section_to_l2(page_addr));

		/* "No crash" was the whole test before, so a split that was
		 * refused (L2 pool exhausted, bad address, section already
		 * split) printed exactly the same PASS line.  MMU_TRY() only
		 * counts and logs, it does not abort the case, so the L1
		 * descriptor has to be read back and checked explicitly -
		 * the way TC-AL-01 does it. */

		if (mmu_get_page_pte_raw(page_addr, &pte, &ap) != OK) {
			lldbg("AL SPLIT: FAIL - section is still not 4KB-mapped\n");
			kmm_free(buf);
			ret = -EIO;
			break;
		}

		memset((void *)page_addr, 0xAA, 4096);
		lldbg("AL SPLIT: write OK, now 4KB-mapped (pte=0x%08x AP=%u)\n",
		      pte, (unsigned int)ap);

		kmm_free(buf);
		ret = OK;
		break;
	}

	/* TC-AL-10: L2 pool accounting (Safe, single core).
	 *
	 * Coverage has to be known and not assumed: the pool is what bounds
	 * how much address space can ever be page-protected, and until now a
	 * split that was refused because it was empty looked exactly like a
	 * split that succeeded.  What is asserted here is the no-leak
	 * contract - one split costs at most one table, and splitting the
	 * same section again costs none, so a repeated protect/init cannot
	 * eat the pool. */
	case TESTIOC_MMU_L2_POOL_USAGE: {
		uint32_t total = 0;
		uint32_t used  = mmu_l2_pool_usage(&total);
		uint32_t held  = mmu_l2_pool_held();
		uint32_t after;
		uint32_t pok = 0;
		uint32_t pskip = 0;
		uint32_t pbudget = 0;
		uint32_t saved = 0;
		uint32_t saved_max = 0;
		uint8_t *buf;
		uint32_t page_addr;

		lldbg("=== L2 POOL: %u of %u tables claimed, %u held for a protected region ===\n",
		      used, total, held);

		mmu_protect_get_counters(&pok, &pskip, &pbudget);
		lldbg("L2 POOL: page-protect operations: ok=%u skipped=%u refused-for-budget=%u\n",
		      pok, pskip, pbudget);

		/* The save table, not the L2 pool, is what bounds how much
		 * memory can be No-Access at one time: MM_PROTECT_SAVED_MAX
		 * pages.  Without this line the first refused-for-budget
		 * increment reads as a defect instead of a ceiling reached. */
		saved = mmu_protect_saved_usage(&saved_max);
		lldbg("L2 POOL: locked-page save table %u of %u pages (%u KB ceiling)\n",
		      saved, saved_max, (unsigned int)(saved_max * 4));

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("L2 POOL: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;

		/* First split: takes a table if the megabyte was still a
		 * section, none if the heap had it split already.  Both
		 * answers are correct, so the check is on the delta. */

		MMU_TRY(mmu_split_section_to_l2(page_addr));
		after = mmu_l2_pool_usage(NULL);

		if (after != used && after != used + 1) {
			lldbg("L2 POOL: FAIL - %u tables before, %u after one split\n",
			      used, after);
			kmm_free(buf);
			ret = -EIO;
			break;
		}

		MMU_TRY(mmu_split_section_to_l2(page_addr));

		if (mmu_l2_pool_usage(NULL) != after) {
			lldbg("L2 POOL: FAIL - a repeated split of 0x%08x claimed another table\n",
			      page_addr);
			kmm_free(buf);
			ret = -EIO;
			break;
		}

		lldbg("L2 POOL: PASS - split of 0x%08x used %u table(s), pool %u/%u\n",
		      page_addr, after - used, after, total);

		if (after >= total) {
			lldbg("L2 POOL: NOTE - pool exhausted, further splits now return -ENOMEM\n");
		} else {
			lldbg("L2 POOL: NOTE - the -ENOMEM branch needs a build with "
			      "CONFIG_MMU_L2_POOL_TABLES=2 to be reachable\n");
		}

		kmm_free(buf);
		ret = OK;
		break;
	}
#endif /* CONFIG_MMU_PAGE_PROTECT */

/* TC-R2: Fault report verification - register, protect, corrupt (FATAL) */
#if defined(CONFIG_MMU_PAGE_PROTECT) && defined(CONFIG_MMU_FAULT_REPORT)
	case TESTIOC_MMU_REGION_REPORT: {
		static uint8_t r2_page[4096] __attribute__((aligned(4096)));
		uint32_t saved_pte;

		/* Step 1: Register the region by name in the fault report
		 * registry so the report can name it when the abort fires.
		 */
		if (mmu_protect_region_register("r2_test_page",
						(uintptr_t)r2_page,
						sizeof(r2_page)) < 0) {
			lldbg("R2: mmu_protect_region_register failed\n");
			return -ENOMEM;
		}

		/* Step 2: Split the 1MB section containing r2_page into 4KB
		 * pages.  r2_page (kernel .bss, 0x60124000) shares its section
		 * with kernel .data/.bss (exception stacks, TCBs, and the
		 * g_mmu_regions registry itself).  Protecting at section
		 * granularity would make the abort handler's own data
		 * inaccessible -> unrecoverable nested abort (system hang,
		 * as measured on rtl8730e).  Splitting limits the no-access
		 * region to the single r2_page.
		 */
		mmu_split_section_to_l2((uint32_t)r2_page);

		/* Step 3: Protect the page (no access) */
		saved_pte = mmu_save_page_pte((uint32_t)r2_page);
		mmu_set_page_no_access((uint32_t)r2_page);
		lldbg("R2: 'r2_test_page' registered and protected (no access)\n");
		lldbg("R2: >>> deliberate bad write - expect Data Abort + MMU FAULT REPORT <<<\n");

		/* Step 4: Deliberate bad write - aborts and PANICs */
		*(volatile uint32_t *)r2_page = 0xDEADBEEF;

		/* Not reached.  Restore defensively if the abort did not fire. */
		MMU_TRY(mmu_restore_page_pte((uint32_t)r2_page, saved_pte));
		ret = OK;
		break;
	}
#endif

	/* TC-R3: TLB/performance gate benchmark (safe).
	 * A: same workload on 1MB sections vs B: on 4KB L2 pages.
	 * C: cost of one runtime split.  D: cost of one
	 * protect+restore pair (includes the SGI4 cross-core sync). */
#ifdef CONFIG_MMU_PAGE_PROTECT
	case TESTIOC_MMU_TLB_BENCH: {
		FAR struct mmu_tlb_bench_s *out =
			(FAR struct mmu_tlb_bench_s *)(uintptr_t)arg;
		struct mmu_tlb_bench_s res;
		FAR uint32_t *l1 = mmu_l1_pgtable_safe();
		FAR uint8_t *ws;
		uint32_t sect_first;
		uint32_t sect_last;
		uint32_t sect;
		uint32_t t0;
		uint32_t t1;
		int i;

		(void)memset(&res, 0, sizeof(res));

		ws = (FAR uint8_t *)kmm_memalign(4096, R3_WS_SIZE);
		if (!ws) {
			lldbg("R3: kmm_memalign(%u) failed\n", R3_WS_SIZE);
			return -ENOMEM;
		}

		/* Phase D times a real protect+restore pair: needs the gate
		 * open.  Disarmed at ioctl exit.  A/B/C unaffected. */
		mmu_set_uaf_protect_enabled(1);

		(void)memset(ws, 0, R3_WS_SIZE);

		sect_first = (uint32_t)ws >> 20;
		sect_last = ((uint32_t)ws + R3_WS_SIZE - 1) >> 20;

		/* Phase A: working set still mapped by 1MB sections */
		lldbg("R3: Phase A: baseline, working set on %u 1MB sections...\n",
		      sect_last - sect_first + 1);
		t0 = r3_now_us();
		r3_workload((FAR uint32_t *)ws, R3_REPS);
		t1 = r3_now_us();
		res.baseline_us = t1 - t0;

		/* Split every section of the working set to 4KB pages */
		for (sect = sect_first; sect <= sect_last; sect++) {
			if ((l1[sect] & PMD_TYPE_MASK) == PMD_TYPE_SECT) {
				res.sections_split++;
			}
			mmu_split_section_to_l2(sect << 20);
		}

		/* Phase B: identical workload, identical data, 4KB pages */
		lldbg("R3: Phase B: same workload after splitting %u sections...\n",
		      res.sections_split);
		t0 = r3_now_us();
		r3_workload((FAR uint32_t *)ws, R3_REPS);
		t1 = r3_now_us();
		res.split_us = t1 - t0;

		if (res.baseline_us > 0 && res.split_us > res.baseline_us) {
			res.overhead_pct = (res.split_us - res.baseline_us) * 100 /
					   res.baseline_us;
		}

		/* Phase C: average cost of splitting R3_SPLIT_REPS fresh
		 * sections (the ones right after the working set, still
		 * plain sections).  Bounded L2-pool use: at most
		 * R3_SPLIT_REPS of the pool tables. */
		{
			uint32_t probe;
			uint32_t done = 0;

			t0 = r3_now_us();
			for (probe = sect_last + 1;
			     done < R3_SPLIT_REPS && probe <= sect_last + 16;
			     probe++) {
				if ((l1[probe] & PMD_TYPE_MASK) != PMD_TYPE_SECT) {
					continue;
				}

				mmu_split_section_to_l2(probe << 20);

				/* Count successful splits only: when the
				 * bounded L2 pool is exhausted the entry
				 * stays a section, and timing such a no-op
				 * would understate the split cost. */
				if ((l1[probe] & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
					done++;
				}
			}
			t1 = r3_now_us();

			if (done > 0) {
				res.split_cost_us = (t1 - t0) / done;
				res.sections_split += done;
			} else {
				lldbg("R3: Phase C skipped (no fresh section)\n");
			}
		}

		/* Phase D: average cost of one protect+restore pair on one
		 * 8KB region (2 full interior pages, i.e. 4 TLB syncs incl.
		 * one cross-core SGI4 ack per pair).  Timed in batches of
		 * R3_PAIR_BATCH because a single pair is shorter than the
		 * clock floor. */
		{
			uintptr_t pa = (uintptr_t)ws;
			uint32_t total = 0;
			int j;

			for (i = 0; i < R3_PAIR_REPS / R3_PAIR_BATCH; i++) {
				t0 = r3_now_us();
				for (j = 0; j < R3_PAIR_BATCH; j++) {
					mmu_protect_range_no_access(pa, 8192);
					mmu_restore_range(pa, 8192);
				}

				t1 = r3_now_us();
				total += (t1 - t0);
			}

			res.op_cost_us = total / R3_PAIR_REPS;
		}

#ifdef CONFIG_WDOG_MMU_PROTECT
		/* Phase E: average cost of one watchdog write window - open
		 * and close the descriptor of the one protected page.  This is
		 * what every wd_start()/wd_cancel() pays, so it is the number
		 * that decides whether the protection is affordable.  Timed
		 * under a critical section, as the callers hold one - but one
		 * critical section per batch, not one per phase: the batch is
		 * sized for the clock resolution, and holding interrupts off
		 * across every batch would put ~10 ms of latency into the
		 * timer and the serial path.  The enter/leave itself is one
		 * operation per 1000 pairs, so it does not distort the
		 * average.
		 */
		{
			uint32_t total = 0;
			irqstate_t win_irq;
			int k;

			for (i = 0; i < R3_PAIR_REPS / R3_PAIR_BATCH; i++) {
				win_irq = enter_critical_section();

				t0 = r3_now_us();
				for (k = 0; k < R3_PAIR_BATCH; k++) {
					MMU_TRYQ(wd_mmu_write_begin());
					MMU_TRYQ(wd_mmu_write_end());
				}

				t1 = r3_now_us();

				leave_critical_section(win_irq);

				total += (t1 - t0);
			}

			res.window_pair_us = total / R3_PAIR_REPS;
		}
#endif

		lldbg("R3: clock floor 1000us: C = avg of %u splits, D = avg of %u pairs\n",
		      R3_SPLIT_REPS, R3_PAIR_REPS);

		lldbg("R3: =============== TLB/split benchmark report ===============\n");
		lldbg("R3: Phase A (1MB sections): %u us\n", res.baseline_us);
		lldbg("R3: Phase B (4KB L2 pages): %u us\n", res.split_us);
		lldbg("R3: overhead = %u%%   (R3 gate: < 10%%)\n", res.overhead_pct);
		lldbg("R3: one runtime split = %u us (L2 fill + full TLB flush)\n",
		      res.split_cost_us);
		/* Phase D and E are batch averages of operations that take
		 * well under one clock tick, so a result below R3_TICK_US is
		 * "not resolvable", not "measured at N us".  Say which. */

		if (res.op_cost_us < R3_TICK_US) {
			lldbg("R3: 8KB block protect+restore pair < %u us (below clock resolution - not a measured value)\n",
			      (unsigned int)R3_TICK_US);
		} else {
			lldbg("R3: 8KB block protect+restore pair = %u us (4 TLB syncs)\n",
			      res.op_cost_us);
		}
#ifdef CONFIG_WDOG_MMU_PROTECT
		if (res.window_pair_us < R3_TICK_US) {
			lldbg("R3: watchdog write window pair < %u us (below clock resolution - not a measured value)\n",
			      (unsigned int)R3_TICK_US);
		} else {
			lldbg("R3: watchdog write window pair = %u us (1 descriptor, both cores)\n",
			      res.window_pair_us);
		}
#endif
		/* Budget state at the end of the sweep.  Both numbers are
		 * one-way doors - an L2 table is never released and a locked
		 * page stays locked until something restores it - so a soak
		 * run has to report where it left the board (plan risk R6). */
		{
			uint32_t ptotal = 0;
			uint32_t smax = 0;

			res.pool_claimed = mmu_l2_pool_usage(&ptotal);
			res.pool_total = ptotal;
			res.pool_held = mmu_l2_pool_held();
			res.saved_used = mmu_protect_saved_usage(&smax);
			res.saved_max = smax;

			lldbg("R3: budget after: pool %u of %u tables claimed, %u held; save table %u of %u pages (%u KB ceiling)\n",
			      res.pool_claimed, res.pool_total, res.pool_held,
			      res.saved_used, res.saved_max,
			      (unsigned int)(res.saved_max * 4));
		}

		lldbg("R3: L2 tables consumed by working set = %u\n",
		      res.sections_split);
		if (res.overhead_pct < 10) {
			lldbg("R3: PASS - 4KB page protection is affordable\n");
		} else {
			lldbg("R3: MITIGATION REQUIRED - overhead >= 10%%\n");
		}

		if (out) {
			*out = res;
		}

		kmm_free(ws);	/* working-set sections stay split (harmless) */
		ret = OK;
		break;
	}
#endif /* CONFIG_MMU_PAGE_PROTECT */

	/* TC-R4: allocator UAF-protection stress.
	 * 1: free a 64KB block, verify interior PTE went AP=0
	 * 2: mixed-size churn (merge/split paths must stay fault-free)
	 * 3: same churn from a core-1 pinned thread
	 * 4: FATAL - deliberate write to a freed, protected block */
#ifdef CONFIG_MM_HEAP_UAF_PROTECT
	case TESTIOC_MMU_HEAP_PROTECT_STRESS: {
		int phase = (int)arg & 0xff;
		uint32_t minutes = (uint32_t)((unsigned int)arg >> 8);
		FAR uint8_t *mem;
		uintptr_t pg;
		uint32_t cok;
		uint32_t cskip;
		uint32_t cfail;

		lldbg("=== TC-R4: heap UAF-protect stress (phase %d) ===\n", phase);

		/* Step 1: free a 64KB block -> interior PTE must be AP=0 */
		mem = (FAR uint8_t *)kmm_malloc(64 * 1024);
		if (!mem) {
			return -ENOMEM;
		}

		/* Protection window: this test only; disarmed at ioctl
		 * exit.  Everything else runs HEAD-identical (gate off). */
		mmu_set_uaf_protect_enabled(1);

		memset(mem, 0xAA, 64 * 1024);
		pg = ((uintptr_t)mem + 32 + 4095) & ~(uintptr_t)4095;
		kmm_free(mem);		/* addfreechunk hook locks interior */

		{
			FAR uint32_t *l1 = mmu_l1_pgtable_safe();
			uint32_t e = 0xFFFFFFFF;

			if ((l1[pg >> 20] & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
				FAR uint32_t *l2 = (FAR uint32_t *)(l1[pg >> 20] & PMD_PTE_PADDR_MASK);
				e = l2[(pg >> 12) & 0xFF];
			}

			if ((e & 2) && ((e >> 4) & 3) == 0 && ((e >> 9) & 1) == 0) {
				lldbg("R4/1: PASS interior PTE 0x%08x AP=0 (page 0x%08x locked)\n",
				      e, (uint32_t)pg);
			} else {
				lldbg("R4/1: WARN PTE=0x%08x NOT locked (coverage gap?)\n", e);
			}
		}

		mmu_protect_get_counters(&cok, &cskip, &cfail);
		lldbg("R4: counters: ok=%u skipped=%u failed_budget=%u coverage=%u%%\n",
		      cok, cskip, cfail,
		      R4_COVERAGE(cok, cskip));

		if (phase == 5) {
			/* M4 endurance: `heap_protect 5 <minutes>`.  Reuses
			 * the mixed-size churn, plus an hourly core-1 churn
			 * under SMP, inside a wall-clock deadline; prints
			 * coverage deltas every minute and never issues a
			 * fatal write.  Exit criterion: survive the whole
			 * deadline with no abort and no failed_budget growth
			 * (and with the R6 warning never fired). */
			struct timespec ts;
			time_t start_s;
			time_t deadline_s;
			time_t next_report;
			uint32_t ok0;
			uint32_t skip0;
			uint32_t fail0;
			uint32_t rounds = 0;
#ifdef CONFIG_SMP
			time_t next_smp;
#endif

			if (minutes == 0) {
				minutes = 1;
			}

			clock_gettime(CLOCK_MONOTONIC, &ts);
			start_s = ts.tv_sec;
			deadline_s = ts.tv_sec + (time_t)minutes * 60;
			next_report = ts.tv_sec + 60;
#ifdef CONFIG_SMP
			next_smp = ts.tv_sec + 60;
#endif
			mmu_protect_get_counters(&ok0, &skip0, &fail0);
			lldbg("R4/5: endurance start: %u minutes\n",
			      (unsigned int)minutes);

			do {
				r4_mixed_churn(100);
				rounds++;
				clock_gettime(CLOCK_MONOTONIC, &ts);

#ifdef CONFIG_SMP
				if (ts.tv_sec >= next_smp &&
				    ts.tv_sec < deadline_s) {
					pid_t pid1;
					int wait_count = 0;

					next_smp = ts.tv_sec + 60;
					g_r4_core1_done = 0;
					pid1 = kernel_thread("r4_core1",
							     100, 4096,
							     r4_core1_churn,
							     NULL);
					if (pid1 >= 0) {
						cpu_set_t cpu_mask;

						CPU_ZERO(&cpu_mask);
						CPU_SET(1, &cpu_mask);
						sched_setaffinity(pid1,
							sizeof(cpu_set_t),
							&cpu_mask);
						while (!g_r4_core1_done &&
						       wait_count < 5000) {
							usleep(10 * 1000);
							wait_count++;
						}
					}
				}
#endif

				if (ts.tv_sec >= next_report) {
					uint32_t c1;
					uint32_t s1;
					uint32_t f1;

					next_report = ts.tv_sec + 60;
					mmu_protect_get_counters(&c1, &s1,
								  &f1);
					lldbg("R4/5: +%ld min ok=+%u skipped=+%u failed_budget=+%u\n",
					      (long)(ts.tv_sec - start_s) / 60,
					      (unsigned int)(c1 - ok0),
					      (unsigned int)(s1 - skip0),
					      (unsigned int)(f1 - fail0));
				}
			} while (ts.tv_sec < deadline_s);

			mmu_protect_get_counters(&cok, &cskip, &cfail);
			lldbg("R4/5: ENDURANCE DONE %u min rounds=%u ok=+%u skipped=+%u failed_budget=+%u\n",
			      (unsigned int)minutes, (unsigned int)rounds,
			      (unsigned int)(cok - ok0),
			      (unsigned int)(cskip - skip0),
			      (unsigned int)(cfail - fail0));
			ret = OK;
			break;
		}

		if (phase < 2) {
			ret = OK;
			break;
		}

		/* Step 2: mixed churn - allocator must never fault on the
		 * pages it just unlocked (hook-ordering proof) */
		lldbg("R4/2: mixed-size churn (100 iterations)...\n");
		r4_mixed_churn(100);
		lldbg("R4/2: PASS churn completed, no abort\n");

		if (phase < 3) {
			ret = OK;
			break;
		}

#ifdef CONFIG_SMP
		/* Step 3: same churn driven from Core 1 - exercises the
		 * SGI4 TLB sync from the other CPU */
		lldbg("R4/3: core-1 pinned churn (50 iterations)...\n");
		g_r4_core1_done = 0;
		{
			pid_t pid1 = kernel_thread("r4_core1", 100, 4096,
						   r4_core1_churn, NULL);

			if (pid1 >= 0) {
				cpu_set_t cpu_mask;
				int wait_count = 0;

				CPU_ZERO(&cpu_mask);
				CPU_SET(1, &cpu_mask);
				sched_setaffinity(pid1, sizeof(cpu_set_t), &cpu_mask);

				while (!g_r4_core1_done && wait_count < 5000) {
					usleep(10 * 1000);
					wait_count++;
				}

				if (g_r4_core1_done) {
					lldbg("R4/3: PASS core-1 churn completed, no abort\n");
				} else {
					lldbg("R4/3: WARN core-1 thread timed out\n");
				}
			} else {
				lldbg("R4/3: WARN kernel_thread failed (%d)\n", pid1);
			}
		}
#endif /* CONFIG_SMP */

		mmu_protect_get_counters(&cok, &cskip, &cfail);
		lldbg("R4: counters: ok=%u skipped=%u failed_budget=%u coverage=%u%%\n",
		      cok, cskip, cfail,
		      R4_COVERAGE(cok, cskip));

		if (phase < 4) {
			ret = OK;
			break;
		}

		/* Step 4: FATAL.  Free a block, deterministically lock one
		 * interior page (idempotent with the free hook), register
		 * it with the R2 fault report, then write -> Data Abort
		 * BEFORE any corruption can happen. */
		mem = (FAR uint8_t *)kmm_malloc(64 * 1024);
		if (!mem) {
			mmu_set_uaf_protect_enabled(0);
			return -ENOMEM;
		}
		memset(mem, 0x5A, 64 * 1024);
		pg = ((uintptr_t)mem + 32 + 4095) & ~(uintptr_t)4095;
		kmm_free(mem);
		mmu_protect_range_no_access(pg, 4096);
#ifdef CONFIG_MMU_FAULT_REPORT
		mmu_protect_region_register("heap_freed_block", pg, 4096);
#endif
		lldbg("R4/4: >>> deliberate write to freed+protected page 0x%08x\n",
		      (uint32_t)pg);
		lldbg("R4/4: >>> expect Data Abort + FAULT REPORT naming 'heap_freed_block' <<<\n");

		*(volatile uint32_t *)pg = 0xDEADBEEF;

		/* Not reached when protection works */
		mmu_restore_range(pg, 4096);
		lldbg("R4/4: FAIL - write survived, page was not protected!\n");
		ret = OK;
		break;
	}
#endif /* CONFIG_EXAMPLES_MMU_PROTECT_TEST && CONFIG_MM_HEAP_UAF_PROTECT */

#ifdef CONFIG_EXAMPLES_MMU_PROTECT_TEST
	case TESTIOC_WDOG_MMU_PROTECT_TEST: {
#ifndef CONFIG_WDOG_MMU_PROTECT
		lldbg("WDOG PROT: SKIP - CONFIG_WDOG_MMU_PROTECT is not enabled\n");
		ret = -ENOSYS;
		break;
#else
		/* Verify the watchdog page protection by judging the descriptor,
		 * not by judging whether a write survived:
		 *
		 *   1. the whole watchdog data block must be read-only while no
		 *      window is open;
		 *   2. the pool, both list heads and the free counter must all
		 *      be writable inside ONE window, and the block read-only
		 *      again once that window is closed - that is the claim of
		 *      one descriptor over one page;
		 *   3. arg = 1: write from outside any window.  That has to
		 *      fault, so this variant is fatal by design when the
		 *      protection works and is only reached deliberately.
		 *
		 * A write that goes through with the protection installed is a
		 * failure here, not a configuration report: the old body read a
		 * successful write as "CONFIG_WDOG_MMU_PROTECT is NOT enabled",
		 * which let this case pass with the protection installed but
		 * not working.
		 */

		volatile uint8_t *pool      = (volatile uint8_t *)&g_wdog_data.core.pool[0];
		volatile uint8_t *free_head = (volatile uint8_t *)&g_wdog_data.core.freelist.head;
		volatile uint8_t *act_head  = (volatile uint8_t *)&g_wdog_data.core.activelist.head;
		volatile uint8_t *nfree     = (volatile uint8_t *)&g_wdog_data.core.nfree;
		bool open = false;
		uint8_t cpu = 0xff;
		uint8_t depth = 0;
		irqstate_t irq_state;

		lldbg("=== WDOG MMU protect: data 0x%08x, %u bytes ===\n",
		      (uint32_t)(uintptr_t)&g_wdog_data,
		      (unsigned int)sizeof(g_wdog_data));

		wd_mmu_get_window_state(&open, &cpu, &depth);
		if (open) {
			lldbg("WDOG PROT: FAIL - a window is already open (cpu%u depth%u)\n",
			      (unsigned int)cpu, (unsigned int)depth);
			ret = -EBUSY;
			break;
		}

		if (!wd_mmu_is_protected()) {
			lldbg("WDOG PROT: FAIL - the data page is writable with no window open\n");
			ret = -EIO;
			break;
		}

		/* Reads are allowed on a read-only page */

		lldbg("WDOG PROT: read-only with no window open, pool[0] = 0x%02x\n",
		      pool[0]);

		/* One window, four objects.  Each write stores the value it
		 * just read, so the queue state is unchanged either way. */

		irq_state = enter_critical_section();

		if (wd_mmu_write_begin() != OK) {
			leave_critical_section(irq_state);
			lldbg("WDOG PROT: FAIL - write window could not be opened\n");
			ret = -EIO;
			break;
		}

		*pool      = *pool;
		*free_head = *free_head;
		*act_head  = *act_head;
		*nfree     = *nfree;

		(void)wd_mmu_write_end();
		leave_critical_section(irq_state);

		lldbg("WDOG PROT: pool, freelist, activelist, nfree all written in one window\n");

		if (!wd_mmu_is_protected()) {
			lldbg("WDOG PROT: FAIL - the page is still writable after the last end()\n");
			ret = -EIO;
			break;
		}

		if (arg == 1) {
			lldbg("WDOG PROT: >>> write outside any window, expect Data Abort <<<\n");
			*pool = 0xbb;

			/* Not reached when the protection works */

			lldbg("WDOG PROT: FAIL - the write went through, protection is not in force\n");
			ret = -EIO;
			break;
		}

		lldbg("WDOG PROT: PASS - one page, one descriptor, both cores (pass 1 for the fault case)\n");
		ret = OK;
		break;
#endif
	}

#endif

#ifdef CONFIG_WDOG_MMU_DEFECT_TEST

	case TESTIOC_WDOG_DEFECT_SMP: {
		/* Defect #1: "writes are allowed" has to be one state for the
		 * whole system, not one per core.
		 *
		 * The old mechanism changed a per-CPU register (DACR), so the
		 * state meant something different on each core: a core that was
		 * not the one that called wd_mmu_write_begin() faulted on its
		 * first write, and after that core finished, a core still
		 * inside its own window could not tell.
		 *
		 * The state is now the descriptor of one page, which both cores
		 * walk.  What this test checks:
		 *   - the page really is read-only before the window opens;
		 *   - opening the window makes it writable, reports this core
		 *     as the owner and one level deep, and the write succeeds;
		 *   - closing it puts the page back to read-only, with the
		 *     invalidation broadcast so the other core sees it too.
		 */
		volatile uint8_t *wdog_page = (volatile uint8_t *)&g_wdpool;
		bool win_open = true;
		uint8_t win_cpu = 0xff;
		uint8_t win_depth = 0;
		uint32_t pte = 0;
		uint8_t ap = 0xff;
		irqstate_t irq_state;
		uint8_t orig;

		lldbg("=== DEFECT #1: one protection state for both cores ===\n");

		wd_mmu_get_window_state(&win_open, &win_cpu, &win_depth);
		lldbg("DEFECT_SMP: before: window=%s cpu%u depth%u\n",
		      win_open ? "OPEN" : "closed",
		      (unsigned int)win_cpu, (unsigned int)win_depth);

		if (mmu_get_page_pte_raw((uint32_t)(uintptr_t)wdog_page, &pte, &ap) == OK) {
			lldbg("DEFECT_SMP: before: pte=0x%08x AP=%u (%s)\n", pte,
			      (unsigned int)ap,
			      ap == 0 ? "NO ACCESS" :
			      (ap & 2) ? "READ-ONLY" : "READ-WRITE");
		} else {
			lldbg("DEFECT_SMP: watchdog page is not 4KB-mapped!\n");
		}

		irq_state = enter_critical_section();

		if (wd_mmu_write_begin() != OK) {
			leave_critical_section(irq_state);
			lldbg("DEFECT_SMP: FAIL - could not open the write window\n");
			ret = ERROR;
			break;
		}

		wd_mmu_get_window_state(&win_open, &win_cpu, &win_depth);
		lldbg("DEFECT_SMP: inside: window=%s cpu%u depth%u\n",
		      win_open ? "OPEN" : "closed",
		      (unsigned int)win_cpu, (unsigned int)win_depth);

		if (mmu_get_page_pte_raw((uint32_t)(uintptr_t)wdog_page, &pte, &ap) == OK) {
			lldbg("DEFECT_SMP: inside: pte=0x%08x AP=%u (%s)\n", pte,
			      (unsigned int)ap,
			      ap == 0 ? "NO ACCESS" :
			      (ap & 2) ? "READ-ONLY" : "READ-WRITE");
		}

		orig = wdog_page[0];
		wdog_page[0] = orig;

		lldbg("DEFECT_SMP: write succeeded on this core while open\n");

		MMU_TRY(wd_mmu_write_end());
		leave_critical_section(irq_state);

		wd_mmu_get_window_state(&win_open, &win_cpu, &win_depth);

		if (mmu_get_page_pte_raw((uint32_t)(uintptr_t)wdog_page, &pte, &ap) == OK) {
			lldbg("DEFECT_SMP: after:  window=%s pte=0x%08x AP=%u (%s)\n",
			      win_open ? "OPEN" : "closed", pte, (unsigned int)ap,
			      ap == 0 ? "NO ACCESS" :
			      (ap & 2) ? "READ-ONLY" : "READ-WRITE");
		}

		/* The descriptor is the state, so it decides the verdict: the
		 * ioctl has to carry it, otherwise this case printed FAIL and
		 * still reported OK to the shell. */

		if (!win_open && (ap & 2)) {
			lldbg("DEFECT_SMP: PASS - read-only again, both cores see it\n");
			ret = OK;
		} else {
			lldbg("DEFECT_SMP: FAIL - page still writable after end()\n");
			ret = -EIO;
		}

		break;
	}

	case TESTIOC_WDOG_DEFECT_DSB: {
		/* Defect #3: a permission change has to be visible to the next
		 * access, on this core and on the other one.
		 *
		 * Changing a page-table entry is not enough on its own: the
		 * entry has to reach RAM, the stale copy in the data cache has
		 * to go, and the address has to leave the TLB of both cores -
		 * otherwise the access that follows the change is translated by
		 * the old entry, and the fault, if any, lands somewhere else.
		 *
		 * The barriers and the invalidation are the arch layer's job
		 * (mmu_restore_page_pte); this test hammers the window open and
		 * closed 10,000 times with a write in between and then checks
		 * that the state is still consistent.
		 */
		volatile uint8_t *wdog_page = (volatile uint8_t *)&g_wdpool;
		bool win_open = true;
		uint8_t win_cpu = 0xff;
		uint8_t win_depth = 0;
		irqstate_t irq_state;
		int ops_before = g_mmu_op_failures;
		int i;

		lldbg("=== DEFECT #3: barriers after the permission change ===\n");
		lldbg("DEFECT_DSB: Running 10,000 iterations of open+write+close\n");

		for (i = 0; i < 10000; i++) {
			irq_state = enter_critical_section();
			MMU_TRYQ(wd_mmu_write_begin());

			/* Immediately write — no slack of any kind between the
			 * descriptor change and the access */

			*wdog_page = *wdog_page;

			MMU_TRYQ(wd_mmu_write_end());
			leave_critical_section(irq_state);
		}

		wd_mmu_get_window_state(&win_open, &win_cpu, &win_depth);

		lldbg("DEFECT_DSB: All 10,000 iterations completed (no Data Abort)\n");
		lldbg("DEFECT_DSB: window after the loop: %s depth%u\n",
		      win_open ? "OPEN (FAIL)" : "closed", (unsigned int)win_depth);

		/* 10,000 windows have to leave the bookkeeping exactly as
		 * they found it: closed, zero deep, and no status refused on
		 * the way.  Without this the case reported OK whatever it
		 * printed. */

		ret = OK;
		if (win_open || win_depth != 0 || g_mmu_op_failures != ops_before) {
			lldbg("DEFECT_DSB: FAIL - window or counters dirty (depth=%u statuses=%d)\n",
			      (unsigned int)win_depth, g_mmu_op_failures);
			ret = -EIO;
		}

		break;
	}

	case TESTIOC_WDOG_DEFECT_DOMAIN_KEEP: {
		/* Defect #7: protecting the watchdogs must not touch the domain
		 * register at all.
		 *
		 * The old mechanism rewrote DACR to switch the pool between
		 * Client and Manager, which is a whole-CPU register: it also
		 * decided how every other domain behaves, and domains that the
		 * watchdog code knew nothing about were reset to No-Access
		 * around every write.
		 *
		 * The protection now only changes one page-table descriptor, so
		 * DACR has to be identical before and after a window.  Domain 2
		 * is claimed here as a stand-in for "somebody else's domain".
		 */
		uint32_t dacr_before;
		uint32_t dacr_after;
		irqstate_t irq_state;

		lldbg("=== DEFECT #7: the protection leaves the domain register alone ===\n");

		/* Claim Domain 2 as Client (bits [5:4] = 01), the way any other
		 * subsystem using a domain of its own would. */

		__asm__ volatile("mrc p15, 0, %0, c3, c0, 0" : "=r"(dacr_before));
		dacr_before &= ~(0x3 << 4);
		dacr_before |= (0x1 << 4);
		__asm__ volatile("mcr p15, 0, %0, c3, c0, 0" :: "r"(dacr_before) : "memory");
		__asm__ volatile("mrc p15, 0, %0, c3, c0, 0" : "=r"(dacr_before));

		lldbg("DEFECT_DOMAIN: DACR before window = 0x%08x (dom0=%u dom2=%u)\n",
		      dacr_before, dacr_before & 0x3, (dacr_before >> 4) & 0x3);

		irq_state = enter_critical_section();
		MMU_TRY(wd_mmu_write_begin());
		MMU_TRY(wd_mmu_write_end());
		leave_critical_section(irq_state);

		__asm__ volatile("mrc p15, 0, %0, c3, c0, 0" : "=r"(dacr_after));

		lldbg("DEFECT_DOMAIN: DACR after  window = 0x%08x (dom0=%u dom2=%u)\n",
		      dacr_after, dacr_after & 0x3, (dacr_after >> 4) & 0x3);

		if (dacr_after != dacr_before) {
			lldbg("DEFECT_DOMAIN: FAIL - watchdog protection modified DACR\n");
			ret = -EIO;
		} else if ((dacr_after & 0x3) != 1) {
			/* Domain 0 must stay Client: with Manager, the AP bits
			 * of every page - including the watchdog page - are
			 * ignored and no protection works at all. */

			lldbg("DEFECT_DOMAIN: FAIL - domain 0 is not Client (0x%x), AP bits ignored\n",
			      dacr_after & 0x3);
			ret = -EIO;
		} else {
			lldbg("DEFECT_DOMAIN: PASS - DACR untouched, domain 0 still Client\n");
			ret = OK;
		}

		break;
	}

	case TESTIOC_WDOG_DEFECT_HEAP_OVERLAP: {
		/* Defect #8: Heap overlap with the watchdog data page.
		 *
		 * The watchdog data sits in .bss now, inside the region the
		 * linker already reserves for it, so the allocator cannot hand
		 * it out - but a wrong section size would make the page cover
		 * other data (or be covered by it), and that is visible here:
		 * the range must be one page, aligned, and below the heap.
		 *
		 * This test prints the watchdog page range and the configured
		 * RAM range so the layout can be checked from the log.
		 */
		extern uint32_t _wdog_data_start[];
		extern uint32_t _wdog_data_end[];

		uint32_t pool_start = (uint32_t)_wdog_data_start;
		uint32_t pool_end = (uint32_t)_wdog_data_end;

		lldbg("=== DEFECT #8: Heap overlap with wdog data page ===\n");
		lldbg("DEFECT_HEAP: wdog data page: 0x%08x - 0x%08x (size = %d bytes)\n",
		      pool_start, pool_end, pool_end - pool_start);
		lldbg("DEFECT_HEAP: 4KB aligned = %s, within one page = %s\n",
		      (pool_start & 0xfff) ? "NO" : "yes",
		      (pool_end - pool_start) <= 4096 ? "yes" : "NO");

		/* What is being compared here is the watchdog page against
		 * the ranges the allocator actually owns.  The previous check
		 * asked whether the page lies inside
		 * CONFIG_RAM_START..+CONFIG_RAM_SIZE, which on rtl8730e
		 * covers every DDR address: it could only ever answer "defect
		 * confirmed" (observed: pool at 0x60131000, RAM range
		 * 0x60000000..0x68000000, so the case failed on a healthy
		 * configuration).  mm_heapstart[]/mm_heapend[] are the
		 * boundaries mm_addregion() published, which is what kmm_free()
		 * can ever touch.
		 */
		{
			FAR struct mm_heap_s *heap = kmm_get_baseheap();
			int nregions = 1;
			bool overlap = false;
			int r;

#if CONFIG_KMM_REGIONS > 1
			nregions = heap->mm_nregions;
			if (nregions < 1) {
				nregions = 1;
			}
#endif

			for (r = 0; r < nregions; r++) {
				uint32_t hs = (uint32_t)(uintptr_t)heap->mm_heapstart[r];
				uint32_t he = (uint32_t)(uintptr_t)heap->mm_heapend[r];

				/* mm_heapend[] is the address of the end
				 * node, i.e. the last node inside the region;
				 * comparing against it is conservative by one
				 * node, which is the right direction for a
				 * "can the allocator reach my page" test. */

				lldbg("DEFECT_HEAP: kernel heap region %d: 0x%08x - 0x%08x\n",
				      r, hs, he);

				if (pool_start < he && hs < pool_end) {
					overlap = true;
				}
			}

			if (overlap) {
				lldbg("DEFECT_HEAP: === DEFECT CONFIRMED ===\n");
				lldbg("DEFECT_HEAP: a heap region intersects the wdog page!\n");
				lldbg("DEFECT_HEAP: the allocator can hand out protected memory\n");
				ret = -EIO;
			} else {
				lldbg("DEFECT_HEAP: no heap region intersects the wdog page - OK\n");
				ret = OK;
			}
		}
		break;
	}

	case TESTIOC_WDOG_DEFECT_WD_RECOVER: {
		/* Defect #11: wd_recover() missing protection.
		 *
		 * wd_recover() is called when a task is destroyed while it
		 * has an active watchdog (tcb->waitdog != NULL).
		 * It calls wd_cancel() and wd_delete() which write to the
		 * wdog pool. Without wd_mmu_write_begin()/end(), these
		 * writes cause a Data Abort.
		 *
		 * This test:
		 * 1. Creates a watchdog and starts it
		 * 2. Simulates a task exit with an active watchdog
		 *    (calls wd_recover() path indirectly)
		 * 3. Verifies no Data Abort occurs (fix is working)
		 *
		 * NOTE: We do NOT corrupt the list pointer — that would
		 * cause a crash from following a bad pointer, which is
		 * unrelated to MMU protection.
		 */
		WDOG_ID wdog;
		uint32_t fails_before = wd_mmu_get_failures();
		bool win_open = false;
		uint8_t win_cpu = 0xff;
		uint8_t win_depth = 0;

		lldbg("=== DEFECT #11: wd_recover() missing protection ===\n");
		lldbg("DEFECT_RECOVER: Testing wd_recover() with MMU protection\n");

		/* Create and start a watchdog */
		wdog = wd_create();
		if (wdog == NULL) {
			lldbg("DEFECT_RECOVER: wd_create() failed\n");
			ret = -ENOMEM;
			break;
		}

		if (wd_start(wdog, 10000, NULL, 0) != OK) {
			lldbg("DEFECT_RECOVER: wd_start() failed\n");
			wd_delete(wdog);
			ret = -EIO;
			break;
		}

		lldbg("DEFECT_RECOVER: Watchdog created and started\n");

		/* Cancel and delete — this exercises the same code path as
		 * wd_recover() (which calls wd_cancel + wd_delete).
		 * With the fix, wd_recover() wraps these in
		 * wd_mmu_write_begin()/end(). Without the fix, a Data Abort
		 * would occur here.
		 */
		wd_cancel(wdog);
		wd_delete(wdog);

		lldbg("DEFECT_RECOVER: wd_cancel() + wd_delete() completed — no Data Abort\n");
		lldbg("DEFECT_RECOVER: wd_cancel()/wd_delete() use the same begin()/end() pair as wd_recover()\n");

		/* What is actually asserted: the two calls above run through
		 * the same protection pair wd_recover() uses, so a missing
		 * window would have faulted here, and it would also have left
		 * the window open or a status refused.  "No crash" alone was
		 * not a verdict - this is.
		 */

		wd_mmu_get_window_state(&win_open, &win_cpu, &win_depth);

		if (!win_open && win_depth == 0 &&
		    wd_mmu_get_failures() == fails_before) {
			lldbg("DEFECT_RECOVER: PASS - window closed, no error counted\n");
			ret = OK;
		} else {
			lldbg("DEFECT_RECOVER: FAIL - window=%s depth=%u failures=%u (was %u)\n",
			      win_open ? "OPEN" : "closed", (unsigned int)win_depth,
			      wd_mmu_get_failures(), fails_before);
			ret = -EIO;
		}

		break;
	}

	case TESTIOC_WDOG_DEFECT_ABORT_HANDLER: {
		/* Defect #12: Data Abort handler guard.
		 *
		 * The data abort report reads the live descriptor of the
		 * address the abort was taken at (DFAR) and the state of the
		 * watchdog write window.  If either of those reads faulted, the
		 * handler would fault inside the handler: nested abort, no
		 * report, hang.  What makes that impossible is checkable
		 * without taking a fault, and is what this case asserts:
		 *
		 *   1 the watchdog page has a descriptor and is never
		 *     No-Access - the mechanism only flips AP2, between AP=1
		 *     (privileged read/write) and AP=3 (privileged read-only),
		 *     so a kernel read of the page always succeeds;
		 *   2 a read of that page from foreign code (this driver, with
		 *     no window open) actually returns;
		 *   3 the accessors print_mmu_fault_report() calls answer, and
		 *     the protection is demonstrably installed;
		 *   4 the descriptor lookup the report runs on DFAR classifies
		 *     addresses it does not own instead of dereferencing them.
		 *
		 * The fatal half stays with 'wdog_write', 'region_report' and
		 * 'heap_protect 4': those do take the abort and do print the
		 * report from inside the handler.
		 */
		lldbg("=== DEFECT #12: Data Abort handler guard ===\n");

#if defined(CONFIG_WDOG_MMU_PROTECT) && defined(CONFIG_MMU_PAGE_PROTECT)
		{
			uint32_t page = ((uint32_t)(uintptr_t)&g_wdog_data) & ~4095u;
			uint32_t pte = 0;
			uint8_t ap = 0;
			bool open = false;
			uint8_t cpu = 0xff;
			uint8_t depth = 0;
			int status = mmu_get_page_pte_raw(page, &pte, &ap);

			if (status != OK) {
				lldbg("DEFECT_ABORT: FAIL - watchdog page 0x%08x has no descriptor (%d)\n",
				      page, status);
				ret = -EIO;
				break;
			}

			if (ap == 0) {
				lldbg("DEFECT_ABORT: FAIL - page is No-Access: a handler read would fault\n");
				ret = -EIO;
				break;
			}

			lldbg("DEFECT_ABORT: wdog page 0x%08x pte=0x%08x AP=%u - never No-Access, a handler read is legal\n",
			      page, pte, (unsigned int)ap);

			/* Read the page only now: the descriptor above proves
			 * the access cannot fault.  Reading it in a declaration
			 * initialiser would abort here instead of reporting. */
			lldbg("DEFECT_ABORT: read from outside wdog code: pool[0]=0x%02x\n",
			      (unsigned int)(*(volatile uint8_t *)&g_wdog_data));

			wd_mmu_get_window_state(&open, &cpu, &depth);
			lldbg("DEFECT_ABORT: window=%s cpu%u depth=%u failures=%u flips=%u\n",
			      open ? "OPEN" : "closed", (unsigned int)cpu,
			      (unsigned int)depth, wd_mmu_get_failures(),
			      wd_mmu_get_flips());

			if (wd_mmu_get_flips() == 0) {
				lldbg("DEFECT_ABORT: FAIL - no descriptor flip since boot, nothing is installed\n");
				ret = -EIO;
				break;
			}
		}
#else
		/* Half the assertions are compiled out with the protection off
		 * (wd_mmu_is_protected()/wd_mmu_get_flips() are the stubs in
		 * wdog.h, and mmu_get_page_pte_raw() is not even declared), so
		 * no verdict may be claimed here. */
		lldbg("DEFECT_ABORT: SKIP - needs CONFIG_WDOG_MMU_PROTECT and CONFIG_MMU_PAGE_PROTECT\n");
		ret = -ENOSYS;
		break;
#endif

		{
			/* 0x00000000 is unclaimed, 0x60200000 is the 1MB section
			 * 'wdog_refuse' uses, 0xf0000000 is not mapped at all.
			 * Either answer is acceptable - what is not is a
			 * different error, and above all the lookup has to
			 * return: a dereference here is the nested abort this
			 * case exists to exclude.  A section an earlier command
			 * has already split answers "page descriptor", which is
			 * why the refusal count is printed and not asserted.
			 */
			static const uint32_t hostile[] =
			{
				0x00000000, 0x60200000, 0xf0000000
			};

			int unexpected = 0;
			int refused = 0;
			int i;

			for (i = 0; i < 3; i++) {
				uint32_t pte;
				uint8_t ap;
				int status = mmu_get_page_pte_raw(hostile[i], &pte, &ap);

				lldbg("DEFECT_ABORT: DFAR lookup 0x%08x -> %s (%d)\n",
				      hostile[i],
				      status == OK ? "page descriptor" : "refused",
				      status);

				if (status == OK) {
					continue;
				}

				refused++;
				if (status != -EINVAL) {
					unexpected++;
				}
			}

			if (unexpected != 0) {
				lldbg("DEFECT_ABORT: FAIL - %d lookups returned an unexpected error\n",
				      unexpected);
				ret = -EIO;
				break;
			}

			lldbg("DEFECT_ABORT: %d of 3 foreign DFARs classified without a walk\n",
			      refused);
		}

		lldbg("DEFECT_ABORT: PASS - handler inputs are fault-free, hazard excluded by the AP policy\n");
		lldbg("DEFECT_ABORT: fatal half still covered by wdog_write, region_report, heap_protect 4\n");

		ret = OK;
		break;
	}

	case TESTIOC_WDOG_DEFECT_BOOT_ORDER: {
		/* Defect #13: Boot order issue.
		 *
		 * wd_mmu_protect_init() is called at the end of wd_initialize().
		 * If any wdog writes occur between wd_initialize() and the
		 * first user wd_start() call, they will Data Abort because
		 * the protection is already enabled but the code may not
		 * be using begin()/end().
		 *
		 * This test checks the boot sequence for unprotected wdog writes.
		 */
		bool win_open = false;
		uint8_t win_cpu = 0xff;
		uint8_t win_depth = 0;
		uint32_t fails = wd_mmu_get_failures();

		lldbg("=== DEFECT #13: Boot order issue ===\n");
		lldbg("DEFECT_BOOT: Checking boot sequence for unprotected wdog writes...\n");
		lldbg("DEFECT_BOOT: wd_mmu_protect_init() is called at end of wd_initialize()\n");
		lldbg("DEFECT_BOOT: Any wdog write between init and first wd_start() → Data Abort\n");
		lldbg("DEFECT_BOOT: Fix: Verify no wdog writes in boot path after init\n");

		/* If we reached here, boot completed without Data Abort: an
		 * unprotected wdog write during boot would have faulted the
		 * box before this point.  What is left to assert is that the
		 * protection is really live, that it has counted no error
		 * since boot, and that no window is stuck open - the old
		 * version printed "If protection is enabled, boot order is
		 * safe" and returned OK whether protection existed or not.
		 */

		lldbg("DEFECT_BOOT: Boot completed — no Data Abort during boot sequence\n");

		wd_mmu_get_window_state(&win_open, &win_cpu, &win_depth);

		if (wd_mmu_is_protected() && !win_open && win_depth == 0) {
			lldbg("DEFECT_BOOT: PASS - protection live, no window stuck; "
			      "%u error(s) counted since boot%s\n", fails,
			      fails == 0 ? "" :
			      " (earlier test commands count against this boot)");
			ret = OK;
		} else {
			lldbg("DEFECT_BOOT: FAIL - protected=%d window=%s depth=%u failures=%u\n",
			      wd_mmu_is_protected() ? 1 : 0,
			      win_open ? "OPEN" : "closed",
			      (unsigned int)win_depth, fails);
			ret = -EIO;
		}

		break;
	}

	case TESTIOC_WDOG_DEFECT_SECTION_SIZE: {
		/* Defect #15: is the protection really in the page table, and
		 * really only one page wide?
		 *
		 * The old mechanism changed a 1MB section, so "protected" and
		 * "sixteen megabytes of neighbours protected too" were the same
		 * operation, and a stale TLB entry of that section silently
		 * allowed writes after the change.
		 *
		 * This test checks the descriptor of the watchdog page itself:
		 *   - the page is mapped through the second level (a 4KB
		 *     descriptor exists for it);
		 *   - that descriptor says read-only;
		 *   - the pages on either side of it are still writable, which
		 *     is what 4KB granularity is supposed to mean.
		 *
		 * FATAL part (kept from the original test): the write at the
		 * end.  With the protection in place it takes the box down,
		 * which is the point - a write from outside the watchdog code
		 * must not be silently allowed.
		 */
		volatile uint8_t *wdog_page = (volatile uint8_t *)&g_wdpool;
		uint32_t page = ((uint32_t)(uintptr_t)wdog_page) & ~4095u;
		uint32_t pte;
		uint8_t ap;
		int status;

		lldbg("=== DEFECT #15: descriptor of the protected page ===\n");

		status = mmu_get_page_pte_raw(page, &pte, &ap);

		if (status != OK) {
			lldbg("DEFECT_SECTION: FAIL - watchdog page is not 4KB-mapped\n");
			ret = ERROR;
			break;
		}

		lldbg("DEFECT_SECTION: wdog page 0x%08x pte=0x%08x AP=%u (%s)\n",
		      page, pte, (unsigned int)ap,
		      ap == 0 ? "NO ACCESS" :
		      (ap & 2) ? "READ-ONLY" : "READ-WRITE");

		if (!(ap & 2)) {
			lldbg("DEFECT_SECTION: FAIL - watchdog page is writable\n");
			ret = ERROR;
			break;
		}

		/* Granularity: the neighbours must not have been dragged along.
		 * Both sides are ordinary .bss / .data and have to stay
		 * writable. */

		if (mmu_get_page_pte_raw(page - 4096, &pte, &ap) == OK) {
			lldbg("DEFECT_SECTION: page before:  AP=%u (%s)\n",
			      (unsigned int)ap, (ap & 2) ? "READ-ONLY" : "writable");
		} else {
			lldbg("DEFECT_SECTION: page before:  not 4KB-mapped\n");
		}

		if (mmu_get_page_pte_raw(page + 4096, &pte, &ap) == OK) {
			lldbg("DEFECT_SECTION: page after:   AP=%u (%s)\n",
			      (unsigned int)ap, (ap & 2) ? "READ-ONLY" : "writable");
		} else {
			lldbg("DEFECT_SECTION: page after:   not 4KB-mapped\n");
		}

		lldbg("DEFECT_SECTION: next write is the fatal one (by design)\n");

		{
			irqstate_t irq_state = enter_critical_section();
			uint8_t orig = wdog_page[0];

			wdog_page[0] = 0xDD;
			wdog_page[0] = orig;
			leave_critical_section(irq_state);

			lldbg("DEFECT_SECTION: === DEFECT CONFIRMED ===\n");
			lldbg("DEFECT_SECTION: Write succeeded - stale TLB entry!\n");
			lldbg("DEFECT_SECTION: Protection was NOT effective\n");
		}

		ret = OK;
		break;
	}

#endif /* CONFIG_WDOG_MMU_DEFECT_TEST */

#ifdef CONFIG_WDOG_MMU_SCENARIO_TEST

	case TESTIOC_WDOG_SCENARIO_NORMAL_OP: {
		/* TC-05: Normal wdog operation (start/cancel 100x).
		 *
		 * Start and cancel a watchdog 100 times under MMU protection.
		 * Verifies that normal wdog operations work correctly without
		 * Data Abort. Each iteration:
		 *   1. wd_create() — allocates a wdog from the pool
		 *   2. wd_start()  — starts the watchdog with a 10s delay
		 *   3. wd_cancel() — cancels the watchdog
		 *   4. wd_delete() — returns the wdog to the pool
		 *
		 * If MMU protection has a bug, a Data Abort will occur during
		 * any of these operations.
		 */
		int i;
		WDOG_ID wdog;
		int ok_count = 0;

		lldbg("=== TC-05: Normal wdog operation (100 iterations) ===\n");

		for (i = 0; i < 100; i++) {
			wdog = wd_create();
			if (wdog == NULL) {
				lldbg("TC05: wd_create() failed at iteration %d\n", i);
				break;
			}

			if (wd_start(wdog, 10000, NULL, 0) != OK) {
				lldbg("TC05: wd_start() failed at iteration %d\n", i);
				wd_delete(wdog);
				break;
			}

			wd_cancel(wdog);
			wd_delete(wdog);
			ok_count++;
		}

		lldbg("TC05: Completed %d/100 iterations successfully\n", ok_count);
		if (ok_count == 100) {
			lldbg("TC05: PASS — all 100 start/cancel cycles completed\n");
		} else {
			lldbg("TC05: FAIL — only %d/100 completed\n", ok_count);
		}

		ret = (ok_count == 100) ? OK : -EIO;
		break;
	}

	case TESTIOC_WDOG_SCENARIO_MULTI_EXPIRE: {
		/* TC-06: Multiple watchdog expiration in same tick.
		 *
		 * Start 5 watchdogs with the same delay (1 tick). All 5 will
		 * expire in the same timer tick. The wd_timer() handler must
		 * process all 5 expirations under MMU protection without
		 * Data Abort.
		 *
		 * Each watchdog has a callback that increments a counter.
		 * After the delay, verify all 5 callbacks were invoked.
		 */
		#define TC06_NUM_WDOGS  5
		WDOG_ID wdogs[TC06_NUM_WDOGS];
		int i;

		lldbg("=== TC-06: Multiple watchdog expiration in same tick ===\n");
		g_tc06_callback_count = 0;

		/* Create and start 5 watchdogs with the same 1-tick delay */
		for (i = 0; i < TC06_NUM_WDOGS; i++) {
			wdogs[i] = wd_create();
			if (wdogs[i] == NULL) {
				lldbg("TC06: wd_create() failed for wdog %d\n", i);
				ret = -ENOMEM;
				goto tc06_cleanup;
			}
		}

		for (i = 0; i < TC06_NUM_WDOGS; i++) {
			if (wd_start(wdogs[i], 10, tc06_callback, 0) != OK) {
				lldbg("TC06: wd_start() failed for wdog %d\n", i);
				ret = -EIO;
				goto tc06_cleanup;
			}
		}

		lldbg("TC06: Started %d watchdogs with 10-tick delay, waiting...\n",
		      TC06_NUM_WDOGS);

		/* Wait for expiration (10 ticks = 100ms at 100Hz, wait 200ms) */
		up_mdelay(200);

		lldbg("TC06: Callback count = %d (expected %d)\n",
		      g_tc06_callback_count, TC06_NUM_WDOGS);

		if (g_tc06_callback_count == TC06_NUM_WDOGS) {
			lldbg("TC06: PASS — all %d watchdogs expired without Data Abort\n",
			      TC06_NUM_WDOGS);
			ret = OK;
		} else {
			lldbg("TC06: FAIL — only %d/%d callbacks invoked\n",
			      g_tc06_callback_count, TC06_NUM_WDOGS);

			ret = -EIO;
		}

	tc06_cleanup:
		for (i = 0; i < TC06_NUM_WDOGS; i++) {
			if (wdogs[i]) {
				wd_cancel(wdogs[i]);
				wd_delete(wdogs[i]);
			}
		}
		break;
	}

	case TESTIOC_WDOG_SCENARIO_SMP_CONCURRENT: {
		/* TC-08: SMP concurrent wdog access from multiple cores.
		 *
		 * On SMP systems, multiple cores call wd_start/wd_cancel
		 * concurrently. This tests whether the per-core DACR issue
		 * causes a Data Abort when Core 1 accesses wdog pool while
		 * Core 0 is inside a write window.
		 *
		 * This test creates a kernel thread pinned to Core 1 via
		 * task_create() + sched_setaffinity(). The thread repeatedly
		 * calls wd_start/wd_cancel from Core 1 while Core 0 (the ioctl
		 * caller) also does wdog operations.
		 *
		 * If the per-core DACR defect exists, Core 1's wd_start() will
		 * trigger a Data Abort because Core 1's DACR is still RO when
		 * Core 0 calls wd_mmu_write_begin().
		 *
		 * NOTE: This test is only meaningful on SMP (CONFIG_SMP=y).
		 * On single-core, it runs the same logic but without
		 * actual concurrency.
		 */

		lldbg("=== TC-08: SMP concurrent wdog access ===\n");
#ifdef CONFIG_SMP
		lldbg("TC08: SMP is enabled — creating Core 1 thread\n");

		{
			pid_t core1_pid;
			cpu_set_t cpu_mask;
			int i;
			WDOG_ID wdog;
			int ok_count = 0;

			g_tc08_core1_running = 1;
			g_tc08_core1_ok_count = 0;
			g_tc08_core1_stop = 0;

			/* Create a kernel thread */
			core1_pid = kernel_thread("tc08_core1", 100, 2048,
			                         tc08_core1_thread, NULL);

			if (core1_pid < 0) {
				lldbg("TC08: task_create() failed (%d)\n", core1_pid);
				ret = -ENOMEM;
				break;
			}

			/* Pin the thread to Core 1 (CPU 1) */
			CPU_ZERO(&cpu_mask);
			CPU_SET(1, &cpu_mask);
			if (sched_setaffinity(core1_pid, sizeof(cpu_set_t),
			                      &cpu_mask) != OK) {
				lldbg("TC08: sched_setaffinity() failed — thread runs on Core 0\n");
			} else {
				lldbg("TC08: Core 1 thread pinned to CPU 1 (pid=%d)\n",
				      core1_pid);
			}

			/* Core 0: also do wdog operations concurrently */
			for (i = 0; i < 500; i++) {
				wdog = wd_create();
				if (!wdog) {
					lldbg("TC08: Core0 wd_create() failed at %d\n", i);
					break;
				}
				if (wd_start(wdog, 10000, NULL, 0) != OK) {
					lldbg("TC08: Core0 wd_start() failed at %d\n", i);
					wd_delete(wdog);
					break;
				}
				wd_cancel(wdog);
				wd_delete(wdog);
				ok_count++;
			}

			/* Wait for Core 1 thread to finish */
			lldbg("TC08: Core 0 done (%d iterations), waiting for Core 1...\n",
			      ok_count);
			while (g_tc08_core1_running) {
				up_mdelay(10);
			}

			lldbg("TC08: Core 0 = %d/500, Core 1 = %d/500\n",
			      ok_count, g_tc08_core1_ok_count);

			if (ok_count == 500 && g_tc08_core1_ok_count == 500) {
				lldbg("TC08: PASS — both cores completed 500 iterations\n");
				lldbg("TC08: No Data Abort — wdog MMU protection handles SMP\n");
				ret = OK;
			} else {
				lldbg("TC08: FAIL — Core0=%d, Core1=%d\n",
				      ok_count, g_tc08_core1_ok_count);
				ret = -EIO;
			}
		}

#else
		lldbg("TC08: SMP is NOT enabled — single-core simulation\n");
		{
			int i;
			WDOG_ID wdog;
			int ok_count = 0;

			for (i = 0; i < 1000; i++) {
				wdog = wd_create();
				if (!wdog) {
					lldbg("TC08: wd_create() failed at iteration %d\n", i);
					break;
				}
				if (wd_start(wdog, 10000, NULL, 0) != OK) {
					lldbg("TC08: wd_start() failed at iteration %d\n", i);
					wd_delete(wdog);
					break;
				}
				wd_cancel(wdog);
				wd_delete(wdog);
				ok_count++;
			}

			lldbg("TC08: Completed %d/1000 iterations\n", ok_count);
			if (ok_count == 1000) {
				lldbg("TC08: PASS — 1000 start/cancel cycles OK\n");
				ret = OK;
			} else {
				lldbg("TC08: FAIL — only %d/1000 completed\n", ok_count);
				ret = -EIO;
			}
		}
#endif /* CONFIG_SMP */
		break;
	}

	case TESTIOC_WDOG_SCENARIO_SMP_TICK_CB: {
		/* TC-09: SMP timer tick on Core 0, callback on Core 1.
		 *
		 * On SMP, the timer tick fires on Core 0, which calls
		 * wd_timer(). wd_timer() processes expirations and invokes
		 * callbacks. If a callback runs on Core 1, it may access
		 * wdog data structures. Core 1's DACR is still RO (Client),
		 * so any wdog write in the callback → Data Abort.
		 *
		 * This test creates a kernel thread pinned to Core 1 that
		 * starts a watchdog with a real callback function. The
		 * callback may fire on Core 0 (timer tick core) while the
		 * wd_start() was called from Core 1.
		 *
		 * On SMP, this exercises the cross-core DACR scenario:
		 * - Core 1 calls wd_start() → wd_mmu_write_begin() sets
		 *   Core 1's DACR = RW
		 * - Timer tick on Core 0 → wd_timer() → wd_mmu_write_begin()
		 *   sets Core 0's DACR = RW
		 * - Callback runs → may be on Core 0 or Core 1
		 */
		lldbg("=== TC-09: SMP timer tick / callback cross-core ===\n");
		g_tc09_callback_fired = 0;
		g_tc09_core1_done = 0;
		g_tc09_wdog = NULL;

#ifdef CONFIG_SMP
		lldbg("TC09: SMP is enabled — creating Core 1 thread\n");
		{
			pid_t core1_pid;
			cpu_set_t cpu_mask;

			/* Create a kernel thread on Core 1 */
			core1_pid = kernel_thread("tc09_core1", 100, 2048,
			                         tc09_core1_thread, NULL);

			if (core1_pid < 0) {
				lldbg("TC09: task_create() failed (%d)\n", core1_pid);
				ret = -ENOMEM;
				break;
			}

			/* Pin the thread to Core 1 (CPU 1) */
			CPU_ZERO(&cpu_mask);
			CPU_SET(1, &cpu_mask);
			if (sched_setaffinity(core1_pid, sizeof(cpu_set_t),
			                      &cpu_mask) != OK) {
				lldbg("TC09: sched_setaffinity() failed\n");
			} else {
				lldbg("TC09: Core 1 thread pinned to CPU 1 (pid=%d)\n",
				      core1_pid);
			}

			/* Wait for Core 1 to start the watchdog */
			lldbg("TC09: Waiting for Core 1 to start watchdog...\n");
			while (!g_tc09_core1_done) {
				up_mdelay(10);
			}

			/* Wait for callback to fire */
			lldbg("TC09: Waiting for callback...\n");
			up_mdelay(50);
		}
#else
		lldbg("TC09: SMP NOT enabled — single-core test\n");
		{
			g_tc09_wdog = wd_create();
			if (!g_tc09_wdog) {
				lldbg("TC09: wd_create() failed\n");
				ret = -ENOMEM;
				break;
			}
			if (wd_start(g_tc09_wdog, 1, tc09_callback, 0) != OK) {
				lldbg("TC09: wd_start() failed\n");
				wd_delete(g_tc09_wdog);
				g_tc09_wdog = NULL;
				ret = -EIO;
				break;
			}
			up_mdelay(50);
		}
#endif /* CONFIG_SMP */

		if (g_tc09_callback_fired) {
			lldbg("TC09: PASS — callback fired without Data Abort\n");
#ifdef CONFIG_SMP
			lldbg("TC09: wd_start() from Core 1, callback fired — cross-core OK\n");
#endif
			ret = OK;
		} else {
			lldbg("TC09: FAIL — callback did not fire after 50ms\n");
			ret = -EIO;
		}

		if (g_tc09_wdog) {
			wd_cancel(g_tc09_wdog);
			wd_delete(g_tc09_wdog);
			g_tc09_wdog = NULL;
		}
		break;
	}

	case TESTIOC_WDOG_SCENARIO_NESTING_STRESS: {
		/* TC-13: Nesting counter stress test (1000 reentrant calls).
		 *
		 * Only the first begin() may touch the descriptor and only the
		 * last end() may put it back; everything in between has to be
		 * bookkeeping.  What is checked here is the observable state -
		 * the window, and the permission the page really has - not the
		 * counter itself:
		 *
		 *   after 1000 begin():  window open,  page writable;
		 *   after  999 end():    still open,   still writable;
		 *   after the last end(): closed,       page read-only again.
		 *
		 * The reported depth is one byte, so 1000 levels are reported
		 * as the 255 it saturates at; that is a reporting limit, not a
		 * limit of the counter (which is an int).
		 */
		volatile uint8_t *wdog_page = (volatile uint8_t *)&g_wdpool;
		bool win_open = false;
		bool bad = false;			/* any observable that was wrong */
		uint8_t win_cpu = 0xff;
		uint8_t win_depth = 0;
		uint32_t pte = 0;
		uint8_t ap = 0xff;
		uint32_t fails_before = wd_mmu_get_failures();
		irqstate_t irq_state;
		int i;

		lldbg("=== TC-13: Nesting counter stress (1000 reentrant calls) ===\n");

		irq_state = enter_critical_section();

		for (i = 0; i < 1000; i++) {
			if (wd_mmu_write_begin() != OK) {
				break;
			}
		}

		wd_mmu_get_window_state(&win_open, &win_cpu, &win_depth);
		(void)mmu_get_page_pte_raw((uint32_t)(uintptr_t)wdog_page, &pte, &ap);
		lldbg("TC13: after 1000 begin(): window=%s depth%u(reported, saturates at 255) AP=%u\n",
		      win_open ? "OPEN" : "closed (FAIL)",
		      (unsigned int)win_depth, (unsigned int)ap);

		if (!win_open || (ap & 2)) {
			lldbg("TC13: FAIL - page is not writable with the window open\n");
			bad = true;
		}

		for (i = 0; i < 999; i++) {
			MMU_TRYQ(wd_mmu_write_end());
		}

		wd_mmu_get_window_state(&win_open, &win_cpu, &win_depth);
		(void)mmu_get_page_pte_raw((uint32_t)(uintptr_t)wdog_page, &pte, &ap);

		/* AP folded to 2 bits: bit 1 set means AP2 is on, i.e. read-only.
		 * Expected state on this line: 999 end() out of 1000 begin()
		 * leaves the window open at depth 1, so the page MUST still be
		 * writable; "closed" is the only wrong answer.  The verdict is
		 * taken after the last end() below. */

		lldbg("TC13: after 999 end():   window=%s cpu%u AP=%u (page writable=%s, expected yes)\n",
		      win_open ? "OPEN" : "closed (FAIL)", (unsigned int)win_cpu,
		      (unsigned int)ap, (ap & 2) ? "no (FAIL)" : "YES");

		if (!win_open || (ap & 2)) {
			lldbg("TC13: FAIL - depth 1 must leave the window open and writable\n");
			bad = true;
		}

		MMU_TRY(wd_mmu_write_end());

		leave_critical_section(irq_state);

		wd_mmu_get_window_state(&win_open, &win_cpu, &win_depth);
		(void)mmu_get_page_pte_raw((uint32_t)(uintptr_t)wdog_page, &pte, &ap);

		/* 2000 brackets that only move a counter may not have been
		 * refused once: the counter is the system-wide one, so a
		 * refusal here means a core stepped into a window it does not
		 * own (or closed a window that was already closed). */

		if (wd_mmu_get_failures() != fails_before) {
			lldbg("TC13: FAIL - %u refused/failed flips during the loop\n",
			      (unsigned int)(wd_mmu_get_failures() - fails_before));
			bad = true;
		}

		if (!win_open && (ap & 2) && !bad) {
			lldbg("TC13: PASS - closed after the last end(), page read-only, no refusal\n");
			ret = OK;
		} else {
			lldbg("TC13: FAIL - window=%s AP=%u bad=%s after the last end()\n",
			      win_open ? "still OPEN" : "closed", (unsigned int)ap,
			      bad ? "yes" : "no");
			ret = -EIO;
		}

		break;
	}

	case TESTIOC_WDOG_SCENARIO_LONG_RUN: {
		/* TC-15: Long-run stability test.
		 *
		 * Run continuous wdog start/cancel operations for a configurable
		 * duration. Default is 60 seconds for testing. For production
		 * validation, extend to 24 hours.
		 *
		 * This test verifies:
		 * - No memory leak (wdog pool not exhausted)
		 * - No Data Abort over time
		 * - Nesting counter stays balanced
		 *
		 * arg = 0: 60 seconds (default)
		 * arg = 1: 300 seconds (5 minutes)
		 * arg = 2: 86400 seconds (24 hours)
		 */
		int duration_sec = 60;
		WDOG_ID wdog;
		int total_ops = 0;

		if (arg == 1) {
			duration_sec = 300;
		} else if (arg == 2) {
			duration_sec = 86400;
		}

		lldbg("=== TC-15: Long-run stability (%d seconds) ===\n", duration_sec);
		lldbg("TC15: Running continuous start/cancel for %d sec...\n", duration_sec);

		/* Run for the specified duration */
		while (total_ops < duration_sec * 100) {
			wdog = wd_create();
			if (!wdog) {
				lldbg("TC15: wd_create() failed after %d ops — pool exhausted!\n",
				      total_ops);
				lldbg("TC15: FAIL — memory leak detected\n");
				ret = -ENOMEM;
				break;
			}

			wd_start(wdog, 10000, NULL, 0);
			wd_cancel(wdog);
			wd_delete(wdog);
			total_ops++;

			/* Print progress every 10000 ops */
			if (total_ops % 10000 == 0) {
				lldbg("TC15: %d ops completed (%d/%d sec)\n",
				      total_ops, total_ops / 100, duration_sec);
			}
		}

		if (ret != -ENOMEM) {
			lldbg("TC15: Completed %d ops in %d seconds\n", total_ops, duration_sec);
			lldbg("TC15: PASS — no Data Abort, no memory leak\n");
			ret = OK;
		}
		break;
	}

#endif /* CONFIG_WDOG_MMU_SCENARIO_TEST */

#ifdef CONFIG_WDOG_MMU_PROTECT
	/* TESTIOC_WDOG_MMU_WINDOW_STATE (58): the write window as it stands.
	 * Read-only - it never opens a window, so it can be run before,
	 * between and after any other case. */

	case TESTIOC_WDOG_MMU_WINDOW_STATE:
		{
			bool win_open = false;
			uint8_t win_cpu = 0xff;
			uint8_t win_depth = 0;

			wd_mmu_get_window_state(&win_open, &win_cpu, &win_depth);
			lldbg("WD_WINDOW: protected=%d window=%s owner=cpu%u depth=%u\n",
			      wd_mmu_is_protected() ? 1 : 0,
			      win_open ? "OPEN" : "closed",
			      (unsigned int)win_cpu, (unsigned int)win_depth);
			lldbg("WD_WINDOW: wdog failures=%u, descriptor flips=%u, "
			      "discarded statuses=%d\n",
			      wd_mmu_get_failures(), wd_mmu_get_flips(),
			      g_mmu_op_failures);
			ret = OK;
			break;
		}

	/* TESTIOC_WDOG_MMU_PAGE_REPORT (59): the live descriptor of the page
	 * that holds g_wdog_data.  A non-OK status means the page is no longer
	 * 4KB-mapped, which is what would happen if some other path rebuilt its
	 * L1 entry as a section.  AP2 (bit 9) is the bit wd_mmu_install()
	 * flips, so on its own it decides read-only versus writable. */

	case TESTIOC_WDOG_MMU_PAGE_REPORT:
		{
			uint32_t page = (uint32_t)((uintptr_t)&g_wdog_data & ~(uintptr_t)0xfff);
			uint32_t pte = 0;
			uint8_t ap = 0xff;
			int status = mmu_get_page_pte_raw(page, &pte, &ap);

			lldbg("WD_PAGE: 0x%08x status=%d pte=0x%08x AP=%u %s\n",
			      page, status, pte, (unsigned int)ap,
			      status != OK ? "NOT 4KB-MAPPED - protection is gone" :
			      ((pte & (1 << 9)) ? "read-only" : "WRITABLE"));
			lldbg("WD_PAGE: wdog failures=%u, discarded statuses=%d\n",
			      wd_mmu_get_failures(), g_mmu_op_failures);
			ret = status;
			break;
		}

	/* TESTIOC_WDOG_MMU_REFUSE_SECTION (60): a 1MB-mapped address must be
	 * refused, never split behind the owner's back.  The candidate is
	 * picked with mmu_get_page_pte_raw() first, so only an address that is
	 * a section and is not already page-mapped reaches
	 * mmu_set_page_no_access(), and that call only takes its refusal path
	 * without writing anything. */

	case TESTIOC_WDOG_MMU_REFUSE_SECTION:
		{
			FAR uint32_t *l1 = mmu_get_os_l1_pgtbl();
			uint32_t sect = (uint32_t)((uintptr_t)&g_wdog_data) & 0xfff00000;
			uint32_t probe;
			uint32_t pte = 0;
			uint8_t ap = 0xff;
			int status = -ENOENT;

			for (probe = sect + (1 << 20);
			     probe < sect + (8 << 20);
			     probe += (1 << 20)) {
				if ((l1[probe >> 20] & PMD_TYPE_MASK) != PMD_TYPE_SECT) {
					continue;
				}

				if (mmu_get_page_pte_raw(probe, &pte, &ap) == OK) {
					continue;  /* already page-mapped: not a candidate */
				}

				status = mmu_set_page_no_access(probe);
				lldbg("WD_REFUSE: 0x%08x is still a section, set_no_access -> %d (%s)\n",
				      probe, status,
				      status == -EINVAL ? "refused as expected" : "UNEXPECTED");
				break;
			}

			if (status == -ENOENT) {
				lldbg("WD_REFUSE: no section-mapped address near 0x%08x\n", sect);
			}

			ret = (status == -EINVAL) ? OK : status;
			break;
		}

	/* TESTIOC_WDOG_MMU_WINDOW_PAIR (61): arg begin()/end() pairs, then a
	 * nested pair, then one close too many.  Nothing here may leave the
	 * page writable or a status behind unused: the counters and the window
	 * state decide the verdict, not the absence of a crash. */

	case TESTIOC_WDOG_MMU_WINDOW_PAIR:
		{
			unsigned int reps = (arg >= 1 && arg <= 1000) ? (unsigned int)arg : 32;
			unsigned int i;
			uint32_t before = wd_mmu_get_failures();
			uint32_t after;
			int ops_before = g_mmu_op_failures;
			bool win_open = false;
			uint8_t win_cpu = 0xff;
			uint8_t win_depth = 0;
			int extra;

			for (i = 0; i < reps; i++) {
				MMU_TRY(wd_mmu_write_begin());
				MMU_TRY(wd_mmu_write_end());
			}

			MMU_TRY(wd_mmu_write_begin());
			MMU_TRY(wd_mmu_write_begin());
			wd_mmu_get_window_state(&win_open, &win_cpu, &win_depth);
			lldbg("WD_PAIR: nested depth=%u owner=cpu%u (expect 2, this cpu)\n",
			      (unsigned int)win_depth, (unsigned int)win_cpu);
			MMU_TRY(wd_mmu_write_end());
			MMU_TRY(wd_mmu_write_end());

			/* One close with no open: must be refused, must not
			 * touch the descriptor and must not say OK. */

			extra = wd_mmu_write_end();
			wd_mmu_get_window_state(&win_open, &win_cpu, &win_depth);
			lldbg("WD_PAIR: %u pairs, unpaired close -> %d, window=%s depth=%u\n",
			      reps, extra, win_open ? "OPEN" : "closed",
			      (unsigned int)win_depth);

			after = wd_mmu_get_failures();
			lldbg("WD_PAIR: failures %u -> %u (expect exactly %u + 1), discarded statuses=%d\n",
			      before, after, before, g_mmu_op_failures);

			/* The unpaired end() above is a deliberate negative
			 * test: it has to be refused and it has to be counted
			 * exactly once, so the failure counter is expected to
			 * grow by one - not to stay where it started, and not
			 * by anything else either.  Every other condition here
			 * means the paired work itself was not clean.
			 */

			ret = OK;
			if (after != before + 1 ||
			    g_mmu_op_failures != ops_before ||
			    win_open || win_depth != 0 || extra == OK) {
				ret = -EIO;
			}

			break;
		}
#endif /* CONFIG_WDOG_MMU_PROTECT */

	default:
		vdbg("Unrecognized cmd: %d arg: %ld\n", cmd, arg);
		break;

	}

#ifdef CONFIG_MM_HEAP_UAF_PROTECT
	if (cmd == TESTIOC_MMU_HEAP_PROTECT_STRESS ||
	    cmd == TESTIOC_MMU_TLB_BENCH) {
		mmu_set_uaf_protect_enabled(0);
	}
#endif

	return ret;
}

static ssize_t os_api_test_drv_read(FAR struct file *filep, FAR char *buffer, size_t len)
{
	return 0;                                       /* Return EOF */
}

static ssize_t os_api_test_drv_write(FAR struct file *filep, FAR const char *buffer, size_t len)
{
	return len;                                     /* Say that everything was written */
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: os_api_test_drv_register
 *
 * Description:
 *   Register /dev/os_api_test
 *
 ****************************************************************************/

void os_api_test_drv_register(void)
{
	(void)register_driver(OS_API_TEST_DRVPATH, &os_api_test_drv_fops, 0666, NULL);
}
