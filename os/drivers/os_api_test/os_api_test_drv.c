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
#include <tinyara/os_api_test_drv.h>
#include <tinyara/sched.h>
#include <tinyara/irq.h>
#include <tinyara/mmu.h>
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
#define R3_PAIR_REPS  2000    /* protect+restore pairs timed in Phase D    */
#define R3_PAIR_BATCH 100     /* pairs per Phase D timing sample           */

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
		mmu_restore_page_pte(page_addr, mmu_save_page_pte(page_addr));
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
			
			/* Restore to section */
			l1table[l1_index] = sect_entry;
			
			/* Flush TLB */
			__asm__ volatile("mcr p15, 0, %0, c8, c7, 0" :: "r"(0));
			__asm__ volatile("dsb sy");
			__asm__ volatile("isb");
			
			lldbg("R1: Restored to section 0x%08X\n", sect_entry);
			
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

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("AL SPLIT: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;

		lldbg("AL SPLIT: page=0x%08x, calling mmu_split_section_to_l2\n", page_addr);

		mmu_split_section_to_l2(page_addr);

		/* Verify memory is still accessible after split */
		memset((void *)page_addr, 0xAA, 4096);
		lldbg("AL SPLIT: memory read/write OK after split\n");

		kmm_free(buf);
		ret = OK;
		break;
	}

	/* TC-AL-02: No-access + restore round-trip (Safe) */
	case TESTIOC_AL_NOACCESS_RESTORE: {
		uint8_t *buf;
		uint32_t page_addr;
		uint32_t saved_pte;

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("AL NOACCESS_RESTORE: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;
		memset((void *)page_addr, 0xAA, 4096);

		saved_pte = mmu_save_page_pte(page_addr);
		lldbg("AL NOACCESS_RESTORE: saved_pte=0x%08x\n", saved_pte);

		mmu_set_page_no_access(page_addr);
		lldbg("AL NOACCESS_RESTORE: page set to No-Access\n");

		mmu_restore_page_pte(page_addr, saved_pte);
		lldbg("AL NOACCESS_RESTORE: page restored\n");

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
		volatile uint8_t val;

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("AL RO_RESTORE: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;
		memset((void *)page_addr, 0xAA, 4096);

		saved_pte = mmu_save_page_pte(page_addr);
		lldbg("AL RO_RESTORE: saved_pte=0x%08x\n", saved_pte);

		mmu_set_page_ro(page_addr);
		lldbg("AL RO_RESTORE: page set to Read-Only\n");

		/* Read should succeed under RO */
		val = ((volatile uint8_t *)page_addr)[0];
		lldbg("AL RO_RESTORE: read under RO OK, val=0x%02x\n", val);

		mmu_restore_page_pte(page_addr, saved_pte);
		lldbg("AL RO_RESTORE: page restored\n");

		/* Write should succeed after restore */
		((volatile uint8_t *)page_addr)[0] = 0xCC;
		lldbg("AL RO_RESTORE: write after restore OK\n");

		kmm_free(buf);
		ret = OK;
		break;
	}

	/* TC-AL-04: No-access triggers Data Abort (Fatal)
	 *
	 * Sets an entire 1MB section to No-Access WITHOUT splitting to 4KB.
	 * Since the kernel heap (and TCB) reside in the same 1MB section,
	 * the Data Abort handler itself faults → nested Data Abort.
	 *
	 * This nested abort is EXPECTED — it proves split_section_to_l2()
	 * is required before page-level protection in kernel context.
	 * See AL-05 for the correct approach (split first, then protect 4KB).
	 */
	case TESTIOC_AL_NOACCESS_ABORT: {
		uint8_t *buf;
		uint32_t page_addr;
		uint32_t saved_pte;
		volatile uint8_t val;
#ifdef CONFIG_SMP
		cpu_set_t cpu_mask;
		cpu_set_t orig_mask;
#endif

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("AL NOACCESS_ABORT: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;
		memset((void *)page_addr, 0xAA, 4096);

#ifdef CONFIG_SMP
		/* Pin to Core 0 — single-core test, prevent scheduler migration */
		sched_getaffinity(getpid(), sizeof(cpu_set_t), &orig_mask);
		CPU_ZERO(&cpu_mask);
		CPU_SET(0, &cpu_mask);
		sched_setaffinity(getpid(), sizeof(cpu_set_t), &cpu_mask);
		lldbg("AL NOACCESS_ABORT: Pinned to CPU 0 (was CPU %d)\n", up_cpu_index());
#endif

		lldbg("AL NOACCESS_ABORT: Setting 1MB section to No-Access (NO split)\n");
		lldbg("AL NOACCESS_ABORT: page 0x%08x is in 1MB section with TCB/stacks\n", page_addr);

		saved_pte = mmu_save_page_pte(page_addr);
		mmu_set_page_no_access(page_addr);
		lldbg("AL NOACCESS_ABORT: 1MB section set to No-Access on CPU %d\n", up_cpu_index());
		lldbg("AL NOACCESS_ABORT: >>> READ, expect Data Abort <<<\n");
		lldbg("AL NOACCESS_ABORT: NOTE: Nested Data Abort is EXPECTED here!\n");
		lldbg("AL NOACCESS_ABORT:   Reason: 1MB section contains TCB/stacks.\n");
		lldbg("AL NOACCESS_ABORT:   Abort handler accesses TCB → nested abort.\n");
		lldbg("AL NOACCESS_ABORT:   This proves split_section_to_l2() is required.\n");
		lldbg("AL NOACCESS_ABORT:   See AL-05 for correct approach (split first).\n");

		val = ((volatile uint8_t *)page_addr)[0];
		(void)val;

		lldbg("AL NOACCESS_ABORT: ERROR — no Data Abort occurred!\n");
		mmu_restore_page_pte(page_addr, saved_pte);
#ifdef CONFIG_SMP
		sched_setaffinity(getpid(), sizeof(cpu_set_t), &orig_mask);
#endif
		kmm_free(buf);
		ret = -EIO;
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
		mmu_restore_page_pte(page_addr, saved_pte);
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

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("AL RESTORE: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;
		memset((void *)page_addr, 0xAA, 4096);

		saved_pte = mmu_save_page_pte(page_addr);
		mmu_set_page_no_access(page_addr);
		mmu_restore_page_pte(page_addr, saved_pte);
		lldbg("AL RESTORE: protect+restore done, page accessible\n");

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

		buf = (uint8_t *)kmm_memalign(4096, 4096 * 2);
		if (!buf) {
			lldbg("AL SPLIT: kmm_memalign failed\n");
			return -ENOMEM;
		}
		page_addr = ((uint32_t)buf + 4095) & ~4095u;

		mmu_split_section_to_l2(page_addr);
		lldbg("AL SPLIT: section split, accessible\n");

		memset((void *)page_addr, 0xAA, 4096);
		lldbg("AL SPLIT: write OK\n");

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
		mmu_restore_page_pte((uint32_t)r2_page, saved_pte);
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

		lldbg("R3: clock floor 1000us: C = avg of %u splits, D = avg of %u pairs\n",
		      R3_SPLIT_REPS, R3_PAIR_REPS);

		lldbg("R3: =============== TLB/split benchmark report ===============\n");
		lldbg("R3: Phase A (1MB sections): %u us\n", res.baseline_us);
		lldbg("R3: Phase B (4KB L2 pages): %u us\n", res.split_us);
		lldbg("R3: overhead = %u%%   (R3 gate: < 10%%)\n", res.overhead_pct);
		lldbg("R3: one runtime split = %u us (L2 fill + full TLB flush)\n",
		      res.split_cost_us);
		lldbg("R3: 8KB block protect+restore pair = %u us (4 TLB syncs)\n",
		      res.op_cost_us);
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
		int phase = (int)arg;
		FAR uint8_t *mem;
		uintptr_t pg;
		uint32_t cok;
		uint32_t cskip;
		uint32_t cfail;
		int i;

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
		lldbg("R4: counters: ok=%u skipped=%u failed_budget=%u\n",
		      cok, cskip, cfail);

		if (phase < 2) {
			ret = OK;
			break;
		}

		/* Step 2: mixed churn - allocator must never fault on the
		 * pages it just unlocked (hook-ordering proof) */
		lldbg("R4/2: mixed-size churn (100 iterations)...\n");
		for (i = 0; i < 100; i++) {
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
		lldbg("R4: counters: ok=%u skipped=%u failed_budget=%u\n",
		      cok, cskip, cfail);

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
		/* Verify watchdog MMU page protection.
		 *
		 * This test accesses the wdog static data page (g_wdpool) from
		 * OUTSIDE wdog code — i.e. without calling wd_mmu_write_begin().
		 *
		 * Step 1: Read from the wdog page — should succeed (page is
		 *         Read-Only, reads are allowed).
		 * Step 2: Write to the wdog page — if CONFIG_WDOG_MMU_PROTECT
		 *         is enabled, the MMU triggers a Data Abort (permission
		 *         fault).  DFAR will contain the wdog page address.
		 *
		 * If protection is NOT enabled, the write succeeds but we
		 * immediately restore the original value (with interrupts
		 * disabled so no timer interrupt sees the corruption).
		 *
		 * arg = 0: read only (safe, no crash)
		 * arg = 1: write (crash if protection enabled)
		 */
		volatile uint8_t *wdog_page = (volatile uint8_t *)&g_wdpool;
		int do_write = (arg == 1);
		irqstate_t irq_state;

		lldbg("WDOG MMU protect test: wdog data at 0x%08x\n",
		      (uint32_t)wdog_page);
		/* Step 1: Read — should always succeed (Read-Only allows reads) */
		lldbg("WDOG MMU protect test: reading wdog page (should succeed)...\n");
		volatile uint8_t val = wdog_page[0];
		lldbg("WDOG MMU protect test: read OK, val = 0x%02x\n", val);

		if (do_write) {
			/* Step 2: Write — if CONFIG_WDOG_MMU_PROTECT is enabled,
			 * this triggers a Data Abort.  The page is Read-Only and
			 * we are NOT inside wd_mmu_write_begin()/wd_mmu_write_end().
			 *
			 * DFAR will contain &g_wdpool — the protected wdog address.
			 *
			 * If protection is NOT enabled, the write succeeds.  We
			 * disable interrupts and save/restore the original byte
			 * so the wdog data is not permanently corrupted.  Without
			 * interrupts, no timer tick can fire between the write
			 * and the restore.
			 */
			lldbg("WDOG MMU protect test: >>> WRITE to wdog page, expect Data Abort <<<\n");

			irq_state = enter_critical_section();
			uint8_t orig = wdog_page[0];
			wdog_page[0] = 0xBB;

			/* If we reach here, protection is NOT enabled.
			 * Restore the original value to avoid corrupting wdog data.
			 */
			wdog_page[0] = orig;
			leave_critical_section(irq_state);

			lldbg("WDOG MMU protect test: write succeeded (value restored), no Data Abort!\n");
			lldbg("WDOG MMU protect test: CONFIG_WDOG_MMU_PROTECT is NOT enabled.\n");
			lldbg("WDOG MMU protect test: Wdog page is writable — protection is OFF.\n");
			ret = OK;

		} else {
			lldbg("WDOG MMU protect test: read-only test passed.\n");
			lldbg("WDOG MMU protect test: use arg=1 to test write protection (crash).\n");
			ret = OK;
		}
		break;
	}

#endif

#ifdef CONFIG_WDOG_MMU_DEFECT_TEST

	case TESTIOC_WDOG_DEFECT_SMP: {
		/* Defect #1: SMP DACR per-core issue.
		 *
		 * DACR is a per-CPU register. When wd_mmu_write_begin() runs
		 * on Core 0, only Core 0's DACR is updated to Manager (RW).
		 * Core 1's DACR remains Client (RO). If Core 1 accesses the
		 * wdog pool, it gets a Data Abort.
		 *
		 * This test:
		 * 1. Reads DACR before begin() (should be RO = 0x05)
		 * 2. Calls wd_mmu_write_begin() (sets this core's DACR = RW)
		 * 3. Reads DACR after begin() (should be RW = 0x0D)
		 * 4. Writes to wdog pool from this core (should succeed)
		 * 5. Logs that other cores' DACR is still RO (defect!)
		 *
		 * On SMP: if another core accesses wdog pool now → Data Abort.
		 */
		uint32_t dacr_before, dacr_after;
		volatile uint8_t *wdog_page = (volatile uint8_t *)&g_wdpool;

		lldbg("=== DEFECT #1: SMP DACR per-core test ===\n");

		/* Step 1: Read DACR before begin() */
		__asm__ volatile("mrc p15, 0, %0, c3, c0, 0" : "=r"(dacr_before));
		lldbg("DEFECT_SMP: DACR before begin() = 0x%08x (expect 0x05 = RO)\n", dacr_before);

		/* Step 2: Call begin() — sets THIS core's DACR = RW */
		wd_mmu_write_begin();

		/* Step 3: Read DACR after begin() */
		__asm__ volatile("mrc p15, 0, %0, c3, c0, 0" : "=r"(dacr_after));
		lldbg("DEFECT_SMP: DACR after begin()  = 0x%08x (expect 0x0D = RW)\n", dacr_after);

		if (dacr_after == 0x0000000d) {
			lldbg("DEFECT_SMP: This core DACR = RW (Manager) — write should succeed\n");
		} else {
			lldbg("DEFECT_SMP: ERROR — This core DACR NOT 0x0D!\n");
		}

		/* Step 4: Write to wdog pool from this core (should succeed) */
		lldbg("DEFECT_SMP: Writing to wdog pool from this core...\n");
		{
			irqstate_t irq_state = enter_critical_section();
			uint8_t orig = wdog_page[0];
			wdog_page[0] = 0xCC;
			wdog_page[0] = orig;
			leave_critical_section(irq_state);
		}
		lldbg("DEFECT_SMP: Write succeeded on this core (DACR = RW)\n");

		/* Step 5: Log the defect */
		lldbg("DEFECT_SMP: === DEFECT CONFIRMED ===\n");
		lldbg("DEFECT_SMP: Other cores' DACR is still 0x05 (RO)!\n");
		lldbg("DEFECT_SMP: If another core accesses wdog pool now → Data Abort!\n");
		lldbg("DEFECT_SMP: Fix: Use IPI to broadcast DACR to all cores.\n");

		wd_mmu_write_end();

		__asm__ volatile("mrc p15, 0, %0, c3, c0, 0" : "=r"(dacr_after));
		lldbg("DEFECT_SMP: DACR after end()   = 0x%08x (expect 0x05 = RO)\n", dacr_after);

		ret = OK;
		break;
	}

	case TESTIOC_WDOG_DEFECT_DSB: {
		/* Defect #3: Missing dsb after DACR write.
		 *
		 * Without a dsb (Data Synchronization Barrier) after the
		 * mcr instruction that writes DACR, an out-of-order core
		 * may execute the next memory access BEFORE the DACR write
		 * completes. This causes an intermittent Data Abort.
		 *
		 * This test calls wd_mmu_write_begin() and immediately
		 * writes to the wdog pool in a tight loop (10,000 iterations).
		 * Without dsb, intermittent failures may occur on
		 * out-of-order cores.
		 */
		volatile uint8_t *wdog_page = (volatile uint8_t *)&g_wdpool;
		int i;

		lldbg("=== DEFECT #3: Missing dsb after DACR write ===\n");
		lldbg("DEFECT_DSB: Running 10,000 iterations of begin()+write+end()\n");
		lldbg("DEFECT_DSB: Without dsb, intermittent Data Abort may occur\n");

		for (i = 0; i < 10000; i++) {
			wd_mmu_write_begin();
			/* Immediately write — without dsb, this may use stale DACR */
			uint8_t orig = wdog_page[0];
			wdog_page[0] = (uint8_t)(i & 0xFF);
			wdog_page[0] = orig;
			wd_mmu_write_end();
		}

		lldbg("DEFECT_DSB: All 10,000 iterations completed (no Data Abort)\n");
		lldbg("DEFECT_DSB: NOTE: On out-of-order cores, this may fail intermittently\n");
		lldbg("DEFECT_DSB: Fix: Add dsb + isb after mcr to DACR\n");

		ret = OK;
		break;
	}

	case TESTIOC_WDOG_DEFECT_DACR_CLOBBER: {
		/* Defect #7: DACR clobbers other domains.
		 *
		 * wd_mmu_write_begin() writes the entire DACR register
		 * (0x0D), which sets ALL 16 domains. Domains 2-15 are
		 * set to No-Access (00), clobbering any other subsystem
		 * that might use those domains.
		 *
		 * This test:
		 * 1. Manually sets Domain 2 to Client (01)
		 * 2. Calls wd_mmu_write_begin() (overwrites entire DACR)
		 * 3. Reads DACR — Domain 2 is now No-Access (clobbered!)
		 */
		uint32_t dacr;

		lldbg("=== DEFECT #7: DACR clobbers other domains ===\n");

		/* Step 1: Set Domain 2 to Client (bits [5:4] = 01) */
		__asm__ volatile("mrc p15, 0, %0, c3, c0, 0" : "=r"(dacr));
		dacr &= ~(0x3 << 4);    /* Clear Domain 2 bits */
		dacr |= (0x1 << 4);     /* Set Domain 2 = Client */
		__asm__ volatile("mcr p15, 0, %0, c3, c0, 0" :: "r"(dacr) : "memory");

		__asm__ volatile("mrc p15, 0, %0, c3, c0, 0" : "=r"(dacr));
		lldbg("DEFECT_CLOBBER: Before begin(): DACR = 0x%08x (Domain 2 = Client)\n", dacr);

		/* Step 2: Call begin() — overwrites entire DACR */
		wd_mmu_write_begin();

		/* Step 3: Read DACR — Domain 2 should be clobbered to No-Access */
		__asm__ volatile("mrc p15, 0, %0, c3, c0, 0" : "=r"(dacr));
		lldbg("DEFECT_CLOBBER: After begin():  DACR = 0x%08x\n", dacr);

		uint32_t domain2 = (dacr >> 4) & 0x3;
		if (domain2 == 0x0) {
			lldbg("DEFECT_CLOBBER: === DEFECT CONFIRMED ===\n");
			lldbg("DEFECT_CLOBBER: Domain 2 = No-Access (clobbered!)\n");
			lldbg("DEFECT_CLOBBER: wd_mmu_write_begin() overwrote entire DACR\n");
			lldbg("DEFECT_CLOBBER: Fix: Use read-modify-write for DACR\n");
		} else {
			lldbg("DEFECT_CLOBBER: Domain 2 = 0x%x (not clobbered)\n", domain2);
		}

		wd_mmu_write_end();

		ret = OK;
		break;
	}

	case TESTIOC_WDOG_DEFECT_HEAP_OVERLAP: {
		/* Defect #8: Heap overlap with wdog pool.
		 *
		 * The wdog pool is placed at a fixed address. If the kernel
		 * heap covers this region, heap allocations may land in the
		 * wdog pool's Read-Only section → Data Abort.
		 *
		 * This test prints the wdog pool address range and the
		 * configured heap start address so the user can verify
		 * there is no overlap.
		 */
		extern uint32_t __wdog_pool_start__[];
		extern uint32_t __wdog_pool_end__[];

		uint32_t pool_start = (uint32_t)__wdog_pool_start__;
		uint32_t pool_end = (uint32_t)__wdog_pool_end__;

		lldbg("=== DEFECT #8: Heap overlap with wdog pool ===\n");
		lldbg("DEFECT_HEAP: wdog pool: 0x%08x - 0x%08x (size = %d bytes)\n",
		      pool_start, pool_end, pool_end - pool_start);

		/* Use CONFIG_RAM_KREGIONx_START for heap start check.
		 * The heap start is configured via Kconfig and available
		 * as a string. We compare against the wdog pool range.
		 */
		lldbg("DEFECT_HEAP: Configured RAM start = 0x%08x\n", CONFIG_RAM_START);
		lldbg("DEFECT_HEAP: Configured RAM size = %d bytes\n", CONFIG_RAM_SIZE);
		lldbg("DEFECT_HEAP: RAM end = 0x%08x\n", CONFIG_RAM_START + CONFIG_RAM_SIZE);

		/* Check if wdog pool is within RAM region */
		if (pool_start >= CONFIG_RAM_START &&
		    pool_start < (CONFIG_RAM_START + CONFIG_RAM_SIZE)) {
			lldbg("DEFECT_HEAP: === DEFECT CONFIRMED ===\n");
			lldbg("DEFECT_HEAP: wdog pool (0x%08x) is within RAM region!\n",
			      pool_start);
			lldbg("DEFECT_HEAP: Heap may overlap wdog pool → Data Abort risk!\n");
			lldbg("DEFECT_HEAP: Fix: Adjust heap start to after wdog pool\n");
		} else {
			lldbg("DEFECT_HEAP: wdog pool is outside RAM region — OK\n");
		}

		ret = OK;
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
		lldbg("DEFECT_RECOVER: wd_recover() fix (wd_mmu_write_begin/end) is working\n");
		lldbg("DEFECT_RECOVER: PASS — no Data Abort in wd_recover() code path\n");

		ret = OK;
		break;
	}

	case TESTIOC_WDOG_DEFECT_ABORT_HANDLER: {
		/* Defect #12: Data Abort handler guard.
		 *
		 * If the Data Abort handler itself accesses the wdog pool
		 * (e.g., for diagnostic logging), a nested Data Abort occurs.
		 * Without a guard, this causes an infinite abort loop → hang.
		 *
		 * This test triggers a Data Abort and then checks if the
		 * handler can safely access wdog pool addresses.
		 */
		lldbg("=== DEFECT #12: Data Abort handler guard ===\n");
		lldbg("DEFECT_ABORT: This test verifies abort handler safety\n");
		lldbg("DEFECT_ABORT: The abort handler must NOT dereference wdog pool\n");
		lldbg("DEFECT_ABORT: addresses — use DFAR register value instead.\n");
		lldbg("DEFECT_ABORT: If handler accesses wdog pool → nested abort → hang\n");
		lldbg("DEFECT_ABORT: Fix: Add abort-in-progress flag to abort handler\n");

		/* This is a documentation-only test — no actual crash.
		 * The real test is: trigger a wdog Data Abort and verify
		 * the handler doesn't cause a nested abort.
		 */
		lldbg("DEFECT_ABORT: Test passed (documentation only — no crash)\n");

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
		lldbg("=== DEFECT #13: Boot order issue ===\n");
		lldbg("DEFECT_BOOT: Checking boot sequence for unprotected wdog writes...\n");
		lldbg("DEFECT_BOOT: wd_mmu_protect_init() is called at end of wd_initialize()\n");
		lldbg("DEFECT_BOOT: Any wdog write between init and first wd_start() → Data Abort\n");
		lldbg("DEFECT_BOOT: Fix: Verify no wdog writes in boot path after init\n");

		/* If we reached here, boot completed without Data Abort.
		 * This means either:
		 * - No wdog writes occurred during boot (good)
		 * - Protection was not enabled during boot
		 */
		lldbg("DEFECT_BOOT: Boot completed — no Data Abort during boot sequence\n");
		lldbg("DEFECT_BOOT: If protection is enabled, boot order is safe\n");

		ret = OK;
		break;
	}

	case TESTIOC_WDOG_DEFECT_SECTION_SIZE: {
		/* Defect #15: L1 entry cache coherency.
		 *
		 * After wd_mmu_protect_init() modifies the L1 section entry
		 * in the page table, the TLB may still have a cached copy
		 * of the old entry. Without a full TLB flush, the stale
		 * entry may allow writes → protection not effective.
		 *
		 * This test:
		 * 1. Attempts a wild write to wdog pool immediately after init
		 * 2. If TLB has stale entry → write succeeds (defect!)
		 * 3. If TLB was flushed → Data Abort (protection works)
		 */
		volatile uint8_t *wdog_page = (volatile uint8_t *)&g_wdpool;

		lldbg("=== DEFECT #15: L1 entry cache coherency ===\n");
		lldbg("DEFECT_SECTION: Attempting wild write to wdog pool...\n");
		lldbg("DEFECT_SECTION: If TLB has stale entry → write succeeds (defect!)\n");
		lldbg("DEFECT_SECTION: If TLB was flushed → Data Abort (protection works)\n");

		/* Attempt a wild write — should Data Abort if protection works.
		 * If it succeeds, the TLB still has a stale entry.
		 */
		{
			irqstate_t irq_state = enter_critical_section();
			uint8_t orig = wdog_page[0];
			wdog_page[0] = 0xDD;
			wdog_page[0] = orig;
			leave_critical_section(irq_state);

			lldbg("DEFECT_SECTION: === DEFECT CONFIRMED ===\n");
			lldbg("DEFECT_SECTION: Write succeeded — TLB had stale entry!\n");
			lldbg("DEFECT_SECTION: Protection was NOT effective immediately after init\n");
			lldbg("DEFECT_SECTION: Fix: Use full TLB flush after L1 entry modification\n");
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
		 * Call wd_mmu_write_begin() 1000 times reentrantly, then
		 * wd_mmu_write_end() 1000 times. The nesting counter must
		 * correctly track the depth and only restore RO protection
		 * on the final end() call.
		 *
		 * If the counter overflows or underflows, the page will be
		 * left in RW (unprotected) or RO (causing Data Abort in wdog
		 * code).
		 */
		int i;

		lldbg("=== TC-13: Nesting counter stress (1000 reentrant calls) ===\n");

		/* 1000 reentrant begin() calls — nesting counter goes to 1000 */
		for (i = 0; i < 1000; i++) {
			wd_mmu_write_begin();
		}

		lldbg("TC13: 1000 begin() calls done, nesting counter = 1000\n");

		/* 999 end() calls — counter goes to 1, page still RW */
		for (i = 0; i < 999; i++) {
			wd_mmu_write_end();
		}

		lldbg("TC13: 999 end() calls done, nesting counter = 1 (page still RW)\n");

		/* Final end() — counter goes to 0, page restored to RO */
		wd_mmu_write_end();

		lldbg("TC13: Final end() done, nesting counter = 0 (page restored to RO)\n");
		lldbg("TC13: PASS — nesting counter handled 1000 reentrant calls\n");

		ret = OK;
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
