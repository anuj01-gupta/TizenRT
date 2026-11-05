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
/****************************************************************************
 * apps/examples/mmu_protect_test/mmu_protect_test_main.c
 *
 * Use-after-free detection test using MMU page protection.
 *
 * This test triggers a real use-after-free scenario:
 *   1. A page-aligned buffer is allocated in kernel space (via the
 *      os_api_test driver ioctl).
 *   2. The page is set to No-Access via mmu_set_page_no_access() —
 *      simulating a "free" that makes the memory inaccessible.
 *   3. The freed (protected) memory is then read or written.
 *   4. The MMU detects the illegal access and triggers a Data Abort.
 *
 * The Data Abort handler prints PC/DFAR/DFSR.  DFAR contains the address
 * of the freed memory that was illegally accessed — this confirms the
 * use-after-free was detected.
 *
 * The Data Abort is fatal: the system crashes/reboots (or recovers via
 * binary manager if CONFIG_BINMGR_RECOVERY is enabled).
 *
 * Usage from TASH:
 *   mmu_protect_test read    — UAF via read from freed page (Data Abort)
 *   mmu_protect_test write   — UAF via write to freed page (Data Abort)
 *
 * NOTE: Requires CONFIG_DRIVERS_OS_API_TEST and CONFIG_EXAMPLES_MMU_PROTECT_TEST.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>

#include <sys/ioctl.h>

#include <tinyara/fs/ioctl.h>

#ifdef CONFIG_DRIVERS_OS_API_TEST
#include <tinyara/os_api_test_drv.h>
#else
/* Fallback definitions if the OS API test driver config is not enabled.
 * The ioctl numbers must match those in os_api_test_drv.h.
 */
#define OS_API_TEST_DRVPATH        "/dev/os_api_test"
#define _TESTIOC(nr)               _IOC(_TESTIOCBASE, nr)
#define TESTIOC_UAF_MMU_PROTECT    _TESTIOC(25)
#define TESTIOC_UAF_NO_MMU         _TESTIOC(26)
#define TESTIOC_WDOG_MMU_PROTECT_TEST _TESTIOC(27)
#endif

/* Fallbacks, for a build without CONFIG_DRIVERS_OS_API_TEST.  When the driver
 * header is present it stays the only source of these numbers, so no TESTIOC_*
 * name is defined twice.  The defect group also follows the header's guard
 * (CONFIG_WDOG_MMU_DEFECT_TEST, not the scenario one) and the header's current
 * name for id 30: TESTIOC_WDOG_DEFECT_DOMAIN_KEEP.  Id 30 was sent out under
 * the retired DACR-era name, which made "defect_dacr_clobber" quietly run the
 * domain-keep case while printing its own label. */

#if defined(CONFIG_WDOG_MMU_DEFECT_TEST) && !defined(CONFIG_DRIVERS_OS_API_TEST)
#ifndef _TESTIOC
#define _TESTIOC(nr)                _IOC(_TESTIOCBASE, nr)
#endif
#ifndef OS_API_TEST_DRVPATH
#define OS_API_TEST_DRVPATH         "/dev/os_api_test"
#endif
#define TESTIOC_WDOG_DEFECT_SMP           _TESTIOC(28)
#define TESTIOC_WDOG_DEFECT_DSB           _TESTIOC(29)
#define TESTIOC_WDOG_DEFECT_DOMAIN_KEEP   _TESTIOC(30)
#define TESTIOC_WDOG_DEFECT_HEAP_OVERLAP  _TESTIOC(31)
#define TESTIOC_WDOG_DEFECT_WD_RECOVER    _TESTIOC(32)
#define TESTIOC_WDOG_DEFECT_ABORT_HANDLER _TESTIOC(33)
#define TESTIOC_WDOG_DEFECT_BOOT_ORDER    _TESTIOC(34)
#define TESTIOC_WDOG_DEFECT_SECTION_SIZE  _TESTIOC(35)
#endif

#if defined(CONFIG_WDOG_MMU_PROTECT) && !defined(CONFIG_DRIVERS_OS_API_TEST)
#ifndef _TESTIOC
#define _TESTIOC(nr)                _IOC(_TESTIOCBASE, nr)
#endif
#ifndef OS_API_TEST_DRVPATH
#define OS_API_TEST_DRVPATH         "/dev/os_api_test"
#endif
#define TESTIOC_WDOG_MMU_WINDOW_STATE     _TESTIOC(58)
#define TESTIOC_WDOG_MMU_PAGE_REPORT      _TESTIOC(59)
#define TESTIOC_WDOG_MMU_REFUSE_SECTION   _TESTIOC(60)
#define TESTIOC_WDOG_MMU_WINDOW_PAIR      _TESTIOC(61)
#endif

#if defined(CONFIG_WDOG_MMU_SCENARIO_TEST) && !defined(CONFIG_DRIVERS_OS_API_TEST)
#ifndef _TESTIOC
#define _TESTIOC(nr)               _IOC(_TESTIOCBASE, nr)
#endif
#ifndef OS_API_TEST_DRVPATH
#define OS_API_TEST_DRVPATH        "/dev/os_api_test"
#endif
#define TESTIOC_WDOG_SCENARIO_NORMAL_OP      _TESTIOC(36)
#define TESTIOC_WDOG_SCENARIO_MULTI_EXPIRE    _TESTIOC(37)
#define TESTIOC_WDOG_SCENARIO_SMP_CONCURRENT _TESTIOC(38)
#define TESTIOC_WDOG_SCENARIO_SMP_TICK_CB     _TESTIOC(39)
#define TESTIOC_WDOG_SCENARIO_NESTING_STRESS  _TESTIOC(40)
#define TESTIOC_WDOG_SCENARIO_LONG_RUN        _TESTIOC(41)
#endif /* CONFIG_WDOG_MMU_SCENARIO_TEST */

#if defined(CONFIG_MMU_PAGE_PROTECT) && !defined(CONFIG_DRIVERS_OS_API_TEST)
#ifndef _TESTIOC
#define _TESTIOC(nr)               _IOC(_TESTIOCBASE, nr)
#endif
#ifndef OS_API_TEST_DRVPATH
#define OS_API_TEST_DRVPATH        "/dev/os_api_test"
#endif
#define TESTIOC_AL_SPLIT              _TESTIOC(42)
#define TESTIOC_AL_NOACCESS_RESTORE   _TESTIOC(43)
#define TESTIOC_AL_RO_RESTORE         _TESTIOC(44)
#define TESTIOC_AL_NOACCESS_ABORT     _TESTIOC(45)
#define TESTIOC_AL_SPLIT_PROTECT      _TESTIOC(46)
#define TESTIOC_AL_SMP_NOACCESS      _TESTIOC(47)
#define TESTIOC_AL_SMP_RO             _TESTIOC(48)
#define TESTIOC_AL_SMP_RESTORE        _TESTIOC(49)
#define TESTIOC_AL_SMP_SPLIT          _TESTIOC(50)
#define TESTIOC_AL_R1_SPLIT_CORRECTNESS _TESTIOC(51)
#define TESTIOC_MMU_L2_POOL_USAGE       _TESTIOC(62)
#ifdef CONFIG_MMU_FAULT_REPORT
#define TESTIOC_MMU_REGION_REPORT     _TESTIOC(52)
#endif
#endif /* CONFIG_MMU_PAGE_PROTECT */

/* ioctl arg values: 0 = read test, 1 = write test */

#define UAF_TEST_READ   0
#define UAF_TEST_WRITE  1



/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mmu_protect_test_usage(void)
{
	printf("Usage: mmu_protect_test <command>\n");
	printf("\n");
	printf("UAF Tests:\n");
	printf("  read       — Trigger UAF by reading from freed (protected) page\n");
	printf("               MMU protection enabled → Data Abort (system crash)\n");
	printf("  write      — Trigger UAF by writing to freed (protected) page\n");
	printf("               MMU protection enabled → Data Abort (system crash)\n");
	printf("  nouaf      — UAF without MMU protection (no crash)\n");
	printf("               free() the buffer, then read freed memory.\n");
	printf("               Access succeeds silently — UAF goes undetected.\n");
	printf("               Shows why MMU protection is needed.\n");
	printf("  wdog       — Read from wdog protected page (safe, no crash)\n");
	printf("               Verifies wdog page is accessible for reads.\n");
	printf("  wdog_write — Write to wdog protected page from outside wdog code\n");
	printf("               If CONFIG_WDOG_MMU_PROTECT=y → Data Abort (crash)\n");
	printf("               If CONFIG_WDOG_MMU_PROTECT=n → write succeeds (no crash)\n");
#ifdef CONFIG_WDOG_MMU_PROTECT
	printf("\n");
	printf("Watchdog write-window tests (read-only, safe any time):\n");
	printf("  wdog_state   — window open/closed, owner CPU, nesting depth,\n");
	printf("                 wdog failure counter + discarded-status counter\n");
	printf("  wdog_page    — live L2 descriptor of the protected page (PTE, AP)\n");
	printf("  wdog_refuse  — a section-mapped address must be refused (-EINVAL)\n");
	printf("  wdog_pair N  — N begin()/end() pairs, a nested pair, one end() too\n");
	printf("                 many.  PASS only if both failure counters stayed at\n");
	printf("                 zero, so it asserts instead of just surviving.\n");
#endif
#ifdef CONFIG_WDOG_MMU_SCENARIO_TEST
	printf("\n");
	printf("Defect Reproduction Tests (requires CONFIG_WDOG_MMU_SCENARIO_TEST):\n");
	printf("  defect_smp            — Defect #1: SMP DACR per-core issue\n");
	printf("  defect_dsb            — Defect #3: Missing dsb after DACR write\n");
	printf("  defect_domain_keep    — Defect #7: domain kept across a write window\n");
	printf("                        (the old name defect_dacr_clobber still works)\n");
	printf("  defect_heap_overlap   — Defect #8: Heap overlap with wdog pool\n");
	printf("  defect_wd_recover     — Defect #11: wd_recover() missing protection\n");
	printf("  defect_abort_handler  — Defect #12: Data Abort handler guard (assertive, non-fatal)\n");
	printf("  defect_boot_order     — Defect #13: Boot order issue\n");
	printf("  defect_section_size   — Defect #15: L1 entry cache coherency\n");
#endif
#ifdef CONFIG_WDOG_MMU_SCENARIO_TEST
	printf("\n");
	printf("Scenario Tests (requires CONFIG_WDOG_MMU_SCENARIO_TEST):\n");
	printf("  scenario_normal       — TC-05: Normal wdog operation (100x start/cancel)\n");
	printf("  scenario_multi_expire — TC-06: Multiple watchdog expiration in same tick\n");
	printf("  scenario_smp_concurrent — TC-08: SMP concurrent wdog access (1000x)\n");
	printf("  scenario_smp_tick_cb — TC-09: SMP timer tick / callback cross-core\n");
	printf("  scenario_nesting     — TC-13: Nesting counter stress (1000 reentrant)\n");
	printf("  scenario_long_run    — TC-15: Long-run stability (60s default)\n");
	printf("                          arg: 0=60s, 1=300s, 2=86400s (24h)\n");
#endif
#ifdef CONFIG_MMU_PAGE_PROTECT
	printf("\n");
	printf("AL Layer Tests (requires CONFIG_MMU_PAGE_PROTECT):\n");
	printf("  al_split              - TC-AL-01: Section split (safe)\n");
	printf("  al_r1_split           - TC-AL-R1: Split correctness - 256 L2 entries (safe)\n");
	printf("  al_noaccess_restore   - TC-AL-02: No-access + restore (safe)\n");
	printf("  al_ro_restore         - TC-AL-03: RO + restore (safe)\n");
	printf("  al_noaccess_abort     - TC-AL-04: Section -> refused; already-4KB page -> accepted, AP=0 proven, restored (safe)\n");
	printf("  al_split_protect      - TC-AL-05: Split + 4KB protect -> Data Abort (fatal)\n");
	printf("  l2_pool               - TC-AL-10: L2 page-table pool accounting (safe)\n");
#ifdef CONFIG_SMP
	printf("  al_smp_noaccess       - TC-AL-06: SMP no-access -> Data Abort (fatal)\n");
	printf("  al_smp_ro             - TC-AL-07: SMP RO write -> Data Abort (fatal)\n");
	printf("  al_smp_restore        - TC-AL-08: SMP restore (safe)\n");
	printf("  al_smp_split          - TC-AL-09: SMP section split (safe)\n");
#endif
#ifdef CONFIG_MMU_FAULT_REPORT
	printf("  region_report         - TC-R2: Fault report probe - register+protect+corrupt (fatal)\n");
#endif
#ifdef CONFIG_MMU_PAGE_PROTECT
	printf("  tlb_bench             - TC-R3: TLB/split overhead benchmark (safe, ~5s)\n");
#endif
#ifdef CONFIG_MM_HEAP_UAF_PROTECT
	printf("  heap_protect [1-5]  - TC-R4: allocator UAF protection (1-3 safe, 4 fatal, 5 endurance <minutes>)\n");
#endif
#endif
	printf("\n");


	printf("  read/write/wdog_write cause a fatal Data Abort.  Run separately.\n");
	printf("  nouaf and wdog do NOT crash — safe to run anytime.\n");
}




/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmu_protect_test_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int mmu_protect_test_main(int argc, char *argv[])
#endif
{
	int fd;
	int ret;
	unsigned long ioctl_arg;

	if (argc < 2) {
		mmu_protect_test_usage();
		return ERROR;
	}

	if (strcmp(argv[1], "read") == 0) {
		ioctl_arg = UAF_TEST_READ;
	} else if (strcmp(argv[1], "write") == 0) {
		ioctl_arg = UAF_TEST_WRITE;
	} else if (strcmp(argv[1], "nouaf") == 0) {
		/* No-MMU mode: uses a different ioctl command */
		printf("=== UAF Test WITHOUT MMU Protection ===\n");
		printf("[mmu_protect_test] Opening %s...\n", OS_API_TEST_DRVPATH);

		fd = open(OS_API_TEST_DRVPATH, O_RDWR);
		if (fd < 0) {
			printf("[mmu_protect_test] FAIL: cannot open %s (errno %d)\n",
			       OS_API_TEST_DRVPATH, errno);
			printf("[mmu_protect_test] Is CONFIG_DRIVERS_OS_API_TEST enabled?\n");
			return ERROR;
		}

		printf("[mmu_protect_test] UAF without MMU protection (no crash expected)\n");

		/* This ioctl runs in kernel space and does:
		 *   1. Allocate a page-aligned buffer
		 *   2. Write data to it (normal use)
		 *   3. free() the buffer (real free, NO MMU protection)
		 *   4. Read from the freed memory — access succeeds silently
		 *   5. Write to the freed memory — also succeeds silently
		 *
		 * Without MMU protection, the UAF goes undetected.
		 * The test returns OK and demonstrates silent memory corruption.
		 */

		ret = ioctl(fd, TESTIOC_UAF_NO_MMU, 0);

		if (ret == OK) {
			printf("[mmu_protect_test] UAF went UNDETECTED without MMU protection.\n");
			printf("[mmu_protect_test] This demonstrates why MMU protection is needed.\n");
		} else {
			printf("[mmu_protect_test] ioctl returned %d (errno %d)\n", ret, errno);
		}

		close(fd);
		return ret;
	} else if (strcmp(argv[1], "wdog") == 0 || strcmp(argv[1], "wdog_write") == 0) {
		/* Watchdog MMU protection verification test */
		int do_write = (strcmp(argv[1], "wdog_write") == 0);

		printf("=== WDOG MMU Protection Test (%s) ===\n", argv[1]);
		printf("[mmu_protect_test] Opening %s...\n", OS_API_TEST_DRVPATH);

		fd = open(OS_API_TEST_DRVPATH, O_RDWR);
		if (fd < 0) {
			printf("[mmu_protect_test] FAIL: cannot open %s (errno %d)\n",
			       OS_API_TEST_DRVPATH, errno);
			printf("[mmu_protect_test] Is CONFIG_DRIVERS_OS_API_TEST enabled?\n");
			return ERROR;
		}

		if (do_write) {
			printf("[mmu_protect_test] Writing to wdog protected page from outside wdog code\n");
			printf("[mmu_protect_test] If CONFIG_WDOG_MMU_PROTECT=y → expect Data Abort (crash)\n");
			printf("[mmu_protect_test] If CONFIG_WDOG_MMU_PROTECT=n → write succeeds (no crash)\n");
		} else {
			printf("[mmu_protect_test] Reading from wdog protected page (safe, no crash)\n");
		}

		/* This ioctl runs in kernel space and does:
		 *   1. Read from g_wdpool (wdog static data page) — always succeeds

		 *   2. If arg=1: Write to g_wdpool without wd_mmu_write_begin()

		 *      → Data Abort if CONFIG_WDOG_MMU_PROTECT is enabled
		 *      → silent corruption if protection is disabled
		 *
		 * arg = 0: read only (safe)
		 * arg = 1: write (crash if protection enabled)
		 */
		ret = ioctl(fd, TESTIOC_WDOG_MMU_PROTECT_TEST, do_write);

		if (ret == OK) {
			if (do_write) {
				printf("[mmu_protect_test] Write to wdog page succeeded (no crash).\n");
				printf("[mmu_protect_test] CONFIG_WDOG_MMU_PROTECT is NOT enabled.\n");
				printf("[mmu_protect_test] Wdog page is writable — protection is OFF.\n");
				printf("[mmu_protect_test] Enable CONFIG_WDOG_MMU_PROTECT=y to get Data Abort on write.\n");
			} else {
				printf("[mmu_protect_test] Wdog page read test passed.\n");
			}
		} else {
			printf("[mmu_protect_test] ioctl returned %d (errno %d)\n", ret, errno);
		}


		close(fd);
		return ret;
	}
#ifdef CONFIG_WDOG_MMU_SCENARIO_TEST
	else if (strcmp(argv[1], "defect_smp") == 0 ||
	         strcmp(argv[1], "defect_dsb") == 0 ||
	         strcmp(argv[1], "defect_domain_keep") == 0 ||
	         strcmp(argv[1], "defect_dacr_clobber") == 0 ||
	         strcmp(argv[1], "defect_heap_overlap") == 0 ||
	         strcmp(argv[1], "defect_wd_recover") == 0 ||
	         strcmp(argv[1], "defect_abort_handler") == 0 ||
	         strcmp(argv[1], "defect_boot_order") == 0 ||
	         strcmp(argv[1], "defect_section_size") == 0) {

		int defect_cmd;

		if (strcmp(argv[1], "defect_smp") == 0) {
			defect_cmd = TESTIOC_WDOG_DEFECT_SMP;
		} else if (strcmp(argv[1], "defect_dsb") == 0) {
			defect_cmd = TESTIOC_WDOG_DEFECT_DSB;
		} else if (strcmp(argv[1], "defect_domain_keep") == 0 ||
			   strcmp(argv[1], "defect_dacr_clobber") == 0) {
			defect_cmd = TESTIOC_WDOG_DEFECT_DOMAIN_KEEP;
		} else if (strcmp(argv[1], "defect_heap_overlap") == 0) {
			defect_cmd = TESTIOC_WDOG_DEFECT_HEAP_OVERLAP;
		} else if (strcmp(argv[1], "defect_wd_recover") == 0) {
			defect_cmd = TESTIOC_WDOG_DEFECT_WD_RECOVER;
		} else if (strcmp(argv[1], "defect_abort_handler") == 0) {
			defect_cmd = TESTIOC_WDOG_DEFECT_ABORT_HANDLER;
		} else if (strcmp(argv[1], "defect_boot_order") == 0) {
			defect_cmd = TESTIOC_WDOG_DEFECT_BOOT_ORDER;
		} else {
			defect_cmd = TESTIOC_WDOG_DEFECT_SECTION_SIZE;
		}

		printf("=== WDOG MMU Defect Test (%s) ===\n", argv[1]);
		printf("[mmu_protect_test] Opening %s...\n", OS_API_TEST_DRVPATH);

		fd = open(OS_API_TEST_DRVPATH, O_RDWR);
		if (fd < 0) {
			printf("[mmu_protect_test] FAIL: cannot open %s (errno %d)\n",
			       OS_API_TEST_DRVPATH, errno);
			printf("[mmu_protect_test] Is CONFIG_DRIVERS_OS_API_TEST enabled?\n");
			return ERROR;
		}

		printf("[mmu_protect_test] Running defect reproduction test...\n");
		printf("[mmu_protect_test] Check debug log (lldbg) for DEFECT_ messages\n");

		ret = ioctl(fd, defect_cmd, 0);

		if (ret == OK) {
			printf("[mmu_protect_test] Defect test completed. Check debug log for results.\n");
		} else if (errno == ENOSYS) {
			/* A defect case the driver implements without a verdict
			 * reports -ENOSYS, and that must not read as a defect
			 * found.  id 33 takes this branch when
			 * CONFIG_WDOG_MMU_PROTECT or CONFIG_MMU_PAGE_PROTECT is
			 * off: there is no assertion left to make, only a
			 * configuration to state. */

			printf("[mmu_protect_test] Defect test NOT RUN (no assertion in driver)\n");
		} else {
			printf("[mmu_protect_test] ioctl returned %d (errno %d)\n", ret, errno);
		}

		close(fd);
		return ret;
	}
	else if (strcmp(argv[1], "scenario_normal") == 0 ||
	         strcmp(argv[1], "scenario_multi_expire") == 0 ||
	         strcmp(argv[1], "scenario_smp_concurrent") == 0 ||
	         strcmp(argv[1], "scenario_smp_tick_cb") == 0 ||
	         strcmp(argv[1], "scenario_nesting") == 0 ||
	         strcmp(argv[1], "scenario_long_run") == 0) {

		int scenario_cmd;
		unsigned long scenario_arg = 0;

		if (strcmp(argv[1], "scenario_normal") == 0) {
			scenario_cmd = TESTIOC_WDOG_SCENARIO_NORMAL_OP;
		} else if (strcmp(argv[1], "scenario_multi_expire") == 0) {
			scenario_cmd = TESTIOC_WDOG_SCENARIO_MULTI_EXPIRE;
		} else if (strcmp(argv[1], "scenario_smp_concurrent") == 0) {
			scenario_cmd = TESTIOC_WDOG_SCENARIO_SMP_CONCURRENT;
		} else if (strcmp(argv[1], "scenario_smp_tick_cb") == 0) {
			scenario_cmd = TESTIOC_WDOG_SCENARIO_SMP_TICK_CB;
		} else if (strcmp(argv[1], "scenario_nesting") == 0) {
			scenario_cmd = TESTIOC_WDOG_SCENARIO_NESTING_STRESS;
		} else {
			scenario_cmd = TESTIOC_WDOG_SCENARIO_LONG_RUN;
			/* long_run accepts optional duration arg */
			if (argc >= 3) {
				/* Manual string-to-int (atoi not available in kernel) */
				if (argv[2][0] >= '0' && argv[2][0] <= '9') {
					scenario_arg = (unsigned long)(argv[2][0] - '0');
				}
			}

		}

		printf("=== WDOG MMU Scenario Test (%s) ===\n", argv[1]);
		printf("[mmu_protect_test] Opening %s...\n", OS_API_TEST_DRVPATH);

		fd = open(OS_API_TEST_DRVPATH, O_RDWR);
		if (fd < 0) {
			printf("[mmu_protect_test] FAIL: cannot open %s (errno %d)\n",
			       OS_API_TEST_DRVPATH, errno);
			printf("[mmu_protect_test] Is CONFIG_DRIVERS_OS_API_TEST enabled?\n");
			return ERROR;
		}

		printf("[mmu_protect_test] Running scenario test...\n");
		printf("[mmu_protect_test] Check debug log (lldbg) for TC messages\n");

		ret = ioctl(fd, scenario_cmd, scenario_arg);

		if (ret == OK) {
			printf("[mmu_protect_test] Scenario test PASSED. Check debug log for details.\n");
		} else {
			printf("[mmu_protect_test] Scenario test FAILED (ret=%d, errno %d)\n",
			       ret, errno);
		}

		close(fd);
		return ret;
	}
#endif /* CONFIG_WDOG_MMU_SCENARIO_TEST */
#ifdef CONFIG_MMU_PAGE_PROTECT
	else if (strcmp(argv[1], "al_r1_split") == 0 ||
	         strcmp(argv[1], "al_split") == 0 ||
	         strcmp(argv[1], "al_noaccess_restore") == 0 ||
	         strcmp(argv[1], "al_ro_restore") == 0 ||
	         strcmp(argv[1], "al_noaccess_abort") == 0 ||
	         strcmp(argv[1], "al_split_protect") == 0 ||
	         strcmp(argv[1], "l2_pool") == 0
#ifdef CONFIG_SMP
	         || strcmp(argv[1], "al_smp_noaccess") == 0 ||
	         strcmp(argv[1], "al_smp_ro") == 0 ||
	         strcmp(argv[1], "al_smp_restore") == 0 ||
	         strcmp(argv[1], "al_smp_split") == 0
#endif
	         ) {

		int al_cmd;

		if (strcmp(argv[1], "al_r1_split") == 0) {
			al_cmd = TESTIOC_AL_R1_SPLIT_CORRECTNESS;
		} else if (strcmp(argv[1], "al_split") == 0) {
			al_cmd = TESTIOC_AL_SPLIT;
		} else if (strcmp(argv[1], "al_noaccess_restore") == 0) {
			al_cmd = TESTIOC_AL_NOACCESS_RESTORE;
		} else if (strcmp(argv[1], "al_ro_restore") == 0) {
			al_cmd = TESTIOC_AL_RO_RESTORE;
		} else if (strcmp(argv[1], "al_noaccess_abort") == 0) {
			al_cmd = TESTIOC_AL_NOACCESS_ABORT;
		} else if (strcmp(argv[1], "al_split_protect") == 0) {
			al_cmd = TESTIOC_AL_SPLIT_PROTECT;
		} else if (strcmp(argv[1], "l2_pool") == 0) {
			al_cmd = TESTIOC_MMU_L2_POOL_USAGE;
		}
#ifdef CONFIG_SMP
		else if (strcmp(argv[1], "al_smp_noaccess") == 0) {
			al_cmd = TESTIOC_AL_SMP_NOACCESS;
		} else if (strcmp(argv[1], "al_smp_ro") == 0) {
			al_cmd = TESTIOC_AL_SMP_RO;
		} else if (strcmp(argv[1], "al_smp_restore") == 0) {
			al_cmd = TESTIOC_AL_SMP_RESTORE;
		} else {
			al_cmd = TESTIOC_AL_SMP_SPLIT;
		}
#else
		else {
			al_cmd = TESTIOC_AL_SPLIT;
		}
#endif

		printf("=== AL Layer Test (%s) ===\n", argv[1]);
		printf("[mmu_protect_test] Opening %s...\n", OS_API_TEST_DRVPATH);

		fd = open(OS_API_TEST_DRVPATH, O_RDWR);
		if (fd < 0) {
			printf("[mmu_protect_test] FAIL: cannot open %s (errno %d)\n",
			       OS_API_TEST_DRVPATH, errno);
			printf("[mmu_protect_test] Is CONFIG_DRIVERS_OS_API_TEST enabled?\n");
			return ERROR;
		}

		if (strcmp(argv[1], "al_r1_split") == 0) {
			printf("[mmu_protect_test] Running R1 split correctness test (256 L2 entries)\n");
		} else {
			printf("[mmu_protect_test] Running AL test, check debug log (lldbg)\n");
		}

		ret = ioctl(fd, al_cmd, 0);

		if (ret == OK) {
			printf("[mmu_protect_test] AL test completed. Check debug log for results.\n");
		} else {
			printf("[mmu_protect_test] ioctl returned %d (errno %d)\n", ret, errno);
		}

		close(fd);
		return ret;
	}
#ifdef CONFIG_MMU_FAULT_REPORT
	else if (strcmp(argv[1], "region_report") == 0) {
		printf("=== MMU Fault Report Test (R2) ===\n");
		printf("[mmu_protect_test] Registers 'r2_test_page', protects it, then corrupts it.\n");
		printf("[mmu_protect_test] FATAL: expect Data Abort with full fault report before PANIC.\n");

		fd = open(OS_API_TEST_DRVPATH, O_RDWR);
		if (fd < 0) {
			printf("[mmu_protect_test] FAIL: cannot open %s (errno %d)\n",
			       OS_API_TEST_DRVPATH, errno);
			printf("[mmu_protect_test] Is CONFIG_DRIVERS_OS_API_TEST enabled?\n");
			return ERROR;
		}

		ret = ioctl(fd, TESTIOC_MMU_REGION_REPORT, 0);

		if (ret == OK) {
			printf("[mmu_protect_test] UNEXPECTED: survived the bad write - report not exercised?\n");
		} else {
			printf("[mmu_protect_test] ioctl returned %d (errno %d)\n", ret, errno);
		}

		close(fd);
		return ret;
	}
#endif /* CONFIG_MMU_FAULT_REPORT */

	else if (strcmp(argv[1], "tlb_bench") == 0) {
		struct mmu_tlb_bench_s res;

		printf("=== TLB / section-split benchmark (R3) ===\n");
		printf("[mmu_protect_test] Takes ~5s. A: 1MB sections, B: same data on 4KB pages.\n");

		fd = open(OS_API_TEST_DRVPATH, O_RDWR);
		if (fd < 0) {
			printf("[mmu_protect_test] FAIL: cannot open %s (errno %d)\n",
			       OS_API_TEST_DRVPATH, errno);
			return ERROR;
		}

		memset(&res, 0, sizeof(res));
		ret = ioctl(fd, TESTIOC_MMU_TLB_BENCH, (unsigned long)&res);

		if (ret == OK) {
			printf("--- TC-R3 report ---\n");
			printf("  baseline (1MB sections) : %u us\n", res.baseline_us);
			printf("  split    (4KB L2 pages) : %u us\n", res.split_us);
			printf("  overhead                : %u %%  (gate: <10%%)\n",
			       res.overhead_pct);
			printf("  one runtime split       : %u us\n", res.split_cost_us);
			printf("  protect+restore pair    : %u us (4 TLB syncs)\n",
			       res.op_cost_us);
#ifdef CONFIG_WDOG_MMU_PROTECT
			if (res.window_pair_us == 0) {
				printf("  wdog write window pair  : < 1 us (below timer resolution)\n");
			} else {
				printf("  wdog write window pair  : %u us (1 descriptor, both cores)\n",
				       res.window_pair_us);
			}
#endif
			printf("  L2 tables used          : %u\n", res.sections_split);
			printf("  budget after            : pool %u of %u claimed, %u held; save %u of %u pages (%u KB)\n",
			       res.pool_claimed, res.pool_total, res.pool_held,
			       res.saved_used, res.saved_max,
			       (unsigned int)(res.saved_max * 4));
			if (res.overhead_pct < 10) {
				printf(">>> R3 PASS: 4KB heap protection is affordable <<<\n");
			} else {
				printf(">>> R3 MITIGATION REQUIRED: overhead >= 10%% <<<\n");
			}
		} else {
			printf("[mmu_protect_test] ioctl returned %d (errno %d)\n", ret, errno);
		}

		close(fd);
		return ret;
	}
#ifdef CONFIG_MM_HEAP_UAF_PROTECT
	else if (strcmp(argv[1], "heap_protect") == 0) {
		int phase = 3;
		int minutes = 1;
		unsigned long iarg;

		if (argc > 2) {
			phase = argv[2][0] - '0';
		}
		if (phase == 5 && argc > 3) {
			minutes = atoi(argv[3]);
			if (minutes < 1) {
				minutes = 1;
			}
		}
		if (phase < 1 || phase > 5) {
			printf("[mmu_protect_test] heap_protect phase must be 1..5 (5: endurance, 4th arg = minutes)\n");
			return ERROR;
		}

		if (phase == 5) {
			printf("=== Heap UAF protection stress (R4), endurance %d minute(s) ===\n",
			       minutes);
		} else {
			printf("=== Heap UAF protection stress (R4), phase %d ===\n", phase);
		}
		if (phase == 4) {
			printf("[mmu_protect_test] FATAL: step 4 writes to freed+protected memory\n");
			printf("[mmu_protect_test] -> expect Data Abort + FAULT REPORT + PANIC.\n");
		}

		fd = open(OS_API_TEST_DRVPATH, O_RDWR);
		if (fd < 0) {
			printf("[mmu_protect_test] FAIL: cannot open %s (errno %d)\n",
			       OS_API_TEST_DRVPATH, errno);
			return ERROR;
		}

		iarg = (unsigned long)((phase == 5) ?
			(phase | (minutes << 8)) : phase);
		ret = ioctl(fd, TESTIOC_MMU_HEAP_PROTECT_STRESS, iarg);

		if (ret == OK) {
			printf("[mmu_protect_test] phase %d completed (details in debug log)\n",
			       phase);
			if (phase == 4) {
				printf("[mmu_protect_test] UNEXPECTED: no abort - not protected?\n");
			}
		} else {
			printf("[mmu_protect_test] ioctl returned %d (errno %d)\n", ret, errno);
		}

		close(fd);
		return ret;
	}
#endif /* CONFIG_MM_HEAP_UAF_PROTECT */
#endif /* CONFIG_MMU_PAGE_PROTECT */
#ifdef CONFIG_WDOG_MMU_PROTECT
	/* Write-window state (id 58) and the live descriptor of the protected page
	 * (id 59).  Both are read-only - no fault, no write, safe at any time.
	 * They exist so the SMP rules (owner, depth, failure counters) can be read
	 * from the shell instead of only from a rebuild with extra prints.
	 */
	else if (strcmp(argv[1], "wdog_state") == 0 ||
		 strcmp(argv[1], "wdog_page") == 0) {
		int cmd = (strcmp(argv[1], "wdog_state") == 0)
			? TESTIOC_WDOG_MMU_WINDOW_STATE : TESTIOC_WDOG_MMU_PAGE_REPORT;

		fd = open(OS_API_TEST_DRVPATH, O_RDWR);
		if (fd < 0) {
			printf("[mmu_protect_test] FAIL: cannot open %s (errno %d)\n",
			       OS_API_TEST_DRVPATH, errno);
			return ERROR;
		}

		ret = ioctl(fd, cmd, 0);
		printf("[mmu_protect_test] %s -> %d (errno %d), details in debug log\n",
		       argv[1], ret, errno);

		close(fd);
		return ret;

	/* Verdict commands: the driver decides and the ioctl carries the verdict.
	 * wdog_refuse (id 60) has to see -EINVAL for an address that is still a
	 * section; wdog_pair (id 61) has to leave both failure counters where
	 * they were, so a non-OK return here is a real defect, not a crash.
	 */
	} else if (strcmp(argv[1], "wdog_refuse") == 0 ||
		   strcmp(argv[1], "wdog_pair") == 0) {
		int cmd = TESTIOC_WDOG_MMU_REFUSE_SECTION;
		unsigned long arg = 0;

		if (strcmp(argv[1], "wdog_pair") == 0) {
			unsigned long n = 32;

			cmd = TESTIOC_WDOG_MMU_WINDOW_PAIR;

			if (argc > 2 && sscanf(argv[2], "%lu", &n) != 1) {
				printf("[mmu_protect_test] wdog_pair: bad count '%s'\n",
				       argv[2]);
				return ERROR;
			}

			if (n < 1 || n > 1000) {
				printf("[mmu_protect_test] wdog_pair count must be 1..1000\n");
				return ERROR;
			}

			arg = n;
		}

		fd = open(OS_API_TEST_DRVPATH, O_RDWR);
		if (fd < 0) {
			printf("[mmu_protect_test] FAIL: cannot open %s (errno %d)\n",
			       OS_API_TEST_DRVPATH, errno);
			return ERROR;
		}

		ret = ioctl(fd, cmd, arg);
		printf("[mmu_protect_test] %s -> %d : %s\n", argv[1], ret,
		       (ret == OK) ? "PASS" : "FAIL (see debug log)");

		close(fd);
		return ret;
	}
#endif /* CONFIG_WDOG_MMU_PROTECT */
	else {

		mmu_protect_test_usage();
		return ERROR;
	}




	printf("=== MMU UAF Detection Test (%s) ===\n", argv[1]);

	printf("[mmu_protect_test] Opening %s...\n", OS_API_TEST_DRVPATH);

	fd = open(OS_API_TEST_DRVPATH, O_RDWR);
	if (fd < 0) {
		printf("[mmu_protect_test] FAIL: cannot open %s (errno %d)\n",
		       OS_API_TEST_DRVPATH, errno);
		printf("[mmu_protect_test] Is CONFIG_DRIVERS_OS_API_TEST enabled?\n");
		return ERROR;
	}

	printf("[mmu_protect_test] Triggering use-after-free via MMU protection...\n");
	printf("[mmu_protect_test] >>> EXPECT DATA ABORT — system will crash <<<\n");

	/* This ioctl runs in kernel space and does:
	 *   1. Allocate a page-aligned buffer
	 *   2. Write data to it (normal use)
	 *   3. mmu_set_page_no_access() — make page inaccessible (simulate free)
	 *   4. Read or write the protected page → Data Abort (UAF detected)
	 *
	 * The Data Abort handler prints:
	 *   PC:   faulting instruction
	 *   DFAR: address of freed memory accessed
	 *   DFSR: permission fault status
	 *
	 * This function does NOT return — the Data Abort is fatal.
	 */
	ret = ioctl(fd, TESTIOC_UAF_MMU_PROTECT, ioctl_arg);

	/* If we reach here, the MMU protection did not work */
	printf("[mmu_protect_test] ERROR: ioctl returned %d, Data Abort was NOT triggered!\n",
	       ret);
	printf("[mmu_protect_test] MMU protection FAILED to detect use-after-free\n");

	close(fd);
	return ERROR;
}