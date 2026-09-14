# kasan_test

Deliberately corrupts memory so that KASan can be shown to catch it.

A board with no memory errors and a board where the instrumentation was never
emitted both boot silently. A clean boot therefore says nothing on its own
about whether KASan is working. This is what tells the two apart.

**A report is the pass. Silence is the failure.**

## Enabling

Set `CONFIG_MM_KASAN_SELFTEST` under Memory Management. It builds both the
kernel side (`os/kernel/debug/kasan_selftest.c`) and this app. It depends on
`CONFIG_MM_KASAN_INSTRUMENT`, so KASan has to be on and in a checking mode
first.

Turn it off once the port is trusted on the board. Never ship it.

## Why the work happens in the kernel

The faulting access is made by the kernel, against a block from the kernel
heap, and this app only asks for it over `prctl(PR_KASAN_SELFTEST, ...)`.

In a protected build user space allocates from a heap that is deliberately
never registered with KASan, and user code is not instrumented. A test written
entirely in user space would report nothing however broken the memory manager
was, and would read as a clean pass.

The kernel side also cannot live under `os/mm/`. Everything there is compiled
with `-fno-sanitize=kernel-address` so that the sanitizer does not instrument
itself, which means an access made from there is never checked.

## Running

Run one case per boot. A caught case asserts, and the board resets if it is
configured to reset on assert.

```
TASH>> kasan_test
```
with no argument lists the cases.

```
TASH>> kasan_test inbounds
```

Start here. It writes to the last accessible byte of a block and must stay
silent:

```
kasan_test: inbounds - write inside the block
kasan_test: expecting no report
KASAN-SELFTEST: block 0x6019a3c0, requested 64 bytes, 64 accessible
KASAN-SELFTEST: case 0: write at offset 63, inside the block
KASAN-SELFTEST: case 0: no report, as expected
kasan_test: PASS - 'inbounds' completed with no report.
```

If this one reports, the allocator hook is unpoisoning less than it hands out
and no other result is meaningful.

```
TASH>> kasan_test overflow
```

Then any of `overflow`, `read`, `underflow`, `uaf`, `straddle`. Each should
produce a report naming the address, the access size and the direction,
followed by an assert:

```
KASAN-SELFTEST: case 1: write at offset 64, one past the block
KASAN: invalid write of size 1 at address 0x6019a400
KASAN: detected from 0x600a1f3c
KASAN: memory around the faulty address:
...
```

`straddle` is worth running even when the others pass. It is a four byte store
that begins in the last accessible word and ends in the first poisoned one,
which a shadow test that looks only at the size of an access misses.

## Reading a silent result

| Case | Silent means |
|---|---|
| `inbounds` | correct |
| `read` | correct **only** if `CONFIG_MM_KASAN_DISABLE_READS_CHECK` is set |
| any other | the kernel was not built with `-fsanitize=kernel-address`, or the heap it allocated from was never registered |

## Accessible size is not requested size

`mm_malloc()` rounds the request up with `MM_ALIGN_UP(size +
SIZEOF_MM_ALLOCNODE)` and hands back the whole chunk, and the KASan hook
unpoisons everything past the node header. So this port reports an overflow
past the end of the **chunk**, not past the end of the **request**, and the
slack between the two is not covered.

On rtl8730e with `CONFIG_DEBUG_MM_HEAPINFO` and `CONFIG_DEBUG_MM_FREEINFO`
set, `SIZEOF_MM_ALLOCNODE` is 16 and `MM_ALIGN_UP` rounds to 16, so:

| request | chunk | accessible | slack |
|---|---|---|---|
| 56 | 80 | 64 | 8 |
| 60 | 80 | 64 | 4 |
| 64 | 80 | 64 | 0 |
| 72 | 96 | 80 | 8 |

The 64 byte request this test uses happens to land exactly on a boundary, so
its overflow is one byte past what was asked for. A 60 byte request would have
four accessible bytes beyond the request that KASan would not report on.

This is why the test computes the payload extent from `MM_ALIGN_UP` and
`SIZEOF_MM_ALLOCNODE` rather than assuming the requested size: it uses the
same expression the allocator and the Phase 2 hook use, so it stays correct if
either constant changes with the configuration.
