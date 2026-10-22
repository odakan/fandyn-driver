/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2026 deniz <onurdenizakan@gmail.com> */
/* LD_PRELOAD shim: counts heap calls and reports them at exit (leak/heap proof). */
#define _GNU_SOURCE
#include <stddef.h>
#include <stdio.h>
#include <unistd.h>

extern void *__libc_malloc(size_t);
extern void *__libc_calloc(size_t, size_t);
extern void *__libc_realloc(void *, size_t);
extern void __libc_free(void *);

static unsigned long n_alloc, n_free;

void *malloc(size_t n) { n_alloc++; return __libc_malloc(n); }
void *calloc(size_t a, size_t b) { n_alloc++; return __libc_calloc(a, b); }
void *realloc(void *p, size_t n) { if (!p) n_alloc++; return __libc_realloc(p, n); }
void free(void *p) { if (p) n_free++; __libc_free(p); }

__attribute__((destructor)) static void report(void)
{
    char b[96];
    int l = snprintf(b, sizeof b, "ALLOC_COUNT allocs=%lu frees=%lu\n", n_alloc, n_free);
    if (l > 0 && write(2, b, (size_t)l) < 0) { }
}
