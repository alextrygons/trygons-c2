/*
 * tc_stack.h - Stack high-water marks (step 13s): the fill pattern + a bounded scan. Pure C (no iLLD):
 * the board (hal_tc375.c: every core fills its own stacks at the start of core<n>_main, core 0 scans
 * all six for the BENCH TEST screen with stk_round; the board keeps only the addresses) and the PC
 * simulation (sim tests SK1-SK7 on arrays) use it.
 *
 * A stack grows DOWN: from its top (highest address) toward lo (the lowest word, the end of the
 * reserved block). Every word below the stack pointer is written with STK_PATTERN at the start; the
 * deepest use since then = the lowest word that no longer holds the pattern. stk_scan() looks at no
 * more than `budget` words per call (never a long loop in the control step) and walks up from lo:
 * the first word found used is the high-water mark, so a full pass costs at most the words below
 * the mark. The mark only ever moves down (deeper). The lowest word itself overwritten = the stack
 * reached its end (it may have run past it): overflow.
 * (A used word that happens to hold the pattern by chance is read as unused - the usual limit of
 * the method; the pattern is not a likely data value.)
 */
#ifndef TC_STACK_H
#define TC_STACK_H

#include <stdbool.h>
#include <stdint.h>

#define STK_PATTERN 0x5AC3A55Au     /* the fill word */

typedef struct {
    const volatile uint32_t *lo;    /* the lowest word of the block */
    uint32_t words;                 /* size of the block in words */
    uint32_t pos;                   /* next word to look at in this pass (index from lo) */
    uint32_t mark;                  /* index of the lowest used word found (words = none found yet) */
    uint32_t checked;               /* words looked at by the last stk_scan() call (tests: the budget) */
    bool     filled;                /* the block was filled at the start (else nothing is ever known) */
    bool     known;                 /* one full pass done: mark is the high-water mark */
} stk_scan_t;

/* write STK_PATTERN into lo[0 .. words-1] */
void     stk_fill(volatile uint32_t *lo, uint32_t words);
void     stk_init(stk_scan_t *s, const volatile uint32_t *lo, uint32_t words, bool filled);
/* look at no more than budget words (bounded); call it again and again */
void     stk_scan(stk_scan_t *s, uint32_t budget);
uint32_t stk_size_bytes(const stk_scan_t *s);
uint32_t stk_used_bytes(const stk_scan_t *s);   /* deepest use seen, bytes (0 until known) */
bool     stk_overflow(const stk_scan_t *s);     /* the lowest word was overwritten */
/* 13s review LOW-2: the scheduler over a set of n stacks (the board: 6, core 0's hal_get_diag): stk_scan on
 * set[*next], then the next one, until `budget` words IN ALL were looked at this call (never more); *next
 * keeps whose turn it is between calls (wraps; out of range = 0). n = 0 / NULL = nothing. */
void     stk_round(stk_scan_t *set, uint32_t n, uint32_t *next, uint32_t budget);

#endif /* TC_STACK_H */
