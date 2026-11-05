/*
 * tc_stack.c - Stack high-water marks: fill pattern + bounded scan (tc_stack.h). Pure C.
 */
#include "tc_stack.h"

void stk_fill(volatile uint32_t *lo, uint32_t words)
{
    for (uint32_t i = 0; i < words; i++) lo[i] = STK_PATTERN;
}

void stk_init(stk_scan_t *s, const volatile uint32_t *lo, uint32_t words, bool filled)
{
    s->lo = lo;
    s->words = words;
    s->pos = 0u;
    s->mark = words;
    s->checked = 0u;
    s->filled = filled && lo != 0 && words > 0u;
    s->known = false;
}

void stk_scan(stk_scan_t *s, uint32_t budget)
{
    s->checked = 0u;
    if (!s->filled) return;
    while (s->checked < budget) {
        if (s->pos >= s->mark) {                    /* nothing deeper than the known mark: a full pass */
            s->known = true;
            s->pos = 0u;
            return;
        }
        s->checked++;
        if (s->lo[s->pos] != STK_PATTERN) {         /* the first used word from the bottom = the deepest use */
            s->mark = s->pos;
            s->known = true;
            s->pos = 0u;
            return;
        }
        s->pos++;
    }
}

uint32_t stk_size_bytes(const stk_scan_t *s)
{
    return s->words * 4u;
}

uint32_t stk_used_bytes(const stk_scan_t *s)
{
    return s->known ? (s->words - s->mark) * 4u : 0u;
}

bool stk_overflow(const stk_scan_t *s)
{
    return s->known && s->mark == 0u;
}

void stk_round(stk_scan_t *set, uint32_t n, uint32_t *next, uint32_t budget)
{
    if (set == 0 || next == 0 || n == 0u) return;
    if (*next >= n) *next = 0u;
    /* one stack after the other, at most `budget` words in all per call (never a long loop): a stack stays
     * the current one while it is in the middle of its first pass; known / never filled = the next one */
    for (uint32_t k = 0u; k < n && budget > 0u; k++) {
        stk_scan_t *s = &set[*next];
        stk_scan(s, budget);
        budget = s->checked < budget ? budget - s->checked : 0u;
        if (s->checked == 0u || s->known) *next = (*next + 1u) % n;
    }
}
