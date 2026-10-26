/*
 * test_nv.c - Runs the board's data-flash record log (hal/tc_nv.c) on a flash
 * image in RAM on the PC, the way the control loop uses it: saves queued,
 * one bounded service step at a time, the flash "busy" for a few polls, and
 * faults: a POWER LOSS cutting any flash command (the page / sector is left
 * half programmed with garbage, the chip "reboots" = nvlog_init again), a
 * corrupted record, sector swaps (tiny regions), nothing issued while not
 * allowed. The model: after a reboot every slot must read the last COMMITTED
 * value or the one whose write was running - never anything else. A page is
 * never programmed twice between erases (the flash model checks it).
 * Driven and checked by nv_check.py (an independent reader of the image).
 *
 *   test_nv SEED ERASED_WORD(0|1) SECTOR REGION OPS IMAGE_OUT
 *     -> "PASS ..." / "FAIL ..." lines, "STAT ...", "EXPECT slot hexbytes" per slot
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tc_nv.h"

#define MAX_REGION 65536u
#define SLOTS_USED 6u

static uint8_t  img[2][MAX_REGION];
static uint32_t g_region, g_sector, g_erased;
static uint32_t g_cmds;                 /* flash commands issued */
static uint32_t g_cut_at;               /* command index that the power loss cuts (0 = none) */
static bool     g_cut;                  /* the cut happened (reboot needed) */
static uint32_t g_busy_left;            /* polls until the running command finishes */
static bool     g_cmd_err;
static uint32_t g_double_program, g_read_while_busy, g_us;
static uint32_t g_fails;
static uint32_t rng = 1u;

static struct { bool erase; uint8_t region; uint32_t off; uint32_t w0, w1; } g_cmd;

static uint32_t rnd(uint32_t n) { rng = rng * 1664525u + 1013904223u; return (rng >> 8) % n; }

static void fill_erased(uint8_t *p, uint32_t n)
{
    for (uint32_t i = 0; i < n; i += 4u) {
        p[i] = (uint8_t)g_erased; p[i + 1] = (uint8_t)(g_erased >> 8);
        p[i + 2] = (uint8_t)(g_erased >> 16); p[i + 3] = (uint8_t)(g_erased >> 24);
    }
}

static bool page_is_erased(const uint8_t *p)
{
    uint8_t e[8];
    fill_erased(e, 8u);
    return memcmp(p, e, 8u) == 0;
}

/* --- the flash model (port) --- */
static bool p_read(uint8_t region, uint32_t off, void *buf, uint32_t len)
{
    if (g_busy_left) g_read_while_busy++;
    if (region > 1u || off + len > g_region) return false;
    memcpy(buf, &img[region][off], len);
    return true;
}

static void finish_cmd(void)
{
    uint8_t *p = &img[g_cmd.region][g_cmd.off];
    if (g_cmd.erase) fill_erased(p, g_sector);
    else {
        p[0] = (uint8_t)g_cmd.w0; p[1] = (uint8_t)(g_cmd.w0 >> 8); p[2] = (uint8_t)(g_cmd.w0 >> 16); p[3] = (uint8_t)(g_cmd.w0 >> 24);
        p[4] = (uint8_t)g_cmd.w1; p[5] = (uint8_t)(g_cmd.w1 >> 8); p[6] = (uint8_t)(g_cmd.w1 >> 16); p[7] = (uint8_t)(g_cmd.w1 >> 24);
    }
}

static void cut_cmd(void)
{
    /* the power went while the command ran: garbage in part of it */
    uint8_t *p = &img[g_cmd.region][g_cmd.off];
    uint32_t n = g_cmd.erase ? g_sector : 8u;
    uint32_t mode = rnd(3u);
    for (uint32_t i = 0; i < n; i++) {
        if (mode == 0u) p[i] = (uint8_t)rnd(256u);                       /* random */
        else if (mode == 1u) { if (i < n / 2u) p[i] = (uint8_t)rnd(256u); }   /* first half */
        else if (i >= n / 2u) p[i] = (uint8_t)(g_erased ^ rnd(2u));      /* second half nearly erased */
    }
    g_cut = true;
    g_busy_left = 0u;
}

static bool issue(bool erase, uint8_t region, uint32_t off, uint32_t w0, uint32_t w1)
{
    if (g_busy_left) return false;
    g_cmds++;
    g_cmd.erase = erase; g_cmd.region = region; g_cmd.off = off; g_cmd.w0 = w0; g_cmd.w1 = w1;
    g_busy_left = 1u + rnd(3u);
    if (g_cut_at && g_cmds == g_cut_at) cut_cmd();
    return true;
}

static bool p_erase_start(uint8_t region, uint32_t sector)
{
    if (region > 1u || (sector + 1u) * g_sector > g_region) return false;
    return issue(true, region, sector * g_sector, 0u, 0u);
}

static bool p_write_start(uint8_t region, uint32_t off, uint32_t w0, uint32_t w1)
{
    if (region > 1u || (off & 7u) || off + 8u > g_region) return false;
    if (!page_is_erased(&img[region][off])) g_double_program++;
    return issue(false, region, off, w0, w1);
}

static bool p_busy(void)
{
    if (g_busy_left) {
        if (--g_busy_left == 0u && !g_cut) finish_cmd();
        return g_busy_left != 0u;
    }
    return false;
}

static bool p_failed(void) { bool e = g_cmd_err; g_cmd_err = false; return e; }
static uint32_t p_micros(void) { return g_us += 7u; }

static nvlog_port_t port;
static uint32_t tot_swaps, tot_writes, tot_erases, tot_retries, tot_bad;   /* over every reboot */
static uint8_t  last_fault;

static void account(void)
{
    const nvlog_stats_t *st = nvlog_stats();
    tot_swaps += st->swaps; tot_writes += st->writes; tot_erases += st->erases;
    tot_retries += st->retries; tot_bad += st->bad_records;
    if (st->fault_why) last_fault = st->fault_why;
}

/* --- the model of what must be stored --- */
typedef struct { uint8_t data[NV_MAX_LEN]; uint32_t len; bool have; } val_t;
static val_t committed[NV_SLOTS], queued[NV_SLOTS];

static void report(bool ok, const char *what)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) g_fails++;
}

static bool same(const val_t *v, const uint8_t *d, uint32_t len)
{
    return v->have && v->len == len && memcmp(v->data, d, len) == 0;
}

/* After a (re)boot: every slot reads the committed value or the one in flight. */
static bool check_all(const char *when)
{
    bool ok = true;
    for (uint8_t s = 1; s <= SLOTS_USED; s++) {
        uint8_t d[NV_MAX_LEN];
        bool have = false;
        uint32_t len = 0u;
        uint32_t cands[2] = { committed[s].have ? committed[s].len : 0u, queued[s].have ? queued[s].len : 0u };
        for (int c = 0; c < 2 && !have; c++) {
            if (cands[c] && nvlog_load(s, d, cands[c])) { have = true; len = cands[c]; }
        }
        bool good;
        if (!have) good = !committed[s].have;                   /* nothing stored yet (nor in flight) */
        else good = same(&committed[s], d, len) || same(&queued[s], d, len);
        if (!good) { printf("  slot %u: %s value after %s\n", s, have ? "WRONG" : "MISSING", when); ok = false; }
        if (have) { memcpy(committed[s].data, d, len); committed[s].len = len; committed[s].have = true; }
        queued[s].have = false;
    }
    return ok;
}

static void reboot(void)
{
    account();
    g_busy_left = 0u;
    g_cut = false;
    g_cut_at = 0u;
    nvlog_init(&port);
}

/* Service until nothing is pending (or the cut hits). */
static bool drain(bool allowed)
{
    for (int i = 0; i < 200000 && nvlog_pending() && nvlog_ok(); i++) {
        nvlog_service(allowed, 0u);
        if (g_cut) return false;
        if (!allowed && i > 50) break;
    }
    if (!nvlog_pending()) {
        for (uint8_t s = 1; s < NV_SLOTS; s++) if (queued[s].have) { committed[s] = queued[s]; queued[s].have = false; }
        return true;
    }
    return false;
}

static void save_random(uint8_t s, uint32_t maxlen)
{
    val_t *q = &queued[s];
    q->len = 1u + rnd(maxlen);
    for (uint32_t i = 0; i < q->len; i++) q->data[i] = (uint8_t)rnd(256u);
    if (rnd(4u) == 0u) memset(q->data, (int)(g_erased & 0xFFu), q->len);   /* erased-looking payload */
    q->have = nvlog_save(s, q->data, q->len);
}

int main(int argc, char **argv)
{
    if (argc < 7) { fprintf(stderr, "usage: test_nv SEED ERASED SECTOR REGION OPS IMAGE\n"); return 2; }
    rng = (uint32_t)atoi(argv[1]);
    g_erased = atoi(argv[2]) ? 0xFFFFFFFFu : 0u;
    g_sector = (uint32_t)atoi(argv[3]);
    g_region = (uint32_t)atoi(argv[4]);
    int ops = atoi(argv[5]);
    if (g_region > MAX_REGION || g_region % g_sector || g_region % 64u) { printf("FAIL bad geometry\n"); return 1; }
    /* the live set (6 slots) + 2 queued records + room for garbage must fit one region */
    uint32_t maxlen = g_region / 10u - 24u;
    if (maxlen > NV_MAX_LEN) maxlen = NV_MAX_LEN;
    port.region_bytes = g_region; port.sector_bytes = g_sector; port.erased_word = g_erased;
    port.cmd_timeout_us = 1000000u;
    port.read = p_read; port.erase_start = p_erase_start; port.write_start = p_write_start;
    port.busy = p_busy; port.failed = p_failed; port.micros = p_micros;
    fill_erased(img[0], g_region);
    fill_erased(img[1], g_region);
    /* some garbage in B at the very start (a card from the factory is never clean) */
    for (uint32_t i = 0; i < 24u; i++) img[1][g_region - 100u + i] = (uint8_t)rnd(256u);
    reboot();
    report(nvlog_ok(), "init on a fresh (garbage in B) image");

    /* 1. not allowed: nothing is issued, the save stays pending; allowed: written */
    {
        uint8_t v[20];
        for (int i = 0; i < 20; i++) v[i] = (uint8_t)i;
        uint32_t c0 = g_cmds;
        report(nvlog_save(1u, v, 20u), "save queued");
        for (int i = 0; i < 100; i++) nvlog_service(false, 0u);
        bool none = g_cmds == c0 && nvlog_pending();
        uint8_t back[20];
        bool shadow = nvlog_load(1u, back, 20u) && memcmp(v, back, 20u) == 0;
        report(none, "not allowed (driving): no flash command issued, the save waits");
        report(shadow, "a queued save is what a load returns meanwhile");
        bool done = drain(true);
        memcpy(committed[1].data, v, 20u); committed[1].len = 20u; committed[1].have = true;
        reboot();
        bool kept = nvlog_load(1u, back, 20u) && memcmp(v, back, 20u) == 0;
        report(done && kept, "allowed (STANDBY): written, read back after a reboot");
    }

    /* 2. random saves with power losses, corruption and swaps */
    uint32_t cuts = 0u, corruptions = 0u;
    bool model_ok = true;
    for (int op = 0; op < ops && nvlog_ok(); op++) {
        uint8_t s = (uint8_t)(1u + rnd(SLOTS_USED));
        uint32_t t_before = nvlog_stats()->tail[nvlog_stats()->active];
        uint8_t r_before = nvlog_stats()->active;
        save_random(s, maxlen);
        bool two = rnd(10u) < 2u;
        if (two) save_random((uint8_t)(1u + rnd(SLOTS_USED)), maxlen);           /* two at once */
        bool cut = rnd(3u) == 0u;
        if (cut) g_cut_at = g_cmds + 1u + rnd(40u);
        bool done = drain(true);
        if (g_cut) {
            cuts++;
            reboot();
            if (!check_all("power loss")) model_ok = false;
            continue;
        }
        g_cut_at = 0u;
        if (!done) { printf("  op %d: not drained\n", op); model_ok = false; break; }
        if (!two && rnd(8u) == 0u && nvlog_stats()->active == r_before &&
            nvlog_stats()->tail[r_before] == t_before + 16u + ((committed[s].len + 7u) & ~7u) + 8u) {
            /* corrupt the record just written (a byte of its payload / trailer): after a
             * reboot the slot must read the previous value, never the corrupted one */
            uint32_t t_after = nvlog_stats()->tail[r_before];
            uint32_t at = t_before + 16u + rnd(t_after - t_before - 16u);
            img[r_before][at] ^= (uint8_t)(1u << rnd(8u));
            val_t bad = committed[s];
            committed[s].have = false;                       /* unknown: the older record, if any */
            reboot();
            uint8_t d[NV_MAX_LEN];
            bool corrupted_seen = nvlog_load(s, d, bad.len) && memcmp(d, bad.data, bad.len) == 0;
            /* (the flipped byte may sit in the padding: then the record is still valid) */
            bool pad_only = at >= t_before + 16u + bad.len && at < t_after - 8u;
            if (corrupted_seen && !pad_only) {
                /* the same bytes can only come back if the flip hit the padding (not covered by the CRC) */
                printf("  op %d: corrupted record still loaded\n", op); model_ok = false;
            }
            corruptions++;
            if (nvlog_load(s, d, bad.len)) { memcpy(committed[s].data, d, bad.len); committed[s].len = bad.len; committed[s].have = true; }
            else {
                /* an older record of the slot may exist with another length: accept whatever a reboot shows */
                committed[s].have = false;
                for (uint32_t L = 1; L <= maxlen; L++) {
                    if (nvlog_load(s, d, L)) { memcpy(committed[s].data, d, L); committed[s].len = L; committed[s].have = true; break; }
                }
            }
            queued[s].have = false;
        }
        if (rnd(25u) == 0u) {                             /* a plain reboot */
            reboot();
            if (!check_all("reboot")) model_ok = false;
        }
    }
    account();
    if (!nvlog_ok()) printf("  NV FAULT why %u\n", nvlog_stats()->fault_why);
    report(nvlog_ok(), "no NV FAULT during the random run");
    report(model_ok, "every reboot: the committed value or the one in flight, never another");
    report(g_double_program == 0u, "no page programmed twice between erases");
    report(g_read_while_busy == 0u, "the flash is never read while a command runs");
    report(tot_swaps >= 2u, "sector swaps happened (both regions used, live records carried over)");
    report(cuts > 0u && corruptions > 0u, "power losses and corruptions were exercised");

    /* 3. the final state, for nv_check.py: drain, reboot, dump + the committed values */
    drain(true);
    reboot();
    check_all("final");
    FILE *f = fopen(argv[6], "wb");
    if (f) { fwrite(img[0], 1, g_region, f); fwrite(img[1], 1, g_region, f); fclose(f); }
    for (uint8_t s = 1; s <= SLOTS_USED; s++) {
        if (!committed[s].have) continue;
        printf("EXPECT %u ", s);
        for (uint32_t i = 0; i < committed[s].len; i++) printf("%02x", committed[s].data[i]);
        printf("\n");
    }
    account();
    printf("STAT writes %u swaps %u erases %u retries %u bad %u cuts %u corruptions %u cmds %u\n",
           tot_writes, tot_swaps, tot_erases, tot_retries, tot_bad, cuts, corruptions, g_cmds);
    printf("%s\n", g_fails ? "RESULT FAIL" : "RESULT PASS");
    return g_fails ? 1 : 0;
}
