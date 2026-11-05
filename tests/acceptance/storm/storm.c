/*
 * storm.c - Acceptance test (Part 4.4): random fault storm on the AURIX
 * controller in the PC simulation, much faster than real time.
 *
 * Every episode = a seed -> a SCHEDULE of random events (joystick moves and
 * buttons, panel buttons, PC commands, remote driving data with dropouts,
 * sensor failures, corrupted / random frames on the USB link and the
 * gateway UART, gateway loss, navigation requests incl. obstacle caps,
 * slamming, chip resets, frozen core 0, injected unsafe outputs for the
 * CPU1 monitor). After EVERY control step the safety invariants are checked.
 * Step 11e: about a third of the episodes run on the DroneCAN servos (the
 * worst-case 6 s / 12 s or the fast 2 s / 3 s servos, chosen from the seed
 * without changing the other draws) with CAN faults (node lost / silent /
 * rebooted, bus-off, the actuator task stuck); invariants I47-I57.
 * Step 13: AUTO RETURN requests (step 13b: their own draw, part of the NAVREQ slice); invariant I58.
 * Step 13b: invariant I59 (a DP TRIP drives its own copy of the stored program, also after an AUTO RETURN).
 * Step 13q1: LINK TIME (the UNCREWED link-loss detection time): the gateway reports the HELM station's age in
 * 3 of 4 episodes; its own draw (part of the gateway-fuzz slice) sets the LINK TIME, silences the HELM station
 * or brings it back; invariants I60-I62.
 * Step 13q2: UNCREWED REMOTE / MANEUVER start at once on the request (shore stick centred + lever neutral), no
 * release; REMOTE / MANEUVER -> AUTO / DP straight; invariants I63-I64 (I7 for REMOTE / MANEUVER = CREWED only); step 13q2b: I65 (no AUTO RETURN out of REMOTE / MANEUVER).
 * Step 13q2c: UNCREWED HDG LOCK from REMOTE / MANEUVER by the HELM captain's command (E_SHDG, after some UNCREWED
 * REMOTE / MANEUVER requests; navigation requests with a STOP cap / a geofence breach); invariants I66-I68;
 * step 13q2d: I69 (no shore data / GPS lost in UNCREWED HDG LOCK).
 * Step 13q3: the shore stick override in UNCREWED AUTO / HDG LOCK (stick X > 15 % = steering by hand inside the mode,
 * the lever a power ceiling in AUTO, the TRIGGER = the new HDG LOCK heading; E_TRIG, the shore stream also in UNCREWED
 * AUTO); invariants I71-I74. Step 13q3b: the navreq heartbeat also in UNCREWED AUTO; I75 (steering AUTO by hand without shore
 * data / lease / near the shore = idle + neutral), I76 (a HELM change ends the override).
 * Step 13q4: the slamming reduction never goes away by itself - only the HELM station's ACCEPT (E_SLAMRES, some
 * power-raise draws: ACCEPT / REJECT) restores it; invariant I77.
 * Step 13q5: the MANEUVER gear on the shore stick Y (full forward / back held 0.3 s = F / R, centre = N; the lever is
 * not used there): the storm's shore stick sends a Y (E_MNV_Y after MANEUVER requests: full / part / flicks /
 * centre); invariant I78. Step 13q5c: in MANEUVER the shore packets carry a jittering data age (20-139 ms) and some
 * flicks / holds have one packet delayed ~200 ms (it arrives with its true age: I78 times the hold on the SEND
 * time); some E_SHDG commands from MANEUVER come from a stick-Y station packet (no lever); invariant I79.
 * Step 13q2e: the storm's gateway sends its "no danger" navigation request at 2 Hz (like nav_loop) while an
 * UNCREWED boat is in REMOTE / MANEUVER / HDG LOCK; E_NAVBEAT stops it for a while (nav_compose broken / frames
 * lost on the UART) and some shore reports say 40 m; invariant I70 (lapsed / never lease, the shore within the stop
 * distance = STOP; never entered without them).
 * A failing episode is replayed with fewer events until the shortest
 * schedule that still fails remains (the "minimal reproduction").
 *
 *   storm.exe [episodes] [first_seed] [--min-min 5] [--max-min 30] [--quiet]
 *   storm.exe --replay SEED          one episode, every event printed
 *   storm.exe --wrap                 the 32-bit time wraparound run (Part 4.3)
 *   storm.exe --soak HOURS SEED      one long mixed run (Part 4.3)
 *
 * Built from app/ + sim/ (not changed) by build_storm.bat. Output: one line
 * per invariant (PASS / FAIL counts), worst reaction times, failing seeds.
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "smallboat.h"
#include "powerlim.h"
#include "link.h"
#include "proto.h"
#include "monitor.h"
#include "timing.h"
#include "logq.h"
#include "logger.h"
#include "sim_world.h"
#include "hal_sim.h"
#include "logcore0.h"
#include "dronecan.h"
#include "can_act.h"
#include "can_relay.h"
void sim_hal_real_micros(bool on);   /* hal_sim.c: the PC clock for hal_micros() (timing) */
#include "sim_run.h"

/* sim_sb.c / sim_train.c need these from sim_main.c (not linked here) */
void test_report(const char *name, bool ok, const char *detail) { (void)name; (void)ok; (void)detail; }
void test_na(const char *name, const char *why) { (void)name; (void)why; }
results_t run(const scenario_t *sc) { results_t r; (void)sc; memset(&r, 0, sizeof r); return r; }   /* sim_train.c */

/* ------------------------------------------------------------------ rng -- */
static uint32_t rng_next(uint32_t *s)
{
    uint32_t x = *s ? *s : 0x9E3779B9u;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return x;
}
static float rnd(uint32_t *s) { return (float)(rng_next(s) >> 8) / 16777216.0f; }
static int rndi(uint32_t *s, int n) { return (int)(rng_next(s) % (uint32_t)n); }

/* ------------------------------------------------------------- events -- */
typedef enum {
    E_JOY, E_BTN, E_TAKE, E_AUTOBTN, E_CONFIRM,
    E_PC_MODE, E_PC_AUTO_START, E_PC_AUTO_RESUME, E_PC_PAUSE, E_PC_PROG, E_PC_RAISE, E_PC_LOWER,
    E_REMOTE_REQ, E_REMOTE_ON, E_REMOTE_OFF, E_REMOTE_STALE, E_REMOTE_RELEASE,
    E_COMPASS, E_GYRO, E_GPS, E_GPS_JUMP, E_NSW, E_CURRENT, E_LIMITSW, E_JOY_WIRE, E_ACCEL, E_SEA,
    E_GW, E_NAVREQ, E_FUZZ_USB, E_FUZZ_GW,
    E_FREEZE, E_RESET, E_INJ_THR, E_INJ_GEAR,
    E_SCONFIRM, E_MAN_REQ, E_JOY_PLUG, E_HELM_DROP, E_HELM_BACK, E_SHORE,
    E_GEN, E_SUPPLY, E_RELAY, E_ENGINE, E_WDT, E_DP, E_CAN, E_PC_RETURN, E_PC_CONFONLY, E_LINK, E_SHDG,
    E_NAVBEAT, E_TRIG, E_SLAMRES, E_SLAMHIT, E_MNV_Y,
    E_COUNT
} ev_kind_t;

static const char *EV_NAMES[E_COUNT] = {
    "joystick", "panel button", "TAKE", "AUTO button", "CONFIRM",
    "PC mode request", "PC AUTO start", "PC AUTO resume", "PC AUTO pause", "PC program", "PC power raise",
    "PC power lower", "remote request", "remote data on", "remote data off (dropout)", "remote data stale",
    "remote release", "compass", "gyro", "GPS", "GPS jump (spoof)", "neutral switch", "motor current sense",
    "limit switches", "joystick wire", "slamming sensor", "sea state", "gateway status", "navigation request",
    "USB link fuzz", "gateway UART fuzz", "core 0 frozen", "chip reset", "inject throttle over the limit",
    "inject gear change under load", "shore CONFIRM (captain's joystick)", "MANEUVER request",
    "onboard joystick plugged / unplugged", "HELM station dropped (link lost)", "HELM station back",
    "shore info (near / far / none / invalid)", "generator auto on (+ ACK, bank low) / off",
    "bank voltage (low / fine / no data / just low)", "relay board (table offer / relay command / board fault / back)",
    "engine (RESTART / START held / released / e-stop pressed or released)",
    "watchdog reset (the chip restarts: system reset, not a power-on)",
    "DP (thruster settings / START SPOT, CRUISE, TRIP / JOG / SPEED / HEADING / STOP; a current)",
    "DroneCAN (a servo node lost / silent / rebooted, bus-off, the actuator task stuck; back after a while)",
    "PC AUTO return (the program backwards to waypoint 1; then the CONFIRM)",
    "PC CONFIRM_ONLY (academy practical test: a question for the onboard CONFIRM that changes nothing)",
    "LINK TIME (the captain sets 1..70 s / the HELM station goes silent / is heard again / open sea)",
    "shore HDG LOCK (UNCREWED: the HELM captain's mode command from REMOTE / MANEUVER; stick centred mostly)",
    "gateway navreq heartbeat stops for a while (nav_compose broken / frames lost)",
    "shore TRIGGER (13q3: the stick moved first in half of them; UNCREWED HDG LOCK steering by hand = new heading)",
    "slamming power restore (13q4: the HELM station's ACCEPT mostly / REJECT)",
    "a hard wave hit (13q4: one red impact at any speed; the restore answer 35..65 s later)",
    "shore stick Y (13q5: MANEUVER gear - centre / full forward / full back / part / a short flick)"
};

typedef struct {
    uint32_t t;                 /* ms */
    uint8_t  kind;
    int32_t  a, b, c;           /* parameters */
    float    f, g;
} ev_t;

#define MAX_EV 4000
typedef struct {
    uint32_t seed, dur_ms;
    float    sea0;
    bool     boot_deflected;    /* joystick held at power-up (I11) */
    bool     uncrewed;          /* no onboard joystick at power-up: UNCREWED profile (I22) */
    uint8_t  can;               /* step 11e: 0 PWM servos, 1 DroneCAN worst-case servos, 2 DroneCAN fast servos */
    int      n;
    ev_t     ev[MAX_EV];
    bool     on[MAX_EV];        /* used while shrinking */
} sched_t;

static void gen_schedule(sched_t *s, uint32_t seed, uint32_t dur_ms)
{
    uint32_t r = seed * 2654435761u + 1u;
    memset(s, 0, sizeof *s);
    s->seed = seed;
    s->dur_ms = dur_ms;
    s->sea0 = (float)rndi(&r, 5);
    s->boot_deflected = rnd(&r) < 0.15f;
    s->uncrewed = rnd(&r) < 0.25f;
    {   /* (from its own hash: the other draws - and so the PWM episodes - stay what they were) */
        uint32_t h = (seed * 2246822519u + 374761393u) ^ 0x5bd1e995u;
        h ^= h >> 15;
        h *= 2654435761u;
        h ^= h >> 13;
        uint32_t c = h % 100u;
        s->can = c < 20u ? 1u : (c < 35u ? 2u : 0u);
    }
    uint32_t t = 2000u;
    /* step 13q3 (I71-I74): half of the UNCREWED episodes start with the shore captain's own drive (no rng draw: the
     * other draws stay what they were) - half of those in AUTO (below), the others: HELM, REMOTE, the lever forward,
     * HDG LOCK, steering by hand with the stick +
     * the TRIGGER (the new heading), the trigger while not steering, steering + letting go (the old heading), steering
     * + stale data (idle + neutral, STANDBY) */
    if (s->uncrewed && ((seed * 2246822519u) >> 7) % 2u == 0u) {
        static const struct { uint32_t dt; uint8_t kind; int a, b, c; float f, g; } SCRIPT[] = {
            {  9000u, E_HELM_BACK, 0, 0, 0, 0.0f, 0.0f },
            {   100u, E_SHORE, 2, 0, 0, 0.0f, 0.0f },
            {   100u, E_REMOTE_ON, 0, 0, 0, 0.0f, 0.0f },       /* stick centred, lever neutral */
            {  1300u, E_REMOTE_REQ, 0, 0, 0, 0.0f, 0.0f },
            {  1000u, E_REMOTE_ON, 0, 0, 0, 0.0f, 0.6f },       /* lever 0.8 */
            {  6500u, E_SHDG, 0, 0, 600, 0.0f, 0.6f },
            {  1500u, E_REMOTE_ON, 0, 0, 0, 0.7f, 0.6f },       /* steering by hand */
            {  3500u, E_TRIG, 900, 0, 0, 0.0f, 0.0f },          /* the new heading */
            {  1000u, E_REMOTE_ON, 0, 0, 0, 0.0f, 0.6f },
            {  5000u, E_TRIG, 900, 0, 0, 0.0f, 0.0f },          /* not steering: nothing */
            {  2000u, E_REMOTE_ON, 0, 0, 0, -0.7f, 0.6f },
            {  5000u, E_REMOTE_ON, 0, 0, 0, 0.0f, 0.6f },       /* let go: the old heading */
            {  5000u, E_REMOTE_ON, 0, 0, 0, 0.7f, 0.6f },
            {  3000u, E_REMOTE_STALE, 0, 0, 0, 0.0f, 0.0f },    /* stale while steering: idle + N, STANDBY */
            {  7000u, E_REMOTE_ON, 0, 0, 0, 0.0f, 0.0f },
        };
        /* the other half: UNCREWED AUTO - the lever up past the throttle and back (the ceiling cuts), steering by hand,
         * stale data while steering (idle + neutral at once, STANDBY) */
        static const struct { uint32_t dt; uint8_t kind; int a, b, c; float f, g; } SCRIPT_A[] = {
            {  9000u, E_HELM_BACK, 0, 0, 0, 0.0f, 0.0f },
            {   100u, E_SHORE, 2, 0, 0, 0.0f, 0.0f },
            {   100u, E_REMOTE_ON, 0, 0, 0, 0.0f, 0.0f },
            {   300u, E_PC_PROG, 2, 0, 0, 0.0f, 0.0f },
            {   500u, E_PC_AUTO_START, 0, 0, 0, 0.0f, 0.0f },
            {  1000u, E_SCONFIRM, 0, 0, 0, 0.0f, 0.0f },
            { 12000u, E_REMOTE_ON, 0, 0, 0, 0.0f, 1.0f },       /* lever full: up past the throttle */
            {  4000u, E_REMOTE_ON, 0, 0, 0, 0.0f, 0.25f },      /* lever back: the ceiling cuts */
            {  6000u, E_REMOTE_ON, 0, 0, 0, 0.7f, 0.25f },      /* steering by hand */
            {  4000u, E_REMOTE_ON, 0, 0, 0, 0.0f, 1.0f },       /* let go, lever up */
            {  6000u, E_REMOTE_ON, 0, 0, 0, -0.7f, 1.0f },
            {  2000u, E_HELM_BACK, 1, 0, 77, 0.0f, 0.0f },     /* 13q3b (I76): a HELM swap while steering by hand */
            {  2000u, E_REMOTE_ON, 0, 0, 0, 0.0f, 1.0f },
            {  2000u, E_REMOTE_ON, 0, 0, 0, -0.7f, 1.0f },      /* the new captain steers */
            {  3000u, E_REMOTE_STALE, 0, 0, 0, 0.0f, 0.0f },    /* stale while steering: idle + N, STANDBY */
            {  7000u, E_REMOTE_ON, 0, 0, 0, 0.0f, 0.0f },
        };
        bool auto_drive = ((seed * 2246822519u) >> 9) % 2u == 0u;
        size_t ns = auto_drive ? sizeof SCRIPT_A / sizeof SCRIPT_A[0] : sizeof SCRIPT / sizeof SCRIPT[0];
        for (size_t i = 0; i < ns && s->n < MAX_EV; i++) {
            ev_t *e = &s->ev[s->n];
            memset(e, 0, sizeof *e);
            uint32_t dt = auto_drive ? SCRIPT_A[i].dt : SCRIPT[i].dt;
            t += dt;
            e->t = t;
            e->kind = auto_drive ? SCRIPT_A[i].kind : SCRIPT[i].kind;
            e->a = auto_drive ? SCRIPT_A[i].a : SCRIPT[i].a;
            e->b = auto_drive ? SCRIPT_A[i].b : SCRIPT[i].b;
            e->c = auto_drive ? SCRIPT_A[i].c : SCRIPT[i].c;
            e->f = auto_drive ? SCRIPT_A[i].f : SCRIPT[i].f;
            e->g = auto_drive ? SCRIPT_A[i].g : SCRIPT[i].g;
            s->on[s->n] = true;
            s->n++;
        }
    }
    /* the boat should be doing something: a program and a power raise early on */
    while (t < dur_ms && s->n < MAX_EV) {
        t += 200u + (uint32_t)rndi(&r, 9000);
        ev_t *e = &s->ev[s->n];
        e->t = t;
        float p = rnd(&r);
        int k;
        if (p < 0.20f) k = E_JOY;
        else if (p < 0.27f) k = E_BTN;
        else if (p < 0.30f) k = E_TAKE;
        else if (p < 0.32f) k = E_AUTOBTN;
        else if (p < 0.38f) k = E_CONFIRM;
        else if (p < 0.42f) k = E_PC_MODE;
        else if (p < 0.45f) k = E_PC_AUTO_START;
        else if (p < 0.46f) k = E_PC_AUTO_RESUME;
        else if (p < 0.47f) k = E_PC_PAUSE;
        else if (p < 0.49f) k = E_PC_PROG;
        else if (p < 0.53f) k = E_PC_RAISE;
        else if (p < 0.54f) k = E_PC_LOWER;
        else if (p < 0.56f) k = E_REMOTE_REQ;
        else if (p < 0.58f) k = E_REMOTE_ON;
        else if (p < 0.595f) k = E_REMOTE_OFF;
        else if (p < 0.605f) k = E_REMOTE_STALE;
        else if (p < 0.61f) k = E_REMOTE_RELEASE;
        else if (p < 0.63f) k = E_COMPASS;
        else if (p < 0.64f) k = E_GYRO;
        else if (p < 0.66f) k = E_GPS;
        else if (p < 0.665f) k = E_GPS_JUMP;
        else if (p < 0.675f) k = E_NSW;
        else if (p < 0.68f) k = E_CURRENT;
        else if (p < 0.685f) k = E_LIMITSW;
        else if (p < 0.695f) k = E_JOY_WIRE;
        else if (p < 0.70f) k = E_ACCEL;
        else if (p < 0.72f) k = E_SEA;
        else if (p < 0.74f) k = E_GW;
        else if (p < 0.80f) k = E_NAVREQ;
        else if (p < 0.81f) k = E_GEN;
        else if (p < 0.82f) k = E_SUPPLY;
        else if (p < 0.831f) k = E_RELAY;
        else if (p < 0.835f) k = E_WDT;
        else if (p < 0.85f) k = E_ENGINE;
        else if (p < 0.865f) k = E_DP;
        else if (p < 0.875f) k = E_FUZZ_USB;
        else if (p < 0.92f) k = E_FUZZ_GW;
        else if (p < 0.935f) k = E_FREEZE;
        else if (p < 0.95f) k = E_RESET;
        else if (p < 0.970f) k = E_INJ_THR;
        else if (p < 0.980f) k = E_INJ_GEAR;
        else if (p < 0.986f) k = E_MAN_REQ;
        else if (p < 0.989f) k = E_SCONFIRM;
        else if (p < 0.991f) k = E_JOY_PLUG;
        else if (p < 0.994f) k = E_HELM_DROP;
        else if (p < 0.997f) k = E_HELM_BACK;
        else k = E_SHORE;
        e->kind = (uint8_t)k;
        e->a = rndi(&r, 1000);
        e->b = rndi(&r, 1000);
        e->c = rndi(&r, 1000);
        e->f = rnd(&r) * 2.0f - 1.0f;
        e->g = rnd(&r) * 2.0f - 1.0f;
        if (s->can && k == E_FUZZ_GW && e->c < 450) e->kind = E_CAN;    /* DroneCAN episodes: CAN faults */
        /* step 13: some AUTO start / resume draws ask for an AUTO RETURN instead (the other draws unchanged) */
        /* (step 13b, review: a separate draw - part of the NAVREQ slice - so that AUTO START / RESUME keep their coverage) */
        if (k == E_NAVREQ && e->b >= 850) e->kind = E_PC_RETURN;
        /* step 21: some power-raise draws ask a CONFIRM_ONLY instead (the academy practical test's question) */
        if (k == E_PC_RAISE && e->c >= 900) e->kind = E_PC_CONFONLY;
        /* step 13q4 (I77): some power-raise draws answer the slamming restore question instead */
        if (k == E_PC_RAISE && e->c >= 780 && e->c < 900) e->kind = E_SLAMRES;
        if (k == E_SEA && e->c >= 700) e->kind = E_SLAMRES;
        /* ... and some sea-state draws are a hard wave hit (a red impact at any speed: the cap steps down) with the
         * HELM station's answer 35..65 s later (S.slam_ans_t; no rng draw: the other draws stay what they were) */
        if (k == E_SEA && e->c >= 600 && e->c < 700) e->kind = E_SLAMHIT;
        /* step 13q1: some gateway-fuzz draws are LINK TIME events (CAN episodes take c < 450 for E_CAN) */
        if (k == E_FUZZ_GW && e->c >= 860) e->kind = E_LINK;
        if (k == E_GW && e->b >= 700) e->kind = E_LINK;                   /* (and some gateway-status draws) */
        /* step 13q2c (I66): CREWED, some release draws are the shore HDG LOCK command instead (it must be refused) */
        if (k == E_REMOTE_RELEASE && !s->uncrewed && e->a >= 600) e->kind = E_SHDG;
        /* step 13q3 (I74): some stale-data draws are a shore TRIGGER press instead */
        if (k == E_REMOTE_STALE && e->c >= 450) e->kind = E_TRIG;
        s->on[s->n] = true;
        s->n++;
        /* DP: mostly a START (centre the stick, STANDBY, let the boat slow down first) + the CONFIRM */
        if (k == E_DP && e->a >= 150 && e->a < 550 && s->n + 3 < MAX_EV && rnd(&r) < 0.7f) {
            ev_t req = *e;
            memset(e, 0, sizeof *e);
            e->t = t;
            e->kind = E_JOY;                               /* a < 400: centre */
            ev_t *sb = &s->ev[s->n++];
            memset(sb, 0, sizeof *sb);
            sb->t = t + 100u;
            sb->kind = E_BTN;                              /* a < 350: STANDBY */
            s->on[s->n - 1] = true;
            /* step 13b (I59): some TRIP starts (no new program: c >= 500) come after an AUTO RETURN that ran from
             * STANDBY and was stopped again - the way back stays latched in RAM while the DP TRIP starts */
            if (req.b >= 750 && req.c >= 500 && req.c < 800 && s->n + 6 < MAX_EV) {
                ev_t *x1 = &s->ev[s->n++];
                memset(x1, 0, sizeof *x1);
                x1->t = t + 1500u;
                x1->kind = E_PC_RETURN;
                s->on[s->n - 1] = true;
                ev_t *x2 = &s->ev[s->n++];
                memset(x2, 0, sizeof *x2);
                x2->t = t + 2500u;
                x2->kind = (uint8_t)(s->uncrewed ? E_SCONFIRM : E_CONFIRM);
                s->on[s->n - 1] = true;
                ev_t *x3 = &s->ev[s->n++];
                memset(x3, 0, sizeof *x3);
                x3->t = t + 5000u;
                x3->kind = E_BTN;                          /* STANDBY: the return stops, still latched */
                s->on[s->n - 1] = true;
            }
            ev_t *rq = &s->ev[s->n++];
            *rq = req;
            rq->t = t + 15000u;
            s->on[s->n - 1] = true;
            ev_t *cf = &s->ev[s->n++];
            memset(cf, 0, sizeof *cf);
            cf->t = t + 15000u + 1000u + (uint32_t)rndi(&r, 2000);
            cf->kind = (uint8_t)((s->uncrewed ^ (rnd(&r) < 0.1f)) ? E_SCONFIRM : E_CONFIRM);
            s->on[s->n - 1] = true;
            t = cf->t;
            continue;
        }
        /* what a crew does: a program before AUTO, the remote data before its request, and the
           joystick CONFIRM 1-3 s after a request (mostly) */
        /* (step 11f: the boat file's relay table offer is confirmed too - mostly - so that the relays and the
           STARTER rules of I36 / I37 are reached in the episodes, not only refused) */
        if ((k == E_PC_AUTO_START || k == E_PC_RAISE || k == E_REMOTE_REQ || k == E_MAN_REQ ||
             (k == E_RELAY && e->a < 200)) && s->n + 3 < MAX_EV) {
            if (k == E_PC_AUTO_START && rnd(&r) < 0.6f) {
                ev_t pe = *e;
                pe.kind = E_PC_PROG;
                pe.a = 1 + rndi(&r, 5);
                *e = pe;                                   /* the program first ... */
                e->t = t;
                ev_t *st = &s->ev[s->n++];                 /* ... then the start */
                memset(st, 0, sizeof *st);
                st->t = t + 100u;
                st->kind = E_PC_AUTO_START;
                s->on[s->n - 1] = true;
            }
            int ea = e->a, eb = e->b, ec = e->c;          /* (step 13q2c: E_SHDG below, no new rng draw) */
            if (k == E_REMOTE_REQ || k == E_MAN_REQ) {
                ev_t req = *e;
                if (rnd(&r) < 0.6f) {                      /* stick to the centre + STANDBY first */
                    ev_t *jc = e;
                    memset(jc, 0, sizeof *jc);
                    jc->t = t;
                    jc->kind = E_JOY;
                    jc->a = 0;                             /* a < 400: centre */
                    ev_t *sb = &s->ev[s->n++];
                    memset(sb, 0, sizeof *sb);
                    sb->t = t + 100u;
                    sb->kind = E_BTN;
                    sb->a = 0;                             /* a < 350: STANDBY */
                    s->on[s->n - 1] = true;
                    ev_t *rq = &s->ev[s->n++];
                    *rq = req;
                    rq->t = t + 1600u;
                    s->on[s->n - 1] = true;
                    t += 1600u;
                }
                ev_t *on = &s->ev[s->n++];
                *on = req;
                on->t = t + 50u;
                on->kind = E_REMOTE_ON;
                s->on[s->n - 1] = true;
            }
            if (rnd(&r) < 0.75f) {
                ev_t *cf = &s->ev[s->n++];
                memset(cf, 0, sizeof *cf);
                cf->t = t + 1000u + (uint32_t)rndi(&r, 2000);
                /* the CONFIRM where the profile wants it - and sometimes the wrong one (must do nothing) */
                cf->kind = (uint8_t)((s->uncrewed ^ (rnd(&r) < 0.1f)) ? E_SCONFIRM : E_CONFIRM);
                s->on[s->n - 1] = true;
                t = cf->t;
                if ((k == E_REMOTE_REQ || k == E_MAN_REQ) && rnd(&r) < 0.5f && s->n + 1 < MAX_EV) {   /* a dropout / old data */
                    ev_t *dr = &s->ev[s->n++];
                    memset(dr, 0, sizeof *dr);
                    dr->t = t + 3000u + (uint32_t)rndi(&r, 17000);
                    dr->kind = rnd(&r) < 0.6f ? E_REMOTE_OFF : E_REMOTE_STALE;
                    s->on[s->n - 1] = true;
                    t = dr->t;
                }
            }
            /* step 13q5 (I78): after a MANEUVER request the captain works the gear with the stick Y: forward (or a
             * flick), back (or part), then centre (no rng draw: the other draws stay what they were) */
            if (k == E_MAN_REQ && s->n + 5 < MAX_EV) {
                static const int32_t A1[4] = { 300, 900, 300, 600 }, A2[3] = { 600, 750, 900 };
                ev_t *y1 = &s->ev[s->n++];
                memset(y1, 0, sizeof *y1);
                y1->t = t + 1000u + (uint32_t)(ea % 5) * 300u;
                y1->kind = E_MNV_Y;
                y1->a = A1[ea % 4];
                y1->b = eb;
                y1->f = (eb % 2) ? 0.5f : -0.5f;
                s->on[s->n - 1] = true;
                /* step 13q5c (I79): UNCREWED, sometimes the captain's HDG LOCK comes 0.6 s into the first stick-Y
                 * event, the stick where it is (held = refused NOT CENTRED, also after a stick-Y station packet; a
                 * flick let go = HDG LOCK from MANEUVER, in N) - no rng draw */
                if (s->uncrewed && (ea + ec) % 3 != 0) {
                    ev_t *h0 = &s->ev[s->n++];
                    memset(h0, 0, sizeof *h0);
                    h0->t = y1->t + 600u;
                    h0->kind = E_SHDG;
                    h0->a = ea;
                    h0->b = 700 + eb % 300;                        /* (b >= 700: the stick is not centred by it) */
                    h0->c = 500 + ec % 500;                        /* (c >= 500: the lever stays; c odd = stick-Y) */
                    h0->g = 0.0f;
                    s->on[s->n - 1] = true;
                }
                ev_t *y2 = &s->ev[s->n++];
                memset(y2, 0, sizeof *y2);
                y2->t = y1->t + 2500u + (uint32_t)(eb % 5) * 400u;
                y2->kind = E_MNV_Y;
                y2->a = A2[eb % 3];
                y2->b = ec;
                y2->f = (ec % 2) ? 0.6f : -0.6f;
                s->on[s->n - 1] = true;
                /* ... and 1.5 s into the second (a part deflection keeps the asked gear settled: the stick Y is NOT
                 * centred there - refused NOT CENTRED, also after a stick-Y station packet) */
                if (s->uncrewed && (ea + eb) % 2 == 0) {
                    ev_t *h1 = &s->ev[s->n++];
                    memset(h1, 0, sizeof *h1);
                    h1->t = y2->t + 1500u;
                    h1->kind = E_SHDG;
                    h1->a = eb;
                    h1->b = 700 + ec % 300;
                    h1->c = 501 + 2 * (ea % 249);                  /* (odd: a stick-Y station packet) */
                    h1->g = 0.0f;
                    s->on[s->n - 1] = true;
                }
                ev_t *y3 = &s->ev[s->n++];
                memset(y3, 0, sizeof *y3);
                y3->t = y2->t + 2000u + (uint32_t)(ec % 4) * 500u;
                y3->kind = E_MNV_Y;
                y3->a = 0;                                         /* centre */
                s->on[s->n - 1] = true;
                t = y3->t;
            }
            /* step 13q2c: UNCREWED, half of the REMOTE / MANEUVER requests are followed by the captain's HDG LOCK */
            if ((k == E_REMOTE_REQ || k == E_MAN_REQ) && s->uncrewed && (ea * 31 + eb) % 10 < 5 && s->n + 1 < MAX_EV) {
                /* step 13q2e (I70): sometimes the gateway's navreq heartbeat has stopped just before the command
                 * (the entry must be refused once the lease has run out) */
                if ((ea * 7 + ec) % 5 == 0 && s->n + 2 < MAX_EV) {
                    ev_t *pb = &s->ev[s->n++];
                    memset(pb, 0, sizeof *pb);
                    pb->t = t + 100u;
                    pb->kind = E_NAVBEAT;
                    pb->a = 7;                                     /* off 10.5 s */
                    s->on[s->n - 1] = true;
                    t = pb->t;
                }
                ev_t *hl = &s->ev[s->n++];
                memset(hl, 0, sizeof *hl);
                hl->t = t + 2000u + (uint32_t)(ec % 6) * 1000u;
                hl->kind = E_SHDG;
                hl->a = ea;
                hl->b = eb;
                hl->c = ec;
                hl->g = (float)(eb % 100) * 0.01f;
                s->on[s->n - 1] = true;
                t = hl->t;
                /* ... the lever pushed to the end soon after (I67: the ceiling never lifts the throttle above the held
                 * value) ... */
                if ((ea + eb) % 2 == 0 && s->n + 1 < MAX_EV) {
                    ev_t *lv = &s->ev[s->n++];
                    memset(lv, 0, sizeof *lv);
                    lv->t = t + 1500u;
                    lv->kind = E_REMOTE_ON;
                    lv->f = 0.0f;
                    lv->g = 1.0f;                                  /* lever 0.5 + 0.5 g = full */
                    s->on[s->n - 1] = true;
                    t = lv->t;
                }
                /* step 13q3 (I71 / I74): in a third of them the captain steers by hand with the stick (HDG LOCK
                 * waits), presses the TRIGGER while steering (the new heading) and lets the stick go */
                if ((ea + ec) % 3 == 1 && s->n + 3 < MAX_EV) {
                    ev_t *sx = &s->ev[s->n++];
                    memset(sx, 0, sizeof *sx);
                    sx->t = t + 800u;
                    sx->kind = E_REMOTE_ON;
                    sx->f = (eb % 2 == 0) ? 0.7f : -0.7f;
                    sx->g = 1.0f;                                  /* (the lever full: no cut) */
                    s->on[s->n - 1] = true;
                    ev_t *tg = &s->ev[s->n++];
                    memset(tg, 0, sizeof *tg);
                    tg->t = sx->t + 2500u + (uint32_t)(eb % 4) * 500u;
                    tg->kind = E_TRIG;
                    tg->a = 900;                                   /* (the stick stays where it is) */
                    s->on[s->n - 1] = true;
                    ev_t *sc = &s->ev[s->n++];
                    memset(sc, 0, sizeof *sc);
                    sc->t = tg->t + 1000u;
                    sc->kind = E_REMOTE_ON;
                    sc->f = 0.0f;
                    sc->g = 1.0f;
                    s->on[s->n - 1] = true;
                    t = sc->t;
                }
                /* step 13q2e (I70): in some of the others (no GPS / shore event below, no navreq) the gateway's navreq
                 * heartbeat stops for a while (nav_compose broken / frames lost on the UART) or the shore is 40 m
                 * away, 1.5-4.5 s into the HDG LOCK */
                bool i70 = (ea + eb + ec) % 3 == 0 && eb % 2 == 1;
                if (i70 && s->n + 1 < MAX_EV) {
                    ev_t *nb = &s->ev[s->n++];
                    memset(nb, 0, sizeof *nb);
                    nb->t = t + 1500u + (uint32_t)(ec % 4) * 1000u;
                    nb->kind = ((ea + ec) % 4 == 0) ? E_SHORE : E_NAVBEAT;
                    nb->a = ((ea + ec) % 4 == 0) ? 1 : ea;                       /* shore mode 1 = 40 m */
                    s->on[s->n - 1] = true;
                    t = nb->t;
                }
                /* step 13q2d (I69): in half of them the GPS lost (dead reckoning) or the gateway's shore data gone /
                 * not valid 1-2 s into the HDG LOCK (before any navreq below) */
                if (eb % 2 == 0 && s->n + 2 < MAX_EV) {
                    ev_t *g0 = &s->ev[s->n++];
                    memset(g0, 0, sizeof *g0);
                    g0->t = t + 1000u + (uint32_t)(ec % 3) * 500u;
                    g0->kind = ((eb / 2) % 2 == 0) ? E_GPS : E_SHORE;
                    g0->a = ((eb / 2) % 2 == 0) ? ec % 400 : (ec % 2 == 0 ? 0 : 3);
                    s->on[s->n - 1] = true;
                    t = g0->t;
                }
                /* ... and mostly a gateway request a few seconds later: a STOP (cap 0) or a geofence breach (I68) */
                if (i70) {
                    /* (13q2e: nothing more in this one) */
                } else if ((ea + ec) % 3 != 0 && s->n + 1 < MAX_EV) {
                    ev_t *nr = &s->ev[s->n++];
                    memset(nr, 0, sizeof *nr);
                    nr->t = t + 3000u + (uint32_t)(ea % 6) * 1000u;
                    nr->kind = E_NAVREQ;
                    nr->a = ((ea + ec) % 3 == 1) ? 821 + 4 * (ec % 19) : 901 + ec % 90;   /* STOP (kind 1) / fence */
                    nr->b = 100 + eb % 700;                                            /* a valid lease */
                    nr->c = 300 + ec % 600;
                    nr->f = 0.3f;
                    s->on[s->n - 1] = true;
                    t = nr->t;
                } else if (s->n + 1 < MAX_EV) {
                    /* step 13q2d (I69): otherwise the GPS lost (dead reckoning) or the gateway's shore data gone /
                     * not valid a few seconds later */
                    ev_t *gl = &s->ev[s->n++];
                    memset(gl, 0, sizeof *gl);
                    gl->t = t + 2000u + (uint32_t)(eb % 5) * 1000u;
                    gl->kind = (eb % 3 == 0) ? E_SHORE : E_GPS;
                    gl->a = (eb % 3 == 0) ? (eb % 2 == 0 ? 0 : 3) : eb % 400;   /* shore none / not valid; GPS fail */
                    s->on[s->n - 1] = true;
                    t = gl->t;
                }
            }
        }
    }
}

/* -------------------------------------------------------------- the sim -- */
#define AT_OR_AFTER(a, b)  ((int32_t)((uint32_t)(a) - (uint32_t)(b)) >= 0)
typedef struct {
    world_t     w;
    sb_t        sb;
    sb_config_t cfg;
    link_t      usb, gw;
    proto_rx_t  usb_rx, gw_rx;
    mon_t       mon;
    uint32_t    t, next_ctrl, last_kick, boot_until, wdt_ms, frozen_until, resets, boots;
    uint32_t    boot_ms;            /* step 13e (review 1f7817c LOW-3): the time of the last boot() (I51) */
    bool        frozen;
    bool        gw_on;
    uint32_t    gw_next;
    /* the storm's "hands" */
    float       jx, jy;
    uint32_t    joy_hold_until;
    bool        joy_fitted;
    bool        joy_broken;         /* a broken wire: the reading goes out of range */
    bool        confirm_held;
    uint32_t    confirm_release;
    bool        take_held, auto_held;
    uint32_t    btn_release;
    bool        rem_stream, rem_stale;
    bool        sconf_held;         /* the shore captain's CONFIRM button */
    bool        trig_held;          /* step 13q3: the shore stick's TRIGGER */
    uint32_t    trig_release;
    uint8_t     shore_mode;         /* GW_SHORE the Pi sends: 0 none, 1 near (40 m), 2 far (900 m), 3 not valid */
    uint32_t    gw_off_t;           /* when the Pi went quiet (0 = it talks) */
    uint32_t    sconf_release;
    bool        uncrewed;
    uint16_t    rem_seq;
    uint8_t     rem_station;
    float       rx_x, rx_lever;
    float       rx_y;                    /* step 13q5: the shore stick Y (MANEUVER gear; beside the lever) */
    uint32_t    rx_y_until;              /* a flick: back to the centre then (0 = none) */
    uint32_t    mvd_t0, mvd_until;       /* step 13q5c: a MANEUVER packet held back from t0, it arrives at until (0 = none) */
    bool        mvd_saved;
    float       mvd_y;                   /* ... the stick Y it carries, sent at mvd_sent */
    uint32_t    mvd_sent;
    uint32_t    inj_kind, inj_until;
    uint32_t    nav_seq;
    uint32_t    sched_seed;
    bool        pc_on;              /* a ground station on the USB link: heartbeats like the real one */
    uint32_t    pc_next;
    uint32_t    crank_until, crank_next;    /* the station's START button held until (0 = not): CRANK at 5 Hz */
    uint8_t     crank_station;
    bool        estop;              /* the e-stop is pressed (its contact is fitted from the first E_ENGINE e-stop event) */
    uint8_t     can;                /* step 11e: the episode's servos (sched_t.can) */
    uint32_t    can_back_t;         /* a CAN fault ends (0 = none) */
    uint8_t     can_fault_kind, can_fault_node;
    bool        dm_armed;           /* UNCREWED: the dead-man timer relay is in the kill circuit */
    /* step 13h2 (I36 W42): a cut DroneCAN ON - the relay board goes silent, an ON one frame before it is "lost", the
     * board back after w42_away ms (sometimes rebooted while away) */
    uint8_t     w42_n;              /* 0 = no such scenario running */
    uint32_t    w42_t0, w42_cmd_t, w42_away;
    bool        w42_reboot, w42_rebooted, w42_cable;
    /* step 13k (W44; review c162aad LOW-3): a YOUNG board - rebooted at the scenario's start, the silence begins
     * w42_young ms later (so a reboot while away does not send the uptime back: only the clock test sees it) */
    uint32_t    w42_young_t;
    /* step 13q1: LINK TIME - the gateway reports the HELM station's age (GW_STATUS bytes 10-11) in this episode */
    bool        rep_on;
    bool        helm_silent;        /* the HELM station stopped answering at silent_t */
    uint32_t    silent_t;
    uint32_t    heard_t;            /* the storm's view: the HELM station last heard (its pong ~0.5 s old) */
    uint8_t     lt_expect;          /* the LINK TIME the AURIX should hold (3 after a boot / a HELM grant) */
    bool        rep_since_grant;    /* a GW_STATUS with the report went out since the last HELM grant */
    uint32_t    far_since;          /* open sea (fresh far GW_SHORE, gateway talking) since (0 = not) */
    uint32_t    shore_tx_t;         /* the last GW_SHORE pushed */
    uint8_t     shore_tx_mode;      /* ... and what it said (shore_mode changes reach the AURIX only with the next push) */
    uint32_t    fuzz_link_t;        /* a CRC-valid random GW_STATUS / GW_CMD / HELM / GW_STATION frame (I60-I62 wait) */
    /* step 13q2e (I70): the gateway's 2 Hz navreq heartbeat + what the storm last sent */
    uint32_t    nav_beat_next, nav_beat_off_until;
    uint32_t    nav_tx_t;           /* the last VALID navreq pushed (0 = none since the boot) */
    uint16_t    nav_tx_lease;
    uint32_t    fuzz_gw_t;          /* a CRC-valid random gateway frame of any type (I70 waits) */
    uint32_t    slam_ans_t;         /* step 13q4 (I77): the captain answers the restore question then (0 = no) */
    bool        slam_ans_acc;
} sim_t;

static sim_t S;

/* step 13q1: HELM to `to` like link.c helm_set (a grant = LINK TIME back to 3 s, the station counts as heard) */
static void st_helm(sb_t *s, uint8_t to)
{
    uint8_t old = s->helm;
    s->helm = to;
    s->helm_local = false;
    sb_helm_changed(s, old, to, false);
    if (to != SB_HELM_NONE && to != old) {
        S.lt_expect = SB_LINK_TIME_MIN_S;
        S.heard_t = S.t;
        S.rep_since_grant = false;
    }
    if (to != SB_HELM_NONE) S.helm_silent = false;              /* it talks: its command came through */
}
static bool s_log;              /* soak: core 0 log producer + core 2 logger (SD card in test_results/soak_sd) */
static logger_t s_lg;
static uint32_t s_ctrl_max_us, s_ctrl_n;
static uint32_t s_hist[64];                         /* control step time histogram, 100 us bins */
static double   s_ctrl_sum_us;

static uint16_t joy_raw(float v)
{
    if (v > 1.0f) v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    return (uint16_t)lrintf(2048.0f + v * 1800.0f);
}

static void joy_apply(void)
{
    sim_hal_set_joystick(S.joy_fitted, S.joy_broken ? 20u : joy_raw(S.jx), joy_raw(S.jy), S.take_held, S.auto_held);
    sim_hal_joystick_confirm(S.confirm_held);
}

static void boot(void)
{
    sim_hal_can_act_clear();            /* I51: the first ArrayCommand of this boot */
    S.boot_ms = S.t;                    /* (step 13e: here, not at the first check after it) */
    hal_init();
    sb_init(&S.sb, &S.cfg);
    link_init(&S.usb, &S.sb);
    link_init_port(&S.gw, &S.sb, LINK_PORT_GW);
    monitor_init(&S.mon, S.t);
    S.next_ctrl = S.t;
    S.last_kick = S.t;
    S.boots++;
    S.lt_expect = SB_LINK_TIME_MIN_S;   /* LINK TIME: 3 s after every reset (step 13q1) */
    S.nav_tx_t = 0u;                    /* I70: the AURIX's navreq lease is gone with the reset */
    S.helm_silent = false;
    S.heard_t = S.t;
    S.rep_since_grant = false;
    if (S.uncrewed) {                   /* the shore captain holds HELM, its joystick sends (gateway) */
        S.rem_station = 2u;
        st_helm(&S.sb, 2u);
        S.rem_stream = true;
        S.rx_x = 0.0f;
        S.rx_y = 0.0f;
        S.rx_lever = 0.5f;
    }
    S.rx_y_until = 0u;
    if (s_log) {
        g_logstat.boot = S.boots;
        logger_init(&s_lg, S.t);
        logcore0_init(&S.sb, S.boots);
    }
}

static void sim_init(const sched_t *sc)
{
    memset(&S, 0, sizeof S);
    S.can = sc->can;
    sim_hal_set_time_ms(0u);            /* (first: the CAN reset times the nodes from "now") */
    sim_hal_can_reset();                /* step 11e: the DroneCAN bus + servo nodes, or no bus at all */
    sim_hal_can_fault(0u);
    sim_hal_can_task_stuck(false);
    sim_hal_can_fitted(sc->can != 0u);
    if (sc->can == 2u) {
        sim_hal_can_servo_speed(DC_NODE_THROTTLE, 2.0f, 0.0f);
        sim_hal_can_servo_speed(DC_NODE_STEERING, 3.0f, 0.03f);
    }
    sim_hal_deadman_arm(false);
    S.sched_seed = sc->seed;
    world_init(&S.w, sc->seed | 1u, 0.0f, 0.0f);
    S.w.sea_state = sc->sea0;
    sim_hal_set_time_ms(0u);
    sim_hal_fail_heading(false);
    sim_hal_set_rudder_feedback(false);
    sim_hal_set_gyro(true, 0.05f, 0.02f);
    sim_hal_set_accel(true);
    sim_hal_set_attitude(true);
    sim_hal_set_neutral_switch(false, true);
    sim_hal_set_estop(false, false);
    sim_hal_set_supply(true, 12.6f);
    sim_hal_set_motor_current(false, 0.0f);
    sim_hal_set_limit_switches(false, false, false);
    sim_hal_reset_engine();
    sim_hal_bridge_mode(false);
    sim_hal_unforce();
    remove("nv1.bin");
    remove("nv2.bin");
    remove("nv4.bin");
    remove("nv7.bin");                  /* no relay table from an earlier episode */
    remove("nv5.bin");                  /* no TRAINING result from an earlier episode */
    sim_hal_relay_board(true, 5u, 0u);  /* a fresh relay board (every relay off), 16 channels */
    sim_hal_relay_board_reset();
    sim_hal_relay_fitted(16u);
    /* step 11f: the DroneCAN episodes also have the relay board on DroneCAN (can_relay.c + the simulated node 30):
     * I36 / I37 / I38 are judged on both buses */
    sim_hal_relay_bus(sc->can ? SIM_RELAY_DRONECAN : SIM_RELAY_MODBUS);
    S.joy_fitted = !sc->uncrewed;
    S.uncrewed = sc->uncrewed;
    S.jx = sc->boot_deflected ? 0.8f : 0.0f;
    S.jy = 0.0f;
    joy_apply();
    memset((void *)&g_mon, 0, sizeof g_mon);
    world_publish(&S.w, 0u);
    sb_default_config(&S.cfg);
    timing_init_all();
    logq_init_all();
    sim_hal_link_reset();
    sim_hal_gw_reset(921600u);
    proto_rx_init(&S.usb_rx);
    proto_rx_init(&S.gw_rx);
    S.wdt_ms = 100u;
    S.gw_on = true;
    S.shore_mode = (uint8_t)(sc->seed % 4u);
    S.rep_on = ((sc->seed * 2654435761u) >> 20) % 4u != 0u;     /* step 13q1: 3 of 4 episodes, from the seed only */
    if (s_log) {
        (void)system("mkdir soak_sd 2>nul & del /q soak_sd\\*.TLG 2>nul");
        sim_hal_sd_setup("soak_sd", true, 64000000u, false);
        memset((void *)&g_logstat, 0, sizeof g_logstat);
    }
    boot();
}

static void push_frame(bool gw, uint8_t type, const uint8_t *pl, uint8_t len)
{
    uint8_t fr[PROTO_MAX_FRAME];
    size_t n = proto_encode(fr, type, (uint8_t)S.nav_seq, pl, len);
    if (gw) sim_hal_gw_rx_push(fr, (uint32_t)n);
    else sim_hal_link_rx_push(fr, (uint32_t)n);
}

/* the sample relay table of boats/rf49d.json (7 relays); variant = a different one (the same with
 * the bilge pump's time changed), so the AURIX sees a table that differs from the stored one */
static void relay_sample_table(relay_table_t *t, bool variant)
{
    static const relay_def_t R[7] = {
        { 1, RELAY_ONOFF,     RELAY_GRP_ENGINE, 0, 0,    RELAY_F_SHOW_HUD, RELAY_ROLE_IGNITION },   /* IGNITION */
        { 2, RELAY_MOMENTARY, RELAY_GRP_ENGINE, 0, 10,   0, RELAY_ROLE_STARTER },  /* STARTER */
        { 3, RELAY_ONOFF,     RELAY_GRP_ENGINE, 0, 0,    RELAY_F_SHOW_HUD, RELAY_ROLE_SHUTOFF },    /* SHUTOFF */
        { 4, RELAY_MOMENTARY, RELAY_GRP_TRIM,   5, 30,   RELAY_F_SHOW_HUD, 0 },   /* TRIM UP */
        { 5, RELAY_MOMENTARY, RELAY_GRP_TRIM,   4, 30,   RELAY_F_SHOW_HUD, 0 },   /* TRIM DOWN */
        { 6, RELAY_ONOFF,     RELAY_GRP_EXTRA,  0, 0,    RELAY_F_KEEP, 0 },       /* NAV LIGHTS */
        { 7, RELAY_TIMED,     RELAY_GRP_EXTRA,  0, 1200, 0, 0 },                  /* BILGE PUMP */
    };
    memset(t, 0, sizeof *t);
    t->count = 7u;
    memcpy(t->r, R, sizeof R);
    if (variant) t->r[6].max_ds = 600u;
    relay_table_seal(t);
}

static void navreq(uint8_t kind, uint8_t reason, uint8_t alarm, float hdg, float cap, uint16_t lease)
{
    uint8_t pl[11];
    uint16_t h = (uint16_t)lrintf(hdg * 100.0f), c = cap < 0.0f ? 0xFFFFu : (uint16_t)lrintf(cap * 10.0f);
    pl[0] = (uint8_t)(++S.nav_seq);
    pl[1] = kind; pl[2] = reason; pl[3] = alarm;
    pl[4] = (uint8_t)h; pl[5] = (uint8_t)(h >> 8);
    pl[6] = (uint8_t)c; pl[7] = (uint8_t)(c >> 8);
    pl[8] = (uint8_t)lease; pl[9] = (uint8_t)(lease >> 8);
    push_frame(true, PROTO_T_GW_NAVREQ, pl, 10u);
    /* I70 (13q2e): the storm's own copy of link.c's validity rule (an invalid one is ignored by the AURIX) */
    bool valid = kind <= SB_NAV_HDG_SET && !(kind != SB_NAV_NONE && kind != SB_NAV_CAP && h >= 36000u) &&
                 !(kind == SB_NAV_HEADING && c == 0xFFFFu) && !(c != 0xFFFFu && c > 600u) && lease >= 500u &&
                 lease <= 10000u;
    if (valid) {                        /* (pushed = delivered, also while the storm's status is off) */
        S.nav_tx_t = S.t ? S.t : 1u;
        S.nav_tx_lease = lease;
    }
}

static void prog_random(int n, uint32_t *r)
{
    static program_t p;
    program_clear(&p);
    p.n = (uint16_t)n;
    p.final_action = (uint8_t)rndi(r, 2);
    double north = S.w.north_m, east = S.w.east_m;
    for (int i = 0; i < n; i++) {
        double br = rnd(r) * 6.283, d = 300.0 + rnd(r) * 1500.0;
        north += d * cos(br);
        east += d * sin(br);
        world_t tmp = S.w;
        double lat, lon;
        tmp.north_m = north;
        tmp.east_m = east;
        world_latlon(&tmp, &lat, &lon);
        p.wp[i].lat_e7 = (int32_t)lround(lat * 1e7);
        p.wp[i].lon_e7 = (int32_t)lround(lon * 1e7);
        p.wp[i].speed_kn = 4.0f + rnd(r) * 16.0f;
        p.wp[i].radius_m = 25.0f;
        p.wp[i].action = 0;
    }
    program_seal(&p);
    prog_error_t err;
    uint8_t bad;
    (void)sb_program_set(&S.sb, &p, &err, &bad);
}

/* --------------------------------------------------------- invariants -- */
#define NINV 80
static const char *INV_NAMES[NINV] = { "",
    "I1  throttle never above the power limit (except coming down at the closing rate) / remote cap",
    "I2  after power-up / reset the power limit is 30 % fwd / 5 % rev",
    "I3  gear never changes with the throttle open; idle -> neutral -> gear -> throttle",
    "I4  the STANDBY button always wins: idle + neutral within the time",
    "I5  the onboard joystick wins over autopilot / PC / remote / vision within the time",
    "I6  the PC / remote never take control from the onboard joystick",
    "I7  power raise, AUTO start, REMOTE / MANEUVER only with a CONFIRM within 10 s (onboard; UNCREWED: shore)",
    "I8  CONFIRM alone never changes the mode",
    "I9  fault / STANDBY: idle + neutral commanded before (or with) steering off",
    "I10 no mode change outside the allowed-transition table",
    "I11 outputs disarmed at power-up until neutral + joystick centred (CREWED)",
    "I12 remote data old / missing -> idle + neutral in time, never resumes by itself",
    "I13 slamming / caps never increase the throttle (never above what the mode asks)",
    "I14 (gateway: storm_gateway.py)", "I15 (gateway)", "I16 (gateway)", "I17 (gateway)",
    "I18 logging never delays the control step (20 Hz kept)",
    "I19 the CPU1 monitor forces the safe state on every injected violation in time",
    "I20 navigation / vision caps never steer the boat in other modes, never raise the throttle",
    "I21 (gateway)",
    "I22 UNCREWED: the onboard joystick has no effect (no MANUAL / HDG LOCK, CONFIRM only from the shore)",
    "I23 the profile CREWED / UNCREWED never changes outside STANDBY (stopped)",
    "I24 MANEUVER never commands throttle above idle",
    "I25 MANEUVER gear changes only at idle",
    "I26 (gateway: MANEUVER / REMOTE refused without a manual licence)",
    "I27 ENGINE KILL only after idle + neutral (one exception, OD-3: UNCREWED + throttle node lost = kill at once)",
    "I28 lost link near shore or with missing / stale shore info: ENGINE KILL at once (once idle + neutral; OD-3: node loss at once)",
    "I29 lost link at open sea: no kill before the grace time; kill after it unless the link returned",
    "I30 never an ENGINE KILL in CREWED",
    "I31 the ENGINE KILL stays latched until a POWER-ON reset OR the HELM station's restart (idle + neutral, e-stop released); a watchdog / software reset keeps it",
    "I32 generator: throttle above idle in neutral only while the generator is active, gear N confirmed by the switch",
    "I33 generator throttle <= min(generator cap, power limit)",
    "I34 generator only in STANDBY (not stopping), armed, neutral switch closed, no kill / e-stop / trip / FIRST TEST",
    "I35 every generator stop idle first: no throttle rise while stopping",
    "I36 relays: no control path (mode / throttle / gear / steering / alarms), pair never both, pulse <= 5 s, engine relays never automatic, none in FIRST TEST, no power_up after a watchdog reset; DroneCAN (W42): a trim / extra relay goes OFF only by a command / its pulse end / the board itself, ON only <= 10 s after its command and never after a board reboot",
    "I37 STARTER only with idle + neutral confirmed, e-stop released, HELM station + its link, no kill / trip, <= 5 s per hold",
    "I38 the engine-kill latch is cleared only by a power-on reset or the HELM station's restart under its conditions; the monitor latch only by a reset",
    "I39 DP thrusters 0 outside MODE_DP: within one step of every fault / STANDBY / e-stop (and the e-stop contactor)",
    "I40 outboard idle + neutral throughout DP (only the generator's neutral throttle), its steering off",
    "I41 CREWED joystick deflection in DP -> thrusters 0 + MANUAL (in time)",
    "I42 GPS lost / suspect in DP -> thrusters 0 + STANDBY (never a DP step without a trusted GPS)",
    "I43 thrust <= cap and within the heat budget (a HOT thruster at 0)",
    "I44 DP SPOT / JOG more than 50 m or 2 min off the spot -> DP ends (in that step)",
    "I45 the DP set speed (CRUISE / TRIP) never above dp_max_kn",
    "I46 a HOT thruster ends DP (never a DP step with a thruster HOT)",
    "I47 an emergency (STANDBY / e-stop / link loss / node loss / trip, also during a planned or generator stop) commands idle in that same step; a normal decrease never slower than rate_down",
    "I48 the throttle command never rises faster than rate_up (the generator's steps too)",
    "I49 DroneCAN: the gear servo only moves while the SIMULATED throttle servo is at idle",
    "I50 DroneCAN: a required node lost -> alarm + STANDBY within the node-lost time + a step; never out of STANDBY while one is lost",
    "I51 DroneCAN: commands stopped -> the nodes HOLD; after a boot the first frame is throttle IDLE, no gear frame before thr_travel_s; throttle node lost -> no gear change",
    "I52 DroneCAN: a monitor trip -> CPU1's safe frames within 120 ms (20 ms period + the monitor / control step), no core-0 actuator frame on the bus after it (latched; counted from just before the tripping monitor step - on the board TXBCR can't retract a frame already in transmission: 1 frame allowed there, review only)",
    "I53 DroneCAN: the steering adaptation never leaves the TRAINING bounds",
    "I54 DroneCAN UNCREWED: a monitor trip -> no dead-man pulse, the engine stopped within the dead-man time; CREWED: never a pulse",
    "I55 DroneCAN UNCREWED: throttle node lost -> ENGINE KILL in the same step; never in CREWED",
    "I56 DroneCAN: no steering frame after a boot until a mode commands steering; worst-case speed while the position is unknown",
    "I57 DroneCAN UNCREWED: any stop condition (trip / frozen core 0 / throttle node lost / kill) -> no dead-man pulse within a monitor period",
    "I58 AUTO RETURN starts only with a fresh CONFIRM that answers its request; the way back = the stored program's waypoints backwards (speed of the leg driven back over, radius, action, final STOP), which it never changes",
    "I59 a DP TRIP drives its own copy of the STORED program (also after an AUTO RETURN), never past its last waypoint",
    "I60 UNCREWED LINK TIME: the lost link comes no later than the detection time in force (3 s; the captain's LINK TIME only at open sea) + 1.5 s after the HELM station was last heard",
    "I61 UNCREWED LINK TIME: at open sea with the gateway's report the lost link never comes before the captain's LINK TIME (- 1.5 s)",
    "I62 LINK TIME always 3..60 s, 3 s after every reset and HELM grant, never in force near the shore / without shore information",
    "I63 REMOTE / MANEUVER entry: UNCREWED only at once on the HELM station's request with its stick centred + lever neutral (fresh); CREWED never without the onboard CONFIRM, a CREWED MANEUVER CONFIRM only with the shore stick X + Y centred (fresh, WB)",
    "I64 there is no release: a release (command / old flag) never ends REMOTE / MANEUVER, never changes the mode",
    "I65 AUTO RETURN never out of REMOTE / MANEUVER: refused there (nothing pending), never started from them",
    "I66 HDG LOCK straight from REMOTE / MANEUVER only UNCREWED by the HELM captain's command (fresh shore data, stick centred); CREWED never (the panel button only)",
    "I67 UNCREWED HDG LOCK: the throttle never above its value at the entry (never rises by itself), never reverse, the gear only the entry gear or N",
    "I68 UNCREWED HDG LOCK: a navreq STOP (cap 0) = idle + neutral for the rest of it; a geofence breach = STANDBY in that step; stale shore data = idle + neutral",
    "I69 UNCREWED HDG LOCK (13q2d): never entered without valid fresh GW_SHORE / a trusted GPS; GW_SHORE missing / not valid = idle + neutral; GPS lost = held x dr_speed_factor; the DR limit = STANDBY",
    "I70 UNCREWED HDG LOCK (13q2e): never entered without a fresh navreq lease / with the shore within the stop distance; a lapsed (or never) lease or the shore within it = idle + neutral",
    "I71 shore stick override (13q3): only UNCREWED in AUTO / UNCREWED HDG LOCK (never CREWED), the core in angle mode, the command within the outboard range and the REMOTE rate limit",
    "I72 UNCREWED AUTO (13q3): the shore lever is only a ceiling (a cut demand never above it), never reverse",
    "I73 UNCREWED AUTO (13q3): shore data lost while steering by hand = idle at once + neutral, STANDBY by remote_lost_ms; never steering by hand on stale data",
    "I74 the TRIGGER (13q3): a new heading only in UNCREWED HDG LOCK while steering by hand, no CONFIRM pending, = the heading at the press",
    "I75 UNCREWED AUTO steered by hand (13q3b): shore data bad / the navreq lease gone / the shore within the stop distance = idle + neutral for the rest of that AUTO",
    "I76 a HELM change ends a running shore stick override within one step (13q3b)",
    "I77 the slamming cap rises only after the HELM station's ACCEPT, never faster than the throttle up rate (13q4)",
    "I78 MANEUVER gear (13q5): FORWARD / REVERSE asked only after the shore stick Y held full that way >= 0.3 s on taken packets (by their SEND time, 13q5c); centre / the other way = NEUTRAL",
    "I79 UNCREWED HDG LOCK entered from MANEUVER (13q5c): only with the stick Y centred (also at a stick-Y station), held gear N, never a FORWARD demand in it" };

typedef struct {
    uint32_t checks[NINV], fails[NINV];
    float    worst[NINV];           /* reaction times (s) */
    uint32_t worst_seed[NINV];      /* the episode with the worst reaction (--replay it) */
    uint32_t first_fail_seed[NINV];
    char     first_fail_text[NINV][400];
    uint32_t auto_by_button, auto_by_confirm;
    uint32_t dp_trip_ret;           /* I59: DP TRIP steps while an AUTO RETURN was latched */
    uint32_t rem_req[16], remote_entries, remote_direct, release_tries;
    uint32_t uh_entries, uh_refused, uh_stops, uh_fence, uh_stale;    /* step 13q2c (I66-I68) */
    uint32_t uh_nodata, uh_dr, uh_drlim;                              /* step 13q2d (I69): steps checked */
    uint32_t uh_nodata_hot, uh_dr_hot;                                /* ... of them with a held throttle > 2 % */
    uint32_t uh_lapsed, uh_near, uh_lapsed_hot, uh_entry70;           /* step 13q2e (I70): steps checked, entries */
    uint32_t uh_ref_code[32];                                         /* step 13q2e: why the shore HDG LOCK was refused */
    uint32_t ov_steps_auto, ov_steps_hdg, ov_starts, ov_newhdgs, ov_lost, ov_cut, ov_trig;  /* step 13q3 (I71-I74) */
    uint32_t ov_stop_steps, ov_helm;                                  /* step 13q3b (I75 / I76) */
    uint32_t slam_acc, slam_rej, slam_refused, slam_rises, slam_asks; /* step 13q4 (I77) */
    uint32_t mv_y, mv_asks, mv_n_checks;                             /* step 13q5 (I78) */
    uint32_t mv_delayed, mv_jitter, uh_from_mnv, uh_mnv_stick_y;      /* step 13q5c (I78 / I79) */
    uint32_t mnv_crewed_conf;                                         /* step WB (I63): CREWED MANEUVER CONFIRM entries */
    uint32_t engaged_ignored;       /* HDG LOCK / AUTO / REMOTE entered while the joystick is ignored (not centred) */
    uint32_t engaged_ignored_seed;
} stats_t;
static stats_t ST;

/* the allowed transitions per CLAUDE.md (written here independently of app/modes.c) */
static bool spec_allowed(sb_mode_t f, sb_mode_t t)
{
    if (f == t) return true;
    switch (f) {
    case MODE_STANDBY:  return true;                                       /* every mode starts in STANDBY */
    case MODE_MANUAL:   return t == MODE_STANDBY || t == MODE_HDG_LOCK || t == MODE_AUTO;   /* never REMOTE / TRAINING */
    case MODE_HDG_LOCK: return t == MODE_STANDBY || t == MODE_MANUAL || t == MODE_AUTO || t == MODE_REMOTE;
    case MODE_AUTO:     return t == MODE_STANDBY || t == MODE_MANUAL || t == MODE_HDG_LOCK || t == MODE_REMOTE;
    case MODE_TRAINING: return t == MODE_STANDBY || t == MODE_MANUAL;
    /* step 13q2 (owner 2026-10-31): REMOTE / MANEUVER are left by choosing another mode: STANDBY, AUTO, DP (their
       own start rules), the other one; the onboard joystick (MANUAL). Step 13q2c: HDG LOCK (UNCREWED only: I66) */
    case MODE_REMOTE:   return t == MODE_STANDBY || t == MODE_MANUAL || t == MODE_MANEUVER || t == MODE_AUTO || t == MODE_DP ||
                               t == MODE_HDG_LOCK;
    case MODE_MANEUVER: return t == MODE_STANDBY || t == MODE_MANUAL || t == MODE_REMOTE || t == MODE_AUTO ||
                               t == MODE_DP || t == MODE_HDG_LOCK;   /* captain: dock */
    case MODE_BENCH:    return t == MODE_STANDBY || t == MODE_MANUAL;              /* dock test: back, or the joystick */
    case MODE_DP:       return t == MODE_STANDBY || t == MODE_MANUAL;              /* thrusters 0 first; the joystick */
    default:            return false;
    }
}

typedef struct {
    bool     failed;
    int      inv;
    uint32_t t;
    char     text[200];
    /* tracking */
    sb_mode_t mode_prev;
    float    thr_prev, env;
    int      gear_prev;
    uint32_t gear_change_t;
    uint32_t standby_press_t;       /* 0 = none pending */
    float    standby_thr;
    bool     standby_safe;          /* outputs were already idle + neutral at the press */
    uint32_t defl_t;                /* joystick deflection from centre: when */
    bool     defl_pending;
    bool     centred_prev;
    uint32_t confirm_t, autobtn_t, raise_req_t;
    uint32_t confirms_prev;
    bool     returning_prev;                /* I58 */
    uint8_t  conf_kind_prev;                /* I58 (step 13b): the request pending before this step's CONFIRM */
    uint32_t act_frames_pre;                /* I52 (step 13b): core 0's wire frames just before the tripping monitor step */
    bool     act_pre_valid;
    uint32_t gear_changes_prev;             /* I51 (step 13b): the gear node's command changes (sim count) */
    uint32_t confirm_free_t;        /* a CONFIRM with nothing pending */
    sb_mode_t confirm_free_mode;
    uint32_t rem_last_fresh;        /* last fresh remote packet (0 none) */
    bool     rem_check;
    uint32_t inj_t;
    bool     inj_seen_forced;
    uint32_t freeze_t;
    float    plim_f_prev, plim_r_prev;
    uint32_t boot_seen;
    bool     armed_ok_seen;         /* centred + neutral seen since the boot */
    uint32_t steps, steps_t0;
    float    target_prev;
    bool     rem_lost_session;      /* this REMOTE session went LOST: it must never drive again by itself */
    uint32_t last_action_t;         /* the last event that may legitimately change the mode */
    uint32_t sconf_t;               /* last shore CONFIRM press */
    uint8_t  prof_prev;
    bool     killed;                /* the ENGINE KILL output was seen on (since this boot) */
    uint32_t safe_since;            /* outputs idle + neutral since (0 = not) */
    uint32_t lost_t;                /* the link loss the AURIX saw (its ll.lost_ms) */
    float    gen_env;               /* I33: the generator cap envelope (coming down at the closing rate) */
    uint32_t relay_t;               /* I36: a relay event was applied (0 = none pending) */
    sb_mode_t relay_mode_at;        /* ... the mode at the event */
    float    relay_thr_mode;        /* ... what the mode asked for, the step before it */
    gear_t   relay_gear_demand;
    uint8_t  relay_ap_mode;
    uint32_t relay_alm, relay_alm2;
    float    thr_mode_prev;
    gear_t   gear_demand_prev;
    uint8_t  ap_mode_prev;
    uint32_t alm_prev, alm2_prev;
    uint32_t rs_t;                  /* I31 / I38: a RESTART was asked (0 = none) ... */
    bool     rs_ok;                 /* ... by the HELM station with idle + neutral + e-stop released (the storm's view) */
    uint8_t  ll_prev;
    bool     forced_prev;
    uint32_t kc_prev;               /* eng.kill_clears */
    uint32_t st_on_since, st_bad_since;     /* I37: the STARTER coil on since / on while the conditions are gone */
    uint32_t eng_pulses_prev, st_pulse_t;   /* I36: STARTER pulses sent by engine.c / the last one */
    uint32_t edges_prev[4];                 /* I36: off -> on changes of the board's coils 1..3 */
    uint32_t eng_on_t[4];                   /* I36: an accepted engine-relay ON command (relay 1 / 3) */
    bool     w42_prev[10];                  /* I36 W42 (step 13h2): the board's coils 4..9 last step */
    uint32_t w42_on_edge[10];               /* ... when each came ON */
    bool     alarm_flag_prev;
    float    dp_tgt_prev;                   /* I20: DP heading target last step */
    /* step 11e */
    float    standby_est;                   /* I4: the modelled throttle at the STANDBY press */
    float    gear_truth_prev, thr_truth_prev;   /* I49 */
    bool     node_lost_prev;                /* I50 */
    uint32_t node_silenced_t;               /* I50: a required node made silent / unpowered (0 none) */
    uint32_t thr_fault_t;                   /* I29: the storm's own throttle-node fault / bus-off injection (0 none) */
    uint16_t gear_sent_prev;                /* I51 */
    float    gear_node_prev;                /* I51 (step 11g): the gear node's received command */
    bool     gear_node_have_prev;
    bool     thr_node_lost_prev;
    uint32_t boot_t;                        /* I51 / I56: the boot of this episode's last start */
    bool     boot_first_checked;
    bool     boot_gear_checked;             /* I51 (step 13d): the first gear frame on the bus of this boot judged */
    bool     steered_since_boot;            /* I56: a mode commanded steering since the boot */
    float    steer_u_prev;
    bool     steer_have_prev;
    uint32_t trip_t;                        /* I52 / I54: the latch seen (0 none) */
    uint32_t safe_frames_at_trip, act_frames_at_trip;
    bool     safe_seen;
    uint32_t dm_edges_prev;                 /* I54 / I57 */
    uint32_t dm_stop_t;                     /* I57: a stop condition began (0 none) */
    bool     thr_node_lost_prev_kill;       /* I55 */
    uint8_t  ll_prev55;
    uint8_t  prof_prev54;
    uint8_t  ll_prev61;                     /* I61 */
    uint8_t  helm_prev61;                   /* I62 */
    uint32_t lt_sets_prev;                  /* I62 */
    uint32_t uh_cmd_t;                      /* I66 (13q2c): the storm's shore HDG LOCK command changed the mode (0 none) */
    bool     uh_cmd_ok;                     /* ... with its conditions as the storm sees them */
    bool     uh_session;                    /* I67 / I68: UNCREWED HDG LOCK running */
    float    uh_cap, uh_thr0;               /* I67: the entry throttle (demand cap / output) */
    int      uh_gear0;                      /* I67: the gear at the entry (gear_pos: 0 N, 1 F) */
    bool     uh_stopped;                    /* I68: a STOP seen in this HDG LOCK */
    bool     uh_entry_bad;                  /* I69 (13q2d): entered while the storm's shore data / GPS was clearly bad */
    uint32_t uh_entry_t;
    bool     uh_nodata;                     /* I69: the shore data clearly bad in this HDG LOCK (latched) */
    uint32_t uh_nodata_t;
    int      uh_drlim_n;                    /* I69: consecutive HDG LOCK steps past the DR limit */
    int      uh_dr_n;                       /* I69: consecutive HDG LOCK steps without a trusted GPS */
    bool     uh_entry_bad70;                /* I70 (13q2e): entered with no fresh lease / the shore 40 m away */
    bool     uh_lapsed, uh_near;            /* I70: latched in this HDG LOCK */
    uint32_t uh_lapsed_t, uh_near_t;
    uint32_t near_since61;                  /* I60 */
    uint32_t gear_n_since28;                /* I28: the gear machine settled in N since (0 = not) */
    uint32_t rem_direct_t;                  /* I7 / I63: an UNCREWED REMOTE / MANEUVER request that started it at once */
    float    sent_x, sent_lever;            /* I63: the last shore joystick packet the storm sent */
    float    sent_y;                        /* (13q5: its stick Y) */
    uint16_t mv_seq;                        /* I78 (13q5): the AURIX's last taken packet seq */
    uint8_t  mv_zone;                       /* I78: the full-deflection zone of the taken packets (1 fwd, 2 back) */
    uint32_t mv_zone_t, mv_pkt_t;           /* I78: the first / newest taken packet in that zone (SEND times, 13q5c) */
    float    mv_sent_y;                     /* I78 (13q5c): the stick Y of the last packet sent to sb_remote_input */
    uint32_t mv_sent_t;                     /* ... its send time (S.t - its data age) */
    bool     uh_from_mnv;                   /* I79 (13q5c): this UNCREWED HDG LOCK was entered from MANEUVER */
    int      mv_dem_prev;                   /* I78: the gear demand after the last step */
    bool     ov_active_prev;                /* I71-I74 (13q3): the shore stick override before this step */
    float    ov_cmd_prev, ov_hdg_prev;
    bool     ov75, ov76;                    /* I75 / I76 (13q3b): latched / a HELM change while steering by hand */
    uint32_t ov75_t, ov76_t, ov76_starts;
    float    slam_cap_prev;                 /* I77 (13q4): the slamming cap before this step */
    uint32_t slam_now_prev, slam_boots, slam_asks_prev;
    bool     slam_acc;                      /* the storm's ACCEPT was taken (OK) since the last reduction */
    uint8_t  helm_prev76;
    uint32_t ov_now_prev, ov_newhdgs_prev, ov_starts_prev;
    uint32_t sent_t;
    bool     sent_fresh;
} chk_t;

static chk_t C;
static bool s_verbose;

static void fail(int inv, const char *fmt, double a, double b)
{
    ST.checks[inv]++;
    if (C.failed) return;
    C.failed = true;
    C.inv = inv;
    C.t = S.t;
    snprintf(C.text, sizeof C.text, fmt, a, b);
}

static void ok(int inv) { ST.checks[inv]++; }

/* I69 (step 13q2d): the shore data the storm's gateway sent is clearly NOT valid on the AURIX now - nothing sent for
 * longer than shore_max_age_ms + 200 ms, or the last frame said "not valid" (mode 3) and had 100 ms to arrive */
static bool shore_clearly_bad(const sb_t *s)
{
    if (!S.shore_tx_t || S.t - S.shore_tx_t > s->cfg.shore_max_age_ms + 200u) return true;
    return S.shore_tx_mode == 3u && S.t - S.shore_tx_t >= 100u;
}

/* I70 (step 13q2e): the AURIX's navreq lease has clearly run out (or never came since the boot): the last valid
 * request the storm pushed is older than its lease + 2 steps; a CRC-valid random gateway frame in the last 11 s
 * (it may have been a navreq) = not judged */
static bool i70_fuzzed(void) { return S.fuzz_gw_t && S.t - S.fuzz_gw_t < 11000u; }

static bool lease_clearly_gone(void)
{
    return !S.nav_tx_t || S.t - S.nav_tx_t > (uint32_t)S.nav_tx_lease + 2u * AP_PERIOD_MS;
}

/* I70: the last GW_SHORE said 40 m (valid, within SB_SHDG_STOP_M) and had 100 ms to arrive, still fresh */
static bool shore_clearly_near(const sb_t *s)
{
    return S.shore_tx_mode == 1u && S.shore_tx_t && S.t - S.shore_tx_t >= 100u &&
           S.t - S.shore_tx_t <= s->cfg.shore_max_age_ms;
}

/* one GW_RJOY packet of the HELM station's (shore) joystick, as the storm sends it (I63 remembers it) */
static void shore_packet(sb_t *s)
{
    uint16_t age = S.rem_stale ? 900u : 40u;
    sb_shore_joy(s, S.rem_station, age, S.rx_x, S.rx_y, S.rx_lever, true, S.sconf_held);
    sb_shore_trigger(s, S.rem_station, S.trig_held);          /* step 13q3 (link.c: the same GW_RJOY, flags b3) */
    C.sent_x = S.rx_x;
    C.sent_y = S.rx_y;
    C.sent_lever = S.rx_lever;
    C.sent_t = S.t ? S.t : 1u;
    C.sent_fresh = !S.rem_stale;
}

/* I37: the STARTER's other conditions as the storm sees them (smallboat.c helm_link_ok + no kill / trip) */
static bool crank_link_ok(const sb_t *s)
{
    bool link = s->helm == SB_HELM_USB ? !(s->alarm_bits & SB_ALM_PC_LINK) : (s->helm >= 2u && s->gw.state == SB_GW_OK);
    return link && s->ll.state != SB_LL_KILLED && !s->mon_tripped && !hal_outputs_forced();
}

static int gear_pos(uint16_t us)
{
    if (us == 0u) return -1;                                      /* no pulses (reset) */
    if (abs((int)us - (int)S.cfg.gear.pwm_n_us) < 60) return 0;
    if (abs((int)us - (int)S.cfg.gear.pwm_f_us) < 60) return 1;
    if (abs((int)us - (int)S.cfg.gear.pwm_r_us) < 60) return 2;
    return 3;                                                     /* in between (moving) */
}

static void worst(int inv, float v)
{
    if (v > ST.worst[inv]) {
        ST.worst[inv] = v;
        ST.worst_seed[inv] = S.sched_seed;
    }
}

/* step 11e: the DroneCAN servos (the episode runs on them) */
static bool can_ep(void) { return S.can != 0u && sim_hal_servos_can(); }

/* I52: core 0's ArrayCommands on the wire (priority 8; CPU1's safe frames are priority 0) */
static uint32_t core0_wire_frames(void)
{
    uint32_t n = 0u;
    for (uint8_t k = 1u; k <= 3u; k++) {
        sim_can_act_t a;
        sim_hal_can_act(k, &a);
        n += a.n_core0;
    }
    return n;
}

/* I47-I57: every control step (after the old invariants) */
static void check_can(sb_t *s, sb_mode_t m, float thr, bool forced, bool inj, float dt)
{
    float up = s->cfg.throttle.rate_up_pct_s, down = s->cfg.throttle.rate_down_pct_s;
    /* I47: an emergency stop commands idle in that same step; a normal decrease never slower than rate_down */
    if (!forced && !inj) {
        /* (a planned arrival stop, WHY_ARRIVED, keeps rate_down: not an emergency) */
        bool arrived = false;
        for (uint32_t n = mode_log_size(&s->modes), k = n; k > 0u && k + 8u > n; k--) {
            const mode_log_entry_t *e = mode_log_get(&s->modes, k - 1u);
            if (e->verdict == MODE_OK && e->to == MODE_STANDBY) { arrived = e->why == WHY_ARRIVED; break; }
        }
        /* step 11g: an emergency condition in THIS step (e-stop, a required node / the bus lost, the link lost /
         * killed, a STANDBY press) = idle at once also while already in STANDBY - during a planned arrival stop or
         * a normal generator stop too (the storm's own view, not sb emerg) */
        bool cond = s->estop_state == 2u || s->can.node_lost || s->can.bus_bad || s->ll.state == SB_LL_LOST ||
                    s->ll.state == SB_LL_KILLED || (s->buttons_now & HAL_BTN_BIT(HAL_BTN_STANDBY)) != 0u;
        bool emergency = (((m == MODE_STANDBY && C.mode_prev != MODE_STANDBY) || s->stopping) && !arrived) || cond;
        if (emergency) {
            if (s->throttle_demand <= 0.01f && thr <= 0.6f) ok(47);
            else fail(47, "emergency / STANDBY but the throttle command %.1f %% (demand %.1f %%)", thr, s->throttle_demand);
        } else if (s->throttle_demand < C.thr_prev - 0.01f) {
            float want = fmaxf(s->throttle_demand, C.thr_prev - down * dt);
            if (thr <= want + 0.6f) ok(47);
            else fail(47, "throttle coming down too slowly: %.1f %% (at rate_down at most %.1f %%)", thr, want);
        }
        /* I48: never up faster than rate_up */
        if (thr <= C.thr_prev + up * dt + 0.6f || thr <= 0.6f) ok(48);
        else fail(48, "throttle rose %.1f -> %.1f %% in one step (rate_up)", C.thr_prev, thr);
    }
    /* I47 (step 13b, review LOW-1): a monitor trip is an emergency too - core 0's own throttle command idle (the HAL
     * latch forces the outputs anyway; also during a planned arrival or generator stop) */
    if (forced && !inj && s->mon_tripped) {
        if (s->throttle_demand <= 0.01f) ok(47);
        else fail(47, "monitor tripped but core 0's throttle demand %.1f %%%.0f", s->throttle_demand, 0.0);
    }
    if (!can_ep()) return;
    float gt = sim_hal_gear_truth_us(), tt = sim_hal_thr_truth();
    /* I49: the gear servo moves only while the simulated throttle servo is at idle (also after a trip: CPU1's
     * neutral frame only after the throttle travel time) */
    if (!S.boot_until && C.gear_truth_prev > 0.0f && fabsf(gt - C.gear_truth_prev) > 2.0f) {
        if (tt <= 1.0f && C.thr_truth_prev <= 1.0f) ok(49);
        else fail(49, "the gear servo moved with the throttle servo at %.1f %% (before %.1f %%)", tt, C.thr_truth_prev);
    }
    /* I50: a required node lost -> alarm + STANDBY; never out of STANDBY while one is lost */
    if (s->can.node_lost) {
        if (!(s->alarm_bits2 & SB_ALM2_CAN_NODE)) fail(50, "a required node lost without the CAN NODE LOST alarm%.0f%.0f", 0.0, 0.0);
        else if (m != MODE_STANDBY && (C.node_lost_prev || C.mode_prev == MODE_STANDBY))
            fail(50, "a required node lost but mode %.0f%.0f", (double)m, 0.0);
        else ok(50);
    }
    if (C.node_silenced_t && !S.boot_until && !S.frozen) {
        if (s->can.node_lost) { ok(50); C.node_silenced_t = 0u; }
        else if (S.t - C.node_silenced_t > HAL_CAN_NODE_LOST_MS + 300u) {
            fail(50, "a node silent %.0f ms but not lost (limit %.0f)", (double)(S.t - C.node_silenced_t),
                 (double)HAL_CAN_NODE_LOST_MS);
            C.node_silenced_t = 0u;
        }
    }
    C.node_lost_prev = s->can.node_lost;
    /* I51: after a boot the first ArrayCommand is throttle IDLE; no gear frame before thr_travel_s; the throttle
     * node lost -> no gear change sent */
    if (!C.boot_first_checked && sim_hal_can_first_act() != 0u) {
        sim_can_act_t a;
        sim_hal_can_act(CA_ACT_THROTTLE, &a);
        if (sim_hal_can_first_act() == CA_ACT_THROTTLE && a.first <= -0.99f) ok(51);
        else fail(51, "the first frame after the boot: actuator %.0f, value %.2f", (double)sim_hal_can_first_act(), a.first);
        C.boot_first_checked = true;
    }
    if (s->servo.gear_sent && C.gear_sent_prev == 0u) {
        float since = (float)(int32_t)(S.t - S.boot_ms) * 0.001f;     /* step 13e: signed, from boot() itself */
        /* step 13d (review f9c3752 MED-3): the time alone - the old "|| est_pct <= 0" escape was always true here
         * (servo.c sends the first gear frame only at est_pct <= 0), so the clause could never fail */
        if (since + 0.06f >= s->servo.cfg.travel_s) ok(51);
        else fail(51, "a gear frame %.2f s after the boot (travel %.1f s)", since, s->servo.cfg.travel_s);
    }
    /* step 13d: and what the simulated bus really carried - the first gear ArrayCommand of this boot (core 0's or
     * CPU1's) not before thr_travel_s after the boot */
    if (!C.boot_gear_checked) {
        sim_can_act_t ga;
        sim_hal_can_act(CA_ACT_GEAR, &ga);
        if (ga.any) {
            /* step 13e (review 1f7817c LOW-3): signed, from boot() itself - a frame BEFORE the first check after
             * the boot (or before the boot) is negative = FAIL, never a wrapped huge "pass" */
            float since = (float)(int32_t)(ga.first_ms - S.boot_ms) * 0.001f;
            if (since + 0.06f >= s->servo.cfg.travel_s) ok(51);
            else fail(51, "the first gear frame on the bus %.2f s after the boot (travel %.1f s)", since,
                      s->servo.cfg.travel_s);
            C.boot_gear_checked = true;
        }
    }
    bool thr_lost = (s->can.lost_mask & (1u << HAL_CAN_ROLE_THROTTLE)) != 0u;
    /* step 11g: what the simulated GEAR NODE really got (core 0's frames AND CPU1's safe frames), not servo.gear_us */
    float gnode = 0.0f;
    bool ghave = sim_hal_can_servo(DC_NODE_GEAR, &gnode, NULL, NULL, NULL);
    uint32_t gch = sim_hal_can_servo_changes(DC_NODE_GEAR);       /* (step 13b: every changed frame the node got) */
    if (thr_lost && C.thr_node_lost_prev && C.gear_node_have_prev && ghave &&
        (fabsf(gnode - C.gear_node_prev) > 1e-4f || gch != C.gear_changes_prev))
        fail(51, "throttle node lost but the gear node's command changed %.3f -> %.3f", (double)C.gear_node_prev,
             (double)gnode);
    else if (thr_lost) ok(51);
    C.gear_changes_prev = gch;
    C.thr_node_lost_prev = thr_lost;
    C.gear_node_prev = gnode;
    C.gear_node_have_prev = ghave;
    C.gear_sent_prev = s->servo.gear_sent ? s->servo.gear_us : 0u;
    /* I52: a trip -> CPU1's safe frames within 20 ms (+ a monitor tick), core 0's actuator frames stop */
    if (forced && !C.trip_t) {
        C.trip_t = S.t ? S.t : 1u;
        C.safe_frames_at_trip = sim_hal_can_safe_frames();
        /* step 13b (review): the count from just BEFORE the monitor step that tripped - frames flushed by core 0 in
         * the first tick / step after the trip are counted against it */
        C.act_frames_at_trip = C.act_pre_valid ? C.act_frames_pre : core0_wire_frames();
        C.safe_seen = false;
    }
    if (forced && C.trip_t) {
        /* step 11g: core 0's ArrayCommands that really went onto the WIRE (not only queued) */
        uint32_t frames = core0_wire_frames();
        if (!C.safe_seen && sim_hal_can_safe_frames() != C.safe_frames_at_trip) { C.safe_seen = true; ok(52); }
        bool bus_ok = !s->can.bus_bad && (s->can.lost_mask & 7u) != 7u;
        if (!C.safe_seen && bus_ok && S.t - C.trip_t > MON_SAFE_FRAME_MS + 2u * AP_PERIOD_MS)
            fail(52, "no CPU1 safe frame %.0f ms after the trip%.0f", (double)(S.t - C.trip_t), 0.0);
        if (frames > C.act_frames_at_trip)
            fail(52, "core 0 sent %.0f actuator frames onto the bus after the trip%.0f", (double)(frames - C.act_frames_at_trip), 0.0);
        else ok(52);
    }
    /* I53: the adaptation inside the TRAINING bounds */
    if (s->ap.servo_have || s->ap.servo_steering) {
        if (sm_within_bounds(&s->ap.sm)) ok(53);
        else fail(53, "the steering model left the TRAINING bounds (K %.2f, play %.4f)", s->ap.sm.use.k, sm_play(&s->ap.sm));
    }
    /* I54 / I57: the dead-man pulses (CPU1). CREWED: never a pulse. UNCREWED: no pulse after a stop condition
     * (trip / frozen core 0 / throttle node lost / kill latched) - one monitor period of grace */
    uint32_t edges = sim_hal_deadman_edges();
    bool stop_cond = forced || S.frozen || thr_lost || s->ll.state == SB_LL_KILLED;
    if (s->profile == SB_PROFILE_CREWED && C.prof_prev54 == SB_PROFILE_CREWED) {   /* (CPU1 sees a change a step later) */
        if (edges != C.dm_edges_prev) fail(54, "a dead-man pulse in CREWED%.0f%.0f", 0.0, 0.0);
        else ok(54);
    }
    if (stop_cond) {
        if (!C.dm_stop_t) C.dm_stop_t = S.t ? S.t : 1u;
        else if (S.t - C.dm_stop_t > MON_DM_TOGGLE_MS + 30u && edges != C.dm_edges_prev)
            fail(forced ? 54 : 57, "dead-man pulses %.0f ms after a stop condition (trip / frozen / node lost / kill)%.0f",
                 (double)(S.t - C.dm_stop_t), 0.0);
        else if (S.t - C.dm_stop_t > MON_DM_TOGGLE_MS + 30u) ok(forced ? 54 : 57);
    } else {
        C.dm_stop_t = 0u;
    }
    if (forced && S.dm_armed && C.trip_t && S.t - C.trip_t > 1500u + 300u) {
        if (sim_hal_deadman_killed()) ok(54);
        else fail(54, "UNCREWED trip %.0f ms ago, the dead-man relay has not stopped the engine%.0f", (double)(S.t - C.trip_t), 0.0);
    }
    C.dm_edges_prev = edges;
    C.prof_prev54 = s->profile;
    /* I55: UNCREWED throttle node lost -> the kill latch in that same step (and never in CREWED: I30) */
    if (thr_lost && s->profile == SB_PROFILE_UNCREWED && !C.thr_node_lost_prev_kill) {
        if (s->ll.state == SB_LL_KILLED) ok(55);
        else fail(55, "UNCREWED throttle node lost, no ENGINE KILL in that step (state %.0f)%.0f", (double)s->ll.state, 0.0);
    }
    if (thr_lost && s->profile == SB_PROFILE_CREWED) {      /* (a kill latched earlier in UNCREWED may stay) */
        if (s->ll.state == SB_LL_KILLED && C.ll_prev55 != SB_LL_KILLED)
            fail(55, "CREWED throttle node lost and an ENGINE KILL%.0f%.0f", 0.0, 0.0);
        else ok(55);
    }
    C.thr_node_lost_prev_kill = thr_lost;
    C.ll_prev55 = s->ll.state;
    /* I56: no steering frame after a boot until a mode commands steering; the worst-case speed while the position
     * is unknown */
    bool have = false;
    float u = sim_hal_steer_cmd(&have);
    if (m != MODE_STANDBY) C.steered_since_boot = true;
    if (have && !C.steer_have_prev && !C.steered_since_boot)
        fail(56, "a steering frame after the boot in STANDBY (no mode steered)%.0f%.0f", 0.0, 0.0);
    else if (!C.steered_since_boot) ok(56);
    if (have && C.steer_have_prev && !sm_known(&s->ap.sm) && !forced) {
        if (fabsf(u - C.steer_u_prev) <= SM_RATE_WORST * dt * 1.05f + 1e-4f) ok(56);
        else fail(56, "steering command moved %.4f in a step while the position is unknown (worst case %.4f)",
                  fabsf(u - C.steer_u_prev), SM_RATE_WORST * dt);
    }
    C.steer_u_prev = u;
    C.steer_have_prev = have;
    C.gear_truth_prev = gt;
    C.thr_truth_prev = tt;
}

/* after every control step */
static void check_step(void)
{
    sb_t *s = &S.sb;
    sb_mode_t m = sb_mode(s);
    float thr = sim_hal_throttle();
    int gp = gear_pos(sim_hal_gear_pwm());
    bool forced = hal_outputs_forced();
    bool inj = S.inj_kind && !AT_OR_AFTER(S.t, S.inj_until + 51u);
    float dt = AP_PERIOD_MS * 0.001f;
    float down = s->cfg.throttle.rate_down_pct_s;
    C.steps++;

    /* I2: right after a boot */
    if (C.boot_seen != S.boots) {
        C.boot_seen = S.boots;
        C.armed_ok_seen = false;
        C.env = 0.0f;
        if (s->plim.fwd_pct == 30.0f && s->plim.rev_pct == 5.0f) ok(2);
        else fail(2, "power limit after the boot %.0f / %.0f", s->plim.fwd_pct, s->plim.rev_pct);
        C.plim_f_prev = s->plim.fwd_pct;
        C.plim_r_prev = s->plim.rev_pct;
        C.mode_prev = m;
        C.thr_prev = thr;
        C.gear_prev = gp;
        C.confirms_prev = s->confirm.accepted;
        C.prof_prev = s->profile;
        /* I31 / I38 (step 11b): a kill latched before a watchdog / software reset is latched after it too; a
         * power-on reset clears it */
        {
            bool was_killed = C.killed;
            bool now_killed = s->ll.state == SB_LL_KILLED && sim_hal_engine_killed();
            hal_reset_t why = hal_reset_reason(NULL);
            if (why != HAL_RESET_POWER_ON && was_killed) {
                if (now_killed) ok(31);
                else fail(31, "ENGINE KILL lost in a reset that is not a power-on (reason %.0f)%.0f", (double)why, 0.0);
            }
            if (why == HAL_RESET_POWER_ON && s->ll.state == SB_LL_KILLED)
                fail(38, "ENGINE KILL still latched after a power-on reset%.0f%.0f", 0.0, 0.0);
            else if (why == HAL_RESET_POWER_ON) ok(38);
            C.killed = sim_hal_engine_killed();
        }
        C.safe_since = 0u;
        C.ll_prev = s->ll.state;
        C.forced_prev = forced;
        C.kc_prev = s->eng.kill_clears;
        /* step 11e */
        C.boot_t = S.boot_ms;
        C.boot_first_checked = false;
        C.boot_gear_checked = false;
        C.steered_since_boot = false;
        C.steer_have_prev = false;
        C.gear_sent_prev = 0u;
        C.trip_t = 0u;
        C.act_pre_valid = false;
        C.thr_node_lost_prev = C.thr_node_lost_prev_kill = false;
        C.node_lost_prev = false;
        C.gear_truth_prev = sim_hal_gear_truth_us();
        C.thr_truth_prev = sim_hal_thr_truth();
        C.dm_edges_prev = sim_hal_deadman_edges();
        C.dm_stop_t = 0u;
        C.prof_prev54 = s->profile;
        C.relay_t = 0u;                 /* I36: a relay event across a reboot is not judged (the boot changes it all) */
        C.thr_mode_prev = s->throttle_mode;     /* ... and one right after it compares with the booted state */
        C.gear_demand_prev = s->gear_demand;
        C.ap_mode_prev = (uint8_t)s->ap.mode;
        C.alm_prev = s->alarm_bits;
        C.alm2_prev = s->alarm_bits2;
        return;
    }

    /* I10: transitions */
    if (m != C.mode_prev) {
        if (spec_allowed(C.mode_prev, m)) ok(10);
        else fail(10, "mode %.0f -> %.0f not in the table", (double)C.mode_prev, (double)m);
    }

    /* I11: not armed -> nothing moves */
    float tol = s->joy.cfg.center_tol * 0.9f;              /* "centred" as the controller sees it */
    bool near_centre = fabsf(S.jx) < tol * 1.4f && fabsf(S.jy) < tol * 1.4f;   /* (calibration differs a little) */
    if (near_centre && (gp == 0 || gp == -1)) C.armed_ok_seen = true;
    if (!s->armed && !forced && !inj) {
        if (thr <= 0.6f && (gp == 0 || gp == -1) && fabsf(sim_hal_motor_command()) < 0.01f) ok(11);
        else fail(11, "not armed but throttle %.1f %% / gear %.0f", thr, (double)gp);
    }
    if (s->armed && !C.armed_ok_seen && s->profile == SB_PROFILE_CREWED) fail(11, "armed although the joystick was never centred (%.1f, %.1f)",
                                             S.jx, S.jy);

    /* I1: under the limit, or coming down at the closing rate */
    if (!forced && !inj && gp >= 0) {
        float lim = (gp == 2) ? s->plim.rev_pct : s->plim.fwd_pct;
        if (m == MODE_REMOTE) lim = fminf(lim, gp == 2 ? s->cfg.remote_cap_rev_pct : s->cfg.remote_cap_fwd_pct);
        C.env = fmaxf(lim, C.env - down * dt);
        if (thr <= C.env + 1.0f) ok(1);
        else fail(1, "throttle %.1f %% above the limit envelope %.1f %%", thr, C.env);
        C.env = fmaxf(C.env, fminf(thr, C.env));
    }

    /* I3: gear change only at idle; throttle only after the engage wait */
    if (!forced && !inj && gp >= 0 && C.gear_prev >= 0 && gp != C.gear_prev && gp != 3 && C.gear_prev != 3) {
        if (thr <= 0.6f && C.thr_prev <= 0.6f) ok(3);
        else fail(3, "gear moved with the throttle at %.1f %% (before %.1f %%)", thr, C.thr_prev);
        C.gear_change_t = S.t;
    }
    if (!forced && !inj && C.gear_change_t && S.t - C.gear_change_t < (uint32_t)(s->cfg.gear.engage_wait_ms * 0.7f)) {
        if (thr <= 0.6f) ok(3);
        else fail(3, "throttle %.1f %% only %.0f ms after the gear engaged", thr, (double)(S.t - C.gear_change_t));
    }

    /* I4: the STANDBY button */
    if (C.standby_press_t && C.last_action_t > C.standby_press_t) {
        if (m == MODE_STANDBY || S.t - C.standby_press_t < 150u) ok(4);
        C.standby_press_t = 0u;                          /* a new action after STANDBY: it may change the mode */
    }
    if (C.standby_press_t) {
        int32_t dms = (int32_t)(S.t - C.standby_press_t);
        float el = (dms > 0 ? dms : 0) * 0.001f;
        /* step 11e: idle at once, the gear to neutral once the MODELLED throttle servo is at idle (est_pct, the
         * worst-case travel thr_travel_s) */
        float bound = C.standby_est * 0.01f * s->servo.cfg.travel_s + s->cfg.gear.idle_wait_ms * 0.001f + 0.8f;
        if (el > 0.15f && m != MODE_STANDBY && !forced) fail(4, "%.2f s after STANDBY still mode %.0f", el, (double)m);
        if (thr <= 0.6f && (gp == 0 || gp == -1)) {
            ok(4);
            worst(4, C.standby_safe ? 0.0f : el);
            C.standby_press_t = 0u;
        } else if (el > bound) {
            fail(4, "%.2f s after STANDBY: not yet idle + neutral (limit %.2f s)", el, bound);
            C.standby_press_t = 0u;
        }
    }

    /* I5 / I6: the onboard joystick */
    bool joy_ok = S.joy_fitted && !S.joy_broken && s->armed && !forced && !s->joy.fault && !s->mon_tripped &&
                  !s->can.node_block &&                                     /* DroneCAN: a node lost / rebooted: STANDBY only */
                  s->profile == SB_PROFILE_CREWED && s->joy.present && s->ll.state == SB_LL_OK &&   /* killed: STANDBY only */
                  s->estop_state != 2u;                                     /* e-stop pressed: STANDBY only */
    if (C.defl_pending && joy_ok) {
        float el = (S.t - C.defl_t) * 0.001f;
        if (m == MODE_MANUAL) {
            ok(5);
            worst(5, el);
            C.defl_pending = false;
        } else if (C.standby_press_t || (m == MODE_STANDBY && el < 0.12f)) {
            /* wait */
        } else if (el > 0.40f) {
            if (m == MODE_STANDBY && !s->joy.armed) {
                C.defl_pending = false;                  /* held through a STANDBY press: ignored until centred */
            } else {
                fail(5, "joystick deflected %.2f s ago, mode still %.0f", el, (double)m);
                C.defl_pending = false;
            }
        }
    } else if (C.defl_pending && !joy_ok) {
        C.defl_pending = false;
    }
    bool held = fabsf(S.jx) > tol * 1.5f || fabsf(S.jy) > tol * 1.5f;   /* clearly not centred */
    if (held && C.mode_prev == MODE_MANUAL && m != C.mode_prev) {
        if (m == MODE_STANDBY) ok(6);                    /* STANDBY / a fault may stop it */
        else fail(6, "stick held in MANUAL, mode went to %.0f (from %.0f)", (double)m, (double)C.mode_prev);
    }

    if (m != C.mode_prev && (m == MODE_HDG_LOCK || m == MODE_AUTO || m == MODE_REMOTE) && !s->joy.armed &&
        S.joy_fitted && !S.joy_broken && held) {
        if (!ST.engaged_ignored) ST.engaged_ignored_seed = S.sched_seed;
        ST.engaged_ignored++;
    }

    /* I7: raise / AUTO / REMOTE need an onboard CONFIRM within 10 s */
    bool conf_now = s->confirm.accepted != C.confirms_prev;
    if (conf_now) C.confirm_t = S.t;
    if (s->plim.fwd_pct > C.plim_f_prev + 0.1f || s->plim.rev_pct > C.plim_r_prev + 0.1f) {
        if (conf_now && C.raise_req_t && S.t - C.raise_req_t <= 10000u + 200u) ok(7);
        else fail(7, "power limit raised to %.0f / %.0f without a fresh request + CONFIRM",
                  s->plim.fwd_pct, s->plim.rev_pct);
    }
    if (m == MODE_AUTO && C.mode_prev != MODE_AUTO) {
        if (C.confirm_t && S.t - C.confirm_t <= 200u) { ok(7); ST.auto_by_confirm++; }
        else if (C.autobtn_t && S.t - C.autobtn_t <= 400u) { ok(7); ST.auto_by_button++; }
        else fail(7, "AUTO entered from mode %.0f without CONFIRM (last %.0f ms ago)", (double)C.mode_prev,
                  C.confirm_t ? (double)(S.t - C.confirm_t) : -1.0);
    }
    /* I58: AUTO RETURN only with a CONFIRM (also inside AUTO: no mode change), the way back = the stored
     * program backwards from ret_base, the stored program itself never touched (checksum still valid) */
    {
        const sb_auto_t *au = &s->autop;
        if (au->returning && !C.returning_prev) {
            if (C.mode_prev == MODE_REMOTE || C.mode_prev == MODE_MANEUVER)
                fail(65, "AUTO RETURN started out of mode %.0f (now %.0f)", (double)C.mode_prev, (double)m);
            if (conf_now && C.confirm_t && S.t - C.confirm_t <= 200u && C.conf_kind_prev == (uint8_t)CONF_AUTO_RETURN) ok(58);
            else fail(58, "AUTO RETURN started without a fresh CONFIRM of its request (kind before %.0f, last %.0f ms ago)",
                      (double)C.conf_kind_prev, C.confirm_t ? (double)(S.t - C.confirm_t) : -1.0);
        }
        if (au->returning) {
            bool same = program_checksum_ok(&au->orig) && au->ret_base < au->orig.n &&
                        au->prog.n == (au->ret_base ? au->ret_base + 1u : 1u) &&
                        au->prog.final_action == (uint8_t)PROG_END_STOP;
            for (unsigned j = 0; same && j < au->prog.n; j++) {
                unsigned i = au->ret_base - j;
                const prog_wp_t *a1 = &au->prog.wp[j], *b1 = &au->orig.wp[i];
                /* the speed: of the leg driven back over (i -> i+1); base 0 with one waypoint may be its own */
                bool sp = (i + 1u < au->orig.n && a1->speed_kn == au->orig.wp[i + 1u].speed_kn) ||
                          (au->ret_base == 0u && a1->speed_kn == b1->speed_kn);
                same = a1->lat_e7 == b1->lat_e7 && a1->lon_e7 == b1->lon_e7 && sp && a1->radius_m == b1->radius_m &&
                       a1->action == b1->action;
            }
            if (same) ok(58);
            else fail(58, "AUTO RETURN: the way back (%.0f waypoints from %.0f) is not the stored program backwards",
                      (double)au->prog.n, (double)au->ret_base);
        }
        C.returning_prev = au->returning;
    }
    /* I59 (step 13b, review H1): a DP TRIP drives its own copy of the STORED program (the one the gateway checked),
     * also while an AUTO RETURN's way back is the active program in RAM; never past its last waypoint */
    if (m == MODE_DP && s->dp.sub == DP_TRIP) {
        const program_t *sp = sb_program(s), *rt = &s->dp.route;
        bool same = rt->n == sp->n && rt->final_action == sp->final_action && s->dp.wp < rt->n;
        for (unsigned j = 0; same && j < rt->n; j++)
            same = rt->wp[j].lat_e7 == sp->wp[j].lat_e7 && rt->wp[j].lon_e7 == sp->wp[j].lon_e7 &&
                   rt->wp[j].speed_kn == sp->wp[j].speed_kn && rt->wp[j].radius_m == sp->wp[j].radius_m &&
                   rt->wp[j].action == sp->wp[j].action;
        if (s->autop.returning) ST.dp_trip_ret++;
        if (same) ok(59);
        else fail(59, "DP TRIP route (%.0f waypoints, at %.0f) is not the stored program", (double)rt->n, (double)s->dp.wp);
    }
    if (m == MODE_DP && C.mode_prev != MODE_DP) {
        if (C.confirm_t && S.t - C.confirm_t <= 200u) ok(7);
        else fail(7, "DP entered from mode %.0f without CONFIRM (last %.0f ms ago)", (double)C.mode_prev,
                  C.confirm_t ? (double)(S.t - C.confirm_t) : -1.0);
    }
    if ((m == MODE_REMOTE || m == MODE_MANEUVER) && C.mode_prev != m) {
        ST.remote_entries++;
        /* step 13q2: CREWED = the CONFIRM (I7); UNCREWED = at once on the request (I63, checked at the request) */
        if (C.rem_direct_t && S.t - C.rem_direct_t <= 100u) { ok(63); ST.remote_direct++; }
        else if (C.confirm_t && S.t - C.confirm_t <= 200u) {
            ok(7);
            /* step WB (owner W52b): a CREWED MANEUVER entered by the onboard CONFIRM = the HELM station's last shore
             * packet (sent before this step) fresh with the stick X AND Y centred */
            if (m == MODE_MANEUVER && s->profile == SB_PROFILE_CREWED) {
                ST.mnv_crewed_conf++;
                if (C.sent_t && C.sent_fresh && S.t - C.sent_t <= 300u && fabsf(C.sent_x) <= 0.10f &&
                    fabsf(C.sent_y) <= 0.10f)
                    ok(63);
                else
                    fail(63, "CREWED MANEUVER entered by the CONFIRM with the shore stick not centred / stale (x %.2f, "
                         "y %.2f)", (double)C.sent_x, (double)C.sent_y);
            }
        } else fail(7, "REMOTE / MANEUVER entered without CONFIRM / direct request (mode before %.0f, now %.0f)",
                  (double)C.mode_prev, (double)m);
    }

    /* I66 (step 13q2c): HDG LOCK straight from REMOTE / MANEUVER = UNCREWED, the HELM captain's command, fresh +
     * centred (CREWED: never; UNCREWED: never from another mode) */
    if (m == MODE_HDG_LOCK && C.mode_prev != MODE_HDG_LOCK) {
        bool from_rem = C.mode_prev == MODE_REMOTE || C.mode_prev == MODE_MANEUVER;
        if (from_rem || s->profile == SB_PROFILE_UNCREWED) {
            if (s->profile == SB_PROFILE_UNCREWED && from_rem && C.uh_cmd_t && S.t - C.uh_cmd_t <= 100u && C.uh_cmd_ok)
                ok(66);
            else
                fail(66, "HDG LOCK entered from mode %.0f (profile %.0f) without the HELM captain's shore command / its "
                     "conditions (fresh data, stick centred)", (double)C.mode_prev, (double)s->profile);
        }
        C.uh_session = s->profile == SB_PROFILE_UNCREWED;
        C.uh_stopped = false;
        C.uh_nodata = false;
        C.uh_drlim_n = C.uh_dr_n = 0;
        C.uh_lapsed = C.uh_near = false;
        if (C.uh_session && C.uh_entry_t && S.t - C.uh_entry_t <= 100u) {
            if (C.uh_entry_bad) fail(69, "UNCREWED HDG LOCK entered without valid shore data / a trusted GPS%.0f%.0f",
                                     0.0, 0.0);
            else ok(69);
            ST.uh_entry70++;
            if (C.uh_entry_bad70) fail(70, "UNCREWED HDG LOCK entered without a fresh navreq lease (last %.0f ms ago) / "
                                       "with the shore 40 m away (shore mode %.0f)",
                                       S.nav_tx_t ? (double)(S.t - S.nav_tx_t) : -1.0, (double)S.shore_tx_mode);
            else ok(70);
        }
    }
    /* I71-I74 (step 13q3, owner 2026-10-31 C3): the shore stick override in UNCREWED AUTO / HDG LOCK */
    {
        const sb_sovr_t *o = &s->sovr;
        if (o->active) {
            if (m == MODE_AUTO) ST.ov_steps_auto++; else ST.ov_steps_hdg++;
            bool where = s->profile == SB_PROFILE_UNCREWED && (sb_shore_auto(s) || sb_shore_hdg(s));
            float lim = s->ap.cfg.outboard.max_angle_deg;
            float step = s->cfg.remote_steer_rate_dps * (float)(s->now_ms - C.ov_now_prev) * 0.001f;
            bool rate = !C.ov_active_prev || fabsf(o->steer_cmd - C.ov_cmd_prev) <= step + 0.01f;
            if (where && s->ap.mode == AP_MODE_ANGLE && fabsf(o->steer_cmd) <= lim + 0.01f && rate) ok(71);
            else fail(71, "steering by hand with the shore stick in mode %.0f (profile / core mode / range / rate), "
                      "command %.2f deg", (double)m + 10.0 * (double)s->profile, (double)o->steer_cmd);
        }
        if (o->starts != C.ov_starts_prev) ST.ov_starts++;
        if (s->profile == SB_PROFILE_CREWED && o->starts != C.ov_starts_prev)
            fail(71, "a shore stick override started CREWED (mode %.0f)%.0f", (double)m, 0.0);
        if (sb_shore_auto(s) && !forced && !inj) {
            if (s->gear_demand == GEAR_R) fail(72, "UNCREWED AUTO gear demand reverse%.0f%.0f", 0.0, 0.0);
            else ok(72);
            if (o->ceil_cut) {
                ST.ov_cut++;
                /* (13q3b LOW-1: after a HELM swap the old captain's ceiling holds, frozen, until the new lever engages) */
                float b = o->ceil_on ? o->ceil_pct : (o->frozen ? o->frozen_pct : o->ceil_pct);
                if (o->ceil_on && o->frozen) b = fminf(b, o->frozen_pct);
                if (s->throttle_demand <= b + 0.01f) ok(72);
                else fail(72, "UNCREWED AUTO throttle demand %.2f %% above the lever ceiling %.2f %%",
                          (double)s->throttle_demand, (double)b);
            }
            if (o->lost) {
                ST.ov_lost++;
                bool idle = s->throttle_demand <= 0.0f && s->gear_demand == GEAR_N && s->thr.stop;
                bool late = s->now_ms - o->lost_ms > s->cfg.remote_lost_ms + 2u * AP_PERIOD_MS;
                if (idle && !late) ok(73);
                else fail(73, "UNCREWED AUTO after the shore data was lost while steering by hand: demand %.2f %%, "
                          "%.0f ms since (STANDBY due)", (double)s->throttle_demand, (double)(s->now_ms - o->lost_ms));
            }
        }
        /* never steering by hand on data the storm stopped (fuzzed gateway frames may carry GW_RJOY: skipped then) */
        if (o->active && !i70_fuzzed() && C.rem_last_fresh &&
            S.t - C.rem_last_fresh > s->cfg.remote_max_age_ms + 2u * AP_PERIOD_MS)
            fail(73, "steering by hand with the shore stick on data %.0f ms old (mode %.0f)",
                 (double)(S.t - C.rem_last_fresh), (double)m);
        else if (o->active) ok(73);
        if (o->newhdgs != C.ov_newhdgs_prev) {
            ST.ov_newhdgs++;
            bool by_hand = C.ov_active_prev || o->starts != C.ov_starts_prev;
            bool right = sb_shore_hdg(s) && by_hand && C.conf_kind_prev == (uint8_t)CONF_NONE &&
                         fabsf(autopilot_heading_error(autopilot_wrap360(C.ov_hdg_prev), o->newhdg_deg)) < 0.01f;
            if (right) ok(74);
            else fail(74, "a trigger heading %.1f (heading %.1f before the step) not in UNCREWED HDG LOCK steering "
                      "by hand / with a request pending", (double)o->newhdg_deg, (double)C.ov_hdg_prev);
        }
    }
    /* I75 (step 13q3b, review MED-1): steering UNCREWED AUTO by hand with the gateway's shore data clearly bad, the
     * navreq lease clearly gone or the shore clearly within the stop distance = idle + neutral from 2 steps on, for
     * the rest of that AUTO (latched); I76 (LOW-1): a HELM change ends a running override within one step */
    {
        const sb_sovr_t *o = &s->sovr;
        if (!sb_shore_auto(s) || C.mode_prev != MODE_AUTO) C.ov75 = false;
        if (sb_shore_auto(s) && o->active && !C.ov75 && !i70_fuzzed() &&
            (shore_clearly_bad(s) || lease_clearly_gone() || shore_clearly_near(s))) {
            C.ov75 = true;
            C.ov75_t = S.t;
        }
        if (C.ov75 && sb_shore_auto(s) && !forced && !inj && S.t - C.ov75_t >= 2u * AP_PERIOD_MS) {
            ST.ov_stop_steps++;
            if (s->throttle_demand <= 0.0f && s->gear_demand == GEAR_N && o->stopped) ok(75);
            else fail(75, "steering UNCREWED AUTO by hand without shore data / lease / near the shore: throttle demand "
                      "%.2f %%, gear demand %.0f", (double)s->throttle_demand, (double)s->gear_demand);
        }
        if (s->helm != C.helm_prev76 && C.ov_active_prev && sb_shore_auto(s)) {
            C.ov76 = true;
            C.ov76_t = S.t;
            C.ov76_starts = o->starts;
            ST.ov_helm++;
        }
        if (C.ov76) {
            if (!o->active || o->starts != C.ov76_starts || !sb_shore_auto(s)) { ok(76); C.ov76 = false; }
            else if (S.t - C.ov76_t >= 2u * AP_PERIOD_MS)
                fail(76, "the shore stick override still running %.0f ms after a HELM change (helm %.0f)",
                     (double)(S.t - C.ov76_t), (double)s->helm);
        }
        C.helm_prev76 = s->helm;
    }
    /* I77 (step 13q4, owner 2026-10-31): the slamming cap never rises by itself - only after the HELM station's
     * ACCEPT (the storm's own E_SLAMRES that the AURIX took), and never faster than the throttle up rate */
    {
        const slam_t *sl = &s->slam;
        if (C.slam_boots != S.boots) {          /* a reset: slam_init (RAM), no reduction - nothing to judge */
            C.slam_boots = S.boots;
            C.slam_acc = false;
        } else {
            if (sl->cap_pct < C.slam_cap_prev - 0.001f) C.slam_acc = false;     /* a new reduction: a new ACCEPT */
            if (sl->cap_pct > C.slam_cap_prev + 0.001f) {
                ST.slam_rises++;
                float lim = s->thr.cfg.rate_up_pct_s * (float)(s->now_ms - C.slam_now_prev) * 0.001f + 0.01f;
                if (C.slam_acc && sl->cap_pct - C.slam_cap_prev <= lim) ok(77);
                else fail(77, "slamming cap %.2f -> %.2f %% without an accepted restore / faster than the up rate",
                          (double)C.slam_cap_prev, (double)sl->cap_pct);
            }
            if (sl->asks != C.slam_asks_prev) ST.slam_asks++;
        }
        C.slam_cap_prev = sl->cap_pct;
        C.slam_now_prev = s->now_ms;
        C.slam_asks_prev = sl->asks;
    }
    if (m != MODE_HDG_LOCK) C.uh_session = false;
    if (C.uh_session && C.mode_prev == MODE_HDG_LOCK && m == MODE_HDG_LOCK && !forced && !inj) {
        /* I67: never above the entry throttle, never reverse, the entry gear or N */
        float dem = s->throttle_demand;
        if (dem <= C.uh_cap + 0.01f && thr <= C.uh_thr0 + 0.1f) ok(67);
        else fail(67, "UNCREWED HDG LOCK throttle demand %.2f / output %.2f %% above the entry value", dem,
                  fmaxf(thr - C.uh_thr0, dem - C.uh_cap) + (C.uh_thr0 > C.uh_cap ? C.uh_thr0 : C.uh_cap));
        if (s->gear_demand == GEAR_R || gp == 2 || (C.uh_gear0 != 1 && s->gear_demand != GEAR_N))
            fail(67, "UNCREWED HDG LOCK gear demand %.0f / position %.0f (not the entry gear or N)", (double)s->gear_demand,
                 (double)gp);
        else ok(67);
        /* I68: a fresh STOP (cap 0) = idle + N from now on; a fresh geofence breach = no HDG LOCK step; stale data */
        /* (a request the gateway link took in after this step - rx_ms == now_ms - is applied by the NEXT step) */
        bool navf = s->nav.rx_ms != 0u && s->nav.rx_ms != s->now_ms && s->now_ms - s->nav.rx_ms <= s->nav.lease_ms;
        if (navf && (s->nav.kind == SB_NAV_CAP || s->nav.kind == SB_NAV_HEADING) && s->nav.cap_kn >= 0.0f &&
            s->nav.cap_kn < SB_SHDG_STOP_KN) {
            if (!C.uh_stopped) ST.uh_stops++;
            C.uh_stopped = true;
        }
        if (C.uh_stopped) {
            if (s->throttle_demand <= 0.0f && s->gear_demand == GEAR_N) ok(68);
            else fail(68, "UNCREWED HDG LOCK after a STOP: throttle demand %.2f %%, gear demand %.0f", s->throttle_demand,
                      (double)s->gear_demand);
        }
        if (navf && (s->nav.alarm & SB_NAV_ALM_FENCE))
            fail(68, "UNCREWED HDG LOCK goes on with a geofence breach reported (alarm %.0f)%.0f", (double)s->nav.alarm, 0.0);
        if (C.rem_last_fresh && S.t - C.rem_last_fresh > s->cfg.remote_max_age_ms + 2u * AP_PERIOD_MS) {
            ST.uh_stale++;
            if (s->throttle_demand <= 0.0f && s->gear_demand == GEAR_N) ok(68);
            else fail(68, "UNCREWED HDG LOCK: shore data %.0f ms old, throttle demand %.2f %%",
                      (double)(S.t - C.rem_last_fresh), s->throttle_demand);
        }
    }
    if (C.uh_session && C.mode_prev == MODE_HDG_LOCK && m == MODE_HDG_LOCK && !forced && !inj) {
        /* I69 (step 13q2d): the AURIX's own "no data = shallow" - shore data clearly bad = idle + N from 2 steps on
         * (latched for this HDG LOCK) */
        if (!C.uh_nodata && shore_clearly_bad(s)) { C.uh_nodata = true; C.uh_nodata_t = S.t; }
        if (C.uh_nodata && S.t - C.uh_nodata_t >= 2u * AP_PERIOD_MS) {
            ST.uh_nodata++;
            if (C.uh_cap > 2.0f && C.uh_gear0 == 1) ST.uh_nodata_hot++;
            if (s->throttle_demand <= 0.0f && s->gear_demand == GEAR_N) ok(69);
            else fail(69, "UNCREWED HDG LOCK without valid shore data: throttle demand %.2f %%, gear demand %.0f",
                      s->throttle_demand, (double)s->gear_demand);
        }
        /* I70 (step 13q2e): a lapsed / never lease, or the shore within the stop distance = idle + N from 2 steps on
         * (latched for this HDG LOCK) */
        if (!i70_fuzzed()) {
            if (!C.uh_lapsed && lease_clearly_gone()) { C.uh_lapsed = true; C.uh_lapsed_t = S.t; }
            if (!C.uh_near && shore_clearly_near(s)) { C.uh_near = true; C.uh_near_t = S.t; }
        }
        if ((C.uh_lapsed && S.t - C.uh_lapsed_t >= 2u * AP_PERIOD_MS) ||
            (C.uh_near && S.t - C.uh_near_t >= 2u * AP_PERIOD_MS)) {
            if (C.uh_lapsed) ST.uh_lapsed++; else ST.uh_near++;
            if (C.uh_lapsed && C.uh_cap > 2.0f && C.uh_gear0 == 1) ST.uh_lapsed_hot++;
            if (s->throttle_demand <= 0.0f && s->gear_demand == GEAR_N) ok(70);
            else fail(70, "UNCREWED HDG LOCK with the navreq lease gone / the shore 40 m away: throttle demand %.2f %%, "
                      "gear demand %.0f", s->throttle_demand, (double)s->gear_demand);
        }
        /* GPS not trusted: dead reckoning at reduced power; past the DR limit: STANDBY (2 steps allowed) */
        if (!sb_gps_ok(s)) {
            C.uh_dr_n++;
            bool lim = s->dr.unc_m > s->cfg.dr_max_unc_m ||
                       dr_dr_time_s(&s->dr, s->now_ms) * 1000.0f > (float)s->cfg.dr_max_ms;
            C.uh_drlim_n = lim ? C.uh_drlim_n + 1 : 0;
            if (C.uh_dr_n >= 2) {
                ST.uh_dr++;
                if (C.uh_cap > 2.0f && C.uh_gear0 == 1) ST.uh_dr_hot++;
                if (s->throttle_demand <= C.uh_cap * s->cfg.dr_speed_factor + 0.01f) ok(69);
                else fail(69, "UNCREWED HDG LOCK on dead reckoning: throttle demand %.2f %% above %.2f %%",
                          s->throttle_demand, C.uh_cap * s->cfg.dr_speed_factor);
            }
            if (C.uh_drlim_n >= 3) {
                fail(69, "UNCREWED HDG LOCK still running %.0f steps past the DR limit (unc %.0f m)",
                     (double)C.uh_drlim_n, s->dr.unc_m);
            }
        } else {
            C.uh_dr_n = 0;
            C.uh_drlim_n = 0;
        }
    }
    if (m == MODE_STANDBY && C.mode_prev == MODE_HDG_LOCK) {
        const mode_log_entry_t *le = mode_log_get(&s->modes, mode_log_size(&s->modes) - 1u);
        if (le && le->why == WHY_GEOFENCE) { ok(68); ST.uh_fence++; }
        if (le && (le->why == WHY_DR_LIMIT || le->why == WHY_GPS_LOST) && C.uh_session) { ok(69); ST.uh_drlim++; }
    }

    /* I22: UNCREWED - the onboard joystick has no effect */
    if (s->profile == SB_PROFILE_UNCREWED) {
        if (m == MODE_MANUAL || (m == MODE_HDG_LOCK && !C.uh_session))
            fail(22, "UNCREWED but mode %.0f (onboard control)%.0f", (double)m, 0.0);
        else ok(22);
        if (conf_now) {
            if (C.sconf_t && S.t - C.sconf_t <= 300u) ok(22);
            else fail(22, "UNCREWED: a CONFIRM accepted without the shore CONFIRM (last %.0f ms ago)%.0f",
                      C.sconf_t ? (double)(S.t - C.sconf_t) : -1.0, 0.0);
        }
    }

    /* I23: the profile changes only in STANDBY with the boat stopped */
    if (s->profile != C.prof_prev) {
        if (m == MODE_STANDBY && C.mode_prev == MODE_STANDBY && !s->stopping) ok(23);
        else fail(23, "profile changed to %.0f in mode %.0f", (double)s->profile, (double)m);
    }
    if (C.prof_prev == SB_PROFILE_CREWED && s->profile == SB_PROFILE_UNCREWED)
        fail(23, "profile CREWED -> UNCREWED without a power-up%.0f%.0f", 0.0, 0.0);

    /* I27-I31: UNCREWED lost link + ENGINE KILL */
    /* I29's storm view of an OD-3 cause: kept while the injected fault runs, the AURIX still shows the
     * throttle node lost (its latch) or the node-lost time has not passed yet; then forgotten */
    if (C.thr_fault_t && !S.can_back_t && !s->can.node[HAL_CAN_ROLE_THROTTLE].lost &&
        S.t - C.thr_fault_t > HAL_CAN_NODE_LOST_MS + 500u)
        C.thr_fault_t = 0u;
    {
        bool kill = sim_hal_engine_killed();
        bool safe_now = forced || (thr <= 0.6f && (gp == 0 || gp == -1));
        if (safe_now) { if (!C.safe_since) C.safe_since = S.t ? S.t : 1u; } else C.safe_since = 0u;
        if (kill && !C.killed) {
            if (safe_now || s->ll.node_kill) ok(27);       /* OD-3: UNCREWED throttle node lost = at once (I55) */
            else fail(27, "ENGINE KILL with the throttle at %.1f %% / gear %.0f", thr, (double)gp);
            if (s->profile == SB_PROFILE_CREWED) fail(30, "ENGINE KILL started in a CREWED boat%.0f%.0f", 0.0, 0.0);
            else ok(30);
            /* open sea with fresh far shore info (the storm's own view): never before the grace time */
            bool far = S.shore_mode == 2u && S.gw_on && s->gw.state == SB_GW_OK;
            float since = (S.t - s->ll.lost_ms) * 0.001f;
            /* OD-3 kill = at once (I55): only when the AURIX says so (node_kill) AND the storm itself made the
             * throttle node unavailable (its injected fault on that node or a bus-off) - a node_kill without a
             * real cause (a healthy node marked lost, the flag set on the link-loss path) is not excused */
            bool od3 = s->ll.node_kill && C.thr_fault_t != 0u;
            if (far && since < s->cfg.lostlink_grace_s - 0.1f && !od3)
                fail(29, "killed %.1f s after the loss at open sea (grace %.0f s)", since, s->cfg.lostlink_grace_s);
            else ok(29);
        }
        if (C.killed && !kill) {
            /* only the HELM station's RESTART with idle + neutral and the e-stop released (or a reset: boot) */
            if (C.rs_t && S.t - C.rs_t <= 2u * AP_PERIOD_MS && C.rs_ok) ok(31);
            else fail(31, "ENGINE KILL released without a reset or a valid restart (restart %.0f ms ago, valid %.0f)",
                      C.rs_t ? (double)(S.t - C.rs_t) : -1.0, C.rs_ok ? 1.0 : 0.0);
            C.killed = false;
        } else if (C.killed) ok(31);
        if (kill) C.killed = true;
        if (!gear_is(&s->gear, GEAR_N)) C.gear_n_since28 = 0u;
        else if (!C.gear_n_since28) C.gear_n_since28 = S.t ? S.t : 1u;
        if (s->ll.state == SB_LL_LOST && !kill && C.safe_since && S.t - C.safe_since > 1500u && !s->mon_tripped) {
            /* near shore (the storm's view): the Pi said near / nothing / not valid, or it is quiet */
            uint32_t cage = s->coast.rx_ms ? S.t - s->coast.rx_ms : 0xFFFFFFFFu;
            bool fresh = cage < 1500u;
            bool near = s->coast.rx_ms == 0u || cage > s->cfg.shore_max_age_ms + 500u ||
                        (fresh && (S.shore_mode == 1u || S.shore_mode == 3u)) || (S.gw_off_t && S.t - S.gw_off_t > 4500u);
            bool far = S.shore_mode == 2u && S.gw_on && s->gw.state == SB_GW_OK && fresh;
            float since = (S.t - s->ll.lost_ms) * 0.001f;
            /* (step 13q2, seed 709: the AURIX's "neutral confirmed" = the gear machine settled in N - a gear change
             * asked just before (a REMOTE lever) takes its idle + neutral dwell although the pulse never left N) */
            uint32_t allow = 1000u;
            if (!C.gear_n_since28) allow = 1000u + S.cfg.gear.idle_wait_ms + S.cfg.gear.neutral_wait_ms + 200u;
            else if ((int32_t)(C.gear_n_since28 - s->ll.lost_ms) + 200 > (int32_t)allow)
                allow = (uint32_t)((int32_t)(C.gear_n_since28 - s->ll.lost_ms) + 200);
            if (near && !far && S.t - s->ll.lost_ms > allow)
                fail(28, "lost %.1f s ago, near shore / no shore info, idle + neutral - but no kill", since, 0.0);
            else if (near && !far) ok(28);
            if (far && since > s->cfg.lostlink_grace_s + 0.5f)
                fail(29, "lost %.1f s ago at open sea, idle + neutral - no kill after the grace time %.0f s", since,
                     s->cfg.lostlink_grace_s);
            else if (far) ok(29);
        }
    }

    /* I60-I62: LINK TIME (step 13q1) - the storm's own view of the HELM station and the shore */
    {
        bool far_now = S.shore_tx_mode == 2u && S.gw_on && s->gw.state == SB_GW_OK && S.shore_tx_t &&
                       S.t - S.shore_tx_t < 1500u;
        if (far_now) { if (!S.far_since) S.far_since = S.t ? S.t : 1u; } else S.far_since = 0u;
        bool helm_changed = s->helm != C.helm_prev61;
        if (helm_changed && s->helm != SB_HELM_NONE) {             /* a grant (also by a fuzzed frame): heard now */
            S.heard_t = S.t;
            S.rep_since_grant = false;
            /* I62: a HELM grant = LINK TIME 3 s (unless a LINK TIME command came in the same step) */
            if (s->ll.lt_sets != C.lt_sets_prev) { }
            else if (s->ll.link_time_s == SB_LINK_TIME_MIN_S) ok(62);
            else fail(62, "HELM granted to station %.0f, LINK TIME still %.0f s", (double)s->helm, (double)s->ll.link_time_s);
        }
        if (s->helm >= 2u && S.gw_on && !S.helm_silent) S.heard_t = S.t - 500u;
        if (!S.gw_on) S.rep_since_grant = false;      /* the AURIX judges by the report only once one came again */
        uint32_t silent = S.t - S.heard_t;
        bool unc = s->profile == SB_PROFILE_UNCREWED && !s->cfg.first_test;
        bool fuzzed = S.fuzz_link_t && S.t - S.fuzz_link_t < 65000u;      /* a CRC-valid random gateway frame */
        uint8_t lt = s->ll.link_time_s;
        /* I62: the setting in range; never in force near the shore / without fresh shore information */
        if (lt >= SB_LINK_TIME_MIN_S && lt <= SB_LINK_TIME_MAX_S) ok(62);
        else fail(62, "LINK TIME %.0f s on the AURIX (3..60)%.0f", (double)lt, 0.0);
        /* (the push is read by the link after the next control step: 100 ms) */
        bool near_now = S.gw_on && S.shore_tx_t && S.t - S.shore_tx_t < 1500u && S.t - S.shore_tx_t >= 100u &&
                        (S.shore_tx_mode == 1u || S.shore_tx_mode == 3u);
        bool none_now = !S.gw_on ? (S.gw_off_t && S.t - S.gw_off_t > 4500u) :
                        (S.shore_mode == 0u && (!S.shore_tx_t || S.t - S.shore_tx_t > s->cfg.shore_max_age_ms + 1500u));
        if (unc && s->ll.limit_s > SB_LINK_TIME_MIN_S && !fuzzed) {
            if (near_now || none_now) fail(62, "LINK TIME %.0f s in force near the shore / without shore info%.0f",
                                           (double)s->ll.limit_s, 0.0);
            else ok(62);
        }
        if (unc && S.rep_on && S.rep_since_grant && !fuzzed && !forced && s->ll.armed && s->ll.state == SB_LL_OK &&
            C.ll_prev61 == SB_LL_OK && s->helm != SB_HELM_USB) {
            /* I60: no later than the time in force (+ 1.5 s for the 1 Hz status and the step) */
            uint32_t lim = (uint32_t)(lt > SB_LINK_TIME_MIN_S ? lt : SB_LINK_TIME_MIN_S) * 1000u;
            if (silent > lim + 1500u)
                fail(60, "the HELM station silent %.1f s, LINK TIME %.0f s - no lost link", silent * 0.001, lt);
            else ok(60);
            if (silent > SB_LINK_TIME_MIN_S * 1000u + 1500u && (near_now || none_now) && !S.far_since &&
                C.near_since61 && S.t - C.near_since61 > 1500u)
                fail(60, "near the shore: the HELM station silent %.1f s - no lost link after 3 s%.0f", silent * 0.001, 0.0);
        }
        if (near_now || none_now) { if (!C.near_since61) C.near_since61 = S.t ? S.t : 1u; } else C.near_since61 = 0u;
        if (unc && s->ll.state == SB_LL_LOST && C.ll_prev61 == SB_LL_OK) {
            /* I61: a new loss at open sea with the gateway's report: not before the captain's LINK TIME */
            bool open = S.far_since && S.t - S.far_since > (uint32_t)lt * 1000u + 3000u;
            if (S.rep_on && S.rep_since_grant && lt > SB_LINK_TIME_MIN_S && open && S.gw_on && !S.gw_off_t &&
                !s->ll.gave_up && !fuzzed && s->ll.lt_sets == C.lt_sets_prev) {
                if (silent + 1500u < (uint32_t)lt * 1000u)
                    fail(61, "lost link at open sea after %.1f s of silence, LINK TIME %.0f s", silent * 0.001, lt);
                else ok(61);
            }
        }
        C.ll_prev61 = s->ll.state;
        C.helm_prev61 = s->helm;
        C.lt_sets_prev = s->ll.lt_sets;
    }

    /* I32-I35: generator mode (CLAUDE.md K) - the one automatic throttle rise, in neutral only */
    if (!forced && !inj) {
        bool nsw = false;
        bool fitted = hal_read_neutral_switch(&nsw);
        const gen_t *g = &s->gen;
        if (g->active) {
            /* I32: gear at N and the switch confirming it (STOPPING: the switch may just have opened) */
            /* DroneCAN: gear HOLD (no gear frame, OA-2 / node loss) of a servo standing at N counts as N */
            bool gear_n = gp == 0 || (gp == -1 && can_ep() &&
                                      fabsf(sim_hal_gear_truth_us() - (float)S.cfg.gear.pwm_n_us) < 60.0f);
            if (gear_n && (g->state == GEN_STOPPING || (fitted && nsw))) ok(32);
            else fail(32, "generator active with the gear at %.0f / switch %.0f (-1 none)", (double)gp,
                      fitted ? (nsw ? 1.0 : 0.0) : -1.0);
            /* I33: under min(cap, power limit), or coming down to it at the closing rate */
            float cap = fminf(g->cap_pct, s->plim.fwd_pct);
            C.gen_env = fmaxf(cap, C.gen_env - down * dt);
            if (thr <= C.gen_env + 1.0f && g->demand_pct <= cap + 0.01f) ok(33);
            else fail(33, "generator throttle %.1f %% above the cap %.1f %%", thr, cap);
            C.gen_env = fmaxf(C.gen_env, fminf(thr, C.gen_env));
            /* I34: the run conditions (a stop that has begun may outlive them) */
            if (g->state == GEN_STOPPING ||
                (((m == MODE_STANDBY && !s->stopping) || (m == MODE_DP && (s->dp.sub == DP_SPOT || s->dp.sub == DP_JOG))) &&
                 s->armed && !s->mon_tripped && !sim_hal_engine_killed() &&
                 s->estop_state != 2u && !s->cfg.first_test && fitted && nsw)) ok(34);
            else fail(34, "generator running in mode %.0f (armed %.0f)", (double)m, s->armed ? 1.0 : 0.0);
        } else {
            /* I32 the other way round: no throttle rise in neutral without the generator */
            if (gp == 0 && thr > 0.6f && thr > C.thr_prev + 0.01f)
                fail(32, "throttle %.1f %% rising in neutral without the generator (before %.1f %%)", thr, C.thr_prev);
            else if (gp == 0) ok(32);
            C.gen_env = 0.0f;
        }
        /* I35: a stop is idle first - the demand is 0 and the output never rises until idle */
        if (g->state == GEN_STOPPING) {
            if (g->demand_pct == 0.0f && thr <= C.thr_prev + 0.01f) ok(35);
            else fail(35, "generator stopping but the throttle rose to %.1f %% (before %.1f %%)", thr, C.thr_prev);
        }
    }

    /* I78 (step 13q5): the MANEUVER gear from the shore stick Y. The storm follows the packets the AURIX took (its
     * seq moved = the newest packet = the storm's stick now): the zone (full forward / back) and since when. A NEW
     * forward / reverse demand needs that zone held >= SB_MNV_HOLD_MS between taken packets (one step slack); a
     * taken packet centred or the other way than the demand = NEUTRAL. */
    {
        const sb_remote_t *rr = &s->remote;
        bool taken = rr->have && rr->seq != C.mv_seq;
        if (taken) {
            /* step 13q5c: the packet the AURIX took (its own stick Y + send time - a delayed packet carries an older Y) */
            uint8_t z = C.mv_sent_y >= SB_MNV_Y_FULL ? 1u : C.mv_sent_y <= -SB_MNV_Y_FULL ? 2u : 0u;
            if (z != C.mv_zone) { C.mv_zone = z; C.mv_zone_t = C.mv_sent_t; }
            C.mv_pkt_t = C.mv_sent_t;
        }
        C.mv_seq = rr->seq;
        if (m == MODE_MANEUVER && C.mode_prev == MODE_MANEUVER && rr->state == SB_REMOTE_DRIVING && rr->have &&
            !forced && !inj) {
            int dem = (int)s->gear_demand;
            if ((dem == GEAR_F || dem == GEAR_R) && dem != C.mv_dem_prev) {
                uint8_t want = dem == GEAR_F ? 1u : 2u;
                ST.mv_asks++;
                if (C.mv_zone == want && (int32_t)(C.mv_pkt_t - C.mv_zone_t) + (int32_t)AP_PERIOD_MS >= (int32_t)SB_MNV_HOLD_MS) ok(78);
                else fail(78, "MANEUVER: a gear (1 F, 2 R: %.0f) asked before the stick Y was held full that way for "
                          "0.3 s (held %.0f ms)", (double)want,
                          C.mv_zone == want ? (double)(int32_t)(C.mv_pkt_t - C.mv_zone_t) : -1.0);
            }
            float ty = C.mv_sent_y;
            if (taken && (fabsf(ty) <= 0.14f || (dem == GEAR_F && ty < 0.0f) || (dem == GEAR_R && ty > 0.0f))) {
                ST.mv_n_checks++;
                if (dem == GEAR_N) ok(78);
                else fail(78, "MANEUVER: the stick Y %.2f (centre / the other way) but gear %.0f asked", ty, (double)dem);
            }
        }
        C.mv_dem_prev = (int)s->gear_demand;
    }

    /* I79 (step 13q5c, review LOW-1 / LOW-2): an UNCREWED HDG LOCK entered from MANEUVER holds gear N - never a
     * FORWARD demand for the whole of it (the throttle was idle: nothing to hold, the lever never opens a gear) */
    if (sb_shore_hdg(s) && C.uh_from_mnv && !forced && !inj) {
        if (s->shdg.hold_gear == GEAR_N && s->gear_demand != GEAR_F) ok(79);
        else fail(79, "UNCREWED HDG LOCK from MANEUVER: held gear %.0f, gear demand %.0f (N = 0, F = 1)",
                  (double)s->shdg.hold_gear, (double)s->gear_demand);
    }
    if (!sb_shore_hdg(s)) C.uh_from_mnv = false;

    /* I24 / I25: MANEUVER - throttle always idle, gear changes only at idle */
    if (m == MODE_MANEUVER && !forced && !inj) {
        if (s->throttle_mode <= 0.01f && thr <= fmaxf(C.thr_prev, 0.6f) + 0.01f) ok(24);
        else fail(24, "MANEUVER: throttle asked %.1f %%, output %.1f %%", s->throttle_mode, thr);
        if (gp >= 0 && C.gear_prev >= 0 && gp != C.gear_prev && gp != 3 && C.gear_prev != 3) {
            if (thr <= 0.6f && C.thr_prev <= 0.6f) ok(25);
            else fail(25, "MANEUVER: gear moved with the throttle at %.1f %% (before %.1f %%)", thr, C.thr_prev);
        } else if (gp >= 0) {
            ok(25);
        }
    }

    /* I8: a CONFIRM with nothing pending */
    if (C.confirm_free_t && C.last_action_t > C.confirm_free_t) C.confirm_free_t = 0u;
    if (C.confirm_free_t && S.t - C.confirm_free_t >= 200u) {
        if (m == C.confirm_free_mode || m == MODE_STANDBY) ok(8);
        else fail(8, "CONFIRM alone changed the mode %.0f -> %.0f", (double)C.confirm_free_mode, (double)m);
        C.confirm_free_t = 0u;
    }

    /* I9: STANDBY by a fault / the button: idle + neutral commanded at once */
    if (m == MODE_STANDBY && C.mode_prev != MODE_STANDBY) {
        if (s->throttle_mode <= 0.01f && s->gear_demand == GEAR_N) ok(9);
        else fail(9, "STANDBY with throttle asked %.1f %% / gear %.0f", s->throttle_mode, (double)s->gear_demand);
    }
    if (s->stopping) {
        if (s->throttle_mode <= 0.01f) ok(9);
        else fail(9, "safe stop running but throttle asked %.1f %%%.0f", s->throttle_mode, 0.0);
    }

    /* I12: remote data stops / gets old */
    if (m != MODE_REMOTE) { C.rem_check = false; C.rem_lost_session = false; }
    if (m == MODE_REMOTE && C.mode_prev != MODE_REMOTE) {
        C.rem_last_fresh = S.t;
        if (!S.rem_stream || S.rem_stale) C.rem_check = true;     /* entered without fresh data */
    }
    if (m == MODE_REMOTE && s->remote.state == SB_REMOTE_LOST) C.rem_lost_session = true;
    if (C.rem_check && m == MODE_REMOTE) {
        float el = (S.t - C.rem_last_fresh) * 0.001f;
        if (s->remote.state == SB_REMOTE_LOST) {
            if (s->throttle_mode <= 0.01f && s->gear_demand == GEAR_N) {
                ok(12);
                worst(12, el);
                C.rem_check = false;
            } else if (el > 0.6f) {
                fail(12, "remote LOST %.2f s ago but throttle asked %.1f %%", el, s->throttle_mode);
                C.rem_check = false;
            }
        } else if (el > 0.5f) {
            fail(12, "remote data %.2f s old, state %.0f (not LOST)", el, (double)s->remote.state);
            C.rem_check = false;
        }
    }
    if (m == MODE_REMOTE && C.rem_lost_session) {
        if (s->remote.state == SB_REMOTE_LOST) ok(12);
        else fail(12, "remote LOST, then driving again by itself (state %.0f, mode %.0f)", (double)s->remote.state,
                  (double)m);
    }

    /* I13 / I20: never more throttle than the mode asks (slamming, caps, limits only reduce) */
    if (!forced && !inj) {
        float ask = fabsf(s->throttle_mode), out = fabsf(s->throttle_demand);
        if (out <= ask + 0.5f || (ask == 0.0f && out <= 0.6f)) ok(13);
        else fail(13, "throttle demand %.1f %% above what the mode asked (%.1f %%)", out, ask);
        bool changed = fabsf(s->ap.target_heading - C.target_prev) > 0.01f;
        if (changed && m != MODE_AUTO && (m != MODE_HDG_LOCK || s->profile == SB_PROFILE_UNCREWED) &&
            s->nav.kind == SB_NAV_HEADING &&
            fabsf(s->ap.target_heading - s->nav.heading) < 0.01f) {
            fail(20, "navigation heading %.1f applied in mode %.0f", s->nav.heading, (double)m);
        } else ok(20);
        if (m == MODE_DP && C.mode_prev == MODE_DP && s->dp.sub != DP_TRIP && s->nav.kind == SB_NAV_HEADING &&
            fabsf(s->dp.hdg_target - C.dp_tgt_prev) > 0.01f && fabsf(s->dp.hdg_target - s->nav.heading) < 0.01f)
            fail(20, "navigation heading %.1f applied in DP sub-state %.0f (not a TRIP)", s->nav.heading, (double)s->dp.sub);
    }

    /* I19: injected violations / frozen core 0 -> the monitor's safe state */
    if (C.inj_t && forced && !C.inj_seen_forced) {
        float el = (S.t - C.inj_t) * 0.001f;
        C.inj_seen_forced = true;
        worst(19, el);
        if (el <= 0.15f) ok(19);
        else fail(19, "monitor safe state %.2f s after the injected violation%.0f", el, 0.0);
    }
    if (C.inj_t && !C.inj_seen_forced && S.t - C.inj_t > 300u) {
        fail(19, "no safe state %.2f s after the injected violation (kind %.0f)", (S.t - C.inj_t) * 0.001,
             (double)S.inj_kind);
        C.inj_t = 0u;
    }

    /* I36: the control step right after a relay event (table offer, relay command, board fault /
     * back): the mode, the gear demand, the steering engagement and what the mode asks of the
     * throttle are what they were, and no alarm that is not an advisory appeared - unless another
     * event of this step may change them legitimately (then this one is not judged) */
    if (C.relay_t && AT_OR_AFTER(S.t, C.relay_t)) {
        if (C.last_action_t < C.relay_t && !forced && !inj && !S.boot_until) {
            bool mode_same = m == C.relay_mode_at;
            bool gear_same = s->gear_demand == C.relay_gear_demand || m != C.relay_mode_at;
            bool steer_same = s->ap.mode == C.relay_ap_mode;
            float dthr = fabsf(s->throttle_mode - C.relay_thr_mode);
            bool thr_ok = m != MODE_MANUAL ? dthr <= fmaxf(2.0f, s->cfg.throttle.rate_up_pct_s * dt * 2.0f) : true;
            /* alarms: both words. Word 2: everything but the advisories (RELAY BOARD SILENT is one).
             * Word 1: the bits a relay event could only reach through the control path (a monitor
             * trip, a gear fault, the e-stop / profile / takeover / TRAINING alarms); the bits that
             * fire on their own timers or sensors in any step (link / sensor / GPS timeouts, NOT
             * ARMED after a trip, gateway / navigation advisories, LINK LOST / ENGINE KILLED) are
             * not judged - they come from other events of the storm */
            const uint32_t judged1 = SB_ALM_MONITOR | SB_ALM_GEAR | SB_ALM_ESTOP | SB_ALM_PROFILE | SB_ALM_JOYSTICK |
                                     SB_ALM_TRAINING | SB_ALM_OVERRUN;
            uint32_t new_alm = s->alarm_bits & ~C.relay_alm & judged1;
            uint32_t new_alm2 = s->alarm_bits2 & ~C.relay_alm2 & ~SB_ALM2_ADVISORY;
            /* a servo node event runs: CAN NODE LOST is excused only for the injected node's role (any node
             * only for a bus-off) - a relay event knocking out another required node is still caught */
            if (S.can_back_t || C.node_silenced_t) {
                /* only the REQUIRED nodes raise CAN NODE LOST (the relay board node 30 is not one) */
                const uint8_t REQ_ROLES = (uint8_t)((1u << HAL_CAN_ROLE_THROTTLE) | (1u << HAL_CAN_ROLE_STEERING) |
                                                    (1u << HAL_CAN_ROLE_GEAR));
                uint8_t role = S.can_fault_node == DC_NODE_THROTTLE ? HAL_CAN_ROLE_THROTTLE :
                               S.can_fault_node == DC_NODE_STEERING ? HAL_CAN_ROLE_STEERING : HAL_CAN_ROLE_GEAR;
                if (S.can_fault_kind == 3u || (s->can.lost_mask & REQ_ROLES & (uint8_t)~(1u << role)) == 0u)
                    new_alm2 &= ~SB_ALM2_CAN_NODE;
            }
            if (mode_same && gear_same && steer_same && thr_ok && new_alm == 0u && new_alm2 == 0u) ok(36);
            else fail(36, "a relay event changed the driving: what %.0f (1 gear 2 steering 4 throttle 8 alarm; + mode*16), "
                      "new alarm bits %.0f (word 2 x 2^32)",
                      (double)((gear_same ? 0 : 1) | (steer_same ? 0 : 2) | (thr_ok ? 0 : 4) | ((new_alm || new_alm2) ? 8 : 0)) +
                      16.0 * (double)m, (double)new_alm + 4294967296.0 * (double)new_alm2);
            if (new_alm2 & SB_ALM2_CAN_NODE)
                printf("  (I36: CAN lost mask %u, injected node %u kind %u, window %u)\n", (unsigned)s->can.lost_mask,
                       (unsigned)S.can_fault_node, (unsigned)S.can_fault_kind, (unsigned)(S.can_back_t || C.node_silenced_t));
        }
        C.relay_t = 0u;
    }
    /* I38: the AURIX's kill latch and the monitor latch: cleared only by a reset (boot, above) or, for the
     * kill, the HELM station's RESTART under its conditions */
    if (C.ll_prev == SB_LL_KILLED && s->ll.state != SB_LL_KILLED) {
        if (s->eng.kill_clears != C.kc_prev && C.rs_t && S.t - C.rs_t <= 2u * AP_PERIOD_MS && C.rs_ok) ok(38);
        else fail(38, "kill latch cleared without a valid restart (clears %.0f -> %.0f)", (double)C.kc_prev,
                  (double)s->eng.kill_clears);
    } else if (s->ll.state == SB_LL_KILLED) ok(38);
    if (C.forced_prev && !forced && !S.boot_until) fail(38, "monitor latch released without a reset%.0f%.0f", 0.0, 0.0);
    else if (forced) ok(38);
    C.ll_prev = s->ll.state;
    C.forced_prev = forced;
    C.kc_prev = s->eng.kill_clears;
    /* I37: the STARTER coil (relay 2 of the storm's table) never on longer than one hold (5 s + the 300 ms
     * board pulse + the board's latency), never on long after the conditions went (the board-timed pulse
     * ends by itself within 300 ms + latency) */
    {
        bool on = sim_hal_relay_state(2u);
        bool cond = thr <= 0.6f && gp == 0 && !S.estop && crank_link_ok(s);
        if (on && !C.st_on_since) C.st_on_since = S.t ? S.t : 1u;
        if (!on) C.st_on_since = 0u;
        if (cond || !on) C.st_bad_since = 0u;
        else if (!C.st_bad_since) C.st_bad_since = S.t ? S.t : 1u;
        if (on && S.t - C.st_on_since > ENG_CRANK_MAX_MS + ENG_CRANK_PULSE_MS + 300u)
            fail(37, "STARTER on %.0f ms in one go (limit %.0f + pulse)", (double)(S.t - C.st_on_since), (double)ENG_CRANK_MAX_MS);
        else if (on && C.st_bad_since && S.t - C.st_bad_since > ENG_CRANK_PULSE_MS + 300u)
            fail(37, "STARTER on %.0f ms after idle / neutral / e-stop released / HELM link / no kill-trip went (throttle %.1f)",
                 (double)(S.t - C.st_bad_since), thr);
        else if (on) ok(37);
    }
    /* I36 (step 7b): the relay BOARD every step - the interlocked pair (4 / 5) never both on, every
     * board-timed pulse <= 5 s, the engine coils (1 IGNITION, 2 STARTER, 3 SHUTOFF) only ever switched
     * on right after an accepted command / crank pulse (never automatically: no power_up, no restart),
     * no power_up after a reset that is not a power-on (watchdog / software reset), and an advisory
     * never sets the alarm flag */
    if (!S.boot_until) {
        if (sim_hal_relay_state(4u) && sim_hal_relay_state(5u)) fail(36, "TRIM UP and DOWN both on%.0f%.0f", 0.0, 0.0);
        else ok(36);
        if (sim_hal_relay_max_pulse_ms() > RELAY_PULSE_MAX_MS)
            fail(36, "a board-timed pulse of %.0f ms (limit %.0f)", (double)sim_hal_relay_max_pulse_ms(), (double)RELAY_PULSE_MAX_MS);
        if (s->eng.pulses != C.eng_pulses_prev) C.st_pulse_t = S.t ? S.t : 1u;
        C.eng_pulses_prev = s->eng.pulses;
        for (uint8_t n = 1u; n <= 3u; n++) {
            uint32_t e = sim_hal_relay_edges(n);
            if (e != C.edges_prev[n]) {
                uint32_t since = n == 2u ? C.st_pulse_t : C.eng_on_t[n];
                if (since && S.t - since <= 800u) ok(36);
                else fail(36, "engine relay %.0f switched on without a command (%.0f ms after the last one)", (double)n,
                          since ? (double)(S.t - since) : -1.0);
            }
            C.edges_prev[n] = e;
        }
        if (hal_reset_reason(NULL) != HAL_RESET_POWER_ON) {
            if (s->relays.power_ups == 0u) ok(36);
            else fail(36, "relay power_up after a %.0f reset (not a power-on), %.0f relays", (double)hal_reset_reason(NULL),
                      (double)s->relays.power_ups);
        }
        uint32_t new1 = s->alarm_bits & ~C.alm_prev, new2 = s->alarm_bits2 & ~C.alm2_prev;
        if (s->alarm && !C.alarm_flag_prev && new1 == 0u && new2 != 0u && (new2 & ~SB_ALM2_QUIET) == 0u)
            fail(36, "an advisory (word 2 bits %.0f) set the alarm flag%.0f", (double)new2, 0.0);
        else if (new2) ok(36);
    }
    /* I36 W42 (step 13h2, review 3bef4c8 LOW-1; owner 2026-10-29): on the DroneCAN relay board every step, for the
     * trim / extra relays (4..9; the engine coils have their own check above): an ON -> OFF edge only after an
     * accepted OFF command since the last ON command, the end of a pulse (momentary: AURIX- or board-timed), the
     * board's own R-3 timeout or the board switching everything OFF itself (reboot / power loss) - NEVER an
     * automatic OFF of the driver; an OFF -> ON edge only within CR_ON_RETRY_MS (+ the board latency and a step) of
     * an accepted ON command and never after a board reboot / power loss since that command (a cut ON finished
     * late: W42) */
    if (sim_hal_relay_bus_now() == SIM_RELAY_DRONECAN) {
        for (uint8_t n = 4u; n <= 9u; n++) {
            bool on = sim_hal_relay_state(n);
            sim_relay_w42_t w;
            (void)sim_hal_relay_w42(n, &w);
            if (on && !C.w42_prev[n]) {
                C.w42_on_edge[n] = S.t ? S.t : 1u;
                if (!w.on_cmd_ms || S.t - w.on_cmd_ms > CR_ON_RETRY_MS + 400u)
                    fail(36, "W42: relay %.0f came ON %.0f ms after its ON command (no late ON after 10 s)", (double)n,
                         w.on_cmd_ms ? (double)(S.t - w.on_cmd_ms) : -1.0);
                else if (w.board_off_ms && (int32_t)(w.board_off_ms - w.on_cmd_ms) >= 0)
                    fail(36, "W42: relay %.0f came ON after a board reboot / power loss %.0f ms after its command", (double)n,
                         (double)(w.board_off_ms - w.on_cmd_ms));
                else ok(36);
            } else if (!on && C.w42_prev[n]) {
                uint32_t e = C.w42_on_edge[n];
                bool cmd_off = w.off_cmd_ms && (!w.on_cmd_ms || (int32_t)(w.off_cmd_ms - w.on_cmd_ms) >= 0);
                bool board = (w.r3_off_ms && (int32_t)(w.r3_off_ms - e) >= 0) ||
                             (w.board_off_ms && (int32_t)(w.board_off_ms - e) >= 0);
                if (cmd_off || w.last_on_pulse || board) ok(36);
                else fail(36, "W42: relay %.0f went OFF without a command (ON for %.0f ms) - never an automatic OFF",
                          (double)n, (double)(S.t - e));
            }
            C.w42_prev[n] = on;
        }
    }
    /* I39-I43: DP (section P). The thruster outputs every step: 0 outside MODE_DP (so within one step of every
     * fault / STANDBY / e-stop / takeover: the mode left DP in that step), 0 at the contactor while the e-stop is
     * pressed; in DP the outboard idle + neutral, its steering off; never a DP step without a trusted GPS; the
     * caps + the heat budget */
    {
        float tb, ts, hb, hs;
        sim_hal_thrusters(&tb, &ts);
        sim_hal_thrust_truth(&hb, &hs);
        const thruster_t *t = &s->thrust;
        if (m != MODE_DP && (tb != 0.0f || ts != 0.0f || t->active))
            fail(39, "thrusters %.1f / %.1f %% outside MODE_DP", tb, ts);
        else if (S.estop && (hb != 0.0f || hs != 0.0f || tb != 0.0f || ts != 0.0f))
            fail(39, "thrusters %.1f / %.1f %% with the e-stop pressed", tb, ts);
        else ok(39);
        if (m == MODE_DP && !forced && !inj) {
            bool gen = s->gen.active;
            bool idle = gen ? true : (thr <= 0.6f && s->throttle_mode <= 0.01f);
            if (idle && gp == 0 && s->gear_demand == GEAR_N && fabsf(sim_hal_motor_command()) < 0.01f) ok(40);
            else fail(40, "DP with the outboard throttle %.1f %% / gear %.0f (or its steering on)", thr, (double)gp);
            if (sb_gps_ok(s)) ok(42);
            else fail(42, "a DP step without a trusted GPS (DR state %.0f)%.0f", (double)s->dr.state, 0.0);
        }
        if (C.mode_prev == MODE_DP && m == MODE_MANUAL) {
            if (tb == 0.0f && ts == 0.0f && !t->active) ok(41);
            else fail(41, "joystick took over from DP but the thrusters run %.1f / %.1f %%", tb, ts);
        }
        if (C.defl_pending && m == MODE_DP && joy_ok && S.t - C.defl_t > 400u)
            fail(41, "joystick deflected %.2f s ago, still DP (sub %.0f)", (S.t - C.defl_t) * 0.001, (double)s->dp.sub);
        bool over = fabsf(tb) > thruster_cap(t, THR_BOW) + 0.01f || fabsf(ts) > thruster_cap(t, THR_STERN) + 0.01f;
        bool hot = (t->hot[THR_BOW] && tb != 0.0f) || (t->hot[THR_STERN] && ts != 0.0f);
        bool budget = (!thruster_fitted(t, THR_BOW) || t->heat_s[THR_BOW] <= (float)t->cfg.unit[THR_BOW].run_s + 0.01f) &&
                      (!thruster_fitted(t, THR_STERN) || t->heat_s[THR_STERN] <= (float)t->cfg.unit[THR_STERN].run_s + 0.01f);
        if (over || hot || !budget) fail(43, "thrust %.1f / %.1f %% over the cap / with a HOT thruster / beyond the budget", tb, ts);
        else if (tb != 0.0f || ts != 0.0f || m == MODE_DP) ok(43);
        /* I44-I46 (step 11b): the DP ends the control step itself must make */
        if (m == MODE_DP && !forced) {
            const dp_t *dpp = &s->dp;
            bool spot = dpp->sub == DP_SPOT || dpp->sub == DP_JOG;
            if (spot && (dpp->err_m > DP_OFF_STOP_M || dpp->off_ms >= DP_OFF_STOP_MS))
                fail(44, "still DP %.1f m off the spot / off it for %.0f ms", dpp->err_m, (double)dpp->off_ms);
            else ok(44);
            if (dpp->set_kn > s->cfg.dp_max_kn + 0.001f || dpp->trip_kn > s->cfg.dp_max_kn + 0.001f)
                fail(45, "DP set speed %.2f kn / trip %.2f kn above dp_max_kn", dpp->set_kn, dpp->trip_kn);
            else ok(45);
            if (thruster_any_hot(t)) fail(46, "a DP step with a thruster HOT (bow %.0f, stern %.0f)", (double)t->hot[THR_BOW],
                                          (double)t->hot[THR_STERN]);
            else ok(46);
        }
    }
    check_can(s, m, thr, forced, inj, dt);
    C.dp_tgt_prev = s->dp.hdg_target;
    C.alarm_flag_prev = s->alarm;
    C.mode_prev = m;
    C.thr_prev = thr;
    C.gear_prev = gp;
    C.confirms_prev = s->confirm.accepted;
    C.plim_f_prev = s->plim.fwd_pct;
    C.plim_r_prev = s->plim.rev_pct;
    C.target_prev = s->ap.target_heading;
    C.prof_prev = s->profile;
    C.thr_mode_prev = s->throttle_mode;
    C.gear_demand_prev = s->gear_demand;
    C.ap_mode_prev = (uint8_t)s->ap.mode;
    C.alm_prev = s->alarm_bits;
    C.alm2_prev = s->alarm_bits2;
}

/* during a frozen core 0: the monitor must trip by itself */
static void check_frozen(void)
{
    if (C.freeze_t && hal_outputs_forced()) {
        float el = (S.t - C.freeze_t) * 0.001f;
        worst(19, el);
        if (el <= (MON_HB_MS + 60u) * 0.001f) ok(19);
        else fail(19, "core 0 frozen: safe state after %.2f s (limit %.2f s)", el, (MON_HB_MS + 60u) * 0.001);
        C.freeze_t = 0u;
    }
}

/* ------------------------------------------------------------- events -- */
static uint32_t s_er;       /* rng for event details (from the event's own numbers) */

static void apply(const ev_t *e)
{
    sb_t *s = &S.sb;
    switch (e->kind) {
    case E_JOY: case E_BTN: case E_TAKE: case E_AUTOBTN: case E_CONFIRM: case E_PC_MODE: case E_PC_AUTO_START:
    case E_PC_AUTO_RESUME: case E_REMOTE_REQ: case E_REMOTE_RELEASE: case E_RESET: case E_FREEZE: case E_COMPASS:
    case E_JOY_WIRE: case E_GPS: case E_NSW: case E_INJ_THR: case E_INJ_GEAR: case E_FUZZ_USB: case E_FUZZ_GW:
    case E_SCONFIRM: case E_MAN_REQ: case E_JOY_PLUG: case E_HELM_DROP: case E_HELM_BACK: case E_GEN: case E_ENGINE:
    case E_WDT: case E_DP: case E_CAN: case E_PC_RETURN: case E_PC_CONFONLY: case E_LINK: case E_SHDG:
        C.last_action_t = S.t + 1u;
        break;
    default:
        break;
    }
    uint32_t r = (uint32_t)(e->a * 7919 + e->b * 104729 + e->c + 1);
    switch (e->kind) {
    case E_JOY: {
        float tl = s->joy.cfg.center_tol * 0.7f;
        bool was_c = fabsf(S.jx) < tl && fabsf(S.jy) < tl;
        if (e->a < 400) { S.jx = 0.0f; S.jy = 0.0f; }                       /* back to centre */
        else { S.jx = e->f; S.jy = (e->a < 700) ? 0.0f : e->g; }
        joy_apply();
        if (was_c && fabsf(S.jx) >= 0.6f) { C.defl_pending = true; C.defl_t = S.t; }
        break;
    }
    case E_BTN: {
        static const hal_button_t B[] = { HAL_BTN_STANDBY, HAL_BTN_HDG_LOCK, HAL_BTN_PLUS_1, HAL_BTN_MINUS_1,
                                          HAL_BTN_PLUS_10, HAL_BTN_MINUS_10 };
        hal_button_t b = B[e->a < 350 ? 0 : 1 + e->a % 5];
        sim_hal_press(b);
        if (b == HAL_BTN_STANDBY) {
            int g = gear_pos(sim_hal_gear_pwm());
            C.standby_press_t = S.t + 1u;
            C.standby_thr = sim_hal_throttle();
            C.standby_est = s->servo.est_pct;
            /* already idle + neutral (e.g. the CPU1 monitor holds the safe state while core 0
             * is frozen / rebooting): nothing to wait for, reaction time 0 */
            C.standby_safe = C.standby_thr <= 0.6f && (g == 0 || g == -1);
        }
        break;
    }
    case E_TAKE: S.take_held = true; S.btn_release = S.t + 150u; joy_apply(); break;
    case E_AUTOBTN: S.auto_held = true; S.btn_release = S.t + 150u; C.autobtn_t = S.t; joy_apply(); break;
    case E_CONFIRM:
        /* step WB (I63): a CREWED MANEUVER CONFIRM needs the shore stick X + Y centred - the shore captain mostly
         * centres it first (no rng draw: the other draws stay what they were) */
        if (!S.uncrewed && s->confirm.kind == CONF_MANEUVER && (S.t / AP_PERIOD_MS) % 10u < 7u) {
            S.rx_x = 0.0f;
            S.rx_y = 0.0f;
            S.rx_y_until = 0u;
            S.rem_stream = true;                          /* (WB review LOW-2: the 30 Hz stream runs: one fresh */
            S.rem_stale = false;                          /*  packet before the step that takes the press) */
        }
        if (s->confirm.kind == CONF_NONE && fabsf(S.jx) < 0.05f && fabsf(S.jy) < 0.05f) {
            C.confirm_free_t = S.t + 1u;
            C.confirm_free_mode = sb_mode(s);
        }
        S.confirm_held = true; S.confirm_release = S.t + 120u; joy_apply();
        break;
    case E_PC_MODE: (void)sb_request_mode(s, (sb_mode_t)(e->a % MODE_COUNT), WHY_USER); break;
    case E_PC_AUTO_START: (void)sb_auto_start(s); break;
    case E_PC_AUTO_RESUME: (void)sb_auto_resume(s); break;
    case E_PC_RETURN: {
        /* step 13q2b (I65): a copilot-free command must never take the captain out of REMOTE / MANEUVER */
        sb_mode_t before = sb_mode(s);
        sb_result_t rr = sb_auto_return(s);
        if (before == MODE_REMOTE || before == MODE_MANEUVER) {
            if (rr != SB_ERR_WRONG_MODE || s->confirm.kind == CONF_AUTO_RETURN || sb_mode(s) != before)
                fail(65, "AUTO RETURN in mode %.0f not refused / left pending (result %.0f)", (double)before,
                     (double)rr + 100.0 * (s->confirm.kind == CONF_AUTO_RETURN));
            else ok(65);
        }
        break;
    }
    case E_PC_CONFONLY: (void)sb_confirm_only(s, s->helm, (uint16_t)e->a); break;
    case E_SLAMRES: {                   /* step 13q4 (I77): the HELM station's answer to the restore question */
        bool acc = e->a < 800;
        sb_result_t rr = sb_slam_restore(s, acc);
        if (acc && rr == SB_OK) { C.slam_acc = true; ST.slam_acc++; }
        else if (acc) ST.slam_refused++;
        else ST.slam_rej++;
        break;
    }
    case E_PC_PAUSE: (void)sb_auto_pause(s); break;
    case E_PC_PROG: prog_random(1 + e->a % 6, &r); break;
    case E_PC_RAISE: {
        uint16_t code = 0;
        uint8_t dir = (uint8_t)(e->a % 2);
        if (sb_power_code(s, dir, &code) == SB_OK) {
            if (e->b < 200) code = (uint16_t)(code + 1u);                   /* typed wrong */
            if (sb_power_raise(s, dir, code) == SB_OK) C.raise_req_t = S.t;
        }
        break;
    }
    case E_PC_LOWER: sb_power_lower(s); break;
    case E_REMOTE_REQ:
    case E_MAN_REQ: {
        if (!S.uncrewed) S.rem_station = (uint8_t)(2 + e->a % 3);
        else if (e->c == 77) S.rem_station = (uint8_t)(2 + (S.rem_station - 1u) % 3u);   /* 13q3b: a swap (script) */
        st_helm(s, S.rem_station);                        /* the shore station took HELM first (gateway) */
        /* step 13q2: UNCREWED the captain mostly centres the shore stick + lever first (the 30 Hz stream: one
           packet just before the request); otherwise whatever the stick is doing (must be refused) */
        if (S.uncrewed && e->b < 650) {
            S.rx_x = 0.0f;
            S.rx_y = 0.0f;
            S.rx_y_until = 0u;
            S.rx_lever = 0.5f;
            S.rem_stream = true;
            S.rem_stale = false;
            shore_packet(s);
        }
        sb_mode_t before = sb_mode(s);
        sb_result_t rr = e->kind == E_REMOTE_REQ ? sb_remote_request(s, S.rem_station) : sb_maneuver_request(s, S.rem_station);
        if (e->kind == E_REMOTE_REQ) ST.rem_req[(int)rr & 15]++;
        sb_mode_t after = sb_mode(s);
        if (after != before) {
            /* I63: only UNCREWED, only the HELM station, its last packet fresh with the stick centred + lever neutral */
            bool centred = C.sent_t && C.sent_fresh && S.t - C.sent_t <= 300u && fabsf(C.sent_x) <= 0.10f &&
                           fabsf(C.sent_y) <= 0.10f && C.sent_lever >= 0.45f && C.sent_lever <= 0.55f;
            bool right = s->profile == SB_PROFILE_UNCREWED && s->helm == S.rem_station && rr == SB_OK &&
                         (after == MODE_REMOTE || after == MODE_MANEUVER) && s->confirm.kind == CONF_NONE;
            C.steered_since_boot = true;                  /* (I56: a mode that steers, entered between two steps) */
            if (centred && right) { ok(63); C.rem_direct_t = S.t ? S.t : 1u; }
            else fail(63, "REMOTE / MANEUVER started at once (mode %.0f) with the shore stick / lever not centred, "
                      "not fresh, CREWED or not HELM (x %.2f)", (double)after, (double)C.sent_x);
        } else if (s->profile == SB_PROFILE_CREWED) {
            ok(63);                                       /* CREWED: never at once */
        }
        break;
    }
    case E_SCONFIRM:
        if (e->a < 700) { S.rx_x = 0.0f; S.rx_y = 0.0f; S.rx_y_until = 0u; S.rx_lever = 0.5f; }   /* centres first (mostly) */
        if (s->confirm.kind == CONF_NONE && S.uncrewed) {
            C.confirm_free_t = S.t + 1u;
            C.confirm_free_mode = sb_mode(s);
        }
        S.sconf_held = true;
        S.sconf_release = S.t + 150u;
        C.sconf_t = S.t;
        break;
    case E_JOY_PLUG: S.joy_fitted = !S.joy_fitted; joy_apply(); break;
    case E_HELM_DROP: st_helm(s, SB_HELM_NONE); S.rem_stream = false; break;  /* gateway dropped it */
    case E_HELM_BACK:
        if (!S.uncrewed) S.rem_station = (uint8_t)(2 + e->a % 3);
        else if (e->c == 77) S.rem_station = (uint8_t)(2 + (S.rem_station - 1u) % 3u);   /* 13q3b: a swap (script) */
        st_helm(s, S.rem_station);
        S.rem_stream = e->b < 700;
        break;
    case E_LINK: {
        /* step 13q1: LINK TIME. The command path (link.c): only a gateway station holding HELM */
        int k = e->a % 5;
        if (k <= 1) {
            uint8_t v = (uint8_t)(1 + e->b % 70);                         /* 1..70: some out of range */
            if (s->helm >= 2u) (void)sb_link_time_set(s, v);
        } else if (k == 2) {
            if (!S.helm_silent) { S.helm_silent = true; S.silent_t = S.t; }   /* its internet is gone */
            S.rem_stream = false;
        } else if (k == 3) {
            S.helm_silent = false;                                         /* heard again */
            S.rem_stream = e->c < 700;
        } else {
            S.shore_mode = 2u;                                             /* open sea + a LINK TIME 10..60 s */
            if (s->helm >= 2u) (void)sb_link_time_set(s, (uint8_t)(10 + e->b % 51));
        }
        break;
    }
    case E_SHORE: S.shore_mode = (uint8_t)(e->a % 4); break;
    case E_REMOTE_ON: S.rem_stream = true; S.rem_stale = false; S.rx_x = e->f; S.rx_lever = 0.5f + 0.5f * e->g; break;
    case E_REMOTE_OFF:
        if (S.rem_stream && sb_mode(s) == MODE_REMOTE && s->remote.state == 1u) C.rem_check = true;
        S.rem_stream = false;
        break;
    case E_REMOTE_STALE:
        if (S.rem_stream && sb_mode(s) == MODE_REMOTE && s->remote.state == 1u) C.rem_check = true;
        S.rem_stale = true;
        break;
    case E_SHDG: {
        /* step 13q2c (I66): the HELM captain's HDG LOCK - mostly with the stick centred, the lever where it was (or
         * forward: then the held throttle is what REMOTE drove) */
        if (e->b < 700) { S.rx_x = 0.0f; S.rx_y = 0.0f; S.rx_y_until = 0u; }
        if (e->c < 500) S.rx_lever = 0.5f + 0.5f * fabsf(e->g);
        if (S.rem_stream) {
            shore_packet(s);
            if (sb_mode(s) == MODE_REMOTE || sb_mode(s) == MODE_MANEUVER || sb_shore_hdg(s) || sb_shore_auto(s)) {
                /* step 13q5c (I79): from MANEUVER some commands come after a stick-Y station packet (no lever: the
                 * stick Y must still be centred - it is the gear there) */
                bool stick_y = sb_mode(s) == MODE_MANEUVER && (e->c % 2) == 1;
                if (stick_y) ST.uh_mnv_stick_y++;
                uint16_t age = S.rem_stale ? 900u : 40u;
                sb_remote_input(s, S.rem_station, ++S.rem_seq, age, 30u, S.rx_x, S.rx_y, S.rx_lever, !stick_y);
                C.mv_sent_y = S.rx_y;
                C.mv_sent_t = S.t - age;
                if (!S.rem_stale) C.rem_last_fresh = S.t;
            }
        }
        sb_mode_t before = sb_mode(s);
        float thr_before = fmaxf(sim_hal_throttle(), s->thr.out_pct);
        int gp_before = gear_pos(sim_hal_gear_pwm());
        bool bad_before = shore_clearly_bad(s) || !sb_gps_ok(s);      /* I69 (13q2d) */
        bool bad70 = !i70_fuzzed() && (lease_clearly_gone() || shore_clearly_near(s));   /* I70 (13q2e) */
        sb_result_t rr = sb_hdg_lock_shore(s, s->helm);
        C.uh_entry_t = S.t ? S.t : 1u;
        C.uh_entry_bad = bad_before;
        C.uh_entry_bad70 = bad70;
        if (sb_mode(s) != before) {
            bool fresh = C.sent_t && C.sent_fresh && S.t - C.sent_t <= 300u && fabsf(C.sent_x) <= 0.10f &&
                         fabsf(C.sent_y) <= 0.10f && C.rem_last_fresh && S.t - C.rem_last_fresh <= 300u;
            bool right = s->profile == SB_PROFILE_UNCREWED && (before == MODE_REMOTE || before == MODE_MANEUVER) &&
                         rr == SB_OK && sb_mode(s) == MODE_HDG_LOCK && s->helm == S.rem_station;
            C.uh_cmd_t = S.t ? S.t : 1u;
            C.uh_cmd_ok = fresh && right;
            C.uh_cap = fminf(thr_before, s->cfg.remote_cap_fwd_pct);
            C.uh_thr0 = thr_before;
            C.uh_gear0 = gp_before;
            C.steered_since_boot = true;                  /* (I56: a mode that steers, entered between two steps) */
            ST.uh_entries++;
            C.uh_from_mnv = before == MODE_MANEUVER && sb_shore_hdg(s);
            if (C.uh_from_mnv) {                          /* I79 (13q5c): the stick Y centred at the entry */
                ST.uh_from_mnv++;
                if (fabsf(s->remote.y) < s->joy.cfg.center_tol && s->shdg.hold_gear == GEAR_N) ok(79);
                else fail(79, "UNCREWED HDG LOCK entered from MANEUVER with the stick Y %.2f (lever 1 / stick-Y station 0: "
                          "%.0f)", s->remote.y, s->remote.has_lever ? 1.0 : 0.0);
            }
        } else {
            ST.uh_refused++;
            ST.uh_ref_code[(unsigned)rr & 31u]++;
        }
        if (s->profile != SB_PROFILE_UNCREWED) {          /* CREWED: refused, nothing changes (the panel button only) */
            if (rr == SB_ERR_NOT_ALLOWED && sb_mode(s) == before) ok(66);
            else fail(66, "CREWED: the shore HDG LOCK command was not refused (result %.0f, mode %.0f)", (double)rr,
                      (double)sb_mode(s));
        }
        break;
    }
    case E_REMOTE_RELEASE: {
        /* step 13q2 (I64): there is no release - it never ends REMOTE / MANEUVER, never changes the mode */
        sb_mode_t before = sb_mode(s);
        sb_result_t rr = sb_remote_release(s, S.rem_station);
        ST.release_tries++;
        if (sb_mode(s) != before) fail(64, "a release changed the mode %.0f -> %.0f", (double)before, (double)sb_mode(s));
        else if ((before == MODE_REMOTE || before == MODE_MANEUVER) && rr != SB_ERR_WRONG_MODE)
            fail(64, "a release in mode %.0f was not refused (%.0f)", (double)before, (double)rr);
        else ok(64);
        break;
    }
    case E_COMPASS: sim_hal_fail_heading(e->a < 500); break;
    case E_GYRO: sim_hal_set_gyro(e->a >= 300, 0.05f, 0.02f); break;
    case E_GPS: S.w.gps_fail = e->a < 500; break;
    case E_GPS_JUMP: S.w.gps_off_n += (e->f * 3000.0); S.w.gps_off_e += (e->g * 3000.0); break;
    case E_NSW: sim_hal_set_neutral_switch(e->a < 600, e->b < 900); break;
    case E_GEN:
        if (e->a < 500) {                                                  /* the captain: ACK, bank low, generator auto on */
            S.w.neutral_switch = true;                                     /* a neutral switch that follows the gearbox
                                                                            * (the world publishes it every tick) */
            S.w.neutral_sw_force = 0;
            sb_alarm_ack(s);
            sim_hal_set_supply(true, 11.5f);
            (void)sb_generator_set(s, true);
        } else {
            (void)sb_generator_set(s, false);
        }
        break;
    case E_SUPPLY: {
        static const float V[4] = { 11.5f, 12.6f, 0.0f, 11.9f };
        sim_hal_set_supply(e->a % 4 != 2, V[e->a % 4]);
        break;
    }
    case E_RELAY: {
        /* the relay board is no control path: whatever happens here, I36 expects the driving unchanged */
        C.relay_t = S.t + 1u;
        C.relay_mode_at = sb_mode(s);
        C.relay_thr_mode = C.thr_mode_prev;
        C.relay_gear_demand = C.gear_demand_prev;
        C.relay_ap_mode = C.ap_mode_prev;
        C.relay_alm = C.alm_prev;
        C.relay_alm2 = C.alm2_prev;
        /* step 13h2 (I36 W42): with a table active, a cut ON - the board goes away, an ON just before it counts as lost (run_until),
         * back after 1..15 s, sometimes rebooted while away */
        if (sim_hal_relay_bus_now() == SIM_RELAY_DRONECAN && !S.w42_n && S.sb.relays.have_table) {
            S.w42_n = (uint8_t)(6u + (uint32_t)e->c % 2u);
            S.w42_t0 = S.t;
            S.w42_cmd_t = 0u;
            S.w42_away = 1000u + ((uint32_t)e->c * 97u) % 14000u;
            S.w42_reboot = e->c % 3 == 0;
            S.w42_rebooted = false;
            /* half: its NodeStatus stops (it still hears: a part of the ON burst arrives); half: its drop cable
             * cut (statuses stop, it hears nothing: none of the ON arrives) */
            S.w42_cable = (e->c / 2) % 2 == 1;
            /* step 13k (W44): the board's first NodeStatus after the silence sometimes waited in its TX mailbox
             * (delivered 0.5-1 s late) and / or is followed by a STALE older one (sim node faults; kept on for the
             * rest of the episode: every later quiet time of the board too) */
            if (e->c % 5 == 1 || e->c % 5 == 2)
                sim_hal_can_node_late(DC_NODE_RELAY, 500u + ((uint32_t)e->c * 37u) % 501u, e->c % 5 == 2);
            else if (e->c % 7 == 3) sim_hal_can_node_late(DC_NODE_RELAY, 0u, true);
            /* step 13k (review c162aad LOW-3): sometimes a young board (rebooted 1.2 s before the silence) */
            S.w42_young_t = 0u;
            if (S.w42_reboot && e->c % 2 == 0) {
                sim_hal_can_node_reboot(DC_NODE_RELAY);
                S.w42_young_t = S.t ? S.t : 1u;
                break;
            }
            if (S.w42_cable) sim_hal_relay_board(false, 5u, 0u);
            else sim_hal_can_node_silent(DC_NODE_RELAY, true);
            break;                                                         /* (this event is only that) */
        }
        if (e->a < 200) {                                                  /* the boat file's table (a CONFIRM follows or not) */
            static relay_table_t t;
            relay_sample_table(&t, e->b % 3 == 0);
            (void)sb_relay_table(s, &t);
        } else if (e->a < 750) {                                           /* a relay command of the HELM station */
            uint8_t n = (uint8_t)(1u + e->b % 9u), act = (uint8_t)(e->c % 3);
            /* FIRST TEST (a shadow relays_t with the same table): every command refused, no power_up */
            {
                static relays_t ft;
                static relay_table_t t;
                relay_sample_table(&t, false);
                relays_init(&ft, &t, true, true);
                bool none = ft.power_up_done && relays_cmd(&ft, n, act, (uint16_t)(e->c * 3), S.t) == RELAY_R_FIRST_TEST &&
                            relays_engine_set(&ft, n, act != RELAY_CMD_OFF) == RELAY_R_FIRST_TEST &&
                            relays_starter_pulse(&ft, 300u, S.t) == RELAY_R_FIRST_TEST && ft.cmd_on == 0u;
                if (none) ok(36);
                else fail(36, "FIRST TEST: relay %.0f action %.0f not refused", (double)n, (double)act);
            }
            bool cond = sim_hal_throttle() <= 0.6f && gear_pos(sim_hal_gear_pwm()) == 0 && !S.estop;
            sb_result_t res = sb_relay_cmd(s, n, act, (uint16_t)(e->c * 3));
            const relay_def_t *rd = relay_table_find(&s->relays.table, n);    /* the ACTIVE table's role, never n */
            uint8_t role = rd ? rd->role : RELAY_ROLE_NONE;
            if ((role == RELAY_ROLE_IGNITION || role == RELAY_ROLE_SHUTOFF) && act != RELAY_CMD_OFF && res == SB_OK) {
                if (n < 4u) C.eng_on_t[n] = S.t ? S.t : 1u;                /* IGNITION / SHUTOFF on: the engine rules */
                if (cond || role == RELAY_ROLE_SHUTOFF) ok(36);             /* SHUTOFF ON: always allowed (11b) */
                else fail(36, "engine relay %.0f switched on with throttle %.1f %% / gear / e-stop", (double)n,
                          sim_hal_throttle());
            }
        } else if (e->a < 900) {                                           /* the board: slow / silent / corrupt replies */
            uint32_t f = (e->c % 3 == 1) ? SIM_RB_SILENT : (e->c % 3 == 2) ? SIM_RB_CORRUPT_SOME : 0u;
            sim_hal_relay_board(true, 5u + (uint32_t)e->b % 200u, f);
        } else {
            sim_hal_relay_board(true, 5u, 0u);                             /* back to normal */
            /* step 13h2 (I36 W42): sometimes the DroneCAN board browns out and reboots (every relay OFF, R-2) -
             * a cut ON must never be finished on it */
            if (sim_hal_relay_bus_now() == SIM_RELAY_DRONECAN && e->b % 4 == 0) sim_hal_can_node_reboot(DC_NODE_RELAY);
        }
        break;
    }
    case E_ENGINE: {
        /* section N: mostly the HELM station, sometimes another one (must be refused) */
        uint8_t st = (e->b % 4 == 0) ? (uint8_t)(2u + (uint32_t)e->c % 3u) : s->helm;
        if (e->a < 350) {                                                  /* RESTART */
            C.rs_t = S.t ? S.t : 1u;
            C.rs_ok = st != 0u && st == s->helm && sim_hal_throttle() <= 0.6f && gear_pos(sim_hal_gear_pwm()) == 0 &&
                      !S.estop;
            if (sb_restart(s, st, ENG_ACT_RESTART) == SB_OK && C.rs_ok) {
                sim_hal_deadman_restart();                                 /* the operator restarts the engine */
                C.trip_t = 0u;                                             /* (I54: judged again from here) */
            }
        } else if (e->a < 850) {                                           /* START held 0.2 .. 8 s */
            S.crank_until = S.t + 200u + (uint32_t)e->c * 8u;
            S.crank_next = S.t;
            S.crank_station = st;
        } else if (e->a < 920) {                                           /* START released */
            S.crank_until = 0u;
            (void)sb_restart(s, st, ENG_ACT_RELEASE);
        } else {                                                           /* the e-stop */
            S.estop = e->b < 500;
            sim_hal_set_estop(true, S.estop);
        }
        break;
    }
    case E_DP: {
        /* section P: the HELM station (CREWED: it takes HELM first, like the remote requests) */
        if (!S.uncrewed) S.rem_station = (uint8_t)(2 + e->c % 3);
        st_helm(s, S.rem_station);
        if (!s->thrust.have_cfg || e->a < 150) {               /* the boat file's thrusters (STANDBY only) */
            thr_cfg_t c;
            memset(&c, 0, sizeof c);
            bool small = e->b % 3 == 0;                         /* sometimes a tiny heat budget (HOT in DP) */
            c.unit[THR_BOW].max_pct = (float)(30 + e->b % 71);
            c.unit[THR_BOW].run_s = small ? 20u : 600u;
            c.unit[THR_BOW].rest_s = small ? 20u : 600u;
            c.unit[THR_STERN].max_pct = (float)(30 + e->c % 71);
            c.unit[THR_STERN].run_s = small ? 15u : 600u;
            c.unit[THR_STERN].rest_s = 30u;
            c.jog_m = 1.5f;
            c.spot_alarm_m = 15.0f;
            (void)sb_thruster_cfg(s, &c, (uint16_t)(1u + e->b));
        }
        S.w.current_e = e->f * 0.4f;                            /* a current (DP must cope or give up) */
        S.w.current_n = e->g * 0.2f;
        if (e->a < 150) break;
        if (e->a < 550) {
            uint8_t sub = (e->b < 500) ? DP_SPOT : (e->b < 750) ? DP_CRUISE
                        : (uint8_t)(DP_TRIP | (e->c < 300 ? DP_TRIP_CONTINUE : 0u));
            if ((sub & 0x7Fu) == DP_TRIP && e->c < 500) prog_random(1 + e->c % 4, &r);
            (void)sb_dp_cmd(s, s->helm, DP_ACT_START, sub, (int16_t)(e->c * 3 % 301));
        } else if (e->a < 700) {
            (void)sb_dp_cmd(s, s->helm, DP_ACT_JOG, (uint8_t)(e->b % 4), 0);
        } else if (e->a < 780) {
            (void)sb_dp_cmd(s, s->helm, DP_ACT_SPEED, 0u, (int16_t)(e->b * 3 % 320));
        } else if (e->a < 880) {
            (void)sb_dp_cmd(s, s->helm, DP_ACT_HEADING, (uint8_t)(e->b % 2),
                            (int16_t)(e->b % 2 ? e->c * 3 - 1500 : e->c * 3));
        } else {
            (void)sb_dp_cmd(s, (e->b % 4 == 0) ? (uint8_t)(2u + (uint32_t)e->c % 3u) : s->helm, DP_ACT_STOP, 0u, 0);
        }
        break;
    }
    case E_CAN: {
        if (!can_ep() || S.can_back_t) break;
        static const uint8_t N[3] = { DC_NODE_THROTTLE, DC_NODE_STEERING, DC_NODE_GEAR };
        uint8_t id = N[e->b % 3];
        S.can_fault_node = id;
        S.can_back_t = S.t + 300u + (uint32_t)e->c * 12u;
        if (e->a < 250) {                                   /* a node loses power: no status, no action (holds) */
            S.can_fault_kind = 1u;
            sim_hal_can_node(id, false, 0u, 0u);
            C.node_silenced_t = S.t ? S.t : 1u;
        } else if (e->a < 400) {                            /* powered + acting, its NodeStatus stops */
            S.can_fault_kind = 2u;
            sim_hal_can_node_silent(id, true);
            C.node_silenced_t = S.t ? S.t : 1u;
        } else if (e->a < 520) {                            /* a brown-out reboot */
            S.can_fault_kind = 0u;
            sim_hal_can_node_reboot(id);
            S.can_back_t = 0u;
        } else if (e->a < 700) {                            /* our controller bus-off */
            S.can_fault_kind = 3u;
            C.thr_fault_t = S.t ? S.t : 1u;                 /* I29: every node, the throttle node too */
            sim_hal_can_fault(SIM_CAN_BUSOFF);
        } else {                                            /* core 0's actuator task stuck (MON_TRIP_CAN_TX) */
            S.can_fault_kind = 4u;
            S.can_back_t = S.t + 400u;
            sim_hal_can_task_stuck(true);
        }
        if (id == DC_NODE_THROTTLE && S.can_fault_kind <= 2u) C.thr_fault_t = S.t ? S.t : 1u;   /* I29 */
        break;
    }
    case E_CURRENT: sim_hal_set_motor_current(e->a < 500, (float)e->b * 0.02f); break;
    case E_LIMITSW: sim_hal_set_limit_switches(e->a < 500, e->b < 100, e->c < 100); break;
    case E_JOY_WIRE: S.joy_broken = e->a < 300; joy_apply(); break;
    case E_ACCEL: sim_hal_set_accel(e->a >= 300); break;
    case E_SEA: S.w.sea_state = (float)(e->a % 6); S.w.slam_extra_g = e->b < 200 ? 2.0f : 0.0f; break;
    case E_SLAMHIT:                     /* step 13q4: one hard wave hit (the boat model's 40 ms pulse) */
        S.slam_ans_t = S.t + 35000u + (uint32_t)e->b * 30u;     /* ... and the captain's answer 35..65 s later */
        S.slam_ans_acc = e->a < 850;
        S.w.slam_left[0] = 3.5f + (float)(e->b % 10) * 0.1f;
        S.w.slam_left[1] = 0.5f * S.w.slam_left[0];
        S.w.slam_left[2] = 0.2f * S.w.slam_left[0];
        break;
    case E_GW: S.gw_on = e->a >= 300; break;
    case E_NAVREQ: {
        uint8_t kind = (uint8_t)(e->a % 4), reason = (uint8_t)(1u << (e->b % 4));
        float cap = (e->c < 300) ? -1.0f : (float)(e->c % 160) * 0.1f;
        if (reason == 8u) kind = (uint8_t)(e->a % 2);                      /* vision: none / cap only */
        uint16_t lease = (uint16_t)(e->b < 50 ? 60000u : 500u + (uint32_t)e->c * 9u);   /* some invalid */
        uint8_t alarm = (uint8_t)(1u << (e->c % 4));
        if (e->a >= 900) alarm |= SB_NAV_ALM_FENCE;                        /* step 13q2c: a geofence breach */
        if (e->a >= 820 && e->a < 900 && cap >= 0.0f && kind != 0u) cap = 0.0f;   /* ... and a STOP */
        navreq(kind, reason, alarm, fabsf(e->f) * 359.0f, cap, lease);
        S.nav_beat_next = S.t + 1000u;                                     /* (13q2e: the heartbeat waits a bit) */
        break;
    }
    case E_MNV_Y:
        /* step 13q5 (I78): the shore stick Y - the MANEUVER gear (elsewhere: a lever station's Y does nothing) */
        S.rx_y_until = 0u;
        if (e->a < 250) S.rx_y = 0.0f;
        else if (e->a < 500) S.rx_y = 1.0f;
        else if (e->a < 700) S.rx_y = -1.0f;
        else if (e->a < 800) S.rx_y = 0.75f * e->f;                        /* part (never "full") */
        else {                                                             /* a flick: let go before 0.3 s */
            S.rx_y = e->f >= 0.0f ? 1.0f : -1.0f;
            S.rx_y_until = S.t + 50u + (uint32_t)(e->b % 200);
        }
        /* step 13q5c (I78): a full deflection / flick with one packet delayed ~200 ms two steps after it began (the
         * packet arrives with its true age; the freshness limits still hold) - no rng draw */
        if ((e->a >= 250 && e->a < 700 && e->b % 4 == 0) || (e->a >= 800 && e->b % 3 != 0)) {
            S.mvd_t0 = S.t + 2u * AP_PERIOD_MS;
            S.mvd_until = S.mvd_t0 + 200u;
            S.mvd_saved = false;
        }
        ST.mv_y++;
        break;
    case E_TRIG:
        /* step 13q3 (I74): the shore stick's TRIGGER - in half of them the stick moved first (steering by hand) */
        if (e->a < 500) S.rx_x = (e->f >= 0.0f ? 0.3f : -0.3f) + 0.6f * e->f;
        S.trig_held = true;
        S.trig_release = S.t + 150u;
        ST.ov_trig++;
        break;
    case E_NAVBEAT: S.nav_beat_off_until = (S.t + 3500u + (uint32_t)(e->a % 8) * 1000u) | 1u; break;
    case E_FUZZ_USB:
    case E_FUZZ_GW: {
        bool gw = e->kind == E_FUZZ_GW;
        uint8_t pl[PROTO_MAX_PAYLOAD];
        uint8_t len = (uint8_t)(e->b % 64);
        for (int i = 0; i < len; i++) pl[i] = (uint8_t)rng_next(&r);
        if (e->a < 300) {                                                  /* garbage bytes */
            uint8_t g[200];
            uint32_t n = 1u + (uint32_t)e->c % 200u;
            for (uint32_t i = 0; i < n; i++) g[i] = (uint8_t)rng_next(&r);
            if (gw) sim_hal_gw_rx_push(g, n); else sim_hal_link_rx_push(g, n);
        } else if (e->a < 600) {                                           /* a valid frame, random type */
            uint8_t ty = (uint8_t)(rng_next(&r) % 0x40u);
            push_frame(gw, ty, pl, len);
            if (gw) S.fuzz_gw_t = S.t ? S.t : 1u;                                                       /* I70 */
            if (gw && ty >= PROTO_T_GW_STATUS && ty <= PROTO_T_GW_STATION) S.fuzz_link_t = S.t ? S.t : 1u;  /* I60-I62 */
            if (e->c < 300) {
                ty = (uint8_t)(rng_next(&r) % 0x40u);
                push_frame(gw, ty, pl, len);                                    /* duplicated */
                if (gw) S.fuzz_gw_t = S.t ? S.t : 1u;
                if (gw && ty >= PROTO_T_GW_STATUS && ty <= PROTO_T_GW_STATION) S.fuzz_link_t = S.t ? S.t : 1u;
            }
        } else {                                                           /* a corrupted frame */
            uint8_t fr[PROTO_MAX_FRAME];
            size_t n = proto_encode(fr, (uint8_t)(rng_next(&r) % 0x40u), 0u, pl, len);
            fr[rng_next(&r) % n] ^= (uint8_t)(1u + rng_next(&r) % 255u);
            if (gw) sim_hal_gw_rx_push(fr, (uint32_t)n); else sim_hal_link_rx_push(fr, (uint32_t)n);
        }
        break;
    }
    case E_FREEZE:
        if (!S.frozen && !S.boot_until) {
            S.frozen = true;
            S.wdt_ms = e->a < 500 ? 100u : 0u;
            S.frozen_until = S.t + 400u + (uint32_t)e->b * 3u;
            C.freeze_t = S.wdt_ms ? 0u : S.t;                 /* without the watchdog: the monitor must trip */
        }
        break;
    case E_RESET:
        sim_hal_chip_reset(HAL_RESET_POWER_ON);
        S.boot_until = (S.t + 200u) ? S.t + 200u : 1u;
        break;
    case E_WDT:                                                            /* a watchdog reset: SYSTEM, never power-on */
        if (!S.boot_until) {
            S.resets++;
            sim_hal_chip_reset(HAL_RESET_SYSTEM);
            S.boot_until = (S.t + 200u) ? S.t + 200u : 1u;
            S.frozen = false;
        }
        break;
    case E_INJ_THR:
    case E_INJ_GEAR:
        if (!hal_outputs_forced() && !S.boot_until && !S.frozen &&
            (e->kind == E_INJ_THR || (sim_hal_throttle() > 5.0f && gear_pos(sim_hal_gear_pwm()) == 1))) {
            S.inj_kind = e->kind == E_INJ_THR ? 1u : 2u;
            S.inj_until = S.t + 300u;
            C.inj_t = S.t;
            C.inj_seen_forced = false;
        }
        break;
    default: break;
    }
    (void)s_er;
}

/* --------------------------------------------------------- the run -- */
static void run_until(uint32_t until)
{
    while (!AT_OR_AFTER(S.t, until)) {
        sim_hal_set_time_ms(S.t);
        world_publish(&S.w, S.t);
        if (S.boot_until && AT_OR_AFTER(S.t, S.boot_until)) {
            S.boot_until = 0u;
            S.frozen = false;
            S.inj_kind = 0u;
            memset((void *)&g_mon, 0, sizeof g_mon);
            sim_hal_unforce();
            boot();
            if (C.node_silenced_t) C.node_silenced_t = S.t ? S.t : 1u;   /* I50: the new boot's node table starts now */
        }
        if (S.frozen && AT_OR_AFTER(S.t, S.frozen_until) && !S.wdt_ms) {      /* no watchdog: it stays frozen until reset */
            S.frozen = false;
            sim_hal_chip_reset(HAL_RESET_POWER_ON);
            S.boot_until = (S.t + 200u) ? S.t + 200u : 1u;
            C.freeze_t = 0u;
        }
        /* step 11e: core 0's 10 ms actuator task (frozen / booting: it does not run) */
        if (!S.boot_until && !S.frozen && S.t % 10u == 0u) hal_can_tick();
        if (S.can_back_t && AT_OR_AFTER(S.t, S.can_back_t)) {
            S.can_back_t = 0u;
            if (S.can_fault_kind == 1u) sim_hal_can_node(S.can_fault_node, true, 0u, 0u);
            if (S.can_fault_kind == 2u) sim_hal_can_node_silent(S.can_fault_node, false);
            if (S.can_fault_kind == 3u) sim_hal_can_fault(0u);
            if (S.can_fault_kind == 4u) sim_hal_can_task_stuck(false);
            C.node_silenced_t = 0u;
        }
        if (S.w42_n && S.w42_young_t) {                 /* step 13k: the young board's silence begins */
            if (S.t - S.w42_young_t >= 1200u) {
                S.w42_young_t = 0u;
                S.w42_t0 = S.t;
                if (S.w42_cable) sim_hal_relay_board(false, 5u, 0u);
                else sim_hal_can_node_silent(DC_NODE_RELAY, true);
            }
        } else if (S.w42_n) {                           /* step 13h2: the cut-ON scenario (I36 W42) */
            hal_can_node_t nn;
            if (!S.w42_cmd_t) {
                bool there = dc_node(HAL_CAN_ROLE_RELAY, &nn) && nn.seen;
                if (there && nn.age_ms <= HAL_RELAY_LOST_MS && nn.age_ms + 80u >= HAL_RELAY_LOST_MS) {
                    (void)sb_relay_cmd(&S.sb, S.w42_n, RELAY_CMD_ON, 0u);
                    S.w42_cmd_t = S.t ? S.t : 1u;
                } else if (!there || nn.age_ms > HAL_RELAY_LOST_MS || S.t - S.w42_t0 > 5000u) {
                    if (S.w42_cable) sim_hal_relay_board(true, 5u, 0u); else sim_hal_can_node_silent(DC_NODE_RELAY, false);
                    S.w42_n = 0u;
                }
            } else {
                if (S.w42_reboot && !S.w42_rebooted && S.t - S.w42_cmd_t >= 300u) {
                    sim_hal_can_node_reboot(DC_NODE_RELAY);
                    S.w42_rebooted = true;
                }
                if (S.t - S.w42_cmd_t >= S.w42_away) {
                    if (S.w42_cable) sim_hal_relay_board(true, 5u, 0u); else sim_hal_can_node_silent(DC_NODE_RELAY, false);
                    S.w42_n = 0u;
                }
            }
        }
        /* UNCREWED: the crew switches the dead-man relay into the kill circuit once the pulses run (OA-3) */
        if (S.uncrewed && can_ep() && !S.dm_armed && sim_hal_deadman_edges() > 2u) {
            S.dm_armed = true;
            sim_hal_deadman_arm(true);
        }
        if (!S.boot_until && AT_OR_AFTER(S.t, S.next_ctrl)) {
            S.next_ctrl += AP_PERIOD_MS;
            if (!S.frozen) {
                /* the hands: release buttons, remote stream */
                if (S.confirm_held && AT_OR_AFTER(S.t, S.confirm_release)) { S.confirm_held = false; joy_apply(); }
                if ((S.take_held || S.auto_held) && AT_OR_AFTER(S.t, S.btn_release)) { S.take_held = S.auto_held = false; joy_apply(); }
                if (S.sconf_held && AT_OR_AFTER(S.t, S.sconf_release)) S.sconf_held = false;
                if (S.trig_held && AT_OR_AFTER(S.t, S.trig_release)) S.trig_held = false;     /* 13q3 */
                if (S.rx_y_until && AT_OR_AFTER(S.t, S.rx_y_until)) { S.rx_y = 0.0f; S.rx_y_until = 0u; }   /* 13q5 */
                if (S.crank_until && AT_OR_AFTER(S.t, S.crank_until)) S.crank_until = 0u;
                if (S.crank_until && AT_OR_AFTER(S.t, S.crank_next)) {
                    S.crank_next = S.t + 200u;
                    uint32_t p0 = S.sb.eng.pulses;
                    bool cond = sim_hal_throttle() <= 0.6f && gear_pos(sim_hal_gear_pwm()) == 0 && !S.estop &&
                                !hal_outputs_forced() && crank_link_ok(&S.sb) && S.crank_station == S.sb.helm;
                    (void)sb_restart(&S.sb, S.crank_station, ENG_ACT_CRANK);
                    if (S.sb.eng.pulses != p0) {
                        if (cond) ok(37);
                        else fail(37, "STARTER pulse with throttle %.1f %% / gear %.0f / e-stop / not HELM / no link / kill / trip", sim_hal_throttle(),
                                  (double)gear_pos(sim_hal_gear_pwm()));
                    }
                }
                if (S.rem_stream) shore_packet(&S.sb);     /* every packet: the shore CONFIRM source */
                if (S.rem_stream && (sb_mode(&S.sb) == MODE_REMOTE || sb_mode(&S.sb) == MODE_MANEUVER ||
                                     sb_shore_hdg(&S.sb) || sb_shore_auto(&S.sb))) {
                    /* (13q2c: the lever ceiling in UNCREWED HDG LOCK; 13q3: the override + ceiling in UNCREWED AUTO) */
                    uint16_t age = S.rem_stale ? 900u : 40u;
                    float py = S.rx_y;
                    bool send = true;
                    if (sb_mode(&S.sb) == MODE_MANEUVER && !S.rem_stale) {
                        /* step 13q5c (I78): the data age jitters 20-139 ms; a held-back packet arrives late with
                         * its true age and the stick Y of its send time (no rng draw) */
                        age = (uint16_t)(20u + ((uint32_t)S.rem_seq * 37u) % 120u);
                        ST.mv_jitter++;
                        if (S.mvd_t0 && AT_OR_AFTER(S.t, S.mvd_t0)) {
                            if (!S.mvd_saved) { S.mvd_saved = true; S.mvd_y = S.rx_y; S.mvd_sent = S.t; }
                            if (!AT_OR_AFTER(S.t, S.mvd_until)) send = false;
                            else {
                                py = S.mvd_y;
                                age = (uint16_t)(S.t - S.mvd_sent);
                                S.mvd_t0 = 0u;
                                ST.mv_delayed++;
                            }
                        }
                    } else {
                        S.mvd_t0 = 0u;                             /* (only in MANEUVER) */
                    }
                    if (send) {
                        sb_remote_input(&S.sb, S.rem_station, ++S.rem_seq, age, 30u, S.rx_x, py, S.rx_lever, true);
                        C.mv_sent_y = py;
                        C.mv_sent_t = S.t - age;
                        if (!S.rem_stale) C.rem_last_fresh = S.t;
                    }
                } else if (sb_mode(&S.sb) == MODE_REMOTE && C.rem_last_fresh == 0u) {
                    C.rem_last_fresh = S.t;
                }
                C.conf_kind_prev = (uint8_t)S.sb.confirm.kind;     /* I58: the request pending before this step */
                C.ov_active_prev = S.sb.sovr.active;              /* I71-I74 (13q3): the override before this step */
                C.ov_cmd_prev = S.sb.sovr.steer_cmd;
                C.ov_now_prev = S.sb.now_ms;
                C.ov_hdg_prev = S.sb.ap.heading;
                C.ov_newhdgs_prev = S.sb.sovr.newhdgs;
                C.ov_starts_prev = S.sb.sovr.starts;
                uint32_t t0 = hal_micros();
                sb_step(&S.sb);
                if (s_log) logcore0_step(&S.sb);
                uint32_t us = hal_micros() - t0;
                bool sw_reset = sim_hal_reset_requested();         /* remote restart after a trip (after the checks) */
                if (us > s_ctrl_max_us) s_ctrl_max_us = us;
                s_ctrl_n++;
                s_ctrl_sum_us += us;
                s_hist[us / 100u < 63u ? us / 100u : 63u]++;
                link_step(&S.usb, &S.sb);
                link_step(&S.gw, &S.sb);
                if (S.inj_kind && !AT_OR_AFTER(S.t, S.inj_until + 1u)) {                   /* a "bug" after the normal step */
                    if (S.inj_kind == 1u) { hal_set_throttle_pwm(1900u); g_mon.throttle_pct = 90.0f; }
                    else { hal_set_gear_pwm(S.cfg.gear.pwm_r_us); g_mon.gear_pwm_us = S.cfg.gear.pwm_r_us; }
                } else if (S.inj_kind && AT_OR_AFTER(S.t, S.inj_until + 3001u)) {
                    sim_hal_chip_reset(HAL_RESET_POWER_ON);               /* the crew power-cycles after a trip */
                    S.boot_until = (S.t + 200u) ? S.t + 200u : 1u;
                    S.inj_kind = 0u;
                    C.inj_t = 0u;
                }
                S.last_kick = S.t;
                check_step();
                if (sw_reset) {                                    /* hal_system_reset(): the chip restarts */
                    sim_hal_chip_reset(HAL_RESET_APPLICATION);
                    S.boot_until = (S.t + 200u) ? S.t + 200u : 1u;
                }
            }
        }
        if (S.frozen && !S.boot_until) check_frozen();
        if (S.wdt_ms && !S.boot_until && S.t - S.last_kick > S.wdt_ms) {
            S.resets++;
            sim_hal_chip_reset(HAL_RESET_SYSTEM);
            S.boot_until = (S.t + 200u) ? S.t + 200u : 1u;
            S.frozen = false;
        }
        if (!S.boot_until) {
            /* I52 (step 13b): core 0's wire count just before the monitor step - the baseline if this step trips */
            bool was = hal_outputs_forced();
            uint32_t f0 = can_ep() ? core0_wire_frames() : 0u;
            monitor_step(&S.mon, &g_mon, S.t);
            if (!was && hal_outputs_forced()) { C.act_frames_pre = f0; C.act_pre_valid = true; }
        }
        if (s_log && !S.boot_until && S.t % 10u == 0u) logger_step(&s_lg, S.t);
        if (S.pc_on && AT_OR_AFTER(S.t, S.pc_next)) {             /* the ground station's heartbeat */
            S.pc_next = S.t + 500u;
            push_frame(false, PROTO_T_HEARTBEAT, NULL, 0u);
        }
        /* the Pi: status once a second, read what the AURIX sends */
        if (S.gw_on && AT_OR_AFTER(S.t, S.gw_next)) {
            uint8_t pl[12] = { 1u, 0u, 1u, 1u };
            S.gw_next = S.t + 1000u;
            if (S.rep_on && S.sb.helm >= 2u) {          /* step 13q1: the HELM station's age (pongs ~0.5 s old) */
                uint32_t age = 500u + (S.helm_silent ? S.t - S.silent_t : 0u);
                pl[10] = S.sb.helm;
                pl[11] = (uint8_t)(age / 250u > 254u ? 254u : age / 250u);
                S.rep_since_grant = true;
            }
            push_frame(true, PROTO_T_GW_STATUS, pl, sizeof pl);
            if (S.shore_mode != 0u) { S.shore_tx_t = S.t ? S.t : 1u; S.shore_tx_mode = S.shore_mode; }
            if (S.shore_mode != 0u) {
                uint16_t m = S.shore_mode == 1u ? 40u : S.shore_mode == 2u ? 900u : 0xFFFFu;
                uint8_t sp[3] = { (uint8_t)(S.shore_mode == 3u ? 0u : (S.shore_mode == 1u ? 3u : 2u)), (uint8_t)m,
                                  (uint8_t)(m >> 8) };
                push_frame(true, PROTO_T_GW_SHORE, sp, 3u);
            }
        }
        /* step 13q2e: the gateway's nav_loop - a "no danger" request at 2 Hz while an UNCREWED boat is in REMOTE /
         * MANEUVER / HDG LOCK (UNCREWED HDG LOCK needs the lease), except while E_NAVBEAT stopped it */
        if (S.gw_on && S.uncrewed && AT_OR_AFTER(S.t, S.nav_beat_next) &&
            (sb_mode(&S.sb) == MODE_REMOTE || sb_mode(&S.sb) == MODE_MANEUVER || sb_mode(&S.sb) == MODE_HDG_LOCK ||
             sb_mode(&S.sb) == MODE_AUTO) &&      /* (13q3b: the real gateway sends it in AUTO too) */
            !(S.nav_beat_off_until && !AT_OR_AFTER(S.t, S.nav_beat_off_until))) {
            S.nav_beat_next = S.t + 500u;
            navreq(SB_NAV_NONE, 0u, 0u, 0.0f, -1.0f, 3000u);
        }
        if (S.slam_ans_t && AT_OR_AFTER(S.t, S.slam_ans_t)) {     /* step 13q4: the captain answers the question */
            S.slam_ans_t = 0u;
            ev_t an;
            memset(&an, 0, sizeof an);
            an.kind = E_SLAMRES;
            an.a = S.slam_ans_acc ? 100 : 900;
            apply(&an);
        }
        if (S.gw_on) S.gw_off_t = 0u;
        else if (!S.gw_off_t) S.gw_off_t = S.t ? S.t : 1u;
        uint8_t buf[512];
        uint32_t n;
        proto_frame_t f;
        while ((n = sim_hal_gw_tx_pop(buf, sizeof buf)) > 0u) {
            proto_rx_push(&S.gw_rx, buf, n);
            while (proto_rx_next(&S.gw_rx, &f)) { }
        }
        while ((n = sim_hal_link_tx_pop(buf, sizeof buf)) > 0u) {
            proto_rx_push(&S.usb_rx, buf, n);
            while (proto_rx_next(&S.usb_rx, &f)) { }
        }
        world_step(&S.w, PHYSICS_DT_MS * 0.001f);
        S.t += PHYSICS_DT_MS;
    }
}

/* one episode; returns true if an invariant failed (C.inv, C.text) */
static bool episode(const sched_t *sc, bool print)
{
    sim_init(sc);
    memset(&C, 0, sizeof C);
    C.boot_seen = 0u;
    for (int i = 0; i < sc->n; i++) {
        if (!sc->on[i]) continue;
        run_until(sc->ev[i].t);
        if (C.failed) break;
        if (print) printf("  [mode %d joy %.2f %.2f armed %d fault %d] ", (int)sb_mode(&S.sb), S.sb.joy.x, S.sb.joy.y,
                          (int)S.sb.joy.armed, (int)S.sb.joy.fault);
        if (print) printf("  %8.1f s  %-28s a=%d b=%d c=%d f=%.2f g=%.2f\n", sc->ev[i].t * 0.001,
                          EV_NAMES[sc->ev[i].kind], sc->ev[i].a, sc->ev[i].b, sc->ev[i].c, sc->ev[i].f, sc->ev[i].g);
        apply(&sc->ev[i]);
    }
    if (!C.failed) run_until(sc->dur_ms);
    return C.failed;
}

/* the shortest event list that still fails the same invariant */
static int shrink(sched_t *sc, int inv)
{
    int changed = 1, kept = 0;
    while (changed) {
        changed = 0;
        for (int i = sc->n - 1; i >= 0; i--) {
            if (!sc->on[i]) continue;
            sc->on[i] = false;
            stats_t save = ST;
            bool f = episode(sc, false) && C.inv == inv;
            ST = save;                                                   /* shrinking doesn't count */
            if (!f) sc->on[i] = true; else changed = 1;
        }
    }
    for (int i = 0; i < sc->n; i++) kept += sc->on[i];
    return kept;
}

static sched_t SC;

static void report(int episodes, double hours)
{
    printf("\n=== FAULT STORM: %d episodes, %.1f simulated hours ===\n", episodes, hours);
    for (int i = 1; i < NINV; i++) {
        if (i >= 14 && i <= 17) continue;
        if (i == 21 || i == 26) continue;
        if (ST.checks[i] == 0u && i >= 27) { printf("N/A  %-96s (not reached in these episodes)\n", INV_NAMES[i]); continue; }
        if (i == 18) continue;
        printf("%-4s %-96s checks %9lu  fails %lu%s", ST.fails[i] ? "FAIL" : "PASS", INV_NAMES[i],
               (unsigned long)ST.checks[i], (unsigned long)ST.fails[i], "");
        if (i == 4 || i == 5 || i == 12 || i == 19)
            printf("  worst %.2f s (seed %lu)", ST.worst[i], (unsigned long)ST.worst_seed[i]);
        printf("\n");
        if (ST.fails[i]) printf("     first failing seed %lu: %s\n", (unsigned long)ST.first_fail_seed[i],
                                ST.first_fail_text[i]);
    }
    for (int i = 1; i < NINV; i++)                      /* for run_storm.py (several processes merged) */
        printf("DATA %d %lu %lu %.3f %lu %lu\n", i, (unsigned long)ST.checks[i], (unsigned long)ST.fails[i], ST.worst[i],
               (unsigned long)ST.first_fail_seed[i], (unsigned long)ST.worst_seed[i]);
    printf("DATA_RUN %d %.3f %lu %lu %lu %lu %lu\n", episodes, hours, (unsigned long)ST.auto_by_confirm,
           (unsigned long)ST.auto_by_button, (unsigned long)ST.remote_entries, (unsigned long)ST.engaged_ignored,
           (unsigned long)ST.engaged_ignored_seed);
    printf("NOTE: HDG LOCK / AUTO / REMOTE entered while the onboard joystick was held and ignored (not centred since "
           "STANDBY): %lu times (first seed %lu)\n", (unsigned long)ST.engaged_ignored,
           (unsigned long)ST.engaged_ignored_seed);
    printf("NOTE: DP TRIP control steps with an AUTO RETURN latched (I59): %lu\n", (unsigned long)ST.dp_trip_ret);
    printf("DATA_I59 %lu\n", (unsigned long)ST.dp_trip_ret);     /* step 13d: run_storm.py fails a full run with 0 */
    printf("AUTO entries: %lu by PC + CONFIRM, %lu by the joystick AUTO button; REMOTE entries %lu "
           "(requests: ok %lu, refused", (unsigned long)ST.auto_by_confirm, (unsigned long)ST.auto_by_button,
           (unsigned long)ST.remote_entries, (unsigned long)ST.rem_req[0]);
    for (int i = 1; i < 16; i++) if (ST.rem_req[i]) printf(" code %d: %lu", i, (unsigned long)ST.rem_req[i]);
    printf(")\n");
    printf("REMOTE / MANEUVER started at once (UNCREWED, step 13q2): %lu; release tries (never end it): %lu\n",
           (unsigned long)ST.remote_direct, (unsigned long)ST.release_tries);
    printf("UNCREWED HDG LOCK (step 13q2c): %lu entries, %lu commands refused, %lu STOPs, %lu geofence STANDBYs, "
           "%lu stale-data steps\n", (unsigned long)ST.uh_entries, (unsigned long)ST.uh_refused,
           (unsigned long)ST.uh_stops, (unsigned long)ST.uh_fence, (unsigned long)ST.uh_stale);
    printf("UNCREWED HDG LOCK no-data / GPS (step 13q2d, I69): %lu no-shore-data steps, %lu dead-reckoning steps, %lu "
           "DR-limit STANDBYs; %lu / %lu of the steps with a held throttle\n", (unsigned long)ST.uh_nodata,
           (unsigned long)ST.uh_dr, (unsigned long)ST.uh_drlim, (unsigned long)ST.uh_nodata_hot,
           (unsigned long)ST.uh_dr_hot);
    printf("UNCREWED HDG LOCK lease / shore (step 13q2e, I70): %lu entries judged, %lu lapsed-lease steps (%lu with a "
           "held throttle), %lu shore-near steps\n", (unsigned long)ST.uh_entry70, (unsigned long)ST.uh_lapsed,
           (unsigned long)ST.uh_lapsed_hot, (unsigned long)ST.uh_near);
    printf("Shore stick override (step 13q3, I71-I74): %lu steps by hand in AUTO, %lu in HDG LOCK, %lu starts, %lu new "
           "headings (%lu trigger presses), %lu stale-data stops, %lu lever-ceiling steps\n", (unsigned long)ST.ov_steps_auto,
           (unsigned long)ST.ov_steps_hdg, (unsigned long)ST.ov_starts, (unsigned long)ST.ov_newhdgs,
           (unsigned long)ST.ov_trig, (unsigned long)ST.ov_lost, (unsigned long)ST.ov_cut);
    printf("Shore stick override protections (step 13q3b, I75 / I76): %lu stopped steps checked, %lu HELM changes while "
           "steering by hand\n", (unsigned long)ST.ov_stop_steps, (unsigned long)ST.ov_helm);
    printf("Slamming power restore (step 13q4, I77): %lu questions, %lu ACCEPTs taken, %lu refused (not calm / "
           "nothing reduced), %lu REJECTs, %lu cap rises checked\n", (unsigned long)ST.slam_asks,
           (unsigned long)ST.slam_acc, (unsigned long)ST.slam_refused, (unsigned long)ST.slam_rej,
           (unsigned long)ST.slam_rises);
    printf("CREWED MANEUVER entries by the CONFIRM (step WB, I63: shore stick X + Y centred): %lu\n",
           (unsigned long)ST.mnv_crewed_conf);
    printf("DATA_MNVC %lu\n", (unsigned long)ST.mnv_crewed_conf);   /* WB review LOW-2: run_storm.py checks it */
    printf("MANEUVER stick gear (step 13q5, I78): %lu stick-Y events, %lu F / R asks checked, %lu centre / other-way "
           "steps checked\n", (unsigned long)ST.mv_y, (unsigned long)ST.mv_asks, (unsigned long)ST.mv_n_checks);
    printf("MANEUVER packet timing (step 13q5c, I78 / I79): %lu jittered packets, %lu delayed packets; %lu UNCREWED HDG "
           "LOCK entries from MANEUVER (%lu commands after a stick-Y packet)\n", (unsigned long)ST.mv_jitter,
           (unsigned long)ST.mv_delayed, (unsigned long)ST.uh_from_mnv, (unsigned long)ST.uh_mnv_stick_y);
    printf("UNCREWED HDG LOCK refused by result code:");
    for (int i = 0; i < 32; i++) if (ST.uh_ref_code[i]) printf(" %d: %lu", i, (unsigned long)ST.uh_ref_code[i]);
    printf("\n");
}

/* 32-bit time wraparound: the clock starts 3 minutes before 2^32 ms */
static int wrap_test(void)
{
    sched_t *sc = &SC;
    gen_schedule(sc, 4242u, 10u * 60000u);
    sim_init(sc);
    memset(&C, 0, sizeof C);
    uint32_t start = 0xFFFFFFFFu - 180000u;
    S.t = start;
    S.next_ctrl = start;
    S.last_kick = start;
    S.gw_next = start;
    sim_hal_set_time_ms(start);
    boot();
    uint32_t overruns0 = g_timing.core0.overruns, steps0 = 0u;
    for (int i = 0; i < sc->n; i++) {
        uint32_t when = start + sc->ev[i].t;
        while ((int32_t)(S.t - when) < 0) {
            run_until(S.t + 10u);
            steps0++;
        }
        apply(&sc->ev[i]);
        if (C.failed) break;
    }
    printf("\n=== TIME WRAPAROUND: started at %lu ms (3 min before 2^32), ran 10 min across the wrap ===\n",
           (unsigned long)start);
    printf("%s invariants across the wrap (%s)\n", C.failed ? "FAIL" : "PASS", C.failed ? C.text : "all held");
    printf("     end time %lu ms (wrapped), control steps %lu, boots %lu, overruns %lu, mode %d\n",
           (unsigned long)S.t, (unsigned long)C.steps, (unsigned long)S.boots,
           (unsigned long)(g_timing.core0.overruns - overruns0), (int)sb_mode(&S.sb));
    (void)steps0;
    return C.failed ? 1 : 0;
}


/* ===================================================== a day at sea (Part 4.2, AURIX part) == */
typedef struct { double t; char what[90], did[150], exp[90]; bool pass; } row_t;
static row_t R[64];
static int nR;
static FILE *s_md;

static void row(const char *what, bool pass, const char *exp, const char *fmt, ...)
{
    va_list ap;
    row_t *r = &R[nR < 63 ? nR++ : 63];
    r->t = S.t * 0.001;
    snprintf(r->what, sizeof r->what, "%s", what);
    snprintf(r->exp, sizeof r->exp, "%s", exp);
    va_start(ap, fmt);
    vsnprintf(r->did, sizeof r->did, fmt, ap);
    va_end(ap);
    r->pass = pass && !C.failed;
    printf("| %7.0f s | %s | %s | %s | %s |\n", r->t, r->what, r->did, r->exp, r->pass ? "PASS" : "FAIL");
    if (s_md) fprintf(s_md, "| %.0f s | %s | %s | %s | %s |\n", r->t, r->what, r->did, r->exp, r->pass ? "PASS" : "FAIL");
}

static void run_s(double s) { run_until(S.t + (uint32_t)(s * 1000.0)); }
static void sync_mode(void) { C.mode_prev = sb_mode(&S.sb); }     /* a mode change by an API call (not a step) */
/* the USB laptop: take HELM, acknowledge the alarms (through the real link parser) */
static void ack_alarms(void)
{
    uint8_t take = 0u;
    S.pc_on = true;
    push_frame(false, PROTO_T_HELM, &take, 1u);
    run_until(S.t + 1200u);
    push_frame(false, PROTO_T_ALARM_ACK, NULL, 0u);
    run_until(S.t + 300u);
}
static void press_btn(hal_button_t b) { sim_hal_press(b); run_until(S.t + 120u); }
static bool do_confirm(void)
{
    uint32_t c0 = S.sb.confirm.accepted;
    S.confirm_held = true; S.confirm_release = S.t + 150u; joy_apply();
    run_until(S.t + 400u);
    return S.sb.confirm.accepted > c0;
}
static bool wait_for_c(bool (*cond)(void), double max_s)
{
    uint32_t end = S.t + (uint32_t)(max_s * 1000.0);
    while (!cond() && AT_OR_AFTER(end, S.t)) run_until(S.t + 100u);
    return cond();
}
static bool c_training_result(void) { return S.sb.train.stage == TS_RESULT; }
static bool c_dr_dead(void) { return S.sb.dr.state == DR_DEAD; }
static bool c_dr_gps(void) { return S.sb.dr.state == DR_GPS; }
static bool c_not_gps(void) { return S.sb.dr.state != DR_GPS; }
static bool c_lost(void) { return S.sb.remote.state == SB_REMOTE_LOST; }
static bool c_standby(void) { return sb_mode(&S.sb) == MODE_STANDBY; }
static bool c_arrived(void) { return (S.sb.alarm_bits & SB_ALM_ARRIVED) != 0u; }

static void wp_at(prog_wp_t *w, double north, double east, float kn, int act)
{
    world_t tmp = S.w;
    double la, lo;
    tmp.north_m = north;
    tmp.east_m = east;
    world_latlon(&tmp, &la, &lo);
    memset(w, 0, sizeof *w);
    w->lat_e7 = (int32_t)lround(la * 1e7);
    w->lon_e7 = (int32_t)lround(lo * 1e7);
    w->speed_kn = kn;
    w->radius_m = 30.0f;
    w->action = (uint8_t)act;
}

static int day_at_sea(void)
{
    sched_t *sc = &SC;
    memset(sc, 0, sizeof *sc);
    sc->seed = 2026u;
    sc->boot_deflected = true;                                  /* the stick is held at power-up */
    sim_init(sc);
    memset(&C, 0, sizeof C);
    s_md = fopen("day_at_sea_A.md", "w");
    if (s_md) fprintf(s_md, "| time | what happened | what the system did | expected | result |\n|---|---|---|---|---|\n");
    printf("| time | what happened | what the system did | expected | result |\n|---|---|---|---|---|\n");
    sb_t *b = &S.sb;
    char note[160];

    /* 1. power-up */
    run_s(3.0);
    row("power-up with the joystick held", !b->armed && sim_hal_throttle() < 0.6f && gear_pos(sim_hal_gear_pwm()) == 0,
        "outputs not armed, idle, neutral", "armed %d, throttle %.1f %%, gear %s", b->armed, sim_hal_throttle(),
        gear_pos(sim_hal_gear_pwm()) == 0 ? "N" : "?");
    S.jx = 0.0f; joy_apply(); run_s(1.5);
    row("joystick centred, neutral confirmed", b->armed && b->plim.fwd_pct == 30.0f && b->plim.rev_pct == 5.0f,
        "armed; power limit 30 / 5", "armed %d, power limit %.0f / %.0f %%", b->armed, b->plim.fwd_pct, b->plim.rev_pct);

    /* 2. TRAINING in calm water */
    S.w.sea_state = 0.0f;
    bool tr = sb_request_mode(b, MODE_TRAINING, WHY_USER);
    sync_mode();
    bool res = tr && wait_for_c(c_training_result, 25.0 * 60.0);
    run_s(5.0);
    bool acc = res && sb_confirm_training(b, true, 0.0f);
    sync_mode();
    row("TRAINING (PC, calm water), result accepted", tr && res && acc && sb_mode(b) == MODE_STANDBY,
        "calibration runs, idle + neutral, accept -> STANDBY", "started %d, result %s (code %d), accepted %d, mode %d",
        tr, res ? "reached" : "NOT reached", (int)b->ap.train.result, acc, (int)sb_mode(b));

    /* 3. HEADING LOCK out of the harbour, throttle on the joystick, +-1 / +-10 */
    press_btn(HAL_BTN_HDG_LOCK);
    S.jy = 0.45f; joy_apply(); run_s(60.0);
    float t0 = b->ap.target_heading, d1, d2, d3, d4;
    press_btn(HAL_BTN_PLUS_1);  d1 = b->ap.target_heading - t0;
    press_btn(HAL_BTN_PLUS_10); d2 = b->ap.target_heading - t0;
    press_btn(HAL_BTN_MINUS_1); d3 = b->ap.target_heading - t0;
    press_btn(HAL_BTN_MINUS_10); d4 = b->ap.target_heading - t0;
    row("HEADING LOCK, joystick throttle, +1 +10 -1 -10", sb_mode(b) == MODE_HDG_LOCK && S.w.speed_kn > 3.0f &&
        fabsf(d1 - 1.0f) < 0.01f && fabsf(d2 - 11.0f) < 0.01f && fabsf(d3 - 10.0f) < 0.01f && fabsf(d4) < 0.01f,
        "mode HDG LOCK, speed > 3 kn, target +1 +11 +10 0", "mode %d, %.1f kn, target %+.0f %+.0f %+.0f %+.0f",
        (int)sb_mode(b), S.w.speed_kn, d1, d2, d3, d4);

    /* 4. power limit 50 then 75 (PC request + code + CONFIRM each step); alarms acknowledged first.
     *    CONFIRM counts only with the stick centred: in HDG LOCK the throttle (stick Y) goes back first */
    S.jy = 0.0f; joy_apply(); run_s(3.0);
    ack_alarms();
    uint16_t code = 0;
    (void)sb_power_code(b, 0u, &code);
    (void)sb_power_raise(b, 0u, code);
    C.raise_req_t = S.t;
    run_s(11.0);                                                /* no CONFIRM */
    float no_conf = b->plim.fwd_pct;
    (void)sb_power_code(b, 0u, &code); (void)sb_power_raise(b, 0u, code); C.raise_req_t = S.t; bool c50 = do_confirm();
    float p50 = b->plim.fwd_pct;
    (void)sb_power_code(b, 0u, &code); (void)sb_power_raise(b, 0u, code); C.raise_req_t = S.t; bool c75 = do_confirm();
    row("power limit raised 30 -> 50 -> 75 %", no_conf == 30.0f && c50 && p50 == 50.0f && c75 && b->plim.fwd_pct == 75.0f,
        "no CONFIRM: stays 30; each step needs CONFIRM", "without CONFIRM %.0f %%; then %.0f %%, %.0f %%", no_conf, p50,
        b->plim.fwd_pct);

    /* 5. AUTO program: 6 waypoints, different leg speeds */
    static program_t pg;
    program_clear(&pg);
    double n0 = S.w.north_m, e0 = S.w.east_m;
    static const double L[6][2] = { {1200, 0}, {1200, 1200}, {2400, 1500}, {2400, 300}, {1200, -600}, {300, -200} };
    static const float KN6[6] = { 8, 12, 16, 10, 14, 6 };
    for (int i = 0; i < 6; i++) wp_at(&pg.wp[i], n0 + L[i][0], e0 + L[i][1], KN6[i], 0);
    pg.n = 6; pg.final_action = PROG_END_STOP;
    program_seal(&pg);
    prog_error_t err; uint8_t bad;
    sb_result_t pr = sb_program_set(b, &pg, &err, &bad);
    S.jy = 0.0f; joy_apply(); run_s(2.0);
    sb_result_t as = sb_auto_start(b);
    bool ca = do_confirm();
    row("AUTO program (6 waypoints, 6-16 kn) started with CONFIRM", pr == SB_OK && as == SB_OK && ca &&
        sb_mode(b) == MODE_AUTO, "program accepted, AUTO after CONFIRM", "program %d, start %d, CONFIRM %d, mode %d",
        (int)pr, (int)as, ca, (int)sb_mode(b));
    run_s(90.0);
    row("AUTO on leg 1", sb_mode(b) == MODE_AUTO && b->autop.state == AUTO_RUN && S.w.speed_kn > 5.0f,
        "running, about the leg speed (limit 75 %)", "mode %d, state %d, %.1f kn (target %.1f kn)", (int)sb_mode(b),
        (int)b->autop.state, S.w.speed_kn, b->cruise.target_kn);

    /* 6. sea state rising to 4 on a fast leg: slamming, step reductions, the power limit stays (STEP) */
    for (int i = 0; i < 6000 && b->cruise.target_kn < 12.0f; i++) run_until(S.t + 100u);
    run_s(30.0);
    S.w.sea_state = 4.0f;
    S.w.slam_scale = getenv("STORM_SLAM") ? (float)atof(getenv("STORM_SLAM")) : 2.6f;
    uint32_t imp0 = b->slam.impacts, asks_before = b->slam.asks;
    float cap_min = 100.0f;
    for (int i = 0; i < 1800; i++) { run_until(S.t + 100u); cap_min = fminf(cap_min, b->slam.cap_pct); }
    row("sea state 4, slamming on the fast leg", b->slam.impacts > imp0 && cap_min < 100.0f && b->plim.fwd_pct == 75.0f,
        "impacts, throttle reduced in steps, power limit kept", "impacts %lu, lowest cap %.0f %%, power limit %.0f %%, "
        "state %d", (unsigned long)(b->slam.impacts - imp0), cap_min, b->plim.fwd_pct, (int)b->slam.state);
    /* 6b. (step 13q4) the sea calms down: the reduction stays until the captain's ACCEPT, then back at the up rate */
    S.w.sea_state = 0.0f;
    float cap_hold = b->slam.cap_pct;
    bool asked6 = false;
    for (int i = 0; i < 1200 && !slam_restore_avail(&b->slam); i++) { run_until(S.t + 100u); asked6 |= b->slam.asking; }
    for (int i = 0; i < 20 && !b->slam.asking; i++) run_until(S.t + 100u);
    asked6 |= b->slam.asking || b->slam.asks > asks_before;   /* (asked already in a calm spell of the sea 4) */
    float cap_calm = b->slam.cap_pct;
    sb_result_t sr = sb_slam_restore(b, true);
    if (sr == SB_OK) C.slam_acc = true;
    run_s(15.0);
    row("calm sea after slamming (13q4)", cap_calm <= cap_hold + 0.01f && asked6 && sr == SB_OK &&
        b->slam.cap_pct >= 100.0f, "reduced until the captain's ACCEPT, asked on screen 1, then back",
        "cap %.0f %% -> %.0f %% in calm water (asked %d), ACCEPT %d, cap %.0f %% 15 s later", cap_hold, cap_calm,
        asked6, (int)sr, b->slam.cap_pct);
    S.w.sea_state = 2.0f;

    /* 7. GPS lost for 4 minutes: dead reckoning, AUTO goes on */
    S.w.gps_fail = true;
    bool dead = wait_for_c(c_dr_dead, 10.0);
    run_s(240.0);
    float unc = b->dr.unc_m;
    bool still = sb_mode(b) == MODE_AUTO;
    S.w.gps_fail = false;
    bool back = wait_for_c(c_dr_gps, 120.0);
    row("GPS lost for 4 min", dead && still && back, "dead reckoning, AUTO continues (slower), GPS trusted again",
        "DR %d, after 4 min mode %d, uncertainty %.0f m; GPS trusted again %d", dead, (int)sb_mode(b), unc, back);

    /* 8. GPS spoof attempt: the position jumps 800 m */
    S.w.gps_off_n += 800.0;
    bool caught = wait_for_c(c_not_gps, 15.0);
    bool alarm_s = (b->alarm_bits & SB_ALM_GPS_SUSPECT) != 0u;
    S.w.gps_off_n -= 800.0;
    bool rec = false;
    for (int k = 0; k < 120 && !rec; k++) {                     /* up to 10 min, traced every 5 s */
        run_s(5.0);
        rec = c_dr_gps();
        if (getenv("STORM_DEBUG"))
            printf("debug spoof +%3d s: dr state %d why %d unc %.0f m, mode %d, sog %.1f, gps fail %d\n", (k + 1) * 5,
                   (int)b->dr.state, (int)b->dr.why, b->dr.unc_m, (int)sb_mode(b), S.w.speed_kn, (int)S.w.gps_fail);
    }
    row("GPS spoof attempt (800 m jump)", caught && alarm_s && rec && sb_mode(b) == MODE_AUTO,
        "rejected, GPS SUSPECT, dead reckoning, recovers", "rejected %d, alarm %d, recovered %d, mode %d", caught, alarm_s,
        rec, (int)sb_mode(b));

    /* 9. compass glitch (0.8 s, below the 2 s timeout) */
    uint32_t a0 = b->alarm_bits;
    sim_hal_fail_heading(true); run_s(0.8); sim_hal_fail_heading(false); run_s(5.0);
    row("compass glitch 0.8 s", sb_mode(b) == MODE_AUTO && !((b->alarm_bits & ~a0) & SB_ALM_COMPASS),
        "no reaction (below the 2 s timeout)", "mode %d, new compass alarm %d", (int)sb_mode(b),
        ((b->alarm_bits & ~a0) & SB_ALM_COMPASS) != 0u);

    /* 10. onboard joystick takeover during AUTO -> MANUAL + SAVE; resume with PC + CONFIRM */
    uint16_t sv0 = b->save.seq;
    S.jx = 0.7f; joy_apply();
    run_s(0.5);
    bool man = sb_mode(b) == MODE_MANUAL;
    bool saved = b->save.seq != sv0 && b->save.trig == SB_SAVE_TAKEOVER;
    S.jx = 0.0f; joy_apply(); run_s(2.0);
    ack_alarms();
    sb_result_t rr = sb_auto_resume(b);
    bool cr = do_confirm();
    row("joystick takeover in AUTO, then resume", man && saved && rr == SB_OK && cr && sb_mode(b) == MODE_AUTO,
        "MANUAL at once + SAVE; resume = PC + CONFIRM", "MANUAL %d, SAVE %d, resume %d + CONFIRM %d -> mode %d", man, saved,
        (int)rr, cr, (int)sb_mode(b));

    /* 11. remote driving (HELM + onboard CONFIRM), a 2 s link dropout */
    S.rem_station = 3u; b->helm = 3u; b->helm_local = false;
    sb_result_t rq = sb_remote_request(b, 3u);
    S.rem_stream = true; S.rem_stale = false; S.rx_x = 0.0f; S.rx_lever = 0.8f;
    bool crm = do_confirm();
    run_s(10.0);
    bool driving = sb_mode(b) == MODE_REMOTE && b->remote.state == SB_REMOTE_DRIVING;
    S.rem_stream = false;
    uint32_t td = S.t;
    bool lost = wait_for_c(c_lost, 1.0);
    float t_lost = (S.t - td) * 0.001f;
    run_s(2.0 - t_lost);
    S.rem_stream = true;                                        /* data back after 2 s */
    run_s(1.0);
    bool no_resume = b->remote.state == SB_REMOTE_LOST && b->throttle_mode <= 0.01f && b->gear_demand == GEAR_N;
    bool sb5 = wait_for_c(c_standby, 6.0);
    row("remote driving, 2 s link dropout", rq == SB_OK && crm && driving && lost && t_lost < 0.6f && no_resume && sb5,
        "LOST within 0.4 s: idle + neutral + gyro hold, never resumes, STANDBY after 5 s",
        "request %d, CONFIRM %d, driving %d; LOST after %.2f s, data back -> still LOST/idle %d, STANDBY %d", (int)rq, crm,
        driving, t_lost, no_resume, sb5);
    S.rem_stream = false;

    /* 12. back to AUTO, arrival, stop in neutral, STANDBY */
    ack_alarms();
    rr = sb_auto_resume(b);
    cr = do_confirm();
    if (sb_mode(b) != MODE_AUTO) {                             /* after the remote STANDBY: AUTO start again */
        rr = sb_auto_resume(b) == SB_OK ? SB_OK : sb_auto_start(b);
        cr = do_confirm();
    }
    bool arr = wait_for_c(c_arrived, 40.0 * 60.0);
    run_s(10.0);
    snprintf(note, sizeof note, "%s", "");
    row("arrival at the last waypoint (STOP)", rr == SB_OK && cr && arr && sim_hal_throttle() < 0.6f &&
        gear_pos(sim_hal_gear_pwm()) == 0, "ARRIVED, idle + neutral", "resume %d + CONFIRM %d, arrived %d, throttle %.1f %%, "
        "gear %s, mode %d%s", (int)rr, cr, arr, sim_hal_throttle(), gear_pos(sim_hal_gear_pwm()) == 0 ? "N" : "?",
        (int)sb_mode(b), note);
    press_btn(HAL_BTN_STANDBY);
    row("STANDBY", sb_mode(b) == MODE_STANDBY, "STANDBY", "mode %d", (int)sb_mode(b));
    int fails = 0;
    for (int i = 0; i < nR; i++) fails += !R[i].pass;
    printf("\nDAY AT SEA (AURIX part): %d steps, %d failed; invariants during the whole day: %s\n", nR, fails,
           C.failed ? C.text : "all held");
    if (s_md) {
        fprintf(s_md, "\nInvariants checked after every control step during the whole day: %s\n",
                C.failed ? C.text : "all held");
        fclose(s_md);
    }
    return fails || C.failed;
}

/* Proof that the checks catch real violations: each case breaks one rule behind the
 * controller's back and the matching invariant must FAIL. */
static int selftest(void)
{
    static const struct { int inv; const char *what; } T[] = {
        { 7,  "power limit raised directly (no request, no CONFIRM)" },
        { 10, "mode forced MANUAL -> REMOTE (not in the table)" },
        { 1,  "throttle servo written above the limit (not by the servo clamp)" },
    };
    int bad = 0;
    printf("=== SELF-TEST: the checks must catch these ===\n");
    for (int k = 0; k < 3; k++) {
        sched_t *sc = &SC;
        memset(sc, 0, sizeof *sc);
        sc->seed = 77u;
        sc->dur_ms = 60000u;
        sim_init(sc);
        memset(&C, 0, sizeof C);
        S.jx = 0.0f;
        joy_apply();
        run_until(5000u);
        if (k == 0) {
            (void)plim_step_up(&S.sb.plim, 0u, S.sb.now_ms);
        } else if (k == 1) {
            S.jx = 0.8f;
            joy_apply();
            run_until(S.t + 1000u);                                    /* MANUAL */
            S.sb.modes.mode = MODE_REMOTE;
            check_step();                                              /* look before the next step */
        } else {
            S.sb.test_drive.on = false;
            run_until(S.t + 100u);
            hal_set_throttle_pwm(1800u);                               /* ~80 % at the 30 % limit */
            check_step();
        }
        run_until(S.t + 2000u);
        bool caught = C.failed && C.inv == T[k].inv;
        printf("%s  %-62s -> %s\n", caught ? "PASS" : "FAIL", T[k].what,
               C.failed ? C.text : "NOT DETECTED");
        bad += !caught;
    }
    return bad;
}

int main(int argc, char **argv)
{
    int episodes = 200;
    uint32_t seed0 = 1u;
    int min_min = 5, max_min = 30;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--quiet")) continue;
        if (!strcmp(argv[i], "--wrap")) return wrap_test();
        if (!strcmp(argv[i], "--selftest")) return selftest();
        if (!strcmp(argv[i], "--dayatsea")) return day_at_sea();
        if (!strcmp(argv[i], "--replay") && i + 1 < argc) {
            uint32_t sd = (uint32_t)strtoul(argv[i + 1], NULL, 10);
            uint32_t r = sd * 7u + 3u;
            gen_schedule(&SC, sd, (uint32_t)(5 + rndi(&r, 26)) * 60000u);
            s_verbose = true;
            bool f = episode(&SC, true);
            printf("seed %lu: %s\n", (unsigned long)sd, f ? C.text : "all invariants held");
            if (f) {                                    /* the mode manager's last entries (what happened) */
                uint32_t n = mode_log_size(&S.sb.modes);
                for (uint32_t i = n > 8u ? n - 8u : 0u; i < n; i++) {
                    const mode_log_entry_t *e = mode_log_get(&S.sb.modes, i);
                    printf("  mode log %8.2f s  %s -> %s  why %s  %s\n", e->t_ms * 0.001, mode_name(e->from),
                           mode_name(e->to), mode_reason_name(e->why), e->verdict == MODE_OK ? "OK" : "refused");
                }
            }
            return f ? 1 : 0;
        }
        if (!strcmp(argv[i], "--soak") && i + 2 < argc) {
            double h = atof(argv[i + 1]);
            uint32_t sd = (uint32_t)strtoul(argv[i + 2], NULL, 10);
            gen_schedule(&SC, sd, (uint32_t)(h * 3600000.0));
            s_log = (i + 3 < argc && !strcmp(argv[i + 3], "--log"));
            sim_hal_real_micros(true);                  /* I18: real PC time of each control step */
            bool f = episode(&SC, false);
            printf("=== SOAK %.1f simulated hours, seed %lu: %s; control steps %lu, boots %lu, core 0 overruns %lu, "
                   "log queue drops %lu ===\n", h, (unsigned long)sd, f ? C.text : "all invariants held",
                   (unsigned long)C.steps, (unsigned long)S.boots, (unsigned long)g_timing.core0.overruns,
                   (unsigned long)g_logq_gw.dropped);
            printf("I18 control step with%s logging: %lu steps, longest step %lu us of the 50000 us budget (PC time); "
                   "log records %lu, files %lu, bytes %lu, queue drops core0 %lu / pi %lu (counted, never waited)\n",
                   s_log ? "" : "out", (unsigned long)s_ctrl_n, (unsigned long)s_ctrl_max_us,
                   (unsigned long)g_logstat.records, (unsigned long)g_logstat.files, (unsigned long)g_logstat.bytes,
                   (unsigned long)g_logq0.dropped, (unsigned long)g_logq_gw.dropped);
            {
                uint32_t acc = 0u, p999 = 0u;
                for (int b = 0; b < 64; b++) {
                    acc += s_hist[b];
                    if (acc >= s_ctrl_n - s_ctrl_n / 1000u) { p999 = (uint32_t)(b + 1) * 100u; break; }
                }
                printf("I18 control step time: average %.0f us, 99.9 %% of the steps below %lu us, steps over 1 ms: %lu "
                       "(PC; the TC375 must be measured on the board)\n", s_ctrl_n ? s_ctrl_sum_us / s_ctrl_n : 0.0,
                       (unsigned long)p999, (unsigned long)(s_ctrl_n - (s_hist[0] + s_hist[1] + s_hist[2] + s_hist[3] +
                       s_hist[4] + s_hist[5] + s_hist[6] + s_hist[7] + s_hist[8] + s_hist[9])));
            }
            report(1, h);
            /* step 13d (review 49353b2 LOW-2): I59's key case (a DP TRIP while an AUTO RETURN is latched) must be
             * reached, else I59 passed without testing anything */
            if (ST.dp_trip_ret == 0u)
                printf("FAIL I59 coverage: no DP TRIP step with an AUTO RETURN latched in this soak - I59 not tested\n");
            return (f || ST.dp_trip_ret == 0u) ? 1 : 0;
        }
        if (!strcmp(argv[i], "--min-min") && i + 1 < argc) { min_min = atoi(argv[++i]); continue; }
        if (!strcmp(argv[i], "--max-min") && i + 1 < argc) { max_min = atoi(argv[++i]); continue; }
        if (i == 1) episodes = atoi(argv[i]);
        if (i == 2) seed0 = (uint32_t)strtoul(argv[i], NULL, 10);
    }
    double hours = 0.0;
    int failing = 0;
    for (int k = 0; k < episodes; k++) {
        uint32_t sd = seed0 + (uint32_t)k;
        uint32_t r = sd * 7u + 3u;
        uint32_t dur = (uint32_t)(min_min + rndi(&r, max_min - min_min + 1)) * 60000u;
        gen_schedule(&SC, sd, dur);
        hours += dur / 3600000.0;
        if (episode(&SC, false)) {
            int inv = C.inv;
            char text[200];
            snprintf(text, sizeof text, "%s", C.text);
            uint32_t at = C.t;
            ST.fails[inv]++;
            failing++;
            int kept = shrink(&SC, inv);
            if (ST.fails[inv] == 1u) {
                ST.first_fail_seed[inv] = sd;
                snprintf(ST.first_fail_text[inv], sizeof ST.first_fail_text[inv], "%s at %.1f s; minimal "
                         "reproduction: %d of %d events (storm.exe --replay %lu)", text, at * 0.001, kept, SC.n,
                         (unsigned long)sd);
            }
            printf("FAIL seed %lu %s: %s at %.1f s - minimal %d events:\n", (unsigned long)sd, INV_NAMES[inv],
                   text, at * 0.001, kept);
            for (int i = 0; i < SC.n; i++)
                if (SC.on[i]) printf("      %8.1f s  %s a=%d b=%d c=%d f=%.2f g=%.2f\n", SC.ev[i].t * 0.001,
                                     EV_NAMES[SC.ev[i].kind], SC.ev[i].a, SC.ev[i].b, SC.ev[i].c, SC.ev[i].f,
                                     SC.ev[i].g);
        }
        if ((k + 1) % 100 == 0) {
            printf("... %d episodes, %.0f simulated hours, %d failing\n", k + 1, hours, failing);
            fflush(stdout);
        }
    }
    report(episodes, hours);
    /* WB review LOW-2: a run of >= 200 episodes that never entered a CREWED MANEUVER by the CONFIRM did not test
     * I63's WB clause (a 200-episode run reaches ~45) */
    bool mnvc_fail = episodes >= 200 && ST.mnv_crewed_conf == 0u;
    if (mnvc_fail) printf("FAIL I63 coverage: no CREWED MANEUVER entered by the CONFIRM - the WB clause not tested\n");
    return (failing || mnvc_fail) ? 1 : 0;
}
