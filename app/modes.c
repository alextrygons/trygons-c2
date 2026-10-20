/*
 * modes.c - Small-boat mode manager. See modes.h.
 */
#include "modes.h"

static const bool k_allowed[MODE_COUNT][MODE_COUNT] = {
    /* to:          STANDBY MANUAL HDG_LOCK AUTO  TRAINING REMOTE */
    /* STANDBY  */ { false, true,  true,    true,  true,    true  },
    /* MANUAL   */ { true,  false, true,    true,  false,   false },   /* the onboard joystick has it */
    /* HDG_LOCK */ { true,  true,  false,   true,  false,   true  },
    /* AUTO     */ { true,  true,  true,    false, false,   true  },
    /* TRAINING */ { true,  true,  false,   false, false,   false },
    /* REMOTE   */ { true,  true,  false,   false, false,   false },   /* onboard joystick takes over */
};

void mode_init(mode_mgr_t *m, uint32_t now_ms)
{
    m->mode = MODE_STANDBY;
    m->since_ms = now_ms;
    m->log_count = 0u;
}

bool mode_allowed(sb_mode_t from, sb_mode_t to)
{
    if ((unsigned)from >= MODE_COUNT || (unsigned)to >= MODE_COUNT) return false;
    return k_allowed[from][to];
}

static void log_add(mode_mgr_t *m, sb_mode_t to, mode_reason_t why,
                    mode_verdict_t v, uint32_t now_ms)
{
    mode_log_entry_t *e = &m->log[m->log_count % MODE_LOG_N];
    e->t_ms = now_ms;
    e->from = m->mode;
    e->to = to;
    e->why = why;
    e->verdict = v;
    m->log_count++;
}

mode_verdict_t mode_request(mode_mgr_t *m, sb_mode_t to, mode_reason_t why,
                            bool precond_ok, uint32_t now_ms)
{
    mode_verdict_t v;
    if (!mode_allowed(m->mode, to))  v = MODE_REJECT_TABLE;
    else if (!precond_ok)            v = MODE_REJECT_PRECOND;
    else                             v = MODE_OK;

    log_add(m, to, why, v, now_ms);
    if (v == MODE_OK) {
        m->mode = to;
        m->since_ms = now_ms;
    }
    return v;
}

uint32_t mode_log_size(const mode_mgr_t *m)
{
    return (m->log_count < MODE_LOG_N) ? m->log_count : MODE_LOG_N;
}

const mode_log_entry_t *mode_log_get(const mode_mgr_t *m, uint32_t i)
{
    uint32_t n = mode_log_size(m);
    if (i >= n) return 0;
    uint32_t first = m->log_count - n;
    return &m->log[(first + i) % MODE_LOG_N];
}

const char *mode_name(sb_mode_t m)
{
    static const char *const n[] = { "STANDBY", "MANUAL", "HDG_LOCK", "AUTO", "TRAINING", "REMOTE_MANUAL" };
    return ((unsigned)m < MODE_COUNT) ? n[m] : "?";
}

const char *mode_reason_name(mode_reason_t r)
{
    static const char *const n[] = {
        "user", "standby button", "compass fault", "GPS lost", "arrived",
        "training done", "training aborted", "test", "gear fault", "DR limit",
        "arrival uncertain", "joystick", "joystick hand-back", "joystick fault", "PC",
        "HDG LOCK button", "PC + CONFIRM", "outputs not armed", "safety monitor",
        "remote driving (HELM + CONFIRM)", "remote driver released", "remote driving link lost"
    };
    return ((unsigned)r < WHY_COUNT) ? n[r] : "?";
}
