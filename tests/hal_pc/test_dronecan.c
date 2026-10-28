/*
 * test_dronecan.c - PC harness for hal/dronecan.c + hal/libcanard (driven by
 * dronecan_check.py, which checks every frame against its OWN encoder written
 * from the DroneCAN DSDL / transport rules). A fake CAN controller stands in for
 * the MCMCAN (hal/tc_can.c on the board).
 *
 * Commands on stdin, one per line (numbers in hex where it says so):
 *   T <ms>                 set the time (decimal) and run one dc_service
 *   S                      one dc_service at the same time
 *   RX <id hex> <data hex> a frame from the bus into the RX FIFO
 *   STATE <0|1|2> [tec] [rec]   controller: 0 active, 1 error passive, 2 bus-off
 *   ROOM <n>               frames the controller TX FIFO takes from now on (decimal)
 *   ACT <id> <type> <float>     dc_send_actuator
 *   HP <id> <cmd>          dc_send_hardpoint (decimal)
 *   ACK                    dc_ack
 *   OWN <health> <mode>    dc_set_own_status
 *   ENC_STATUS <uptime> <health> <mode> <sub> <vendor>   payload of dc_encode_status (decimal)
 *   ENC_ACMD <n> {<id> <type> <float>}   payload of dc_encode_array_command
 *   ENC_HP <id> <cmd>      payload of dc_encode_hardpoint
 *   DEC_ACMD <hex>         decoded commands
 *   DEC_STATUS <hex>       decoded NodeStatus
 *   ENC_INFO <uptime>      payload of dc_encode_node_info (health OK, OPERATIONAL)
 *   CRCSIG <sig hex>       dc_crc_signature
 *   F16 <float>            canardConvertNativeFloatToFloat16
 *   START_FAIL             the next INIT fails
 *   INIT                   dc_init (time unchanged)
 *   SAFE <act> <float> <tid>    CPU1's safe frame (hal/can_safe.c cs_frame): "SAFE <id> <data>"
 *   CANCELTX               dc_cancel_tx (step 11e: a trip / the latch came in between)
 *   TID                    dc_cmd_tid: "TID <n>"
 * The controller's FIFO cancel (port cancel, step 11e) prints "CANCEL".
 * Output: "TX <id> <data>" for every frame the controller took, then the
 * answer of the command ("NODE ...", "BUS ...", "PAYLOAD ...", ...).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dronecan.h"
#include "can_safe.h"
#include "libcanard/canard.h"

static uint32_t s_ms;
static dc_frame_t s_rx[512];
static uint32_t s_rx_head, s_rx_n;
static uint8_t s_state, s_tec, s_rec;
static uint32_t s_room = 1000000u, s_recovers;
static bool s_start_fail;

static bool p_start(void) { return !s_start_fail; }
static bool p_send(const dc_frame_t *f)
{
    if (s_room == 0u) return false;
    s_room--;
    printf("TX %08X ", (unsigned)f->id);
    for (uint32_t i = 0; i < f->len; i++) printf("%02X", f->data[i]);
    printf("\n");
    return true;
}
static bool p_recv(dc_frame_t *f)
{
    if (s_rx_n == 0u) return false;
    *f = s_rx[s_rx_head];
    s_rx_head = (s_rx_head + 1u) % 512u;
    s_rx_n--;
    return true;
}
static uint8_t p_state(uint8_t *tec, uint8_t *rec)
{
    *tec = s_tec;
    *rec = s_rec;
    return s_state;
}
static void p_recover(void) { s_recovers++; }
static uint32_t p_millis(void) { return s_ms; }
static void p_cancel(void) { printf("CANCEL\n"); }
static const dc_port_t s_port = { p_start, p_send, p_recv, p_state, p_recover, p_millis, p_cancel };

static uint32_t hexbytes(const char *h, uint8_t *out, uint32_t max)
{
    uint32_t n = 0;
    while (h[0] && h[1] && n < max) {
        unsigned v;
        if (sscanf(h, "%2x", &v) != 1) break;
        out[n++] = (uint8_t)v;
        h += 2;
    }
    return n;
}

static void dump(const char *tag, const uint8_t *p, uint32_t n)
{
    printf("%s ", tag);
    for (uint32_t i = 0; i < n; i++) printf("%02X", p[i]);
    printf("\n");
}

static void report(void)
{
    hal_can_bus_t b;
    dc_bus(&b);
    const dc_stats_t *st = dc_stats();
    printf("BUS state=%u fault=%d conflict=%d anon=%d silent=%d tec=%u rec=%u rx=%u tx=%u drops=%u rxerr=%u busoffs=%u "
           "recovers=%u replies=%u pool=%u\n",
           b.state, b.fault, b.id_conflict, b.anon_node, b.silent, b.tec, b.rec, (unsigned)b.rx, (unsigned)b.tx,
           (unsigned)b.tx_drops, (unsigned)b.rx_errors, (unsigned)b.busoffs, (unsigned)s_recovers,
           (unsigned)st->nodeinfo_replies, (unsigned)st->pool_peak);
    for (uint8_t i = 0; i < HAL_CAN_NODES; i++) {
        hal_can_node_t n;
        if (!dc_node(i, &n)) continue;
        printf("NODE id=%u role=%u req=%d seen=%d lost=%d why=%u warn=%d reb=%d health=%u mode=%u up=%u restarts=%u "
               "statuses=%u\n", n.id, n.role, n.required, n.seen, n.lost, n.why, n.warning, n.rebooted, n.health,
               n.mode, (unsigned)n.uptime_s, n.restarts, (unsigned)n.statuses);
    }
}

int main(void)
{
    char line[1024];
    setvbuf(stdout, NULL, _IONBF, 0);
    while (fgets(line, sizeof line, stdin)) {
        char cmd[32] = "";
        if (sscanf(line, "%31s", cmd) != 1) continue;
        const char *a = line + strlen(cmd);
        if (!strcmp(cmd, "INIT")) {
            s_rx_n = s_rx_head = 0u;
            dc_init(&s_port);
        } else if (!strcmp(cmd, "START_FAIL")) {
            s_start_fail = true;
        } else if (!strcmp(cmd, "T")) {
            s_ms = (uint32_t)strtoul(a, NULL, 10);
            dc_service();
            report();
        } else if (!strcmp(cmd, "S")) {
            dc_service();
            report();
        } else if (!strcmp(cmd, "RX")) {
            char id[16], data[64] = "";
            if (sscanf(a, "%15s %63s", id, data) < 1) continue;
            dc_frame_t f;
            memset(&f, 0, sizeof f);
            f.id = (uint32_t)strtoul(id, NULL, 16);
            f.len = (uint8_t)hexbytes(data, f.data, 8u);
            if (s_rx_n < 512u) {
                s_rx[(s_rx_head + s_rx_n) % 512u] = f;
                s_rx_n++;
            }
        } else if (!strcmp(cmd, "STATE")) {
            unsigned st = 0, t = 0, r = 0;
            sscanf(a, "%u %u %u", &st, &t, &r);
            s_state = (uint8_t)st;
            s_tec = (uint8_t)t;
            s_rec = (uint8_t)r;
        } else if (!strcmp(cmd, "ROOM")) {
            s_room = (uint32_t)strtoul(a, NULL, 10);
        } else if (!strcmp(cmd, "ACT")) {
            unsigned id = 0, ty = 0;
            float v = 0.0f;
            sscanf(a, "%u %u %f", &id, &ty, &v);
            printf("QUEUED %d\n", dc_send_actuator((uint8_t)id, (uint8_t)ty, v));
        } else if (!strcmp(cmd, "HP")) {
            unsigned id = 0, c = 0;
            sscanf(a, "%u %u", &id, &c);
            printf("QUEUED %d\n", dc_send_hardpoint((uint8_t)id, (uint16_t)c));
        } else if (!strcmp(cmd, "ACK")) {
            dc_ack();
            report();
        } else if (!strcmp(cmd, "OWN")) {
            unsigned h = 0, m = 0;
            sscanf(a, "%u %u", &h, &m);
            dc_set_own_status((uint8_t)h, (uint8_t)m);
        } else if (!strcmp(cmd, "ENC_STATUS")) {
            unsigned long up = 0;
            unsigned h = 0, m = 0, sm = 0, v = 0;
            sscanf(a, "%lu %u %u %u %u", &up, &h, &m, &sm, &v);
            dc_status_t st = { (uint32_t)up, (uint8_t)h, (uint8_t)m, (uint8_t)sm, (uint16_t)v };
            uint8_t out[DC_NODESTATUS_LEN];
            dc_encode_status(out, &st);
            dump("PAYLOAD", out, DC_NODESTATUS_LEN);
        } else if (!strcmp(cmd, "ENC_ACMD")) {
            dc_cmd_t c[15];
            char *e = (char *)a;
            unsigned long n = strtoul(e, &e, 10);
            if (n > 15u) n = 15u;
            for (unsigned long i = 0; i < n; i++) {
                c[i].actuator_id = (uint8_t)strtoul(e, &e, 10);
                c[i].type = (uint8_t)strtoul(e, &e, 10);
                c[i].value = strtof(e, &e);
            }
            uint8_t out[60];
            dump("PAYLOAD", out, dc_encode_array_command(out, c, (uint32_t)n));
        } else if (!strcmp(cmd, "ENC_HP")) {
            unsigned id = 0, cm = 0;
            sscanf(a, "%u %u", &id, &cm);
            uint8_t out[DC_HARDPOINT_LEN];
            dc_encode_hardpoint(out, (uint8_t)id, (uint16_t)cm);
            dump("PAYLOAD", out, DC_HARDPOINT_LEN);
        } else if (!strcmp(cmd, "DEC_ACMD")) {
            char h[128] = "";
            sscanf(a, "%127s", h);
            uint8_t p[64];
            uint32_t n = hexbytes(h, p, sizeof p);
            dc_cmd_t c[15];
            uint32_t k = dc_decode_array_command(p, n, c, 15u);
            printf("CMDS %u", (unsigned)k);
            for (uint32_t i = 0; i < k; i++) printf(" %u:%u:%.6f", c[i].actuator_id, c[i].type, (double)c[i].value);
            printf("\n");
        } else if (!strcmp(cmd, "DEC_STATUS")) {
            char h[64] = "";
            sscanf(a, "%63s", h);
            uint8_t p[16];
            uint32_t n = hexbytes(h, p, sizeof p);
            dc_status_t st;
            bool ok = dc_decode_status(p, n, &st);
            printf("STATUS %d %u %u %u %u %u\n", ok, (unsigned)st.uptime_s, st.health, st.mode, st.sub_mode, st.vendor);
        } else if (!strcmp(cmd, "ENC_INFO")) {
            unsigned long up = 0;
            sscanf(a, "%lu", &up);
            dc_status_t st = { (uint32_t)up, 0u, 0u, 0u, 0u };
            uint8_t out[128];
            dump("PAYLOAD", out, dc_encode_node_info(out, sizeof out, &st));
        } else if (!strcmp(cmd, "CRCSIG")) {
            unsigned long long sig = strtoull(a, NULL, 16);
            printf("CRC %04X\n", dc_crc_signature(sig));
        } else if (!strcmp(cmd, "SAFE")) {
            unsigned act = 0u, tid = 0u;
            char v[64] = "";
            if (sscanf(a, "%u %63s %u", &act, v, &tid) == 3) {
                dc_frame_t f;
                cs_frame(&f, (uint8_t)act, strtof(v, NULL), (uint8_t)tid);
                printf("SAFE %08X ", (unsigned)f.id);
                for (uint32_t i = 0; i < f.len; i++) printf("%02X", f.data[i]);
                printf("\n");
            }
        } else if (!strcmp(cmd, "CANCELTX")) {
            dc_cancel_tx();
        } else if (!strcmp(cmd, "TID")) {
            printf("TID %u\n", (unsigned)dc_cmd_tid());
        } else if (!strcmp(cmd, "F16")) {
            printf("F16 %04X\n", canardConvertNativeFloatToFloat16(strtof(a, NULL)));
        } else if (!strcmp(cmd, "SIGS")) {
            printf("SIGS %016llX %016llX %016llX %016llX %016llX\n", (unsigned long long)DC_NODESTATUS_SIG,
                   (unsigned long long)DC_ARRAYCMD_SIG, (unsigned long long)DC_HARDPOINT_SIG,
                   (unsigned long long)DC_GETNODEINFO_SIG, (unsigned long long)DC_ALLOCATION_SIG);
            printf("IDS %u %u %u %u %u OWN %u NODES %u %u %u %u\n", DC_NODESTATUS_ID, DC_ARRAYCMD_ID, DC_HARDPOINT_ID,
                   DC_GETNODEINFO_ID, DC_ALLOCATION_ID, DC_OWN_NODE_ID, DC_NODE_THROTTLE, DC_NODE_STEERING,
                   DC_NODE_GEAR, DC_NODE_RELAY);
        }
        printf("END\n");
    }
    return 0;
}
