/*
 * Trygons C2 - USB joystick bridge (Teensy 4.1).
 *
 * ONE job: read up to two USB HID joysticks (through a small powered USB
 * hub on the Teensy's USB host port) and send their state to the AURIX on
 * Serial1 (pin 1 TX, pin 0 RX, 3.3 V) every 10 ms. Frame format:
 * app/jsframe.h. Nothing else - no Wi-Fi, no control logic, nothing the
 * AURIX could depend on for safety except "fresh, CRC-checked frames".
 *
 * USB serial console (115200), one-letter commands:
 *   l  learn mode on/off - prints every axis / button / hat you move,
 *      with the code number to use in the AURIX button map
 *   c  calibrate: move every axis to both ends, then press c again (saves)
 *   x  cancel calibration
 *   w  swap the two devices (stick <-> throttle), saved
 *   r  remove all saved calibrations / swaps
 *   ?  status
 * In learn mode the frames carry the "learn" flag and the AURIX keeps the
 * USB joystick disarmed.
 */
#include <Arduino.h>
#include <EEPROM.h>
#include <USBHost_t36.h>

#include <Watchdog_t4.h>     /* lib_deps in platformio.ini - required, never optional */
#define HAVE_WDT 1

#include "../../app/jsframe.h"
#include "devices.h"

/* ---- USB host: hub + up to 4 HID interfaces + 2 joysticks ---- */
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHIDParser hid1(myusb), hid2(myusb), hid3(myusb), hid4(myusb);
JoystickController joyA(myusb), joyB(myusb);
static JoystickController *const JOYS[2] = { &joyA, &joyB };

#define HAT_AXIS 9          /* HID usage 0x39 */

/* ---- saved calibration / roles (EEPROM, with CRC) ---- */
#define STORE_MAGIC 0x4A534231u     /* "JSB1" */
#define STORE_ENTRIES 4

struct CalEntry {
    uint16_t vid, pid;
    uint8_t  used, role;            /* role: ROLE_STICK / ROLE_THROTTLE / ROLE_ANY */
    uint8_t  has_range, pad;
    int32_t  lo[JSF_AXES], hi[JSF_AXES];
};

struct Store {
    uint32_t magic;
    CalEntry e[STORE_ENTRIES];
    uint16_t crc;
};

static Store store;

static uint16_t store_crc(void)
{
    return jsf_crc16((const uint8_t *)&store, offsetof(Store, crc));
}

static void store_load(void)
{
    EEPROM.get(0, store);
    if (store.magic != STORE_MAGIC || store.crc != store_crc()) {
        memset(&store, 0, sizeof store);
        store.magic = STORE_MAGIC;
    }
}

static void store_save(void)
{
    store.crc = store_crc();
    EEPROM.put(0, store);
}

static CalEntry *store_find(uint16_t vid, uint16_t pid, bool create)
{
    for (int i = 0; i < STORE_ENTRIES; i++)
        if (store.e[i].used && store.e[i].vid == vid && store.e[i].pid == pid) return &store.e[i];
    if (!create) return nullptr;
    for (int i = 0; i < STORE_ENTRIES; i++)
        if (!store.e[i].used) {
            CalEntry *e = &store.e[i];
            memset(e, 0, sizeof *e);
            e->used = 1;
            e->vid = vid;
            e->pid = pid;
            e->role = ROLE_ANY;
            return e;
        }
    return nullptr;
}

static const known_dev_t *known_find(uint16_t vid, uint16_t pid)
{
    for (unsigned i = 0; i < KNOWN_DEV_COUNT; i++)
        if (KNOWN_DEVS[i].vid == vid && KNOWN_DEVS[i].pid == pid) return &KNOWN_DEVS[i];
    return nullptr;
}

/* ---- device -> frame slot ---- */
static int slot_of[2] = { -1, -1 };     /* JOYS[i] -> frame device 0/1, -1 = not connected */

static uint8_t role_of(JoystickController *j)
{
    const CalEntry *e = store_find(j->idVendor(), j->idProduct(), false);
    if (e && e->role != ROLE_ANY) return e->role;
    const known_dev_t *k = known_find(j->idVendor(), j->idProduct());
    return k ? k->role : ROLE_ANY;
}

static void assign_slots(void)
{
    bool taken[2] = { false, false };
    for (int i = 0; i < 2; i++) {
        if (!*JOYS[i]) { slot_of[i] = -1; continue; }
        if (slot_of[i] >= 0) taken[slot_of[i]] = true;
    }
    for (int i = 0; i < 2; i++) {
        if (!*JOYS[i] || slot_of[i] >= 0) continue;
        uint8_t r = role_of(JOYS[i]);
        int s = (r != ROLE_ANY && !taken[r]) ? r : (!taken[0] ? 0 : (!taken[1] ? 1 : -1));
        slot_of[i] = s;
        if (s >= 0) taken[s] = true;
    }
}

static bool axis_range(JoystickController *j, int a, int32_t *lo, int32_t *hi)
{
    const CalEntry *e = store_find(j->idVendor(), j->idProduct(), false);
    if (e && e->has_range) { *lo = e->lo[a]; *hi = e->hi[a]; return *lo < *hi; }
    const known_dev_t *k = known_find(j->idVendor(), j->idProduct());
    if (k) { *lo = k->lo[a]; *hi = k->hi[a]; return *lo < *hi; }
    return false;
}

/* ---- build one frame ---- */
static jsf_t frame;
static uint8_t seq;
static bool learn;

static void read_device(JoystickController *j, jsf_dev_t *d)
{
    jsf_dev_clear(d);
    d->vid = j->idVendor();
    d->pid = j->idProduct();
    uint64_t mask = j->axisMask();
    for (int a = 0; a < JSF_AXES; a++) {
        int32_t lo, hi;
        if (!(mask & (1ull << a)) || !axis_range(j, a, &lo, &hi)) continue;
        d->axis[a] = jsf_norm(j->getAxis(a), lo, hi, false);
    }
    d->buttons = j->getButtons();
    if (mask & (1ull << HAT_AXIS)) {
        int h = j->getAxis(HAT_AXIS);
        d->hats = (uint8_t)((JSF_HAT_CENTRE << 4) | ((h >= 0 && h <= 7) ? h : JSF_HAT_CENTRE));
    }
}

static void build_frame(void)
{
    assign_slots();
    frame.seq = seq++;
    frame.flags = 0;
    jsf_dev_clear(&frame.dev[0]);
    jsf_dev_clear(&frame.dev[1]);
    for (int i = 0; i < 2; i++) {
        int s = slot_of[i];
        if (s < 0) continue;
        read_device(JOYS[i], &frame.dev[s]);
        frame.flags |= (s == 0) ? JSF_F_DEV0 : JSF_F_DEV1;
    }
    if (learn) frame.flags |= JSF_F_LEARN;
    if (hub1 || hub2) frame.flags |= JSF_F_HUB;
}

/* ---- learn mode: print what moved ---- */
static jsf_t shown;
static const char *const AXIS_NAME[JSF_AXES] = { "X", "Y", "Z", "Rx", "Ry", "Rz", "Slider", "Dial" };
static const char *const DEV_NAME[2] = { "stick", "throttle" };
static const char *const DIR_NAME[4] = { "UP", "RIGHT", "DOWN", "LEFT" };

static void learn_print(void)
{
    for (int d = 0; d < JSF_DEVS; d++) {
        const jsf_dev_t *n = &frame.dev[d];
        jsf_dev_t *o = &shown.dev[d];
        for (int a = 0; a < JSF_AXES; a++) {
            if (n->axis[a] == JSF_AXIS_NONE) {
                if (o->axis[a] != JSF_AXIS_NONE) { Serial.printf("dev%d (%s) axis %d %s: gone/invalid\n", d, DEV_NAME[d], a, AXIS_NAME[a]); o->axis[a] = JSF_AXIS_NONE; }
                continue;
            }
            if (o->axis[a] == JSF_AXIS_NONE || abs(n->axis[a] - o->axis[a]) > 3000) {
                Serial.printf("dev%d (%s) axis %d %s: %+6.1f %%   (axis code %u)\n", d, DEV_NAME[d], a, AXIS_NAME[a],
                              n->axis[a] * 100.0 / 32767.0, JSF_AXIS_CODE(d, a));
                o->axis[a] = n->axis[a];
            }
        }
        uint32_t ch = n->buttons ^ o->buttons;
        for (int b = 0; b < 32; b++)
            if (ch & (1u << b))
                Serial.printf("dev%d (%s) button %d %s   (code %u)\n", d, DEV_NAME[d], b + 1,
                              (n->buttons >> b) & 1u ? "PRESSED" : "released", JSF_CODE_BTN(d, b));
        o->buttons = n->buttons;
        uint8_t h = jsf_hat(n, 0);
        if (h != jsf_hat(o, 0)) {
            if (h == JSF_HAT_CENTRE) Serial.printf("dev%d (%s) hat centred\n", d, DEV_NAME[d]);
            else if (h % 2u) Serial.printf("dev%d (%s) hat diagonal (ignored by the AURIX)\n", d, DEV_NAME[d]);
            else Serial.printf("dev%d (%s) hat %s   (code %u)\n", d, DEV_NAME[d], DIR_NAME[h / 2u], JSF_CODE_HAT(d, 0, h / 2u));
            o->hats = n->hats;
        }
    }
}

/* ---- calibration ---- */
static bool calibrating;
static int32_t cal_lo[2][JSF_AXES], cal_hi[2][JSF_AXES];

static void cal_start(void)
{
    for (int i = 0; i < 2; i++)
        for (int a = 0; a < JSF_AXES; a++) { cal_lo[i][a] = INT32_MAX; cal_hi[i][a] = INT32_MIN; }
    calibrating = true;
    Serial.println("CALIBRATE: move EVERY axis slowly to both ends (stick, twist, lever, rocker...), then press c. x = cancel.");
}

static void cal_sample(void)
{
    for (int i = 0; i < 2; i++) {
        if (!*JOYS[i]) continue;
        uint64_t mask = JOYS[i]->axisMask();
        for (int a = 0; a < JSF_AXES; a++) {
            if (!(mask & (1ull << a))) continue;
            int32_t v = JOYS[i]->getAxis(a);
            if (v < cal_lo[i][a]) cal_lo[i][a] = v;
            if (v > cal_hi[i][a]) cal_hi[i][a] = v;
        }
    }
}

static void cal_finish(void)
{
    calibrating = false;
    for (int i = 0; i < 2; i++) {
        if (!*JOYS[i]) continue;
        CalEntry *e = store_find(JOYS[i]->idVendor(), JOYS[i]->idProduct(), true);
        if (!e) { Serial.println("no free calibration slot - press r first"); return; }
        e->has_range = 1;
        for (int a = 0; a < JSF_AXES; a++) {
            bool ok = cal_lo[i][a] < cal_hi[i][a] && (cal_hi[i][a] - cal_lo[i][a]) >= 50;
            e->lo[a] = ok ? cal_lo[i][a] : 0;
            e->hi[a] = ok ? cal_hi[i][a] : 0;
            if (ok) Serial.printf("  %04X:%04X axis %d %s: %ld .. %ld\n", e->vid, e->pid, a, AXIS_NAME[a],
                                  (long)e->lo[a], (long)e->hi[a]);
        }
    }
    store_save();
    Serial.println("CALIBRATION SAVED. Axes not moved are now 'invalid'.");
}

static void swap_roles(void)
{
    for (int i = 0; i < 2; i++) {
        if (!*JOYS[i] || slot_of[i] < 0) continue;
        CalEntry *e = store_find(JOYS[i]->idVendor(), JOYS[i]->idProduct(), true);
        if (e) e->role = (uint8_t)(1 - slot_of[i]);
        slot_of[i] = -1;
    }
    store_save();
    Serial.println("devices swapped and saved");
}

static void status(void)
{
    Serial.printf("Trygons joystick bridge  learn=%s  hub=%s  frames sent=%u\n", learn ? "ON" : "off",
                  (hub1 || hub2) ? "yes" : "no", (unsigned)seq);
    for (int i = 0; i < 2; i++) {
        if (!*JOYS[i]) continue;
        const known_dev_t *k = known_find(JOYS[i]->idVendor(), JOYS[i]->idProduct());
        const CalEntry *e = store_find(JOYS[i]->idVendor(), JOYS[i]->idProduct(), false);
        Serial.printf("  dev%d = %04X:%04X %s%s\n", slot_of[i], JOYS[i]->idVendor(), JOYS[i]->idProduct(),
                      k ? k->name : "(unknown joystick - calibrate with c)", (e && e->has_range) ? " [calibrated]" : "");
    }
    Serial.println("commands: l learn, c calibrate, x cancel, w swap devices, r reset saved data, ? status");
}

static void console(void)
{
    while (Serial.available()) {
        int c = Serial.read();
        switch (c) {
        case 'l': learn = !learn; memset(&shown, 0, sizeof shown); Serial.printf("learn mode %s\n", learn ? "ON" : "OFF"); break;
        case 'c': if (calibrating) cal_finish(); else cal_start(); break;
        case 'x': calibrating = false; Serial.println("calibration cancelled"); break;
        case 'w': swap_roles(); break;
        case 'r': memset(&store, 0, sizeof store); store.magic = STORE_MAGIC; store_save(); Serial.println("saved data cleared"); break;
        case '?': status(); break;
        default: break;
        }
    }
}

#ifdef HAVE_WDT
static WDT_T4<WDT1> wdt;
#endif

void setup()
{
    Serial.begin(115200);
    Serial1.begin(JSF_BAUD);
    store_load();
    myusb.begin();
#ifdef HAVE_WDT
    WDT_timings_t cfg;
    cfg.timeout = 0.5;          /* s - a hung bridge stops sending, the AURIX sees a frame timeout */
    wdt.begin(cfg);
#endif
}

void loop()
{
    static elapsedMicros since_frame;
    myusb.Task();
    console();
    if (calibrating) cal_sample();
    if (since_frame >= JSF_PERIOD_MS * 1000u) {
        since_frame -= JSF_PERIOD_MS * 1000u;
        if (since_frame > JSF_PERIOD_MS * 1000u) since_frame = 0;     /* fell behind: no burst */
        build_frame();
        uint8_t buf[JSF_FRAME_LEN];
        size_t n = jsf_encode(&frame, buf);
        Serial1.write(buf, n);
        if (learn) learn_print();
#ifdef HAVE_WDT
        wdt.feed();
#endif
    }
}
