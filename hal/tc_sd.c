/*
 * tc_sd.c - microSD over SPI (QSPI2): the card protocol and its start-up as a
 * state machine; files through the FAT32 writer (tc_fat32.c). See tc_sd.h.
 *
 * Card (SPI mode): CMD0 -> CMD8 -> ACMD41 (until ready) -> CMD58 (SDHC?)
 * [-> CMD16 512 for old SDSC cards], then 10 MHz. Blocks: CMD17 read, CMD24
 * write, each with a time limit.
 */
#include "tc_sd.h"

#include <string.h>

#include "Ifx_Types.h"
#include "IfxPort.h"

#include "hal.h"
#include "hal_pins.h"
#include "tc_fat32.h"
#include "tc_spi.h"

#define SEC          512u
#define TOKEN_MS     100u
#define BUSY_MS      150u
#define ACMD41_MS    2000u

enum { S_NOCARD = 0, S_POWER, S_CMD0, S_CMD8, S_ACMD41, S_OCR, S_FAT, S_DIRSCAN, S_READY, S_FAIL };

static sd_stat_t s_st;
static uint8_t s_state;
static bool s_v2;
static uint32_t s_t0, s_fail_ms;
static uint8_t s_cmd0_tries;

/* --- SPI / card ------------------------------------------------------------------ */
static void cs(bool on) { if (on) IfxPort_setPinLow(PIN_SD_CS); else IfxPort_setPinHigh(PIN_SD_CS); }
static bool card_detect(void) { return !IfxPort_getPinState(PIN_SD_CD); }

static uint8_t byte_x(uint8_t b)
{
    uint8_t r = 0xFF;
    if (!spi_xfer(SPI_SD, &b, &r, 1u, 5u)) return 0xFF;
    return r;
}

static bool wait_ready(uint32_t ms)
{
    uint32_t t0 = hal_millis();
    do {
        if (byte_x(0xFF) == 0xFF) return true;
    } while ((uint32_t)(hal_millis() - t0) < ms);
    s_st.timeouts++;
    return false;
}

/* Command with CS already low; returns R1 (0xFF = no answer). */
static uint8_t cmd(uint8_t c, uint32_t arg)
{
    uint8_t f[6] = { (uint8_t)(0x40u | c), (uint8_t)(arg >> 24), (uint8_t)(arg >> 16), (uint8_t)(arg >> 8), (uint8_t)arg,
                     (uint8_t)(c == 0u ? 0x95u : c == 8u ? 0x87u : 0x01u) };
    (void)byte_x(0xFF);
    if (!spi_xfer(SPI_SD, f, NULL, 6u, 5u)) return 0xFF;
    for (int i = 0; i < 10; i++) {
        uint8_t r = byte_x(0xFF);
        if (!(r & 0x80u)) return r;
    }
    return 0xFF;
}

static void deselect(void)
{
    cs(false);
    (void)byte_x(0xFF);
}

static bool fail_io(void)
{
    s_st.errors++;
    deselect();
    return false;
}

static bool read_block(uint32_t lba, uint8_t *buf)
{
    cs(true);
    if (cmd(17u, s_st.sdhc ? lba : lba * SEC) != 0u) return fail_io();
    uint32_t t0 = hal_millis();
    uint8_t t;
    while ((t = byte_x(0xFF)) == 0xFF) {
        if ((uint32_t)(hal_millis() - t0) > TOKEN_MS) { s_st.timeouts++; return fail_io(); }
    }
    if (t != 0xFE) return fail_io();
    if (!spi_xfer(SPI_SD, NULL, buf, SEC, 20u)) return fail_io();
    (void)byte_x(0xFF);
    (void)byte_x(0xFF);                                         /* CRC */
    deselect();
    return true;
}

static bool write_block(uint32_t lba, uint8_t *buf)
{
    cs(true);
    if (cmd(24u, s_st.sdhc ? lba : lba * SEC) != 0u) return fail_io();
    (void)byte_x(0xFF);
    (void)byte_x(0xFE);                                         /* start token */
    if (!spi_xfer(SPI_SD, buf, NULL, SEC, 20u)) return fail_io();
    (void)byte_x(0xFF);
    (void)byte_x(0xFF);                                         /* CRC (not checked in SPI mode) */
    if ((byte_x(0xFF) & 0x1Fu) != 0x05u) return fail_io();      /* data accepted */
    if (!wait_ready(BUSY_MS)) return fail_io();                 /* programming */
    deselect();
    return true;
}

void sd_service(uint32_t budget_ms)
{
    static bool pins_done;
    if (!pins_done) {
        IfxPort_setPinHigh(PIN_SD_CS);
        IfxPort_setPinModeOutput(PIN_SD_CS, IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
        IfxPort_setPinModeInput(PIN_SD_CD, IfxPort_InputMode_pullUp);
        pins_done = true;
    }
    s_st.card = card_detect();
    if (!s_st.card) {
        if (s_state != S_NOCARD) { fat32_unmount(); s_st.mounted = false; }
        s_state = S_NOCARD;
        s_st.state = s_state;
        return;
    }
    uint32_t t0 = hal_millis();
    while (s_state != S_READY && (uint32_t)(hal_millis() - t0) < budget_ms) {
        switch (s_state) {
        case S_NOCARD:
        case S_POWER:
            spi_init(SPI_SD, 400000.0f);
            cs(false);
            for (int i = 0; i < 10; i++) (void)byte_x(0xFF);   /* >= 74 clocks with CS high */
            s_cmd0_tries = 0;
            s_state = S_CMD0;
            break;
        case S_CMD0:
            cs(true);
            if (cmd(0u, 0u) == 0x01u) s_state = S_CMD8;
            else if (++s_cmd0_tries > 20u) { s_state = S_FAIL; s_fail_ms = hal_millis(); }
            deselect();
            break;
        case S_CMD8: {
            cs(true);
            uint8_t r = cmd(8u, 0x1AAu), r7[4] = { 0, 0, 0, 0 };
            if (r == 0x01u) (void)spi_xfer(SPI_SD, NULL, r7, 4u, 5u);
            deselect();
            s_v2 = (r == 0x01u && r7[3] == 0xAAu);
            s_t0 = hal_millis();
            s_state = S_ACMD41;
            break;
        }
        case S_ACMD41: {
            cs(true);
            (void)cmd(55u, 0u);
            uint8_t r = cmd(41u, s_v2 ? 0x40000000u : 0u);
            deselect();
            if (r == 0x00u) s_state = S_OCR;
            else if ((uint32_t)(hal_millis() - s_t0) > ACMD41_MS) { s_state = S_FAIL; s_fail_ms = hal_millis(); }
            break;
        }
        case S_OCR: {
            uint8_t ocr[4] = { 0, 0, 0, 0 };
            s_st.sdhc = false;
            cs(true);
            if (s_v2 && cmd(58u, 0u) == 0x00u) {
                (void)spi_xfer(SPI_SD, NULL, ocr, 4u, 5u);
                s_st.sdhc = (ocr[0] & 0x40u) != 0u;
            }
            if (!s_st.sdhc) (void)cmd(16u, SEC);
            deselect();
            spi_set_speed(SPI_SD, 10000000.0f);
            s_state = S_FAT;
            break;
        }
        case S_FAT:
            if (fat32_mount(read_block, write_block)) s_state = S_DIRSCAN;
            else { s_state = S_FAIL; s_fail_ms = hal_millis(); }
            break;
        case S_DIRSCAN: {
            int r = fat32_scan_step();
            if (r < 0) { s_state = S_FAIL; s_fail_ms = hal_millis(); }
            else if (r > 0) { s_state = S_READY; s_st.mounted = true; s_st.mounts++; }
            break;
        }
        case S_FAIL:
        default:
            if ((uint32_t)(hal_millis() - s_fail_ms) > 1000u) s_state = S_POWER;   /* try again */
            else budget_ms = 0u;
            break;
        }
    }
    s_st.state = s_state;
}

bool sd_present(void)
{
    if (s_state != S_READY || !card_detect()) sd_service(150u);
    return s_state == S_READY;
}

int sd_open(const char *name)
{
    if (!sd_present()) return -1;
    return fat32_open(name);
}

/* a failed write / sync: the card is started again before the next file */
static bool check(bool ok)
{
    if (!ok) { s_state = S_FAIL; s_fail_ms = hal_millis(); fat32_unmount(); s_st.mounted = false; }
    return ok;
}

bool sd_write(int h, const void *data, uint32_t len)
{
    return s_state == S_READY && check(fat32_write(h, data, len));
}

bool sd_sync(int h)
{
    return s_state == S_READY && check(fat32_sync(h));
}

bool sd_close(int h)
{
    return s_state == S_READY && check(fat32_close(h));
}

uint32_t sd_free_kb(void)
{
    if (s_state != S_READY) return 0u;
    const fat32_stat_t *f = fat32_stat();
    s_st.free_clusters = f->free_clusters;
    s_st.cluster_bytes = f->cluster_bytes;
    return fat32_free_kb();
}

const sd_stat_t *sd_stat(void)
{
    s_st.state = s_state;
    return &s_st;
}
