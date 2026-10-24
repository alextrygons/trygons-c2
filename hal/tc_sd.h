/*
 * tc_sd.h - microSD card over SPI (QSPI2, microSD click) with a small FAT32
 * writer: files in the root folder, long file names, create / append / sync /
 * close, free space from FSInfo. Used ONLY by CPU2 (the logger).
 *
 * Every call does bounded work (card start-up and mounting are a state
 * machine: sd_service(budget)); a card that doesn't answer in time = the
 * call fails (LOG FAILURE), it never blocks longer than HAL_SD_MAX_MS.
 * Not supported (not needed by the logger): sub-folders, reading files,
 * overwriting (a name already on the card gets "-2", "-3", ... - e.g. the
 * same boot number again while the data flash is not used yet).
 */
#ifndef TC_SD_H
#define TC_SD_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool     card;                  /* card detect: inserted */
    bool     mounted;               /* FAT32 found, ready for files */
    bool     sdhc;                  /* block addressing */
    uint8_t  state;                 /* mount state (0 no card .. 8 ready, 9 failed) */
    uint32_t errors, timeouts, mounts;
    uint32_t free_clusters, cluster_bytes;
} sd_stat_t;

void sd_service(uint32_t budget_ms);    /* card start-up / mount steps (CPU2) */
bool sd_present(void);
int  sd_open(const char *name);
bool sd_write(int h, const void *data, uint32_t len);
bool sd_sync(int h);
bool sd_close(int h);
uint32_t sd_free_kb(void);
const sd_stat_t *sd_stat(void);

#endif /* TC_SD_H */
