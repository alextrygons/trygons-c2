/*
 * tc_fat32.h - A small FAT32 writer on top of a block device (512-byte
 * sectors): files in the root folder, long file names, create / append /
 * sync / close, free space from FSInfo. Pure C99 (no hardware): the board
 * uses it on the microSD card (tc_sd.c); tests/hal_pc/test_fat32.c runs it on
 * a disk image on the PC and checks the result with an independent reader.
 * Every call does bounded work (FAT search / folder scan in limited steps).
 * Not supported (the logger doesn't need it): sub-folders, reading files,
 * overwriting, deleting. A name already on the card gets "-2", "-3", ...
 * before the extension (fat32_last_name() = the name really used).
 */
#ifndef TC_FAT32_H
#define TC_FAT32_H

#include <stdbool.h>
#include <stdint.h>

#define FAT32_FILES 2

typedef bool (*fat32_block_fn)(uint32_t lba, uint8_t *buf);

typedef struct {
    uint32_t free_clusters, cluster_bytes;
    bool     free_known;
    uint32_t errors;
} fat32_stat_t;

/* Mount: false = no FAT32 file system found / read error. */
bool fat32_mount(fat32_block_fn rd, fat32_block_fn wr);
/* Find the end of the root folder in steps: 1 = done, 0 = more steps, -1 = error. */
int  fat32_scan_step(void);
void fat32_unmount(void);
int  fat32_open(const char *name);              /* -1 = error */
const char *fat32_last_name(void);              /* the name the last open really used */
bool fat32_write(int h, const void *data, uint32_t len);
bool fat32_sync(int h);
bool fat32_close(int h);
uint32_t fat32_free_kb(void);                   /* 0x7FFFFFFF = not known */
const fat32_stat_t *fat32_stat(void);

#endif /* TC_FAT32_H */
