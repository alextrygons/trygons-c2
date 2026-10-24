/*
 * test_fat32.c - Runs the board's FAT32 writer (hal/tc_fat32.c) on a disk
 * image on the PC, the way the logger uses it: two files open at the same
 * time, random chunk sizes, syncs, many files (folder grows past a cluster),
 * names that are already on the card (the writer adds -2, -3 ...).
 * Driven and checked by fat32_check.py (an independent reader).
 *
 *   test_fat32 IMAGE NFILES SEED   -> prints "FILE name size seed" per file
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tc_fat32.h"

static FILE *s_img;
static unsigned long s_reads, s_writes;

static bool rd(uint32_t lba, uint8_t *buf)
{
    s_reads++;
    return fseek(s_img, (long)lba * 512L, SEEK_SET) == 0 && fread(buf, 1, 512, s_img) == 512;
}

static bool wr(uint32_t lba, uint8_t *buf)
{
    s_writes++;
    return fseek(s_img, (long)lba * 512L, SEEK_SET) == 0 && fwrite(buf, 1, 512, s_img) == 512;
}

/* the same content generator as fat32_check.py */
static uint8_t content(uint32_t seed, uint32_t i)
{
    uint32_t v = i * 2654435761u + seed;
    return (uint8_t)((v >> 13) & 0xFFu);
}

static uint32_t rng = 1u;
static uint32_t rnd(uint32_t n) { rng = rng * 1664525u + 1013904223u; return (rng >> 8) % n; }

int main(int argc, char **argv)
{
    if (argc < 4) { fprintf(stderr, "usage: test_fat32 IMAGE NFILES SEED\n"); return 2; }
    s_img = fopen(argv[1], "r+b");
    if (!s_img) { perror(argv[1]); return 2; }
    int nfiles = atoi(argv[2]);
    rng = (uint32_t)atoi(argv[3]);
    if (!fat32_mount(rd, wr)) { printf("MOUNT FAILED\n"); return 1; }
    int r, steps = 0;
    while ((r = fat32_scan_step()) == 0) steps++;
    if (r < 0) { printf("SCAN FAILED\n"); return 1; }
    printf("MOUNTED scan_steps %d free_kb %lu\n", steps, (unsigned long)fat32_free_kb());
    static uint8_t chunk[5000];
    for (int f = 0; f < nfiles; f += 2) {
        /* a log file and (every other time) a black-box file at the same time */
        char n1[64], n2[64];
        snprintf(n1, sizeof n1, "B%05d_%03d_20261023-%04d.TLG", 12 + f, f + 1, 1000 + f);
        snprintf(n2, sizeof n2, "BB%05d_%03d.TLG", 12 + f, f + 2);
        uint32_t s1 = 1000u + (uint32_t)f, s2 = 5000u + (uint32_t)f;
        static char names[2][128];
        int h1 = fat32_open(n1);
        snprintf(names[0], sizeof names[0], "%s", fat32_last_name());
        int h2 = (f + 1 < nfiles) ? fat32_open(n2) : -1;
        snprintf(names[1], sizeof names[1], "%s", fat32_last_name());
        if (h1 < 0 || (f + 1 < nfiles && h2 < 0)) { printf("OPEN FAILED %d\n", f); return 1; }
        uint32_t want1 = rnd(60000u) + (f == 0 ? 3u : 1u), want2 = rnd(30000u) + 1u, o1 = 0, o2 = 0;
        if (f % 6 == 4) want1 = 0u;                                     /* an empty file too */
        while (o1 < want1 || (h2 >= 0 && o2 < want2)) {
            if (o1 < want1) {
                uint32_t k = rnd(4096u) + 1u;
                if (k > want1 - o1) k = want1 - o1;
                for (uint32_t i = 0; i < k; i++) chunk[i] = content(s1, o1 + i);
                if (!fat32_write(h1, chunk, k)) { printf("WRITE FAILED\n"); return 1; }
                o1 += k;
            }
            if (h2 >= 0 && o2 < want2) {
                uint32_t k = rnd(700u) + 1u;
                if (k > want2 - o2) k = want2 - o2;
                for (uint32_t i = 0; i < k; i++) chunk[i] = content(s2, o2 + i);
                if (!fat32_write(h2, chunk, k)) { printf("WRITE FAILED\n"); return 1; }
                o2 += k;
            }
            if (rnd(4u) == 0u && !fat32_sync(h1)) { printf("SYNC FAILED\n"); return 1; }
        }
        if (!fat32_close(h1)) { printf("CLOSE FAILED\n"); return 1; }
        printf("FILE %s %lu %lu\n", names[0], (unsigned long)want1, (unsigned long)s1);
        if (h2 >= 0) {
            if (!fat32_close(h2)) { printf("CLOSE FAILED\n"); return 1; }
            printf("FILE %s %lu %lu\n", names[1], (unsigned long)want2, (unsigned long)s2);
        }
    }
    /* names already on the card: a PC file, one written above, the same again after a remount
     * (= the next power-up without a stored boot number) */
    static const char *again[] = { "a.TLG", "B00012_001_20261023-1000.TLG", "B00001_001.TLG", "B00001_001.TLG" };
    for (int i = 0; i < 4; i++) {
        if (i == 3) {
            if (!fat32_mount(rd, wr)) { printf("REMOUNT FAILED\n"); return 1; }
            while ((r = fat32_scan_step()) == 0) { }
            if (r < 0) { printf("RESCAN FAILED\n"); return 1; }
        }
        int h = fat32_open(again[i]);
        if (h < 0) { printf("OPEN FAILED %s\n", again[i]); return 1; }
        uint32_t s = 9000u + (uint32_t)i, want = 700u + 100u * (uint32_t)i;
        for (uint32_t k = 0; k < want; k++) chunk[k] = content(s, k);
        if (!fat32_write(h, chunk, want) || !fat32_close(h)) { printf("WRITE FAILED\n"); return 1; }
        printf("FILE %s %lu %lu\n", fat32_last_name(), (unsigned long)want, (unsigned long)s);
        printf("AGAIN %s %s\n", again[i], fat32_last_name());
    }
    printf("DONE reads %lu writes %lu free_kb %lu errors %lu\n", s_reads, s_writes, (unsigned long)fat32_free_kb(),
           (unsigned long)fat32_stat()->errors);
    fclose(s_img);
    return 0;
}
