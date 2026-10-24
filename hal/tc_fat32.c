/*
 * tc_fat32.c - A small FAT32 writer on a block device. See tc_fat32.h.
 *
 * Mount: MBR (or none) -> boot sector -> FSInfo; the root folder is scanned
 * once for its end (new entries are appended there, never in old gaps).
 * A new file = long-name entries + an 8.3 alias "TLxxxxxx.EXT". File data: a
 * 512-byte buffer per file; full sectors are written at once, the partial one
 * on every sync (and again until it is full).
 * Names already on the card: the scan keeps a 16-bit hash of every long name;
 * opening a name whose hash is known adds "-2", "-3", ... before the extension
 * (a hash collision only adds a needless suffix), so an old log is never hidden
 * behind a second entry with the same name (e.g. boot number not stored).
 */
#include "tc_fat32.h"

#include <string.h>

#define SEC          512u
#define EOC          0x0FFFFFF8u
#define FAT_SCAN_SEC 32u            /* FAT sectors searched per allocation (4096 clusters) */
#define DIR_SCAN_SEC 16u            /* root folder sectors per scan step */
#define NAME_TABLE   2048u          /* long names remembered (more: no duplicate check) */

typedef struct {
    bool     used;
    uint32_t first, cur, cur_index; /* first cluster, the cluster of the buffer, its index in the file */
    uint32_t size;                  /* bytes in the file (incl. the buffer) */
    uint32_t fill;                  /* bytes in buf */
    uint32_t dir_lba;               /* the 8.3 entry: sector and offset */
    uint16_t dir_off;
    uint8_t  buf[SEC];
} fatfile_t;

static fat32_block_fn s_rd, s_wr;
static fat32_stat_t s_st;
static bool s_ready;
/* geometry */
static uint32_t s_part, s_fat_lba, s_fat_sz, s_data_lba, s_root, s_fsinfo_lba, s_max_cluster, s_next_free;
static uint8_t s_spc, s_nfats;
/* root folder end (where new entries go) */
static uint32_t s_dir_cluster, s_dir_entry;     /* entry index within that cluster */
static uint32_t s_dir_ordinal;                  /* entries in the whole root folder before the end (8.3 aliases) */
static uint32_t s_scan_cluster, s_scan_sector;
/* buffers */
static uint8_t s_sec[SEC], s_fat[SEC];
static uint32_t s_fat_cached = 0xFFFFFFFFu;
static bool s_fat_dirty;
static fatfile_t s_f[FAT32_FILES];
/* long names on the card (hashes) */
static uint16_t s_names[NAME_TABLE];
static uint32_t s_nnames, s_lfn_acc;
static bool s_names_full, s_lfn_have;
static char s_last_name[128];

static bool read_block(uint32_t lba, uint8_t *buf)
{
    if (s_rd(lba, buf)) return true;
    s_st.errors++;
    return false;
}

static bool write_block(uint32_t lba, const uint8_t *buf)
{
    if (s_wr(lba, (uint8_t *)buf)) return true;
    s_st.errors++;
    return false;
}

/* --- FAT ---------------------------------------------------------------------- */
static uint32_t le32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void put32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

static uint32_t cluster_lba(uint32_t c) { return s_data_lba + (c - 2u) * s_spc; }

static bool fat_flush(void)
{
    if (!s_fat_dirty) return true;
    for (uint8_t i = 0; i < s_nfats; i++)
        if (!write_block(s_fat_cached + i * s_fat_sz, s_fat)) return false;
    s_fat_dirty = false;
    return true;
}

static bool fat_load(uint32_t c)
{
    uint32_t sec = s_fat_lba + (c * 4u) / SEC;
    if (sec == s_fat_cached) return true;
    if (!fat_flush()) return false;
    if (!read_block(sec, s_fat)) { s_fat_cached = 0xFFFFFFFFu; return false; }
    s_fat_cached = sec;
    return true;
}

static bool fat_get(uint32_t c, uint32_t *v)
{
    if (!fat_load(c)) return false;
    *v = le32(&s_fat[(c * 4u) % SEC]) & 0x0FFFFFFFu;
    return true;
}

static bool fat_set(uint32_t c, uint32_t v)
{
    if (!fat_load(c)) return false;
    uint8_t *p = &s_fat[(c * 4u) % SEC];
    put32(p, (le32(p) & 0xF0000000u) | (v & 0x0FFFFFFFu));
    s_fat_dirty = true;
    return true;
}

/* A free cluster, chained after prev (0 = none); 0 = none found this time. */
static uint32_t alloc_cluster(uint32_t prev)
{
    uint32_t c = (s_next_free >= 2u && s_next_free <= s_max_cluster) ? s_next_free : 2u;
    for (uint32_t n = 0; n < FAT_SCAN_SEC * (SEC / 4u); n++) {
        uint32_t v;
        if (!fat_get(c, &v)) return 0u;
        if (v == 0u) {
            if (!fat_set(c, 0x0FFFFFFFu)) return 0u;
            if (prev && !fat_set(prev, c)) return 0u;
            s_next_free = c + 1u;
            if (s_st.free_known && s_st.free_clusters) s_st.free_clusters--;
            return c;
        }
        if (++c > s_max_cluster) c = 2u;
    }
    s_st.errors++;                                              /* full, or not found in this window */
    return 0u;
}

bool fat32_mount(fat32_block_fn rd, fat32_block_fn wr)
{
    s_rd = rd;
    s_wr = wr;
    s_ready = false;
    memset(s_f, 0, sizeof s_f);
    if (!read_block(0u, s_sec)) return false;
    if (le16(&s_sec[510]) != 0xAA55u) return false;
    s_part = 0u;
    if (!(s_sec[0] == 0xEB || s_sec[0] == 0xE9)) {              /* MBR: first FAT32 partition */
        uint8_t type = s_sec[0x1C2];
        if (type != 0x0B && type != 0x0C) return false;
        s_part = le32(&s_sec[0x1C6]);
        if (!read_block(s_part, s_sec)) return false;
    }
    if (le16(&s_sec[11]) != SEC || le16(&s_sec[22]) != 0u) return false;    /* 512 B sectors, FAT32 */
    s_spc = s_sec[13];
    s_nfats = s_sec[16];
    s_fat_sz = le32(&s_sec[36]);
    s_fat_lba = s_part + le16(&s_sec[14]);
    s_data_lba = s_fat_lba + s_nfats * s_fat_sz;
    s_root = le32(&s_sec[44]);
    s_fsinfo_lba = s_part + le16(&s_sec[48]);
    uint32_t total = le32(&s_sec[32]);
    if (s_spc == 0u || s_nfats == 0u || s_root < 2u) return false;
    s_max_cluster = (total - (s_data_lba - s_part)) / s_spc + 1u;
    s_st.cluster_bytes = (uint32_t)s_spc * SEC;
    s_st.free_known = false;
    s_next_free = 2u;
    if (read_block(s_fsinfo_lba, s_sec) && le32(&s_sec[0]) == 0x41615252u && le32(&s_sec[484]) == 0x61417272u) {
        uint32_t fc = le32(&s_sec[488]), nf = le32(&s_sec[492]);
        if (fc != 0xFFFFFFFFu && fc <= s_max_cluster) { s_st.free_clusters = fc; s_st.free_known = true; }
        if (nf >= 2u && nf <= s_max_cluster) s_next_free = nf;
    }
    s_fat_cached = 0xFFFFFFFFu;
    s_fat_dirty = false;
    s_scan_cluster = s_root;
    s_scan_sector = 0u;
    s_dir_ordinal = 0u;
    s_nnames = 0u;
    s_names_full = false;
    s_lfn_acc = 0u;
    s_lfn_have = false;
    return true;
}

/* --- long-name hashes (order of the parts doesn't matter: a sum) ------------------- */
static uint32_t name_mix(uint32_t k, uint16_t c)
{
    if (c >= 'a' && c <= 'z') c = (uint16_t)(c - 32u);             /* FAT names ignore case */
    uint32_t x = (k << 16) ^ c ^ 0x5BD1E995u;
    x *= 0x9E3779B1u;
    x ^= x >> 15;
    x *= 0x85EBCA77u;
    x ^= x >> 13;
    return x;
}

static uint16_t name_fold(uint32_t acc) { return (uint16_t)(acc ^ (acc >> 16)); }

static uint16_t name_hash(const char *name)
{
    uint32_t acc = 0u;
    for (uint32_t k = 0; name[k]; k++) acc += name_mix(k, (uint8_t)name[k]);
    return name_fold(acc);
}

static bool name_known(uint16_t h)
{
    for (uint32_t i = 0; i < s_nnames; i++) if (s_names[i] == h) return true;
    return false;
}

static void name_add(uint16_t h)
{
    if (s_nnames < NAME_TABLE) s_names[s_nnames++] = h;
    else s_names_full = true;
}

/* one folder entry seen by the scan */
static void scan_entry(const uint8_t *e)
{
    static const uint8_t pos[13] = { 1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30 };
    if (e[0] == 0xE5) { s_lfn_acc = 0u; s_lfn_have = false; return; }          /* deleted */
    if (e[11] == 0x0F) {                                                        /* a long-name part */
        uint32_t ord = e[0] & 0x3Fu;
        for (uint32_t i = 0; ord >= 1u && i < 13u; i++) {
            uint16_t c = le16(&e[pos[i]]);
            if (c != 0x0000u && c != 0xFFFFu) s_lfn_acc += name_mix((ord - 1u) * 13u + i, c);
        }
        s_lfn_have = true;
        return;
    }
    if (s_lfn_have && !(e[11] & 0x08u)) name_add(name_fold(s_lfn_acc));        /* a file / folder, not the label */
    s_lfn_acc = 0u;
    s_lfn_have = false;
}

/* Root folder: find the first never-used entry (name[0] == 0). */
int fat32_scan_step(void)
{
    for (uint32_t n = 0; n < DIR_SCAN_SEC; n++) {
        if (!read_block(cluster_lba(s_scan_cluster) + s_scan_sector, s_sec)) return -1;
        for (uint32_t e = 0; e < SEC / 32u; e++) {
            if (s_sec[e * 32u] == 0x00) {
                s_dir_cluster = s_scan_cluster;
                s_dir_entry = s_scan_sector * (SEC / 32u) + e;
                s_ready = true;
                return 1;
            }
            scan_entry(&s_sec[e * 32u]);
            s_dir_ordinal++;
        }
        if (++s_scan_sector >= s_spc) {
            uint32_t next;
            if (!fat_get(s_scan_cluster, &next)) return -1;
            if (next >= EOC || next < 2u) {                     /* full to the end: next entry needs a cluster */
                s_dir_cluster = s_scan_cluster;
                s_dir_entry = (uint32_t)s_spc * (SEC / 32u);
                s_ready = true;
                return 1;
            }
            s_scan_cluster = next;
            s_scan_sector = 0u;
        }
    }
    return 0;                                                   /* more next time */
}

void fat32_unmount(void)
{
    memset(s_f, 0, sizeof s_f);
    s_ready = false;
    s_fat_cached = 0xFFFFFFFFu;
    s_fat_dirty = false;
}

/* --- folder entries ---------------------------------------------------------------- */
static uint8_t lfn_sum(const uint8_t *sfn)
{
    uint8_t s = 0;
    for (int i = 0; i < 11; i++) s = (uint8_t)(((s & 1u) << 7) + (s >> 1) + sfn[i]);
    return s;
}

static void lfn_entry(uint8_t *e, const char *name, uint32_t len, uint32_t ord, bool last, uint8_t sum)
{
    static const uint8_t pos[13] = { 1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30 };
    memset(e, 0, 32u);
    e[0] = (uint8_t)(ord | (last ? 0x40u : 0u));
    e[11] = 0x0F;
    e[13] = sum;
    for (uint32_t i = 0; i < 13u; i++) {
        uint32_t k = (ord - 1u) * 13u + i;
        uint16_t ch = (k < len) ? (uint8_t)name[k] : (k == len ? 0x0000u : 0xFFFFu);
        put16(&e[pos[i]], ch);
    }
}

/* Write one 32-byte entry at the folder end (allocating a folder cluster if needed). */
static bool put_entry(const uint8_t *e, uint32_t *lba_out, uint16_t *off_out)
{
    uint32_t per_cluster = (uint32_t)s_spc * (SEC / 32u);
    if (s_dir_entry >= per_cluster) {                           /* folder full: one more cluster, zeroed */
        uint32_t c = alloc_cluster(s_dir_cluster);
        if (c == 0u || !fat_flush()) return false;
        memset(s_sec, 0, SEC);
        for (uint8_t i = 0; i < s_spc; i++) if (!write_block(cluster_lba(c) + i, s_sec)) return false;
        s_dir_cluster = c;
        s_dir_entry = 0u;
    }
    uint32_t lba = cluster_lba(s_dir_cluster) + s_dir_entry / (SEC / 32u);
    uint16_t off = (uint16_t)((s_dir_entry % (SEC / 32u)) * 32u);
    if (!read_block(lba, s_sec)) return false;
    memcpy(&s_sec[off], e, 32u);
    if (!write_block(lba, s_sec)) return false;
    if (lba_out) *lba_out = lba;
    if (off_out) *off_out = off;
    s_dir_entry++;
    s_dir_ordinal++;
    return true;
}

/* name, or name-2 / -3 ... if it is already on the card -> s_last_name */
static void unique_name(const char *name, uint32_t len)
{
    memcpy(s_last_name, name, len + 1u);
    const char *dot = strrchr(name, '.');
    uint32_t stem = dot ? (uint32_t)(dot - name) : len;
    for (uint32_t k = 2u; k <= 99u && name_known(name_hash(s_last_name)); k++) {
        uint32_t o = stem;
        s_last_name[o++] = '-';
        if (k >= 10u) s_last_name[o++] = (char)('0' + k / 10u);
        s_last_name[o++] = (char)('0' + k % 10u);
        memcpy(&s_last_name[o], name + stem, len - stem + 1u);
    }
}

int fat32_open(const char *name_in)
{
    if (!s_ready) return -1;
    int h = -1;
    for (int i = 0; i < FAT32_FILES; i++) if (!s_f[i].used) { h = i; break; }
    uint32_t len = (uint32_t)strlen(name_in);
    if (h < 0 || len == 0u || len > 120u) return -1;
    unique_name(name_in, len);
    const char *name = s_last_name;
    len = (uint32_t)strlen(name);
    /* 8.3 alias: TL + 6 hex digits of the entry position, the long name's extension */
    uint8_t sfn[11];
    static const char hx[] = "0123456789ABCDEF";
    uint32_t tag = s_dir_ordinal & 0xFFFFFFu;            /* unique: entries only ever get added */
    memset(sfn, ' ', sizeof sfn);
    sfn[0] = 'T'; sfn[1] = 'L';
    for (int i = 0; i < 6; i++) sfn[2 + i] = (uint8_t)hx[(tag >> (20 - 4 * i)) & 0xFu];
    const char *dot = strrchr(name, '.');
    for (int i = 0; dot && i < 3 && dot[1 + i]; i++) {
        char c = dot[1 + i];
        sfn[8 + i] = (uint8_t)((c >= 'a' && c <= 'z') ? c - 32 : c);
    }
    uint8_t sum = lfn_sum(sfn), e[32];
    uint32_t n = (len + 12u) / 13u;
    for (uint32_t ord = n; ord >= 1u; ord--) {                  /* long-name entries, last part first */
        lfn_entry(e, name, len, ord, ord == n, sum);
        if (!put_entry(e, NULL, NULL)) return -1;
    }
    memset(e, 0, sizeof e);
    memcpy(e, sfn, 11u);
    e[11] = 0x20;                                               /* archive */
    put16(&e[16], (uint16_t)(((2026 - 1980) << 9) | (1 << 5) | 1));    /* date 2026-01-01 (no RTC) */
    put16(&e[24], (uint16_t)(((2026 - 1980) << 9) | (1 << 5) | 1));
    fatfile_t *f = &s_f[h];
    memset(f, 0, sizeof *f);
    if (!put_entry(e, &f->dir_lba, &f->dir_off)) return -1;
    f->used = true;
    name_add(name_hash(name));
    return h;
}

const char *fat32_last_name(void)
{
    return s_last_name;
}

/* --- file data ----------------------------------------------------------------------- */
static bool ensure_cluster(fatfile_t *f, uint32_t base)
{
    uint32_t ci = (base / SEC) / s_spc;
    if (f->first == 0u) {
        f->first = f->cur = alloc_cluster(0u);
        f->cur_index = 0u;
        if (f->first == 0u) return false;
    }
    while (f->cur_index < ci) {
        uint32_t next;
        if (!fat_get(f->cur, &next)) return false;
        if (next >= EOC || next < 2u) next = alloc_cluster(f->cur);
        if (next == 0u) return false;
        f->cur = next;
        f->cur_index++;
    }
    return true;
}

static bool write_buffer(fatfile_t *f)
{
    uint32_t base = f->size - f->fill;                          /* file offset of buf[0] (a sector start) */
    if (!ensure_cluster(f, base)) return false;
    return write_block(cluster_lba(f->cur) + (base / SEC) % s_spc, f->buf);
}

bool fat32_write(int h, const void *data, uint32_t len)
{
    if (h < 0 || h >= FAT32_FILES || !s_f[h].used || !s_ready) return false;
    fatfile_t *f = &s_f[h];
    const uint8_t *p = (const uint8_t *)data;
    while (len > 0u) {
        uint32_t k = SEC - f->fill;
        if (k > len) k = len;
        memcpy(&f->buf[f->fill], p, k);
        f->fill += k;
        f->size += k;
        p += k;
        len -= k;
        if (f->fill == SEC) {
            if (!write_buffer(f)) return false;
            f->fill = 0u;
        }
    }
    return true;
}

bool fat32_sync(int h)
{
    if (h < 0 || h >= FAT32_FILES || !s_f[h].used || !s_ready) return false;
    fatfile_t *f = &s_f[h];
    if (f->fill > 0u && !write_buffer(f)) return false;         /* the partial sector (again later) */
    if (!fat_flush()) return false;
    if (!read_block(f->dir_lba, s_sec)) return false;           /* size + first cluster in the entry */
    put16(&s_sec[f->dir_off + 20u], (uint16_t)(f->first >> 16));
    put16(&s_sec[f->dir_off + 26u], (uint16_t)f->first);
    put32(&s_sec[f->dir_off + 28u], f->size);
    if (!write_block(f->dir_lba, s_sec)) return false;
    if (s_st.free_known && read_block(s_fsinfo_lba, s_sec)) {      /* FSInfo: free count, next free hint */
        put32(&s_sec[488], s_st.free_clusters);
        put32(&s_sec[492], s_next_free);
        (void)write_block(s_fsinfo_lba, s_sec);
    }
    return true;
}

bool fat32_close(int h)
{
    if (h < 0 || h >= FAT32_FILES || !s_f[h].used) return false;
    bool ok = fat32_sync(h);
    s_f[h].used = false;
    return ok;
}

uint32_t fat32_free_kb(void)
{
    if (!s_ready) return 0u;
    if (!s_st.free_known) return 0x7FFFFFFFu;                      /* not known: don't report "full" */
    return (uint32_t)(((uint64_t)s_st.free_clusters * s_st.cluster_bytes) / 1024u);
}

const fat32_stat_t *fat32_stat(void)
{
    return &s_st;
}
