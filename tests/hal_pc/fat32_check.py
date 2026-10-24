"""PC test of the board's FAT32 writer (hal/tc_fat32.c) - written from the
FAT32 specification, independently of the C code:

  1. makes FAT32 disk images (MBR + partition, 2 FATs, FSInfo; 512 B and 2 KB
     clusters) with a few files already on them, like a card used on a PC
  2. runs test_fat32.exe (the C writer, the logger's way of writing)
  3. reads the image back and checks every file byte for byte, the long names,
     both FAT copies identical, no cluster used twice, FSInfo free count right,
     no long name twice (a name already on the card gets -2, -3 ...)

    python tests/hal_pc/fat32_check.py        -> "FAT32 TESTS: n passed, m failed"
"""
import os
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
EXE = os.path.join(tempfile.gettempdir(), "test_fat32.exe")
SEC = 512


def content(seed, n):
    return bytes(((i * 2654435761 + seed) & 0xFFFFFFFF) >> 13 & 0xFF for i in range(n))


def lfn_entries(name, sfn):
    s = 0
    for c in sfn:
        s = (((s & 1) << 7) + (s >> 1) + c) & 0xFF
    chars = [ord(c) for c in name] + [0]
    while len(chars) % 13:
        chars.append(0xFFFF)
    out = []
    n = len(chars) // 13
    for k in range(n, 0, -1):
        part = chars[(k - 1) * 13:k * 13]
        e = bytearray(32)
        e[0] = k | (0x40 if k == n else 0)
        e[11], e[13] = 0x0F, s
        for i, p in enumerate((1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30)):
            struct.pack_into("<H", e, p, part[i])
        out.append(bytes(e))
    return out


def make_image(path, size_mb, spc, pc_files):
    """FAT32 like a PC formats it: MBR, partition at sector 2048."""
    total = size_mb * 2048
    part, psize = 2048, total - 2048
    reserved, nfats = 32, 2
    # FAT size: clusters * 4 bytes, solved iteratively
    fat_sz = 1
    while True:
        clusters = (psize - reserved - nfats * fat_sz) // spc
        need = (clusters + 2) * 4 // SEC + 1
        if need <= fat_sz:
            break
        fat_sz = need
    assert clusters >= 65525, "too small for FAT32"
    img = bytearray(total * SEC)
    mbr = bytearray(SEC)
    struct.pack_into("<BBBBBBBBII", mbr, 0x1BE, 0, 0, 0, 0, 0x0C, 0, 0, 0, part, psize)
    mbr[510:512] = b"\x55\xAA"
    img[0:SEC] = mbr
    bs = bytearray(SEC)
    bs[0:3] = b"\xEB\x58\x90"
    bs[3:11] = b"MSWIN4.1"
    struct.pack_into("<HBHBHHBHHHII", bs, 11, SEC, spc, reserved, nfats, 0, 0, 0xF8, 0, 63, 255, part, psize)
    struct.pack_into("<IHHIHH", bs, 36, fat_sz, 0, 0, 2, 1, 6)
    bs[66] = 0x29
    bs[71:82] = b"TRYGONS    "
    bs[82:90] = b"FAT32   "
    bs[510:512] = b"\x55\xAA"
    img[part * SEC:(part + 1) * SEC] = bs
    img[(part + 6) * SEC:(part + 7) * SEC] = bs
    fat_lba = part + reserved
    data_lba = fat_lba + nfats * fat_sz
    fat = [0] * (clusters + 2)
    fat[0], fat[1], fat[2] = 0x0FFFFFF8, 0x0FFFFFFF, 0x0FFFFFFF        # cluster 2 = root folder
    root = bytearray(spc * SEC)
    nxt = 3
    slot = 0
    for name, data in pc_files:          # files a PC put there (long names, real data)
        sfn = ("PC%06d" % slot).encode() + b"TXT"
        for e in lfn_entries(name, sfn):
            root[slot * 32:slot * 32 + 32] = e
            slot += 1
        first = nxt if data else 0
        ncl = (len(data) + spc * SEC - 1) // (spc * SEC)
        for k in range(ncl):
            c = nxt + k
            fat[c] = c + 1 if k < ncl - 1 else 0x0FFFFFFF
            off = (data_lba + (c - 2) * spc) * SEC
            chunk = data[k * spc * SEC:(k + 1) * spc * SEC]
            img[off:off + len(chunk)] = chunk
        nxt += ncl
        e = bytearray(32)
        e[0:11] = sfn
        e[11] = 0x20
        struct.pack_into("<H", e, 26, first & 0xFFFF)
        struct.pack_into("<H", e, 20, first >> 16)
        struct.pack_into("<I", e, 28, len(data))
        root[slot * 32:slot * 32 + 32] = e
        slot += 1
    img[data_lba * SEC:(data_lba + spc) * SEC] = root
    fatb = b"".join(struct.pack("<I", v) for v in fat)
    for k in range(nfats):
        o = (fat_lba + k * fat_sz) * SEC
        img[o:o + len(fatb)] = fatb
    free = clusters - (nxt - 2)
    fsi = bytearray(SEC)
    struct.pack_into("<I", fsi, 0, 0x41615252)
    struct.pack_into("<III", fsi, 484, 0x61417272, free, nxt)
    fsi[510:512] = b"\x55\xAA"
    img[(part + 1) * SEC:(part + 2) * SEC] = fsi
    open(path, "wb").write(img)


def read_image(path):
    """Independent FAT32 reader -> (files {long name: data}, problems)."""
    img = open(path, "rb").read()
    probs = []
    part = struct.unpack_from("<I", img, 0x1C6)[0]
    bs = part * SEC
    spc = img[bs + 13]
    reserved = struct.unpack_from("<H", img, bs + 14)[0]
    nfats = img[bs + 16]
    psize = struct.unpack_from("<I", img, bs + 32)[0]
    fat_sz = struct.unpack_from("<I", img, bs + 36)[0]
    root = struct.unpack_from("<I", img, bs + 44)[0]
    fat_lba = part + reserved
    data_lba = fat_lba + nfats * fat_sz
    clusters = (psize - reserved - nfats * fat_sz) // spc
    fats = [img[(fat_lba + k * fat_sz) * SEC:(fat_lba + (k + 1) * fat_sz) * SEC] for k in range(nfats)]
    if any(f != fats[0] for f in fats[1:]):
        probs.append("the FAT copies differ")
    fat = [struct.unpack_from("<I", fats[0], 4 * c)[0] & 0x0FFFFFFF for c in range(clusters + 2)]

    def chain(c):
        out, seen = [], set()
        while 2 <= c < 0x0FFFFFF8:
            if c in seen or c > clusters + 1:
                probs.append("bad chain at %d" % c)
                break
            seen.add(c)
            out.append(c)
            c = fat[c]
        return out

    def data_of(cl):
        return b"".join(img[(data_lba + (c - 2) * spc) * SEC:(data_lba + (c - 1) * spc) * SEC] for c in cl)

    rootcl = chain(root)
    rootdata = data_of(rootcl)
    files, used, lfn, sfns, lfn_sum = {}, {c: "root" for c in rootcl}, {}, set(), None
    for o in range(0, len(rootdata), 32):
        e = rootdata[o:o + 32]
        if e[0] == 0:
            break
        if e[0] == 0xE5:
            lfn = {}
            continue
        if e[11] == 0x0F:
            k = e[0] & 0x3F
            chars = [struct.unpack_from("<H", e, p)[0] for p in (1, 3, 5, 7, 9, 14, 16, 18, 20, 22, 24, 28, 30)]
            lfn[k] = "".join(chr(c) for c in chars if c not in (0, 0xFFFF))
            lfn_sum = e[13]
            continue
        sfn = bytes(e[0:11])
        if sfn in sfns:
            probs.append("8.3 name used twice: %r" % sfn)
        sfns.add(sfn)
        s = 0
        for c in sfn:
            s = (((s & 1) << 7) + (s >> 1) + c) & 0xFF
        if lfn and lfn_sum != s:                  # a PC would ignore the long name
            probs.append("long-name checksum wrong for %r" % sfn)
            lfn = {}
        name = "".join(lfn[k] for k in sorted(lfn)) if lfn else sfn.decode(errors="replace")
        lfn = {}
        if name.upper() in (n.upper() for n in files):
            probs.append("name used twice: " + name)
        first = struct.unpack_from("<H", e, 20)[0] << 16 | struct.unpack_from("<H", e, 26)[0]
        size = struct.unpack_from("<I", e, 28)[0]
        cl = chain(first) if first else []
        if len(cl) != (size + spc * SEC - 1) // (spc * SEC):
            probs.append("%s: %d clusters for %d bytes" % (name, len(cl), size))
        for c in cl:
            if c in used:
                probs.append("cluster %d in %s and %s" % (c, used[c], name))
            used[c] = name
        files[name] = data_of(cl)[:size]
    # every cluster marked used in the FAT belongs to a file / the folder (no lost clusters)
    marked = [c for c in range(2, clusters + 2) if fat[c] != 0]
    lost = [c for c in marked if c not in used]
    if lost:
        probs.append("%d clusters marked used but in no file (e.g. %d)" % (len(lost), lost[0]))
    fsi = (part + 1) * SEC
    free_fsi = struct.unpack_from("<I", img, fsi + 488)[0]
    free_real = clusters - len(marked)
    if free_fsi != free_real:
        probs.append("FSInfo free count %d, really free %d" % (free_fsi, free_real))
    return files, probs, len(rootcl)


def run(case, size_mb, spc, nfiles, seed):
    img = os.path.join(tempfile.gettempdir(), "fat32_%s.img" % case)
    pc = [("Notes from the PC with a long name.txt", content(77, 5000)),
          ("empty.txt", b""), ("a.TLG", content(78, 513))]
    make_image(img, size_mb, spc, pc)
    r = subprocess.run([EXE, img, str(nfiles), str(seed)], capture_output=True, text=True)
    want, again = {}, []
    for line in r.stdout.splitlines():
        p = line.split()
        if p and p[0] == "FILE":
            want[p[1]] = (int(p[2]), int(p[3]))
        if p and p[0] == "AGAIN":
            again.append((p[1], p[2]))
    files, probs, rootcl = read_image(img)
    if r.returncode != 0:
        probs.append("writer: " + r.stdout.strip().splitlines()[-1] if r.stdout.strip() else "writer crashed")
    bad = 0
    for name, (size, s) in want.items():
        if name not in files:
            probs.append("missing: " + name)
        elif files[name] != content(s, size):
            bad += 1
            probs.append("content differs: %s" % name)
    for name, data in pc:
        if files.get(name) != data:
            probs.append("the PC's file changed: " + name)
    expect = [("a.TLG", "a-2.TLG"), ("B00012_001_20261023-1000.TLG", "B00012_001_20261023-1000-2.TLG"),
              ("B00001_001.TLG", "B00001_001.TLG"), ("B00001_001.TLG", "B00001_001-2.TLG")]
    if again != expect:
        probs.append("names already on the card: got %s" % again)
    os.remove(img)
    ok = not probs
    print("[%s] FAT32 %s (%d MB, %d B clusters): %d files written + 3 from the PC, root folder %d clusters%s" % (
        "PASS" if ok else "FAIL", case, size_mb, spc * SEC, len(want), rootcl,
        "" if ok else " - " + "; ".join(probs[:6])))
    return ok


def main():
    src = os.path.join(REPO, "hal", "tc_fat32.c")
    test = os.path.join(HERE, "test_fat32.c")
    r = subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-I" + os.path.join(REPO, "hal"), test, src,
                        "-o", EXE], capture_output=True, text=True)
    if r.returncode != 0 or r.stderr.strip():
        print("BUILD PROBLEM:\n" + r.stderr)
        if r.returncode != 0:
            return 1
    res = [run("small-clusters", 70, 1, 40, 11),         # 512 B clusters: folder grows over many clusters
           run("2k-clusters", 160, 4, 120, 12)]          # 2 KB clusters, 120 files
    print("FAT32 TESTS: %d passed, %d failed" % (sum(res), len(res) - sum(res)))
    return 0 if all(res) else 1


if __name__ == "__main__":
    sys.exit(main())
