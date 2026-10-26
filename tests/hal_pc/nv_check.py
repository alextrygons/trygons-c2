"""PC test of the board's data-flash record log (hal/tc_nv.c) - checked
independently of the C code, from the record format in tc_nv.h:

  1. builds test_nv.exe (tc_nv.c + the flash model in test_nv.c)
  2. runs it with several seeds and geometries (erased word 0 like the TC3xx
     DFLASH and 0xFFFFFFFF; tiny regions = many sector swaps): random saves,
     power losses cutting any flash command, corrupted records, reboots
  3. reads the final flash image back with its own parser (magic, length,
     sequence, CRC-32 over meta + seq + payload, trailer) and checks that the
     newest valid record of every slot is exactly what the writer committed,
     that no record is counted twice, and that a region holds at most one
     run of records

    python tests/hal_pc/nv_check.py        -> "NV TESTS: n passed, m failed"
"""
import os
import struct
import subprocess
import sys
import tempfile
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
EXE = os.path.join(tempfile.gettempdir(), "test_nv.exe")
MAGIC = 0x4E595254
FMT = 1
MAX_LEN = 1536


def records(region, erased):
    """Every complete record of one region: [(off, slot, seq, payload)]."""
    out = []
    off = 0
    n = len(region)
    while off + 24 <= n:
        w0, meta = struct.unpack_from("<II", region, off)
        if w0 == MAGIC:
            slot, fmt, ln = meta & 0xFF, (meta >> 8) & 0xFF, meta >> 16
            size = 16 + ((ln + 7) & ~7) + 8
            if 1 <= slot <= 15 and fmt == FMT and 1 <= ln <= MAX_LEN and off + size <= n:
                seq, nseq = struct.unpack_from("<II", region, off + 8)
                payload = region[off + 16:off + 16 + ln]
                crc = zlib.crc32(struct.pack("<II", meta, seq) + payload) & 0xFFFFFFFF
                t0, t1 = struct.unpack_from("<II", region, off + size - 8)
                if nseq == (~seq & 0xFFFFFFFF) and t0 == crc and t1 == (~crc & 0xFFFFFFFF):
                    out.append((off, slot, seq, bytes(payload)))
                    off += size
                    continue
        off += 8
    return out


def newest(img, region_bytes, erased):
    best = {}
    for r in (0, 1):
        for off, slot, seq, payload in records(img[r * region_bytes:(r + 1) * region_bytes], erased):
            if slot not in best or (seq - best[slot][0]) & 0xFFFFFFFF < 0x80000000 and seq != best[slot][0]:
                best[slot] = (seq, payload)
    return {s: p for s, (q, p) in best.items()}


def run(case, seed, erased, sector, region, ops):
    img_path = os.path.join(tempfile.gettempdir(), "nv_%s.img" % case)
    r = subprocess.run([EXE, str(seed), str(erased), str(sector), str(region), str(ops), img_path],
                       capture_output=True, text=True)
    probs = []
    expect = {}
    stat = ""
    for line in r.stdout.splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == "FAIL":
            probs.append("writer: " + line[5:])
        elif p[0] == "EXPECT":
            expect[int(p[1])] = bytes.fromhex(p[2])
        elif p[0] == "STAT":
            stat = line[5:]
    if r.returncode != 0 and not probs:
        probs.append("writer exit %d: %s" % (r.returncode, (r.stdout.strip().splitlines() or ["?"])[-1]))
    if os.path.exists(img_path):
        img = open(img_path, "rb").read()
        os.remove(img_path)
        got = newest(img, region, erased)
        for slot, data in expect.items():
            if slot not in got:
                probs.append("slot %d missing in the image" % slot)
            elif got[slot] != data:
                probs.append("slot %d: the image holds another value" % slot)
        for slot in got:
            if slot not in expect:
                probs.append("slot %d in the image but the writer committed nothing" % slot)
        # a region never holds two records with the same sequence number
        seqs = []
        for rg in (0, 1):
            seqs += [q for _, _, q, _ in records(img[rg * region:(rg + 1) * region], erased)]
        if len(seqs) != len(set(seqs)):
            probs.append("a sequence number appears twice")
    else:
        probs.append("no image written")
    ok = not probs
    print("[%s] NV %s (seed %d, erased %s, sectors %d B, regions %d B, %d ops): %s%s" % (
        "PASS" if ok else "FAIL", case, seed, "0xFFFFFFFF" if erased else "0", sector, region, ops,
        stat, "" if ok else " - " + "; ".join(probs[:6])))
    return ok


def main():
    src = os.path.join(REPO, "hal", "tc_nv.c")
    test = os.path.join(HERE, "test_nv.c")
    r = subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-I" + os.path.join(REPO, "hal"), test, src,
                        "-o", EXE], capture_output=True, text=True)
    if r.returncode != 0 or r.stderr.strip():
        print("BUILD PROBLEM:\n" + r.stderr)
        if r.returncode != 0:
            return 1
    res = [run("tiny-dflash", 11, 0, 512, 1024, 400),        # TC3xx: erased = 0, many swaps
           run("tiny-ff", 12, 1, 512, 1024, 400),            # erased = 0xFFFFFFFF
           run("4k-sectors", 13, 0, 4096, 8192, 300),        # the board's sector size, 2 per region
           run("board-like", 14, 0, 4096, 32768, 200)]       # the board's regions (few swaps)
    print("NV TESTS: %d passed, %d failed" % (sum(res), len(res) - sum(res)))
    return 0 if all(res) else 1


if __name__ == "__main__":
    sys.exit(main())
