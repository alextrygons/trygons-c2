"""PC test of the board's DroneCAN layer (hal/dronecan.c + hal/libcanard, step 11d),
checked independently of the C code: this file has its OWN DroneCAN encoder written
from the DSDL definitions and the transport rules (it never calls libcanard):

  - data type signatures: CRC-64-WE of the normalized DSDL definition, extended by
    the nested types' signatures (the DroneCAN rule) - compared with dronecan.h;
  - payloads: bit-packed fields (multi-byte values little-endian, sub-byte fields
    from the most significant bit), float16 (IEEE half, Python's struct 'e'),
    tail-array optimisation of the last dynamic array;
  - frames: 29-bit message / service identifiers, tail byte (start / end / toggle /
    transfer ID), multi-frame transfers with the CRC-16-CCITT seeded with the
    signature;
  - the driver state machine on a fake controller: bus-off + recovery, error
    passive, the 100 ms bus-fault limit, a full TX FIFO, stale frames dropped,
    bounded RX per step, the node table rules (silent / never / not OPERATIONAL /
    ERROR / WARNING = notice / reboot latch + acknowledge), our own node ID on the
    bus, a node asking for a dynamic ID, the GetNodeInfo server.

    python tests/hal_pc/dronecan_check.py   -> "DRONECAN TESTS: n passed, m failed"

(The reference frames are the DroneCAN specification's, built here; if pydronecan
is installed nothing more is needed - this check does not depend on it.)
"""
import os
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
EXE = os.path.join(tempfile.gettempdir(), "test_dronecan.exe")

# --------------------------------------------------------------- the reference ----
M64, POLY64 = 0xFFFFFFFFFFFFFFFF, 0x42F0E1EBA9EA3693


def crc64we(data, ext=None):
    c = (ext ^ M64) if ext is not None else M64
    for b in data:
        c ^= (b << 56) & M64
        for _ in range(8):
            c = ((c << 1) & M64) ^ POLY64 if c & (1 << 63) else (c << 1) & M64
    return c ^ M64


def dsdl_sig(name, fields, nested=(), service_split=None):
    """Data type signature: CRC-64-WE of 'full.name\\nfield\\nfield...' (normalized, fields only, no
    constants), then extended with each nested compound type's signature."""
    if service_split is None:
        text = name + "\n" + "\n".join(fields)
    else:                                   # service: request fields, '---', response fields
        req, resp = service_split
        text = name + "\n" + "\n".join(req) + ("\n" if req else "") + "---\n" + "\n".join(resp)
    s = crc64we(text.encode())
    for n in nested:
        s = crc64we(struct.pack("<Q", n) + struct.pack("<Q", s), s)
    return s


assert crc64we(b"123456789") == 0x62EC59E3F1A4F00A       # the CRC-64-WE check value
SIG_STATUS = dsdl_sig("uavcan.protocol.NodeStatus", [
    "saturated uint32 uptime_sec", "saturated uint2 health", "saturated uint3 mode", "saturated uint3 sub_mode",
    "saturated uint16 vendor_specific_status_code"])
SIG_CMD = dsdl_sig("uavcan.equipment.actuator.Command", [
    "saturated uint8 actuator_id", "saturated uint8 command_type", "saturated float16 command_value"])
SIG_ARRAYCMD = dsdl_sig("uavcan.equipment.actuator.ArrayCommand",
                        ["uavcan.equipment.actuator.Command[<=15] commands"], [SIG_CMD])
SIG_HARDPOINT = dsdl_sig("uavcan.equipment.hardpoint.Command", ["saturated uint8 hardpoint_id",
                                                                "saturated uint16 command"])
SIG_SW = dsdl_sig("uavcan.protocol.SoftwareVersion", [
    "saturated uint8 major", "saturated uint8 minor", "saturated uint8 optional_field_flags",
    "saturated uint32 vcs_commit", "saturated uint64 image_crc"])
SIG_HW = dsdl_sig("uavcan.protocol.HardwareVersion", [
    "saturated uint8 major", "saturated uint8 minor", "saturated uint8[16] unique_id",
    "saturated uint8[<=255] certificate_of_authenticity"])
SIG_NODEINFO = dsdl_sig("uavcan.protocol.GetNodeInfo", None, [SIG_STATUS, SIG_SW, SIG_HW], service_split=(
    [], ["uavcan.protocol.NodeStatus status", "uavcan.protocol.SoftwareVersion software_version",
         "uavcan.protocol.HardwareVersion hardware_version", "saturated uint8[<=80] name"]))
SIG_ALLOC = dsdl_sig("uavcan.protocol.dynamic_node_id.Allocation", [
    "saturated uint7 node_id", "saturated bool first_part_of_unique_id", "saturated uint8[<=16] unique_id"])
ID_STATUS, ID_ARRAYCMD, ID_HARDPOINT, ID_NODEINFO, ID_ALLOC = 341, 1010, 1070, 1, 1   # the DSDL file names


def crc16(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def f16(v):
    return struct.unpack("<H", struct.pack("<e", v))[0]


def enc_status(up, health, mode, sub=0, vendor=0):
    return struct.pack("<I", up) + bytes([(health & 3) << 6 | (mode & 7) << 3 | (sub & 7)]) + struct.pack("<H", vendor)


def enc_cmd(cmds):
    return b"".join(struct.pack("<BBH", a, t, f16(v)) for a, t, v in cmds)       # tail array: no length


def enc_hp(i, c):
    return struct.pack("<BH", i, c)


NAME = b"org.trygons.c2.aurix"


def enc_info(up):
    status = enc_status(up, 0, 0)
    sw = bytes([1, 0, 0]) + struct.pack("<IQ", 0, 0)
    hw = bytes([0, 0]) + bytes(16) + bytes([0])        # certificate: 8-bit length (not the last field)
    return status + sw + hw + NAME                       # name: the last field, no length


def msg_id(prio, dtid, src):
    return (prio << 24) | (dtid << 8) | src


def svc_id(prio, dtid, request, dst, src):
    return (prio << 24) | (dtid << 16) | (int(request) << 15) | (dst << 8) | 0x80 | src


def tail(sot, eot, tog, tid):
    return (0x80 if sot else 0) | (0x40 if eot else 0) | (0x20 if tog else 0) | (tid & 0x1F)


def frames_of(payload, sig, tid):
    """The CAN frame data of one transfer (single frame <= 7 bytes, else CRC + 7-byte pieces)."""
    if len(payload) <= 7:
        return [payload + bytes([tail(True, True, False, tid)])]
    c = crc16(payload, crc16(struct.pack("<Q", sig)))
    data = struct.pack("<H", c) + payload
    out, tog = [], False
    for i in range(0, len(data), 7):
        piece = data[i:i + 7]
        out.append(piece + bytes([tail(i == 0, i + 7 >= len(data), tog, tid)]))
        tog = not tog
    return out


def reassemble(frames, sig):
    """Payload of a multi-frame transfer [data bytes], checks start / end / toggle / tid / CRC."""
    tid = frames[0][-1] & 0x1F
    data = b""
    for k, fr in enumerate(frames):
        t = fr[-1]
        assert bool(t & 0x80) == (k == 0), "start bit"
        assert bool(t & 0x40) == (k == len(frames) - 1), "end bit"
        assert bool(t & 0x20) == (k % 2 == 1), "toggle"
        assert t & 0x1F == tid, "transfer id"
        data += fr[:-1]
    c, payload = struct.unpack_from("<H", data)[0], data[2:]
    assert c == crc16(payload, crc16(struct.pack("<Q", sig))), "transfer CRC"
    return payload, tid


# --------------------------------------------------------------- the harness ------
def run(cmds):
    r = subprocess.run([EXE], input="\n".join(cmds) + "\n", capture_output=True, text=True, timeout=60)
    blocks, cur = [], []
    for line in r.stdout.splitlines():
        if line == "END":
            blocks.append(cur)
            cur = []
        else:
            cur.append(line)
    return blocks


def tx(block):
    return [(int(l.split()[1], 16), bytes.fromhex(l.split()[2]) if len(l.split()) > 2 else b"")
            for l in block if l.startswith("TX ")]


def kv(line):
    return {k: int(v) for k, v in (p.split("=") for p in line.split()[1:])}


def bus(block):
    return kv(next(l for l in block if l.startswith("BUS ")))


def nodes(block):
    return {d["id"]: d for d in (kv(l) for l in block if l.startswith("NODE "))}


def payload(block):
    return bytes.fromhex(next(l for l in block if l.startswith("PAYLOAD")).split()[1] if True else "")


def rx_status(src, up, health=0, mode=0, tid=0):
    return "RX %08X %s" % (msg_id(24, ID_STATUS, src), (enc_status(up, health, mode) + bytes([tail(True, True, False,
                                                                                                    tid)])).hex())


RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append(ok)
    print("[%s] %s%s" % ("PASS" if ok else "FAIL", name, (": " + detail) if detail else ""))


# --------------------------------------------------------------- the tests --------
def t_signatures():
    b = run(["SIGS"])[0]
    sigs = [int(x, 16) for x in b[0].split()[1:]]
    ids = b[1].split()
    want = [SIG_STATUS, SIG_ARRAYCMD, SIG_HARDPOINT, SIG_NODEINFO, SIG_ALLOC]
    ok = sigs == want and [int(x) for x in ids[1:6]] == [ID_STATUS, ID_ARRAYCMD, ID_HARDPOINT, ID_NODEINFO, ID_ALLOC]
    ok = ok and ids[7] == "10" and ids[9:13] == ["20", "21", "22", "30"]
    check("D1 data type signatures (from the DSDL text) + IDs + the static node table", ok,
          " ".join("%016X" % s for s in sigs))


def t_payloads():
    cmds, exp = [], []
    for up, h, m, sm, v in [(0, 0, 0, 0, 0), (0x12345678, 1, 2, 5, 0xBEEF), (4294967295, 3, 7, 7, 65535),
                            (86400, 2, 1, 0, 1)]:
        cmds.append("ENC_STATUS %d %d %d %d %d" % (up, h, m, sm, v))
        exp.append(enc_status(up, h, m, sm, v))
    acmds = [[(1, 0, -1.0)], [(2, 0, 0.0)], [(3, 0, 0.5), (1, 0, 1.0), (2, 0, -0.25)], [(1, 4, 1500.0)],
             [(7, 1, -2.0), (8, 0, 0.75)]]
    for c in acmds:
        cmds.append("ENC_ACMD %d %s" % (len(c), " ".join("%d %d %r" % x for x in c)))
        exp.append(enc_cmd(c))
    for i, c in [(0, 0), (3, 4660), (5, 65535)]:
        cmds.append("ENC_HP %d %d" % (i, c))
        exp.append(enc_hp(i, c))
    cmds.append("ENC_INFO 123")
    exp.append(enc_info(123))
    got = [payload(b) for b in run(cmds)]
    bad = [i for i, (g, e) in enumerate(zip(got, exp)) if g != e]
    check("D2 payload encoders (NodeStatus, ArrayCommand tail array + float16, hardpoint, GetNodeInfo) = the DSDL "
          "bit layout", not bad and len(got) == len(exp), "mismatch at %s: %s vs %s" % (
              bad, got[bad[0]].hex() if bad else "", exp[bad[0]].hex() if bad else ""))
    # float16 + decoders
    vals = [-1.0, 0.0, 1.0, 0.5, -0.25, 0.1, 0.3333, -0.999, 65504.0]
    out = run(["F16 %r" % v for v in vals] + ["DEC_ACMD " + enc_cmd([(1, 0, -1.0), (3, 0, 0.5)]).hex(),
                                               "DEC_STATUS " + enc_status(77, 1, 2, 3, 4).hex()])
    f = [int(b[0].split()[1], 16) for b in out[:len(vals)]]
    exact = all(f[i] == f16(v) for i, v in enumerate(vals) if v in (-1.0, 0.0, 1.0, 0.5, -0.25, 65504.0))
    near = all(abs(f[i] - f16(v)) <= 1 for i, v in enumerate(vals))
    dec = out[len(vals)][0] == "CMDS 2 1:0:-1.000000 3:0:0.500000" and out[len(vals) + 1][0] == "STATUS 1 77 1 2 3 4"
    check("D3 float16 (exact values bit-exact, others within 1 ulp) + decoders of reference payloads",
          exact and near and dec, " ".join("%04X" % x for x in f))


def t_tx_frames():
    b = run(["INIT", "T 0", "T 500", "T 1000", "ACT 1 0 -1.0", "S", "ACT 2 0 0.5", "S", "HP 3 4660", "S",
             "ACT 1 0 7.5", "S", "ACT 1 0 nan", "S"])
    st0, none, st1 = tx(b[1]), tx(b[2]), tx(b[3])
    a1, a2, hp, clamp = tx(b[5]), tx(b[7]), tx(b[9]), tx(b[11])
    want_st0 = [(msg_id(24, ID_STATUS, 10), frames_of(enc_status(0, 0, 0), SIG_STATUS, 0)[0])]
    want_st1 = [(msg_id(24, ID_STATUS, 10), frames_of(enc_status(1, 0, 0), SIG_STATUS, 1)[0])]
    want_a1 = [(msg_id(8, ID_ARRAYCMD, 10), frames_of(enc_cmd([(1, 0, -1.0)]), SIG_ARRAYCMD, 0)[0])]
    want_a2 = [(msg_id(8, ID_ARRAYCMD, 10), frames_of(enc_cmd([(2, 0, 0.5)]), SIG_ARRAYCMD, 1)[0])]
    want_hp = [(msg_id(16, ID_HARDPOINT, 10), frames_of(enc_hp(3, 4660), SIG_HARDPOINT, 0)[0])]
    want_cl = [(msg_id(8, ID_ARRAYCMD, 10), frames_of(enc_cmd([(1, 0, 1.0)]), SIG_ARRAYCMD, 2)[0])]
    nan_refused = b[12] == ["QUEUED 0"] and tx(b[13]) == []
    ok = st0 == want_st0 and none == [] and st1 == want_st1 and a1 == want_a1 and a2 == want_a2 and hp == want_hp
    ok = ok and clamp == want_cl and nan_refused
    check("D4 our frames on the wire: NodeStatus 1 Hz (node 10, uptime, tid), ArrayCommand = ONE command in ONE "
          "frame (tid per transfer), hardpoint, UNITLESS clamped to +-1, NaN refused", ok,
          "%s %s %s" % (st0, a1, hp))


def t_nodeinfo():
    req_ok = svc_id(24, ID_NODEINFO, True, 10, 100)
    bad = [bytearray(x) for x in frames_of(bytes(range(10)), SIG_NODEINFO, 6)]   # a 2-frame request ...
    bad[0][0] ^= 0xFF                                           # ... with a wrong CRC
    two = frames_of(bytes(range(10)), SIG_NODEINFO, 7)          # the next one (payload ignored), right CRC
    other = svc_id(24, ID_NODEINFO, True, 11, 100)              # for another node
    b = run(["INIT", "T 0", "RX %08X %02X" % (req_ok, tail(True, True, False, 5)), "T 3",
             "RX %08X %s" % (other, bytes([tail(True, True, False, 7)]).hex()), "T 6"] +
            ["RX %08X %s" % (req_ok, bytes(x).hex()) for x in bad] + ["T 9"] +
            ["RX %08X %s" % (req_ok, x.hex()) for x in two] + ["T 12"])
    f = tx(b[3])
    want_id = svc_id(24, ID_NODEINFO, False, 100, 10)
    try:
        ids_ok = all(i == want_id for i, _ in f) and len(f) >= 2
        pl, tid = reassemble([d for _, d in f], SIG_NODEINFO)
        ok1 = ids_ok and tid == 5 and pl == enc_info(0)
    except AssertionError as e:
        ok1, pl = False, str(e).encode()
    ok2 = tx(b[5]) == []                                        # not for us: no reply
    ok3 = tx(b[8]) == [] and bus(b[8])["rxerr"] >= 1            # bad CRC: dropped + counted
    f4 = tx(b[11])
    try:
        pl4, tid4 = reassemble([d for _, d in f4], SIG_NODEINFO)
        ok4 = tid4 == 7 and pl4 == enc_info(0) and bus(b[11])["replies"] == 2
    except (AssertionError, IndexError):
        ok4 = False
    check("D5 GetNodeInfo server: multi-frame response (CRC seeded with the signature, toggles, tid echo, our name), "
          "a request for another node ignored, a bad-CRC multi-frame request dropped + counted", ok1 and ok2 and ok3 and
          ok4, "frames %d, payload %s" % (len(f), pl.hex()))


def t_node_table():
    b = run(["INIT", "T 0", rx_status(20, 100), rx_status(21, 200, health=1), rx_status(22, 300, mode=2),
             rx_status(30, 400, health=2), "T 100",
             rx_status(22, 301, mode=0, tid=1), rx_status(30, 401, health=3, tid=1), rx_status(20, 101, mode=1, tid=1),
             "T 600"])
    n1, n2 = nodes(b[6]), nodes(b[10])
    ok = (n1[20]["seen"] and not n1[20]["lost"] and n1[20]["up"] == 100 and n1[20]["req"] and
          n1[21]["warn"] and not n1[21]["lost"] and                         # WARNING = notice only (OD-6)
          n1[22]["lost"] and n1[22]["why"] == 3 and                         # not OPERATIONAL
          n1[30]["lost"] and n1[30]["why"] == 4 and not n1[30]["req"] and   # ERROR (relay board: not required)
          not n2[22]["lost"] and n2[30]["why"] == 4 and n2[20]["why"] == 3)  # CRITICAL / INITIALIZATION
    check("D6 node table: NodeStatus decoded (uptime, health, mode), WARNING = notice only, not OPERATIONAL / "
          "ERROR / CRITICAL = lost, required = throttle / steering / gear", ok, str(n1))


def t_silence():
    b = run(["INIT", "T 0", rx_status(20, 5), rx_status(21, 5), "T 10", "T 1400", "T 1600",
             rx_status(20, 7, tid=1), rx_status(22, 9), "T 1700"])
    a, c, d = nodes(b[5]), nodes(b[6]), nodes(b[9])
    ok = (not a[22]["lost"] and not a[20]["lost"] and                         # grace after the boot
          c[22]["lost"] and c[22]["why"] == 1 and c[20]["why"] == 2 and c[21]["why"] == 2 and
          not d[20]["lost"] and not d[22]["lost"] and d[21]["why"] == 2)
    check("D7 heartbeat timeout 1500 ms: a node never seen = lost (NEVER) only after the limit, a silent node = "
          "lost (SILENT), back with its next NodeStatus", ok, "%s / %s" % (c[22], c[20]))


def t_reboot():
    seq = ["INIT", "T 0", rx_status(20, 100), "T 10", rx_status(20, 3, tid=1), "T 20"]
    t = 20
    for k in range(6):
        t += 500
        seq += [rx_status(20, 4 + k, tid=2 + k), "T %d" % t]
        if k == 1:
            seq.append("ACK")
    seq.append("ACK")
    b = run(seq)
    r1 = nodes(b[5])[20]
    early = nodes(b[10])[20]      # the ACK after 1 s OPERATIONAL
    late = nodes(b[-1])[20]
    ok = r1["reb"] and r1["lost"] and r1["why"] == 5 and r1["restarts"] == 1 and early["reb"] and not late["reb"] \
        and not late["lost"]
    check("D8 node reboot (uptime back) = lost + latched; the acknowledge clears it only after 2 s OPERATIONAL",
          ok, "%s / %s / %s" % (r1, early, late))


def t_conflict_anon():
    anon_id = (24 << 24) | (0x1234 << 10) | (ID_ALLOC << 8)
    anon = "RX %08X %s" % (anon_id, (bytes([0x01]) + bytes(range(0x11, 0x17)) + bytes([tail(True, True, False, 0)])).hex())
    b = run(["INIT", "T 0", rx_status(10, 50), "T 10", "T 2900", "T 3100", anon, "T 3200", "T 8300"])
    ok = (bus(b[3])["conflict"] == 1 and nodes(b[3])[20]["seen"] == 0 and bus(b[4])["conflict"] == 1 and
          bus(b[5])["conflict"] == 0 and bus(b[7])["anon"] == 1 and bus(b[8])["anon"] == 0)
    answered = [i for i, _ in tx(b[7]) + tx(b[8]) if (i >> 8) & 0xFFFF != ID_STATUS]
    check("D9 a frame from OUR node ID 10 = ID conflict (3 s), never a table node; a node asking for a dynamic ID "
          "= notice (5 s), never answered", ok and not answered, "%s" % bus(b[3]))


def t_busoff():
    b = run(["INIT", "T 0", rx_status(20, 1), rx_status(21, 1), rx_status(22, 1), "T 10", "STATE 2 255 0", "T 20",
             "T 80", "T 130", "ACT 1 0 0.0", "T 1100", "STATE 0", "T 1200",
             rx_status(20, 3, tid=1), rx_status(21, 3, tid=1), rx_status(22, 3, tid=1), "T 1210"])
    s1, s2, s3, s4, s6 = bus(b[7]), bus(b[8]), bus(b[9]), bus(b[11]), bus(b[13])
    n3 = nodes(b[9])
    ok = (s1["state"] == 3 and s1["busoffs"] == 1 and s1["recovers"] == 1 and not s1["fault"] and
          not s2["fault"] and s3["fault"] and all(n["why"] == 6 for n in n3.values()) and
          b[10] == ["QUEUED 0"] and s4["recovers"] == 2 and s6["state"] == 1 and not s6["fault"] and
          not nodes(b[17])[20]["lost"])
    check("D10 bus-off: recovery requested at once and every 1 s, every node lost after 100 ms (BUS), nothing "
          "queued while dead, back when the controller is error active again", ok, "%s %s" % (s1, s3))


def t_passive_stale():
    b = run(["INIT", "T 0", "ROOM 0", "ACT 1 0 0.2", "ACT 2 0 0.3", "ACT 3 0 0.4", "T 10", "STATE 1 136 0", "T 20",
             "T 200", "STATE 0 0 0", "ROOM 100", "T 1100", "T 1200"])
    s_full, s_fault = bus(b[6]), bus(b[9])
    later = tx(b[12]) + tx(b[13])
    stale = [i for i, _ in later if (i >> 8) & 0xFFFF == ID_ARRAYCMD]
    ok = (tx(b[6]) == [] and s_fault["fault"] and s_fault["drops"] >= 3 and s_fault["state"] == 2 and
          not stale and any((i >> 8) & 0xFFFF == ID_STATUS for i, _ in later))
    check("D11 TX FIFO full = frames wait; error passive > 100 ms = bus fault, the waiting frames are DROPPED "
          "(counted) - no stale actuator command goes out after the bus comes back", ok, "%s, later %s" % (
              s_fault, [hex(i) for i, _ in later]))


def t_fifo_wait():
    b = run(["INIT", "T 0", "ROOM 0", "ACT 1 0 0.5", "T 10", "ROOM 5", "T 20"])
    ok = tx(b[4]) == [] and [(i >> 8) & 0xFFFF for i, _ in tx(b[6])] == [ID_ARRAYCMD]
    check("D12 a full controller FIFO never blocks: the frame waits in the queue and goes in the next step",
          ok, str(tx(b[6])))


def t_bounded_rx():
    b = run(["INIT", "T 0"] + [rx_status(20, 10 + k, tid=k % 32) for k in range(100)] + ["T 10", "T 20"])
    ok = bus(b[102])["rx"] == 64 and bus(b[103])["rx"] == 100 and nodes(b[103])[20]["statuses"] == 100
    check("D13 RX bounded: at most 64 frames per step (the rest in the next step), none lost", ok,
          "%d then %d" % (bus(b[102])["rx"], bus(b[103])["rx"]))


def t_start_fail():
    b = run(["START_FAIL", "INIT", "T 0", "ACT 1 0 0.0", "T 2000"])
    s = bus(b[2])
    ok = s["state"] == 4 and s["fault"] and tx(b[2]) == [] and b[3] == ["QUEUED 0"] and \
        all(n["why"] == 6 for n in nodes(b[4]).values())
    check("D14 the controller does not start: FAULT, every node lost (BUS), nothing sent", ok, str(s))


def t_bad_frames():
    b = run(["INIT", "T 0", "RX %08X " % msg_id(24, ID_STATUS, 20),                       # empty frame
             "RX %08X %s" % (msg_id(24, ID_STATUS, 20), (enc_status(5, 0, 0) + bytes([0x01])).hex()),   # no start
             "RX %08X %s" % (msg_id(24, 999, 20), bytes([1, 2, tail(True, True, False, 0)]).hex()),   # not ours
             "T 10", rx_status(20, 6), "T 20"])
    s = bus(b[5])
    ok = s["rxerr"] >= 1 and not nodes(b[5])[20]["seen"] and nodes(b[7])[20]["seen"]
    check("D15 bad frames (empty, a middle frame without its start, an unknown type) never reach the node table",
          ok, str(s))


def main():
    hal = os.path.join(REPO, "hal")
    srcs = [os.path.join(HERE, "test_dronecan.c"), os.path.join(hal, "dronecan.c"),
            os.path.join(hal, "libcanard", "canard.c")]
    r = subprocess.run(["gcc", "-std=c99", "-O2", "-Wall", "-Wextra", "-DCANARD_64_BIT=1", "-I" + hal] + srcs +
                       ["-o", EXE, "-lm"], capture_output=True, text=True)
    if r.returncode != 0 or r.stderr.strip():
        print("BUILD PROBLEM:\n" + r.stderr)
        if r.returncode != 0:
            return 1
    for t in (t_signatures, t_payloads, t_tx_frames, t_nodeinfo, t_node_table, t_silence, t_reboot, t_conflict_anon,
              t_busoff, t_passive_stale, t_fifo_wait, t_bounded_rx, t_start_fail, t_bad_frames):
        try:
            t()
        except Exception as e:          # a crash of one test is a FAIL, the others still run
            check(t.__name__, False, "exception %r" % e)
    print("DRONECAN TESTS: %d passed, %d failed" % (sum(RESULTS), len(RESULTS) - sum(RESULTS)))
    return 0 if all(RESULTS) else 1


if __name__ == "__main__":
    sys.exit(main())
