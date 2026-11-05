"""Stack check of the board firmware (step 13s): call graph, recursion, worst-case stack per core.

    python tools/stackcheck.py <folder with .ci files> [<more .ci files / folders>]
        The call graphs GCC writes with -fcallgraph-info=su (tools/check.sh compiles app/ + hal/ +
        hal/libcanard + Cpu*_Main.c for the TriCore with it): every function's own stack frame and
        every call. Function-pointer calls are resolved with INDIRECT below (the ONLY places the
        firmware calls through a pointer); a new one = FAIL until its targets are listed here. The
        table itself is PROVED against the source (13s review MED-1): source_indirect() finds every
        function whose address is taken in app/ hal/ Cpu*_Main.c and the pointer it goes to (the port
        struct initializers, canardInit / fat32_mount / uart_on_tx_done, link's sim_fn); INDIRECT must
        list exactly those targets, an address taken anywhere else = FAIL.
    python tools/stackcheck.py --selftest
        The checker's own tests (fixture graphs + fixture map text; tools/check.sh runs them).

The same analysis (analyse()) runs on the TASKING map of an ADS build in tools/mapreport.py (its
"Call Graph" section: the real compiler's frame sizes, with the iLLD and the C library included).

Per core it adds up the deepest call chain from core<n>_main (= the user stack) and, for the
interrupt stack, EVERY interrupt routine of that core nested on top of each other (any higher
priority may interrupt a lower one) plus the largest trap handler. FAIL (exit 2) when:
  - any recursion (a call cycle, also through a resolved function pointer),
  - a function-pointer call that INDIRECT does not list, or a listed target that is gone,
  - INDIRECT differs from the targets the source gives each pointer (missing / stale), a function's
    address taken outside a known pointer setup, or (map mode) a function in TASKING's __INDIRECT__
    (address-taken) list that is neither an INDIRECT target nor a start-up / trap / runtime entry,
  - a function with a dynamic stack frame (a variable-length array / alloca),
  - an interrupt routine whose core is unknown,
  - estimate x MARGIN (1.5 = 50 % margin) > the stack reserved in Lcf_Tasking_Tricore_Tc.lsl,
  - the two linker scripts (TASKING + GCC) reserve different stack / CSA sizes,
  - the context save area (CSA, 64 B per call level / 2 per interrupt) would not hold the deepest
    chain + every nested interrupt + a trap with the same margin.
Functions without a graph of their own (the iLLD, libc: not compiled by tools/check.sh) count
EXTERNAL_B each (PRINTF_B for the printf family) and one call level - the TASKING map has their
real numbers.
"""
import collections
import glob
import os
import re
import sys

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MARGIN = 1.5            # reserved stack >= 1.5 x the estimate (= at least 50 % margin)
EXTERNAL_B = 128        # GCC mode: a function without a graph (iLLD, libc): the largest iLLD chain in the
                        # 2026-11-05 TASKING map is 120 B (IfxQspi_SpiMaster_initModule)
PRINTF_B = 256          # GCC mode: snprintf & co. (TASKING's snprintf -> _doprint chain = 104 B in that map)
TRAP_B = 64             # GCC mode: the iLLD trap handlers (they only record the trap: [8,8] in the TASKING map)
CSA_FRAME = 64          # bytes per context save area entry (TriCore)
STARTUP_LEVELS = 4      # call levels below core<n>_main (iLLD start-up: _START -> ... -> core<n>_main)
PRINTF = {"snprintf", "vsnprintf", "sprintf", "printf", "_doprint", "_vfprintf_r", "_svfprintf_r"}

# Every call through a function pointer in the firmware, by the source module that makes it, and every
# function that pointer can be on the BOARD (hal_tc375.c fills the ports; the sim / PC tests fill them with
# their own functions, which never run on the board). Keep in step with the code: stackcheck FAILS on a
# pointer call in a module not listed here, on a listed target that no longer exists, and (13s review MED-1,
# check_source) when the table differs from what the SOURCE gives each pointer (source_indirect).
INDIRECT = {
    # tc_nv.c: the data-flash port (hal_tc375.c s_nv_port)
    "tc_nv": ["hal_tc375:nv_port_read", "hal_tc375:nv_port_erase_start", "hal_tc375:nv_port_write_start",
              "hal_tc375:nv_port_busy", "hal_tc375:nv_port_failed", "hal_micros"],
    # tc_fat32.c: the block device (tc_sd.c fat32_mount(read_block, write_block))
    "tc_fat32": ["tc_sd:read_block", "tc_sd:write_block"],
    # tc_uart.c: the TX-complete hook (hal_tc375.c uart_on_tx_done(UART_RELAY, relay_de_release): RS485 DE)
    "tc_uart": ["hal_tc375:relay_de_release"],
    # tc_modbus.c: the RS485 port (hal_tc375.c s_relay_port)
    "tc_modbus": ["hal_tc375:relay_port_write", "hal_tc375:relay_port_read", "hal_millis"],
    # dronecan.c: the CAN controller port (hal_tc375.c s_can_port; only with HAL_CAN_FITTED)
    # (13s review: the can_* port functions are tc_can.c's GLOBAL functions - the 13s table named them
    #  "hal_tc375:can_*", which matched nothing and was hidden by OPTIONAL_TARGETS; check_source found it)
    "dronecan": ["can_start", "can_send", "can_recv", "can_state", "can_recover", "hal_millis", "can_cancel_fifo"],
    # libcanard: the two callbacks dronecan.c gives canardInit
    "canard": ["dronecan:on_reception", "dronecan:should_accept"],
    # link.c: l->sim_fn = the simulator's PROTO_T_SIM_FAULT hook - set only in sim/ (NULL on the board)
    "link": [],
    # TASKING C library / iLLD start-up (map only): no C++ constructors / destructors in this C99 build;
    # the start-up phases are reached by jumps (Ifx_Ssw_jumpToFunction), never by these calls
    "__runtime__": [],
}
RUNTIME = {"__call_ctors()", "__call_dtors()", "_main", "__finalize_destructions"}
OPTIONAL_TARGETS = {"can_start", "can_send", "can_recv", "can_state", "can_recover", "can_cancel_fifo",
                    "hal_tc375:relay_port_write",
                    "hal_tc375:relay_port_read", "hal_tc375:relay_de_release",   # compiled only with a flag 1
                    "dronecan:on_reception", "dronecan:should_accept"}  # the linker drops them without the CAN bus

# 13s review MED-1: INDIRECT is no longer trusted as hand-kept - source_indirect() DERIVES the targets from the
# firmware source (every function whose address is taken = named anywhere but in a call / its own declaration)
# and attributes each to the module that will call it, by the pointer-call SETUP it is passed to:
SETUP_CALLS = {"canardInit": "canard",          # canardInit(..., on_reception, should_accept, ...)
               "fat32_mount": "tc_fat32",       # fat32_mount(read_block, write_block)
               "uart_on_tx_done": "tc_uart"}    # uart_on_tx_done(id, hook)
SETUP_TYPES = {"nvlog_port_t": "tc_nv",         # static const nvlog_port_t s_nv_port = { ... }
               "mb_port_t": "tc_modbus",
               "dc_port_t": "dronecan"}
SETUP_MEMBERS = {"sim_fn": "link"}              # l->sim_fn = fn / .sim_fn = fn (only sim/ does it)
MAP_ONLY = {"__runtime__"}                      # no source rule: the TASKING library / start-up (map mode)
# the board's firmware source (= what tools/check.sh compiles for the TriCore; sim/ and tests/ never run there)
SRC_GLOBS = ["app/*.c", "app/*.h", "hal/*.c", "hal/*.h", "hal/libcanard/*.c", "hal/libcanard/*.h", "Cpu*_Main.c"]
# map mode: the children of TASKING's "__INDIRECT__" root = every address-taken function of the whole image. Allowed
# without an INDIRECT entry: the iLLD trap vector handlers, the iLLD start-up phases (reached by jumps) and the
# core entries, the C++ runtime of the C library (none of them called by a firmware pointer call)
MAP_ADDR_OK = re.compile(r"^(IfxCpu_Trap_\w+|Ifx_Ssw\w*:\w+|Ifx_Ssw_\w+|core[0-2]_main|_START|__call_[cd]tors\(\)|"
                         r"__finalize_destructions\w*)$")
# a function's name where it is NOT its address (checked by hand, keep short)
NOT_POINTERS = {("app/logfields.h", "slam_cap"): "the X-macro's log field NAME slam_cap (the call is slam_cap(...))"}
C_KEYWORDS = {"if", "while", "for", "switch", "return", "sizeof", "do", "else", "case", "defined"}

STACKS = [("ustack0", "user stack core 0", 0), ("istack0", "interrupt stack core 0", 0),
          ("ustack1", "user stack CPU1", 1), ("istack1", "interrupt stack CPU1", 1),
          ("ustack2", "user stack core 2", 2), ("istack2", "interrupt stack core 2", 2)]


def norm(name, module=None):
    """'hal/tc_sd.c:read_block' / 'tc_sd.src:read_block' -> 'tc_sd:read_block'; globals stay as they are."""
    name = name.strip()
    if ":" in name and not name.endswith("()"):
        path, fn = name.rsplit(":", 1)
        mod = os.path.basename(path)
        mod = re.sub(r"\.(c|h|src)$", "", mod)
        return "%s:%s" % (mod, fn)
    return name


class Graph:
    def __init__(self):
        self.self_b = {}                                # function -> own frame (bytes); missing = no graph
        self.edges = collections.defaultdict(set)
        self.indirect = {}                              # caller -> module making the pointer call
        self.dynamic = set()
        self.module = {}                                # function -> module (where known)
        self.addr_taken = None                          # map mode: the children of TASKING's __INDIRECT__ root

    def add_node(self, fn, size, module=None):
        self.self_b[fn] = max(size, self.self_b.get(fn, 0))
        if module:
            self.module.setdefault(fn, module)


def parse_ci(paths):
    """GCC -fcallgraph-info=su files (VCG) -> Graph."""
    g = Graph()
    for p in paths:
        unit = os.path.splitext(os.path.basename(p))[0]
        txt = open(p, encoding="utf-8", errors="replace").read()
        for m in re.finditer(r'node: \{ title: "([^"]+)" label: "([^"]*)"', txt):
            title, label = m.group(1), m.group(2)
            if title == "__indirect_call":
                continue
            s = re.search(r"\\n(\d+) bytes \(([a-z,]+)\)", label)
            if s:
                fn = norm(title)
                g.add_node(fn, int(s.group(1)), unit)
                if "dynamic" in s.group(2) and "bounded" not in s.group(2):
                    g.dynamic.add(fn)
        for m in re.finditer(r'edge: \{ sourcename: "([^"]+)" targetname: "([^"]+)"', txt):
            a, b = norm(m.group(1)), m.group(2)
            if b == "__indirect_call":
                g.indirect[a] = unit
            else:
                g.edges[a].add(norm(b))
    return g


def parse_map(txt):
    """TASKING map: the "Call Graph" section (trees, [s0:own,all]) -> Graph."""
    g = Graph()
    lines = txt.split("\n")
    try:
        s = next(i for i, l in enumerate(lines) if "*  Call Graph  *" in l)
    except StopIteration:
        return None
    e = next((i for i in range(s + 1, len(lines)) if lines[i].startswith("Note:")), len(lines))
    mods = {}                                           # global function -> module, from the section list
    for m in re.finditer(r"\.text\.(\w+?)\.([A-Za-z_]\w*)\b", txt):  # .text.<module>.<function> (a table
        mods.setdefault(m.group(2), m.group(1))                     # column may cut it: another one has it)
    stack = []
    for l in lines[s + 1:e]:
        p = l.find("+-- ")
        if p >= 0:
            depth, rest = p // 4 + 1, l[p + 4:]
        elif l and l[0] not in "| ":
            depth, rest = 0, l
        else:
            continue
        rest = re.sub(r"^\*R? ", "", rest)
        m = re.match(r"(.+?)(?: \[s0:(\d+),(\d+)\])?(?: \*R?)?\s*$", rest)
        if not m:
            continue
        name = norm(m.group(1).strip())
        if m.group(2) is not None:
            mod = name.split(":")[0] if ":" in name else mods.get(name)
            g.add_node(name, int(m.group(2)), mod)
        stack = stack[:depth]
        if depth == 1 and stack[0] == "__INDIRECT__":
            g.addr_taken = (g.addr_taken or set()) | {name}
        if depth:
            caller = stack[-1]
            if name == "__INDIRECT__":
                if caller in RUNTIME or caller.startswith("__finalize_destructions"):
                    g.indirect[caller] = "__runtime__"
                else:
                    g.indirect[caller] = caller.split(":")[0] if ":" in caller else mods.get(caller, "?")
            elif caller != "__INDIRECT__":
                g.edges[caller].add(name)
        stack.append(name)
    for fn, mod in g.module.items():
        if fn in g.indirect and g.indirect[fn] == "?":
            g.indirect[fn] = mod
    return g


def isr_table(root=HERE):
    """Every interrupt routine of the firmware -> its core (IFX_INTERRUPT(name, core, prio), also via a macro)."""
    out = {}
    files = glob.glob(os.path.join(root, "hal", "*.c")) + glob.glob(os.path.join(root, "app", "*.c")) + \
        glob.glob(os.path.join(root, "Cpu*_Main.c"))
    for f in files:
        txt = open(f, encoding="utf-8", errors="replace").read()
        macros = {}
        for m in re.finditer(r"#define (\w+)\((\w+)[^)]*\)((?:[^\n]*\\\n)+[^\n]*)", txt):
            body = m.group(3)
            for i in re.finditer(r"IFX_INTERRUPT\(\s*(\w*)##%s##(\w*)\s*,\s*(\d+)" % m.group(2), body):
                macros.setdefault(m.group(1), []).append((i.group(1), i.group(2), int(i.group(3))))
        for m in re.finditer(r"^IFX_INTERRUPT\(\s*(\w+)\s*,\s*(\d+)", txt, re.M):
            out[m.group(1)] = int(m.group(2))
        for name, pats in macros.items():
            for inv in re.finditer(r"^%s\(\s*(\w+)\s*," % name, txt, re.M):
                for pre, post, core in pats:
                    out[pre + inv.group(1) + post] = core
    return out


def lsl_sizes(root=HERE):
    """Bytes reserved per stack + CSA in BOTH linker scripts: ({'ustack0': n, ..., 'csa0': n}, problems)."""
    res, probs = {}, []
    for lsl, pat in (("Lcf_Tasking_Tricore_Tc.lsl", r"#define LCF_%s%d_SIZE\s+(\d+)k"),
                     ("Lcf_Gnuc_Tricore_Tc.lsl", r"LCF_%s%d_SIZE\s*=\s*(\d+)k\s*;")):
        txt = open(os.path.join(root, lsl), encoding="utf-8", errors="replace").read()
        for c in range(3):
            for k in ("USTACK", "ISTACK", "CSA"):
                m = re.search(pat % (k, c), txt)
                key = "%s%d" % (k.lower(), c)
                if not m:
                    probs.append("%s: LCF_%s%d_SIZE not found" % (lsl, k, c))
                    continue
                v = int(m.group(1)) * 1024
                if key in res and res[key] != v:
                    probs.append("the linker scripts differ: %s %d B (TASKING) vs %d B (GCC)" % (key, res[key], v))
                res.setdefault(key, v)
    return res, probs


def _strip_c(txt):
    """Comments, string / char literals and #include lines blanked (same length: offsets = line numbers stay)."""
    def blank(m):
        return re.sub(r"[^\n]", " ", m.group(0))
    txt = re.sub(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'', blank, txt, flags=re.S)
    return re.sub(r"^[ \t]*#[ \t]*include[^\n]*", blank, txt, flags=re.M)


def _enclosing(txt, pos):
    """The setup context of the token at pos: (names of the calls whose argument list holds it, innermost first;
    the statement text before it). Walks back to the start of the statement / initializer."""
    calls, pd, bd, i = [], 0, 0, pos - 1
    while i >= 0:
        c = txt[i]
        if c == ")":
            pd += 1
        elif c == "(":
            if pd:
                pd -= 1
            else:
                m = re.search(r"(\w+)\s*$", txt[:i])
                calls.append(m.group(1) if m else "")
        elif c == "}":
            if txt[i + 1:].lstrip()[:1] in (",", "}") or bd:
                bd += 1                         # a nested initializer { ... },
            else:
                break                           # the end of the previous block / definition
        elif c == "{":
            if bd:
                bd -= 1
            elif txt[:i].rstrip()[-1:] not in ("=", ",", "{"):
                break                           # a block: the statement starts after it
        elif c == ";" and not pd and not bd:
            break
        i -= 1
    return calls, txt[i + 1:pos]


def source_indirect(root=HERE):
    """Every function-pointer target of the firmware source, by the module that calls through the pointer:
    ({module: {target: 'file:line'}}, problems). A function whose address is taken where no SETUP rule attributes
    it = a problem (a new hook / port nobody listed: no proof of "no recursion" without it)."""
    files = []
    for pat in SRC_GLOBS:
        files += sorted(glob.glob(os.path.join(root, pat)))
    texts, defined, globs, statics = {}, set(), set(), collections.defaultdict(set)
    for f in files:
        t = _strip_c(open(f, encoding="utf-8", errors="replace").read())
        texts[f] = t
        mod = re.sub(r"\.(c|h)$", "", os.path.basename(f))
        for m in re.finditer(r"^[A-Za-z_][\w \t\*]*?\b(\w+)\s*\([^;{}()]*(?:\([^;{}()]*\)[^;{}()]*)*\)\s*\{", t, re.M):
            name = m.group(1)
            if name in C_KEYWORDS:
                continue
            defined.add(name)
            if re.match(r"\s*static\b", m.group(0)):
                statics[mod].add(name)
            else:
                globs.add(name)
    out = {m: {} for m in list(SETUP_CALLS.values()) + list(SETUP_TYPES.values()) + list(SETUP_MEMBERS.values())}
    problems = []
    for f, t in texts.items():
        mod = re.sub(r"\.(c|h)$", "", os.path.basename(f))
        rel = os.path.relpath(f, root).replace("\\", "/")
        for m in re.finditer(r"\b[A-Za-z_]\w*\b", t):
            name = m.group(0)
            if name not in defined or name in C_KEYWORDS:
                continue
            if name not in statics[mod] and name not in globs:
                continue                        # another file's static function: not reachable from here
            before, after = t[:m.start()].rstrip(), t[m.end():].lstrip()
            if after[:1] == "(":
                continue                        # a call / a declaration / a definition
            if before.endswith(".") or before.endswith("->") or re.search(r"\(\s*\*\s*$", before):
                continue                        # a struct member / a pointer declarator of that name
            in_body = re.search(r"#\s*define\s+\w+(\([^)\n]*\))?[ \t]+[^\n]*$", before)  # a macro's body
            if re.search(r"#\s*(define|undef|ifn?def|if|elif)\b[^\n]*$", before) and not in_body:
                continue                        # a macro name test / the defined macro's own name
            pm = re.search(r"(\w+)[\s\*]*$", before)
            if pm and pm.group(1) != "return" and not in_body:
                continue                        # "type name" = a variable / member declared with that name
            if (rel, name) in NOT_POINTERS:
                continue
            target = "%s:%s" % (mod, name) if name in statics[mod] else name
            where = "%s:%d" % (rel, t.count("\n", 0, m.start()) + 1)
            calls, stmt = _enclosing(t, m.start())
            owner = next((SETUP_CALLS[c] for c in calls if c in SETUP_CALLS), None)
            if owner is None:
                tm = re.search(r"\b(%s)\b[^=;]*=\s*\{" % "|".join(SETUP_TYPES), stmt)
                owner = SETUP_TYPES[tm.group(1)] if tm else None
            if owner is None:
                mm = re.search(r"(?:\.|->)\s*(\w+)\s*=\s*(?:\([^()]*\)\s*)?&?\s*$", stmt)
                owner = SETUP_MEMBERS.get(mm.group(1)) if mm else None
            if owner is None:
                problems.append("the address of %s is taken at %s and no pointer-call setup of stackcheck "
                                "(SETUP_CALLS / SETUP_TYPES / SETUP_MEMBERS) attributes it: list the module that "
                                "calls it + the target in INDIRECT" % (target, where))
                continue
            out[owner].setdefault(target, where)
    return out, problems


def check_source(src, problems, table=None):
    """INDIRECT (table) must list EXACTLY the targets the source gives each module (13s review MED-1)."""
    table = INDIRECT if table is None else table
    targets, probs = src
    problems += probs
    for mod in sorted(set(table) | set(targets)):
        if mod in MAP_ONLY:
            continue
        if mod not in targets:
            problems.append("stackcheck.INDIRECT[%s]: no source rule finds this module's pointer setup" % mod)
            continue
        listed = set(table.get(mod, []))
        for t in sorted(set(targets[mod]) - listed):
            problems.append("stackcheck.INDIRECT[%s] misses %s (its address goes to that pointer at %s)" %
                            (mod, t, targets[mod][t]))
        for t in sorted(listed - set(targets[mod])):
            problems.append("stackcheck.INDIRECT[%s] lists %s but the source never sets that pointer to it" % (mod, t))


def resolve(g, problems):
    """Function-pointer calls -> edges to the INDIRECT targets."""
    known = set(g.self_b) | set(g.edges)
    for caller, mod in sorted(g.indirect.items()):
        if mod not in INDIRECT:
            problems.append("function-pointer call in %s (module %s) is not in stackcheck.INDIRECT - list its "
                            "targets" % (caller, mod))
            continue
        for t in INDIRECT[mod]:
            g.edges[caller].add(t)
    used = set(g.indirect.values())
    for mod, targets in INDIRECT.items():
        if mod not in used:
            continue
        for t in targets:
            if t not in known and t not in OPTIONAL_TARGETS:
                problems.append("stackcheck.INDIRECT[%s]: target %s not in the build (stale table?)" % (mod, t))


def cycles(g):
    """Strongly connected components with a cycle (iterative Tarjan)."""
    index, low, on, st, out = {}, {}, set(), [], []
    counter = [0]
    nodes = set(g.edges) | {b for v in g.edges.values() for b in v}
    for root in sorted(nodes):
        if root in index:
            continue
        work = [(root, iter(sorted(g.edges.get(root, ()))))]
        index[root] = low[root] = counter[0]
        counter[0] += 1
        st.append(root)
        on.add(root)
        while work:
            v, it = work[-1]
            nxt = next(it, None)
            if nxt is not None:
                if nxt not in index:
                    index[nxt] = low[nxt] = counter[0]
                    counter[0] += 1
                    st.append(nxt)
                    on.add(nxt)
                    work.append((nxt, iter(sorted(g.edges.get(nxt, ())))))
                elif nxt in on:
                    low[v] = min(low[v], index[nxt])
                continue
            work.pop()
            if work:
                low[work[-1][0]] = min(low[work[-1][0]], low[v])
            if low[v] == index[v]:
                comp = []
                while True:
                    w = st.pop()
                    on.discard(w)
                    comp.append(w)
                    if w == v:
                        break
                if len(comp) > 1 or v in g.edges.get(v, ()):
                    out.append(sorted(comp))
    return out


def cost(g, fn):
    if fn in g.self_b:
        return g.self_b[fn]
    return PRINTF_B if fn in PRINTF else EXTERNAL_B


def worst(g, fn, memo):
    """(bytes, call levels, chain) of the deepest chain from fn (the graph has no cycle)."""
    if fn in memo:
        return memo[fn]
    best = (0, 0, [])
    for c in g.edges.get(fn, ()):
        r = worst(g, c, memo)
        if (r[0], r[1]) > (best[0], best[1]):
            best = r
    depth = max([best[1]] + [worst(g, c, memo)[1] for c in g.edges.get(fn, ())])
    memo[fn] = (cost(g, fn) + best[0], 1 + depth, [fn] + best[2])
    return memo[fn]


def check_map_addr(g, problems):
    """Map mode (13s review MED-1): every function TASKING saw address-taken must be an INDIRECT target or one of
    the start-up / trap / runtime entries (MAP_ADDR_OK) - a pointer target the table does not know = FAIL."""
    if g.addr_taken is None:
        return
    union = {t for v in INDIRECT.values() for t in v}
    for fn in sorted(g.addr_taken):
        if fn not in union and not MAP_ADDR_OK.match(fn):
            problems.append("the map's __INDIRECT__ (address taken): %s is no stackcheck.INDIRECT target" % fn)


def analyse(g, reserved, isrs, trap=None, label="GCC call graph", source=None):
    """The checks of the module help. Returns (ok, report lines, per-stack estimates).
    source = source_indirect()'s result: INDIRECT must match it exactly (check_source); None = not checked
    (the fixture graphs of the self-tests)."""
    problems, lines = [], []
    if source is not None:
        check_source(source, problems)
    resolve(g, problems)
    check_map_addr(g, problems)
    for fn in sorted(g.dynamic):
        problems.append("%s has a dynamic stack frame (variable-length array / alloca)" % fn)
    cyc = cycles(g)
    for c in cyc:
        problems.append("RECURSION: " + " -> ".join(c[:12]) + (" ..." if len(c) > 12 else ""))
    est = {}
    if not cyc:
        sys.setrecursionlimit(max(10000, sys.getrecursionlimit()))
        memo = {}
        present = set(g.self_b) | set(g.edges)
        for fn in sorted(isrs):
            if fn in present and isrs[fn] not in (0, 1, 2):
                problems.append("interrupt %s: unknown core %s" % (fn, isrs[fn]))
        for fn in sorted(present):
            if fn.endswith("_isr") and ":" not in fn and fn not in isrs:
                problems.append("interrupt %s: core unknown (no IFX_INTERRUPT found)" % fn)
        if trap is None:
            trap_b, trap_lv = TRAP_B, 1
        else:
            trap_b, trap_lv = trap
        for key, title, core in STACKS:
            if key.startswith("u"):
                entry = "core%d_main" % core
                if entry not in present:
                    problems.append("%s not in the call graph" % entry)
                    continue
                b, lv, chain = worst(g, entry, memo)
                est[key] = (b, lv + STARTUP_LEVELS, chain)
            else:
                b, lv = trap_b, 2 + trap_lv
                names = [fn for fn, c in sorted(isrs.items()) if c == core and fn in present]
                for fn in names:
                    r = worst(g, fn, memo)
                    b += r[0]
                    lv += 2 + r[1]              # 2 contexts per interrupt (upper by the hardware, lower by bisr)
                est[key] = (b, lv, names)
        lines.append("STACK per core (%s; estimate x %.1f must fit, i.e. >= %d %% margin)" %
                     (label, MARGIN, round((MARGIN - 1) * 100)))
        lines.append("  %-26s %10s %10s %8s  %s" % ("stack", "estimate", "reserved", "margin", "deepest chain"))
        for key, title, core in STACKS:
            if key not in est:
                continue
            b, lv, chain = est[key]
            r = reserved.get(key, 0)
            flag = ""
            if b * MARGIN > r:
                flag = "  <-- FAIL"
                problems.append("%s: estimate %d B x %.1f > reserved %d B" % (title, b, MARGIN, r))
            what = " -> ".join(chain[:6]) + (" ..." if len(chain) > 6 else "") if key.startswith("u") else \
                "%d interrupts nested + a trap" % len(chain)
            lines.append("  %-26s %8d B %8d B %7s  %s%s" % (title, b, r, "%d %%" % (100 * (r - b) / b) if b else "-",
                                                            what, flag))
        lines.append("CSA per core (%d B per call level; deepest chain + every interrupt nested + a trap)" % CSA_FRAME)
        for core in range(3):
            if "ustack%d" % core not in est or "istack%d" % core not in est:
                continue
            need = (est["ustack%d" % core][1] + est["istack%d" % core][1]) * CSA_FRAME
            r = reserved.get("csa%d" % core, 0)
            flag = ""
            if need * MARGIN > r:
                flag = "  <-- FAIL"
                problems.append("core %d CSA: %d B x %.1f > reserved %d B" % (core, need, MARGIN, r))
            lines.append("  core %d: %d levels = %d B of %d B%s" % (core, need // CSA_FRAME, need, r, flag))
    lines.append("recursion: %s" % ("NONE (every function-pointer call resolved: %d call sites)" % len(g.indirect)
                                    if not cyc else "FOUND"))
    if source is not None:
        lines.append("pointer targets: INDIRECT checked against the source (%d targets in %d modules)%s" % (
            sum(len(v) for v in source[0].values()), len(source[0]),
            "" if g.addr_taken is None else "; the map's %d address-taken functions known" % len(g.addr_taken)))
    for p in problems:
        lines.append("FAIL: " + p)
    return not problems, lines, est


def selftest():
    """Fixture graphs: recursion, pointer calls, the margin, dynamic frames, ISR nesting, map parsing."""
    fails = []

    def check(name, cond):
        print("[%s] %s" % ("PASS" if cond else "FAIL", name))
        if not cond:
            fails.append(name)

    res = {"ustack0": 2048, "istack0": 1024, "ustack1": 2048, "istack1": 1024, "ustack2": 2048, "istack2": 1024,
           "csa0": 8192, "csa1": 8192, "csa2": 8192}

    def base():
        g = Graph()
        for fn, b in (("core0_main", 100), ("core1_main", 40), ("core2_main", 40), ("a", 200), ("b", 300),
                      ("tc_sd:read_block", 16), ("tc_sd:write_block", 16), ("x_isr", 32), ("y_isr", 48), ("z_isr", 8)):
            g.add_node(fn, b)
        g.edges["core0_main"] |= {"a", "b"}
        g.edges["a"].add("b")
        return g
    isrs = {"x_isr": 0, "y_isr": 0, "z_isr": 2}
    ok, lines, est = analyse(base(), res, isrs)
    check("ST1 a clean graph passes; deepest chain core0_main -> a -> b = 600 B", ok and est["ustack0"][0] == 600)
    check("ST2 interrupt stack = every interrupt of the core nested + a trap (32 + 48 + 64)",
          est["istack0"][0] == 32 + 48 + TRAP_B and est["istack2"][0] == 8 + TRAP_B)
    g = base()
    g.edges["b"].add("a")
    ok, lines, _ = analyse(g, res, isrs)
    check("ST3 a call cycle a -> b -> a = FAIL (recursion)", not ok and any("RECURSION" in l for l in lines))
    g = base()
    g.edges["b"].add("b")
    check("ST4 direct recursion b -> b = FAIL", not analyse(g, res, isrs)[0])
    g = base()
    g.indirect["b"] = "tc_fat32"
    ok, lines, est = analyse(g, res, isrs)
    check("ST5 a listed pointer call is resolved to its targets (tc_fat32 -> tc_sd:read_block)",
          ok and "tc_sd:read_block" in g.edges["b"])
    g = base()
    g.indirect["b"] = "newmodule"
    check("ST6 a pointer call in a module not in INDIRECT = FAIL", not analyse(g, res, isrs)[0])
    g = base()
    g.add_node("tc_fat32:read_block", 0)
    g.indirect["tc_fat32:read_block"] = "tc_fat32"
    g.edges["tc_sd:read_block"].add("tc_fat32:read_block")
    g.edges["core0_main"].add("tc_sd:read_block")
    check("ST7 a cycle through a resolved function pointer = FAIL", not analyse(g, res, isrs)[0])
    g = base()
    g.self_b["b"] = 1300
    ok, lines, _ = analyse(g, res, isrs)
    check("ST8 estimate 1600 B x 1.5 > 2048 B reserved = FAIL", not ok and any("x 1.5 >" in l for l in lines))
    g = base()
    g.dynamic.add("a")
    check("ST9 a dynamic stack frame = FAIL", not analyse(g, res, isrs)[0])
    g = base()
    g.add_node("w_isr", 8)
    check("ST10 an interrupt routine without a known core = FAIL", not analyse(g, res, isrs)[0])
    g = base()
    g.edges["b"].add("snprintf")
    ok, _, est = analyse(g, res, isrs)
    check("ST11 an external printf counts %d B" % PRINTF_B, est["ustack0"][0] == 100 + 200 + 300 + PRINTF_B)
    ci = ('graph: { title: "hal/x.c"\nnode: { title: "core0_main" label: "core0_main\\nCpu0_Main.c:1:6\\n'
          '24 bytes (static)" }\nnode: { title: "hal/x.c:f" label: "f\\nhal/x.c:2:13\\n8 bytes (dynamic)" }\n'
          'edge: { sourcename: "core0_main" targetname: "hal/x.c:f" label: "x" }\n'
          'edge: { sourcename: "hal/x.c:f" targetname: "__indirect_call" label: "y" }\n}\n')
    import tempfile
    d = tempfile.mkdtemp()
    p = os.path.join(d, "tc_nv.ci")
    open(p, "w").write(ci)
    g = parse_ci([p])
    check("ST12 .ci parsing: frames, static names, pointer calls by unit, dynamic frames",
          g.self_b.get("core0_main") == 24 and "x:f" in g.edges["core0_main"] and g.indirect.get("x:f") == "tc_nv"
          and "x:f" in g.dynamic)
    m = MAP_FIXTURE
    g = parse_map(m)
    check("ST13 map parsing: frames + edges + __INDIRECT__ callers by module",
          g.self_b.get("core0_main") == 816 and "sb_step" in g.edges["core0_main"] and
          g.indirect.get("tc_fat32:read_block") == "tc_fat32" and g.indirect.get("dc_tx_flush") == "dronecan" and
          g.indirect.get("__call_ctors()") == "__runtime__")
    cyc = cycles(parse_map(m))
    check("ST14 the map's recursion goes only through __INDIRECT__ (no direct cycle)", not cyc)

    # 13s review MED-1: the table is checked against the SOURCE (source_indirect / check_source) and the map
    real = source_indirect()

    def src_fail(mutate=None, src=None):
        saved = {k: list(v) for k, v in INDIRECT.items()}
        try:
            if mutate:
                mutate(INDIRECT)
            probs = []
            check_source(src if src is not None else real, probs)
            ok_a = analyse(base(), res, isrs, source=src if src is not None else real)[0]
            return probs, ok_a
        finally:
            INDIRECT.clear()
            INDIRECT.update(saved)
    probs, ok = src_fail()
    check("ST15 the firmware source gives exactly the INDIRECT targets (%d targets, %d modules)" % (
        sum(len(v) for v in real[0].values()), len(real[0])), not probs and ok and not real[1])
    probs, ok = src_fail(lambda t: t.__setitem__("canard", []))
    check("ST16 reviewer mutation: INDIRECT \"canard\": [] = FAIL (the source gives canardInit 2 callbacks)",
          not ok and any("INDIRECT[canard] misses dronecan:on_reception" in p for p in probs))
    probs, ok = src_fail(lambda t: t["tc_fat32"].remove("tc_sd:write_block"))
    check("ST17 reviewer mutation: tc_sd:write_block removed from tc_fat32 = FAIL",
          not ok and any("INDIRECT[tc_fat32] misses tc_sd:write_block" in p for p in probs))
    probs, ok = src_fail(lambda t: t["tc_uart"].append("hal_tc375:gone"))
    check("ST18 a table target the source never sets = FAIL (stale)", not ok and any("never sets" in p for p in probs))

    def fixture_src(files):
        d2 = tempfile.mkdtemp()
        for rel, body in files.items():
            os.makedirs(os.path.join(d2, os.path.dirname(rel)), exist_ok=True)
            open(os.path.join(d2, rel), "w").write(body)
        return source_indirect(d2)
    quiet = ("static int cb(int x) { return x; }\nstruct s { int cb; int (*fp)(int); };\n"
             "/* cb as a comment */\nint use(struct s *p) { int cb = 1; p->cb = cb(2); return p->cb + cb(3); }\n")
    fs = fixture_src({"app/q.c": quiet})
    check("ST19 calls, declarations, members, comments of a function's name = no address taken",
          not fs[1] and not any(fs[0].values()))
    fs = fixture_src({"app/n.c": "static void hook(void) { }\nvoid setup(void) { foo_register(hook); }\n"})
    check("ST20 a NEW hook (a function's address given to an unknown setup) = FAIL",
          any("address of n:hook is taken at app/n.c:2" in p for p in fs[1]))
    fs = fixture_src({"hal/hal_tc375.c": "static bool nv_port_read(void) { return 1; }\nstatic bool extra(void) "
                      "{ return 0; }\nstatic const nvlog_port_t s_nv_port = {\n    1u, nv_port_read,\n    extra };\n"})
    probs, ok = src_fail(src=fs)
    check("ST21 a new function in a known port (nvlog_port_t) = FAIL until INDIRECT[tc_nv] lists it",
          not ok and any("INDIRECT[tc_nv] misses hal_tc375:extra" in p for p in probs))
    fs = fixture_src({"app/l.c": "static uint8_t simhook(void *c, const void *f) { return 0; }\n"
                      "void link_set(link_t *l) { l->sim_fn = simhook; }\n"})
    probs, ok = src_fail(src=fs)
    check("ST22 a board function put into link.c's sim_fn = FAIL (INDIRECT[link] is empty)",
          not ok and any("INDIRECT[link] misses l:simhook" in p for p in probs))
    g = parse_map(m.replace("+-- core0_main [s0:816,1872]", "+-- newdrv.src:hook [s0:0,0]\n|\n"
                            "+-- core0_main [s0:816,1872]"))
    probs = []
    check_map_addr(g, probs)
    probs0 = []
    check_map_addr(parse_map(m), probs0)
    check("ST23 map mode: an address-taken function (__INDIRECT__ child) no INDIRECT lists = FAIL; the fixture's OK",
          not probs0 and any("newdrv:hook" in p for p in probs))
    print("STACKCHECK SELFTEST: %s" % ("PASS" if not fails else "FAIL (%d)" % len(fails)))
    return not fails


# A cut-down TASKING "Call Graph" section (the real format of the 2026-11-05 ADS map) for the self-tests
MAP_FIXTURE = """
| dronecan.o                       | .text.dronecan.dc_tx_flush     | 0x0000009e      | 0x0          |
| smallboat.o                      | .text.smallboat.sb_step (1)    | 0x00000100      | 0x0          |
*************************************************************************  Call Graph  *************************************************************************
_START [s0:0,0]

*R Ifx_Ssw_doCppInit [s0:0,0]
|
+-- _main [s0:0,0]
|   |
|   +-- __call_ctors() [s0:0,0]
|       |
|       +-- __INDIRECT__ *R

*R __INDIRECT__ [s0:0,1872]
|
+-- tc_sd.src:read_block [s0:0,24]
|
+-- core0_main [s0:816,1872]
    |
    +-- sb_step [s0:520,1056]
    |   |
    |   +-- dc_tx_flush *R
    |
    +-- tc_fat32.src:read_block *R

*R dc_tx_flush [s0:16,16]
|
+-- canardPeekTxQueue [s0:0,0]
|
+-- __INDIRECT__ *R

*R tc_fat32.src:read_block [s0:0,0]
|
+-- __INDIRECT__ *R

* tc_sd.src:write_block [s0:0,24]

* hal_millis [s0:0,0]

* hal_micros [s0:0,0]


Note:
"""


def main(argv):
    if argv == ["--selftest"]:
        return 0 if selftest() else 2
    if not argv:
        print(__doc__)
        return 1
    paths = []
    for a in argv:
        paths += sorted(glob.glob(os.path.join(a, "*.ci"))) if os.path.isdir(a) else [a]
    if not paths:
        print("no .ci files in %s" % " ".join(argv))
        return 2
    g = parse_ci(paths)
    reserved, probs = lsl_sizes()
    ok, lines, _ = analyse(g, reserved, isr_table(), label="GCC call graph of %d units" % len(paths),
                           source=source_indirect())
    print("\n".join(lines))
    for p in probs:
        print("FAIL: " + p)
    ok = ok and not probs
    print("STACK CHECK: %s" % ("OK" if ok else "FAILED"))
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
