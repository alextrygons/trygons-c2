#!/usr/bin/env python3
"""Direct Anthropic API review tool (step API1 / API1-r, owner 2026-11-09 / 10).

Independent SAFETY review of a commit (or range) through the Messages API with
the `anthropic` SDK - NOT `claude -p`, no Claude Code, no tools: the model sees
ONLY (1) the CLAUDE.md safety-rules text (system block, cache_control = prompt
caching), (2) the step spec, (3) the commit's diff (size capped, see below).

  python tools/api_review.py HEAD --step PF6 --focus "lost link"
  python tools/api_review.py abc123..def456 --spec docs/spec.txt --model sonnet
  python tools/api_review.py HEAD --urgent               # synchronous even if non-safety
  python tools/api_review.py HEAD --batch --wait 20      # Batch API, 50 % price
  python tools/api_review.py --collect msgbatch_xxx       # resume a batch
  python tools/api_review.py --audit "Is X ever Y?" --files a.c,b.c

RULES (API1-r):
* model by the files in the diff: opus ONLY for boat / command / safety paths
  (SAFETY_GLOBS), sonnet for display / docs / tools / test-only diffs;
  --model overrides; the model and the reason are printed.
* a SAFETY review is synchronous (it blocks the next step); a non-safety review
  goes through the Batch API (half price, bounded --wait, --collect resumes it).
  --urgent forces synchronous, --batch forces the batch.
* never more than ~60k input tokens per call (rules + spec included): generated /
  data files are excluded (EXCLUDE_GLOBS, --exclude), a bigger diff is SPLIT by
  area into several calls and the findings are MERGED into one file whose header
  says what was excluded and how it was split.
* an answer with no text or stop_reason max_tokens is an ERROR (no findings file).
* the real cost of every review (all calls) and a month total (cost_total.json,
  outside git) are printed.

The API key is read ONLY from the key file (default C:\\trygons\\private\\api_key.txt,
--key-file overrides). It is never printed, logged, put in the environment, or
written to any output; error text is scrubbed of it.
Python: the default `python` (pip install -r tools/requirements_api.txt, exact
pin). check.sh tests (tools/test_api_review.py) use a fake client: no SDK needed.
"""
import argparse
import difflib
import fnmatch
import json
import os
import re
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_KEY_FILE = r"C:\trygons\private\api_key.txt"
DEFAULT_OUT_DIR = r"C:\trygons\private\reviews"
DEFAULT_COST_FILE = r"C:\trygons\private\reviews\cost_total.json"

MODELS = {"opus": "claude-opus-5-5", "sonnet": "claude-sonnet-5-5"}

# USD per million tokens, price table dated 2026-11-09 (check the pricing page
# when models change). cache write = 1.25 x input (5 min), cache read = 0.1 x input.
PRICES_DATE = "2026-11-09"
PRICES = {  # model: (input, output)
    "claude-opus-5-5": (5.0, 25.0),
    "claude-sonnet-5-5": (3.0, 15.0),
}
BATCH_FACTOR = 0.5
CACHE_WRITE_FACTOR = 1.25
CACHE_READ_FACTOR = 0.10

CHARS_PER_TOKEN = 3.5
MAX_REVIEW_TOKENS = 60_000      # per call, incl. the cached rules + spec
MAX_FILE_CHARS = 120_000
MAX_TOKENS = 16000              # room for thinking + the findings text
SAFETY_HEADINGS = ("Rules", "Safety rules")
DEFAULT_SECTIONS = ("B", "T")

# Not sent (fnmatch on the repo path; '*' also crosses '/'). Extend with --exclude.
EXCLUDE_GLOBS = (
    "*.png", "*.jpg", "*.jpeg", "*.gif", "*.ico", "*.bmp", "*.webp", "*.heic",
    "*.svg", "*.pdf", "*.ogg", "*.wav", "*.mp3", "*.mp4", "*.mov",
    "*.bin", "*.hex", "*.elf", "*.zip", "*.onnx", "*.gguf", "*.ttf", "*.otf",
    "*.lock", "package-lock.json", "*/package-lock.json", "*.jsonl", "*.csv",
    "docs/progress_archive.md",
    "groundstation/lang/*", "academy/lang/*",
    "groundstation/assistant/index/*",
    "academy/tour_shots/*", "academy/tour_voice/*", "docs/screens/*",
    "docs/presentation/*", "test_results/*", "tests/assistant/*.json",
    "*/fixtures/*", "*/testdata/*", "*/test_data/*", "charts/*",
)

# Files whose change needs an OPUS safety review (boat commands / safety paths).
SAFETY_GLOBS = (
    "app/*", "hal/*", "Cpu*_Main.c", "*.ld", ".cproject",
    "gateway/stations.py", "gateway/aurix.py", "gateway/uav/*",
    "groundstation/app.py", "groundstation/gwclient.py",
    "groundstation/shorejoy.py", "groundstation/proto.py",
    "groundstation/link.py", "groundstation/uavfly.py", "groundstation/uavroute.py",
)

INSTRUCTIONS = (
    "You are an independent SAFETY reviewer of a boat-autopilot firmware repository. "
    "You are read only: you see only the material below and change nothing. "
    "Look for real defects only: throttle rising or staying up in an emergency, gear "
    "change while the modelled throttle is not idle, engine kill in CREWED, core 0 / "
    "CPU1 races, sim vs board differences, unit or sign errors, storm invariants that "
    "cannot fail, and any violation of the rules in the system block. "
    "Do not invent problems; if the diff alone cannot show a defect, say what you "
    "could not verify. Report HIGH / MED / LOW with file:line, the failure scenario "
    "and the fix, max about 800 words, plain English."
)


class ApiReviewError(Exception):
    pass


# ---------------------------------------------------------------- key
def read_key(path):
    try:
        with open(path, "r", encoding="utf-8-sig") as f:
            key = f.read().strip()
    except OSError as e:
        raise ApiReviewError("cannot read the key file (%s)" % type(e).__name__)
    if not key:
        raise ApiReviewError("the key file is empty")
    return key


def scrub(text, key):
    text = str(text)
    if key:
        text = text.replace(key, "<key>")
    return re.sub(r"sk-ant-[A-Za-z0-9_\-]+", "<key>", text)


# ---------------------------------------------------------------- input
def git(args):
    r = subprocess.run(["git", "-C", ROOT] + args, capture_output=True, check=False)
    if r.returncode != 0:
        raise ApiReviewError("git %s failed: %s" % (args[0], r.stderr.decode("utf-8", "replace").strip()[:200]))
    return r.stdout.decode("utf-8", "replace")


def est_tokens(text):
    return int(len(text) / CHARS_PER_TOKEN) + 1


def changed_files(commit):
    if ".." in commit:
        out = git(["diff", "--name-only", commit])
    else:
        out = git(["show", "--name-only", "--format=", commit])
    return [x.strip() for x in out.splitlines() if x.strip()]


def file_diff(commit, path):
    if ".." in commit:
        return git(["diff", commit, "--", path])
    return git(["show", "--format=", "--patch", commit, "--", path])


def matches(path, globs):
    return any(fnmatch.fnmatch(path, g) for g in globs)


def area_of(path):
    return path.split("/", 1)[0] + "/" if "/" in path else "(root)"


def choose_model(args, paths):
    """-> (model id, why, safety bool). --model overrides the choice."""
    hits = [p for p in paths if matches(p, SAFETY_GLOBS)]
    safety = bool(hits)
    if args.model:
        return MODELS.get(args.model, args.model), "--model", safety
    if safety:
        return (MODELS["opus"], "safety files in the diff: " + ", ".join(hits[:4]) +
                (" ..." if len(hits) > 4 else ""), True)
    return MODELS["sonnet"], "no boat / command / safety files in the diff", False


def _nl(t):
    return t if t.endswith("\n") else t + "\n"


def split_material(commit, paths, exclude, budget_tokens):
    """-> (chunks [(header, text)], excluded [path], notes). Every chunk <= budget."""
    kept, excluded = [], []
    for pth in paths:
        (excluded if matches(pth, exclude) else kept).append(pth)
    items, notes = [], []
    cap = int(budget_tokens * CHARS_PER_TOKEN)
    for pth in kept:
        txt = file_diff(commit, pth)
        if not txt.strip():
            continue
        if len(txt) > cap:
            notes.append("%s truncated (%d chars, cap %d)" % (pth, len(txt), cap))
            txt = txt[:cap] + "\n[TRUNCATED: %s was longer than one call allows]\n" % pth
        items.append((area_of(pth), pth, txt))
    total = sum(est_tokens(t) for _, _, t in items)
    if total <= budget_tokens:
        return ([("all areas", "".join(_nl(t) for _, _, t in items))] if items else []), excluded, notes
    areas = {}
    for a, pth, txt in items:
        areas.setdefault(a, []).append((pth, txt))
    chunks = []
    state = {"names": [], "text": [], "tok": 0}

    def flush():
        if state["text"]:
            chunks.append((", ".join(state["names"]), "".join(state["text"])))
        state["names"], state["text"], state["tok"] = [], [], 0
    for name in sorted(areas):
        files = areas[name]
        atok = sum(est_tokens(t) for _, t in files)
        if atok <= budget_tokens:
            if state["tok"] + atok > budget_tokens:
                flush()
            state["names"].append(name)
            state["text"].append("".join(_nl(t) for _, t in files))
            state["tok"] += atok
        else:
            flush()
            part, ptok, n = [], 0, 1
            for pth, txt in files:
                t = est_tokens(txt)
                if ptok + t > budget_tokens and part:
                    chunks.append(("%s (part %d)" % (name, n), "".join(part)))
                    part, ptok, n = [], 0, n + 1
                part.append(_nl(txt))
                ptok += t
            if part:
                chunks.append(("%s (part %d)" % (name, n), "".join(part)))
    flush()
    return chunks, excluded, notes


def truncate(text, limit, what):
    if len(text) <= limit:
        return text
    return text[:limit] + "\n\n[TRUNCATED: %s was %d characters, only the first %d are shown]\n" % (what, len(text), limit)


def claude_sections(text, headings=SAFETY_HEADINGS, letters=DEFAULT_SECTIONS):
    """The CLAUDE.md '## <heading>' blocks named in headings + the lettered
    sections ('## B. ...', '## T. ...') named in letters."""
    parts = re.split(r"(?m)^(?=## )", text)
    keep = []
    for p in parts:
        title = p.split("\n", 1)[0][3:].strip()
        if any(title == h or title.startswith(h) for h in headings) or \
           any(re.match(r"%s\. " % re.escape(l), title) for l in letters):
            keep.append(p.rstrip())
    if not keep:
        raise ApiReviewError("no safety-rules sections found in CLAUDE.md")
    return "\n\n".join(keep)


def get_rules(claude_md, letters):
    with open(claude_md, "r", encoding="utf-8") as f:
        return claude_sections(f.read(), letters=letters)


def progress_row(progress_md, step):
    """The PROGRESS.md row of a step: exact first cell, else the ONE row whose
    first cell contains the id ('PF3-r + PF2-r' for PF3-r); otherwise a clear
    message with the near matches."""
    cells = []
    with open(progress_md, "r", encoding="utf-8") as f:
        for line in f:
            m = re.match(r"\|\s*([^|]+?)\s*\|", line)
            if m and not set(m.group(1)) <= set("-: "):
                cells.append((m.group(1), line.strip()))
    for c, line in cells:
        if c == step:
            return line
    pat = re.compile(r"(?<![A-Za-z0-9])%s(?![A-Za-z0-9])" % re.escape(step), re.I)
    loose = [(c, l) for c, l in cells if pat.search(c)]
    if len(loose) == 1:
        return loose[0][1]
    if loose:
        raise ApiReviewError("step %s matches several PROGRESS.md rows: %s (use the exact id or --spec)" %
                             (step, "; ".join(c for c, _ in loose[:8])))
    near = difflib.get_close_matches(step, [c for c, _ in cells], n=6, cutoff=0.4)
    raise ApiReviewError("step %s: no row in PROGRESS.md%s" %
                         (step, "; near: " + "; ".join(near) if near else ""))


def get_spec(args):
    if args.spec:
        try:
            with open(args.spec, "r", encoding="utf-8") as f:
                return f.read()
        except OSError as e:
            raise ApiReviewError("cannot read the spec file (%s)" % type(e).__name__)
    if args.step:
        return progress_row(os.path.join(ROOT, "PROGRESS.md"), args.step)
    return ""


def build_request(args, rules, spec, material, model):
    """The ONE place the prompt is assembled: system = instructions + cached rules;
    user = spec + (focus) + the diff / files."""
    user = []
    if spec:
        user.append("STEP SPEC:\n" + spec)
    if args.focus:
        user.append("FOCUS: " + args.focus)
    if args.audit:
        user.append("QUESTION (read-only audit): " + args.audit)
    user.append(material)
    return {
        "model": model,
        "max_tokens": MAX_TOKENS,
        "system": [
            {"type": "text", "text": INSTRUCTIONS},
            {"type": "text", "text": "CLAUDE.md safety rules:\n\n" + rules,
             "cache_control": {"type": "ephemeral"}},
        ],
        "messages": [{"role": "user", "content": "\n\n".join(user)}],
    }


def get_files_material(names):
    out = []
    for n in names:
        p = n if os.path.isabs(n) else os.path.join(ROOT, n)
        try:
            with open(p, "r", encoding="utf-8", errors="replace") as f:
                txt = f.read()
        except OSError as e:
            raise ApiReviewError("cannot read %s (%s)" % (n, type(e).__name__))
        out.append("=== FILE %s ===\n%s" % (n, truncate(txt, MAX_FILE_CHARS, n)))
    return "\n\n".join(out)


# ---------------------------------------------------------------- cost
def cost_usd(model, usage, batch=False):
    if model not in PRICES:
        raise ApiReviewError("no price for model %s" % model)
    pin, pout = PRICES[model]
    inp = usage.get("input", 0)
    out = usage.get("output", 0)
    cr = usage.get("cache_read", 0)
    cw = usage.get("cache_write", 0)
    usd = (inp * pin + out * pout + cw * pin * CACHE_WRITE_FACTOR + cr * pin * CACHE_READ_FACTOR) / 1e6
    return usd * (BATCH_FACTOR if batch else 1.0)


def usage_of(u):
    g = lambda n: int(getattr(u, n, 0) or 0)
    return {"input": g("input_tokens"), "output": g("output_tokens"),
            "cache_read": g("cache_read_input_tokens"), "cache_write": g("cache_creation_input_tokens")}


def text_of(msg):
    return "".join(getattr(b, "text", "") for b in msg.content if getattr(b, "type", "") == "text")


class Spend:
    """Every API call of one review (also those before an error)."""

    def __init__(self):
        self.calls = []   # (model, usage, batch)

    def add(self, model, usage, batch):
        self.calls.append((model, usage, batch))

    def usd(self):
        return sum(cost_usd(m, u, b) for m, u, b in self.calls)

    def tokens(self):
        t = {"input": 0, "output": 0, "cache_read": 0, "cache_write": 0}
        for _, u, _ in self.calls:
            for k in t:
                t[k] += u.get(k, 0)
        return t


def month_key(now):
    return time.strftime("%Y-%m", time.localtime(now()))


def record_cost(path, label, spend, now):
    """Append the review to the month ledger; returns the month total."""
    if not spend.calls or not path:
        return None
    data = {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except (OSError, ValueError):
        data = {}
    if not isinstance(data, dict):
        data = {}
    months = data.setdefault("months", {})
    m = months.setdefault(month_key(now), {"total_usd": 0.0, "reviews": []})
    m["reviews"].append({"time": time.strftime("%Y-%m-%d %H:%M", time.localtime(now())),
                         "label": label, "models": sorted({c[0] for c in spend.calls}),
                         "calls": len(spend.calls), "usd": round(spend.usd(), 4),
                         "batch": any(c[2] for c in spend.calls)})
    m["total_usd"] = round(sum(r["usd"] for r in m["reviews"]), 4)
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=1)
    os.replace(tmp, path)
    return m["total_usd"]


def report_cost(args, label, spend, out, now):
    """Honest cost line over ALL calls made (also when the review failed)."""
    if not spend.calls:
        return
    t = spend.tokens()
    batch = all(c[2] for c in spend.calls)
    out("tokens: input %d, output %d, cache-read %d, cache-write %d | %d call(s) | cost $%.4f%s (prices %s)" %
        (t["input"], t["output"], t["cache_read"], t["cache_write"], len(spend.calls), spend.usd(),
         " (batch -50%)" if batch else "", PRICES_DATE))
    try:
        tot = record_cost(args.cost_file, label, spend, now)
    except OSError as e:
        out("api_review: cost file not written (%s)" % type(e).__name__)
        return
    if tot is not None:
        out("month total %s: $%.4f (%s)" % (month_key(now), tot, args.cost_file))


# ---------------------------------------------------------------- errors
def explain(e, key):
    name = type(e).__name__
    code = getattr(e, "status_code", None)
    if name == "AuthenticationError" or code in (401, 403):
        why = "authentication failed (check the key file)"
    elif code == 402 or "credit" in str(e).lower():
        why = "no API credit (billing)"
    elif name == "RateLimitError" or code == 429:
        why = "rate limited, try again later"
    elif name in ("APIConnectionError", "APITimeoutError"):
        why = "network error"
    elif code:
        why = "API error %s: %s" % (code, scrub(str(e), key)[:200])
    else:
        why = "%s: %s" % (name, scrub(str(e), key)[:200])
    return "api_review: " + why


def checked_text(msg, where):
    """The answer text, or an ERROR for an empty / cut answer."""
    stop = getattr(msg, "stop_reason", None) or "end_turn"
    text = text_of(msg)
    if stop == "max_tokens":
        raise ApiReviewError("%s: the answer was cut at max_tokens (stop_reason=%s, %d chars of text) - "
                             "no findings written" % (where, stop, len(text.strip())))
    if not text.strip():
        raise ApiReviewError("%s: the answer has no text (stop_reason=%s) - no findings written" % (where, stop))
    return text


# ---------------------------------------------------------------- output
def write_out(path, text, key):
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(scrub(text, key))


def out_paths(args, label):
    safe = re.sub(r"[^A-Za-z0-9._-]+", "_", label)[:60]
    first = os.path.join(args.out_dir, "review_%s.txt" % safe)
    return [first] + ([args.copy_to] if args.copy_to else [])


def merge_findings(header, parts):
    """parts: [(heading, text)]; one part = no sub-headings."""
    body = parts[0][1] if len(parts) == 1 else \
        "\n\n".join("## Area: %s\n\n%s" % (h, t.strip()) for h, t in parts)
    return header.rstrip() + "\n\n" + body.strip() + "\n"


def make_header(label, model, why, meta):
    areas = meta["areas"]
    ex = meta.get("excluded") or []
    lines = ["REVIEW %s | model %s (%s)" % (label, model, why),
             "calls: %d%s" % (len(areas), " (split by area: %s)" % "; ".join(areas) if len(areas) > 1 else ""),
             "excluded from the diff (%d): %s" % (
                 len(ex), (", ".join(ex[:25]) + (" ..." if len(ex) > 25 else "")) if ex else "none")]
    if meta.get("notes"):
        lines.append("notes: " + "; ".join(meta["notes"]))
    return "\n".join(lines)


def finish(args, label, text, key, out):
    for p in out_paths(args, label):
        write_out(p, text, key)
    out("findings: " + out_paths(args, label)[0])


def meta_path(args, bid):
    return os.path.join(args.out_dir, "batch_%s.json" % re.sub(r"[^A-Za-z0-9_]+", "_", bid))


# ---------------------------------------------------------------- main
def make_client(key):
    try:
        import anthropic
    except ImportError:
        raise ApiReviewError("the anthropic SDK is not installed (pip install -r tools/requirements_api.txt)")
    return anthropic.Anthropic(api_key=key, max_retries=2, timeout=600.0)


def poll_batch(client, bid, wait_min, sleep, now):
    deadline = now() + wait_min * 60
    while True:
        b = client.messages.batches.retrieve(bid)
        if b.processing_status == "ended":
            return b
        if now() >= deadline:
            return None
        sleep(30)


def collect(client, args, key, out, sleep, now, bid, spend):
    b = poll_batch(client, bid, args.wait, sleep, now)
    if b is None:
        out("api_review: batch %s not finished yet; resume with --collect %s" % (bid, bid))
        return 3
    try:
        with open(meta_path(args, bid), "r", encoding="utf-8") as f:
            meta = json.load(f)
    except (OSError, ValueError):
        meta = {}
    results = sorted(client.messages.batches.results(bid), key=lambda r: r.custom_id)
    label = meta.get("label") or (results[0].custom_id.split("__")[0] if results else bid)
    parts, models, err = [], set(), None
    for r in results:
        if r.result.type != "succeeded":
            err = "batch item %s %s" % (r.custom_id, r.result.type)
            continue
        msg = r.result.message
        m = msg.model if msg.model in PRICES else (meta.get("model") or MODELS["sonnet"])
        models.add(m)
        spend.add(m, usage_of(msg.usage), True)
        try:
            idx = int(r.custom_id.rsplit("__", 1)[-1])
        except ValueError:
            idx = len(parts)
        areas = meta.get("areas") or []
        head = areas[idx] if idx < len(areas) else r.custom_id
        try:
            parts.append((head, checked_text(msg, r.custom_id)))
        except ApiReviewError as e:
            err = scrub(e, key)
    if err or not parts:
        out("api_review: " + (err or "batch has no results"))
        report_cost(args, label, spend, out, now)
        return 1
    hdr = make_header(label, meta.get("model") or sorted(models)[0], meta.get("why", "batch"),
                      {"areas": meta.get("areas") or [p[0] for p in parts],
                       "excluded": meta.get("excluded"), "notes": meta.get("notes")})
    finish(args, label, merge_findings(hdr, parts), key, out)
    report_cost(args, label, spend, out, now)
    return 0


def run(argv, client_factory=None, out=print, sleep=None, now=None):
    sleep = sleep or time.sleep
    now = now or time.time
    ap = argparse.ArgumentParser(
        description="Direct Anthropic API review (API1 / API1-r)",
        epilog="Rule: a SAFETY review (boat / command files, or --urgent) is synchronous; a "
               "NON-urgent review goes through the Batch API at half price (wait --wait minutes, then "
               "resume with --collect). --batch forces the batch, --urgent forces synchronous.")
    ap.add_argument("commit", nargs="?", default=None, help="commit or range a..b (default HEAD)")
    ap.add_argument("--step", help="step id: the PROGRESS.md row is the spec (loose id match)")
    ap.add_argument("--spec", help="spec file")
    ap.add_argument("--focus", help="extra focus text")
    ap.add_argument("--model", default=None, help="opus | sonnet | a model id (default: by the files)")
    ap.add_argument("--batch", action="store_true", help="force the Batch API (50 %% price)")
    ap.add_argument("--urgent", action="store_true", help="force a synchronous call (default for safety reviews)")
    ap.add_argument("--wait", type=float, default=30, help="minutes to wait for a batch")
    ap.add_argument("--collect", metavar="BATCH_ID", help="collect a submitted batch")
    ap.add_argument("--audit", help="read-only question (with --files)")
    ap.add_argument("--files", help="comma list of files for --audit")
    ap.add_argument("--exclude", action="append", default=[], help="extra glob to leave out of the diff")
    ap.add_argument("--max-tokens-in", type=int, default=MAX_REVIEW_TOKENS,
                    help="input tokens per call incl. rules (default 60000)")
    ap.add_argument("--sections", default=",".join(DEFAULT_SECTIONS),
                    help="CLAUDE.md lettered sections sent besides Rules + Safety rules")
    ap.add_argument("--claude-md", default=os.path.join(ROOT, "CLAUDE.md"))
    ap.add_argument("--key-file", default=DEFAULT_KEY_FILE)
    ap.add_argument("--out-dir", default=DEFAULT_OUT_DIR)
    ap.add_argument("--cost-file", default=DEFAULT_COST_FILE, help="local month cost ledger (outside git)")
    ap.add_argument("--copy-to", help="also write the findings here")
    args = ap.parse_args(argv)
    key = None
    spend = Spend()
    label = args.commit or "review"
    try:
        if args.batch and args.urgent:
            raise ApiReviewError("--batch and --urgent exclude each other")
        key = read_key(args.key_file)
        client = (client_factory or make_client)(key)
        if args.collect:
            return collect(client, args, key, out, sleep, now, args.collect, spend)
        letters = tuple(s.strip() for s in args.sections.split(",") if s.strip())
        rules = get_rules(args.claude_md, letters)
        spec = get_spec(args)
        fixed = est_tokens(INSTRUCTIONS) + est_tokens(rules) + est_tokens(spec) + est_tokens(args.focus or "") + 800
        budget = args.max_tokens_in - fixed
        if budget < 4000:
            raise ApiReviewError("the rules + spec alone are ~%d tokens: no room under %d" % (fixed, args.max_tokens_in))
        exclude = EXCLUDE_GLOBS + tuple(args.exclude)
        if args.audit:
            if not args.files:
                raise ApiReviewError("--audit needs --files a,b,c")
            names = [f.strip() for f in args.files.split(",") if f.strip()]
            chunks = [("audit files", truncate(get_files_material(names), int(budget * CHARS_PER_TOKEN), "the files"))]
            excluded, notes, paths = [], [], names
            label = "audit_" + re.sub(r"\W+", "_", args.audit)[:30]
        else:
            commit = args.commit or "HEAD"
            label = git(["rev-parse", "--short", commit.split("..")[-1] or "HEAD"]).strip()
            paths = changed_files(commit)
            chunks, excluded, notes = split_material(commit, paths, exclude, budget)
            if not chunks:
                raise ApiReviewError("nothing to review: every changed file is excluded (%d) or empty" % len(excluded))
        model, why, safety = choose_model(args, [p for p in paths if not matches(p, exclude)])
        use_batch = args.batch or (not args.urgent and not safety)
        out("model: %s (%s) | %s | %d call(s)%s" % (model, why, "batch" if use_batch else "synchronous",
                                                   len(chunks), "" if len(chunks) == 1 else " (split by area)"))
        if excluded:
            out("excluded %d file(s) (images, data, lang, archive ...)" % len(excluded))
        if notes:
            out("notes: " + "; ".join(notes))
        reqs = []
        for head, text in chunks:
            body = text if args.audit else "DIFF OF %s (%s):\n%s" % (args.commit or "HEAD", head, text)
            reqs.append(build_request(args, rules, spec, body, model))
        meta = {"label": label, "model": model, "why": why, "areas": [h for h, _ in chunks],
                "excluded": excluded, "notes": notes}
        if use_batch:
            b = client.messages.batches.create(
                requests=[{"custom_id": "%s__%d" % (re.sub(r"[^A-Za-z0-9_-]", "_", label)[:40], i), "params": r}
                          for i, r in enumerate(reqs)])
            out("batch submitted: %s" % b.id)
            try:
                os.makedirs(args.out_dir, exist_ok=True)
                with open(meta_path(args, b.id), "w", encoding="utf-8") as f:
                    json.dump(meta, f)
            except OSError:
                pass
            return collect(client, args, key, out, sleep, now, b.id, spend)
        parts = []
        for (head, _), req in zip(chunks, reqs):
            msg = client.messages.create(**req)
            spend.add(model, usage_of(msg.usage), False)
            parts.append((head, checked_text(msg, "area " + head)))
        finish(args, label, merge_findings(make_header(label, model, why, meta), parts), key, out)
        report_cost(args, label, spend, out, now)
        return 0
    except ApiReviewError as e:
        out("api_review: " + scrub(e, key))
        report_cost(args, label, spend, out, now)
        return 1 if spend.calls else 2
    except Exception as e:  # SDK errors: one clear line, never the key
        out(explain(e, key))
        report_cost(args, label, spend, out, now)
        return 1


if __name__ == "__main__":
    sys.exit(run(sys.argv[1:]))
