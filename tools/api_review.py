#!/usr/bin/env python3
"""Direct Anthropic API review tool (step API1, owner 2026-11-09).

Independent SAFETY review of a commit (or range) through the Messages API with
the `anthropic` SDK - NOT `claude -p`, no Claude Code, no tools: the model sees
ONLY (1) the CLAUDE.md safety-rules text (system block, cache_control = prompt
caching), (2) the step spec, (3) the commit's diff (size capped).

  python tools/api_review.py HEAD --step PF6 --focus "lost link"
  python tools/api_review.py abc123..def456 --spec docs/spec.txt --model sonnet
  python tools/api_review.py HEAD --batch --wait 20      # Batch API, 50 % price
  python tools/api_review.py --collect msgbatch_xxx       # resume a batch
  python tools/api_review.py --audit "Is X ever Y?" --files a.c,b.c

The API key is read ONLY from the key file (default C:\\trygons\\private\\api_key.txt,
--key-file overrides). It is never printed, logged, put in the environment, or
written to any output; error text is scrubbed of it.
Python: the default `python` (pip install -r tools/requirements_api.txt, exact
pin). check.sh tests (tools/test_api_review.py) use a fake client: no SDK needed.
"""
import argparse
import os
import re
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_KEY_FILE = r"C:\trygons\private\api_key.txt"
DEFAULT_OUT_DIR = r"C:\trygons\private\reviews"

MODELS = {"opus": "claude-opus-5-5", "sonnet": "claude-sonnet-5-5"}
DEFAULT_MODEL = MODELS["opus"]

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

MAX_DIFF_CHARS = 300_000
MAX_FILE_CHARS = 120_000
MAX_TOKENS = 4000
SAFETY_HEADINGS = ("Rules", "Safety rules")
DEFAULT_SECTIONS = ("B", "T")

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


def get_diff(commit):
    if ".." in commit:
        out = git(["diff", "--stat", "--patch", commit])
    else:
        out = git(["show", "--stat", "--patch", "--format=fuller", commit])
    return out


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
    with open(progress_md, "r", encoding="utf-8") as f:
        for line in f:
            if re.match(r"\|\s*%s\s*\|" % re.escape(step), line):
                return line.strip()
    raise ApiReviewError("step %s: no row in PROGRESS.md" % step)


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


def usage_line(model, usage, batch):
    return ("tokens: input %d, output %d, cache-read %d, cache-write %d | cost $%.4f%s (prices %s)" %
            (usage["input"], usage["output"], usage["cache_read"], usage["cache_write"],
             cost_usd(model, usage, batch), " (batch -50%)" if batch else "", PRICES_DATE))


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


# ---------------------------------------------------------------- main
def make_client(key):
    try:
        import anthropic
    except ImportError:
        raise ApiReviewError("the anthropic SDK is not installed (pip install -r tools/requirements_api.txt)")
    return anthropic.Anthropic(api_key=key, max_retries=2, timeout=600.0)


def finish(args, label, model, text, usage, batch, key, out):
    for p in out_paths(args, label):
        write_out(p, text, key)
    out("findings: " + out_paths(args, label)[0])
    out(usage_line(model, usage, batch))


def poll_batch(client, bid, wait_min, sleep, now):
    deadline = now() + wait_min * 60
    while True:
        b = client.messages.batches.retrieve(bid)
        if b.processing_status == "ended":
            return b
        if now() >= deadline:
            return None
        sleep(30)


def collect(client, args, key, out, sleep, now, bid, model):
    b = poll_batch(client, bid, args.wait, sleep, now)
    if b is None:
        out("api_review: batch %s not finished yet; resume with --collect %s" % (bid, bid))
        return 3
    for r in client.messages.batches.results(bid):
        if r.result.type != "succeeded":
            out("api_review: batch item %s %s" % (r.custom_id, r.result.type))
            return 1
        msg = r.result.message
        finish(args, r.custom_id, msg.model if msg.model in PRICES else model,
               text_of(msg), usage_of(msg.usage), True, key, out)
    return 0


def run(argv, client_factory=None, out=print, sleep=None, now=None):
    import time
    sleep = sleep or time.sleep
    now = now or time.time
    ap = argparse.ArgumentParser(description="Direct Anthropic API review (API1)")
    ap.add_argument("commit", nargs="?", default=None, help="commit or range a..b (default HEAD)")
    ap.add_argument("--step", help="step id: the PROGRESS.md row is the spec")
    ap.add_argument("--spec", help="spec file")
    ap.add_argument("--focus", help="extra focus text")
    ap.add_argument("--model", default="opus", help="opus | sonnet | a model id")
    ap.add_argument("--batch", action="store_true", help="Batch API (50 %% price)")
    ap.add_argument("--wait", type=float, default=30, help="minutes to wait for a batch")
    ap.add_argument("--collect", metavar="BATCH_ID", help="collect a submitted batch")
    ap.add_argument("--audit", help="read-only question (with --files)")
    ap.add_argument("--files", help="comma list of files for --audit")
    ap.add_argument("--sections", default=",".join(DEFAULT_SECTIONS),
                    help="CLAUDE.md lettered sections sent besides Rules + Safety rules")
    ap.add_argument("--claude-md", default=os.path.join(ROOT, "CLAUDE.md"))
    ap.add_argument("--key-file", default=DEFAULT_KEY_FILE)
    ap.add_argument("--out-dir", default=DEFAULT_OUT_DIR)
    ap.add_argument("--copy-to", help="also write the findings here")
    args = ap.parse_args(argv)
    model = MODELS.get(args.model, args.model)
    key = None
    try:
        key = read_key(args.key_file)
        client = (client_factory or make_client)(key)
        if args.collect:
            return collect(client, args, key, out, sleep, now, args.collect, model)
        letters = tuple(s.strip() for s in args.sections.split(",") if s.strip())
        rules = get_rules(args.claude_md, letters)
        if args.audit:
            if not args.files:
                raise ApiReviewError("--audit needs --files a,b,c")
            material = get_files_material([f.strip() for f in args.files.split(",") if f.strip()])
            label = "audit_" + re.sub(r"\W+", "_", args.audit)[:30]
        else:
            commit = args.commit or "HEAD"
            material = "DIFF OF %s:\n%s" % (commit, truncate(get_diff(commit), MAX_DIFF_CHARS, "the diff"))
            label = git(["rev-parse", "--short", commit.split("..")[-1] or "HEAD"]).strip()
        spec = get_spec(args)
        req = build_request(args, rules, spec, material, model)
        if args.batch:
            b = client.messages.batches.create(requests=[{"custom_id": label, "params": req}])
            out("batch submitted: %s" % b.id)
            return collect(client, args, key, out, sleep, now, b.id, model)
        msg = client.messages.create(**req)
        finish(args, label, model, text_of(msg), usage_of(msg.usage), False, key, out)
        return 0
    except ApiReviewError as e:
        out("api_review: " + scrub(e, key))
        return 2
    except Exception as e:  # SDK errors: one clear line, never the key
        out(explain(e, key))
        return 1


if __name__ == "__main__":
    sys.exit(run(sys.argv[1:]))
