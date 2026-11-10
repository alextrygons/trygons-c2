#!/usr/bin/env python3
"""Tests for tools/api_review.py (API1 / API1-r): fake client, no network, no SDK."""
import contextlib
import io
import json
import os
import sys
import tempfile
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import api_review as ar  # noqa: E402

KEY = "sk-ant-api03-SECRETSECRET123"
fails = []
FILES = {}


def check(name, cond, info=""):
    print(("[PASS] " if cond else "[FAIL] ") + name + (" " + info if (info and not cond) else ""))
    if not cond:
        fails.append(name)


def msg(text="HIGH: x", i=1000, o=200, cr=0, cw=0, model="claude-opus-5-5", stop="end_turn"):
    content = [NS(type="text", text=text)] if text is not None else [NS(type="thinking", thinking="...")]
    return NS(content=content, model=model, stop_reason=stop,
              usage=NS(input_tokens=i, output_tokens=o, cache_read_input_tokens=cr,
                       cache_creation_input_tokens=cw))


class Fake:
    def __init__(self, raises=None, answer=None):
        self.calls = []
        self.raises = raises
        self.answer = answer
        self.created = []
        self.polls = 0
        self.ended_after = 2
        outer = self

        class Batches:
            def create(s, requests):
                outer.created.append(requests)
                return NS(id="msgbatch_1")

            def retrieve(s, bid):
                outer.polls += 1
                return NS(processing_status="ended" if outer.polls >= outer.ended_after else "in_progress")

            def results(s, bid):
                return [NS(custom_id="abc1234__%d" % i, result=NS(type="succeeded", message=msg("BATCHED%d" % i, 100, 50)))
                        for i in range(len(outer.created[0]) if outer.created else 1)]
        self.messages = NS(create=self._create, batches=Batches())

    def _create(self, **kw):
        self.calls.append(kw)
        if self.raises:
            raise self.raises
        return self.answer if self.answer is not None else msg()


class Echo(Fake):
    def _create(self, **kw):
        return msg("leak " + KEY)


class AuthenticationError(Exception):
    status_code = 401


class APIConnectionError(Exception):
    pass


class RateLimitError(Exception):
    status_code = 429


class Other(Exception):
    status_code = 400


def fake_git_on(files):
    FILES.clear()
    FILES.update(files)
    ar.changed_files = lambda c: list(FILES)
    ar.file_diff = lambda c, pth: FILES[pth]
    ar.git = lambda a: "abc1234\n"


def big(n):
    return "+" + "x" * n + "\n"


def main():
    d = tempfile.mkdtemp(prefix="trygons_apirev_")
    kf = os.path.join(d, "key.txt")
    open(kf, "w").write(KEY + "\n")
    cm = os.path.join(d, "CLAUDE.md")
    open(cm, "w", encoding="utf-8").write(
        "# T\n\n## Architecture\nSECRET-ARCH\n\n## Rules\nR1\n\n## Safety rules - never\nS1\n\n"
        "## B. Lost link\nBB\n\n## T. DroneCAN\nTT\n\n## Q. Gateway\nQQ-OTHER\n")
    pr = os.path.join(d, "spec.txt")
    open(pr, "w").write("SPEC-TEXT")
    od = os.path.join(d, "out")
    cf = os.path.join(d, "cost.json")
    lines = []
    base = ["--key-file", kf, "--claude-md", cm, "--out-dir", od, "--spec", pr, "--cost-file", cf]
    base3 = ["--key-file", kf, "--claude-md", cm, "--cost-file", cf]

    fake_git_on({"app/x.c": "+int a;\n"})
    f = Fake()
    rc = ar.run(["HEAD"] + base + ["--focus", "FOCUS-X"], lambda k: f, lines.append)
    req = f.calls[0]
    sysblocks = req["system"]
    systext = "\n".join(b["text"] for b in sysblocks)
    user = req["messages"][0]["content"]
    check("exit 0", rc == 0)
    check("caching block present", any("cache_control" in b and "R1" in b["text"] for b in sysblocks))
    check("rules: Rules, Safety, B, T", all(x in systext for x in ("R1", "S1", "BB", "TT")))
    check("rules: nothing else from CLAUDE.md", "SECRET-ARCH" not in systext + user and "QQ-OTHER" not in systext + user)
    check("user = spec + focus + diff", "SPEC-TEXT" in user and "FOCUS-X" in user and "DIFF OF HEAD" in user)
    check("safety file (app/) -> opus, synchronous", req["model"] == "claude-opus-5-5" and not f.created)
    check("no tools sent", "tools" not in req)

    allout = "\n".join(lines) + systext + user + repr(req)
    for p in os.listdir(od):
        allout += open(os.path.join(od, p), encoding="utf-8").read()
    allout += open(cf, encoding="utf-8").read()
    check("key never in output/prompt/findings/ledger", KEY not in allout and "SECRETSECRET" not in allout)
    check("findings file written", any(p.startswith("review_") for p in os.listdir(od)))
    check("tokens + cost printed", any("cache-read" in x and "$" in x for x in lines))

    lines.clear()
    ar.run(["HEAD"] + base, lambda k: Echo(), lines.append)
    txt = "".join(open(os.path.join(od, p), encoding="utf-8").read() for p in os.listdir(od))
    check("echoed key scrubbed from findings", KEY not in txt)

    u = {"input": 1_000_000, "output": 1_000_000, "cache_read": 1_000_000, "cache_write": 1_000_000}
    check("cost opus", abs(ar.cost_usd("claude-opus-5-5", u) - (5 + 25 + 6.25 + 0.5)) < 1e-9)
    check("cost batch half", abs(ar.cost_usd("claude-opus-5-5", u, True) - (5 + 25 + 6.25 + 0.5) / 2) < 1e-9)
    check("cost sonnet", abs(ar.cost_usd("claude-sonnet-5-5", {"input": 1_000_000, "output": 0}) - 3.0) < 1e-9)

    t = ar.truncate("x" * 100, 10, "the diff")
    check("truncation note", t.startswith("x" * 10) and "TRUNCATED" in t and "100" in t)
    check("no truncation when small", ar.truncate("abc", 10, "d") == "abc")

    f = Fake()
    ar.run(["HEAD"] + base + ["--model", "sonnet"], lambda k: f, lines.append)
    check("sonnet model override", f.calls[0]["model"] == "claude-sonnet-5-5")

    nf = os.path.join(d, "a.c")
    open(nf, "w").write("int AUDITED;")
    f = Fake()
    ar.run(["--audit", "why?", "--files", nf, "--model", "sonnet", "--urgent"] + base3 + ["--out-dir", od],
           lambda k: f, lines.append)
    u2 = f.calls[0]["messages"][0]["content"]
    check("audit sends the named file + question only", "AUDITED" in u2 and "why?" in u2 and "DIFF OF" not in u2)
    lines.clear()
    rc = ar.run(["--audit", "q"] + base3, lambda k: Fake(), lines.append)
    check("audit without files = error", rc == 2)

    f = Fake()
    lines.clear()
    rc = ar.run(["HEAD", "--batch", "--wait", "5"] + base, lambda k: f, lines.append,
                sleep=lambda s: None, now=lambda: 0)
    check("batch submit + poll + result", rc == 0 and f.created and any("batch -50%" in x for x in lines))
    check("batch request has caching block",
          any("cache_control" in b for b in f.created[0][0]["params"]["system"]))
    f = Fake()
    f.ended_after = 99
    tt = [0]

    def now():
        tt[0] += 400
        return tt[0]
    lines.clear()
    rc = ar.run(["--collect", "msgbatch_1", "--wait", "1"] + base3 + ["--out-dir", od], lambda k: f,
                lines.append, sleep=lambda s: None, now=now)
    check("collect not finished = exit 3 + hint", rc == 3 and any("--collect msgbatch_1" in x for x in lines))
    f = Fake()
    f.ended_after = 1
    rc = ar.run(["--collect", "msgbatch_1"] + base3 + ["--out-dir", od], lambda k: f, lines.append,
                sleep=lambda s: None, now=lambda: 0)
    check("collect finished", rc == 0)

    for exc, word in ((AuthenticationError("bad " + KEY), "authentication"),
                      (APIConnectionError("net " + KEY), "network"),
                      (RateLimitError("r"), "rate limited"),
                      (Other("credit balance too low " + KEY), "credit"),
                      (Other("weird " + KEY), "API error 400")):
        lines.clear()
        rc = ar.run(["HEAD"] + base, lambda k, e=exc: Fake(raises=e), lines.append)
        check("error %s: exit 1, no key, no findings line" % word,
              rc == 1 and word in lines[-1] and KEY not in "".join(lines) and
              not any(x.startswith("findings:") for x in lines), str(lines))
    lines.clear()
    rc = ar.run(["HEAD", "--key-file", os.path.join(d, "none.txt"), "--claude-md", cm],
                lambda k: Fake(), lines.append)
    check("missing key file = clear line", rc == 2 and len(lines) == 1 and "key file" in lines[0])
    open(kf, "w").write("  \n")
    lines.clear()
    rc = ar.run(["HEAD"] + base, lambda k: Fake(), lines.append)
    check("empty key file", rc == 2)
    open(kf, "w").write(KEY + "\n")

    # ---- API1-r (a): empty / cut answer = error, no file, honest cost
    fake_git_on({"app/x.c": "+int a;\n"})
    for label, m in (("no text block", msg(None)), ("blank text", msg("  ")),
                     ("max_tokens", msg("partial", stop="max_tokens"))):
        od2 = os.path.join(d, "out_" + label.replace(" ", "_"))
        lines.clear()
        rc = ar.run(["HEAD"] + base[:2] + ["--claude-md", cm, "--spec", pr, "--out-dir", od2, "--cost-file", cf],
                    lambda k, m=m: Fake(answer=m), lines.append)
        check("%s: exit 1, no findings file" % label,
              rc == 1 and (not os.path.exists(od2) or not os.listdir(od2)), str(lines))
        check("%s: stop_reason + cost printed" % label,
              any("stop_reason=" in x for x in lines) and any("cost $" in x for x in lines), str(lines))
    check("max_tokens raised", ar.MAX_TOKENS >= 16000)

    # (b) exclusion + split + merge
    fake_git_on({"app/a.c": big(100), "gateway/g.py": big(100), "groundstation/lang/el.txt": big(900000),
                 "docs/progress_archive.md": big(900000), "x.png": "bin", "tools/z.lock": big(500000),
                 "groundstation/assistant/index/manual_el.json": big(900000)})
    f = Fake()
    lines.clear()
    rc = ar.run(["HEAD", "--urgent"] + base, lambda k: f, lines.append)
    send = "\n".join(c["messages"][0]["content"] for c in f.calls)
    check("excluded files never sent", rc == 0 and len(f.calls) == 1 and len(send) < 5000, str(len(send)))
    txt = open(os.path.join(od, "review_abc1234.txt"), encoding="utf-8").read()
    check("header lists the exclusions", "excluded from the diff (5)" in txt and "progress_archive.md" in txt, txt[:300])

    fake_git_on({"app/a.c": big(150000), "gateway/g.py": big(150000), "tests/t.py": big(150000),
                 "docs/d.md": big(10000), "tools/z.py": big(10000)})
    f = Fake()
    lines.clear()
    rc = ar.run(["HEAD", "--urgent"] + base, lambda k: f, lines.append)
    sizes = [ar.est_tokens(c["messages"][0]["content"]) + ar.est_tokens("".join(b["text"] for b in c["system"]))
             for c in f.calls]
    check("big diff is split (>1 call), each <= 60k tokens", len(f.calls) > 1 and max(sizes) <= 60000, str(sizes))
    txt = open(os.path.join(od, "review_abc1234.txt"), encoding="utf-8").read()
    check("merged file, headed per area", txt.count("## Area:") == len(f.calls) and "split by area" in txt, txt[:400])
    sent = "\n".join(c["messages"][0]["content"] for c in f.calls)
    check("every kept file sent once", sent.count("x" * 150000) == 3 and sent.count("x" * 10000) >= 2)
    check("cost line sums all calls", any("%d call(s)" % len(f.calls) in x for x in lines), str(lines))

    fake_git_on({"app/a.c": big(900000)})
    f = Fake()
    ar.run(["HEAD", "--urgent"] + base, lambda k: f, lines.append)
    check("one huge file is truncated to the cap",
          len(f.calls) == 1 and "TRUNCATED" in f.calls[0]["messages"][0]["content"])
    fake_git_on({"x.png": "bin"})
    lines.clear()
    rc = ar.run(["HEAD"] + base, lambda k: Fake(), lines.append)
    check("everything excluded = error", rc == 2 and any("nothing to review" in x for x in lines))

    # (c) model by files
    for files, model, why in (
            (["app/servo.c"], "claude-opus-5-5", "app"), (["hal/can_act.c"], "claude-opus-5-5", "hal"),
            (["Cpu0_Main.c"], "claude-opus-5-5", "Cpu"), (["gateway/stations.py"], "claude-opus-5-5", "stations"),
            (["gateway/uav/allow.py"], "claude-opus-5-5", "uav allow"),
            (["groundstation/app.py"], "claude-opus-5-5", "station sends commands"),
            (["groundstation/trendpanel.py", "docs/x.md", "tools/a.py", "tests/b.py"], "claude-sonnet-5-5",
             "display/docs/tools/tests")):
        m_, w_, s_ = ar.choose_model(NS(model=None), files)
        check("model by files: %s" % why, m_ == model, m_)
    check("--model overrides", ar.choose_model(NS(model="sonnet"), ["app/servo.c"])[0] == "claude-sonnet-5-5")
    fake_git_on({"docs/x.md": "+d\n"})
    f = Fake()
    lines.clear()
    ar.run(["HEAD", "--urgent"] + base, lambda k: f, lines.append)
    check("model and reason printed", f.calls[0]["model"] == "claude-sonnet-5-5" and
          any(x.startswith("model: claude-sonnet-5-5") and "no boat" in x for x in lines), str(lines))

    # (d) batch / urgent
    f = Fake()
    lines.clear()
    ar.run(["HEAD"] + base, lambda k: f, lines.append, sleep=lambda s: None, now=lambda: 0)
    check("non-safety default = batch (no sync call)", f.created and not f.calls and any("batch" in x for x in lines))
    fake_git_on({"app/x.c": "+int a;\n"})
    f = Fake()
    ar.run(["HEAD"] + base, lambda k: f, lines.append)
    check("safety default = synchronous", f.calls and not f.created)
    f = Fake()
    ar.run(["HEAD", "--batch"] + base, lambda k: f, lines.append, sleep=lambda s: None, now=lambda: 0)
    check("--batch forces the batch on a safety diff", f.created and not f.calls)
    fake_git_on({"docs/x.md": "+d\n"})
    f = Fake()
    ar.run(["HEAD", "--urgent"] + base, lambda k: f, lines.append)
    check("--urgent forces synchronous", f.calls and not f.created)
    check("--batch + --urgent = error", ar.run(["HEAD", "--batch", "--urgent"] + base, lambda k: Fake(), lines.append) == 2)
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf):
            ar.run(["--help"], lambda k: Fake(), lines.append)
    except SystemExit:
        pass
    check("--help states the rule", "SAFETY review" in buf.getvalue() and "Batch API" in buf.getvalue())

    # (e) cost ledger, per calendar month
    cf2 = os.path.join(d, "ledger", "cost_total.json")
    b2 = ["--key-file", kf, "--claude-md", cm, "--out-dir", od, "--spec", pr, "--cost-file", cf2]
    fake_git_on({"app/x.c": "+int a;\n"})
    lines.clear()
    ar.run(["HEAD"] + b2, lambda k: Fake(), lines.append, now=lambda: 1_700_000_000)
    ar.run(["HEAD"] + b2, lambda k: Fake(), lines.append, now=lambda: 1_700_000_100)
    ar.run(["HEAD"] + b2, lambda k: Fake(), lines.append, now=lambda: 1_705_000_000)
    led = json.load(open(cf2, encoding="utf-8"))["months"]
    one = round(ar.cost_usd("claude-opus-5-5", {"input": 1000, "output": 200}), 4)
    mk = sorted(led)
    check("ledger: per month, per review lines",
          len(mk) == 2 and len(led[mk[0]]["reviews"]) == 2 and abs(led[mk[0]]["total_usd"] - 2 * one) < 1e-3, str(led))
    check("month total printed", any(x.startswith("month total") for x in lines))
    check("ledger has no key", KEY not in open(cf2, encoding="utf-8").read())

    # (f) --step loose match
    pm = os.path.join(d, "PROGRESS.md")
    open(pm, "w", encoding="utf-8").write(
        "| Step | What |\n|---|---|\n| PF4 | four |\n| PF3-r + PF2-r | fixes |\n"
        "| PF5-r | five |\n| AB1 | a |\n| AB1-r | b |\n")
    check("step exact", "four" in ar.progress_row(pm, "PF4"))
    check("step loose: 'PF3-r + PF2-r' for PF3-r and PF2-r",
          "fixes" in ar.progress_row(pm, "PF3-r") and "fixes" in ar.progress_row(pm, "PF2-r"))
    check("step exact wins over loose", "| a |" in ar.progress_row(pm, "AB1"))
    try:
        ar.progress_row(pm, "PF9-r")
        check("step unknown = error", False)
    except ar.ApiReviewError as e:
        check("step unknown = clear message", "no row" in str(e), str(e))
    try:
        ar.progress_row(pm, "PF")
        check("step ambiguous = error", False)
    except ar.ApiReviewError as e:
        check("step ambiguous lists the rows", "several" in str(e) or "no row" in str(e), str(e))

    print("API REVIEW TESTS %s" % ("FAILED" if fails else "OK"))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
