#!/usr/bin/env python3
"""Tests for tools/api_review.py (API1): fake client, no network, no SDK."""
import os
import sys
import tempfile
from types import SimpleNamespace as NS

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import api_review as ar  # noqa: E402

KEY = "sk-ant-api03-SECRETSECRET123"
fails = []


def check(name, cond, info=""):
    print(("[PASS] " if cond else "[FAIL] ") + name + (" " + info if (info and not cond) else ""))
    if not cond:
        fails.append(name)


def msg(text="HIGH: x", i=1000, o=200, cr=0, cw=0, model="claude-opus-5-5"):
    return NS(content=[NS(type="text", text=text)], model=model,
              usage=NS(input_tokens=i, output_tokens=o, cache_read_input_tokens=cr,
                       cache_creation_input_tokens=cw))


class Fake:
    def __init__(self, raises=None):
        self.calls = []
        self.raises = raises
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
                return [NS(custom_id="abc1234", result=NS(type="succeeded", message=msg("BATCHED", 100, 50)))]
        self.messages = NS(create=self._create, batches=Batches())

    def _create(self, **kw):
        self.calls.append(kw)
        if self.raises:
            raise self.raises
        return msg()


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
    lines = []
    base = ["--key-file", kf, "--claude-md", cm, "--out-dir", od, "--spec", pr]
    base3 = ["--key-file", kf, "--claude-md", cm]

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
    check("default model opus", req["model"] == "claude-opus-5-5")
    check("no tools sent", "tools" not in req)

    allout = "\n".join(lines) + systext + user + repr(req)
    for p in os.listdir(od):
        allout += open(os.path.join(od, p), encoding="utf-8").read()
    check("key never in output/prompt/findings", KEY not in allout and "SECRETSECRET" not in allout)
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
    check("sonnet model", f.calls[0]["model"] == "claude-sonnet-5-5")

    nf = os.path.join(d, "a.c")
    open(nf, "w").write("int AUDITED;")
    f = Fake()
    ar.run(["--audit", "why?", "--files", nf] + base3 + ["--out-dir", od], lambda k: f, lines.append)
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
        check("error %s: one line, exit 1, no key" % word,
              rc == 1 and len(lines) == 1 and word in lines[0] and KEY not in lines[0],
              lines[0] if lines else "")
    lines.clear()
    rc = ar.run(["HEAD", "--key-file", os.path.join(d, "none.txt"), "--claude-md", cm],
                lambda k: Fake(), lines.append)
    check("missing key file = clear line", rc == 2 and len(lines) == 1 and "key file" in lines[0])
    open(kf, "w").write("  \n")
    lines.clear()
    rc = ar.run(["HEAD"] + base, lambda k: Fake(), lines.append)
    check("empty key file", rc == 2)

    print("API REVIEW TESTS %s" % ("FAILED" if fails else "OK"))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
