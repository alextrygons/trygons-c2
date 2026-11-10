# tools/api_review.py - direct Anthropic API review (step API1)

Independent safety review of a commit through the Messages API (anthropic Python SDK
2.3.0, pinned in tools/requirements_api.txt; install into the default `python`:
`python -m pip install -r tools/requirements_api.txt`). Not `claude -p`: the model gets
ONLY the system block (instructions + the CLAUDE.md "Rules", "Safety rules" and sections
B, T, cached with cache_control), the step spec and the commit diff (cap 300 000
characters, a clear TRUNCATED note).

    python tools/api_review.py HEAD --step PF6 --focus "lost link" [--model sonnet]
    python tools/api_review.py a..b --spec spec.txt
    python tools/api_review.py HEAD --batch --wait 20     # Batch API, 50 % price
    python tools/api_review.py --collect msgbatch_xxx     # resume (exit 3 = not finished)
    python tools/api_review.py --audit "question" --files a.c,b.py   # only those files
    C:\trygons\private\review.bat HEAD --step PF6         # same thing

* Key: read only by the code from C:\trygons\private\api_key.txt (`--key-file`); never
  printed, logged, in the environment or in a findings file (scrubbed).
* Output: C:\trygons\private\reviews\review_<commit>.txt (`--out-dir`, `--copy-to`),
  tokens (input / output / cache-read / cache-write) and cost in USD on the console.
  Prices are in the script (PRICES_DATE); batch = 50 %.
* Errors (auth / credit / network / rate limit) = one line, exit 1; setup errors exit 2.
* Tests: tools/test_api_review.py (fake client, no network), in tools/check.sh.

## API1-r (2026-11-10): size cap, split, model per review, batch, month cost

The first real Opus run (commit 64521b1) cost $1.08 (181,279 input / 4,000 output tokens)
and wrote an EMPTY file: the answer was cut at max_tokens before any text. Fixes:

* **Empty / cut answer = error.** No text block, blank text or `stop_reason=max_tokens`:
  exit 1, `stop_reason` printed, NO findings file; the cost line still lists every call
  that was made. `MAX_TOKENS` is 16000 (room for thinking + the findings).
* **At most ~60k input tokens per call** (`--max-tokens-in`, estimate chars / 3.5, rules +
  spec included). Not sent: images, data, lockfiles, `docs/progress_archive.md`, lang dumps
  (`groundstation/lang`, `academy/lang`), the assistant index json, `academy/tour_shots`,
  `test_results/` ... (`EXCLUDE_GLOBS` in the tool, `--exclude GLOB` adds more). A bigger diff
  is SPLIT by area (`gateway/`, `groundstation/`, `tests/`, `docs/`, `tools/` ...; a huge area
  by files; one huge file is truncated with a note) into several calls and the findings are
  MERGED into one file with a header (model + why, calls, split, excluded files, notes) and
  one `## Area:` heading per call.
* **Model per review.** Default from the files: opus ONLY when the diff touches `app/`,
  `hal/`, `Cpu*_Main.c`, `gateway/stations.py`, `gateway/aurix.py`, `gateway/uav/`, or the
  station files that send commands (`groundstation/app.py`, `gwclient.py`, `shorejoy.py`,
  `proto.py`, `link.py`, `uavfly.py`, `uavroute.py`); sonnet for display / docs / tools /
  test-only steps. `--model opus|sonnet|ID` overrides. The model and the reason are printed.
* **Batch vs urgent.** A safety review is synchronous (it blocks the next step). A non-safety
  review goes through the Batch API (half price): the tool waits `--wait` minutes (default 30),
  then says `--collect msgbatch_...` (exit 3). `--urgent` forces synchronous, `--batch` forces
  the batch (the area names are kept in `batch_<id>.json` beside the findings).
* **Cost.** Every review prints the real cost of ALL its calls and the month total; the local
  ledger is `C:\trygons\private\reviews\cost_total.json` (`--cost-file`; per calendar month,
  one line per review; no key in it; outside git).
* **`--step ID`** also finds `PF3-r + PF2-r` for `PF3-r`: an exact first cell wins, else the one
  row whose first cell contains the id; several rows or none = a clear message (near matches).
* Tests: `tools/test_api_review.py` (fake client, no network). Reviewer models: see
  PROGRESS.md LOOP RULE 8.
