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
