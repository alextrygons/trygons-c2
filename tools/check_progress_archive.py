#!/usr/bin/env python3
"""CL4a: docs/progress_archive.md must hold the moved text intact (manifest line count + sha256),
PROGRESS.md must link to it and stay small."""
import hashlib, re, sys, os
root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAX_LINES = 400
err = []
try:
    a = open(os.path.join(root, "docs", "progress_archive.md"), "rb").read()
    p = open(os.path.join(root, "PROGRESS.md"), "rb").read()
except OSError as e:
    print("progress archive: %s" % e); sys.exit(1)
# CRLF checkout (core.autocrlf) must not change the result: judge the LF text
a = a.replace(b"\r\n", b"\n")
p = p.replace(b"\r\n", b"\n")
mark = b"--- BEGIN MOVED TEXT ---\n"
if mark not in a:
    err.append("archive marker missing")
else:
    head, body = a.split(mark, 1)
    m = re.search(rb"ARCHIVE-MANIFEST: lines=(\d+) sha256=([0-9a-f]{64})", head)
    if not m:
        err.append("archive manifest missing")
    else:
        if body.count(b"\n") != int(m.group(1)):
            err.append("archive line count differs from the manifest")
        if hashlib.sha256(body).hexdigest() != m.group(2).decode():
            err.append("archive text differs from the manifest sha256 (edited or damaged)")
if b"docs/progress_archive.md" not in p:
    err.append("PROGRESS.md has no link to docs/progress_archive.md")
n = p.count(b"\n") + 1
if n >= MAX_LINES:
    err.append("PROGRESS.md has %d lines (limit %d): move finished tasks to the archive" % (n, MAX_LINES))
if err:
    print("PROGRESS ARCHIVE CHECK FAILED: " + "; ".join(err)); sys.exit(1)
print("progress archive OK (PROGRESS.md %d lines)" % n)
