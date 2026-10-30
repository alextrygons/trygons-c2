"""The version shown on the start-up screen and in Help -> About (step 13i). DISPLAY ONLY.

VERSION is set by hand (the owner) for a release; commit() = the git commit this copy runs from, read
straight from the .git folder (no git program, no process: the academy may import this - test_academy
AC1), "" when it is not a git clone.
"""
import os

VERSION = "2026.10"
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _read(path):
    try:
        with open(path, encoding="utf-8") as f:
            return f.read().strip()
    except (OSError, UnicodeDecodeError):
        return ""


def commit(repo=REPO, n=7):
    """The short commit id of HEAD ("" if unknown): .git/HEAD -> the branch ref, a loose ref file or
    packed-refs; a worktree / submodule ".git" file ("gitdir: ...") is followed too."""
    git = os.path.join(repo, ".git")
    if os.path.isfile(git):
        g = _read(git)
        if g.startswith("gitdir:"):
            git = os.path.normpath(os.path.join(repo, g[7:].strip()))
    head = _read(os.path.join(git, "HEAD"))
    if not head:
        return ""
    if not head.startswith("ref:"):
        return head[:n]
    ref = head[4:].strip()
    dirs = [git]
    common = _read(os.path.join(git, "commondir"))
    if common:
        dirs.append(os.path.normpath(os.path.join(git, common)))
    for d in dirs:
        sha = _read(os.path.join(d, *ref.split("/")))
        if sha:
            return sha[:n]
        for line in _read(os.path.join(d, "packed-refs")).splitlines():
            parts = line.split()
            if len(parts) == 2 and parts[1] == ref:
                return parts[0][:n]
    return ""


def text():
    """"2026.10 · abc1234" (the commit only when known)."""
    c = commit()
    return VERSION + (" · " + c if c else "")
