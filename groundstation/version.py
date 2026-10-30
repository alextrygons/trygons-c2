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


def _gitdir(repo):
    git = os.path.join(repo, ".git")
    if os.path.isfile(git):
        g = _read(git)
        if g.startswith("gitdir:"):
            git = os.path.normpath(os.path.join(repo, g[7:].strip()))
    return git


def _blob_sha(data):
    import hashlib
    return hashlib.sha1(b"blob %d\0" % len(data) + data).digest()


def dirty(repo=REPO, max_files=20000):
    """True when a tracked file differs from the commit's index (13i2, review 473694b L3) - read from
    .git/index like git does, no git program: a file whose size / mtime match the index is clean, else
    its content is hashed (also with CRLF -> LF, core.autocrlf). Deleted = dirty. Untracked files don't
    count (like git describe --dirty). Unknown index format / no index: False."""
    import struct
    try:
        with open(os.path.join(_gitdir(repo), "index"), "rb") as f:
            data = f.read()
    except OSError:
        return False
    if len(data) < 12 or data[:4] != b"DIRC":
        return False
    ver, count = struct.unpack(">II", data[4:12])
    if ver not in (2, 3, 4) or count > max_files:
        return False
    pos, prev = 12, b""
    try:
        for _ in range(count):
            start = pos
            (_cs, _cn, ms, _mn, _dev, _ino, mode, _uid, _gid, size) = struct.unpack(">10I", data[pos:pos + 40])
            sha = data[pos + 40:pos + 60]
            flags = struct.unpack(">H", data[pos + 60:pos + 62])[0]
            pos += 62
            if ver >= 3 and flags & 0x4000:
                ext = struct.unpack(">H", data[pos:pos + 2])[0]
                pos += 2
            else:
                ext = 0
            if ver == 4:                                        # prefix-compressed path
                strip = 0
                while True:
                    b = data[pos]
                    pos += 1
                    strip = (strip << 7) | (b & 0x7F)
                    if not b & 0x80:
                        break
                    strip += 1
                end = data.index(b"\0", pos)
                path = prev[:len(prev) - strip] + data[pos:end]
                pos = end + 1
            else:
                end = data.index(b"\0", pos)
                path = data[pos:end]
                pos = start + ((end - start) // 8 + 1) * 8       # 1..8 NUL bytes: a multiple of 8
            prev = path
            if flags & 0x8000 or ext & 0x4000 or (mode >> 12) not in (0o10, ):   # assume-valid / skip-worktree / not a file
                continue
            fp = os.path.join(repo, *path.decode("utf-8", "replace").split("/"))
            try:
                st = os.stat(fp)
            except OSError:
                return True                                     # a tracked file deleted
            if st.st_size == size and int(st.st_mtime) == ms:
                continue
            with open(fp, "rb") as g:
                body = g.read()
            if _blob_sha(body) != sha and _blob_sha(body.replace(b"\r\n", b"\n")) != sha:
                return True
    except (struct.error, ValueError, IndexError):
        return False
    return False


def commit(repo=REPO, n=7, mark=True):
    """The short commit id of HEAD ("" if unknown): .git/HEAD -> the branch ref, a loose ref file or
    packed-refs; a worktree / submodule ".git" file ("gitdir: ...") is followed too. mark: "+dirty"
    appended when the working tree differs from it (dirty(); 13i2) - the code is then NOT that commit."""
    c = _commit(repo, n)
    return c + "+dirty" if c and mark and dirty(repo) else c


def _commit(repo, n):
    git = _gitdir(repo)
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
