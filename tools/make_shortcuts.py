"""Desktop shortcuts with the app icon (step 13i): "Trygons C2 Demo" and "Trygons Ακαδημία".

    python tools/make_shortcuts.py                 the Desktop of this user (ONLY when the owner wants them)
    python tools/make_shortcuts.py --dest FOLDER   another folder (the tests: a temporary one)
    python tools/make_shortcuts.py --dry-run       only print what would be made

Windows: .lnk files (made through PowerShell's WScript.Shell) that start pythonw (no console window):
  Trygons C2 Demo     pythonw demo\\demo.py start --splash   (= START_DEMO.bat: the start-up screen, then the demo)
  Trygons Ακαδημία    pythonw academy\\academy_app.py         (the academy: simulator only)
icon assets\\trygons_c2.ico (tools/make_icon.py). Other systems: .desktop files (python3, the PNG icon).
Nothing else is changed; running it again replaces the two shortcuts. No test ever writes the real Desktop:
make() needs its folder given.
"""
import base64
import os
import shutil
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ICO = os.path.normpath(os.path.join(REPO, "assets", "trygons_c2.ico"))
PNG = os.path.normpath(os.path.join(REPO, "assets", "trygonsdrones_mark.png"))
NO_WINDOW = 0x08000000 if os.name == "nt" else 0
# (file name, script, arguments, working folder)
SHORTCUTS = (
    ("Trygons C2 Demo", os.path.normpath(os.path.join(REPO, "demo", "demo.py")), ["start", "--splash"],
     os.path.normpath(REPO)),
    ("Trygons Ακαδημία", os.path.normpath(os.path.join(REPO, "academy", "academy_app.py")), [],
     os.path.normpath(os.path.join(REPO, "academy"))),
)


def pythonw():
    """pythonw.exe next to this Python (or MSYS2's / on the PATH); python itself elsewhere."""
    if os.name != "nt":
        return sys.executable or shutil.which("python3") or "python3"
    for cand in (os.path.join(os.path.dirname(sys.executable), "pythonw.exe"), r"C:\msys64\ucrt64\bin\pythonw.exe",
                 shutil.which("pythonw") or ""):
        if cand and os.path.exists(cand):
            return cand
    return sys.executable


def desktop():
    """This user's Desktop folder (also when OneDrive moved it)."""
    if os.name == "nt":
        try:
            r = subprocess.run(["powershell", "-NoProfile", "-Command", "[Environment]::GetFolderPath('Desktop')"],
                               capture_output=True, text=True, timeout=20, creationflags=NO_WINDOW)
            p = r.stdout.strip()
            if p and os.path.isdir(p):
                return p
        except (OSError, subprocess.SubprocessError):
            pass
    return os.path.join(os.path.expanduser("~"), "Desktop")


def plan(dest, exe=None):
    """What would be made: [{path, target, args, workdir, icon}]."""
    exe = os.path.normpath(exe or pythonw())
    dest = os.path.normpath(dest)
    out = []
    for name, script, args, cwd in SHORTCUTS:
        if os.name == "nt":
            out.append({"path": os.path.join(dest, name + ".lnk"), "target": exe,
                        "args": " ".join(['"%s"' % script] + args), "workdir": cwd, "icon": ICO})
        else:
            out.append({"path": os.path.join(dest, name + ".desktop"), "target": exe,
                        "args": " ".join(["'%s'" % script] + args), "workdir": cwd, "icon": PNG})
    return out


def _ps_quote(s):
    return "'" + s.replace("'", "''") + "'"


def make(dest, exe=None, dry=False):
    """Makes the shortcuts in the folder `dest` (required: never a default). Returns the plan."""
    if not dest:
        raise ValueError("no folder given")
    items = plan(dest, exe)
    if dry:
        return items
    os.makedirs(dest, exist_ok=True)
    if os.name == "nt":
        # WScript.Shell cannot save a Greek file name: an ASCII name first, renamed by Python afterwards
        tmp = [os.path.join(dest, "_trygons_shortcut_%d.lnk" % i) for i in range(len(items))]
        lines = ["$s = New-Object -ComObject WScript.Shell"]
        for it, t in zip(items, tmp):
            lines += ["$l = $s.CreateShortcut(%s)" % _ps_quote(t),
                      "$l.TargetPath = %s" % _ps_quote(it["target"]),
                      "$l.Arguments = %s" % _ps_quote(it["args"]),
                      "$l.WorkingDirectory = %s" % _ps_quote(it["workdir"]),
                      "$l.IconLocation = %s" % _ps_quote(it["icon"] + ",0"),
                      "$l.Save()"]
        enc = base64.b64encode("\n".join(lines).encode("utf-16-le")).decode("ascii")    # Greek names intact
        r = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-EncodedCommand", enc],
                           capture_output=True, text=True, timeout=60, creationflags=NO_WINDOW)
        if r.returncode != 0 or not all(os.path.exists(t) for t in tmp):
            raise RuntimeError("PowerShell: %s" % (r.stderr or r.stdout).strip()[-400:])
        for it, t in zip(items, tmp):
            os.replace(t, it["path"])
    else:
        for it in items:
            with open(it["path"], "w", encoding="utf-8") as f:
                f.write("[Desktop Entry]\nType=Application\nName=%s\nExec=%s %s\nPath=%s\nIcon=%s\nTerminal=false\n" % (
                    os.path.splitext(os.path.basename(it["path"]))[0], it["target"], it["args"], it["workdir"],
                    it["icon"]))
            os.chmod(it["path"], 0o755)
    return items


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    dest = argv[argv.index("--dest") + 1] if "--dest" in argv else desktop()
    items = make(dest, dry="--dry-run" in argv)
    for it in items:
        print("%s%s -> %s %s" % ("(dry run) " if "--dry-run" in argv else "", it["path"], it["target"], it["args"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
