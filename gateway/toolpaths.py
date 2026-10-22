"""Where the external programs are (MediaMTX, ffmpeg) - one place for the
gateway, the tests and the demo. Never a temp folder:

  MediaMTX   $MEDIAMTX_BIN, "mediamtx" on the PATH, <repo>/tools/bin/mediamtx/
             (python tools/get_mediamtx.py puts it there), /usr/local/bin/mediamtx
  ffmpeg     $FFMPEG_BIN, "ffmpeg" on the PATH, C:\\msys64\\ucrt64\\bin\\ffmpeg.exe
             (MSYS2: pacman -S mingw-w64-ucrt-x86_64-ffmpeg), /usr/bin/ffmpeg
Each returns the full path, or None when the program is missing.
"""
import os
import shutil

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
EXE = ".exe" if os.name == "nt" else ""


def _first(*cands):
    for c in cands:
        if c and os.path.isfile(c):
            return os.path.abspath(c)
    return None


def mediamtx():
    return _first(os.environ.get("MEDIAMTX_BIN"), shutil.which("mediamtx"),
                  os.path.join(REPO, "tools", "bin", "mediamtx", "mediamtx" + EXE), "/usr/local/bin/mediamtx")


def ffmpeg():
    return _first(os.environ.get("FFMPEG_BIN"), shutil.which("ffmpeg"), r"C:\msys64\ucrt64\bin\ffmpeg.exe",
                  "/usr/bin/ffmpeg")


if __name__ == "__main__":
    print("MediaMTX:", mediamtx() or "MISSING (python tools/get_mediamtx.py)")
    print("ffmpeg:  ", ffmpeg() or "MISSING (MSYS2: pacman -S mingw-w64-ucrt-x86_64-ffmpeg)")
