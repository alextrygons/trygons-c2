"""gateway/setup_all.sh (step 13j: the gateway PC one-command setup) - tested on ANY PC, nothing changes.

    python test_setup.py        (exit code 0 = all passed)

The script runs in its test mode (TRYGONS_SETUP_TEST=1): a fake root folder (etc/, var/, run/ ...), the
answers from a file instead of /dev/tty, a small fake repository, and EVERY system command stubbed:
commands that change the system are only printed ([TEST] ...), read-only queries (ip, systemctl,
journalctl, tailscale, timedatectl, df, id, python3 gateway.py --list-serial / --check-config) run the
stub scripts of this test. Only the JSON / password-hash helpers run for real (into the fake root).

  ST1  shellcheck (if installed: PATH or $SHELLCHECK) - else says so and does bash -n only; with
       TRYGONS_REQUIRE_SHELLCHECK=1 a missing shellcheck is a FAIL (not a silent skip); bash -n always
  ST2  refuses non-root, non-Ubuntu and Ubuntu 22.04 with a clear message (Greek default); nothing written
  ST3  first run: every question answered from the file (boat, users + password retries, Tailscale key,
       router / camera ports by the plug-in test, chart area, serial adapters) -> secrets.json (hashes that
       users.check_pw accepts), gateway.json (boat, adapters), and every step in order
  ST4  no secret anywhere: not in the log, the screen output, the prompts, any other file; the key /
       user temp files are gone; secrets.json mode 600 (POSIX) + umask / chmod in the script; secrets.json
       git-ignored and not tracked; the real files live in /etc/trygons (outside the repository)
  ST5  run again: Enter everywhere keeps gateway.json / secrets.json byte for byte, the ports, the
       Tailscale login; nothing changed = no service restart, no tailscale up
  ST6  --dry-run: every action printed as [DRY-RUN], no file written (no log, no secrets, no config)
  ST7  check list: all OK with good fakes; each failing part -> its own FAIL line (service, AURIX, storage,
       lid, battery 80 %, Tailscale, cameras reboot / no cable, time); --checks asks nothing; exit codes
  ST8  never touches the AURIX UART or the boat: no serial device opened / written, gateway.py only with
       --list-serial / --check-config, no ping / ssh / nc to the boat network, only /dev/tty + /dev/null
  ST9  adapters not plugged in: AURIX TBD with a clear note, AIS disabled, the service NOT started, the check
       list says TBD; a bad answer to "adapter number" (not a number) is ignored
  ST10 docs + gate: README section 0 with the raw URL of `git remote get-url origin` (branch main), the
       safety notes, MANUAL.txt + MANUAL_EL.txt, tools/check.sh runs this test
  ST11 review fixes (13j2): "ALL OK" only when no step failed (a failing stubbed step, every check line
       OK); check lines for MediaMTX, vision Python, vision model, apt, tailscale up, a failed chartprep;
       a stale "AURIX connected" journal line from before this run's restart is never OK; a crash-looping
       service (NRestarts grows) is not OK; an adapter flagged RS232 is refused by default (Enter), an
       unflagged one answered "not 3.3 V TTL" too; the users temp file is removed when the input closes
       during the questions; Tailscale hostname trygons-gw-<boat>; NEEDRESTART_MODE=l on apt
"""
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SCRIPT = os.path.join(HERE, "setup_all.sh")
sys.path.insert(0, HERE)
import users  # noqa: E402

results = []

PW_ALEX = "Secret-Alex-123"
PW_GUEST = "Guest-Pass-999"
PW_SHORT = "abc"
PW_WRONG = "Secret-Alex-124"
TSKEY = "tskey-auth-TESTKEY0123456789secret"
SECRETS_ALL = [PW_ALEX, PW_GUEST, PW_WRONG, TSKEY, "TESTKEY0123456789"]

LIST_SERIAL = """/dev/ttyUSB0   TTL232R-3V3
    use: /dev/serial/by-id/usb-FTDI_TTL232R-3V3_A50285BI-if00-port0
    use: usb:0403:6001:A50285BI
/dev/ttyACM0   USB AIS Receiver
    use: /dev/serial/by-id/usb-AIS_Receiver_4F1E-if00
    use: usb:16d0:0b03:4F1E
AURIX: ONLY a 3.3 V TTL USB-UART adapter (FTDI / CP2102 at 3.3 V). NEVER RS232: +-12 V destroys the AURIX pins.
"""


def report(name, ok, detail=""):
    results.append((name, ok))
    print("[%s] %s%s" % ("PASS" if ok else "FAIL", name, (" - " + detail) if detail else ""), flush=True)


def fwd(p):
    return p.replace("\\", "/")


def find_bash():
    """Git Bash / MSYS2 bash on Windows (never C:\\Windows\\System32\\bash.exe = WSL), else bash."""
    if os.name != "nt":
        return shutil.which("bash")
    cands = []
    git = shutil.which("git")
    if git:
        g = os.path.dirname(os.path.dirname(os.path.realpath(git)))
        cands += [os.path.join(g, "bin", "bash.exe"), os.path.join(g, "usr", "bin", "bash.exe")]
    cands += [r"C:\Program Files\Git\bin\bash.exe", r"C:\Program Files\Git\usr\bin\bash.exe", r"C:\msys64\usr\bin\bash.exe"]
    for c in cands:
        if os.path.exists(c) and "system32" not in c.lower():
            return c
    return None


BASH = find_bash()

STUBS = {
    "id": 'd=$(dirname "$0"); if [ "$1" = -u ]; then cat "$d/uid"; else exit 0; fi\n',
    "python3": 'd=$(dirname "$0"); case "$*" in *--list-serial*) cat "$d/list_serial.txt";; '
               '*--check-config*) exit "$(cat "$d/check_rc")";; *) exit 1;; esac\n',
    "systemctl": 'd=$(dirname "$0"); case "$1 $2" in "is-active trygons-gateway") cat "$d/svc_active";; '
                 '"is-enabled sleep.target") cat "$d/sleep_state";; '
                 '"show -p") case "$3" in NRestarts) n=$(cat "$d/nrestarts"); echo "$n"; '
                 '[ -f "$d/nrestarts_inc" ] && echo $((n + 1)) > "$d/nrestarts";; '
                 'InvocationID) cat "$d/invocation";; esac;; *) exit 1;; esac\n',
    # the whole boot's journal; with --since only the part written since the script started
    "journalctl": 'd=$(dirname "$0"); case "$*" in *--since*) cat "$d/journal_since.txt";; *) cat "$d/journal.txt";; esac\n',
    "python": 'exit "$(cat "$(dirname "$0")/vision_rc")"\n',     # the vision venv's python
    "nft": 'd=$(dirname "$0"); [ -f "$d/nft.txt" ] || exit 1; cat "$d/nft.txt"\n',   # nft list ruleset
    "tailscale": 'd=$(dirname "$0"); case "$1" in status) cat "$d/ts.json";; version) echo 1.80.0;; *) exit 1;; esac\n',
    "timedatectl": 'd=$(dirname "$0"); case "$*" in *NTPSynchronized*) cat "$d/ntp";; *Timezone*) cat "$d/tz";; esac\n',
    "df": 'printf "Avail\\n %sG\\n" "$(cat "$(dirname "$0")/free_gb")"\n',
    "tlp-stat": 'exit 1\n',
    # the Ethernet ports follow the plug-in test (the prompts file says how far it got)
    "ip": r'''d=$(dirname "$0")
[ "$1 $2" = "-br link" ] || exit 1
if [ -f "$d/ip_static" ]; then cat "$d/ip_static"; exit 0; fi
p="$TRYGONS_SETUP_ROOT/prompts.txt"
nin=$(grep -o -e "Enter when it is in" -e "Enter όταν είναι μέσα" "$p" 2>/dev/null | wc -l)
nout=$(grep -o -e "Enter when it is out" -e "Enter όταν είναι έξω" "$p" 2>/dev/null | wc -l)
a=NO-CARRIER; b=NO-CARRIER
{ [ "$nout" -eq 0 ] || [ "$nin" -ge 1 ]; } && a=LOWER_UP
{ [ "$nout" -eq 0 ] || [ "$nin" -ge 2 ]; } && b=LOWER_UP
echo "lo               UNKNOWN        00:00:00:00:00:00 <LOOPBACK,UP,LOWER_UP>"
echo "enp0s31f6        UP             8c:16:45:aa:bb:01 <BROADCAST,MULTICAST,UP,$a>"
echo "enx00e04c680002  UP             00:e0:4c:68:00:02 <BROADCAST,MULTICAST,UP,$b>"
echo "wlp4s0           UP             a0:b1:c2:d3:e4:f5 <BROADCAST,MULTICAST,UP,LOWER_UP>"
''',
}


class Env:
    def __init__(self, uid="0", os_id="ubuntu", os_ver="24.04"):
        self.base = tempfile.mkdtemp(prefix="trygons_setup_")
        self.root = os.path.join(self.base, "root")
        self.stub = os.path.join(self.base, "stub")
        self.repo = os.path.join(self.base, "repo")
        for d in (self.root, self.stub, os.path.join(self.root, "etc")):
            os.makedirs(d, exist_ok=True)
        with open(os.path.join(self.root, "etc", "os-release"), "w", newline="\n") as f:
            f.write('NAME="Ubuntu"\nID=%s\nVERSION_ID="%s"\n' % (os_id, os_ver))
        for name, body in STUBS.items():
            with open(os.path.join(self.stub, name), "w", encoding="utf-8", newline="\n") as f:
                f.write(body)
        self.put("uid", uid)
        self.put("list_serial.txt", LIST_SERIAL)
        self.put("check_rc", "0")
        self.put("svc_active", "active")
        self.put("sleep_state", "masked")
        self.put("journal.txt", "storage /var/lib/trygons/data ok\nAURIX connected (usb:0403:6001:A50285BI = "
                                "/dev/ttyUSB0, 921600 baud)\n")
        self.put("journal_since.txt", open(os.path.join(self.stub, "journal.txt"), encoding="utf-8").read())
        self.put("nrestarts", "0")
        self.put("invocation", "4f2a9c0d")
        self.put("vision_rc", "1")          # the vision venv is not there yet (first run)
        self.put("ts.json", '{"BackendState": "NeedsLogin"}')
        self.put("ntp", "yes")
        self.put("tz", "UTC")
        self.put("free_gb", "300")
        # a small fake repository: only what the script reads
        for rel in ("gateway/users.py", "gateway/secrets.example.json", "gateway/config.example.json",
                    "gateway/systemd/install_ubuntu.sh", "tools/get_vision_python.py",
                    "boats/rf49d.json", "boats/er69d.json"):
            dst = os.path.join(self.repo, rel)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            shutil.copy(os.path.join(REPO, rel), dst)
        os.makedirs(os.path.join(self.repo, ".git"))
        open(os.path.join(self.repo, "gateway", "gateway.py"), "w").close()
        # what get_mediamtx.py / get_vision_model.py would have downloaded (they are only printed in the test)
        for rel in ("tools/bin/mediamtx/mediamtx", "gateway/vision/models/ewasr_resnet18.onnx"):
            dst = os.path.join(self.repo, rel)
            os.makedirs(os.path.dirname(dst), exist_ok=True)
            with open(dst, "w") as f:
                f.write("fake")
        os.makedirs(os.path.join(self.repo, "charts"))
        with open(os.path.join(self.repo, "charts", "areas.json"), "w") as f:
            json.dump({"attica": {"lat": [37.3, 38.35], "lon": [22.95, 24.65], "stride": 1}}, f)
        # what install_ubuntu.sh would have made (it is only printed in the test)
        self.write_root("etc/systemd/logind.conf.d/trygons.conf", "[Login]\nHandleLidSwitch=ignore\n")
        self.write_root("sys/class/power_supply/BAT0/charge_control_end_threshold", "80\n")

    def put(self, name, text):
        with open(os.path.join(self.stub, name), "w", encoding="utf-8", newline="\n") as f:
            f.write(text)

    def write_root(self, rel, text):
        p = os.path.join(self.root, rel)
        os.makedirs(os.path.dirname(p), exist_ok=True)
        with open(p, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)

    def path(self, rel):
        return os.path.join(self.root, rel)

    def read(self, rel):
        p = self.path(rel)
        if not os.path.exists(p):
            return None
        with open(p, encoding="utf-8") as f:
            return f.read()

    def run(self, answers, args=("--lang", "en"), timeout=120):
        ans = os.path.join(self.base, "answers.txt")
        with open(ans, "w", encoding="utf-8", newline="\n") as f:
            f.write("".join(a + "\n" for a in answers))
        try:
            os.remove(self.path("prompts.txt"))
        except OSError:
            pass
        env = dict(os.environ, TRYGONS_SETUP_TEST="1", TRYGONS_SETUP_ROOT=fwd(self.root),
                   TRYGONS_SETUP_STUB=fwd(self.stub), TRYGONS_SETUP_PY=fwd(sys.executable),
                   TRYGONS_SETUP_TTY=fwd(ans), TRYGONS_SETUP_REPO=fwd(self.repo), PYTHONIOENCODING="utf-8",
                   LANG="C.UTF-8")
        p = subprocess.run([BASH, fwd(SCRIPT)] + list(args), env=env, capture_output=True, timeout=timeout)
        self.out = p.stdout.decode("utf-8", "replace") + p.stderr.decode("utf-8", "replace")
        self.prompts = self.read("prompts.txt") or ""
        self.rc = p.returncode
        return p.returncode

    def all_text(self):
        """Every file under the fake root (but secrets.json) + the screen output."""
        parts = [self.out]
        for dp, _dn, fn in os.walk(self.root):
            for n in fn:
                if os.path.join(dp, n) == self.path("etc/trygons/secrets.json"):
                    continue
                with open(os.path.join(dp, n), "rb") as f:
                    parts.append(f.read().decode("utf-8", "replace"))
        return "\n".join(parts)

    def close(self):
        shutil.rmtree(self.base, ignore_errors=True)


FIRST_ANSWERS = [
    "RF49D",                    # boat
    "alex", "", "y",            # user, role HELM (default), licence
    PW_SHORT, PW_SHORT,         # too short
    PW_ALEX, PW_WRONG,          # not the same
    PW_ALEX, PW_ALEX,           # ok
    "guest", "VIEWER",          # second user (no licence question for a VIEWER)
    PW_GUEST, PW_GUEST,
    "",                         # no more users
    TSKEY,                      # Tailscale key
    "", "", "y",                # router plug-in test: out, in, right?
    "", "", "y",                # cameras plug-in test
    "",                         # chart area (default)
    "1", "y", "2",              # AURIX = 1 (confirmed: a 3.3 V TTL adapter), AIS = 2
]


def actions(out, tag="[TEST]"):
    return [ln[len(tag) + 1:] for ln in out.splitlines() if ln.startswith(tag + " ")]


def st1():
    sc = os.environ.get("SHELLCHECK") or shutil.which("shellcheck")
    if sc:
        p = subprocess.run([sc, "-s", "bash", "-f", "gcc", SCRIPT], capture_output=True, text=True,
                           encoding="utf-8", errors="replace")
        report("ST1 shellcheck clean", p.returncode == 0, (p.stdout + p.stderr).strip()[:600])
    elif os.environ.get("TRYGONS_REQUIRE_SHELLCHECK") == "1":
        report("ST1 shellcheck installed (TRYGONS_REQUIRE_SHELLCHECK=1)", False,
               "not found: apt install shellcheck / pip install shellcheck-py, or set SHELLCHECK")
    else:
        print("ST1: shellcheck not available (install: apt install shellcheck / pip install shellcheck-py, or set "
              "SHELLCHECK; TRYGONS_REQUIRE_SHELLCHECK=1 makes this a FAIL) - bash -n syntax check only")
    p = subprocess.run([BASH, "-n", fwd(SCRIPT)], capture_output=True, text=True)
    report("ST1 bash -n syntax", p.returncode == 0, p.stderr.strip())
    with open(SCRIPT, "rb") as f:
        raw = f.read()
    report("ST1 LF line endings + runs as one function (curl | bash safe)", b"\r\n" not in raw
           and raw.rstrip().endswith(b'main "$@"\nexit $?') and raw.startswith(b"#!/usr/bin/env bash"))


def st2():
    for uid, osid, ver, rc, text, label in (
            ("1000", "ubuntu", "24.04", 3, "Χρειάζεται root", "non-root"),
            ("0", "debian", "12", 2, "Μόνο για Ubuntu 24.04", "Debian"),
            ("0", "ubuntu", "22.04", 2, "Μόνο για Ubuntu 24.04", "Ubuntu 22.04")):
        e = Env(uid=uid, os_id=osid, os_ver=ver)
        try:
            got = e.run([], args=())
            report("ST2 refuses %s (exit %d, clear message, Greek default)" % (label, rc),
                   got == rc and text in e.out and e.read("var/log/trygons-setup.log") is None
                   and not os.path.exists(e.path("etc/trygons")), "rc %s: %s" % (got, e.out.strip()[:200]))
        finally:
            e.close()
    e = Env(uid="1000")
    try:
        e.run([], args=("--lang", "en"))
        report("ST2 English message with --lang en", "Needs root" in e.out, e.out.strip()[:200])
    finally:
        e.close()


def st3_st4_st5():
    e = Env()
    try:
        rc = e.run(FIRST_ANSWERS)
        sec = e.read("etc/trygons/secrets.json")
        cfg = e.read("etc/trygons/gateway.json")
        ok = sec is not None and cfg is not None
        report("ST3 first run finished, both files written", ok, "rc %s\n%s" % (rc, e.out[-1500:]))
        if not ok:
            return
        s = json.loads(sec)
        u = s.get("users", {})
        report("ST3 secrets.json: the 2 users, roles, licence, hashes users.check_pw accepts",
               set(u) == {"alex", "guest"} and u["alex"]["role"] == "HELM" and u["alex"].get("licence") is True
               and u["guest"]["role"] == "VIEWER" and "licence" not in u["guest"]
               and users.check_pw(PW_ALEX, u["alex"]["pw"]) and users.check_pw(PW_GUEST, u["guest"]["pw"])
               and not users.check_pw(PW_WRONG, u["alex"]["pw"]) and "academy_key" in s, json.dumps(u)[:300])
        c = json.loads(cfg)
        report("ST3 gateway.json: boat + the stable adapter names (usb:VID:PID:SERIAL) from --list-serial",
               c["boat"] == "RF49D" and c["aurix"] == "usb:0403:6001:A50285BI"
               and c["ais"]["source"] == "usb:16d0:0b03:4F1E" and c["ais"]["enabled"] is True
               and c["baud"] == 921600 and "_aurix_tbd" not in c, json.dumps(c)[:300])
        report("ST3 the adapters listed with their usb: names, no RS232 caution for the 3.3 V FTDI",
               "1) /dev/ttyUSB0  TTL232R-3V3  -> usb:0403:6001:A50285BI" in e.prompts and "CAUTION" not in e.prompts
               and "listed only - never opened" in e.prompts, e.prompts[-600:])
        report("ST3 password retries: too short and not the same are refused",
               "Too short" in e.prompts and "Not the same" in e.prompts, e.prompts[-400:])
        acts = actions(e.out)
        want = ["apt-get update", "apt-get install -y git python3-serial python3-venv",
                "git -C %s pull --ff-only origin main" % fwd(e.repo),
                "NEEDRESTART_MODE=l apt-get -y -o Dpkg::Options::=--force-confold full-upgrade",
                "timedatectl set-timezone UTC",
                "LAN_MAC=8c:16:45:aa:bb:01 CAM_MAC=00:e0:4c:68:00:02 GWU=trygons sh %s/gateway/systemd/install_ubuntu.sh"
                % fwd(e.repo),
                "tailscale up --auth-key=file:", "get_mediamtx.py", "-m venv", "pip install --no-warn-script-location "
                "onnxruntime==1.20.1 numpy==2.1.3", "get_vision_model.py", "chartprep.py --area attica",
                "systemctl enable trygons-gateway", "systemctl restart trygons-gateway"]
        pos, missing = -1, []
        for w in want:
            idx = next((i for i, a in enumerate(acts) if w in a and i > pos), None)
            if idx is None:
                missing.append(w)
            else:
                pos = idx
        report("ST3 every step, in order (upgrade, install_ubuntu.sh with the MACs, Tailscale, MediaMTX, vision, "
               "charts, start)", not missing, "missing / out of order: %s\n%s" % (missing, "\n".join(acts)))
        report("ST3 Tailscale machine name per boat: --ssh --hostname=trygons-gw-rf49d (boat RF49D)",
               any("tailscale up" in a and a.endswith("--ssh --hostname=trygons-gw-rf49d") for a in acts)
               and "Machines -> trygons-gw-rf49d -> Disable key expiry" in e.out, "\n".join(acts))
        report("ST3 the plug-in test found router = enp0s31f6, cameras = enx00e04c680002 (Wi-Fi / lo never listed)",
               "Found port enp0s31f6" in e.prompts and "Found port enx00e04c680002" in e.prompts
               and "wlp4s0" not in e.prompts and not re.search(r"^  lo ", e.prompts, re.M), e.prompts[-600:])
        report("ST3 the check list ran (OK / FAIL lines)", "Check list" in e.out and "  OK    service" in e.out
               and "FAIL  camera network" in e.out, e.out[-900:])
        log = e.read("var/log/trygons-setup.log") or ""
        report("ST3 the log has the answers (without secrets) and the actions",
               "answers: boat=RF49D" in log and "tailscale_key=given" in log and "+ apt-get update" in log, log[-400:])

        # ST4 secrets
        text = e.all_text() + e.prompts
        leaks = [x for x in SECRETS_ALL if x in text]
        report("ST4 no password / key in the log, output, prompts or any file", not leaks, "LEAK: %s" % leaks)
        report("ST4 no plain password in secrets.json either (hashes only)",
               not any(x in sec for x in SECRETS_ALL), "")
        leftovers = [n for n in os.listdir(e.path("run")) if n.startswith("trygons-")] if os.path.isdir(e.path("run")) else []
        report("ST4 the Tailscale key file and the user temp file are removed", not leftovers, str(leftovers))
        if os.name == "posix":
            mode = stat.S_IMODE(os.stat(e.path("etc/trygons/secrets.json")).st_mode)
            report("ST4 secrets.json mode 600", mode == 0o600, oct(mode))
        else:
            print("ST4 secrets.json mode 600: N/A on Windows (NTFS) - checked in the script text below")
        with open(SCRIPT, encoding="utf-8") as f:
            src = f.read()
        report("ST4 the script writes secrets.json with umask 077 + chmod 600, never with set -x",
               "os.umask(0o077)" in src and 'chmod 600 "$SECRETS"' in src and "os.chmod(tmp, 0o600)" in src
               and not re.search(r"^\s*set -[a-z]*x", src, re.M) and "read -r -s -u 3" in src)
        ign = subprocess.run(["git", "-C", REPO, "check-ignore", "-q", "gateway/secrets.json"]).returncode == 0
        tracked = subprocess.run(["git", "-C", REPO, "ls-files"], capture_output=True, text=True).stdout.split()
        report("ST4 secrets.json git-ignored, no secrets.json tracked, real file outside the repo (/etc/trygons)",
               ign and not [t for t in tracked if t.endswith("secrets.json")] and 'ETC="$R/etc/trygons"' in src,
               "ignored %s" % ign)

        # ST5 run again: Enter everywhere = keep
        e.write_root("etc/systemd/network/10-trygons-boatlan.link", "[Match]\nMACAddress=8c:16:45:aa:bb:01\n")
        e.write_root("etc/systemd/network/11-trygons-cam.link", "[Match]\nMACAddress=00:e0:4c:68:00:02\n")
        os.makedirs(os.path.join(e.repo, "charts", "attica"))
        open(os.path.join(e.repo, "charts", "attica", "meta.json"), "w").close()
        e.put("ts.json", '{"BackendState": "Running"}')
        before = (sec, cfg)
        rc2 = e.run(["", "", "", "", "", "", "", ""])
        after = (e.read("etc/trygons/secrets.json"), e.read("etc/trygons/gateway.json"))
        report("ST5 run again with Enter: secrets.json + gateway.json byte for byte the same",
               before == after, "rc %s\n%s" % (rc2, e.out[-800:]))
        acts = actions(e.out)
        report("ST5 nothing changed = no restart, no tailscale up, no chartprep, no install_ubuntu MAC change",
               not any("restart" in a or "tailscale up" in a or "chartprep" in a for a in acts)
               and any("LAN_MAC=8c:16:45:aa:bb:01" in a for a in acts) and "already runs" in e.out,
               "\n".join(acts))
        report("ST5 it asked to keep (config, users, ports, charts) and saw the Tailscale login",
               "keep it as it is?" in e.prompts and "K = keep them" in e.prompts and "keep them?" in e.prompts
               and "make them again?" in e.prompts and "already connected" in e.prompts
               and "Tailscale auth key" not in e.prompts, e.prompts[-800:])
        e.put("ip_static", "lo UNKNOWN 00:00:00:00:00:00 <LOOPBACK,UP,LOWER_UP>\n"
                           "boatlan0 UP 8c:16:45:aa:bb:01 <BROADCAST,MULTICAST,UP,LOWER_UP>\n"
                           "cam0 UP 00:e0:4c:68:00:02 <BROADCAST,MULTICAST,UP,LOWER_UP>\n")
        e.put("vision_rc", "0")
        rc3 = e.run([], args=("--lang", "en", "--checks"))
        report("ST7 --checks: no question, every line OK, exit 0", rc3 == 0 and "ALL OK" in e.out
               and "FAIL" not in e.out and not e.prompts, "rc %s\n%s" % (rc3, e.out[-900:]))
        # each failing part -> its own FAIL line
        fails = [("svc_active", "inactive", "FAIL  service trygons-gateway"),
                 ("journal.txt", "AURIX connected (x)\nAURIX link: no such device\n", "FAIL  AURIX link"),
                 ("free_gb", "5", "FAIL  storage"),
                 ("sleep_state", "enabled", "FAIL  lid closed"),
                 ("ts.json", '{"BackendState": "Stopped"}', "FAIL  Tailscale"),
                 ("ntp", "no", "FAIL  time"),
                 ("check_rc", "1", "FAIL  gateway.json")]
        bad = []
        for name, val, line in fails:
            old = open(os.path.join(e.stub, name), encoding="utf-8").read()
            e.put(name, val)
            r = e.run([], args=("--lang", "en", "--checks"))
            if r != 1 or line not in e.out or e.out.count("FAIL  ") != 1:
                bad.append("%s: rc %s %s" % (name, r, e.out[-300:]))
            e.put(name, old)
        thr = e.path("sys/class/power_supply/BAT0/charge_control_end_threshold")
        e.write_root("sys/class/power_supply/BAT0/charge_control_end_threshold", "100\n")
        r = e.run([], args=("--lang", "en", "--checks"))
        if r != 1 or "FAIL  battery: charge limit 80 %" not in e.out:
            bad.append("battery: %s" % e.out[-300:])
        with open(thr, "w") as f:
            f.write("80\n")
        e.put("ip_static", "boatlan0 UP 8c:16:45:aa:bb:01 <BROADCAST,MULTICAST,UP,LOWER_UP>\n"
                           "cam0 UP 00:e0:4c:68:00:02 <NO-CARRIER,BROADCAST,MULTICAST,UP>\n")
        r = e.run([], args=("--lang", "en", "--checks"))
        if "FAIL  camera network (cam0) - no cable" not in e.out:
            bad.append("cam no cable: %s" % e.out[-300:])
        e.put("ip_static", "enp0s31f6 UP 8c:16:45:aa:bb:01 <BROADCAST,MULTICAST,UP,LOWER_UP>\n"
                           "enx00e04c680002 UP 00:e0:4c:68:00:02 <BROADCAST,MULTICAST,UP,LOWER_UP>\n")
        r = e.run([], args=("--lang", "en", "--checks"))
        if "FAIL  camera network (cam0) - reboot needed" not in e.out:
            bad.append("cam reboot: %s" % e.out[-300:])
        report("ST7 each failing part -> its own FAIL line + exit 1 (service, AURIX lost after connect, storage < 20 GB, "
               "lid, Tailscale, time, config, battery 100 %, cameras: no cable / reboot needed)", not bad,
               "\n".join(bad))
        r = e.run([], args=("--checks",))
        report("ST7 Greek default check list", "Λίστα ελέγχου" in e.out and "υπηρεσία trygons-gateway" in e.out,
               e.out[-300:])
    finally:
        e.close()


def st6():
    e = Env()
    try:
        rc = e.run(FIRST_ANSWERS, args=("--lang", "en", "--dry-run"))
        written = [os.path.relpath(os.path.join(dp, n), e.root) for dp, _d, fn in os.walk(e.root) for n in fn]
        expected = {os.path.normpath(p) for p in ("etc/os-release", "etc/systemd/logind.conf.d/trygons.conf",
                                                   "sys/class/power_supply/BAT0/charge_control_end_threshold",
                                                   "prompts.txt")}
        extra = [w for w in written if os.path.normpath(w) not in expected]
        acts = actions(e.out, "[DRY-RUN]")
        report("ST6 --dry-run: no file written (no log, secrets.json, gateway.json, key file)", not extra,
               "rc %s written %s" % (rc, extra))
        report("ST6 --dry-run: every action printed as [DRY-RUN], none executed / [TEST]",
               len(acts) >= 12 and "[TEST]" not in e.out and any("full-upgrade" in a for a in acts)
               and "would write" in e.out and "DRY RUN: nothing is changed" in e.out, "\n".join(acts))
        report("ST6 --dry-run: no secret printed either", not [x for x in SECRETS_ALL if x in e.out + e.prompts])
    finally:
        e.close()


def st8():
    with open(SCRIPT, encoding="utf-8") as f:
        src = f.read()
    code = "\n".join(ln for ln in src.splitlines() if not ln.lstrip().startswith("#"))
    devs = set(re.findall(r"/dev/[A-Za-z0-9_/.-]*", code))
    report("ST8 the only devices are /dev/tty (questions) and /dev/null", devs <= {"/dev/tty", "/dev/null"},
           str(devs))
    bad = re.findall(r"(?:^|[;&|(]|\b(?:run|q|as_user)\s)\s*(ssh|ping|nc|socat|stty|minicom|picocom|screen|cu)\s",
                     code, re.M)
    bad += [w for w in ("serial.Serial", "192.168.", "10.0.") if w in code]
    report("ST8 no serial terminal / port open / ping / ssh / boat LAN address in the script", not bad, str(bad))
    calls = re.findall(r'\b(?:q|run|as_user)\s+python3\s+"\$REPO/gateway/gateway\.py"([^\n|;)]*)', code)
    report("ST8 gateway.py is only called (q = read only) with --list-serial or --check-config",
           len(calls) >= 3 and all("--list-serial" in c or "--check-config" in c for c in calls)
           and not re.search(r'(run|as_user)\s+python3\s+"\$REPO/gateway/gateway\.py"', code), str(calls))
    e = Env()
    try:
        e.run(FIRST_ANSWERS)
        acts = actions(e.out)
        touch = [a for a in acts if re.search(r"/dev/tty(USB|ACM|S)|/dev/serial|stty|gateway\.py", a)]
        report("ST8 no executed action touches a serial device or runs gateway.py", not touch, str(touch))
    finally:
        e.close()


def st9():
    e = Env()
    try:
        e.put("list_serial.txt", "no serial ports found (plug the USB adapter in)\nAURIX: ONLY a 3.3 V TTL ...\n")
        e.put("check_rc", "1")
        ans = [a for a in FIRST_ANSWERS[:-3]]
        e.run(ans)
        c = json.loads(e.read("etc/trygons/gateway.json") or "{}")
        report("ST9 adapters not plugged in: AURIX stays TBD with a note, AIS disabled",
               "VID:PID" in c.get("aurix", "") and "TBD" in c.get("_aurix_tbd", "") and "NEVER RS232" in
               c.get("_aurix_tbd", "") and c.get("ais", {}).get("enabled") is False, json.dumps(c)[:400])
        acts = actions(e.out)
        report("ST9 config errors -> the service is NOT started, the check list says TBD",
               not any("restart" in a for a in acts) and "the service was NOT started" in e.out
               and "adapter not set (TBD)" in e.out, e.out[-700:])
    finally:
        e.close()
    e = Env()
    try:
        ans = FIRST_ANSWERS[:-3] + ["1;w /tmp/x", "abc"]
        e.run(ans)
        c = json.loads(e.read("etc/trygons/gateway.json") or "{}")
        report("ST9 a non-number answer to 'adapter number' is ignored (TBD)",
               "VID:PID" in c.get("aurix", "") and c.get("ais", {}).get("enabled") is False, json.dumps(c)[:300])
    finally:
        e.close()


def all_ok_env():
    """An installed gateway where every check line is OK (first run + the state after the reboot)."""
    e = Env()
    e.run(FIRST_ANSWERS)
    e.write_root("etc/systemd/network/10-trygons-boatlan.link", "[Match]\nMACAddress=8c:16:45:aa:bb:01\n")
    e.write_root("etc/systemd/network/11-trygons-cam.link", "[Match]\nMACAddress=00:e0:4c:68:00:02\n")
    os.makedirs(os.path.join(e.repo, "charts", "attica"), exist_ok=True)
    open(os.path.join(e.repo, "charts", "attica", "meta.json"), "w").close()
    e.put("ts.json", '{"BackendState": "Running"}')
    e.put("vision_rc", "0")
    e.put("ip_static", "boatlan0 UP 8c:16:45:aa:bb:01 <BROADCAST,MULTICAST,UP,LOWER_UP>\n"
                       "cam0 UP 00:e0:4c:68:00:02 <BROADCAST,MULTICAST,UP,LOWER_UP>\n")
    return e


KEEP_ALL = ["", "", "", "", "", "", "", ""]      # run again: keep config, users, ports, area, charts


def st11():
    # (1) ALL OK only when FAILED_STEPS is empty
    e = all_ok_env()
    try:
        rc = e.run(KEEP_ALL)
        report("ST11 control: a clean re-run on an installed gateway says ALL OK, exit 0",
               rc == 0 and "ALL OK" in e.out and "FAIL" not in e.out, "rc %s\n%s" % (rc, e.out[-900:]))
        bad = []
        for pat, line in (("get_vision_model.py", "tools/get_vision_model.py"),   # the model file is still there
                          ("get_mediamtx.py", "tools/get_mediamtx.py"),
                          ("full-upgrade", "FAIL  system packages (apt, this run)"),
                          ("apt-get update", "FAIL  system packages (apt, this run)")):
            e.put("fail_run", pat + "\n")
            r = e.run(KEEP_ALL)
            if r == 0 or "ALL OK" in e.out or "Steps that failed" not in e.out or line not in e.out:
                bad.append("%s: rc %s\n%s" % (pat, r, e.out[-700:]))
        e.put("fail_run", "chartprep.py\n")
        r = e.run(["", "", "", "", "y"])              # make the charts again -> chartprep fails (files still there)
        if r == 0 or "ALL OK" in e.out or "FAIL  depth charts (attica) - missing or partial" not in e.out:
            bad.append("chartprep: rc %s\n%s" % (r, e.out[-700:]))
        os.remove(os.path.join(e.stub, "fail_run"))
        report("ST11 a failing step (model, MediaMTX, apt, chartprep) -> no ALL OK, exit 1, its FAIL line + "
               "'Steps that failed'", not bad, "\n".join(bad))
        bad = []
        for what, act in (("vision Python", lambda: e.put("vision_rc", "1")),
                          ("vision model", lambda: os.remove(os.path.join(e.repo, "gateway", "vision", "models",
                                                                          "ewasr_resnet18.onnx"))),
                          ("MediaMTX", lambda: os.remove(os.path.join(e.repo, "tools", "bin", "mediamtx", "mediamtx")))):
            act()
            r = e.run([], args=("--lang", "en", "--checks"))
            if r != 1 or ("FAIL  " + what) not in e.out or "ALL OK" in e.out:
                bad.append("%s: rc %s\n%s" % (what, r, e.out[-500:]))
        report("ST11 check lines: vision Python, vision model, MediaMTX each FAIL when missing", not bad, "\n".join(bad))
        # the firewall: never applied by the script = a WARN line (not counted), OK once the ruleset is loaded
        e.put("vision_rc", "0")
        for rel in ("tools/bin/mediamtx/mediamtx", "gateway/vision/models/ewasr_resnet18.onnx"):
            with open(os.path.join(e.repo, rel), "w") as f:
                f.write("fake")
        r1 = e.run([], args=("--lang", "en", "--checks"))
        warned = r1 == 0 and "WARN  firewall (nftables) - NOT applied" in e.out and "ALL OK" in e.out
        e.put("nft.txt", 'table inet filter {\n chain input {\n  iifname "tailscale0" tcp dport { 22, 5761 } accept\n')
        r2 = e.run([], args=("--lang", "en", "--checks"))
        applied = r2 == 0 and "OK    firewall (nftables)" in e.out and "WARN" not in e.out
        with open(SCRIPT, encoding="utf-8") as f:
            no_apply = not re.search(r"^\s*(run|q)\s+nft\s+-f", f.read(), re.M)
        report("ST11 firewall: not applied = a WARN line (not counted), applied = OK; the script never runs nft -f",
               warned and applied and no_apply, "rc %s/%s\n%s" % (r1, r2, e.out[-600:]))
    finally:
        e.close()
    # (2) a stale journal line and a crash loop
    e = Env()
    try:
        e.put("journal_since.txt", "storage ok\n")   # the restarted service has not connected (yet)
        e.run(FIRST_ANSWERS)
        stale = "FAIL  AURIX link" in e.out and "OK    AURIX link" not in e.out
    finally:
        e.close()
    e = Env()
    try:
        e.run(FIRST_ANSWERS)
        fresh = "OK    AURIX link" in e.out
        report("ST11 after this run's restart an old 'AURIX connected' line is NOT OK (journal --since start)",
               stale and fresh, e.out[-600:])
        e.put("nrestarts_inc", "1")
        r = e.run([], args=("--lang", "en", "--checks"))
        loop = r == 1 and "FAIL  service trygons-gateway" in e.out and "restarting" in e.out
        os.remove(os.path.join(e.stub, "nrestarts_inc"))
        r = e.run([], args=("--lang", "en", "--checks"))
        report("ST11 a crash-looping service (active, NRestarts grows) is FAIL; a steady one OK",
               loop and "OK    service trygons-gateway" in e.out, e.out[-500:])
    finally:
        e.close()
    # (3) RS232 adapter refused by default, unflagged confirmed
    rs = ("/dev/ttyUSB0   USB-Serial Controller\n    use: usb:067b:2303:XYZ\n"
          "    CAUTION: Prolific PL2303 chips are mostly in RS232 (+-12 V) cables - the AURIX needs a 3.3 V TTL adapter\n"
          + LIST_SERIAL.split("\n", 3)[3])
    bad = []
    for listing, ans, want_aurix, label in (
            (rs, ["1", "", ""], None, "flagged + Enter"),
            (rs, ["1", "n", ""], None, "flagged + no"),
            (rs, ["1", "y", ""], "usb:067b:2303:XYZ", "flagged + an explicit yes"),
            (LIST_SERIAL, ["1", "n", ""], None, "unflagged + 'not 3.3 V TTL'")):
        e = Env()
        try:
            e.put("list_serial.txt", listing)
            e.run(FIRST_ANSWERS[:-3] + ans)
            c = json.loads(e.read("etc/trygons/gateway.json") or "{}")
            got = c.get("aurix", "")
            ok = (got == want_aurix) if want_aurix else ("VID:PID" in got and "TBD" in c.get("_aurix_tbd", "")
                                                         and "Refused" in e.prompts)
            if listing is rs and "probably RS232" not in e.prompts:
                ok = False
            if not ok:
                bad.append("%s: aurix %r\n%s" % (label, got, e.prompts[-500:]))
        finally:
            e.close()
    report("ST11 an adapter flagged RS232 is refused unless an explicit yes; 'not 3.3 V TTL' refused too",
           not bad, "\n".join(bad))
    # (4) the users temp file is removed when the input closes during the questions
    e = Env()
    try:
        rc = e.run(["RF49D", "alex", "", "y"])        # the password question gets EOF -> stop
        left = [n for n in os.listdir(e.path("run")) if n.startswith("trygons-")] if os.path.isdir(e.path("run")) else []
        report("ST11 input closed during the user questions: stopped (exit 4), no users temp file left",
               rc == 4 and not left, "rc %s left %s" % (rc, left))
    finally:
        e.close()
    with open(SCRIPT, encoding="utf-8") as f:
        src = f.read()
    report("ST11 the EXIT trap is set before any temp file is made",
           0 < src.index("trap cleanup EXIT") < src.index('mktemp "$RUNDIR/trygons-users') < src.index(
               'mktemp "$RUNDIR/trygons-ts'))


def st10():
    url = subprocess.run(["git", "-C", REPO, "remote", "get-url", "origin"], capture_output=True,
                         text=True).stdout.strip()
    m = re.match(r"(?:https://github\.com/|git@github\.com:)([^/]+)/(.+?)(?:\.git)?$", url)
    raw = "https://raw.githubusercontent.com/%s/%s/main/gateway/setup_all.sh" % (m.group(1), m.group(2)) if m else None
    with open(os.path.join(HERE, "README.md"), encoding="utf-8") as f:
        rd = f.read()
    sec0 = rd.split("## 0. Quick setup", 1)[1].split("\n## ", 1)[0] if "## 0. Quick setup" in rd else ""
    report("ST10 README section 0 'Quick setup' with the raw URL from git remote (main)",
           bool(raw) and raw in sec0 and "| sudo bash" in sec0, "raw %s" % raw)
    with open(SCRIPT, encoding="utf-8") as f:
        src = f.read()
    report("ST10 the script names the same URLs (raw + clone)", bool(raw) and raw in src and url in src, url)
    report("ST10 README safety notes: 3.3 V adapter NEVER RS232, AURIX off the network, lid + 80 %",
           "NEVER RS232" in sec0 and "never on the network" in sec0 and "lid" in sec0 and "80 %" in sec0)
    man0 = open(os.path.join(REPO, "MANUAL.txt"), encoding="utf-8").read()
    man0_el = open(os.path.join(REPO, "MANUAL_EL.txt"), encoding="utf-8").read()
    report("ST10 curl -fsSL everywhere (a 404 page is never piped to bash), no curl -sL left",
           "curl -fsSL" in sec0 and "curl -fsSL" in src and not any("curl -sL" in x for x in (rd, src, man0, man0_el)))
    report("ST10 README: NEEDRESTART_MODE=l + the final reboot, Tailscale trygons-gw-<boat> + --ssh, "
           "TRYGONS_REQUIRE_SHELLCHECK", "NEEDRESTART_MODE=l" in sec0 and "trygons-gw-<boat>" in sec0
           and "--ssh" in sec0 and "TRYGONS_REQUIRE_SHELLCHECK" in rd)
    man = open(os.path.join(REPO, "MANUAL.txt"), encoding="utf-8").read()
    man_el = open(os.path.join(REPO, "MANUAL_EL.txt"), encoding="utf-8").read()
    report("ST10 MANUAL.txt + MANUAL_EL.txt describe setup_all.sh", "setup_all.sh" in man and "setup_all.sh" in man_el
           and raw in man and raw in man_el)
    chk = open(os.path.join(REPO, "tools", "check.sh"), encoding="utf-8").read()
    report("ST10 tools/check.sh runs test_setup.py", "test_setup.py" in chk)


def main():
    if not BASH:
        report("ST0 bash found (Git Bash / MSYS2 on Windows)", False, "no bash - install Git for Windows")
        return 1
    st1()
    st2()
    st3_st4_st5()
    st6()
    st8()
    st9()
    st10()
    st11()
    n_fail = sum(1 for _n, ok in results if not ok)
    print("SETUP TESTS: %d/%d passed" % (len(results) - n_fail, len(results)))
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main())
