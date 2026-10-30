#!/usr/bin/env bash
# Trygons C2 - gateway PC one-command setup (step 13j). Ubuntu 24.04 LTS on the ThinkPad L390 Yoga.
#
#   curl -sL https://raw.githubusercontent.com/alextrygons/trygons-c2/main/gateway/setup_all.sh | sudo bash
#   curl -sL ...setup_all.sh | sudo bash -s -- --dry-run          (shows every action, changes nothing)
#   curl -sL ...setup_all.sh | sudo bash -s -- --lang en          (English; Greek is the default)
#   sudo bash ~trygons/trygons-c2/gateway/setup_all.sh --checks   (only the final check list)
#
# It asks ONLY: the boat name, the gateway users + passwords (-> /etc/trygons/secrets.json, hashed,
# mode 600, never in git, never shown, never in the log), the Tailscale auth key (never shown / logged),
# which Ethernet port goes to the router and which to the cameras (a plug-in test), the chart area and
# which USB adapter is the AURIX / the AIS receiver. Then everything else by itself, safe to run again
# (an existing secrets.json / gateway.json / port choice is kept unless you say otherwise).
# Log: /var/log/trygons-setup.log (no secrets in it).
#
# SAFETY: it never opens the AURIX UART or anything on the boat (it only LISTS the USB serial adapters
# and checks gateway.json); the AURIX stays off the network; the AURIX adapter is a 3.3 V TTL USB-UART
# adapter - NEVER RS232 (+-12 V destroys the AURIX pins). Run it in harbour, the boat in STANDBY:
# restarting the gateway service shows GATEWAY LOST for a few seconds on the boat (an alarm only).
#
# Tests (no Ubuntu needed): python gateway/test_setup.py  (TRYGONS_SETUP_TEST=1: a fake root folder,
# answers from a file instead of /dev/tty, every system command stubbed - nothing on the PC changes).

# shellcheck disable=SC2319  # chk "$(test; echo $?)": the status of the test IS the value
main() {
set -u
set -o pipefail
export PYTHONDONTWRITEBYTECODE=1   # root never leaves __pycache__ files in the gateway user's repository

REPO_URL_DEFAULT="https://github.com/alextrygons/trygons-c2.git"
BRANCH=main
GWU=trygons

MODE=real
LANG_SEL=el
CHECKS_ONLY=0
for arg in "$@"; do
  case "$arg" in
    --dry-run) MODE=dry ;;
    --lang=en|--en) LANG_SEL=en ;;
    --lang=el|--el) LANG_SEL=el ;;
    --lang) ;;
    en) LANG_SEL=en ;;
    --checks) CHECKS_ONLY=1 ;;
    -h|--help) sed -n '2,24p' "${BASH_SOURCE[0]:-/dev/null}" 2>/dev/null || echo "see gateway/README.md section 0"; return 0 ;;
    *) echo "unknown option: $arg (--dry-run, --lang en, --checks)"; return 64 ;;
  esac
done

# --- test mode: a fake root, answers from a file, every command stubbed -----------------------------
R=""
STUB=""
PY=python3
TTY_IN=/dev/tty
TTY_OUT=/dev/tty
WAIT_S=10
if [ "${TRYGONS_SETUP_TEST:-0}" = 1 ]; then
  MODE_REQ=$MODE
  MODE="test"
  [ "$MODE_REQ" = dry ] && MODE=testdry
  R=${TRYGONS_SETUP_ROOT:?TRYGONS_SETUP_ROOT}
  STUB=${TRYGONS_SETUP_STUB:?TRYGONS_SETUP_STUB}
  PY=${TRYGONS_SETUP_PY:-python3}
  TTY_IN=${TRYGONS_SETUP_TTY:?TRYGONS_SETUP_TTY}
  TTY_OUT="$R/prompts.txt"
  WAIT_S=0
fi

ETC="$R/etc/trygons"
SECRETS="$ETC/secrets.json"
CONFIG="$ETC/gateway.json"
DATA="$R/var/lib/trygons/data"
NETD="$R/etc/systemd/network"
LINK_LAN="$NETD/10-trygons-boatlan.link"
LINK_CAM="$NETD/11-trygons-cam.link"
HOME_GW="$R/home/$GWU"
REPO="$HOME_GW/trygons-c2"
[ -n "${TRYGONS_SETUP_REPO:-}" ] && [ "$MODE" != real ] && REPO=$TRYGONS_SETUP_REPO
REPO_URL=${TRYGONS_REPO_URL:-$REPO_URL_DEFAULT}
LOG="$R/var/log/trygons-setup.log"
RUNDIR="$R/run"
T0=$(date +%s)
FAILED_STEPS=""
CHANGED=0
REBOOT_NOTE=0

# --- language: Greek default, English with --lang en ------------------------------------------------
t() { if [ "$LANG_SEL" = en ]; then printf '%s' "$2"; else printf '%s' "$1"; fi; }

# --- output + log (the log never gets a secret: secrets are never passed to say / run / q) -----------
logline() { [ "$MODE" = dry ] || [ "$MODE" = testdry ] || printf '%s %s\n' "$(date '+%F %T')" "$*" >> "$LOG"; }
say() { printf '%s\n' "$*"; logline "$*"; }
head1() { say ""; say "== $*"; }
die() { say "$(t "ΣΤΑΜΑΤΗΣΕ" "STOPPED"): $*"; exit "${2:-1}"; }
fail_step() { FAILED_STEPS="$FAILED_STEPS
  - $*"; say "  FAIL: $*"; }

# run = a command that CHANGES the system: printed + logged, executed only for real
run() {
  logline "+ $*"
  case "$MODE" in
    dry|testdry) printf '[DRY-RUN] %s\n' "$*"; return 0 ;;
    test) printf '[TEST] %s\n' "$*"; return 0 ;;
  esac
  "$@" </dev/null 2>&1 | tee -a "$LOG"
  return "${PIPESTATUS[0]}"
}
# q = a READ-ONLY query (listing, status); in the tests it runs the stub of the same name
q() {
  if [ "$MODE" = test ] || [ "$MODE" = testdry ]; then
    local s; s="$STUB/$(basename "$1")"; shift
    [ -f "$s" ] || return 127
    bash "$s" "$@" </dev/null
    return
  fi
  "$@" </dev/null 2>/dev/null
}
pyrun() { "$PY" "$@" | tr -d '\r'; }   # helper output without CR (a Windows Python in the tests)
writes() { [ "$MODE" = real ] || [ "$MODE" = test ]; }
as_user() { run runuser -u "$GWU" -- "$@"; }

# --- questions: always from the terminal (works under curl | sudo bash), prompts never logged --------
open_tty() {
  if ! { exec 3<"$TTY_IN"; } 2>/dev/null || ! { exec 4>>"$TTY_OUT"; } 2>/dev/null; then
    die "$(t "δεν υπάρχει τερματικό για τις ερωτήσεις (τρέξτε το από ssh -t ή την κονσόλα)" \
             "no terminal for the questions (run it from ssh -t or the console)")" 5
  fi
}
ask() {   # ask VAR "question" [default]
  local __v=$1 __p=$2 __d=${3-} __a=""
  if [ -n "$__d" ]; then printf '%s [%s]: ' "$__p" "$__d" >&4; else printf '%s: ' "$__p" >&4; fi
  IFS= read -r -u 3 __a || die "$(t "η είσοδος έκλεισε" "input closed")" 4
  __a=${__a%$'\r'}
  [ -n "$__a" ] || __a=$__d
  printf -v "$__v" '%s' "$__a"
}
ask_secret() {   # ask_secret VAR "question": not shown, not logged
  local __v=$1 __a=""
  printf '%s: ' "$2" >&4
  IFS= read -r -s -u 3 __a || die "$(t "η είσοδος έκλεισε" "input closed")" 4
  printf '\n' >&4
  __a=${__a%$'\r'}
  printf -v "$__v" '%s' "$__a"
}
yesno() {   # yesno "question" Y|N  -> 0 = yes
  local a
  ask a "$1 ($(t "ν/ο" "y/n"))" "$(if [ "$2" = Y ]; then t ν y; else t ο n; fi)"
  case "$a" in y*|Y*|ν*|Ν*) return 0 ;; *) return 1 ;; esac
}
show() { printf '%s\n' "$*" >&4; }

# --- small Python helpers (JSON, hashes): data in files / stdin, never a password on a command line ---
PY_SECRETS_INFO='
import json, sys
try:
    d = json.load(open(sys.argv[1], encoding="utf-8"))
except FileNotFoundError:
    print("none"); sys.exit()
except Exception:
    print("invalid"); sys.exit()
u = d.get("users") or {}
if not u or any("..." in str(e.get("pw", "")) for e in u.values()):
    print("placeholder"); sys.exit()
print("users " + " ".join("%s(%s%s)" % (n, e.get("role"), ",licence" if e.get("licence") else "") for n, e in u.items()))
'
PY_HASH='
import sys
sys.path.insert(0, sys.argv[1])
import users
print(users.hash_pw(sys.stdin.read()))
'
PY_SECRETS_WRITE='
import json, os, sys
out, how, entries, example = sys.argv[1:5]
os.umask(0o077)
try:
    base = json.load(open(out, encoding="utf-8"))
except FileNotFoundError:
    base = json.load(open(example, encoding="utf-8"))
if how == "new" or not isinstance(base.get("users"), dict):
    base["users"] = {}     # new = the users from scratch; every other key (sync_key, academy_key ...) kept
for line in open(entries, encoding="utf-8").read().splitlines():
    name, role, lic, h = line.split("\t")
    e = {"role": role, "pw": h}
    if lic == "1":
        e["licence"] = True
    base["users"][name] = e
tmp = out + ".new"
with open(tmp, "w", encoding="utf-8") as f:
    json.dump(base, f, indent=1, ensure_ascii=False)
    f.write("\n")
os.chmod(tmp, 0o600)
os.replace(tmp, out)
'
PY_CFG_INFO='
import json, sys
try:
    d = json.load(open(sys.argv[1], encoding="utf-8"))
except FileNotFoundError:
    print("none"); sys.exit()
except Exception:
    print("invalid"); sys.exit()
a = str(d.get("aurix", ""))
state = "placeholder" if (not a or "VID:PID" in a) else "ok"
print("%s\t%s\t%s" % (state, d.get("boat", ""), a))
'
PY_CFG_WRITE='
import json, os, sys
out, base, boat, aurix, ais = sys.argv[1:6]
d = json.load(open(base, encoding="utf-8"))
d["boat"] = boat
if aurix:
    d["aurix"] = aurix
    d.pop("_aurix_tbd", None)
elif "VID:PID" in str(d.get("aurix", "")) or not d.get("aurix"):
    d["_aurix_tbd"] = ("TBD: the AURIX USB-UART adapter was not plugged in during setup. Plug it in, then "
                       "run setup_all.sh again (or: python3 gateway.py --list-serial and put the name here). "
                       "3.3 V TTL adapter ONLY - NEVER RS232.")
a = d.setdefault("ais", {})
if ais:
    a["enabled"] = True
    a["source"] = ais
    a.pop("_tbd", None)
elif "VID:PID" in str(a.get("source", "")) or not a.get("source"):
    a["enabled"] = False
    a["_tbd"] = "TBD: no AIS receiver chosen during setup - run setup_all.sh again with it plugged in"
tmp = out + ".new"
with open(tmp, "w", encoding="utf-8") as f:
    json.dump(d, f, indent=2, ensure_ascii=False)
    f.write("\n")
os.replace(tmp, out)
'
PY_SERIAL='
import sys
devs = []
for line in sys.stdin.read().splitlines():
    if not line.strip() or line.startswith(("AURIX:", "no serial")):
        continue
    if not line.startswith(" "):
        p = line.split(None, 1)
        devs.append([p[0], p[1] if len(p) > 1 else "", "", "", ""])
    elif devs and line.strip().startswith("use:"):
        n = line.split("use:", 1)[1].strip()
        if n.startswith("usb:"):
            devs[-1][2] = n
        elif not devs[-1][3]:
            devs[-1][3] = n
    elif devs and "CAUTION" in line:
        devs[-1][4] = "RS232?"
for d in devs:
    print("\t".join([d[0], d[1], d[2] or d[3], d[4]]))
'
PY_BOATS='
import glob, json, os, sys
for f in sorted(glob.glob(os.path.join(sys.argv[1], "boats", "*.json"))):
    try:
        print(json.load(open(f, encoding="utf-8"))["name"])
    except Exception:
        pass
'
PY_AREAS='
import json, os, sys
f = os.path.join(sys.argv[1], "charts", "areas.json")
if os.path.exists(f):
    print(" ".join(json.load(open(f, encoding="utf-8"))))
else:
    sys.path.insert(0, os.path.join(sys.argv[1], "tools"))
    import chartprep
    print(" ".join(chartprep.DEFAULT_AREAS))
'
PY_VISION_PKGS='
import sys
sys.path.insert(0, sys.argv[1])
import get_vision_python
print(" ".join(get_vision_python.PACKAGES))
'

# --- Ethernet ports (ip -br link: name state MAC flags) -------------------------------------------
eth_table() {   # name<TAB>state<TAB>mac<TAB>cable(1/0)
  q ip -br link | awk '{ n=$1; sub(/@.*/, "", n);
    if (n=="lo" || n ~ /^(wl|tailscale|docker|veth|virbr|br-|ww)/) next;
    if ($3 !~ /^([0-9a-fA-F][0-9a-fA-F]:){5}[0-9a-fA-F][0-9a-fA-F]$/) next;
    c = ($0 ~ /LOWER_UP/) ? 1 : 0; printf "%s\t%s\t%s\t%d\n", n, $2, tolower($3), c }'
}
eth_cabled() { eth_table | awk -F'\t' '$4==1 {print $1}' | sort; }
eth_mac() { eth_table | awk -F'\t' -v n="$1" '$1==n {print $3}'; }
link_mac() { [ -f "$1" ] && sed -n 's/^MACAddress=//p' "$1" | head -1; }
show_ports() {
  show "  $(t "θύρα" "port")              $(t "κατάσταση" "state")   MAC                $(t "καλώδιο" "cable")"
  eth_table | while IFS=$'\t' read -r n s m c; do
    show "$(printf '  %-17s %-11s %-18s %s' "$n" "$s" "$m" "$(if [ "$c" = 1 ]; then t "ΝΑΙ" "YES"; else t "όχι" "no"; fi)")"
  done
}
plug_test() {   # plug_test VAR "what" exclude-port
  local __v=$1 what=$2 excl=${3-} before after new k pick=""
  show ""
  show "$(t "Βγάλτε το καλώδιο" "Unplug the cable") $what $(t "από το PC (αν είναι βαλμένο)." "from the PC (if it is in).")"
  ask k "$(t "Enter όταν είναι έξω" "Enter when it is out")" ""
  before=$(eth_cabled)
  show "$(t "Τώρα βάλτε ΜΟΝΟ το καλώδιο" "Now plug in ONLY the cable") $what."
  ask k "$(t "Enter όταν είναι μέσα" "Enter when it is in")" ""
  for k in $(seq 0 "$WAIT_S"); do
    after=$(eth_cabled)
    new=$(comm -13 <(printf '%s\n' "$before") <(printf '%s\n' "$after") | grep -v "^$excl\$" | grep -v '^$' || true)
    [ -n "$new" ] && break
    [ "$WAIT_S" -gt 0 ] && sleep 1
  done
  if [ -n "$new" ] && [ "$(printf '%s\n' "$new" | wc -l)" -eq 1 ]; then
    if yesno "$(t "Βρέθηκε η θύρα" "Found port") $new ($(eth_mac "$new")) - $(t "σωστό;" "right?")" Y; then pick=$new; fi
  else
    show "$(t "Δεν βρέθηκε ακριβώς μία νέα θύρα με καλώδιο." "Not exactly one newly cabled port found.")"
  fi
  if [ -z "$pick" ]; then
    show_ports
    ask pick "$(t "Γράψτε το όνομα της θύρας (Enter = παράλειψη)" "Type the port name (Enter = skip)")" ""
    if [ -n "$pick" ] && [ -z "$(eth_mac "$pick")" ]; then
      show "$(t "Άγνωστη θύρα - παραλείπεται" "Unknown port - skipped"): $pick"; pick=""
    fi
  fi
  printf -v "$__v" '%s' "$pick"
}

# --- the final check list: OK / FAIL lines -------------------------------------------------------
CHECK_FAILS=0
chk() {   # chk 0|1 "label" "why if FAIL"
  if [ "$1" = 0 ]; then say "  OK    $2"; else say "  FAIL  $2 - $3"; CHECK_FAILS=$((CHECK_FAILS + 1)); fi
}
checklist() {
  CHECK_FAILS=0
  head1 "$(t "Λίστα ελέγχου" "Check list")"
  local s r v m n
  # service
  s=$(q systemctl is-active trygons-gateway)
  chk "$([ "$s" = active ]; echo $?)" "$(t "υπηρεσία trygons-gateway" "service trygons-gateway")" \
      "$(t "δεν τρέχει" "not running") ($s): journalctl -u trygons-gateway"
  # config
  q python3 "$REPO/gateway/gateway.py" --config "$CONFIG" --check-config >/dev/null; r=$?
  chk "$([ "$r" = 0 ]; echo $?)" "gateway.json (--check-config)" \
      "$(t "λάθη" "errors"): python3 $REPO/gateway/gateway.py --config $CONFIG --check-config"
  # AURIX link: the service's own log (this script never opens the UART)
  v=$(q journalctl -u trygons-gateway -b --no-pager -o cat | grep -E "AURIX (connected|link:)" | tail -1)
  if grep -q '"_aurix_tbd"' "$CONFIG" 2>/dev/null; then
    chk 1 "$(t "σύνδεση AURIX" "AURIX link")" "$(t "ο προσαρμογέας δεν ορίστηκε (TBD): βάλτε τον και τρέξτε ξανά" "adapter not set (TBD): plug it in and run again")"
  else
    case "$v" in
      "AURIX connected"*) chk 0 "$(t "σύνδεση AURIX" "AURIX link")" "" ;;
      *) chk 1 "$(t "σύνδεση AURIX" "AURIX link")" "$(t "όχι στο log της υπηρεσίας (καλώδιο; AURIX σε λειτουργία;)" "not in the service log (cable? AURIX powered?)") ${v:-}" ;;
    esac
  fi
  # storage
  v=$(q df -BG --output=avail "$DATA" | tail -1 | tr -dc '0-9')
  chk "$([ -n "$v" ] && [ "$v" -ge 20 ]; echo $?)" "$(t "αποθήκευση" "storage") $DATA (${v:-?} GB $(t "ελεύθερα" "free"))" \
      "$(t "λιγότερο από 20 GB ή δεν υπάρχει" "less than 20 GB or missing")"
  # lid
  s=$(q systemctl is-enabled sleep.target)
  chk "$(grep -q '^HandleLidSwitch=ignore' "$R/etc/systemd/logind.conf.d/trygons.conf" 2>/dev/null && [ "$s" = masked ]; echo $?)" \
      "$(t "καπάκι κλειστό = συνεχίζει (χωρίς ύπνο)" "lid closed = keeps running (no sleep)")" \
      "$(t "logind / sleep.target όχι ρυθμισμένα" "logind / sleep.target not set") (install_ubuntu.sh)"
  # battery 80 %
  v=$(cat "$R/sys/class/power_supply/BAT0/charge_control_end_threshold" 2>/dev/null)
  [ -n "$v" ] || v=$(q tlp-stat -b | sed -n 's/.*\(charge_control_end_threshold\|stop_charge_thresh\)[^=]*= *\([0-9]*\).*/\2/p' | head -1)
  chk "$([ "$v" = 80 ]; echo $?)" "$(t "μπαταρία: όριο φόρτισης 80 %" "battery: charge limit 80 %")" \
      "$(t "όριο" "limit") ${v:-?} (sudo tlp-stat -b)"
  # Tailscale
  chk "$(q tailscale status --json | grep -q '"BackendState": *"Running"'; echo $?)" "Tailscale (VPN)" \
      "$(t "δεν είναι συνδεδεμένο" "not connected"): sudo tailscale up --ssh --hostname=trygons-gw"
  # cameras network
  m=$(link_mac "$LINK_CAM")
  n=$(eth_table | awk -F'\t' '$1=="cam0"')
  if [ -z "$m" ]; then
    chk 1 "$(t "δίκτυο καμερών (cam0)" "camera network (cam0)")" "$(t "η θύρα των καμερών δεν ορίστηκε" "the camera port was not chosen")"
  elif [ -n "$n" ]; then
    chk "$(printf '%s' "$n" | awk -F'\t' '{exit !($4==1)}'; echo $?)" "$(t "δίκτυο καμερών (cam0)" "camera network (cam0)")" \
        "$(t "χωρίς καλώδιο / PoE switch σβηστό" "no cable / PoE switch off")"
  else
    chk 1 "$(t "δίκτυο καμερών (cam0)" "camera network (cam0)")" "$(t "χρειάζεται επανεκκίνηση (sudo reboot) - τα ονόματα θυρών ισχύουν μετά" "reboot needed (sudo reboot) - the port names apply after it")"
  fi
  # time
  s=$(q timedatectl show -p NTPSynchronized --value)
  v=$(q timedatectl show -p Timezone --value)
  chk "$([ "$s" = yes ] && [ "$v" = UTC ]; echo $?)" "$(t "ώρα (UTC, συγχρονισμένη)" "time (UTC, synchronised)")" \
      "$(t "ζώνη" "zone") ${v:-?}, NTP ${s:-?} - $(t "η πύλη τη ρυθμίζει από το GPS όταν το AURIX έχει fix" "the gateway sets it from GPS once the AURIX has a fix")"
  # charts
  chk "$([ -n "$(ls -A "$REPO/charts/${AREA:-attica}" 2>/dev/null)" ]; echo $?)" "$(t "χάρτες βάθους" "depth charts") (${AREA:-attica})" \
      "$(t "λείπουν: τρέξτε ξανά με internet" "missing: run again with internet")"
  say ""
  if [ "$CHECK_FAILS" = 0 ]; then say "$(t "ΟΛΑ ΟΚ" "ALL OK")"; else say "FAIL: $CHECK_FAILS"; fi
}

# ===================================================================================================
# 0. preflight: root, Ubuntu 24.04, log
if [ "$MODE" = test ] || [ "$MODE" = testdry ]; then UID_NOW=$(q id -u); else UID_NOW=$(id -u); fi
if [ "$UID_NOW" != 0 ] && [ "$MODE" != dry ] && [ "$MODE" != testdry ]; then
  t "Χρειάζεται root: τρέξτε το με sudo  (curl -sL ... | sudo bash)" "Needs root: run it with sudo  (curl -sL ... | sudo bash)"; echo
  exit 3
fi
OSID=$(sed -n 's/^ID=//p' "$R/etc/os-release" 2>/dev/null | tr -d '"')
OSVER=$(sed -n 's/^VERSION_ID=//p' "$R/etc/os-release" 2>/dev/null | tr -d '"')
if [ "$OSID" != ubuntu ] || [ "$OSVER" != 24.04 ]; then
  echo "$(t "Μόνο για Ubuntu 24.04 LTS (εδώ:" "Only for Ubuntu 24.04 LTS (this is:") ${OSID:-?} ${OSVER:-?}). Windows 11: gateway/windows/install_gateway.ps1 (README 3W)."
  exit 2
fi
if writes; then mkdir -p "$(dirname "$LOG")" && touch "$LOG" && chmod 600 "$LOG"; fi
say "Trygons C2 - $(t "εγκατάσταση πύλης (gateway PC)" "gateway PC setup") $(date '+%F %T') mode=$MODE"
{ [ "$MODE" = dry ] || [ "$MODE" = testdry ]; } && say "$(t "ΔΟΚΙΜΗ (--dry-run): τίποτα δεν αλλάζει" "DRY RUN: nothing is changed")"
say "$(t "Το AURIX δεν μπαίνει ποτέ στο δίκτυο. Προσαρμογέας AURIX: 3.3 V TTL USB-UART - ΠΟΤΕ RS232 (+-12 V καταστρέφει τα pins)." \
         "The AURIX is never on the network. AURIX adapter: 3.3 V TTL USB-UART - NEVER RS232 (+-12 V destroys its pins).")"
say "$(t "Τρέξτε το στο λιμάνι, σκάφος σε STANDBY (η επανεκκίνηση της πύλης = GATEWAY LOST για λίγα δευτερόλεπτα)." \
         "Run it in harbour, boat in STANDBY (restarting the gateway = GATEWAY LOST for a few seconds).")"

if [ "$CHECKS_ONLY" = 1 ]; then
  AREA=$(sed -n 's/^area=//p' "$ETC/setup.state" 2>/dev/null | head -1)
  checklist
  [ "$CHECK_FAILS" = 0 ]; return
fi
open_tty

# 1. basic packages, the user, the repository (needed for the questions: boats/, the serial list)
head1 "$(t "1. βασικά πακέτα, χρήστης, αποθετήριο" "1. basic packages, user, repository")"
run apt-get update || fail_step "apt-get update"
run env DEBIAN_FRONTEND=noninteractive apt-get install -y git python3-serial python3-venv curl || fail_step "apt-get install git python3-serial python3-venv"
if ! q id "$GWU" >/dev/null; then run adduser --disabled-password --gecos "" "$GWU" || fail_step "adduser $GWU"; fi
if [ -d "$REPO/.git" ]; then
  as_user git -C "$REPO" pull --ff-only origin "$BRANCH" || fail_step "$(t "git pull (τοπικές αλλαγές; κρατήθηκε ο τωρινός κώδικας)" "git pull (local changes? the current code is kept)")"
else
  as_user git clone --branch "$BRANCH" "$REPO_URL" "$REPO" || die "git clone $REPO_URL ($(t "ιδιωτικό repo; κάντε clone πρώτα και τρέξτε sudo bash gateway/setup_all.sh" "private repo? clone first and run sudo bash gateway/setup_all.sh"))"
fi
[ -f "$REPO/gateway/gateway.py" ] || [ "$MODE" = dry ] || die "$REPO/gateway/gateway.py missing"

# 2. questions (everything is asked first; then it runs by itself)
head1 "$(t "2. ερωτήσεις" "2. questions")"
# 2a boat name + config
CFG_INFO=$(pyrun -c "$PY_CFG_INFO" "$CONFIG" 2>/dev/null || echo none)
CFG_STATE=${CFG_INFO%%$'\t'*}
CFG_BOAT=$(printf '%s' "$CFG_INFO" | cut -f2 -s)
CFG_AURIX=$(printf '%s' "$CFG_INFO" | cut -f3 -s)
KEEP_CFG=0
if [ "$CFG_STATE" = invalid ]; then
  yesno "$(t "Το gateway.json δεν διαβάζεται - να γίνει νέο; (το παλιό μένει ως gateway.json.bak)" "gateway.json can't be read - make a new one? (the old one is kept as gateway.json.bak)")" N || KEEP_CFG=1
fi
if [ "$CFG_STATE" = ok ]; then
  if yesno "$(t "Υπάρχει ήδη" "Existing") gateway.json ($(t "σκάφος" "boat") $CFG_BOAT, AURIX $CFG_AURIX) - $(t "να μείνει όπως είναι;" "keep it as it is?")" Y; then KEEP_CFG=1; fi
fi
BOAT=$CFG_BOAT
if [ "$KEEP_CFG" = 0 ]; then
  BOATS=$(pyrun -c "$PY_BOATS" "$REPO" 2>/dev/null | tr '\n' ' ')
  [ -n "$BOATS" ] && show "$(t "Σκάφη στο boats/:" "Boats in boats/:") $BOATS"
  DEF=${CFG_BOAT:-}
  [ "$DEF" = trygons-c2 ] && DEF=""
  [ -n "$DEF" ] || DEF=$(printf '%s' "$BOATS" | awk '{print $1}')
  while :; do
    ask BOAT "$(t "Όνομα σκάφους (όπως στο boats/)" "Boat name (as in boats/)")" "$DEF"
    printf '%s' "$BOAT" | grep -Eq '^[A-Za-z0-9_-]{1,32}$' && break
    show "$(t "Μόνο γράμματα, αριθμοί, - _ (έως 32)" "Letters, digits, - _ only (up to 32)")"
  done
  case " $BOATS " in *" $BOAT "*) ;; *) show "$(t "ΠΡΟΣΟΧΗ: δεν υπάρχει στο boats/ - ο σταθμός θα αρνηθεί τη σύνδεση" "CAUTION: not in boats/ - the station will refuse to connect")" ;; esac
fi
# 2b users -> secrets.json
SEC_INFO=$(pyrun -c "$PY_SECRETS_INFO" "$SECRETS" 2>/dev/null || echo none)
SEC_HOW=new
case "$SEC_INFO" in
  users*)
    show "$(t "Υπάρχον secrets.json, χρήστες:" "Existing secrets.json, users:") ${SEC_INFO#users }"
    ask a "$(t "Κ = κράτα τους, Π = πρόσθεσε κι άλλους, Ν = από την αρχή" "K = keep them, A = add more, N = start over")" "$(t Κ K)"
    case "$a" in Π*|π*|A*|a*) SEC_HOW=add ;; Ν*|ν*|N*|n*) if yesno "$(t "Σίγουρα; οι παλιοί χρήστες σβήνονται" "Sure? the old users are removed")" N; then SEC_HOW=new; else SEC_HOW=keep; fi ;; *) SEC_HOW=keep ;; esac ;;
  invalid)
    show "$(t "ΠΡΟΣΟΧΗ: το secrets.json δεν διαβάζεται - δεν αγγίζεται (διορθώστε το με το χέρι)" "CAUTION: secrets.json can't be read - left alone (fix it by hand)")"
    SEC_HOW=keep ;;
esac
ENTRIES=""
NUSERS=0
if [ "$SEC_HOW" != keep ]; then
  if writes; then mkdir -p "$RUNDIR"; ENTRIES=$(umask 077; mktemp "$RUNDIR/trygons-users.XXXXXX"); fi
  show "$(t "Χρήστες της πύλης (HELM = οδηγεί, VIEWER = μόνο βλέπει). Κενό όνομα = τέλος." "Gateway users (HELM = drives, VIEWER = view only). Empty name = done.")"
  while :; do
    ask UNAME "$(t "Όνομα χρήστη" "User name")" ""
    [ -n "$UNAME" ] || break
    printf '%s' "$UNAME" | grep -Eq '^[A-Za-z0-9_.-]{1,32}$' || { show "$(t "Μόνο γράμματα, αριθμοί, . - _" "Letters, digits, . - _ only")"; continue; }
    ask UROLE "$(t "Ρόλος HELM / VIEWER" "Role HELM / VIEWER")" HELM
    UROLE=$(printf '%s' "$UROLE" | tr '[:lower:]' '[:upper:]')
    case "$UROLE" in HELM|VIEWER) ;; *) show "HELM / VIEWER"; continue ;; esac
    ULIC=0
    if [ "$UROLE" = HELM ] && yesno "$(t "Έχει δίπλωμα χειροκίνητης οδήγησης (κυβερνήτης);" "Holds a manual driving licence (captain)?")" N; then ULIC=1; fi
    UPW=""
    for k in 1 2 3; do
      ask_secret PW1 "$(t "Κωδικός για" "Password for") $UNAME ($(t "τουλάχιστον 10 χαρακτήρες, δεν φαίνεται" "at least 10 characters, not shown"))"
      ask_secret PW2 "$(t "Ξανά" "Again")"
      if [ "${#PW1}" -lt 10 ]; then show "$(t "Πολύ μικρός" "Too short")"; elif [ "$PW1" != "$PW2" ]; then show "$(t "Δεν είναι ίδιοι" "Not the same")"; else UPW=$PW1; break; fi
    done
    PW1=""; PW2=""
    if [ -z "$UPW" ]; then show "$(t "Ο χρήστης παραλείπεται" "User skipped"): $UNAME"; continue; fi
    H=$(printf '%s' "$UPW" | pyrun -c "$PY_HASH" "$REPO/gateway")
    UPW=""
    case "$H" in pbkdf2_sha256\$*) ;; *) show "hash failed - $(t "παραλείπεται" "skipped")"; continue ;; esac
    if writes; then printf '%s\t%s\t%s\t%s\n' "$UNAME" "$UROLE" "$ULIC" "$H" >> "$ENTRIES"; fi
    H=""
    NUSERS=$((NUSERS + 1))
  done
  if [ "$NUSERS" = 0 ]; then
    show "$(t "Κανένας νέος χρήστης: το secrets.json δεν αλλάζει" "No new user: secrets.json is not changed")"
    SEC_HOW=keep
  fi
fi
# 2c Tailscale
TS_KEYFILE=""
if q tailscale status --json | grep -q '"BackendState": *"Running"'; then
  show "$(t "Το Tailscale είναι ήδη συνδεδεμένο - μένει." "Tailscale is already connected - kept.")"
else
  ask_secret TSKEY "$(t "Tailscale auth key (tskey-..., δεν φαίνεται; Enter = αργότερα)" "Tailscale auth key (tskey-..., not shown; Enter = later)")"
  if [ -n "$TSKEY" ]; then
    case "$TSKEY" in tskey-*) ;; *) show "$(t "ΠΡΟΣΟΧΗ: δεν αρχίζει με tskey-" "CAUTION: does not start with tskey-")" ;; esac
    if writes; then mkdir -p "$RUNDIR"; TS_KEYFILE=$(umask 077; mktemp "$RUNDIR/trygons-ts.XXXXXX"); printf '%s' "$TSKEY" > "$TS_KEYFILE"; else TS_KEYFILE="$RUNDIR/trygons-ts.KEY"; fi
  fi
  TSKEY=""
fi
# shellcheck disable=SC2329  # called by the EXIT trap
cleanup() { [ -n "${TS_KEYFILE:-}" ] && [ -f "$TS_KEYFILE" ] && rm -f "$TS_KEYFILE"; [ -n "${ENTRIES:-}" ] && [ -f "$ENTRIES" ] && rm -f "$ENTRIES"; }
trap cleanup EXIT
# 2d Ethernet ports
LAN_MAC=$(link_mac "$LINK_LAN"); CAM_MAC=$(link_mac "$LINK_CAM")
KEEP_NET=0
if [ -n "$LAN_MAC" ] && [ -n "$CAM_MAC" ]; then
  yesno "$(t "Θύρες ήδη ορισμένες: δρομολογητής" "Ports already set: router") $LAN_MAC, $(t "κάμερες" "cameras") $CAM_MAC - $(t "να μείνουν;" "keep them?")" Y && KEEP_NET=1
fi
if [ "$KEEP_NET" = 0 ]; then
  LAN_MAC=""; CAM_MAC=""
  show "$(t "Θύρες Ethernet του PC:" "The PC's Ethernet ports:")"
  show_ports
  if [ "$MODE" = real ]; then for p in $(eth_table | cut -f1); do run ip link set dev "$p" up >/dev/null; done; fi
  plug_test P_LAN "$(t "του ΔΡΟΜΟΛΟΓΗΤΗ (Teltonika)" "of the ROUTER (Teltonika)")" ""
  [ -n "$P_LAN" ] && LAN_MAC=$(eth_mac "$P_LAN")
  plug_test P_CAM "$(t "των ΚΑΜΕΡΩΝ (PoE switch)" "of the CAMERAS (PoE switch)")" "$P_LAN"
  [ -n "$P_CAM" ] && CAM_MAC=$(eth_mac "$P_CAM")
  if [ -n "$LAN_MAC" ] && [ "$LAN_MAC" = "$CAM_MAC" ]; then show "$(t "Η ίδια θύρα δύο φορές - παραλείπεται" "The same port twice - skipped")"; LAN_MAC=""; CAM_MAC=""; fi
  if [ -z "$LAN_MAC" ] || [ -z "$CAM_MAC" ]; then show "$(t "Οι θύρες δεν ορίστηκαν: τα ονόματα boatlan0 / cam0 παραλείπονται (τρέξτε ξανά)" "Ports not set: the names boatlan0 / cam0 are skipped (run again)")"; LAN_MAC=""; CAM_MAC=""; fi
fi
# 2e chart area
AREAS=$(pyrun -c "$PY_AREAS" "$REPO" 2>/dev/null)
[ -n "$AREAS" ] || AREAS=attica
OLD_AREA=$(sed -n 's/^area=//p' "$ETC/setup.state" 2>/dev/null | head -1)
show "$(t "Περιοχές χαρτών:" "Chart areas:") $AREAS"
ask AREA "$(t "Περιοχή του σκάφους" "The boat's area")" "${OLD_AREA:-$(printf '%s' "$AREAS" | awk '{print $1}')}"
case " $AREAS " in *" $AREA "*) ;; *) show "$(t "Άγνωστη περιοχή (προσθέστε την στο charts/areas.json) - χρησιμοποιείται" "Unknown area (add it to charts/areas.json) - using") $(printf '%s' "$AREAS" | awk '{print $1}')"; AREA=$(printf '%s' "$AREAS" | awk '{print $1}') ;; esac
MAKE_CHARTS=1
if [ -n "$(ls -A "$REPO/charts/$AREA" 2>/dev/null)" ]; then
  yesno "$(t "Οι χάρτες" "Charts for") $AREA $(t "υπάρχουν - να ξαναγίνουν;" "exist - make them again?")" N || MAKE_CHARTS=0
fi
# 2f serial adapters (only LISTED - never opened)
AURIX_NAME=""; AIS_NAME=""
if [ "$KEEP_CFG" = 0 ]; then
  SER=$(q python3 "$REPO/gateway/gateway.py" --list-serial | pyrun -c "$PY_SERIAL" 2>/dev/null)
  if [ -z "$SER" ]; then
    show "$(t "Δεν βρέθηκε USB σειριακός προσαρμογέας: AURIX / AIS μένουν TBD (βάλτε τους και τρέξτε ξανά)." "No USB serial adapter found: AURIX / AIS stay TBD (plug them in and run again).")"
  else
    show "$(t "USB σειριακοί προσαρμογείς (μόνο λίστα - δεν ανοίγονται):" "USB serial adapters (listed only - never opened):")"
    i=0
    while IFS=$'\t' read -r dev desc name warn; do
      i=$((i + 1)); show "  $i) $dev  $desc  -> ${name:-?} ${warn:+  $(t "ΠΡΟΣΟΧΗ: μάλλον RS232 - ΟΧΙ για το AURIX" "CAUTION: probably RS232 - NOT for the AURIX")}"
    done <<< "$SER"
    show "$(t "AURIX = ΜΟΝΟ 3.3 V TTL USB-UART (FTDI / CP2102 στα 3.3 V). ΠΟΤΕ RS232." "AURIX = ONLY a 3.3 V TTL USB-UART (FTDI / CP2102 at 3.3 V). NEVER RS232.")"
    ask k "$(t "Αριθμός του προσαρμογέα AURIX (Enter = TBD)" "Number of the AURIX adapter (Enter = TBD)")" ""
    case "$k" in ''|*[!0-9]*) ;; *) AURIX_NAME=$(printf '%s\n' "$SER" | sed -n "${k}p" | cut -f3) ;; esac
    ask k "$(t "Αριθμός του δέκτη AIS (Enter = κανένας / TBD)" "Number of the AIS receiver (Enter = none / TBD)")" ""
    case "$k" in ''|*[!0-9]*) ;; *) AIS_NAME=$(printf '%s\n' "$SER" | sed -n "${k}p" | cut -f3) ;; esac
    if [ -n "$AIS_NAME" ] && [ "$AIS_NAME" = "$AURIX_NAME" ]; then show "$(t "Ίδια συσκευή - το AIS μένει TBD" "Same device - AIS stays TBD")"; AIS_NAME=""; fi
  fi
fi
show ""
show "$(t "Ευχαριστώ - από εδώ συνεχίζει μόνο του (μπορεί να πάρει 20-60 λεπτά)." "Thank you - from here it runs by itself (it may take 20-60 minutes).")"
logline "answers: boat=$BOAT keep_cfg=$KEEP_CFG users=$SEC_HOW/$NUSERS tailscale_key=$([ -n "$TS_KEYFILE" ] && echo given || echo none) lan=$LAN_MAC cam=$CAM_MAC area=$AREA charts=$MAKE_CHARTS aurix=${AURIX_NAME:-TBD} ais=${AIS_NAME:-TBD}"

# 3. full upgrade + time zone
head1 "$(t "3. ενημέρωση συστήματος" "3. system upgrade")"
run env DEBIAN_FRONTEND=noninteractive apt-get -y -o Dpkg::Options::=--force-confold full-upgrade || fail_step "apt-get full-upgrade"
run timedatectl set-timezone UTC || fail_step "timedatectl set-timezone UTC"

# 4. install_ubuntu.sh (packages, user, folders, lid, battery 80 %, updates, interface names, service)
head1 "4. install_ubuntu.sh"
OLD_LAN=$(link_mac "$LINK_LAN"); OLD_CAM=$(link_mac "$LINK_CAM")
if [ -n "$LAN_MAC" ]; then
  run env LAN_MAC="$LAN_MAC" CAM_MAC="$CAM_MAC" GWU="$GWU" sh "$REPO/gateway/systemd/install_ubuntu.sh" || fail_step "install_ubuntu.sh"
else
  run env GWU="$GWU" sh "$REPO/gateway/systemd/install_ubuntu.sh" || fail_step "install_ubuntu.sh"
fi
if [ -n "$LAN_MAC" ] && { [ "$LAN_MAC" != "$OLD_LAN" ] || [ "$CAM_MAC" != "$OLD_CAM" ]; }; then REBOOT_NOTE=1; fi

# 5. secrets.json + gateway.json
head1 "$(t "5. secrets.json + gateway.json" "5. secrets.json + gateway.json")"
if writes; then mkdir -p "$ETC"; fi
if [ "$SEC_HOW" = keep ]; then
  say "  secrets.json: $(t "χωρίς αλλαγή" "unchanged")"
elif writes; then
  if "$PY" -c "$PY_SECRETS_WRITE" "$SECRETS" "$SEC_HOW" "$ENTRIES" "$REPO/gateway/secrets.example.json"; then
    chmod 600 "$SECRETS"
    [ "$MODE" = real ] && chown "$GWU": "$SECRETS"
    say "  secrets.json: $NUSERS $(t "χρήστες γράφτηκαν (κωδικοί μόνο ως hash, mode 600)" "users written (passwords as hashes only, mode 600)") ($SEC_HOW)"
    CHANGED=1
  else
    fail_step "secrets.json"
  fi
  rm -f "$ENTRIES"
else
  say "[DRY-RUN] $(t "θα γραφτεί" "would write") $SECRETS ($NUSERS users, hashed, mode 600, $SEC_HOW)"
fi
if [ "$KEEP_CFG" = 1 ]; then
  say "  gateway.json: $(t "χωρίς αλλαγή" "unchanged")"
elif writes; then
  BASE="$REPO/gateway/config.example.json"
  [ -f "$CONFIG" ] && [ "$CFG_STATE" != invalid ] && BASE=$CONFIG
  if [ -f "$CONFIG" ] && [ "$CFG_STATE" != none ]; then cp -p "$CONFIG" "$CONFIG.bak"; fi
  if "$PY" -c "$PY_CFG_WRITE" "$CONFIG" "$BASE" "$BOAT" "$AURIX_NAME" "$AIS_NAME"; then
    [ "$MODE" = real ] && chown "$GWU": "$CONFIG"
    say "  gateway.json: boat=$BOAT aurix=${AURIX_NAME:-TBD} ais=${AIS_NAME:-TBD}"
    CHANGED=1
  else
    fail_step "gateway.json"
  fi
else
  say "[DRY-RUN] $(t "θα γραφτεί" "would write") $CONFIG (boat=$BOAT aurix=${AURIX_NAME:-TBD} ais=${AIS_NAME:-TBD})"
fi
if writes; then printf 'area=%s\n' "$AREA" > "$ETC/setup.state"; fi

# 6. Tailscale
head1 "6. Tailscale"
if ! q tailscale version >/dev/null; then
  run sh -c "curl -fsSL https://tailscale.com/install.sh | sh" || fail_step "Tailscale install"
fi
if [ -n "$TS_KEYFILE" ]; then
  run tailscale up --auth-key="file:$TS_KEYFILE" --ssh --hostname=trygons-gw || fail_step "tailscale up"
  [ "$MODE" = real ] && rm -f "$TS_KEYFILE"
  say "  $(t "Στο Tailscale admin console: Machines -> trygons-gw -> Disable key expiry" "In the Tailscale admin console: Machines -> trygons-gw -> Disable key expiry")"
fi

# 7. video relay, vision, charts (as the gateway user, into the repository: tools/bin/, charts/)
head1 "$(t "7. MediaMTX, όραση, χάρτες" "7. MediaMTX, vision, charts")"
as_user python3 "$REPO/tools/get_mediamtx.py" || fail_step "tools/get_mediamtx.py"
# tools/get_vision_python.py is the Windows way; on Ubuntu the same pinned packages go into a venv
# at tools/bin/vision-python/ (toolpaths.vision_python() runs tools/bin/vision-python/python)
VP="$REPO/tools/bin/vision-python"
if ! q "$VP/python" -c "import onnxruntime, numpy" >/dev/null; then
  PKGS=$(pyrun -c "$PY_VISION_PKGS" "$REPO/tools" 2>/dev/null)
  [ -n "$PKGS" ] || PKGS="onnxruntime==1.20.1 numpy==2.1.3"
  as_user python3 -m venv "$VP" || fail_step "venv $VP"
  # shellcheck disable=SC2086
  as_user "$VP/bin/python" -m pip install --no-warn-script-location $PKGS || fail_step "pip install $PKGS"
  as_user sh -c "printf '#!/bin/sh\nexec \"\$(dirname \"\$0\")/bin/python\" \"\$@\"\n' > '$VP/python' && chmod 755 '$VP/python'" || fail_step "$VP/python"
fi
as_user python3 "$REPO/tools/get_vision_model.py" || fail_step "tools/get_vision_model.py"
if [ "$MAKE_CHARTS" = 1 ]; then
  as_user python3 "$REPO/tools/chartprep.py" --area "$AREA" || fail_step "$(t "χάρτες" "charts") $AREA (internet?)"
else
  say "  $(t "χάρτες" "charts") $AREA: $(t "υπάρχουν, χωρίς αλλαγή" "present, unchanged")"
fi

# 8. config check + start
head1 "$(t "8. έλεγχος ρυθμίσεων + εκκίνηση" "8. config check + start")"
q python3 "$REPO/gateway/gateway.py" --config "$CONFIG" --check-config; CC=$?
if [ "$CC" = 0 ]; then
  if [ "$(q systemctl is-active trygons-gateway)" = active ] && [ "$CHANGED" = 0 ]; then
    say "  $(t "η υπηρεσία τρέχει ήδη, καμία αλλαγή - δεν επανεκκινείται" "the service already runs, nothing changed - not restarted")"
  else
    run systemctl enable trygons-gateway || fail_step "systemctl enable"
    run systemctl restart trygons-gateway || fail_step "systemctl restart trygons-gateway"
    if [ "$MODE" = real ]; then
      for k in $(seq 1 15); do
        q journalctl -u trygons-gateway --since "@$T0" --no-pager -o cat | grep -q "AURIX connected" && break
        sleep 2
      done
    fi
  fi
else
  fail_step "$(t "gateway.json έχει λάθη (π.χ. AURIX TBD): η υπηρεσία ΔΕΝ ξεκίνησε" "gateway.json has errors (e.g. AURIX TBD): the service was NOT started")"
fi

# 9. the check list
[ -f "$R/var/run/reboot-required" ] && REBOOT_NOTE=1
checklist
if [ -n "$FAILED_STEPS" ]; then say "$(t "Βήματα που απέτυχαν:" "Steps that failed:")$FAILED_STEPS"; fi
if [ "$REBOOT_NOTE" = 1 ]; then
  say "$(t "Επανεκκίνηση: sudo reboot, μετά  sudo bash $REPO/gateway/setup_all.sh --checks  (όλα OK)" \
           "Reboot: sudo reboot, then  sudo bash $REPO/gateway/setup_all.sh --checks  (all OK)")"
fi
say "Log: $LOG"
[ "$CHECK_FAILS" = 0 ] && [ -z "$FAILED_STEPS" ]
}

# the whole script is read before anything runs (safe under curl | sudo bash)
main "$@"
exit $?
