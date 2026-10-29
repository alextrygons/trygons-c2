"""The online academy (ACADEMY_SPEC J, step 21): content only, no route to any boat. No network module here.

How it works (decision D17): the academy keeps running ONLY on the academy host (a shore PC, never a boat
gateway: simguard refuses there). A trainee reaches it from home through the remote link = a remote-desktop /
SSH session to that PC over Tailscale; the academy itself opens no port and no connection (AC1-AC3 unchanged).
The Tailscale policy lets a trainee device reach ONLY the academy host - never a boat (check_acl below proves a
policy file; docs: ACADEMY_SPEC D17, MANUAL).

What an online session changes:
  * is_online(): the academy runs in a remote session (Windows RDP: SESSIONNAME "RDP-...", SSH_CONNECTION /
    SSH_CLIENT, or TRYGONS_ACADEMY_ONLINE=1);
  * exams online only while an instructor marked the trainee's session SUPERVISED (accounts.Store.supervise:
    the instructor's 2FA login, a time limit);
  * NOTHING online upgrades a level (accounts.set_level / grant_practical refuse) - role upgrades only via the
    practical test, which runs on the boat's own ground station, never in the online academy;
  * CAPTAIN + instructor logins need 2FA (auth.py) - online and on the shore PC alike.
"""
import ipaddress
import os

import simguard

ONLINE_ENV = "TRYGONS_ACADEMY_ONLINE"


def is_online(env=None):
    env = os.environ if env is None else env
    if env.get(ONLINE_ENV) == "1":
        return True
    if str(env.get("SESSIONNAME", "")).upper().startswith("RDP-"):
        return True
    return bool(env.get("SSH_CONNECTION") or env.get("SSH_CLIENT"))


# An EXAMPLE Tailscale policy (HuJSON / JSON) for the online academy: trainees (tag:trainee) reach only the
# academy host's remote desktop (tag:academy, port 3389) / SSH (22); boats (tag:boat) are reachable only from
# the crew stations (tag:station). check_acl() checks a real policy the same way.
EXAMPLE_ACL = {
    "tagOwners": {"tag:trainee": ["autogroup:admin"], "tag:academy": ["autogroup:admin"],
                  "tag:boat": ["autogroup:admin"], "tag:station": ["autogroup:admin"]},
    "acls": [
        {"action": "accept", "src": ["tag:trainee"], "dst": ["tag:academy:3389", "tag:academy:22"]},
        {"action": "accept", "src": ["tag:station"], "dst": ["tag:boat:*"]},
    ],
}
TRAINEE = "tag:trainee"


def _dst_host(d):
    """'tag:academy:3389' -> 'tag:academy'; '100.1.2.3:22' -> '100.1.2.3'; '*:*' -> '*'."""
    d = str(d)
    if d.startswith("["):                                    # [ipv6]:port
        return d[1:d.index("]")] if "]" in d else d
    return d.rsplit(":", 1)[0] if ":" in d else d


def check_acl(policy, boats_dir=None, trainee=TRAINEE, academy=("tag:academy",)):
    """Problems of a Tailscale policy for the online academy ([] = fine): a rule from the trainees (or from
    everyone) that reaches anything but the academy host - a boat's tag, a boats/*.json address, the whole
    Tailscale range, '*' - is a problem; so is no rule at all for the trainees."""
    probs = []
    _names, ips = simguard.boat_files(boats_dir)
    rules = (policy or {}).get("acls") or []
    seen = False
    for i, r in enumerate(rules):
        if not isinstance(r, dict) or r.get("action") != "accept":
            continue
        src = [str(x) for x in r.get("src") or []]
        if not (trainee in src or "*" in src or "autogroup:member" in src):
            continue
        seen = seen or trainee in src
        for d in r.get("dst") or []:
            host = _dst_host(d)
            if host in academy:
                continue
            why = None
            if host in ("*", "autogroup:internet") or host.startswith("tag:boat"):
                why = "reaches %s" % d
            elif host in ips:
                why = "reaches a boat's address %s" % d
            else:
                try:
                    net = ipaddress.ip_network(host, strict=False)
                    if net.overlaps(simguard.TAILSCALE) or any(ipaddress.ip_address(ip) in net for ip in ips
                                                               if _is_ip(ip)):
                        why = "reaches the Tailscale range / a boat (%s)" % d
                except ValueError:
                    why = "reaches %s (not the academy host)" % d
            if why:
                probs.append("rule %d (src %s): %s" % (i + 1, ",".join(src), why))
    if not seen:
        probs.append("no rule for %s (the trainees' devices)" % trainee)
    return probs


def _is_ip(s):
    try:
        ipaddress.ip_address(s)
        return True
    except ValueError:
        return False
