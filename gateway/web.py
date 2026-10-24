"""A simple web page for the iPhone (section B6): http://trygons-gw:8080 over
the VPN. Log in with a gateway user (browser login box).

VIEW ONLY by default. Only if "web_helm": true in gateway.json AND the user
has the HELM role, the page also shows: take / release HELM, STANDBY,
heading -10 / -1 / +1 / +10 (HDG LOCK). These go through the same checks
as every station (HELM on the AURIX, stale / duplicate / rate checks).
"""
import asyncio
import base64
import json
import secrets
import time

from common import log, proto
from stations import Station

PAGE = """<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Trygons C2</title>
<style>
body{font-family:-apple-system,Segoe UI,sans-serif;margin:0;padding:12px;background:#fff;color:#000}
.mode{font-size:34px;font-weight:700;padding:8px;color:#fff;background:#606060;text-align:center;border-radius:8px}
.alarm{background:#d00000;color:#fff;font-size:20px;font-weight:700;padding:8px;margin-top:8px;border-radius:8px}
.banner{background:#ff8c00;color:#000;font-size:20px;font-weight:700;padding:8px;margin-top:8px;border-radius:8px}
table{width:100%;font-size:20px;margin-top:8px;border-collapse:collapse}
td{padding:4px 2px;border-bottom:1px solid #ddd}td.v{font-weight:700;text-align:right}
button{font-size:22px;padding:12px;margin:4px;border-radius:8px;border:2px solid #000;background:#eee;min-width:70px}
#helm{display:none;margin-top:10px}.stby{background:#d00000;color:#fff;width:100%}
.small{font-size:14px;color:#555;margin-top:8px}
</style></head><body>
<div id="mode" class="mode">---</div>
<div id="alarms"></div>
<table>
<tr><td>Heading</td><td class="v" id="hdg">-</td></tr>
<tr><td>Target</td><td class="v" id="tgt">-</td></tr>
<tr><td>Speed / course</td><td class="v" id="sog">-</td></tr>
<tr><td>Throttle / gear</td><td class="v" id="thr">-</td></tr>
<tr><td>Position</td><td class="v" id="pos">-</td></tr>
<tr><td>Power limit</td><td class="v" id="plim">-</td></tr>
<tr><td>HELM</td><td class="v" id="helmname">-</td></tr>
<tr><td>Internet</td><td class="v" id="chan">-</td></tr>
</table>
<div id="helm">
<button onclick="cmd('helm_take')">Take HELM</button><button onclick="cmd('helm_release')">Release</button>
<button class="stby" onclick="cmd('standby')">STANDBY</button>
<div><button onclick="cmd('hdg',-10)">-10</button><button onclick="cmd('hdg',-1)">-1</button>
<button onclick="cmd('hdg',1)">+1</button><button onclick="cmd('hdg',10)">+10</button></div>
<div id="answer" class="small"></div></div>
<div class="small" id="age"></div>
<script>
const COL={STANDBY:'#606060',MANUAL:'#ff8c00',HDG_LOCK:'#007a00',AUTO:'#7a00a0',TRAINING:'#a06000',REMOTE_MANUAL:'#0033cc',BENCH_TEST:'#008080'};
let last=0;
async function poll(){
 try{const r=await fetch('api/status',{cache:'no-store'});const d=await r.json();last=Date.now();
  const m=document.getElementById('mode');m.textContent=d.mode;m.style.background=COL[d.mode]||'#606060';
  document.getElementById('alarms').innerHTML=(d.alarms.length?'<div class="alarm">ALARM: '+d.alarms.join(', ')+'</div>':'')+
   (d.banner?'<div class="banner">'+d.banner+'</div>':'');
  hdg.textContent=d.heading;tgt.textContent=d.target;sog.textContent=d.sog;thr.textContent=d.throttle;
  pos.textContent=d.pos;plim.textContent=d.plim;helmname.textContent=d.helm;chan.textContent=d.channel;
  document.getElementById('helm').style.display=d.web_helm?'block':'none';
 }catch(e){}
 document.getElementById('age').textContent=last?('updated '+((Date.now()-last)/1000).toFixed(0)+' s ago'):'no data';
}
async function cmd(c,d){const r=await fetch('api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},
 body:JSON.stringify({cmd:c,d:d})});const a=await r.json();document.getElementById('answer').textContent=a.text;}
setInterval(poll,1000);poll();
</script></body></html>"""


class _WebWriter:
    """Lets the web page use the station code: answers land in a dict."""

    def __init__(self):
        self.acks = {}

    def write(self, line):
        m = json.loads(line)
        if m.get("t") == "ack":
            self.acks[m["seq"]] = m

    def close(self):
        pass


class WebServer:
    def __init__(self, gw, cfg):
        self.gw = gw
        self.port = cfg.get("web_port", 8080)
        self.listen = cfg.get("stations", {}).get("listen", "0.0.0.0")
        self.helm_allowed = bool(cfg.get("web_helm", False))
        self.web_stations = {}              # user -> Station (registered while used)
        self.server = None

    async def start(self):
        self.server = await asyncio.start_server(self._client, self.listen, self.port)
        log.info("web page on port %d (HELM from the web %s)", self.port, "ON" if self.helm_allowed else "off")

    async def stop(self):
        if self.server:
            self.server.close()

    def _auth(self, headers, ip):
        h = headers.get("authorization", "")
        if not h.lower().startswith("basic "):
            return None, None
        try:
            user, pw = base64.b64decode(h[6:]).decode().split(":", 1)
        except (ValueError, UnicodeDecodeError):
            return None, None
        role, why = self.gw.users.login(ip, user, password=pw)
        if role is None:
            self.gw.audit("login_failed", user=user, ip=ip, why=why, via="web")
        return user, role

    async def _client(self, reader, writer):
        ip = (writer.get_extra_info("peername") or ("?",))[0]
        try:
            req = await asyncio.wait_for(reader.readline(), 10)
            parts = req.decode("latin-1").split()
            if len(parts) < 2:
                return
            method, path = parts[0], parts[1].split("?")[0]
            headers = {}
            while True:
                line = await asyncio.wait_for(reader.readline(), 10)
                if line in (b"\r\n", b"\n", b""):
                    break
                k, _, v = line.decode("latin-1").partition(":")
                headers[k.strip().lower()] = v.strip()
            body = b""
            n = int(headers.get("content-length", "0") or 0)
            if 0 < n < 10000:
                body = await reader.readexactly(n)
            if method == "POST" and path == "/mtxauth":
                # the video relay asks: only from the Pi itself
                ok = ip in ("127.0.0.1", "::1") and hasattr(self.gw, "cams") and self.gw.cams.auth(json.loads(body or b"{}"))
                await self._send(writer, 200 if ok else 401, b"", "text/plain")
                return
            user, role = self._auth(headers, ip)
            if role is None:
                await self._send(writer, 401, b"login needed", "text/plain",
                                 {"WWW-Authenticate": 'Basic realm="Trygons C2"'})
            elif method == "GET" and path in ("/", "/index.html"):
                await self._send(writer, 200, PAGE.encode(), "text/html; charset=utf-8")
            elif method == "GET" and path == "/api/status":
                ws = self.web_stations.get(user)
                if ws is not None:
                    ws.last_pong = time.time()          # the page is still open
                await self._send(writer, 200, json.dumps(self.status(role)).encode(), "application/json")
            elif method == "POST" and path == "/api/cmd":
                code, ans = await self.command(user, role, ip, body)
                await self._send(writer, code, json.dumps(ans).encode(), "application/json")
            else:
                await self._send(writer, 404, b"not found", "text/plain")
        except (asyncio.TimeoutError, ConnectionError, ValueError, asyncio.IncompleteReadError):
            pass
        finally:
            try:
                writer.close()
            except Exception:
                pass

    @staticmethod
    async def _send(writer, code, body, ctype, extra=None):
        reason = {200: "OK", 401: "Unauthorized", 403: "Forbidden", 404: "Not Found"}.get(code, "OK")
        head = "HTTP/1.1 %d %s\r\nContent-Type: %s\r\nContent-Length: %d\r\nCache-Control: no-store\r\n" \
               "Connection: close\r\n" % (code, reason, ctype, len(body))
        for k, v in (extra or {}).items():
            head += "%s: %s\r\n" % (k, v)
        writer.write(head.encode() + b"\r\n" + body)
        await writer.drain()

    def status(self, role):
        d = self.gw.telem or {}
        s = self.gw.stations
        h = self.gw.telem2.get("helm")
        alarms = proto.alarm_texts(d.get("alarm_bits", 0)) if d else ["NO DATA FROM THE BOAT"]
        mode = d.get("mode_name", "---") if d else "---"
        banner = ""
        if d and d.get("mode") == proto.MODE_MANUAL:
            banner = "JOYSTICK IN CONTROL"
        chan = self.gw.channel_text() if hasattr(self.gw, "channel_text") else "-"
        return {
            "mode": mode, "alarms": alarms, "banner": banner,
            "heading": "%05.1f°" % d["heading"] if d else "-", "target": "%05.1f°" % d["target"] if d else "-",
            "sog": "%.1f kn  %03.0f°" % (d["est_sog"], d["est_cog"]) if d else "-",
            "throttle": "%.0f %%  %s" % (d["throttle"], proto.GEARS[d["gear"]] if d["gear"] < 3 else "?") if d else "-",
            "pos": "%.5f %.5f" % (d["est_lat"], d["est_lon"]) if d else "-",
            "plim": "%.0f %% / R %.0f %%" % (d["power_fwd"], d["power_rev"]) if d else "-",
            "helm": s.helm_name(h), "channel": chan,
            "web_helm": self.helm_allowed and role == "HELM",
        }

    async def command(self, user, role, ip, body):
        if not self.helm_allowed:
            return 403, {"ok": False, "text": "HELM from the web page is switched off (web_helm)"}
        if role != "HELM":
            return 403, {"ok": False, "text": "VIEWER: watch only"}
        try:
            m = json.loads(body or b"{}")
        except ValueError:
            return 200, {"ok": False, "text": "bad request"}
        c = m.get("cmd")
        if c == "helm_take":
            ftype, p = proto.T_HELM, proto.p_helm(0)
        elif c == "helm_release":
            ftype, p = proto.T_HELM, proto.p_helm(1)
        elif c == "standby":
            ftype, p = proto.T_SET_MODE, proto.p_set_mode(proto.MODE_STANDBY)
        elif c == "hdg" and m.get("d") in (-10, -1, 1, 10):
            ftype, p = proto.T_HEADING_ADJ, proto.p_heading(m["d"])
        else:
            return 200, {"ok": False, "text": "unknown command"}
        st = self._station(user, role, ip)
        if st is None:
            return 200, {"ok": False, "text": "no free station slot"}
        st.last_pong = time.time()
        seq = (st.last_seq or 0) + 1
        self.gw.stations.command(st, {"sid": st.session, "seq": seq, "ts": time.time(), "type": ftype,
                                      "p": p.hex()})
        for _ in range(40):                     # the AURIX answers within ~100 ms
            a = st.writer.acks.pop(seq, None)
            if a:
                return 200, {"ok": a["result"] == 0, "text": a["text"]}
            await asyncio.sleep(0.05)
        return 200, {"ok": False, "text": "no answer from the boat"}

    def _station(self, user, role, ip):
        """The web user acts as a station (its own id, HELM, audit)."""
        ss = self.gw.stations
        st = self.web_stations.get(user)
        if st is not None and ss.stations.get(st.id) is st:
            return st
        sid = ss._new_id()
        if sid is None:
            return None
        st = Station(sid, user, role, ss.is_local(ip), ip, "web (%s)" % user, _WebWriter())
        st.tokens = ss.burst
        st.tmode_pref = "lite"
        ss.stations[sid] = st
        self.web_stations[user] = st
        self.gw.aurix.send(proto.T_GW_STATION, bytes([sid, 0, 1 if st.local else 0]))
        self.gw.audit("connect", station=sid, user=user, name=st.name, role=role, local=st.local, ip=ip, via="web")
        return st
