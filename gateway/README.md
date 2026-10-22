# Trygons C2 gateway (Raspberry Pi 5)

The Pi is the boat's door to the internet. **The AURIX is never on the
network**: it talks only to the Pi, over a UART with the same framed protocol
(CRC) as the USB laptop link. The local USB laptop, the onboard joystick and
the STANDBY button keep working whatever happens to the Pi.

```
 AURIX ──UART 921600──► Raspberry Pi 5 ──LAN──► Teltonika router ──► 4G / RF / Starlink
   │                       │  USB SSD (logs, video)                      │
   └─ USB laptop (local)   └─ boat LAN: cameras, local stations          └─ VPN (Tailscale) ──► shore PC, iPhone
```

What the gateway does (section by section of the project):

| | |
|---|---|
| A | full copy of the AURIX log on the USB SSD, GPS time for the whole boat, SSD watchdog, 1 Hz heartbeat to the AURIX |
| B | stations log in (VIEWER / HELM), commands checked and forwarded, audit log, iPhone web page |
| C | internet channels (4G, 4G SIM 2, RF, Starlink = emergency only) |
| D | remote joystick from the shore PC |
| E/F | cameras: relay, health checks, ring recording, SAVE |
| G | sync of logs and saved video to the shore PC |

Try it on the PC first (no Pi needed):

```
cd sim
autopilot_sim.exe server 5760 1          (the ground station on 5760, the "Pi UART" on 5762)
cd ..\gateway
python gateway.py --sim                  (data goes to gateway\gw_data)
python test_gateway.py                   (all gateway tests)
```

---

## 1. What to buy

See MANUAL.txt "Wiring and what to buy". In short: Raspberry Pi 5 (8 GB),
official active cooler, 12 V → 5.1 V **5 A** USB-C supply, a **USB 3 SSD
512 GB - 1 TB** (with its own USB-C/A cable, not a stick; video ~3-4 GB/h), a 32 GB microSD
card (A2 class) for the system only, a small IP65 box.

## 2. Install the operating system (on your PC)

1. Install **Raspberry Pi Imager** (raspberrypi.com/software).
2. Device "Raspberry Pi 5", OS **Raspberry Pi OS Lite (64-bit)**, storage = the microSD card.
3. Click **Edit settings** (the gear):
   * hostname `trygons-gw`
   * username `trygons` and a **strong password** (write it in your password manager)
   * Wi-Fi: leave empty (the Pi uses the cable)
   * Services: **Enable SSH**, "Allow public-key authentication only", paste your PC's public key
     (on Windows: `type %USERPROFILE%\.ssh\id_ed25519.pub`; if missing, first `ssh-keygen -t ed25519`)
4. Write, put the card in the Pi, connect the Ethernet cable to the Teltonika, power on.
5. From the PC (same network): `ssh trygons@trygons-gw.local`

## 3. Basic settings on the Pi

```
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y python3-serial git ffmpeg nftables unattended-upgrades
sudo raspi-config
```
In raspi-config:
* **Interface Options → Serial Port**: login shell over serial **No**, serial hardware **Yes**
* **Localisation → Timezone**: UTC (logs are in UTC)
* Finish, reboot.

Then make sure the GPIO UART is on: `sudo nano /boot/firmware/config.txt`, add at the end
```
dtparam=uart0=on
```
reboot. `ls -l /dev/ttyAMA0` must exist (GPIO14 = pin 8 TX, GPIO15 = pin 10 RX).

Automatic security updates: `sudo dpkg-reconfigure -plow unattended-upgrades` → Yes.

## 4. The USB SSD

The SD card holds only the OS. Logs and video go to the SSD.

```
lsblk                                    # find the SSD, e.g. /dev/sda
sudo wipefs -a /dev/sda                  # ERASES the SSD
sudo parted /dev/sda --script mklabel gpt mkpart data ext4 0% 100%
sudo mkfs.ext4 -L TRYGONS_SSD /dev/sda1
sudo mkdir -p /mnt/ssd
echo 'LABEL=TRYGONS_SSD /mnt/ssd ext4 defaults,noatime,nofail,x-systemd.device-timeout=10 0 2' | sudo tee -a /etc/fstab
sudo mount -a
sudo mkdir -p /mnt/ssd/trygons && sudo chown trygons: /mnt/ssd/trygons
```
`nofail` = the Pi still boots without the SSD. The gateway then raises the
alarm **GATEWAY STORAGE** and everything else keeps working.

## 5. The gateway software

```
cd ~ && git clone https://github.com/alextrygons/trygons-c2.git
sudo mkdir -p /etc/trygons
sudo cp ~/trygons-c2/gateway/config.example.json /etc/trygons/gateway.json
sudo cp ~/trygons-c2/gateway/secrets.example.json /etc/trygons/secrets.json
sudo chown -R trygons: /etc/trygons && chmod 600 /etc/trygons/secrets.json
nano /etc/trygons/gateway.json           # check the values (see the comments in config.example.json)
nano /etc/trygons/secrets.json           # passwords: router, cameras, users (never in git)
```
Test by hand: `python3 ~/trygons-c2/gateway/gateway.py --config /etc/trygons/gateway.json`
(Ctrl+C to stop). It must say "AURIX connected" and "storage ... ok".

## 6. Start automatically (system service)

```
sudo cp ~/trygons-c2/gateway/systemd/trygons-gateway.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now trygons-gateway
systemctl status trygons-gateway         # "active (running)"
journalctl -u trygons-gateway -f         # live log (Ctrl+C to leave)
```
It restarts by itself if it ever stops, and starts at every boot. The
service may set the clock (GPS time from the AURIX) - nothing else needs root.

## 7. VPN: Tailscale (no open ports)

Why Tailscale: the Pi connects **outward** to the Tailscale network, so it
works behind the 4G operator's CGNAT and behind Starlink, with **no open
ports and no port forwarding**. Traffic is WireGuard-encrypted end to end.
(Plain WireGuard would need a server with a public address on shore. If you
don't want the Tailscale company in the loop: Headscale is the self-hosted
version - same client.)

1. Make a free account at tailscale.com (or use yours: Tailscale is already
   installed on your shore PC).
2. On the Pi:
   ```
   curl -fsSL https://tailscale.com/install.sh | sh
   sudo tailscale up --ssh --hostname=trygons-gw
   ```
   Open the link it prints and log in.
3. In the Tailscale admin console: **Machines → trygons-gw → Disable key expiry**
   (a boat at sea must not lose its VPN after 180 days).
4. Access controls (admin console → Access controls), example:
   ```
   {"acls": [{"action": "accept", "src": ["autogroup:admin"], "dst": ["trygons-gw:5761,8080,8554,8889,8189,22"]}]}
   ```
5. On the shore PC and the iPhone: install Tailscale, log in with the same account.
   The ground station then connects to `trygons-gw:5761`.

## 8. Firewall on the Pi

Nothing may reach the Pi except over the VPN or from the boat LAN.
```
sudo cp ~/trygons-c2/gateway/systemd/nftables.conf /etc/nftables.conf
sudo nano /etc/nftables.conf             # check "boat_lan" (default 10.42.0.0/24)
sudo systemctl enable --now nftables
```
The gateway ports: 5761 stations (TCP, and UDP for the remote joystick),
8080 web page (iPhone), 8554 / 8889 video.

## 9. Updates

```
cd ~/trygons-c2 && git pull
sudo systemctl restart trygons-gateway
```
Only update in harbour, and only after `python3 gateway/test_gateway.py`
passed on the PC with the same version.

## 10. Check list after setup

* `journalctl -u trygons-gateway` shows "AURIX connected", "storage ok", "GPS time".
* Ground station (USB laptop): System tab shows "gateway OK", the Pi's copy
  of the log grows, no GATEWAY alarm.
* Pull the Pi's power: within 3 s the boat shows **GATEWAY LOST**, nothing else changes.

## 11. Users, roles and HELM

Every station logs in. Roles:

| Role | can |
|------|-----|
| VIEWER | watch everything (telemetry, cameras, status). Never commands. |
| HELM | may **hold HELM**. Only one station holds HELM at a time; only its commands are carried out (the AURIX checks it). |

* A station **on the boat** (boat LAN, or the USB laptop) can always take
  HELM, also from a remote one. A remote station only gets HELM when
  nobody has it.
* The laptop on the AURIX's USB cable gets HELM by itself when nobody has it.
* If the remote HELM station's link is lost: alarm **REMOTE LINK LOST** on
  the boat, HELM goes back to nobody, nothing else changes.
* Power raise, AUTO start / resume and remote driving still need the
  **joystick CONFIRM on the boat**. Remote can never do them alone.

Add users (on the Pi):
```
cd ~/trygons-c2/gateway
python3 users.py add alex HELM              # asks for a password (at least 10 characters)
python3 users.py add guest VIEWER
python3 users.py key iphone VIEWER          # a random key instead of a password
```
Each prints a line; paste it into `"users": { ... }` in
`/etc/trygons/secrets.json`, then `sudo systemctl restart trygons-gateway`.
Passwords are stored only as salted hashes. Five wrong logins from one
address lock it out for a minute. Everything is in the audit log
(`/mnt/ssd/trygons/audit/YYYYMMDD.jsonl`): connections, failed logins,
HELM changes, every command with its result.

Every command carries the station's id, a sequence number and a time
stamp. The gateway refuses duplicates, out-of-order, stale (older than
2 s, `max_age_s`) and too many (rate limit) commands **before** they reach
the AURIX. A wrong PC clock doesn't matter: the gateway measures each
station's clock offset with its latency pings.

**iPhone:** `http://trygons-gw:8080` (over Tailscale), log in with a
gateway user. View only. HELM from the web page only if you set
`"web_helm": true` in gateway.json (off by default).

## 12. Security rules - please follow all of them

1. **VPN only.** Nothing on the boat is reachable from the internet: no
   port forwarding on the router, no "remote access" features of the
   router, cameras or Starlink. The Pi firewall (section 8) allows only
   Tailscale and the boat LAN.
2. **Change every default password** before the boat goes to sea:
   * Teltonika router: admin password, and turn off RMS / remote
     management unless you need it.
   * Raspberry Pi: your user's password (SSH is key-only).
   * Starlink app: the account, and the Wi-Fi password if you use its Wi-Fi.
   * Each camera: the TP-Link account and the "Camera Account".
   * Tailscale: two-factor login on the account.
3. **Separate networks:**
   * control network (Pi, AURIX side, cameras) on its own VLAN or switch
     port;
   * guest / crew Wi-Fi on another VLAN, with no access to the control
     VLAN.
4. **Cameras have NO internet access** (router firewall rule, section E).
   Allow it only briefly for a firmware update, then block it again.
5. **Keep software updated:** Pi automatic security updates (section 3),
   router firmware, camera firmware - in harbour, after testing.
6. Give users the smallest role they need (VIEWER for guests and phones).
   Remove users who don't need access any more.
7. Look at the audit log after every trip with remote access.

## 13. Router: Teltonika RUT901 (internet channels)

Your router: **RUT901** (4G LTE Cat 4, **dual SIM with ONE modem**, 4 Ethernet
ports: 1 WAN + 3 LAN, Wi-Fi 2.4 GHz). Channels:

| Channel | RUT901 interface | metered | sync | emergency only |
|---------|------------------|---------|------|----------------|
| 4G (SIM 1) | `mob1s1a1` | no | yes | no |
| 4G SIM 2 | `mob1s2a1` | no | yes | no |
| RF (future) | e.g. `rf` (a LAN port set as WAN, or Wi-Fi client) | no | yes | no |
| STARLINK | `wan` (the WAN port) | **yes** | **never** | **yes** |

**One modem, two SIMs:** only the active SIM works at a time. Switching SIM
takes 30-90 s, and SIM 2's signal can't be measured while SIM 1 is in use
(the ground station shows "SIM 2 standby").

**Please tell me your RutOS version** (WebUI → Status → System → Firmware,
e.g. `RUT9M_R_00.07.10`). The web API needs **07.06 or newer**; the gateway
itself uses SSH + mwan3, which works on all RutOS 7 versions.

Do these steps **in this order**:

1. **Firmware:** WebUI → System → Firmware → update to the newest stable
   version (in harbour). Note the version.
2. **Change the LAN range FIRST** (before Starlink is connected - Starlink
   also uses 192.168.1.x and the two would clash): Network → LAN →
   IP address `10.42.0.1`, netmask `255.255.255.0`. Save, reconnect your PC
   (it gets a new 10.42.0.x address).
3. **Passwords:** System → Administration → change the admin password.
   Turn off remote management (RMS) unless you really use it.
4. **SIM cards:** Network → Mobile → SIM 1 / SIM 2: PIN, APN. Mobile →
   Data limit: set the limits your contract has (the router itself then
   stops at the limit).
5. **Starlink in bypass mode:** in the Starlink app → Settings → Router →
   **Bypass mode** (the Starlink router's Wi-Fi switches off). Cable from the
   Starlink (router's Ethernet port, or the Starlink Ethernet Adapter for
   the Gen 3 "Standard" kit) to the RUT901 **WAN** port.
   If your Starlink has no Ethernet: Network → Wireless → scan → join the
   Starlink Wi-Fi as a client, and use that interface (e.g. `wwan`) as
   "wan" in the channel list.
6. **Failover with health checks:** Network → Failover (RutOS = mwan3):
   * enable `mob1s1a1`, `mob1s2a1`, `wan` (and RF later);
   * priority 4G, 4G SIM 2, RF, then wan (Starlink) **last**;
   * for each: tracking = ping, hosts `1.1.1.1` and `8.8.8.8`, interval 5 s,
     "down" after 3 failures, "up" after 3 replies.
   The gateway then moves the priorities itself (AUTO / MANUAL / STRICT).
7. **DHCP reservations:** Network → LAN → Static leases: the Pi
   (`trygons-gw`) → `10.42.0.2`. (Cameras: section 14.)
8. **SSH for the Pi** (it reads status and sets the priority):
   * System → Administration → Access control: SSH enabled, **LAN only**,
     key authentication.
   * On the Pi: `ssh-keygen -t ed25519` (no passphrase), then
     `ssh-copy-id root@10.42.0.1` (or paste `~/.ssh/id_ed25519.pub` into the
     router's `/etc/dropbear/authorized_keys`).
   * Test: `ssh root@10.42.0.1 mwan3 interfaces`.
9. Check the names of the failover members:
   `ssh root@10.42.0.1 "uci show mwan3 | grep =member"`. If they are not
   `member_mob1s1a1` etc., put the real names as `"mwan3_member"` into
   each channel of `"channels": {"list": [...]}` in gateway.json (copy the
   list from `channels.py` DEFAULT_CHANNELS).
10. Starlink dish status (obstruction, outages) on the ground station:
    `pip install starlink-grpc-tools` on the Pi (optional). The dish answers
    at 192.168.100.1. In bypass mode, add a route on the RUT901 if needed:
    Network → Routing → static route 192.168.100.1/32 via the wan interface.

What the gateway does with the channels: see MANUAL.txt chapter 9 part C.
Ports on the RUT901: WAN = Starlink, LAN1 = the Pi, LAN2 = the TSW101
switch (cameras, see section 14), LAN3 = spare (a laptop on board).
Crew / guest devices: the router's Wi-Fi on its own guest network, never
on the control network (section 12).

## 14. Cameras: 4 x Tapo C310, camera network, video relay

### The camera network (cameras never reach the internet)

Your switch, the **TSW101**, is **unmanaged** (no VLANs). So the cameras get
their own *physical* network:

```
 TSW101 (12 V in, 4 x PoE+)                       Raspberry Pi 5
  port 1 PoE --> FRONT-1     (PoE splitter 9 V)     eth1 = USB 3 Gigabit adapter  10.43.0.1
  port 2 PoE --> REAR-LEFT   (PoE splitter 9 V)       |  DHCP + time server for the cameras,
  port 3 PoE --> REAR-RIGHT  (PoE splitter 9 V)       |  NOTHING is forwarded (no internet)
  port 5     ------------------------------------------+
                                                    eth0 -> RUT901 LAN1 (boat LAN 10.42.0.x)
 FRONT-2 (backup, its own path):
  own 12 V -> 9 V converter + own fuse, cable to RUT901 LAN3 = network "cam2" 10.43.1.x,
  router firewall: cam2 may NOT go to the internet; the Pi may reach it.
```

Why FRONT-2 is separate: if the TSW101, its power or the Pi's USB adapter
fails, FRONT-2 still works (redundancy). The gateway switches to it by itself.

**On the Pi (camera network on eth1):**
```
sudo nmcli con add type ethernet ifname eth1 con-name camnet ipv4.method manual ipv4.addresses 10.43.0.1/24 ipv6.method disabled
sudo apt install -y dnsmasq chrony
sudo tee /etc/dnsmasq.d/camnet.conf <<'EOT'
interface=eth1
bind-interfaces
dhcp-range=10.43.0.100,10.43.0.150,12h
dhcp-option=3                  # no default gateway: the cameras can't route anywhere
dhcp-option=6                  # no DNS
dhcp-option=42,10.43.0.1       # time from the Pi
# fixed addresses (MAC from the camera's label / the Tapo app):
dhcp-host=AA:BB:CC:DD:EE:01,10.43.0.11,front1
dhcp-host=AA:BB:CC:DD:EE:03,10.43.0.13,rearleft
dhcp-host=AA:BB:CC:DD:EE:04,10.43.0.14,rearright
EOT
echo "allow 10.43.0.0/16" | sudo tee /etc/chrony/conf.d/cameras.conf
sudo systemctl restart dnsmasq chrony
```
The Pi's firewall (section 8) never forwards eth1, so the cameras have no
internet. (The Pi's clock comes from GPS, so the cameras get GPS time too.)

**On the RUT901 (FRONT-2):** Network → Interfaces → Add `cam2`, static
`10.43.1.1/24`, bound to the LAN3 port (port-based VLAN: take LAN3 out of the
LAN bridge). Network → LAN → static lease FRONT-2 → `10.43.1.12`.
Network → Firewall → zone `cam2`: input accept, output accept, forward
REJECT, and **no** forwarding to `wan`/mobile. Allow forwarding `lan → cam2`
(the Pi reaches the camera). Add a route on the Pi:
`sudo nmcli con mod "Wired connection 1" +ipv4.routes "10.43.1.0/24 10.42.0.1"`.

### Each Tapo C310 (do this with the camera at home, on your Wi-Fi, once)

1. Tapo app: add the camera, **update the firmware** now (later it has no
   internet).
2. Camera → Settings → **Advanced Settings → Camera Account**: make a user
   name + password (this is the RTSP login, not your TP-Link account). Put it
   into `/etc/trygons/secrets.json`:
   `"cameras": {"FRONT-1": {"user": "...", "password": "..."}, ...}`
3. Settings → Video quality: main stream 1080p (or 3MP), sub stream 360p.
   Frame rate 15 fps is fine. Turn the camera's audio off (not needed).
4. Insert a **microSD card** (high-endurance, 64-128 GB) and turn on
   **continuous recording / loop recording** in the app: an independent
   backup if the Pi fails.
5. Change the TP-Link account password to a strong one; turn off "remote
   access / cloud" features you don't need.
6. Use the camera with the **Ethernet cable** on the boat (not Wi-Fi).
7. Test the stream on a PC: `ffplay rtsp://USER:PASS@IP:554/stream2`.

**Firmware update later:** ground station (on board) → Cameras → "Camera
internet 30 min": the Pi lets the cameras out for 30 minutes, then closes it
by itself. Update in the Tapo app, then press "Camera internet OFF".
(FRONT-2 on the router: allow its zone to `wan` for the update, then remove
it again.)

### The video relay (MediaMTX) on the Pi

```
cd /tmp && wget https://github.com/bluenviron/mediamtx/releases/download/v1.21.2/mediamtx_v1.21.2_linux_arm64.tar.gz
tar xzf mediamtx_v1.21.2_linux_arm64.tar.gz && sudo mv mediamtx /usr/local/bin/
```
The gateway writes its configuration (`/var/lib/trygons/mediamtx.yml`) and
starts / restarts it itself. Why MediaMTX:
* one program, ARM64;
* RTSP in from the cameras, RTSP / WebRTC (iPhone) out;
* **no transcoding** (low CPU, lowest delay);
* streams pulled only when someone watches (an OFF stream costs nothing);
* a local API for health and bitrates.

Every viewer is checked by the gateway: a station only gets the streams its
camera preset allows; on STARLINK / data saver only the front camera, LOW.

### Recording on the SSD (ring buffer, SAVE)

The gateway records the four main streams itself (one `ffmpeg` per camera,
no re-encoding) - install it: `sudo apt install -y ffmpeg`.
* `<ssd>/video/ring/<CAMERA>/YYYYMMDD-HHMMSS.ts`: 1-minute files, UTC
  names (the Pi's clock is GPS time). Older than the ring length (5-60
  min) = deleted.
* `<ssd>/video/events/<id>/`: SAVEs (hard links to the ring files + `event.json`).
* Settings: `recording` in gateway.json (segment_s, ring_min, post_s,
  pre_s, ring_sync, stall_s); `storage.low_free_gb` / `full_free_gb` /
  `quota_gb`.
* Space policy and STORAGE FULL: MANUAL.txt chapter 9 part F.
* Bitrate: in the Tapo app keep the main stream at 1080p / 15 fps (about
  2 Mbit/s): 4 cameras = ~3.6 GB per hour. The Cameras tab shows the real
  "GB/h".

## 15. The shore PC (ground station, remote driving)

1. Install the ground station like on the boat laptop (MANUAL.txt chapter 3)
   and **Tailscale**, logged in to the same account as the Pi.
2. Start it through the gateway: `python app.py gw:trygons-gw:5761` and log
   in with your gateway user.
3. Remote driving (MANUAL.txt chapter 9 part D): plug the USB joystick into
   the shore PC, Remote tab -> Find joysticks -> choose the axes -> Save.
   The settings are in `groundstation/shore_joystick.json`.
4. Somebody must be on board and press the joystick CONFIRM to start it.

## 16. Sync to the shore PC

The Pi connects OUT to the shore PC over Tailscale (nothing listens on the
boat). On the shore PC a small receiver listens on TCP 5770.

1. Make a long random key (on any PC):
   `python -c "import secrets; print(secrets.token_hex(24))"`
2. On the Pi, `/etc/trygons/secrets.json`: add `"sync_key": "<key>"`.
   `/etc/trygons/gateway.json`: `"sync": {"shore": "<shore PC Tailscale name>:5770"}`.
   Restart: `sudo systemctl restart trygons-gateway`.
3. On the shore PC, next to app.py, make `groundstation\sync_receiver.json`
   (it is in .gitignore - never commit it):
   ```
   {"root": "D:\\trygons_sync", "port": 5770, "listen": "0.0.0.0",
    "boats": {"trygons-c2": "<the same key>"}}
   ```
   The ground station starts the receiver by itself when this file exists
   (or run `python syncrecv.py` alone, e.g. on a home server).
4. Windows firewall on the shore PC: allow TCP 5770 **only** from the
   Tailscale network (100.64.0.0/10), e.g. in an admin PowerShell:
   `New-NetFirewallRule -DisplayName "Trygons sync" -Direction Inbound -Protocol TCP -LocalPort 5770 -RemoteAddress 100.64.0.0/10 -Action Allow`
   And in the Tailscale admin ACLs allow only the Pi -> shore PC:5770.
5. Network tab: "SYNC to the shore PC: SYNCING / IDLE"; on the shore PC
   also "THIS PC receives: ...".

Settings (`sync` in gateway.json): `max_kbps` (8000), `target_delay_ms`
(100 - how much extra delay the sync may cause before it slows down),
`settle_s` (60 - a file is sent when it has not changed this long).
Per channel `sync_budget_kbps` (channels list): upload the sync may share with
the cameras. STARLINK has `sync_allowed: false` - keep it that way.

## 17. AIS receiver (receive only)

1. dAISy HAT: `sudo raspi-config` → Interface Options → Serial Port: login
   shell **No**, serial hardware **Yes**; reboot. (The HAT uses
   `/dev/serial0`: the AURIX link then goes over a USB-UART adapter,
   `"aurix": "/dev/ttyUSB0"` - WIRING.md section 10.)
   USB receiver instead: `ls /dev/ttyACM* /dev/ttyUSB*` to find it.
2. `/etc/trygons/gateway.json`: `"ais": {"source": "/dev/serial0", "baud": 38400}`.
3. Test: `cat /dev/serial0` shows `!AIVDM,...` lines when ships are around
   (stop the gateway first: `sudo systemctl stop trygons-gateway`).
4. Ground station: map / AIS list (MANUAL chapter 9 part J).
Online AIS (optional, OFF): an AISHub account (they ask you to share a
receiver feed) → `"ais": {"online": {"enabled": true, "url": "http://data.aishub.net/ws.php?username=YOUR_NAME&format=1&output=json&compress=0&latmin={lat0}&latmax={lat1}&lonmin={lon0}&lonmax={lon1}"}}`.
It is only used on 4G / RF, never on STARLINK.
Second GPS (optional, OFF): `"ais": {"second_gps": true}` on the Pi AND
parameter `gps2_check_m` = 30 on the boat (Parameters tab).
