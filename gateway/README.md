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
256-512 GB** (with its own USB-C/A cable, not a stick), a 32 GB microSD
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
   {"acls": [{"action": "accept", "src": ["autogroup:admin"], "dst": ["trygons-gw:5761,8080,8889,22"]}]}
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
The gateway ports: 5761 stations, 8080 web page (iPhone), 8554 / 8889 video.

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
