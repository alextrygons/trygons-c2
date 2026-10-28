#!/bin/sh
# Trygons gateway PC - Ubuntu 24.04 LTS setup (README section 3-6). Run once, as root,
# from the cloned repository:
#   sudo LAN_MAC=aa:bb:cc:dd:ee:01 CAM_MAC=aa:bb:cc:dd:ee:02 sh gateway/systemd/install_ubuntu.sh
# LAN_MAC = the Ethernet port to the router (boat LAN), CAM_MAC = the one to the PoE switch
# (cameras): "ip -br link" shows them. Without them the interface names are left alone.
# Safe to run again. It never touches the AURIX: the AURIX only sees the USB-UART adapter.
set -e
REPO=$(cd "$(dirname "$0")/../.." && pwd)
GWU=${GWU:-trygons}
[ "$(id -u)" = 0 ] || { echo "run with sudo"; exit 1; }

echo "== packages"
apt-get update
apt-get install -y python3-serial python3-numpy git ffmpeg nftables tlp

echo "== user $GWU (serial ports: group dialout)"
id "$GWU" >/dev/null 2>&1 || adduser --disabled-password --gecos "" "$GWU"
usermod -aG dialout "$GWU"

echo "== folders"
mkdir -p /etc/trygons /var/lib/trygons/data
[ -f /etc/trygons/gateway.json ] || cp "$REPO/gateway/config.example.json" /etc/trygons/gateway.json
[ -f /etc/trygons/secrets.json ] || cp "$REPO/gateway/secrets.example.json" /etc/trygons/secrets.json
chown -R "$GWU": /etc/trygons /var/lib/trygons
chmod 600 /etc/trygons/secrets.json

echo "== lid closed keeps running, no sleep"
mkdir -p /etc/systemd/logind.conf.d
cp "$REPO/gateway/systemd/logind-trygons.conf" /etc/systemd/logind.conf.d/trygons.conf
systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target
systemctl restart systemd-logind || true

echo "== battery = UPS: charge limit 80 % (TLP)"
mkdir -p /etc/tlp.d
cp "$REPO/gateway/systemd/tlp-trygons.conf" /etc/tlp.d/90-trygons.conf
systemctl enable --now tlp
tlp start || true

echo "== updates: security updates yes, automatic restarts NO"
cat > /etc/apt/apt.conf.d/52trygons-no-reboot <<'EOT'
Unattended-Upgrade::Automatic-Reboot "false";
EOT
snap refresh --hold >/dev/null 2>&1 || true     # snaps never refresh by themselves at sea

if [ -n "$LAN_MAC" ] && [ -n "$CAM_MAC" ]; then
  echo "== stable interface names: boatlan0 (router), cam0 (PoE switch)"
  printf '[Match]\nMACAddress=%s\n\n[Link]\nName=boatlan0\n' "$LAN_MAC" > /etc/systemd/network/10-trygons-boatlan.link
  printf '[Match]\nMACAddress=%s\n\n[Link]\nName=cam0\n' "$CAM_MAC" > /etc/systemd/network/11-trygons-cam.link
  echo "   names take effect after a reboot (or re-plugging the USB adapters)"
else
  echo "== interface names: LAN_MAC / CAM_MAC not given - skipped (README section 3)"
fi

echo "== the gateway service"
sed "s#/home/trygons/trygons-c2#$REPO#; s#^User=trygons#User=$GWU#" \
    "$REPO/gateway/systemd/trygons-gateway.service" > /etc/systemd/system/trygons-gateway.service
systemctl daemon-reload
systemctl enable trygons-gateway
echo
echo "Next: python3 $REPO/gateway/gateway.py --list-serial  -> put the names into /etc/trygons/gateway.json"
echo "      python3 $REPO/gateway/gateway.py --config /etc/trygons/gateway.json --check-config"
echo "      sudo systemctl start trygons-gateway; journalctl -u trygons-gateway -f"
echo "Firewall (README section 8) and the camera network (section 14) are separate steps."
