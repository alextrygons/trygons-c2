#!/bin/sh
# Camera internet ON / OFF (for a firmware update only).
# Normally the cameras have NO internet: the camera network (cam0 = its own Ethernet adapter on the gateway PC, Ubuntu;
# the names come from README section 3: systemd .link files)
# is not forwarded anywhere (systemd/nftables.conf: the "camnet" chain is
# empty). "on" puts an accept into it and a NAT for cam0 -> boatlan0; "off"
# removes both. The gateway calls this and switches it off by itself after
# the time you chose (default 30 min).
#   sudo visudo -f /etc/sudoers.d/trygons-cam   and put in:
#   trygons ALL=(root) NOPASSWD: /home/trygons/trygons-c2/gateway/tools/camera_internet.sh
set -e
UP_IF=${UP_IF:-boatlan0}
case "$1" in
  on)
    nft flush chain inet filter camnet
    nft add rule inet filter camnet accept
    nft add table ip trygons_camnat
    nft flush table ip trygons_camnat
    nft add chain ip trygons_camnat post '{ type nat hook postrouting priority 100; }'
    nft add rule ip trygons_camnat post iifname "cam0" oifname "$UP_IF" masquerade
    sysctl -q -w net.ipv4.ip_forward=1
    echo "camera internet ON" ;;
  off)
    nft flush chain inet filter camnet
    nft delete table ip trygons_camnat 2>/dev/null || true
    sysctl -q -w net.ipv4.ip_forward=0
    echo "camera internet OFF" ;;
  *) echo "usage: $0 on|off"; exit 1 ;;
esac
