#!/bin/bash
# Install the udev rule for the MANUS license key
# (SenseShield/SenseLock family, VID 1c57).
#
# Background: the MANUS Core 3 license lives on a USB hardware key, which
# CoreLite (SDK Integrated) reads over HID (hidraw). Default hidraw permissions
# are 0600 (root only), so running as a normal user produces a
# "No compatible license found" warning and no skeleton stream is generated.
#
# Usage: sudo bash install_license_udev.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "root is required: sudo bash $0" >&2
    exit 1
fi

RULES_DST=/etc/udev/rules.d/71-manus-license.rules
cat > "$RULES_DST" <<'EOF'
# MANUS license key (SenseShield/SenseLock USB HID key)
SUBSYSTEMS=="usb", ATTRS{idVendor}=="1c57", MODE:="0666"
KERNEL=="hidraw*", ATTRS{idVendor}=="1c57", MODE:="0666"
EOF
echo "installed: $RULES_DST"

udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --action=change
udevadm trigger --subsystem-match=hidraw --action=change
udevadm settle

echo "=== check ==="
for h in /dev/hidraw*; do
    vendor=$(udevadm info -a -n "$h" 2>/dev/null | grep -m1 'ATTRS{idVendor}=="1c57"' || true)
    if [[ -n "$vendor" ]]; then
        ls -l "$h"
    fi
done
