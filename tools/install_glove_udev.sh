#!/bin/bash
# Install the udev rule for the MANUS glove/dongle (VID 3325).
#
# Background: the MANUS dongle attaches over USB and CoreLite (SDK Integrated)
# opens it directly. Default USB device permissions do not let a normal user
# open it, so the connection fails.
#
# The rule does nothing more than grant access for one vendor ID. Written here
# rather than pulled from an external repository (see install_license_udev.sh
# for the license key).
#
# Usage: sudo bash install_glove_udev.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
    echo "root is required: sudo bash $0" >&2
    exit 1
fi

RULES_DST=/etc/udev/rules.d/70-manus-glove.rules
cat > "$RULES_DST" <<'RULES'
# MANUS gloves / sensor dongle (VID 3325)
SUBSYSTEM=="usb", ATTRS{idVendor}=="3325", MODE:="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="3325", MODE:="0666"
KERNEL=="hidraw*", ATTRS{idVendor}=="3325", MODE:="0666"
RULES
echo "installed: $RULES_DST"

udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --action=change
udevadm trigger --subsystem-match=hidraw --action=change
udevadm settle

echo "=== check (permissions on VID 3325 devices) ==="
found=0
for dev in $(lsusb -d 3325: 2>/dev/null | sed -E 's/Bus ([0-9]+) Device ([0-9]+).*/\1\/\2/'); do
    ls -l "/dev/bus/usb/$dev" && found=1
done
[[ $found -eq 1 ]] || echo "  (no VID 3325 device visible -- check that the dongle is plugged in)"
