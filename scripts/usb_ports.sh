#!/bin/bash
# List every USB serial device with the USB SOCKET it is plugged into (the "KERNELS" name used in
# udev/99-robot-serial.rules). Run it ON THE PI. Unplug and replug a device to see which line is which.
#   bash scripts/usb_ports.sh
for dev in /dev/ttyUSB* /dev/ttyACM*; do
    [ -e "$dev" ] || continue
    info=$(udevadm info -q property -n "$dev" 2>/dev/null)
    id=$(echo "$info" | grep -E "^ID_VENDOR_ID=" | cut -d= -f2):$(echo "$info" | grep -E "^ID_MODEL_ID=" | cut -d= -f2)
    # the USB device name is the first path component after "usbN/" that looks like 1-1.4
    sock=$(udevadm info -q path -n "$dev" | grep -oE "[0-9]+-[0-9]+(\.[0-9]+)*" | head -1)
    links=$(echo "$info" | grep -E "^DEVLINKS=" | cut -d= -f2 | tr ' ' '\n' | grep -E "/dev/(arduino|ydlidar|servo_nano)" | tr '\n' ' ')
    printf "%-14s usb id %-10s socket (KERNELS) %-8s names: %s\n" "$dev" "$id" "$sock" "${links:-none}"
done
