#!/bin/bash
# Find the Raspberry Pi on the laptop's current Wi-Fi network and test the SSH login.
# Usage:  bash src/my_bot/scripts/find_pi.sh
# The Pi gets a new address on every network, and needs 1 to 2 minutes after power-on before it answers.
IFACE=$(ip -4 route | awk '/default/ {print $5; exit}')
NET=$(ip -4 -o addr show "$IFACE" | awk '{print $4}' | cut -d. -f1-3)
echo "Network: $IFACE, searching $NET.1 to $NET.254 ..."
seq 1 254 | xargs -P 100 -I{} sh -c "ping -c 1 -W 1 $NET.{} >/dev/null 2>&1"
# hardware-address prefixes used by Raspberry Pi boards
FOUND=$(ip neigh | grep -iE "lladdr (d8:3a:dd|b8:27:eb|dc:a6:32|e4:5f:01|2c:cf:67|88:a2:9e)" | awk '{print $1}')
if [ -z "$FOUND" ]; then
    echo "No Raspberry Pi found. Is the robot powered on, and is the Pi on the SAME hotspot? Wait a minute and run again."
    exit 1
fi
for ip in $FOUND; do
    echo "Raspberry Pi at $ip"
    if ssh -o BatchMode=yes -o ConnectTimeout=6 -o StrictHostKeyChecking=accept-new "ubuntu@$ip" 'echo "  SSH login works, host: $(hostname), up: $(uptime -p)"' 2>/dev/null; then
        echo "  Use:  ssh ubuntu@$ip"
    else
        echo "  SSH did not answer yet (still booting?). Try again in a minute."
    fi
done
