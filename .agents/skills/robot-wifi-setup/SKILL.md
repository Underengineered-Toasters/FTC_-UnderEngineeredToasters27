---
name: robot-wifi-setup
description: "Configure dual-station Wi-Fi and wireless ADB connections to the REV Control Hub and Driver Hub without losing internet."
---

# Robot Wi-Fi and ADB Setup

Connect to the robot Wi-Fi network while keeping internet access on the primary interface.

## Quick setup

Run these commands to create an isolated `wlan1` interface for the robot:

```bash
# Prevent NetworkManager from managing wlan1
cat << 'EOF' | pkexec tee /etc/NetworkManager/conf.d/99-unmanage-wlan1.conf
[keyfile]
unmanaged-devices=interface-name:wlan1
EOF
pkexec systemctl reload NetworkManager

# Create virtual interface on robot MAC
pkexec iw dev wlan0 interface add wlan1 type managed addr 5a:cd:c9:3c:93:b0
pkexec ip link set wlan1 up
pkexec ip addr add 192.168.43.199/24 dev wlan1

# Start isolated wpa_supplicant for robot network
cat << 'EOF' > /tmp/wpa_robot.conf
ctrl_interface=/var/run/wpa_supplicant_wlan1
update_config=0

network={
    ssid="36424-RC"
    psk="87654321"
    key_mgmt=WPA-PSK
    proto=RSN WPA
}
EOF
pkexec mkdir -p /var/run/wpa_supplicant_wlan1
pkexec wpa_supplicant -B -i wlan1 -c /tmp/wpa_robot.conf
```

## Verify connection

Confirm the link is active and the hubs respond:

```bash
# Check link status
iw dev wlan1 link

# Ping Control Hub
ping -c 2 192.168.43.1

# Connect ADB
adb connect 192.168.43.1:5555
adb connect 192.168.43.141:5555
```

## Troubleshooting

### Control Hub offline or dropping

If the Control Hub disconnects or appears offline:

1. Connect the Control Hub to the laptop with a USB-C cable.
2. Confirm the device appears under `adb devices`.
3. Enable TCP mode:
   ```bash
   adb -s <serial> tcpip 5555
   ```
4. Unplug the cable and reconnect wirelessly.

### Driver Hub requires TCP mode after reboot

Android 10 Driver Hubs disable TCP mode when rebooted:

1. Connect the Driver Hub to the laptop via USB-C.
2. Run `adb tcpip 5555`.
3. Disconnect USB and run `adb connect 192.168.43.141:5555`.
