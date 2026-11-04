---
name: robot-wifi-setup
description: "Configure dual-station Wi-Fi and wireless ADB connections to the REV Control Hub and Driver Hub without losing internet."
---

# Robot Wi-Fi and ADB Setup

Connect to the robot Wi-Fi network while keeping internet access on the primary interface. The robot network is the Control Hub's own access point, SSID `FTC-PnjW`, password `password` (the SDK defaults, read from the hub's log on 2026-11-03). Installing an APK can make the app reapply these defaults, so recheck the name after a deploy.

## Quick setup

Run these commands to create an isolated `wlan1` interface for the robot:

```bash
# Prevent NetworkManager from managing wlan1 (skip on a laptop that uses iwd, where NetworkManager is not running)
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
    ssid="FTC-PnjW"
    psk="password"
    key_mgmt=WPA-PSK
}
EOF
pkexec mkdir -p /var/run/wpa_supplicant_wlan1
pkexec wpa_supplicant -B -i wlan1 -c /tmp/wpa_robot.conf
```

Put the privileged steps in one script and run it as `pkexec bash script.sh` for a single auth prompt. Do not scan from `wlan0` (`iwctl station wlan0 scan`, `iw dev wlan0 scan`), because that can drop the main connection. To see what `wlan1` can reach, run `pkexec wpa_cli -i wlan1 -p /var/run/wpa_supplicant_wlan1 scan_results`. When you are done, stop the supplicant with `pkexec pkill -f "wpa_supplicant -B -i wlan1"`, because it keeps scanning.

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

On 2026-11-03 one USB cable made the hub drop off USB and re-enumerate every 20 seconds, with ADB stuck offline. It was stable after a different cable, so try another cable first. With the cable in, `adb forward tcp:8080 tcp:8080` and `adb forward tcp:8081 tcp:8081` open the dashboard at `http://localhost:8080/ds` without any Wi-Fi.

### Install fails with INSTALL_FAILED_VERSION_DOWNGRADE

The app on the hub has a higher `versionCode` than the APK. Keep `FtcRobotController/src/main/AndroidManifest.xml` at `versionCode` 63 and `versionName` 12.0, which matches the SDK 12.0.0 libraries. Do not force a downgrade with `-d`, because the Driver Hub app may then report a version mismatch.

### Driver Hub requires TCP mode after reboot

Android 10 Driver Hubs disable TCP mode when rebooted:

1. Connect the Driver Hub to the laptop via USB-C.
2. Run `adb tcpip 5555`.
3. Disconnect USB and run `adb connect 192.168.43.141:5555`.
