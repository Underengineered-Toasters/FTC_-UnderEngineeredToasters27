# DOX Framework

DOX is the AGENTS.md hierarchy for this repository. Every agent must follow DOX instructions across all edits.

## Core Contract

AGENTS.md files are binding work contracts for their subtrees. Work products, source materials, instructions, records, assets, and durable documentation must remain understandable from the nearest applicable AGENTS.md plus every parent AGENTS.md above it.

## Read Before Editing

1. Read the root AGENTS.md.
2. For every file or folder you expect to touch, walk from the repository root to the target path and read every AGENTS.md on the route, including any child AGENTS.md whose scope contains the path.
3. The nearest AGENTS.md is the local contract; parents hold repository-wide rules. If docs conflict, the closer document controls local work details. No child document may weaken DOX.

Do not rely on memory. Re-read the applicable DOX chain in the current session before editing.

## Update After Editing

Every meaningful change requires a DOX pass before the task is done:

1. Re-check changed paths against the DOX chain.
2. Update the closest owning AGENTS.md when the change affects purpose, scope, ownership, durable structure, contracts, workflows, required inputs/outputs, constraints, side effects, user preferences, or any AGENTS.md creation, deletion, move, rename, or index contents.
3. Update parent docs when parent-level structure, ownership, workflow, or child index changes; update child docs when parent changes alter local rules.
4. Refresh every affected Child DOX Index and remove stale or contradictory text.
5. Run existing verification when relevant.
6. Report any docs intentionally left unchanged and why.

Small edits that do not change behavior or contracts may leave docs unchanged, but the DOX pass must still happen.

## Hierarchy

- Root AGENTS.md is the DOX rail: project-wide instructions, global preferences, durable workflow rules, and the top-level Child DOX Index.
- Child AGENTS.md files own domain-specific instructions and their own Child DOX Index.
- Each parent explains what its direct children cover and what stays owned by the parent.
- The closer a document is to the work, the more specific and practical it must be.
- Create a child AGENTS.md when a folder becomes a durable boundary with its own purpose, rules, responsibilities, workflow, materials, or quality standards.
- New child docs default to this section order: Purpose, Ownership, Local Contracts, Work Guidance, Verification, Child DOX Index.

## Style

- Keep docs concise, current, and operational.
- Document stable contracts, not diary entries.
- Put broad rules in parent docs and concrete details in child docs.
- Prefer direct bullets with explicit names.
- Do not duplicate rules across files unless each scope needs a local version.
- Delete stale notes instead of explaining history.
- Trim obvious statements, repeated rules, and misplaced detail.

## User Preferences

- Git commit style: match previous repository commits. Imperative mood, concise summary, capitalized first letter, no trailing period (for example: `Add DcMotor interface for motor functionality`).
- Branch and draft PR workflow: never commit directly to `main`. Do all work on a feature branch and open a draft pull request for it. Merge into `main` only after the change works on the feature branch. Follow any further branch rules in `README.md`.
- Issue tracking: GitHub Issues is the tracker (`gh issue`). Before starting work, check for an existing issue; if none, create one. Open a draft PR early while working and link it to the issue.
- Pre-work git check: before starting work, run `git status` and `git fetch` to confirm the current branch, uncommitted changes, and whether the local branch is behind remote commits. If the branch is behind, alert the user once and ensure they acknowledge before proceeding.
- Error diagnostics: runtime errors and telemetry alerts appear on the REV Driver Hub screen, not the Control Hub. Connect via ADB to the Driver Hub (`192.168.43.141:5555`) to screenshot and inspect errors during debugging.

## Builds and CI

- SDK 12 camera code uses Gradle 9.6 and Android Gradle Plugin 9.4 with JDK 25. Retain Road Runner dependencies when updating SDK tooling.

- CI runs on every pull request and on every branch push (`.github/workflows/android-ci.yml`): Node 22 client tests (`node --test TeamCode/ds-ui/tests/app.test.js`), JDK 25 (Temurin), Android SDK platform 30, `./gradlew assembleDebug`, then `./gradlew test`, and the debug APKs are uploaded as the `debug-apks` artifact. Read the job log before claiming a change builds.
- CD runs on `v*` tags (`.github/workflows/android-release.yml`): `./gradlew assembleRelease` and the APKs are published to a GitHub Release named after the tag. It can also be started manually, which uploads the APKs as an artifact without creating a release.
- Release builds are signed with `libs/ftc.debug.keystore` unless `APK_SIGNING_STORE_FILE` and its companion variables are set, so no signing secrets are required.
- `open-code-review.yml` is separate: it only runs when `0xhis` comments `/code-review` on a pull request, on the self-hosted `ftc-review` runner.

## Wireless ADB and Hub Setup

- Robot Wi-Fi is the Control Hub's own access point: SSID `FTC-PnjW`, password `password`. These are the SDK defaults, read from the hub's log on 2026-11-03. After an APK install that day the app applied them and the network that had been `36424-DS` became `FTC-PnjW`. The earlier documented `36424-RC` and `87654321` did not match the hub. Update this line if the hub is renamed.
- Control Hub: `192.168.43.1:5555`
- Driver Hub: `192.168.43.141:5555` (last known address, not rechecked after the Wi-Fi name changed; it may need to rejoin the network)
- Dashboard: `http://192.168.43.1:8080/ds`
- `FtcRobotController/src/main/AndroidManifest.xml` must keep `versionCode` at or above the app on the hub (63, `versionName` 12.0, matching the SDK 12.0.0 libraries). A lower code fails with `INSTALL_FAILED_VERSION_DOWNGRADE`.

### Direct connection

When the laptop is on the robot Wi-Fi network:

```bash
adb connect 192.168.43.141:5555   # Driver Hub
adb connect 192.168.43.1:5555     # Control Hub
```

### Dual station setup (keep internet while connected to robot)

To keep internet on `wlan0` (for example a hotspot) while reaching the robot, use a virtual interface `wlan1` so the primary interface keeps default routing:

```bash
# 0. Only when NetworkManager is running (the dev laptop uses iwd, so skip this step there)
cat << 'EOF' | pkexec tee /etc/NetworkManager/conf.d/99-unmanage-wlan1.conf
[keyfile]
unmanaged-devices=interface-name:wlan1
EOF
pkexec systemctl reload NetworkManager

# 1. Create virtual interface
pkexec iw dev wlan0 interface add wlan1 type managed addr 5a:cd:c9:3c:93:b0
pkexec ip link set wlan1 up
pkexec ip addr add 192.168.43.199/24 dev wlan1

# 2. Connect wlan1 to robot Wi-Fi with isolated control socket
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

# 3. Connect via ADB
adb connect 192.168.43.141:5555
adb connect 192.168.43.1:5555
```

Put all the privileged steps in one script and run it as `pkexec bash script.sh` so there is a single auth prompt. Do not scan from `wlan0` (`iwctl station wlan0 scan` or `iw dev wlan0 scan`), because that can drop the main connection. To see what `wlan1` can reach, use `pkexec wpa_cli -i wlan1 -p /var/run/wpa_supplicant_wlan1 scan_results`. Stop the `wlan1` supplicant (`pkexec pkill -f "wpa_supplicant -B -i wlan1"`) when you are done, because it keeps scanning.

### USB fallback (Control Hub v1.0)

Plug a USB-C data cable into the Control Hub and run `adb devices`; it shows as `Control_Hub_v1_0`. On 2026-11-03 one cable made the hub drop off USB and re-enumerate every 20 seconds, with ADB stuck offline. It was stable after a different cable, so try another cable before suspecting the hub. Run `adb tcpip 5555` to enable wireless ADB. To open the dashboard without Wi-Fi, run `adb forward tcp:8080 tcp:8080` and `adb forward tcp:8081 tcp:8081`, then browse to `http://localhost:8080/ds`.

### Screen mirroring and screenshots (Driver Hub)

```bash
# Mirror and control the Driver Hub screen
scrcpy -s 192.168.43.141:5555

# Capture a screenshot for error inspection
adb -s 192.168.43.141:5555 exec-out screencap -p > /tmp/driver_hub_error.png
```

### Re-enabling TCP mode after reboot

Android 10 Driver Hubs require TCP/IP mode reactivation after every device restart:

1. Connect the Driver Hub to the laptop with a USB-C cable.
2. Enable USB debugging in Developer Options.
3. Run `adb tcpip 5555`.
4. Unplug the cable. Wireless ADB on port 5555 stays active until the next reboot.

The Control Hub (Android 7.1.2) was switched to TCP mode the same way on 2026-11-03. Whether that survives a reboot has not been checked.

## Child DOX Index

- `TeamCode/`: custom team OpModes, subsystems, autonomous routines, and teleoperation programs.
- `FtcRobotController/`: the FTC SDK application module and sample OpModes.
- `PRODUCT.md` (repo root): product context record for the driver station web UI (impeccable skill artifact).
- `.agents/skills/robot-wifi-setup/`: skill for dual-station Wi-Fi and wireless ADB connections.
