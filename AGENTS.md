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
- Issue tracking: before starting work, check whether a GitHub issue already covers it. If none exists, create one. Link the pull request to the issue when opening it.
- Symlink maintenance: keep `CLAUDE.md` symlinked to `AGENTS.md` in the project root.
- Out-of-date sync alert: if the local branch is behind remote commits, alert the user once and ensure they acknowledge before proceeding.
- Error diagnostics: runtime errors and telemetry alerts appear on the REV Driver Hub screen, not the Control Hub. Connect via ADB to the Driver Hub (`192.168.43.141:5555`) to screenshot and inspect errors during debugging.

## Wireless ADB and Hub Setup

- Robot Wi-Fi: SSID `36424-RC`, password `87654321`
- Control Hub: `192.168.43.1:5555`
- Driver Hub: `192.168.43.141:5555`

### Direct connection

When the laptop is on the robot Wi-Fi network:

```bash
adb connect 192.168.43.141:5555   # Driver Hub
adb connect 192.168.43.1:5555     # Control Hub
```

### Dual station setup (keep internet while connected to robot)

To keep internet on `wlan0` (for example a hotspot) while reaching the robot, use a virtual interface `wlan1` so the primary interface keeps default routing:

```bash
# 1. Create virtual interface
pkexec iw dev wlan0 interface add wlan1 type managed addr 5a:cd:c9:3c:93:b0
pkexec ip link set wlan1 up

# 2. Connect wlan1 to the robot Wi-Fi
wpa_passphrase "36424-RC" "87654321" > /tmp/wpa_36424.conf
pkexec wpa_supplicant -B -i wlan1 -c /tmp/wpa_36424.conf

# 3. Assign a static robot-subnet IP with no default gateway
pkexec ip addr add 192.168.43.199/24 dev wlan1

# 4. Connect via ADB
adb connect 192.168.43.141:5555
adb connect 192.168.43.1:5555
```

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

## Child DOX Index

- `TeamCode/`: custom team OpModes, subsystems, autonomous routines, and teleoperation programs.
- `FtcRobotController/`: the FTC SDK application module and sample OpModes.
- `PRODUCT.md` (repo root): product context record for the driver station web UI (impeccable skill artifact).
