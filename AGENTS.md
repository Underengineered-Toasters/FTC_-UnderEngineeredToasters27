# DOX Framework

DOX is the AGENTS.md hierarchy for this repository. Every agent must follow DOX instructions across all edits.

## Core Contract

AGENTS.md files are binding work contracts for their subtrees. Work products, source materials, instructions, records, assets, and durable documentation must remain understandable from the nearest applicable AGENTS.md plus every parent AGENTS.md above it.

## Read Before Editing

1. Read the root AGENTS.md.
2. Identify every file or folder you expect to touch.
3. Walk from the repository root to each target path.
4. Read every AGENTS.md found along each route.
5. If a parent AGENTS.md lists a child AGENTS.md whose scope contains the path, read that child and continue from there.
6. Use the nearest AGENTS.md as the local contract and parent docs for repository-wide rules.
7. If docs conflict, the closer document controls local work details. No child document may weaken DOX.

Do not rely on memory. Re-read the applicable DOX chain in the current session before editing.

## Update After Editing

Every meaningful change requires a DOX pass before the task is done.

Update the closest owning AGENTS.md when a change affects:

- Purpose, scope, ownership, or responsibilities
- Durable structure, contracts, workflows, or operating rules
- Required inputs, outputs, permissions, constraints, side effects, or artifacts
- User preferences about behavior, communication, process, organization, or quality
- AGENTS.md creation, deletion, move, rename, or index contents

Update parent docs when parent-level structure, ownership, workflow, or child index changes. Update child docs when parent changes alter local rules. Remove stale or contradictory text immediately. Small edits that do not change behavior or contracts may leave docs unchanged, but the DOX pass must still happen.

## Hierarchy

- Root AGENTS.md is the DOX rail: project-wide instructions, global preferences, durable workflow rules, and the top-level Child DOX Index.
- Child AGENTS.md files own domain-specific instructions and their own Child DOX Index.
- Each parent explains what its direct children cover and what stays owned by the parent.
- The closer a document is to the work, the more specific and practical it must be.

## Child Doc Shape

Create a child AGENTS.md when a folder becomes a durable boundary with its own purpose, rules, responsibilities, workflow, materials, or quality standards.

Default section order:
- Purpose
- Ownership
- Local Contracts
- Work Guidance
- Verification
- Child DOX Index

## Style

- Keep docs concise, current, and operational.
- Document stable contracts, not diary entries.
- Put broad rules in parent docs and concrete details in child docs.
- Prefer direct bullets with explicit names.
- Do not duplicate rules across files unless each scope needs a local version.
- Delete stale notes instead of explaining history.
- Trim obvious statements, repeated rules, and misplaced detail.

## Closeout

1. Re-check changed paths against the DOX chain.
2. Update nearest owning docs and any affected parents or children.
3. Refresh every affected Child DOX Index.
4. Remove stale or contradictory text.
5. Run existing verification when relevant.
6. Report any docs intentionally left unchanged and why.

## User Preferences

- Git commit style: Always format commit messages to match previous repository commits. Use imperative mood, concise summary, capitalized first letter, and no trailing period (for example: `Add DcMotor interface for motor functionality`).
- Git push and branch rules: Always follow existing branch rules and conventions stated in the current `README.md`.
- Symlink maintenance: Keep `CLAUDE.md` symlinked to `AGENTS.md` in the project root.

## Wireless ADB and Hub Setup

### Network credentials
- Robot Wi-Fi SSID: `36424-RC`
- Robot Wi-Fi Password: `87654321`

### Device network addresses
- Control Hub IP: `192.168.43.1:5555`
- Driver Hub IP: `192.168.43.141:5555`

### Direct connection
When the development laptop is connected to the robot Wi-Fi network (`36424-RC`):

```bash
# Connect to Driver Hub
adb connect 192.168.43.141:5555

# Connect to Control Hub
adb connect 192.168.43.1:5555
```

### Dual station setup (Keep internet while connected to robot)
If the laptop must keep internet via another Wi-Fi network (such as a hotspot on `wlan0`), use a virtual interface (`wlan1`) so the primary interface keeps default routing to the internet:

1. Create virtual interface:
   ```bash
   pkexec iw dev wlan0 interface add wlan1 type managed addr 5a:cd:c9:3c:93:b0
   pkexec ip link set wlan1 up
   ```

2. Connect `wlan1` to the robot Wi-Fi:
   ```bash
   wpa_passphrase "36424-RC" "87654321" > /tmp/wpa_36424.conf
   pkexec wpa_supplicant -B -i wlan1 -c /tmp/wpa_36424.conf
   ```

3. Assign static robot subnet IP without a default gateway:
   ```bash
   pkexec ip addr add 192.168.43.199/24 dev wlan1
   ```

4. Connect via ADB:
   ```bash
   adb connect 192.168.43.141:5555
   adb connect 192.168.43.1:5555
   ```

### Screen mirroring (Driver Hub)
Mirror and control the Driver Hub screen wirelessly:

```bash
scrcpy -s 192.168.43.141:5555
```

### Re-enabling TCP mode after reboot
Android 10 Driver Hubs require TCP/IP mode reactivation after every device restart:

1. Connect the Driver Hub to the laptop with a USB-C cable.
2. Enable USB debugging in Developer Options.
3. Run:
   ```bash
   adb tcpip 5555
   ```
4. Unplug the cable. Wireless ADB on port 5555 will remain active until the next reboot.

## Child DOX Index

- `TeamCode/`: Holds custom team OpModes, subsystems, autonomous routines, and teleoperation programs.
- `FtcRobotController/`: Holds the FTC SDK application module and sample OpModes.
