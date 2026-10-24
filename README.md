# FTC 36424 UnderEngineered Toasters

Team code for the BIOBUZZ (2026-2027) season. This repository is a fork of the
[FIRST Tech Challenge SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController)
(FtcRobotController v12.0).

## Layout

- `TeamCode/` - our OpModes, subsystems, autonomous routines, and teleop programs.
- `FtcRobotController/` - the FTC SDK app module and its sample OpModes.

## Requirements

Android Studio Narwhal 3 Feature Drop or later. Clone the repo and open it in
Android Studio, or use "Import project" on the folder.

## Workflow

- Never commit directly to `main`. Work on a feature branch and open a draft
  pull request. Merge into `main` only after the change works on the robot.
- Commit messages: imperative mood, concise summary, capitalized first letter,
  no trailing period (for example: `Add DcMotor interface for motor functionality`).

## Connecting to the robot

Robot Wi-Fi: SSID `36424-RC`, password `87654321`.

```bash
adb connect 192.168.43.141:5555   # Driver Hub
adb connect 192.168.43.1:5555     # Control Hub
```

Runtime errors and telemetry appear on the Driver Hub screen. See `AGENTS.md`
for the dual-network setup, screen mirroring, and screenshot commands.

## Resources

- [FTC documentation](https://ftc-docs.firstinspires.org/index.html)
- [FTC SDK Javadoc](https://javadoc.io/doc/org.firstinspires.ftc)
- [FTC community forum](https://ftc-community.firstinspires.org/)
- [SDK release notes](https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases) (kept upstream)
