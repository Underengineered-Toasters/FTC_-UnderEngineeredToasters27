# TeamCode DOX

## Purpose

Contains team-written robot control software for competition, including teleoperation OpModes, autonomous paths, subsystem classes, and sensor vision pipelines.

## Ownership

Owned by the FTC student programming team. All robot behavior modifications occur within this module.

## Local Contracts

- Keep OpModes organized under `org.firstinspires.ftc.teamcode`.
- Annotate competition TeleOp classes with `@TeleOp(name = "...", group = "...")`.
- Annotate competition Autonomous classes with `@Autonomous(name = "...", group = "...")`.
- Ensure actuators and sensors match hardware configuration names on the Control Hub.
- Register all AprilTag clusters in the shared `AprilTagVision` library; use each unique ID once and its measured tag size for pose calculations.
- `org.firstinspires.ftc.teamcode.ds` hosts the driver station web UI backend (web handlers, `ds` websocket namespace, diagnostics poller); opmodes publish through the `DriverStation` facade.
- `src/main/assets/ds/` holds the web UI assets served at `/ds`; `app.css` is generated output — edit `ds-ui/input.css` and rebuild with `cd TeamCode/ds-ui && ./build.sh` instead of editing it directly.

- Configure `feeder` as a Continuous Rotation Servo on servo port 1. Feeder commands use power: 0.5 to feed, 0.0 to stop; telemetry reports commanded power. Stop the feeder during initialization, trigger release, and OpMode shutdown.
- `IntakeTeleOp`: gamepad 1 left trigger runs the feeder; A/B operate the intake motor.
- `RobotTeleOp`: gamepad 1 left trigger operates only the feeder; right trigger operates the shooter and feeder. The intake motor stays stopped in this OpMode.
- `RobotActionLog` writes a new `/sdcard/FIRST/debug_logs/robot_actions_<timestamp>_<unique>.csv` per Robot TeleOp run: init/start/trigger changes/stop plus samples every 100 ms. Columns record trigger inputs, feeder requested/commanded power, PWM state, port/connection, shooter target/actual speed, intake and drive power. These values do not measure physical feeder motion. Rows flush as written; file failures appear in CSV Log telemetry and do not disable controls. Preserve older logs.

## Work Guidance

- Follow Java conventions used in existing OpModes and subsystems.
- Keep motor and servo initializations in initialization blocks before `waitForStart()`.
- Test hardware code safely by keeping emergency stops accessible via gamepad.

## Verification

- Build and verify via Gradle: `./gradlew :TeamCode:assembleDebug`.
- Deploy to the robot controller via wireless ADB (`adb connect 192.168.43.1:5555`).

## Child DOX Index

No nested child indexes. All team code resides in `src/main/java/org/firstinspires/ftc/teamcode/`.
