# TeamCode DOX

## Purpose

Contains team-written robot control software for competition, including the main teleoperation OpMode (`RobotTeleOp`), the autonomous path, subsystem classes, and sensor vision pipelines.

## Ownership

Owned by the FTC student programming team. All robot behavior modifications occur within this module.

## Local Contracts

- Keep OpModes organized under `org.firstinspires.ftc.teamcode`.
- Annotate competition TeleOp classes with `@TeleOp(name = "...", group = "...")`.
- Annotate competition Autonomous classes with `@Autonomous(name = "...", group = "...")`.
- Ensure actuators and sensors match hardware configuration names on the Control Hub.
- `Basic Auto - Flower to Hive` is a solo-practice route: audience-side FLOWER 48 inches from the right corner, rear bumper 12 inches from its bottom retrieval opening, rear-mounted `Webcam 1` facing the FLOWER. It reverses to tap, pulls forward outside the hive frame, turns the rear camera toward `BLUE AUDIENCE`, aligns bearing, then turns 180 degrees clockwise away. This placement is not a perimeter-contact competition start.
- Before running `BasicAutonomous`, set wheel diameter, track width, bumper length, robot width and motor-output/wheel gear ratio from robot measurements; zero values block motion. Validate encoder direction/distance, clockwise turns and rear-camera bearing correction on hardware. Field geometry comes from FIRST BIOBUZZ TU04 section 9 and Event Field Setup Guide V1.0; moving CELL tags provide relative bearing, not field coordinates. Movement timeout, missing target or STOP aborts the remaining route.
- `AprilTagVision` uses `AprilTagGameDatabase.getCurrentGameTagLibrary()` (Sample tags plus BIOBUZZ clusters 30-45). Do not register IDs 30-45 as single tags; cluster poses are relative targeting data since the tags move with the CELLs, not absolute field localization.
- `Robot TeleOp` selects alliance in INIT (X = Blue, B = Red, default Red); hold LB or RB to align the chassis to the alliance audience or far CELL cluster using `ftcPose.bearing`.
- `org.firstinspires.ftc.teamcode.ds` hosts the driver station web UI backend (web handlers, `ds` websocket namespace, diagnostics poller); opmodes publish through the `DriverStation` facade.
- `src/main/assets/ds/` holds the web UI assets served at `/ds`; `app.css` is generated output — edit `ds-ui/input.css` and rebuild with `cd TeamCode/ds-ui && ./build.sh` instead of editing it directly.

## Work Guidance

- Follow Java conventions used in existing OpModes and subsystems.
- Keep motor and servo initializations in initialization blocks before `waitForStart()`.
- Test hardware code safely by keeping emergency stops accessible via gamepad.

## Verification

- Build and verify via Gradle: `./gradlew :TeamCode:assembleDebug`.
- Deploy to the robot controller via wireless ADB (`adb connect 192.168.43.1:5555`).

## Child DOX Index

No nested child indexes. All team code resides in `src/main/java/org/firstinspires/ftc/teamcode/`.
