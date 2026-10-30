# TeamCode DOX

## Purpose

Contains team-written robot control software for competition, including the main teleoperation OpMode (`RobotTeleOp`), the autonomous path, subsystem classes, and sensor vision pipelines.

## Ownership

Owned by the FTC student programming team. All robot behavior modifications occur within this module.

## Local Contracts

- `BasicAutonomous.java` runs an out-and-back route with 7-inch initial/return short legs, 46-inch long legs and 180-degree turnaround/final turns. After the long return leg, the robot restores its initial heading and backs up 7 inches to the nominal starting position. Returning to the starting pose depends on hardware calibration of the turnaround. `BasicAutonomous` negates forward and turn inputs at the `Robot.drive` boundary because positive drive power moves this chassis backward; its route uses positive distances for physical forward and left/right helpers for physical turns. Driving and turning use time estimates; calibrate distance speed and turn duration on the robot before relying on endpoint accuracy.
- `BasicAutonomous` first aligns the chassis to the selected scoring cluster after START (X = Blue, B = Red during INIT; default Red), then attempts four pollen before route driving: full shooter power, 2000 ms spin-up, then four 400 ms push/600 ms rest feeder cycles. Missing feeder, alignment failure, lost aim before/during feeding, camera failure, or STOP aborts the route. Shooter/intake stop and feeder returns to rest in cleanup. This is timed power/feeding, not regulated RPM or ball detection; tune on hardware.
- `RobotTeleOp` uses `findCluster` and `AprilTagClusterDetection` with current SDK APIs; odometry telemetry requires real accessors in `Robot`.
- `AutonomousCameraAligner` only turns in place; range is telemetry, not a shooting-distance controller. Aim defaults to 0-degree bearing; calibrate camera-to-shooter offset on hardware. Raw positive turn is physical left on the current drivetrain.
- Alignment requires five distinct fresh frames over 200 ms, within 2 degrees and at most 8 deg/sec bearing change. Frames older than 250 ms, target loss, or non-streaming cameras stop turning; camera failure and a 5-second timeout abort. `Camera Alignment Test` selects alliance and tests this without shooting.
- `AprilTagVision` uses the existing 640x480 YUY2 Arducam setup with exposure/gain controls and startup Logitech fallback (`Webcam 2`). Runtime loss reports failure instead of reopening. Imported lens calibration needs physical bearing verification; its principal point is outside the image.
- Keep OpModes organized under `org.firstinspires.ftc.teamcode`.
- Annotate competition TeleOp classes with `@TeleOp(name = "...", group = "...")`.
- Annotate competition Autonomous classes with `@Autonomous(name = "...", group = "...")`.
- Ensure actuators and sensors match hardware configuration names on the Control Hub.
- `ColorSensorTest.java` provides the `Ball Identifier` TeleOp for a REV Color Sensor V3 named `colorSensor`; it displays raw RGB and yellow POLLEN, red/blue NECTAR, or NO BALL / UNKNOWN labels. Tune color ratios with actual balls and lighting before relying on classification; this test has no distance-based presence check.
- `AprilTagVision` uses `AprilTagGameDatabase.getCurrentGameTagLibrary()` (Sample tags plus BIOBUZZ clusters 30-45). Do not register IDs 30-45 as single tags; cluster poses are relative targeting data since the tags move with the CELLs, not absolute field localization.
- `Robot TeleOp` selects alliance in INIT (X = Blue, B = Red, default Red); hold LB or RB to align the chassis to the alliance audience or far CELL cluster using `ftcPose.bearing`.
- `org.firstinspires.ftc.teamcode.ds` hosts the driver station web UI backend (web handlers, `ds` websocket namespace, diagnostics poller); opmodes publish through the `DriverStation` facade.
- `src/main/assets/ds/` holds the web UI assets served at `/ds`; `app.css` is generated output — edit `ds-ui/input.css` and rebuild with `cd TeamCode/ds-ui && ./build.sh` instead of editing it directly.

## Work Guidance

- Follow Java conventions used in existing OpModes and subsystems.
- Keep motor and servo initializations in initialization blocks before `waitForStart()`.
- Test hardware code safely by keeping emergency stops accessible via gamepad.

## Verification

- Verify `Ball Identifier` with yellow, red, and blue balls, a neutral object, and an empty sensing area; check both labels and raw readings at the intended mounting distance.
- Build and verify via Gradle: `./gradlew :TeamCode:assembleDebug`.
- Deploy to the robot controller via wireless ADB (`adb connect 192.168.43.1:5555`).

## Child DOX Index

- `src/test/`: deterministic JVM alignment tests and their verification contract.
- Production sources, resources, and hardware verification remain owned here.
