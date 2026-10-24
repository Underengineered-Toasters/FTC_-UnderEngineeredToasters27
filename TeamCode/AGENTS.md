# TeamCode DOX

## Purpose

Contains team-written robot control software for competition, including the main teleoperation OpMode (`RobotTeleOp`), the autonomous path, subsystem classes, and sensor vision pipelines.

## Ownership

Owned by the FTC student programming team. All robot behavior modifications occur within this module.

## Local Contracts

- `BasicAutonomous.java` runs an out-and-back route with 7-inch initial/return short legs, 46-inch long legs and 180-degree turnaround/final turns. After the long return leg, the robot restores its initial heading and backs up 7 inches to the nominal starting position. Returning to the starting pose depends on hardware calibration of the turnaround. `BasicAutonomous` negates forward and turn inputs at the `Robot.drive` boundary because positive drive power moves this chassis backward; its route uses positive distances for physical forward and left/right helpers for physical turns. Driving and turning use time estimates; calibrate distance speed and turn duration on the robot before relying on endpoint accuracy.
- `BasicAutonomous` attempts four pollen immediately after movement four (the first 180-degree left turn). The chassis stops, shooter runs at full power with 2000 ms spin-up, then four 400 ms feeder pushes/600 ms returns. These are timed attempts, not measured RPM or counted balls. Missing feeder configuration skips shooting with telemetry. STOP or shooting exceptions stop shooter/intake and return feeder to rest.
- `RobotTeleOp` uses `AprilTagVision.findCluster` and `AprilTagClusterDetection` for cluster metadata. Odometry readouts require real encoder accessors in `Robot`; do not add placeholder readings.
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

No nested child indexes. All team code resides in `src/main/java/org/firstinspires/ftc/teamcode/`.
