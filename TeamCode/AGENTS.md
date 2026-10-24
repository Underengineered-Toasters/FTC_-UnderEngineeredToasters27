# TeamCode DOX

## Purpose

Contains team-written robot control software for competition, including the main teleoperation OpMode (`RobotTeleOp`), the autonomous path, subsystem classes, and sensor vision pipelines.

## Ownership

Owned by the FTC student programming team. All robot behavior modifications occur within this module.

## Local Contracts

- `BasicAutonomous.java` currently runs a single timed right-turn calibration; the full route is commented out. Its configured `imu` uses the confirmed Control Hub orientation: logo UP, USB FORWARD. Yaw resets at START; accumulated angle handles +/-180-degree wrap, reports positive left/negative right, and keeps updating with motors stopped until STOP to show overshoot. Turning remains time-controlled. It negates forward and turn inputs at the `Robot.drive` boundary because positive drive power moves this chassis backward. Recalibrate timing on hardware before restoring the full route.
- `RobotTeleOp` uses `AprilTagVision.findCluster` and `AprilTagClusterDetection` for alignment metadata. Odometry telemetry requires actual encoder accessors in `Robot`; do not add placeholder readings.
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
