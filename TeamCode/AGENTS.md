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
- `AprilTagVision` uses `AprilTagGameDatabase.getCurrentGameTagLibrary()` (Sample tags plus BIOBUZZ clusters 30-45). Do not register IDs 30-45 as single tags; cluster poses are relative targeting data since the tags move with the CELLs, not absolute field localization.
- `AprilTagVision` streams the Arducam OV9281 (`Webcam 1`) as MJPEG at 640x480, because the default YUY2 format caps the camera at 30 fps. Once the camera is streaming it locks manual exposure and gain, and `getCameraState()` reports the camera state with the measured fps. The exposure (6 ms) and gain (50) constants are starting values that still need a sweep at shooting range.
- `AprilTagVision` falls back to the Logitech C270 (hardware config name `Webcam 2`) at SDK defaults when `Webcam 1` is not configured, not plugged in, or fails to open. The fallback uses the SDK's built-in Logitech calibration and skips the exposure and gain lock. Add a `Webcam 2` entry in the robot configuration with the Logitech plugged in to enable it.
- The Arducam has no entry in `src/main/res/xml/teamwebcamcalibrations.xml` until it is calibrated, so `ftcPose` bearing and range are unreliable for it. To calibrate at 640x480: run the `Camera Calibration Capture` OpMode while moving the camera in front of a checkerboard, pull the frames with `adb pull /sdcard/FIRST/data`, run `python3 TeamCode/tools/calibrate_camera.py solve <frames dir>` (`board <out.png>` makes the checkerboard), and paste the printed `<Camera>` element into the XML file. A calibration is only valid at the resolution it was captured at.
- `Robot TeleOp` selects alliance in INIT (X = Blue, B = Red, default Red); hold LB or RB to align the chassis to the alliance audience or far CELL cluster using `ftcPose.bearing`.
- `Basic Auto - Flower to Hive` selects alliance in INIT (X = Blue, B = Red, default Red), starts the intake to collect from the Flower, navigates toward the Hive, aligns to the scoring AprilTag cluster, pauses 3 seconds, and drives to park.
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
