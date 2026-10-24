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
- `AprilTagVision` streams the Arducam OV9281 (`Webcam 1`) as YUY2 at 640x480 (30 fps). Once the camera is streaming it sets exposure and gain, and `getCameraState()` reports the camera state with the measured fps. Exposure and gain then follow frame brightness within `EXPOSURE_MAX_MS` (10 ms) and `GAIN_MAX`, so keep those caps in mind before widening them.
- Do not switch `AprilTagVision` to MJPEG expecting working frames. MJPEG streams but every frame arrives black (brightness 0 at 1280x800 and 640x480, against about 210 in YUY2). The cause is in the SDK: the 12.0 release notes state that MJPEG payload streaming was added to the UVC driver and that an external JPEG decompression routine is required to use it. `VisionPortal` has no hook for one, and the camera's JPEG bytes reach the processors undecoded. Getting MJPEG means capturing outside `VisionPortal` through `ClassFactory.getInstance().getCameraManager()`, decoding with `BitmapFactory`, and passing the frame to `AprilTagProcessor.processFrame`.
- The camera offers MJPG at 120 fps in every size, YUYV at 30 fps at 640x480 and 800x600, and YUYV at 20 fps at 1280x720 and 1280x800 (listed with `v4l2-ctl --list-formats-ext` with the camera on a Linux host). 640x480 is a centre crop that shows about half the width of the 1280x800 view. YUY2 at 800x600 and at 1280x800 both deliver no frames on the Control Hub, which fits the USB 2.0 bandwidth limit at those sizes.
- Capturing MJPEG directly works and gives the full 1280x800 view, but the Control Hub caps capture at 30 fps regardless. Measured by opening the camera through `ClassFactory.getInstance().getCameraManager()`, requesting MJPG at 30, 60 and 120 fps, and counting frames: 30.5 fps at 1280x800, 640x480 and 320x240 alike, with decoding disabled, so decode is not the limit. JPEG decoding plus tag detection at 1280x800 runs at about 26 fps on its own thread. Do not design around the camera's 120 fps figure: the hub cannot deliver it.
- The direct capture recipe is: `CameraManager.requestPermissionAndOpenCamera` with the `WebcamName`, `Camera.createCaptureRequest(256, size, fps)` (256 is the SDK's android format for MJPG), a capture session, then `CameraFrame.getImageData()` on each frame. Those bytes start with `FF D8`, and `BitmapFactory.decodeByteArray` turns them into a real image, which `AprilTagProcessor.processFrame` accepts after `init(width, height, null)` with `setLensIntrinsics` on the builder.
- `AprilTagVision` falls back to the Logitech C270 (hardware config name `Webcam 2`) at SDK defaults when `Webcam 1` is not configured, not plugged in, or fails to open. The fallback uses the SDK's built-in Logitech calibration and skips the exposure and gain lock. Add a `Webcam 2` entry in the robot configuration with the Logitech plugged in to enable it.
- The Arducam is calibrated at 640x480 in `src/main/res/xml/teamwebcamcalibrations.xml` (focal length about 705 px, principal point (667, 455), which lies outside the frame). That fits a centre crop: frames captured at 640x480 and 1280x800 on a laptop show the middle of the 1280x800 view, roughly half its width. Recalibrate whenever the resolution, lens, or camera mount changes, since the entry is only valid at that resolution: run the `Camera Calibration Capture` OpMode while moving the camera in front of a checkerboard (it reports board, tilt, and coverage; about 2 minutes), pull the frames with `adb pull /sdcard/FIRST/calib`, run `python3 TeamCode/tools/calibrate_camera.py solve <frames dir>` (`board <out.png>` makes the checkerboard), and replace the `<Camera>` element in the XML file. Accept a result only with RMS under 1 px and a stable focal length; frames that all show the board at one angle give nonsense.
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
