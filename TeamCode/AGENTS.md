# TeamCode DOX

## Purpose

Contains team-written robot control software for competition, including the main teleoperation OpMode (`RobotTeleOp`), the autonomous path, subsystem classes, and sensor vision pipelines.

## Ownership

Owned by the FTC student programming team. All robot behavior modifications occur within this module.

## Local Contracts

- `BasicAutonomous.java` runs an out-and-back route with 7-inch initial/return short legs, 46-inch long legs and 180-degree turnaround/final turns. After the long return leg, the robot restores its initial heading and backs up 7 inches to the nominal starting position. Returning to the starting pose depends on hardware calibration of the turnaround. `BasicAutonomous` negates forward and turn inputs at the `Robot.drive` boundary because positive drive power moves this chassis backward; its route uses positive distances for physical forward and left/right helpers for physical turns. Driving and turning use time estimates; calibrate distance speed and turn duration on the robot before relying on endpoint accuracy.
- `BasicAutonomous` first aligns the chassis to the selected scoring cluster after START (X = Blue, B = Red during INIT; default Red), then attempts four pollen before route driving: configured shooter power caps (shooter1 40%, shooter2 60%), 2000 ms spin-up, then four 400 ms push/600 ms rest feeder cycles. Missing feeder, alignment failure, lost aim before/during feeding, camera failure, or STOP aborts the route. Shooter/intake stop and feeder returns to rest in cleanup. This is timed power/feeding, not regulated RPM or ball detection; tune on hardware.
- `RobotTeleOp` uses controller 1 left-stick Y directly for manual drive: negative Y (stick forward) commands negative drive power for physical forward on this chassis. Right-stick turning is unchanged. AprilTag visibility remains telemetry; odometry telemetry requires real accessors in `Robot`.
- `AutonomousCameraAligner` only turns in place; range is telemetry, not a shooting-distance controller. Aim defaults to 0-degree bearing; calibrate camera-to-shooter offset on hardware. Raw positive turn is physical left on the current drivetrain.
- Alignment requires five distinct fresh frames over 200 ms, within 2 degrees and at most 8 deg/sec bearing change. Frames older than 250 ms, target loss, or non-streaming cameras stop turning; camera failure and a 5-second timeout abort. `Camera Alignment Test` selects alliance and tests this without shooting.
- `AprilTagVision` uses the existing 640x480 YUY2 Arducam setup with exposure/gain controls and startup Logitech fallback (`Webcam 2`). Runtime loss reports failure instead of reopening. Imported lens calibration needs physical bearing verification; its principal point is outside the image.
- `Shooter` controls both CR servos: `shooter1` on Control Hub servo port 1 and `shooter2` on port 3 (verified Comp configuration). Both share direction and run/stop controls, with independent commanded-power caps of 40% for shooter1 and 60% for shooter2 in shared Shooter code (including autonomous). Lower requested power is honored below each cap. Legacy motor cap stays 40%. Power is not measured RPM; TeleOp displays each servo command separately. Default shooting uses REVERSE servo direction based on the user hardware test; the Y toggle swaps to FORWARD. Legacy motor direction remains unchanged; verify loaded firing physically. Partial pairs fail initialization with the present servo stopped. If neither is configured, legacy `shooter` DcMotorEx remains supported. Port 0 is the flower arm and must never receive shooter commands. CR servo RPM is unavailable; run state is commanded. Comp still lacks `feeder` and reports a `colorSensor` I2C fault; resolve wiring/configuration before autonomous feeding.
- Keep OpModes organized under `org.firstinspires.ftc.teamcode`.
- Annotate competition TeleOp classes with `@TeleOp(name = "...", group = "...")`.
- Annotate competition Autonomous classes with `@Autonomous(name = "...", group = "...")`.
- Ensure actuators and sensors match hardware configuration names on the Control Hub.
- `ColorSensorTest.java` provides the `Ball Identifier` TeleOp for a REV Color Sensor V3 named `colorSensor`; it displays raw RGB and yellow POLLEN, red/blue NECTAR, or NO BALL / UNKNOWN labels. Tune color ratios with actual balls and lighting before relying on classification; this test has no distance-based presence check.
- `AprilTagVision` uses `AprilTagGameDatabase.getCurrentGameTagLibrary()` (Sample tags plus BIOBUZZ clusters 30-45). Do not register IDs 30-45 as single tags; cluster poses are relative targeting data since the tags move with the CELLs, not absolute field localization.
- `Robot TeleOp` controller 1 L1 (left bumper) runs intake in at +1; R1 (right bumper) runs intake out at -1. Without R2, release or both bumpers stops intake. Controller 1 R2 (right trigger >0.05) runs both shooters and intake in, overriding bumper intake commands while held; release returns intake to bumper controls and stops shooters unless controller 2 trigger remains held. Y still toggles shooter direction. Cleanup stops intake after successful initialization. Intake buttons do not move the feeder. L1/R1 remain intake-only. Hold controller 1 L2 (left trigger >0.05) for chassis-only camera alignment; release immediately restores manual driving. INIT Square/X selects Blue, Circle/B selects Red (default Red). Aim uses HoldToAlignController and the shared fresh-frame ClusterAlignmentController, with a 5-second timeout per hold and release/repress to retry. Camera failure, no target or stale frames command zero drive/turn while held. Arm, intake and shooter controls remain responsive; shooting is operator-controlled, not gated by TeleOp alignment. STOP cleanup commands zero chassis drive.
- `org.firstinspires.ftc.teamcode.ds` hosts the driver station web UI backend (web handlers, `ds` websocket namespace, diagnostics poller); opmodes publish through the `DriverStation` facade.
- `src/main/assets/ds/` holds the web UI assets served at `/ds`; `app.css` is generated output — edit `ds-ui/input.css` and rebuild with `cd TeamCode/ds-ui && ./build.sh` instead of editing it directly.

- `FlowerArm` uses positional Servo `flower arm` on Control Hub servo port 0: goBILDA dual-mode torque servo in default positional mode. REV Touch Sensor `flowerUpperLimit` uses digital channel 1 (physical 0–1 socket). INIT commands raised target 0.0, which can move the arm; support/clear it before initialization. Controller 1 D-pad up decreases target and down increases it at 0.5 normalized units/sec, bounded 0–1 with a 50 ms per-update cap. Release/both inputs hold the last target; upper touch blocks upward target changes only. Missing hardware fails before commanding position. Target telemetry is commanded, not physical angle feedback. Verify direction, endpoints and load on hardware; there is no lower switch. Holding depends on servo torque and enabled PWM and is not guaranteed after OpMode STOP or power loss.

- `FlowerArmDiagnostic` provides `Flower Arm Test`: runs the production positional arm controller without Robot, drive, shooter, intake, or vision initialization. Displays controller 1 D-pad inputs, commanded target, SDK command, servo port, PWM controller status, and touch input. INIT still commands raised target 0.0; controls require START. Test DOWN first because UP is clamped at the initial upper command limit. No physical position feedback is available.

## Work Guidance

- Follow Java conventions used in existing OpModes and subsystems.
- Keep motor and servo initializations in initialization blocks before `waitForStart()`.
- Test hardware code safely by keeping emergency stops accessible via gamepad.

## Verification

- Verify `Ball Identifier` with yellow, red, and blue balls, a neutral object, and an empty sensing area; check both labels and raw readings at the intended mounting distance.
- Build and verify via Gradle: `./gradlew :TeamCode:assembleDebug`.
- Deploy to the robot controller via wireless ADB (`adb connect 192.168.43.1:5555`).

## Child DOX Index

- `src/test/`: deterministic JVM alignment, shooter, and flower arm actuator tests and their verification contract.
- Production sources, resources, and hardware verification remain owned here.
