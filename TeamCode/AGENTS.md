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
- Configure `frontLeftServo` and `frontRightServo` as continuous-rotation servos; `Intake` mirrors their directions and owns intake/eject/stop behavior.
- In `Drive + Shooter`, hold the right bumper to intake, hold the left bumper to eject, and release both to stop the front servos.
- `MotorIntake` controls the `intake` motor and positional `feeder` servo; `Intake TeleOp` uses A/B for motor direction and hold Y to push the feeder.
- Register all AprilTag clusters in the shared `AprilTagVision` library; use each unique ID once and its measured tag size for pose calculations.
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

No nested child DOX files. Robot code and the driver station backend reside in `src/main/java/org/firstinspires/ftc/teamcode/`; the driver station UI source is in `ds-ui/` and its served assets are in `src/main/assets/ds/`.
