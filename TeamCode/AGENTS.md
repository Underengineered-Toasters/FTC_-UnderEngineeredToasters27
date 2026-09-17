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

## Work Guidance

- Follow Java conventions used in existing OpModes and subsystems.
- Keep motor and servo initializations in initialization blocks before `waitForStart()`.
- Test hardware code safely by keeping emergency stops accessible via gamepad.

## Verification

- Build and verify via Gradle: `./gradlew :TeamCode:assembleDebug`.
- Deploy to the robot controller via wireless ADB (`adb connect 192.168.43.1:5555`).

## Child DOX Index

No nested child indexes. All team code resides in `src/main/java/org/firstinspires/ftc/teamcode/`.
