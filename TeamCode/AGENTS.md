# TeamCode

## Purpose
Team OpModes, hardware subsystems, vision, and Road Runner routes.

## Ownership
The programming team owns robot behavior. Root-package autonomous programs and their existing hardware classes retain the main-branch implementation.

## Local Contracts
- Competition TeleOp lives in `src/main/java/org/firstinspires/ftc/teamcode/competition/`. Keep its hardware classes separate from autonomous dependencies so control tuning does not silently change autonomous behavior.
- Root-package RedTeleOp, BlueTeleOp, RobotTeleOp and ShooterTeleOp are disabled to avoid duplicate selectable entries; their implementations remain for reference.
- FTC SDK 12 is required for competition AprilTag cluster detection. SDK upgrades affect the whole app; verify compilation of autonomous and Road Runner code too.
- Preserve autonomous routes and root Robot, Shooter, intake, turret and AprilTagVision implementations when integrating TeleOp-only changes.

## Work Guidance
- Match configured hardware names and verify physical controls after deployment.

## Verification
- Run `./gradlew :TeamCode:assembleDebug :TeamCode:testDebugUnitTest`.
- Compare autonomous files and root hardware dependencies against the integration base before merging TeleOp-only work.

## Child DOX Index
- `src/main/java/org/firstinspires/ftc/teamcode/competition/`: competition TeleOp controls and isolated hardware implementation.
- `src/test/`: deterministic controller and actuator tests.
