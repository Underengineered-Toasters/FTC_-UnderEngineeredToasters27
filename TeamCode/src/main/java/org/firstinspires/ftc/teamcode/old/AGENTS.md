# Legacy OpModes

## Purpose
Retained two-motor tank-drive and timed autonomous programs for reference while the mecanum/Pedro replacement is developed.

## Ownership
Owns BasicTankDrive, the basic timed routes, BioBuzzTimedAuto, and its red/blue route subclasses. Shared hardware and vision remain in the parent teamcode package; Road Runner remains outside this package.

## Local Contracts
- Preserve route timing, motor directions, and OpMode registration names during organizational changes.
- Package placement does not disable OpModes. These programs retain the older `left`/`right` hardware configuration and are not configured for the four-motor mecanum robot.
- Keep BioBuzzTimedAuto and its alliance subclasses together; import the shared Robot from the parent package.
- BasicParkingAutonomous registers as Basic Autonomous: forward 7/46/7/46-inch legs with right/left/left/left 90-degree turns, then backward 7 inches. ParkingAutonomous registers as Parking Autonomous: backward 7 inches, right 90 degrees, then backward 46 inches. Both initialize only left/right drive motors using Robot's directions and zero them in finally. Timed distances and turns require field calibration. BasicAutonomous retains its separate out-and-back route.

## Work Guidance
Use testing/MecanumDriveTest and testing/OdometryTest for current drivetrain and sensor checks. Confirm hardware configuration before running legacy programs.

## Verification
Run `./gradlew assembleDebug test` after package or import changes.

## Child DOX Index
None.
