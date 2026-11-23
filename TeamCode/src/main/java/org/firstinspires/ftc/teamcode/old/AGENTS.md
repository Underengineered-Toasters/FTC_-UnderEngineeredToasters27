# Legacy OpModes

## Purpose
Retained simple drive and timed autonomous programs, now running on the four-motor mecanum drivetrain.

## Ownership
Owns BasicTankDrive, the basic timed routes, BioBuzzTimedAuto, and its red/blue route subclasses. Shared hardware, vision and the autonomous motion/action layer (`TimedMotion`, `AutoSequence`) remain in the parent teamcode package; Road Runner remains outside this package.

## Local Contracts
- Preserve route shape and OpMode registration names during organizational changes. Distance, turn and battery conversion are no longer local: every route calls `TimedMotion`, so one field calibration serves all of them.
- Package placement does not disable OpModes. These programs run on the four-motor mecanum drivetrain: they use `Robot`'s wheel-name constants, `Robot.applyMecanumDirections` and `Robot.driveMecanum` (or `TimedMotion.forRawMotors`), never their own motor names or mixing.
- Keep BioBuzzTimedAuto and its alliance subclasses together; import the shared Robot from the parent package.
- BasicParkingAutonomous registers as Basic Autonomous: forward 7/46/7/46-inch legs with right/left/left/left 90-degree turns, then backward 7 inches. ParkingAutonomous registers as Parking Autonomous: backward 7 inches, right 90 degrees, then backward 46 inches. Both initialize only the four drive motors (through `Robot`'s names and directions), drive with `TimedMotion.forRawMotors` (battery compensated) and zero them in finally.
- BasicTankDrive is the manual four-wheel drive check: left stick drives and strafes, right stick turns, using the same names, directions and mixing as `Robot`.
- BasicAutonomous retains its separate out-and-back route. It must not negate the drive axes: `Robot` normalizes positive power to forward, so negating made every leg and every turn run mirrored.
- BioBuzzTimedAuto scores through `AutoSequence.shootBalls` (turret settle, spin-up, then one ball per feed cycle) and intakes through `AutoSequence.intakeAtFlower`, so `Robot.update()` runs during autonomous. Plans, the 7 s wait, the 28/26-inch legs, the ±90/180-degree turns and the 27 s second-shot guard are retained.
- Every route wraps its body in `try/finally`: stop all mechanisms and call `robot.close()` on every exit path.
- The `Basic Tank Drive` name is a registration contract; it now drives four mecanum wheels, not two tank sides.

## Work Guidance
Use testing/MecanumDriveTest and testing/OdometryTest for current drivetrain and sensor checks. Confirm hardware configuration before running legacy programs.
Timed distances and turns remain field estimates; recalibrate `TimedMotion` before trusting a route.

## Verification
Run `./gradlew assembleDebug test` after package or import changes.

## Child DOX Index
None.
