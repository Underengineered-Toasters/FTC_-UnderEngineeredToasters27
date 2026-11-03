# TeamCode

## Purpose
Team OpModes, camera alignment, hardware subsystems and Road Runner routes.

## Ownership
Team programming owns production code and its physical calibration.

## Local Contracts
- FTC SDK 12 uses separate AprilTagSingleDetection and AprilTagClusterDetection types. Use AprilTagTargets for identity/name access through the base type.
- AprilTagVision uses the SDK current-game library. BIOBUZZ clusters are Red Scoring 30-33, Red Audience 34-37, Blue Audience 38-41, Blue Scoring 42-45. Do not register these as individual tags.
- ID lookup for BIOBUZZ returns the containing cluster pose; it does not prove that particular member tag was visible. Poses are relative to moving targets, not absolute field locations.
- SDK migration preserves main's hardware controls and existing route timing. AprilTagAutonomous targets Red Audience then Red Scoring using corrected SDK identities.
- Alliance TeleOps retain main's static turret behavior; SDK compatibility alone does not enable their commented-out alignment.
- DriveMath (pure Java, unit-tested) is the single source of truth for tank mixing. TeleOps (RobotTeleOp, BlueTeleOp, RedTeleOp, BasicTankDrive) shape sticks with DriveMath.shape. In RobotTeleOp, BlueTeleOp and RedTeleOp, holding gamepad1 left stick button multiplies both shaped values by DriveMath.SLOW_MODE_SCALE; the RobotTeleOp LB/RB auto-align PID outputs skip shaping and slow mode. Robot.drive takes raw forward/turn commands (autonomous callers pass them unshaped) and desaturates through DriveMath.leftPower/rightPower instead of clipping. Those three TeleOps call Robot.stopDrive when the loop exits so STOP leaves the drive motors at zero. BasicTankDrive keeps its own motor directions and stick signs and zeroes its motors on exit.

- BasicParkingAutonomous registers as Basic Autonomous with the restored forward 7/46/7/46-inch legs, right/left/left/left 90-degree turns, then backward 7 inches. ParkingAutonomous registers separately as Parking Autonomous: backward 7 inches, right 90 degrees, then backward 46 inches. Both initialize only left/right drive motors. Existing BasicAutonomous remains unchanged. Uses main Robot's motor directions; timed distances/turns require field calibration. STOP/finally zeros both motors.

## Work Guidance
Keep camera API migrations separate from shooter, intake, and drive tuning. Verify physical aiming after deployment.

## Verification
Run `./gradlew assembleDebug test`. Build coverage includes existing autonomous and Road Runner sources.

## Child DOX Index
- `src/test/`: JVM target-selection regression tests.
