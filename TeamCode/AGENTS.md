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

- BasicParkingAutonomous registers as Basic Autonomous with the restored forward 7/46/7/46-inch legs, right/left/left/left 90-degree turns, then backward 7 inches. ParkingAutonomous registers separately as Parking Autonomous: backward 7 inches, right 90 degrees, then backward 46 inches. Both initialize only left/right drive motors. Existing BasicAutonomous remains unchanged. Uses main Robot's motor directions; timed distances/turns require field calibration. STOP/finally zeros both motors.

- FlowerArm uses positional servo `flower arm` and touch sensor `flowerUpperLimit`. INIT requires contact with the upper switch; calibrated upper servo position is 0.0 (smaller raises). Verify this physical calibration before deployment. Hold gamepad1 B to raise, D-pad down to lower; release holds. Contact latches upward motion off until downward movement, including switch bounce.
- BioBuzzTimedAuto lowers the flower arm for 1.2 seconds before intake and raises until upper contact afterwards (4-second timeout or 29-second match deadline aborts the route). Drive-only autonomous routines do not initialize the arm. A positional servo holds its last commanded target; there is no measured angle feedback, so contact stopping accuracy requires hardware verification.

## Work Guidance
Keep camera API migrations separate from shooter, intake, and drive tuning. Verify physical aiming after deployment.

## Verification
Run `./gradlew assembleDebug test`. Build coverage includes existing autonomous and Road Runner sources.

## Child DOX Index
- `src/test/`: JVM target-selection regression tests.
