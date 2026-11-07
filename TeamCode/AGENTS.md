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

- FlowerArm uses goBILDA 2000-0025-0002 Dual Mode Torque servo in positional mode (`flower arm`) and touch sensor `flowerUpperLimit`. Its PWM-capable controller is configured for 600–2500 µs: physical observation confirms larger positions raise, so extend the upward endpoint from 2400 to 2500 µs and keep the downward endpoint at 600 µs. The servo supports 500–2500 µs; do not exceed that range. During INIT, Robot/Red/Blue TeleOp and BioBuzzTimedAuto raise the arm until upper touch contact, holding the contact target. An already-pressed sensor skips raising. STOP, early START or a 4-second homing timeout aborts initialization with telemetry instead of throwing for an unpressed sensor. Wait for READY before START. Configure retains the last servo command (not measured angle); larger positions raise and 1.0 is the maximum upward command. Verify physical calibration before deployment. Press gamepad1 B to toggle endpoints: first press lowers from the upper starting position, next press raises until upper contact. Release continues movement; holding B toggles only once. Lowering stops at servo target 0.0. D-pad down does not control the flower arm. Contact latches upward motion off until downward movement, including switch bounce.
- BioBuzzTimedAuto lowers the flower arm for 1.2 seconds before intake and raises until upper contact afterwards (4-second timeout or 29-second match deadline aborts the route). Drive-only autonomous routines do not initialize the arm. A positional servo holds its last commanded target; there is no measured angle feedback, so contact stopping accuracy requires hardware verification.

## Work Guidance
Keep camera API migrations separate from shooter, intake, and drive tuning. Verify physical aiming after deployment.

## Verification
Run `./gradlew assembleDebug test`. Build coverage includes existing autonomous and Road Runner sources.

## Child DOX Index
- `src/test/`: JVM target-selection regression tests.
