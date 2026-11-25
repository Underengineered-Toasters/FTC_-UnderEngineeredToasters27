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

- Legacy tank-drive and timed autonomous programs live in `old`. Their local DOX owns retained route contracts; shared hardware and vision remain in the parent package.

## Work Guidance
Keep camera API migrations separate from shooter, intake, and drive tuning. Verify physical aiming after deployment.
- Tune `DualShooterSubsystem` PIDF gains live through FTC Dashboard's `@Config` fields; copy tuned values back into source because dashboard edits reset when the app restarts.
- `DualShooterTestTeleOp` starts a burst with gamepad A and cancels with B. It pulses the `matchLoader` servo only when both shooter motors reach the configured RPM tolerance, then waits for recovery before the next ball. Tune burst count, feed pulse duration, RPM tolerance, and readiness timeout through its Dashboard `@Config` fields.
- `DualShooterTestTeleOp` uses the centered IMU as the robot pose origin and projects the shooter's 3.6-inch offset on the side opposite the intake-facing direction along the robot heading. Its default corner start assumes an 18-inch robot flush against the lower and left field boundaries, placing its center at (9,9) facing the lower wall; the shooter point is then at (9,12.6), toward the upper hive. Update those assumptions if the starting pose or mounting changes.

## Verification
Run `./gradlew assembleDebug test`. Build coverage includes existing autonomous and Road Runner sources.

## Child DOX Index
- `src/test/`: JVM target-selection regression tests.
- `old`: retained tank-drive and timed autonomous OpModes.
