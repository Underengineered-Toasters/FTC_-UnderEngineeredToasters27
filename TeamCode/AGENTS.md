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

## Verification
Run `./gradlew assembleDebug test`. Build coverage includes existing autonomous and Road Runner sources.

## Child DOX Index
- `src/test/`: JVM target-selection regression tests.
- `old`: retained tank-drive and timed autonomous OpModes.
