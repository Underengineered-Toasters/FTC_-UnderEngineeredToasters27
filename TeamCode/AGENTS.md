# TeamCode

## Purpose
Team OpModes, camera alignment, hardware subsystems, autonomous routes and Road Runner routes.

## Ownership
Team programming owns production code and its physical calibration.

## Local Contracts
- FTC SDK 12 uses separate AprilTagSingleDetection and AprilTagClusterDetection types. Use AprilTagTargets for identity/name access through the base type.
- AprilTagVision uses the SDK current-game library. BIOBUZZ clusters are Red Scoring 30-33, Red Audience 34-37, Blue Audience 38-41, Blue Scoring 42-45. Do not register these as individual tags.
- ID lookup for BIOBUZZ returns the containing cluster pose; it does not prove that particular member tag was visible. Poses are relative to moving targets, not absolute field locations.
- `Robot.drive(forward, turn)` is positive forward and positive clockwise. SDK yaw/bearing is positive counter-clockwise. Multiply any SDK-derived heading by `Robot.SDK_YAW_TO_TURN` before calling `drive`.
- Autonomous distance/heading conversion lives only in `robot/TimedMotion`; scoring, intake and tag-alignment actions live only in `robot/AutoSequence`. Do not reintroduce per-OpMode distance, turn or battery constants.
- Timed autonomous movement must go through `TimedMotion`, which calls `Robot.update()` every iteration (match-loader touch safety) and stops the drivetrain on every exit. Raw `left`/`right` routes use `TimedMotion.forRawMotors`.
- `turret.setPower` is the only place the `mag` limit switch is enforced: movement continuing into an active stop is refused, reversing out is allowed. Do not bypass it with `turretMotor.setPower`.
- AprilTagAutonomous selects the alliance before start (A = RED, B = BLUE) and scores the preload; it has no flower leg.
- One `VisionPortal` per camera. An OpMode that owns the webcam itself (yellow-ball detection) must call `Robot.init(hardwareMap, false)` so the AprilTag portal stays closed; two portals cannot hold the same camera.
- The Logitech C270 is a standard USB UVC webcam. There is no vendor SDK: it is configured in the Driver Station as `Webcam 1` and opened through `WebcamName`/`VisionPortal`.
- `YellowBallVision` reports `bearingDegrees` positive to the robot's right, which is the sign `Robot.drive` uses for a positive turn. Do not multiply that bearing by `SDK_YAW_TO_TURN`; that constant is only for SDK values (AprilTag `ftcPose.bearing`, which is positive to the left).
- Yellow-ball range is monocular: it is only correct when `YellowBallVision.ballDiameterInches` and `diagonalFovDegrees` match the real ball and camera. Calibrate with `Yellow Ball Camera Test` before trusting the chase.
- Alliance TeleOps retain main's static turret behavior; SDK compatibility alone does not enable their commented-out alignment.

- Legacy tank-drive and timed autonomous programs live in `old`. Their local DOX owns retained route contracts; shared hardware and vision remain in the parent package.

## Work Guidance
Start from the repo-root `DESIGN.md` for architecture and flow; keep it current when changing subsystems, hardware names, OpModes or control flow.
Keep camera API migrations separate from shooter, intake, and drive tuning. Verify physical aiming after deployment.
Before relying on an autonomous route, calibrate `TimedMotion` on the field: measure an actual 90° turn at `TimedMotion.turnPower` and a measured-distance drive, then confirm `AprilTagAutonomous.scoringRangeInches` matches the flywheel's effective range. For the yellow-ball chase, confirm range accuracy at a measured distance with `Yellow Ball Camera Test`.

## Verification
Run `./gradlew assembleDebug test`. JVM tests cover AprilTag cluster identity (`AprilTagTargetsTest`), the autonomous conversion/battery contract (`TimedMotionTest`), and the yellow-ball geometry plus chase control law (`YellowBallVisionTest`); hardware timing, feeding, aiming and colour thresholds still require the robot.

## Child DOX Index
- `src/test/`: JVM regression tests for target selection, autonomous motion conversion and yellow-ball camera geometry.
- `old`: retained tank-drive and timed autonomous OpModes.
