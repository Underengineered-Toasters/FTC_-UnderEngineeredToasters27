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

- Driver Station: a browser UI for practice and debugging only, at http://192.168.43.1:8080/ds with websocket namespace `ds`. Game Manual rule R704.D (section 12, version TU05) prohibits third party logging and streaming tools such as FTC Dashboard and says no continuous video stream is allowed, so it must be off for matches and inspection. Related rules: R704.C requires laptops to be off the robot Wi-Fi during matches, R711.A requires a non-default Control Hub Wi-Fi password, and R711.D leaves only the robot Wi-Fi saved on the Driver Station device. OpModes opt in with `DriverStation.attach` and `DriverStation.detach` (in a `finally`). The wire protocol is shared by `src/main/assets/ds/app.js` and `ds/DsSocketHandler.java`, so change both and keep `ds-ui/tests` in sync. Assets are offline only.
- DriverStationTestOpMode ("Driver Station Test", group `ds`) shows per-detection AprilTag detail and a heartbeat on both the Driver Hub and the web UI. Vision init failure only logs a line.

## Work Guidance
Keep camera API migrations separate from shooter, intake, and drive tuning. Verify physical aiming after deployment.

## Verification
Run `./gradlew assembleDebug test`. Build coverage includes existing autonomous and Road Runner sources. Also run `node --test TeamCode/ds-ui/tests/app.test.js` (Node 22); CI runs it before the Gradle steps.

## Child DOX Index
- `src/test/`: JVM regression tests for target selection and the Driver Station server logic.
- `src/main/java/org/firstinspires/ftc/teamcode/ds/`: Driver Station server, websocket handler and OpMode facade.
- `ds-ui/`: Node tests for the Driver Station browser client.
