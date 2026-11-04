# TeamCode Tests

## Purpose
JVM regression tests for SDK 12 target identity selection and for the pure logic of the Driver Station server.

## Ownership
Owns test sources; production behavior belongs to TeamCode.

## Local Contracts
Use actual SDK detection objects and the SDK game database to verify cluster identities, member lookup, and unknown detection handling. The `ds/` tests cover payload validation, the command policy, telemetry bookkeeping and path handling without Android or hardware. Tests do not prove camera calibration, physical aiming, or behavior on the robot.

## Work Guidance
Avoid hardware access in JVM tests.

## Verification
Run `./gradlew :TeamCode:testDebugUnitTest`.

## Child DOX Index
None.
