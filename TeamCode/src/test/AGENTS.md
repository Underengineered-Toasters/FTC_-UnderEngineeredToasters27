# TeamCode Tests

## Purpose
JVM regression tests for SDK 12 target identity selection and for the pure-Java drive math in `DriveMath` (`DriveMathTest`).

## Ownership
Owns test sources; production behavior belongs to TeamCode.

## Local Contracts
Use actual SDK detection objects and the SDK game database to verify cluster identities, member lookup, and unknown detection handling. `DriveMathTest` checks stick shaping (deadband, continuity, squared curve, NaN and out-of-range input) and tank mixing (pass-through, desaturation that keeps the left/right ratio, output bounded by 1). Tests do not prove camera calibration, physical aiming, or motor polarity.

## Work Guidance
Avoid hardware access in JVM tests.

## Verification
Run `./gradlew :TeamCode:testDebugUnitTest`.

## Child DOX Index
None.
