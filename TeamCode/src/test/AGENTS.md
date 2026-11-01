# TeamCode Tests

## Purpose
JVM regression tests for SDK 12 target identity selection.

## Ownership
Owns test sources; production behavior belongs to TeamCode.

## Local Contracts
Use actual SDK detection objects and the SDK game database to verify cluster identities, member lookup, and unknown detection handling. Tests do not prove camera calibration or physical aiming.

## Work Guidance
Avoid hardware access in JVM tests.

## Verification
Run `./gradlew :TeamCode:testDebugUnitTest`.

## Child DOX Index
None.
