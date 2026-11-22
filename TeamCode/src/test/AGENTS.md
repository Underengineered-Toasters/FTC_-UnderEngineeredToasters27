# TeamCode Tests

## Purpose
JVM regression tests for SDK 12 target identity selection and for the shared autonomous motion contract.

## Ownership
Owns test sources; production behavior belongs to TeamCode.

## Local Contracts
- Use actual SDK detection objects and the SDK game database to verify cluster identities, member lookup, and unknown detection handling.
- `TimedMotionTest` locks the single autonomous calibration contract: inches/degrees to seconds, battery compensation and the unusable-voltage fallback, direction applied to power, and that every move ends stopped and aborts when the OpMode is inactive. Change those expectations only together with the field calibration in `TimedMotion`.
- Tests do not prove camera calibration, feeding, aiming or physical route accuracy.

## Work Guidance
Avoid hardware access in JVM tests. Keep `TimedMotion` conversions reachable without an OpMode or `HardwareMap` so they stay testable.

## Verification
Run `./gradlew :TeamCode:testDebugUnitTest`.

## Child DOX Index
None.
