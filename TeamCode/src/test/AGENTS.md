# TeamCode Tests

## Purpose
JVM regression tests for SDK 12 target identity selection, the shared autonomous motion contract, and yellow-ball camera geometry/control.

## Ownership
Owns test sources; production behavior belongs to TeamCode.

## Local Contracts
- Use actual SDK detection objects and the SDK game database to verify cluster identities, member lookup, and unknown detection handling.
- `TimedMotionTest` locks the single autonomous calibration contract: inches/degrees to seconds, battery compensation and the unusable-voltage fallback, direction applied to power, and that every move ends stopped and aborts when the OpMode is inactive. Change those expectations only together with the field calibration in `TimedMotion`.
- `YellowBallVisionTest` locks the pinhole geometry (focal length, bearing sign, similar-triangle range) and the chase control law (grab window, forward/turn power, final-approach window). The expected numbers are derived from the C270 defaults; re-derive them if the camera, resolution or ball diameter changes.
- Tests do not prove camera calibration, feeding, aiming, colour thresholds or physical route accuracy.

## Work Guidance
Avoid hardware access in JVM tests. Keep `TimedMotion` conversions and `YellowBallVision` geometry as static functions reachable without an OpMode or `HardwareMap` so they stay testable.

## Verification
Run `./gradlew :TeamCode:testDebugUnitTest`.

## Child DOX Index
None.
