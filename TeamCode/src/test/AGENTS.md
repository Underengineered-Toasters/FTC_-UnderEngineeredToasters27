# TeamCode Tests

## Purpose
JVM regression tests for SDK 12 target identity selection and flower-arm upper-limit control.

## Ownership
Owns test sources; production behavior belongs to TeamCode.

## Local Contracts
Use actual SDK detection objects and the SDK game database to verify cluster identities, member lookup, and unknown detection handling. Tests do not prove camera calibration or physical aiming.

## Work Guidance
Avoid hardware access in JVM tests. FlowerArmTest uses Servo/TouchSensor interface doubles to check unpressed startup homing, homing contact/timeout/cancellation, pressed startup, contact latching, switch bounce, downward escape, direct-command release hold, and B-button edge toggling with continued travel after release; physical angle and stopping accuracy require robot validation.

## Verification
Run `./gradlew :TeamCode:testDebugUnitTest`.

## Child DOX Index
None.
