# TeamCode Unit Tests

## Purpose

Deterministic JVM tests for robot control logic that can run without hardware.

## Ownership

Owns tests under `src/test/java`; production behavior remains owned by TeamCode.

## Local Contracts

- Use JUnit 4 and fake timestamps/observations for alignment tests.
- Exercise actual production controller logic; do not substitute Android hardware behavior.
- Cover fresh versus repeated/stale frames, target loss, stable lock, timeout, and motor-command sign/bounds.

## Work Guidance

Keep hardware integration verification explicit; passing JVM tests cannot validate camera calibration or physical steering.

## Verification

Run `./gradlew :TeamCode:testDebugUnitTest`.

## Child DOX Index

None.
