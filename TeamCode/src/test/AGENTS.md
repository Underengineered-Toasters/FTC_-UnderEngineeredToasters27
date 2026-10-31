# TeamCode Unit Tests

## Purpose

Deterministic JVM tests for robot control logic and actuator commands that can run without hardware.

## Ownership

Owns tests under `src/test/java`; production behavior remains owned by TeamCode.

## Local Contracts

- Use JUnit 4, fake timestamps/observations for alignment, and lightweight SDK-interface proxies for shooter actuator commands.
- Exercise actual production controller logic; do not substitute Android hardware behavior.
- Cover fresh versus repeated/stale frames, target loss, stable lock, timeout, and motor-command sign/bounds.
- Shooter tests cover dual CR-servo setup without a motor, partial/missing hardware, both direction/power commands, stopping, unavailable servo RPM, and legacy motor compatibility.

## Work Guidance

Keep hardware integration verification explicit; passing JVM tests cannot validate camera calibration or physical steering.

## Verification

Run `./gradlew :TeamCode:testDebugUnitTest`.

## Child DOX Index

None.
