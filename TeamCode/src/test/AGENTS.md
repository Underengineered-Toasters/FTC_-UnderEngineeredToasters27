# TeamCode Unit Tests

## Purpose

Deterministic JVM tests for robot control logic and actuator commands that can run without hardware.

## Ownership

Owns tests under `src/test/java`; production behavior remains owned by TeamCode.

## Local Contracts

- Use JUnit 4, fake timestamps/observations for alignment, and lightweight SDK-interface proxies for shooter and flower arm actuator commands.
- Exercise actual production controller logic; do not substitute Android hardware behavior.
- Cover fresh versus repeated/stale frames, target loss, stable lock, timeout, and motor-command sign/bounds. HoldToAlignController tests exercise release returning manual control, fresh attempts without retained steering, timeout requiring release, and zero steering on camera failure/target loss.
- Shooter tests cover dual shooter1/shooter2 servo setup, partial/missing hardware, both direction/power commands, live tuning with shared adjustment and bounded gap, initialization reset, stopping, unavailable servo RPM, and legacy motor compatibility. Flower arm tests cover retained positional targets on release/stop, upper switch blocking upward targets with downward escape, conflicting inputs, time-step/range bounds, and missing-switch initialization without movement.

## Work Guidance

Keep hardware integration verification explicit; passing JVM tests cannot validate camera calibration or physical steering.

## Verification

Run `./gradlew :TeamCode:testDebugUnitTest`.

## Child DOX Index

None.
