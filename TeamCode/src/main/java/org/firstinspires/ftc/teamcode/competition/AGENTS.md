# Competition TeleOp

## Purpose
Blue and Red Alliance TeleOps imported from the tuned robot branch, isolated from autonomous hardware code.

## Ownership
Owns the selectable alliance TeleOps, shooter and flower-arm diagnostics, and their local hardware/controller classes.

## Local Contracts
- Controller 1 M2 uses the REV default Cross mapping (`gamepad1.a`); Cross also aligns. M2 must be enabled and mapped accordingly on the controller.
- Hold M2/Cross for turret search/alignment; chassis stops while held. Release stops turret motion. Fresh-frame target loss searches at +/-0.10, initially 400 ms then alternating every 800 ms. Each hold times out after five seconds; release/repress retries. Stale frames/camera failure stop motion. Red targets RED_AUDIENCE, blue BLUE_AUDIENCE.
- Hold L2 for both shooters only, R2 for both shooters plus intake in. L1/R1 command intake in/out. Controller 2 R2 remains a shooter control.
- Shooter caps default to 34%/54%; D-pad left/right adjusts both by one percentage point per press. Y toggles shooter direction after START; INIT Y optionally records aim bearing. Power is not measured RPM.
- Flower arm uses positional servo `flower arm`, starting at 1.0 during INIT; D-pad up/down decreases/increases target at 0.5 units/sec. Release holds target. Touch sensor `flowerUpperLimit` blocks upward commands only.
- Hardware: drive `left`/`right`, motor `intake`, motor `turret`, servos `shooter1`/`shooter2`, camera `Webcam 1` with optional `Webcam 2` fallback. Turret power is capped at 0.15, direction REVERSE; unknown `mag` role means no automatic travel-limit protection.
- Local Robot, Shooter, intake, Turret and AprilTagVision must not be used by root autonomous classes. Their distinct package intentionally preserves main's autonomous hardware behavior.
- Temporary `[DEBUG-camera-align]` logs remain until red alignment is physically diagnosed; physical red lock and camera calibration are unverified.

## Work Guidance
Keep imports within this package for these hardware classes. Update displayed controller instructions when mappings change.

## Verification
Run TeamCode build and JVM tests, then verify physical direction, M2 mapping, shooter power, arm endpoints and camera alignment on the robot. Unit tests do not establish physical calibration.

## Child DOX Index
None.
