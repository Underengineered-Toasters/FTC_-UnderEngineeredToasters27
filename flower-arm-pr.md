## Summary

Refs #77.

```text
INIT -> require upper touch contact
hold B -> raise -> upper contact -> hold
D-pad down -> lower away from switch
timed autonomous -> lower / intake / raise until contact
```

Adds flower arm positional servo control using configured names flower arm and flowerUpperLimit. The three main TeleOps share the upper limit; BioBuzz timed routes abort on raise timeout, STOP or the arm deadline. Startup requires the arm already at the switch.

## Evidence

- **Before:** main has no flower arm B control; match-loader touch logic retracts a different actuator.
  **After:** added JVM tests for pressed startup, unhomed startup, upper contact, bounce, downward escape and release hold.
- Validation in progress. Hardware has not been exercised.

## Merge Danger

**Door:** two-way; revert this change.
**Blast Radius:** flower arm in Robot/Red/Blue TeleOp and BioBuzzTimedAuto. Requires positional servo flower arm, touch sensor flowerUpperLimit, decreasing positions upward, and calibration of UPPER_POSITION (currently 0.0) and timed lowering. Servo target is not measured physical position; stopping accuracy and upper hold torque need robot verification. No deployment performed.
