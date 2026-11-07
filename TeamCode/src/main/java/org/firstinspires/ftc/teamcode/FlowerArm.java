package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/** Incremental position control with a latched upper touch limit. */
public final class FlowerArm {
    // Installed direction confirmed on robot: larger positions raise the arm.
    public static final double UPPER_POSITION = 1.0;
    // Full commanded range in about 0.67 seconds; actual speed depends on the servo.
    private static final double POSITION_PER_SECOND = 1.5;
    private Servo servo;
    private TouchSensor upperLimit;
    private double target;
    private boolean upperStopped;
    private boolean toggleDown;
    private boolean previousB;
    private long previousNanos;

    public void init(HardwareMap hardwareMap) {
        configure(hardwareMap.get(Servo.class, "flower arm"),
                hardwareMap.get(TouchSensor.class, "flowerUpperLimit"));
    }

    void configure(Servo armServo, TouchSensor limit) {
        if (armServo == null || limit == null) {
            throw new IllegalArgumentException("Configure flower arm servo and flowerUpperLimit");
        }
        if (!(armServo instanceof PwmControl)) {
            throw new IllegalArgumentException("Flower arm requires a PWM-range-capable servo controller");
        }
        // goBILDA 2000-0025-0002 supports 500–2500 us.
        // Extend the actual raising end to 2500 us; retain generic 600 us downward limit.
        ((PwmControl) armServo).setPwmRange(new PwmControl.PwmRange(600, 2500));
        servo = armServo;
        upperLimit = limit;
        servo.setDirection(Servo.Direction.FORWARD);
        // getPosition is the last command, not measured arm angle.
        target = servo.getPosition();
        if (!Double.isFinite(target)) target = 0.5;
        target = Math.max(0, Math.min(1, target));
        upperStopped = limit.isPressed();
        toggleDown = false;
        previousB = false;
        previousNanos = System.nanoTime();
    }

    enum HomingState { MOVING, READY, FAILED }

    HomingState homeStep(double elapsedSeconds, double stepSeconds, boolean cancelled) {
        if (cancelled) {
            stop();
            return HomingState.FAILED;
        }
        if (isUpperLimitPressed()) {
            upperStopped = true;
            stop();
            return HomingState.READY;
        }
        if (elapsedSeconds >= 4.0) {
            stop();
            return HomingState.FAILED;
        }
        update(true, false, stepSeconds);
        return HomingState.MOVING;
    }

    /** Raise during INIT; contact completes homing. STOP/early START/timeout aborts. */
    public boolean homeDuringInit(LinearOpMode opMode) {
        long started = System.nanoTime();
        previousNanos = started;
        while (true) {
            long now = System.nanoTime();
            HomingState state = homeStep((now - started) / 1_000_000_000.0,
                    (now - previousNanos) / 1_000_000_000.0,
                    opMode.isStopRequested() || !opMode.opModeInInit());
            previousNanos = now;
            opMode.telemetry.addData("Flower arm", state == HomingState.MOVING
                    ? "Raising to upper touch sensor" : state);
            if (state == HomingState.FAILED) {
                opMode.telemetry.addLine("Homing stopped. Check upper sensor, then retry INIT; wait for READY before START.");
            }
            opMode.telemetry.update();
            if (state != HomingState.MOVING) return state == HomingState.READY;
            opMode.idle();
        }
    }

    /** Each B press selects the opposite endpoint; release continues the selected movement. */
    public void updateToggle(boolean bPressed) {
        long now = System.nanoTime();
        updateToggle(bPressed, (now - previousNanos) / 1_000_000_000.0);
        previousNanos = now;
    }

    void updateToggle(boolean bPressed, double elapsedSeconds) {
        if (bPressed && !previousB) toggleDown = !toggleDown;
        previousB = bPressed;
        update(!toggleDown, toggleDown, elapsedSeconds);
    }

    public void update(boolean up, boolean down) {
        long now = System.nanoTime();
        update(up, down, (now - previousNanos) / 1_000_000_000.0);
        previousNanos = now;
    }

    void update(boolean up, boolean down, double elapsedSeconds) {
        double dt = Double.isFinite(elapsedSeconds)
                ? Math.max(0, Math.min(0.05, elapsedSeconds)) : 0;
        if (isUpperLimitPressed()) upperStopped = true;
        if (up != down) {
            if (down && dt > 0) {
                target -= POSITION_PER_SECOND * dt;
                // Only downward motion permits a later upward attempt.
                upperStopped = isUpperLimitPressed();
            } else if (up && !upperStopped) {
                target += POSITION_PER_SECOND * dt;
            }
        }
        target = Math.max(0, Math.min(1, target));
        servo.setPosition(target);
    }

    public boolean isUpperLimitPressed() { return upperLimit.isPressed(); }
    public double getTargetPosition() { return target; }
    public void stop() { if (servo != null) servo.setPosition(target); }
}
