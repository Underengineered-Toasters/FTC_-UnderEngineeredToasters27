package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/** Incremental position control with a latched upper touch limit. */
public final class FlowerArm {
    // Installed direction: smaller positions raise the arm. Calibrate before deployment.
    public static final double UPPER_POSITION = 0.0;
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
        // The arm must already be at the upper switch; never sweep it during INIT.
        if (!limit.isPressed()) {
            throw new IllegalStateException("Place flower arm against upper touch sensor before INIT");
        }
        servo = armServo;
        upperLimit = limit;
        servo.setDirection(Servo.Direction.FORWARD);
        target = UPPER_POSITION;
        upperStopped = true;
        toggleDown = false;
        previousB = false;
        servo.setPosition(target);
        previousNanos = System.nanoTime();
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
                target += POSITION_PER_SECOND * dt;
                // Only downward motion permits a later upward attempt.
                upperStopped = isUpperLimitPressed();
            } else if (up && !upperStopped) {
                target -= POSITION_PER_SECOND * dt;
            }
        }
        target = Math.max(0, Math.min(1, target));
        servo.setPosition(target);
    }

    public boolean isUpperLimitPressed() { return upperLimit.isPressed(); }
    public double getTargetPosition() { return target; }
    public void stop() { if (servo != null) servo.setPosition(target); }
}
