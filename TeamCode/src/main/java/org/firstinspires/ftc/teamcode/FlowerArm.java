package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/** Position control for the default-mode goBILDA torque servo. */
public class FlowerArm {
    private static final double INITIAL_POSITION = 0.5;
    private static final double POSITION_PER_SECOND = 0.25;
    private Servo servo;
    private TouchSensor upperLimit;
    private double target = INITIAL_POSITION;
    private long previousNanos;

    public void init(HardwareMap hardwareMap) {
        configure(hardwareMap.get(Servo.class, "flower arm"),
                hardwareMap.get(TouchSensor.class, "flowerUpperLimit"));
    }

    void configure(Servo armServo, TouchSensor limit) {
        if (armServo == null || limit == null) {
            throw new IllegalArgumentException("Configure flower arm positional servo and flowerUpperLimit touch sensor");
        }
        servo = armServo;
        upperLimit = limit;
        servo.setDirection(Servo.Direction.FORWARD);
        target = INITIAL_POSITION;
        servo.setPosition(target);
        previousNanos = System.nanoTime();
    }

    public void update(boolean up, boolean down) {
        long now = System.nanoTime();
        update(up, down, (now - previousNanos) / 1_000_000_000.0);
        previousNanos = now;
    }

    void update(boolean up, boolean down, double elapsedSeconds) {
        // Limit delayed loops to 50 ms of travel; release holds the last target.
        double dt = Double.isFinite(elapsedSeconds)
                ? Math.max(0, Math.min(0.05, elapsedSeconds)) : 0;
        if (up != down) {
            if (down) target += POSITION_PER_SECOND * dt;
            else if (!isUpperLimitPressed()) target -= POSITION_PER_SECOND * dt;
        }
        target = Math.max(0, Math.min(1, target));
        servo.setPosition(target);
    }

    public boolean isUpperLimitPressed() { return upperLimit.isPressed(); }
    // Commanded target, not a measurement of physical arm angle.
    public double getTargetPosition() { return target; }
    public void stop() { if (servo != null) servo.setPosition(target); }
}
