package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/** Manual flower extraction arm with an upper travel limit. */
public class FlowerArm {
    // Negative servo power raises the installed arm; positive power lowers it.
    private static final double UP_POWER = -0.3;
    private CRServo servo;
    private TouchSensor upperLimit;

    public void init(HardwareMap hardwareMap) {
        configure(hardwareMap.get(CRServo.class, "flower arm"),
                hardwareMap.get(TouchSensor.class, "flowerUpperLimit"));
    }

    void configure(CRServo armServo, TouchSensor limit) {
        servo = armServo;
        stop();
        if (servo == null || limit == null) {
            throw new IllegalArgumentException("Configure flower arm CR servo and flowerUpperLimit touch sensor");
        }
        upperLimit = limit;
        servo.setDirection(CRServo.Direction.FORWARD);
    }

    public void update(boolean up, boolean down) {
        // Conflicting inputs stop; the upper limit never blocks downward escape.
        double power = 0;
        if (up != down) {
            if (down) power = -UP_POWER;
            else if (!isUpperLimitPressed()) power = UP_POWER;
        }
        servo.setPower(power);
    }

    public boolean isUpperLimitPressed() { return upperLimit.isPressed(); }
    public double getPower() { return servo == null ? 0 : servo.getPower(); }
    public void stop() { if (servo != null) servo.setPower(0); }
}
