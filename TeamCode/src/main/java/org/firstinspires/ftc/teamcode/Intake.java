package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Controls the mirrored positional servos at the front of the robot. */
public class Intake {

    private static final double INTAKE_POSITION = 1.0;
    private static final double EJECT_POSITION = 0.0;
    private static final double NEUTRAL_POSITION = 0.5;

    private Servo frontLeftServo;
    private Servo frontRightServo;

    public void init(HardwareMap hardwareMap) {
        frontLeftServo = hardwareMap.get(Servo.class, "frontLeftServo");
        frontRightServo = hardwareMap.get(Servo.class, "frontRightServo");

        frontLeftServo.setDirection(Servo.Direction.FORWARD);
        frontRightServo.setDirection(Servo.Direction.REVERSE);
        stop();
    }

    public void intake() {
        setPosition(INTAKE_POSITION);
    }

    public void eject() {
        setPosition(EJECT_POSITION);
    }

    public void stop() {
        setPosition(NEUTRAL_POSITION);
    }

    private void setPosition(double position) {
        frontLeftServo.setPosition(position);
        frontRightServo.setPosition(position);
    }
}
