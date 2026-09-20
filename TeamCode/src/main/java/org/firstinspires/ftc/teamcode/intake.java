package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class intake {

    // 100% motor speeds
    public static final double INTAKE_IN = 1.0;
    public static final double INTAKE_OUT = -1.0;
    public static final double INTAKE_STOP = 0.0;

    // Servo positions (adjust 0.0 to 1.0 depending on mechanical travel)
    public static final double FEEDER_REST = 0.0;
    public static final double FEEDER_PUSH = 0.5;

    private DcMotor intakeMotor;
    private Servo feederServo;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        feederServo = hardwareMap.get(Servo.class, "feeder");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Move feeder servo to initial rest position
        feederServo.setPosition(FEEDER_REST);
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void pushBall() {
        feederServo.setPosition(FEEDER_PUSH);
    }

    public void restFeeder() {
        feederServo.setPosition(FEEDER_REST);
    }

    public double getPower() {
        return intakeMotor.getPower();
    }

    public double getServoPosition() {
        return feederServo.getPosition();
    }
}
