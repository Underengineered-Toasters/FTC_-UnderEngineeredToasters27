package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/** Controls the intake motor and the positional feeder servo. */
public class MotorIntake {

    public static final double INTAKE_IN = 1.0;
    public static final double INTAKE_OUT = -1.0;
    public static final double INTAKE_STOP = 0.0;
    public static final double FEEDER_REST = 0.0;
    public static final double FEEDER_PUSH = 0.5;

    private DcMotor intakeMotor;
    private Servo feederServo;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        feederServo = hardwareMap.get(Servo.class, "feeder");
        stopIntake();
        restFeeder();
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void stopIntake() {
        setPower(INTAKE_STOP);
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

    public double getFeederPosition() {
        return feederServo.getPosition();
    }
}
