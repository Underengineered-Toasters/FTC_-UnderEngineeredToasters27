package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class intake {

    // =========================
    // Intake Motor
    // =========================

    public static final double INTAKE_IN = 1.0;
    public static final double INTAKE_OUT = -1.0;
    public static final double INTAKE_STOP = 0.0;

    private DcMotor intakeMotor;

    // =========================
    // Feeder Servo
    // =========================

    public static final double FEEDER_REST = 0.0;
    public static final double FEEDER_PUSH = 0.5;

    private Servo feederServo;

    // =========================
    // Front Servos
    // =========================

    private Servo frontLeftServo;
    private Servo frontRightServo;

    // =========================
    // Initialize
    // =========================

    public void init(HardwareMap hardwareMap) {

        // Intake motor
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        // Feeder servo
        feederServo = hardwareMap.get(Servo.class, "feeder");
        feederServo.setPosition(FEEDER_REST);

        // Front servos
        frontLeftServo = hardwareMap.get(Servo.class, "frontLeftServo");
        frontRightServo = hardwareMap.get(Servo.class, "frontRightServo");
    }

    // =========================
    // Intake Motor Controls
    // =========================

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    public void intakeIn() {
        intakeMotor.setPower(INTAKE_IN);
    }

    public void intakeOut() {
        intakeMotor.setPower(INTAKE_OUT);
    }

    public void stopIntake() {
        intakeMotor.setPower(INTAKE_STOP);
    }

    // =========================
    // Feeder Controls
    // =========================

    public void pushBall() {
        feederServo.setPosition(FEEDER_PUSH);
    }

    public void restFeeder() {
        feederServo.setPosition(FEEDER_REST);
    }

    // =========================
    // Front Servo Controls
    // =========================

    public void setFrontLeftPosition(double position) {
        frontLeftServo.setPosition(position);
    }

    public void setFrontRightPosition(double position) {
        frontRightServo.setPosition(position);
    }

    public void setFrontServos(double leftPosition, double rightPosition) {
        frontLeftServo.setPosition(leftPosition);
        frontRightServo.setPosition(rightPosition);
    }

    // =========================
    // Getters
    // =========================

    public double getPower() {
        return intakeMotor.getPower();
    }

    public double getFeederPosition() {
        return feederServo.getPosition();
    }

    public double getFrontLeftPosition() {
        return frontLeftServo.getPosition();
    }

    public double getFrontRightPosition() {
        return frontRightServo.getPosition();
    }
}
