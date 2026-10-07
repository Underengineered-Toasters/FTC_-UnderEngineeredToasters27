package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
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
    // Feeder Servo (continuous rotation)
    // =========================

    // CRServo power semantics: 0 = stop, +/- = direction/speed.
    public static final double FEEDER_REST = 0.0;
    public static final double FEEDER_PUSH = 0.5;

    private CRServo feederServo;

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

        // Servos are optional until they are wired and added to the config.
        // The feeder is a continuous-rotation servo: look it up as CRServo,
        // not Servo, or the lookup silently returns null (issue #45).
        feederServo = hardwareMap.tryGet(CRServo.class, "feeder");
        if (feederServo != null) {
            feederServo.setPower(FEEDER_REST);
        }
        frontLeftServo = hardwareMap.tryGet(Servo.class, "frontLeftServo");
        frontRightServo = hardwareMap.tryGet(Servo.class, "frontRightServo");
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
        if (feederServo != null) feederServo.setPower(FEEDER_PUSH);
    }

    public void restFeeder() {
        if (feederServo != null) feederServo.setPower(FEEDER_REST);
    }

    // =========================
    // Front Servo Controls
    // =========================

    public void setFrontLeftPosition(double position) {
        if (frontLeftServo != null) frontLeftServo.setPosition(position);
    }

    public void setFrontRightPosition(double position) {
        if (frontRightServo != null) frontRightServo.setPosition(position);
    }

    public void setFrontServos(double leftPosition, double rightPosition) {
        setFrontLeftPosition(leftPosition);
        setFrontRightPosition(rightPosition);
    }

    // =========================
    // Getters
    // =========================

    public double getPower() {
        return intakeMotor.getPower();
    }

    public double getFeederPower() {
        return feederServo != null ? feederServo.getPower() : Double.NaN;
    }

    public double getFrontLeftPosition() {
        return frontLeftServo != null ? frontLeftServo.getPosition() : Double.NaN;
    }

    public double getFrontRightPosition() {
        return frontRightServo != null ? frontRightServo.getPosition() : Double.NaN;
    }
}
