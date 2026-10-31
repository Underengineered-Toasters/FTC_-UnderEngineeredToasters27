package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {

    // =========================
    // Intake Power Constants
    // =========================

    public static final double INTAKE_IN = 1.0;
    public static final double INTAKE_OUT = -1.0;
    public static final double INTAKE_STOP = 0.0;

    private DcMotor intakeMotor;

    // =========================
    // Initialize
    // =========================

    public void init(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        stopIntake();
    }

    // =========================
    // Intake Controls
    // =========================

    /**
     * Sets power to the intake motor.
     */
    public void setPower(double power) {
        if (intakeMotor != null) {
            intakeMotor.setPower(power);
        }
    }

    public void intakeIn() {
        setPower(INTAKE_IN);
    }

    public void intakeOut() {
        setPower(INTAKE_OUT);
    }

    public void stopIntake() {
        setPower(INTAKE_STOP);
    }

    // =========================
    // Getters
    // =========================

    public double getPower() {
        return intakeMotor != null ? intakeMotor.getPower() : 0.0;
    }
}