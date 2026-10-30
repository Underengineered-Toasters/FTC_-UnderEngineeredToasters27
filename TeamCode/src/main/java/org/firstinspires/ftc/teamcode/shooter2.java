package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Controller for Secondary Flywheel Motor ("shooter2" on Motor Port 3).
 * Direction: REVERSE (Opposite Direction to shooter1).
 * Mode: 100% Maximum Voltage (RUN_WITHOUT_ENCODER).
 */
public class shooter2 {

    private DcMotorEx flywheel;

    public static final double POWER_OFF = 0.0;
    public static final double POWER_MAX = 1.00;

    public void init(HardwareMap hardwareMap) {
        try {
            flywheel = hardwareMap.get(DcMotorEx.class, "shooter2");
        } catch (Exception e) {
            flywheel = null;
        }

        if (flywheel != null) {
            flywheel.setMotorEnable();
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel.setDirection(DcMotor.Direction.REVERSE); // Opposite direction
            spinDown();
        }
    }

    public void spinForwardMaxSpeed() {
        if (flywheel != null) {
            flywheel.setPower(POWER_MAX);
        }
    }

    public void spinReverseMaxSpeed() {
        if (flywheel != null) {
            flywheel.setPower(-POWER_MAX);
        }
    }

    public void spinDown() {
        if (flywheel != null) {
            flywheel.setPower(POWER_OFF);
        }
    }

    public double getCurrentPower() {
        return flywheel != null ? flywheel.getPower() : 0.0;
    }

    public double getCurrentVelocityRPM() {
        if (flywheel != null) {
            return flywheel.getVelocity() * 60.0 / 28.0;
        }
        return 0.0;
    }

    public boolean isSpinning() {
        return flywheel != null && Math.abs(flywheel.getPower()) > 0.05;
    }
}
