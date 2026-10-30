package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Controller for Secondary Flywheel Motor ("shooter2" on Motor Port 3).
 * Includes Soft-Start Inrush Voltage Protection to prevent battery sag (from 8.9V back to 12V+).
 */
public class shooter2 {

    private DcMotorEx flywheel;

    public static final double POWER_OFF = 0.0;
    public static final double POWER_MAX = 1.00;

    private double currentPower = POWER_OFF;

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
            // Soft-Start Ramping prevents current spikes and protects battery voltage from dropping to 8.9V
            if (currentPower < 0.50) {
                currentPower = 0.50;
            } else if (currentPower < 0.80) {
                currentPower = 0.80;
            } else {
                currentPower = POWER_MAX;
            }
            flywheel.setPower(currentPower);
        }
    }

    public void spinReverseMaxSpeed() {
        if (flywheel != null) {
            if (currentPower > -0.50) {
                currentPower = -0.50;
            } else if (currentPower > -0.80) {
                currentPower = -0.80;
            } else {
                currentPower = -POWER_MAX;
            }
            flywheel.setPower(currentPower);
        }
    }

    public void spinDown() {
        currentPower = POWER_OFF;
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
