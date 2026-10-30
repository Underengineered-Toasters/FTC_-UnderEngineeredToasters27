package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Controller for Primary Flywheel Motor ("shooter1" on Motor Port 2).
 * Includes Soft-Start Inrush Voltage Protection to prevent battery sag (from 8.9V back to 12V+).
 */
public class shooter1 {

    private DcMotorEx flywheel;

    public static final double POWER_OFF = 0.0;
    public static final double POWER_MAX = 1.00;

    private double currentPower = POWER_OFF;

    public void init(HardwareMap hardwareMap) {
        try {
            flywheel = hardwareMap.get(DcMotorEx.class, "shooter1");
        } catch (Exception e) {
            try {
                flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
            } catch (Exception ex) {
                flywheel = null;
            }
        }

        if (flywheel != null) {
            flywheel.setMotorEnable();
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel.setDirection(DcMotor.Direction.FORWARD);
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
