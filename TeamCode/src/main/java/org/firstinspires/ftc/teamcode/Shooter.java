package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Dual-Motor Shooter Controller for FTC (2 Flywheel Motors on Port 2 & Port 3).
 * Motor 1 ("shooter") runs REVERSE.
 * Motor 2 ("shooter2") retains its REVERSE direction.
 * Both motors deliver 100% maximum raw battery voltage.
 */
public class Shooter {

    private DcMotorEx flywheel1; // Primary flywheel motor ("shooter" or "shooter1")
    private DcMotorEx flywheel2; // Secondary flywheel motor ("shooter2")

    private static final double POWER_OFF = 0.0;
    private static final double POWER_MAX = 1.0;
    public void init(HardwareMap hardwareMap) {

        // Initialize Motor 1 ("shooter" or "shooter1")
        try {
            flywheel1 = hardwareMap.get(DcMotorEx.class, "shooter");
        } catch (Exception e) {
            try {
                flywheel1 = hardwareMap.get(DcMotorEx.class, "shooter1");
            } catch (Exception ex) {
                flywheel1 = null;
            }
        }

        // Initialize Motor 2 ("shooter2")
        try {
            flywheel2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        } catch (Exception e) {
            flywheel2 = null;
        }

        // Configure Motor 1 (reversed primary shooter direction)
        if (flywheel1 != null) {
            flywheel1.setMotorEnable();
            flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel1.setDirection(DcMotor.Direction.REVERSE);
            flywheel1.setPower(POWER_OFF);
        }

        // Configure Motor 2 (OPPOSITE DIRECTION: REVERSE)
        if (flywheel2 != null) {
            flywheel2.setMotorEnable();
            flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheel2.setDirection(DcMotor.Direction.REVERSE);
            flywheel2.setPower(POWER_OFF);
        }
    }

    public void spinForwardMaxSpeed() {
        if (flywheel1 != null) flywheel1.setPower(POWER_MAX);
        if (flywheel2 != null) flywheel2.setPower(POWER_MAX);
    }

    public void spinReverseMaxSpeed() {
        if (flywheel1 != null) flywheel1.setPower(-POWER_MAX);
        if (flywheel2 != null) flywheel2.setPower(-POWER_MAX);
    }

    public void spinUpFullSpeed() {
        spinForwardMaxSpeed();
    }

    public void spinUp6000RPM() {
        spinForwardMaxSpeed();
    }

    public void setDirection(boolean reverse) {
        if (flywheel1 != null) {
            flywheel1.setDirection(reverse ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        }
        if (flywheel2 != null) {
            flywheel2.setDirection(reverse ? DcMotor.Direction.FORWARD : DcMotor.Direction.REVERSE);
        }
    }

    public void setTargetPower(double power) {
        double clippedPower = Math.max(-POWER_MAX, Math.min(POWER_MAX, power));
        if (flywheel1 != null) flywheel1.setPower(clippedPower);
        if (flywheel2 != null) flywheel2.setPower(clippedPower);
    }

    public void spinDown() {
        if (flywheel1 != null) flywheel1.setPower(POWER_OFF);
        if (flywheel2 != null) flywheel2.setPower(POWER_OFF);
    }

    public void update() {
        // Direct power mode requires no periodic update loop.
    }

    public double getCurrentPower() {
        return flywheel1 != null ? flywheel1.getPower() : (flywheel2 != null ? flywheel2.getPower() : 0.0);
    }

    public double getCurrentVelocityRPM() {
        if (flywheel1 != null) {
            return flywheel1.getVelocity() * 60.0 / 28.0;
        } else if (flywheel2 != null) {
            return flywheel2.getVelocity() * 60.0 / 28.0;
        }
        return 0.0;
    }

    public boolean isSpinning() {
        return (flywheel1 != null && Math.abs(flywheel1.getPower()) > 0.05) ||
                (flywheel2 != null && Math.abs(flywheel2.getPower()) > 0.05);
    }
}