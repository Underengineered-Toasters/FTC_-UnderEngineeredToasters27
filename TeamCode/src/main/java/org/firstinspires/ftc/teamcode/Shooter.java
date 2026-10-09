package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private DcMotorEx flywheel;

    // Direct power targets (0.0 to 1.0)
    public static final double POWER_OFF        = 0.0;
    public static final double POWER_CLOSE_SHOT = 0.25;
    public static final double POWER_MID_SHOT   = 0.55;
    public static final double POWER_FAR_SHOT   = 0.55;

    private double targetPower = POWER_OFF;

    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setPower(POWER_OFF);
    }

    public void update() {
        if (flywheel != null) {
            flywheel.setPower(targetPower);
        }
    }

    public void setTargetPower(double power) {
        targetPower = power;
        if (flywheel != null) {
            flywheel.setPower(targetPower);
        }
    }

    public void spinDown() {
        setTargetPower(POWER_OFF);
    }

    public double getCurrentPower() {
        return (flywheel != null) ? flywheel.getPower() : 0.0;
    }

    public double getCurrentVelocity() {
        return (flywheel != null) ? flywheel.getVelocity() : 0.0;
    }

    public boolean isSpinning() {
        return targetPower > 0.05;
    }
}
