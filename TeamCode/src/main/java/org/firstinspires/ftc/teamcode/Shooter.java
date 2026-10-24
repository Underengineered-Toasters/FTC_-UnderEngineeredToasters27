
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private DcMotorEx flywheel;

    private static final double POWER_OFF = 0.0;
    private static final double POWER_MAX = 1.0;

    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");

        flywheel.setMotorEnable();
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setPower(POWER_OFF);
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

    public void spinUpFullSpeed() {
        spinForwardMaxSpeed();
    }

    public void spinUp6000RPM() {
        // Maximum power command; actual RPM depends on load.
        spinForwardMaxSpeed();
    }

    public void setDirection(boolean reverse) {
        if (flywheel != null) {
            flywheel.setDirection(
                    reverse
                            ? DcMotor.Direction.REVERSE
                            : DcMotor.Direction.FORWARD);
        }
    }

    public void setTargetPower(double power) {
        if (flywheel != null) {
            double clippedPower = Math.max(
                    -POWER_MAX,
                    Math.min(POWER_MAX, power));

            flywheel.setPower(clippedPower);
        }
    }

    public void spinDown() {
        if (flywheel != null) {
            flywheel.setPower(POWER_OFF);
        }
    }

    public void update() {
        // RUN_WITHOUT_ENCODER requires no periodic update.
    }

    public double getCurrentPower() {
        return flywheel != null ? flywheel.getPower() : 0.0;
    }

    public double getCurrentVelocityRPM() {
        if (flywheel != null) {
            // Assumes 28 encoder ticks per output revolution.
            return flywheel.getVelocity() * 60.0 / 28.0;
        }

        return 0.0;
    }

    public boolean isSpinning() {
        return flywheel != null
                && Math.abs(flywheel.getVelocity()) > 1.0;
    }
}
