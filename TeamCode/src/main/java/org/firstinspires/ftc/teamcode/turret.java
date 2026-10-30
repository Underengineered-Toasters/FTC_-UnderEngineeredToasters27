package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Controller for 360-Degree Turret Motor ("turret") and Magnetic Limit Switch ("mag").
 * Rotates the shooter mechanism left and right using D-Pad controls.
 */
public class turret {

    private DcMotor turretMotor;
    private TouchSensor limitSwitch;

    public static final double TURRET_SPEED = 0.40;

    public void init(HardwareMap hardwareMap) {
        try {
            turretMotor = hardwareMap.get(DcMotor.class, "turret");
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            turretMotor = null;
        }

        try {
            limitSwitch = hardwareMap.tryGet(TouchSensor.class, "mag");
        } catch (Exception e) {
            limitSwitch = null;
        }

        stopTurret();
    }

    public void rotateLeft() {
        setPower(-TURRET_SPEED);
    }

    public void rotateRight() {
        setPower(TURRET_SPEED);
    }

    public void setPower(double power) {
        if (turretMotor != null) {
            turretMotor.setPower(power);
        }
    }

    public void stopTurret() {
        setPower(0.0);
    }

    public boolean isLimitReached() {
        return limitSwitch != null && limitSwitch.isPressed();
    }

    public double getPower() {
        return turretMotor != null ? turretMotor.getPower() : 0.0;
    }
}
