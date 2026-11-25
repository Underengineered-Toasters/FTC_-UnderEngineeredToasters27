package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Controller for 360-Degree Turret Motor ("turret") and Magnetic Limit Switch ("mag").
 * Rotates the shooter mechanism left and right using D-Pad controls.
 * Tracks position via encoder ticks and supports rotateTo(deg).
 */
public class turret {

    private DcMotor turretMotor;
    private TouchSensor limitSwitch;

    public static final double TURRET_SPEED = 0.40;

    public void init(HardwareMap hardwareMap) {
        try {
            turretMotor = hardwareMap.get(DcMotor.class, "turret");
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        if (turretMotor != null && turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        setPower(-TURRET_SPEED);
    }

    public void rotateRight() {
        if (turretMotor != null && turretMotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        setPower(TURRET_SPEED);
    }

    public void rotateTo(double deg) {
        if (turretMotor != null) {
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setTargetPosition((int) (deg * 110.0 / 20.0 / 180.0));
            turretMotor.setPower(0.30);
        }
    }

    public void setPower(double power) {
        if (turretMotor != null) {
            turretMotor.setPower(power);
        }
    }

    public void stopTurret() {
        setPower(0.0);
    }

    public int getCurrentPosition() {
        return turretMotor != null ? turretMotor.getCurrentPosition() : 0;
    }

    public boolean isLimitReached() {
        return limitSwitch != null && limitSwitch.isPressed();
    }

    public double getPower() {
        return turretMotor != null ? turretMotor.getPower() : 0.0;
    }
}
