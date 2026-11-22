package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Controller for 360-Degree Turret Motor ("turret") and Magnetic Limit Switch ("mag").
 * Rotates the shooter mechanism left and right using D-Pad controls.
 * Tracks position via encoder ticks to hold static initial position.
 */
public class turret {

    private DcMotor turretMotor;
    private TouchSensor limitSwitch;

    private double lastNonZeroPower = 0.0;

    public static final double TURRET_SPEED = 0.40;

    /** Power used by {@link #holdPosition} when the turret is far from its target. */
    public static final double HOLD_POWER = 0.30;

    /** Encoder deadband for the static hold; ~40 ticks is a small fraction of a degree. */
    public static final int HOLD_TOLERANCE_TICKS = 40;

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
        setPower(-TURRET_SPEED);
    }

    public void rotateRight() {
        setPower(TURRET_SPEED);
    }

    /**
     * Commands the turret with limit-switch protection.
     *
     * When the magnetic limit is active, power that continues in the same direction as the last
     * commanded movement is refused: the turret may always reverse out of the stop, but it can
     * never keep pushing into it. Every caller (D-pad, static hold, autonomous) inherits this.
     */
    public void setPower(double power) {
        if (turretMotor == null) {
            return;
        }

        if (power != 0.0) {
            boolean pushingIntoStop = limitSwitch != null
                    && limitSwitch.isPressed()
                    && lastNonZeroPower != 0.0
                    && Math.signum(power) == Math.signum(lastNonZeroPower);
            if (pushingIntoStop) {
                power = 0.0;
            } else {
                lastNonZeroPower = power;
            }
        }

        turretMotor.setPower(power);
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

    /**
     * Drives the turret toward {@code targetTicks} and returns true once it is within
     * {@code toleranceTicks}. Encoder zero is the turret position at init, i.e. where the robot
     * was built to shoot downfield, so callers aim by returning there.
     *
     * Call once per OpMode loop iteration.
     */
    public boolean holdPosition(int targetTicks, int toleranceTicks, double power) {
        int tolerance = Math.max(1, Math.abs(toleranceTicks));
        int error = targetTicks - getCurrentPosition();

        if (Math.abs(error) <= tolerance) {
            stopTurret();
            return true;
        }

        double ramp = Math.min(1.0, Math.abs((double) error) / (4.0 * tolerance));
        setPower(Math.copySign(power * ramp, error));
        return false;
    }

    /** Convenience call for the init heading (encoder zero). */
    public boolean holdInitPosition() {
        return holdPosition(0, HOLD_TOLERANCE_TICKS, HOLD_POWER);
    }
}
