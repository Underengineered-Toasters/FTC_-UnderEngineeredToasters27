package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Turret elevation (the up/down tilt axis) driven by one encoder motor and positioned with a
 * {@link PIDController}.
 *
 * <p>The controller owns the motor power, so the hub must not run its own position or velocity
 * loop on top: the motor is left in {@code RUN_WITHOUT_ENCODER} and {@link #update()} writes power
 * directly.
 *
 * <p>Elevation is <b>not</b> a wrapping axis, so the controller deliberately does not use
 * continuous input. The angle error is a plain subtraction, and the software limits below stop the
 * mechanism being driven past its mechanical range.
 *
 * <p>Gravity is the load here. Two consequences:
 * <ul>
 *   <li>{@code BRAKE} zero-power behaviour, so the turret holds its angle at zero power instead of
 *       sagging while the loop fights gravity;</li>
 *   <li>a constant feed-forward ({@code kF}) is usually needed on top of PID, because a pure P term
 *       has to hold a steady error to hold a steady angle. See {@link #setFeedForward(double)}.</li>
 * </ul>
 *
 * <p>One instance owns one loop. Construct it, call {@link #init(HardwareMap)} once at INIT, then
 * call {@link #update()} once per OpMode loop iteration.
 */
public class TurretElevation {

    /** Must match the device name in the robot configuration. */
    public static final String MOTOR_NAME = "turretElevation";

    /**
     * Encoder ticks per degree of elevation.
     *
     * <p><b>PLACEHOLDER — this value is not measured.</b> Every angle in this class is wrong until
     * it is real. To measure it: reset the encoder at a known mechanical stop, drive the turret a
     * known angle (90 degrees is easiest to eyeball against a square), read the tick delta, and
     * divide. Set the result here or at runtime with {@link #setTicksPerDegree(double)}.
     */
    public static final double TICKS_PER_DEGREE = 1.0;

    /** Software travel limits, in degrees, applied to any target. */
    public static final double MIN_ANGLE_DEGREES = 0.0;
    public static final double MAX_ANGLE_DEGREES = 90.0;

    /**
     * Arrival window, in degrees. Required: {@link PIDController#atSetpoint(double, double)}
     * deliberately returns false until a tolerance is configured, so without this
     * {@link #atTarget()} can never become true and any loop waiting on it hangs forever.
     */
    public static final double TOLERANCE_DEGREES = 1.5;

    private DcMotorEx motor;
    private boolean available;

    private double ticksPerDegree = TICKS_PER_DEGREE;

    private double targetAngleDegrees = MIN_ANGLE_DEGREES;
    private double currentAngleDegrees = 0.0;
    private double lastOutput = 0.0;

    /**
     * Gains are starting points only — they cannot be tuned off the robot. The P term dominates;
     * I is small and clamped; D damps the approach. Tune with the mechanism actually loaded.
     *
     * <p>kF is the constant gravity feed-forward: a pure P term must hold a steady error to hold a
     * steady angle, so this axis usually needs some. Left at 0 until it is measured.
     */
    private double kP = 0.02;
    private double kI = 0.0;
    private double kD = 0.001;
    private double kF = 0.0;

    private final PIDController pid = new PIDController(kP, kI, kD, kF);

    public void init(HardwareMap hardwareMap) {
        // tryGet, not get: a missing device name must report, not throw at INIT.
        motor = hardwareMap.tryGet(DcMotorEx.class, MOTOR_NAME);
        available = (motor != null);
        if (!available) {
            return;
        }

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Gravity-loaded axis: BRAKE holds the angle at zero power.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0.0);

        pid.setTolerance(TOLERANCE_DEGREES);
        currentAngleDegrees = readAngleDegrees();
        targetAngleDegrees = currentAngleDegrees;
    }

    /** True when the motor named {@link #MOTOR_NAME} exists in the robot configuration. */
    public boolean isAvailable() {
        return available;
    }

    /** Override the measured ticks-per-degree without a redeploy. */
    public void setTicksPerDegree(double ticksPerDegree) {
        this.ticksPerDegree = ticksPerDegree;
    }

    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        pid.setPIDF(kP, kI, kD, kF);
    }

    /** Constant feed-forward added to every output, to offset gravity on this axis. */
    public void setFeedForward(double kF) {
        this.kF = kF;
        pid.setPIDF(kP, kI, kD, kF);
    }

    /**
     * Ask for an angle in degrees. The request is clamped to the software travel limits and the
     * controller is reset, so a new move starts without integral carried over from the last one.
     */
    public void setTargetAngle(double degrees) {
        targetAngleDegrees = Math.max(MIN_ANGLE_DEGREES, Math.min(MAX_ANGLE_DEGREES, degrees));
        pid.reset();
    }

    public double getTargetAngleDegrees() {
        return targetAngleDegrees;
    }

    public double getAngleDegrees() {
        return currentAngleDegrees;
    }

    public double getErrorDegrees() {
        return targetAngleDegrees - currentAngleDegrees;
    }

    public double getLastOutput() {
        return lastOutput;
    }

    public double getIntegralSum() {
        return pid.getIntegralSum();
    }

    /** True when the measured angle is inside {@link #TOLERANCE_DEGREES} of the target. */
    public boolean atTarget() {
        if (!available) {
            return false;
        }
        return pid.atSetpoint(targetAngleDegrees, currentAngleDegrees);
    }

    /**
     * Run one iteration of the loop. Call once per OpMode loop with the real loop period.
     *
     * @param dtSeconds seconds since the previous call; pass the measured period, not a guess.
     */
    public void update(double dtSeconds) {
        if (!available) {
            return;
        }
        currentAngleDegrees = readAngleDegrees();
        lastOutput = pid.calculate(targetAngleDegrees, currentAngleDegrees, dtSeconds);
        motor.setPower(lastOutput);
    }

    /** Drop to zero power. Call on every exit path. */
    public void stop() {
        if (available) {
            motor.setPower(0.0);
        }
        lastOutput = 0.0;
        pid.reset();
    }

    private double readAngleDegrees() {
        return motor.getCurrentPosition() / ticksPerDegree;
    }
}
