package org.firstinspires.ftc.teamcode;

/**
 * Generic PID(F) controller for FTC mechanisms.
 *
 * <p>One instance drives one control loop. Call {@link #calculate(double, double, double)} once
 * per OpMode loop iteration (or control-loop tick) with the current target, the current measured
 * value, and the loop period in seconds.
 *
 * <p>Typical usage in an OpMode loop:
 * <pre>{@code
 * private final PIDController heading = new PIDController(0.02, 0.0, 0.001);
 *
 * // in init()
 * heading.setContinuousInput(-180, 180);   // heading wraps around
 * heading.setTolerance(2.0);               // degrees
 *
 * // in the loop
 * double turn = heading.calculate(targetHeading, imu.getYaw(), loopTime.seconds());
 * robot.drive(drivePower, turn);
 * if (heading.atSetpoint(targetHeading, imu.getYaw())) { ... }
 * }</pre>
 *
 * <p>Notes:
 * <ul>
 *   <li>The derivative term is computed from the measurement, not from the error, so changing the
 *       target inside a move does not produce a derivative "kick".</li>
 *   <li>Output is clamped to [-1, 1] by default, i.e. it is a motor <em>power</em> controller.
 *       Use {@link #setOutputRange(double, double)} for other ranges.</li>
 *   <li>Turn on {@link #setContinuousInput(double, double)} for angles so the controller takes the
 *       short way around (target 179 with current -179 must not be a 358 degree error).</li>
 *   <li>Call {@link #reset()} at the start of every independent move.</li>
 * </ul>
 */
public class PIDController {

    private double kP;
    private double kI;
    private double kD;
    private double kF;

    /** Anti-windup clamp on the accumulated integral (in error*seconds). */
    private double maxIntegral = 0.5;

    private double integralSum = 0.0;
    private double lastError = 0.0;

    // Derivative-on-measurement state
    private double lastMeasurement = 0.0;
    private boolean hasLastMeasurement = false;

    private double minOutput = -1.0;
    private double maxOutput = 1.0;

    /** Output units of tolerance; NaN means "not configured, atSetpoint() returns false". */
    private double tolerance = Double.NaN;

    private boolean continuousInput = false;
    private double inputMin = -180.0;
    private double inputMax = 180.0;

    private double lastOutput = 0.0;

    /** Assumed loop period (seconds) when a non-positive dt is passed in. */
    private static final double DEFAULT_DT = 0.02;

    public PIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0.0);
    }

    /**
     * @param kF constant feed-forward added to every output (useful for gravity/elevator loads and
     *           for flywheel-style mechanisms). Pass 0 for plain PID.
     */
    public PIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * Run one iteration of the control loop.
     *
     * @param target  desired value (setpoint)
     * @param current measured value
     * @param dt      seconds since the previous call; if <= 0 a default of 0.02 s is used
     * @return controller output, clamped to the configured output range
     */
    public double calculate(double target, double current, double dt) {
        if (dt <= 0) dt = DEFAULT_DT;

        double error = wrapError(target - current);

        double pTerm = kP * error;

        // Integrate, but never let the accumulator grow past maxIntegral.
        integralSum += error * dt;
        integralSum = Math.max(-maxIntegral, Math.min(maxIntegral, integralSum));
        double iTerm = kI * integralSum;

        // Derivative on measurement: d(error)/dt == -d(measurement)/dt for a constant target,
        // so this keeps the sign convention while removing setpoint kick.
        double derivative = 0.0;
        if (hasLastMeasurement) {
            derivative = -(current - lastMeasurement) / dt;
        }
        double dTerm = kD * derivative;

        lastMeasurement = current;
        hasLastMeasurement = true;
        lastError = error;

        double output = pTerm + iTerm + dTerm + kF;
        output = Math.max(minOutput, Math.min(maxOutput, output));
        lastOutput = output;
        return output;
    }

    /**
     * @return true when the error is inside the configured tolerance. Returns false when no
     *         tolerance has been set, so an untuned loop never reports "arrived".
     */
    public boolean atSetpoint(double target, double current) {
        if (Double.isNaN(tolerance)) return false;
        return Math.abs(wrapError(target - current)) < tolerance;
    }

    /** Clear integral and derivative history. Call before starting an independent move. */
    public void reset() {
        integralSum = 0.0;
        lastError = 0.0;
        hasLastMeasurement = false;
        lastOutput = 0.0;
    }

    /** Wrap the raw error to the shortest path when continuous input is enabled. */
    private double wrapError(double error) {
        if (!continuousInput) return error;

        double range = inputMax - inputMin;
        if (range <= 0) return error;

        double wrapped = error - inputMin;
        wrapped = ((wrapped % range) + range) % range;  // Java % can return negative values
        return wrapped + inputMin;
    }

    // ------------------------------------------------------------------
    // Configuration
    // ------------------------------------------------------------------

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void setPIDF(double kP, double kI, double kD, double kF) {
        setPID(kP, kI, kD);
        this.kF = kF;
    }

    public void setMaxIntegral(double maxIntegral) {
        this.maxIntegral = Math.abs(maxIntegral);
    }

    public void setOutputRange(double minOutput, double maxOutput) {
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    public void setTolerance(double tolerance) {
        this.tolerance = Math.abs(tolerance);
    }

    /** Enable shortest-path error wrapping on the given input range (e.g. -180 to 180 degrees). */
    public void setContinuousInput(double inputMin, double inputMax) {
        this.continuousInput = true;
        this.inputMin = inputMin;
        this.inputMax = inputMax;
    }

    public void setContinuousInput(boolean continuousInput) {
        this.continuousInput = continuousInput;
    }

    // ------------------------------------------------------------------
    // Telemetry helpers
    // ------------------------------------------------------------------

    public double getLastOutput() {
        return lastOutput;
    }

    public double getLastError() {
        return lastError;
    }

    public double getIntegralSum() {
        return integralSum;
    }
}