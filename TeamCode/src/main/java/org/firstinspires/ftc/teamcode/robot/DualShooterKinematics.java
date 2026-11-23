package org.firstinspires.ftc.teamcode.robot;

public class DualShooterKinematics {
    // Gravitational constant in inches/second^2 (32.174 ft/s^2 * 12)
    public static final double GRAVITY_IN_PER_SEC2 = 386.09;

    // --- Heights & Launch Angle ---
    // DeltaY: (6.062177 + 10.7275 + 42.7725) - 12.73 = 46.832177 inches
    public static final double TARGET_HEIGHT_INCHES = 6.062177 + 10.7275 + 42.7725; // 59.562177 in
    public static final double SHOOTER_HEIGHT_INCHES = 12.73;
    public static final double DELTA_Y_INCHES = TARGET_HEIGHT_INCHES - SHOOTER_HEIGHT_INCHES; // 46.832177 in

    public static final double LAUNCH_ANGLE_DEG = 65.0; // 65 degrees from horizontal

    // --- Flywheel Geometry ---
    // 72mm converted to inches = 72 / 25.4 = 2.83464567 inches
    public static final double WHEEL_DIAMETER_INCHES = 72.0 / 25.4;
    public static final double WHEEL_RADIUS_INCHES = WHEEL_DIAMETER_INCHES / 2.0; // ~1.4173 in

    // Dual-wheel shooters have minimal slip due to dual contact traction (typically 0.82 - 0.90)
    public static double SLIP_FACTOR = 0.85;

    /**
     * Trajectory Formula:
     * Vball = (x / cos(theta)) * sqrt( g / (2 * (x * tan(theta) - DeltaY)) )
     *
     * @param horizontalDistanceInches x: distance from shooter exit to target opening
     * @return Ball exit speed in inches/second
     */
    public static double calculateExitVelocity(double horizontalDistanceInches) {
        double thetaRad = Math.toRadians(LAUNCH_ANGLE_DEG);

        // Denominator: 2 * (x * tan(theta) - deltaY)
        double denom = 2.0 * (horizontalDistanceInches * Math.tan(thetaRad) - DELTA_Y_INCHES);

        // Denominator <= 0 means apex cannot physically reach target height
        if (denom <= 0.0) {
            return 0.0;
        }

        double insideSqrt = GRAVITY_IN_PER_SEC2 / denom;
        return (horizontalDistanceInches / Math.cos(thetaRad)) * Math.sqrt(insideSqrt);
    }

    /**
     * Converts ball linear velocity (in/s) to flywheel RPM.
     * With 2 powered wheels, linear ball speed closely tracks wheel surface speed:
     * v_surface = (2 * PI * r) * (RPM / 60)
     * v_ball = v_surface * SLIP_FACTOR
     */
    public static double velocityToRPM(double vBallInchesPerSec) {
        if (vBallInchesPerSec <= 0.0) return 0.0;

        double circumferenceInches = 2.0 * Math.PI * WHEEL_RADIUS_INCHES;
        double wheelSurfaceSpeedNeeded = vBallInchesPerSec / SLIP_FACTOR;

        // RPM = (surface_speed * 60) / circumference
        return (wheelSurfaceSpeedNeeded * 60.0) / circumferenceInches;
    }

    /**
     * Direct conversion: Distance (inches) -> Motor Target RPM
     */
    public static double getTargetRPM(double distanceInches) {
        double vBall = calculateExitVelocity(distanceInches);
        return velocityToRPM(vBall);
    }
}