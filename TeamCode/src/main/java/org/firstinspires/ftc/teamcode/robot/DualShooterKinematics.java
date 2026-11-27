package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.messages.KinematicsModel;

public class DualShooterKinematics {
    // Gravitational constant in inches/second^2 (32.174 ft/s^2 * 12)
    public static final double GRAVITY_IN_PER_SEC2 = 386.089;
    // --- Heights & Launch Angle ---
    // DeltaY: (6.062177 + 10.7275 + 42.7725) - 12.73 = 46.832177 inches
    public static final double TARGET_HIVE_HEIGHT_INCHES = 6.062177 + 10.7275 + 42.7725; // 59.562177 in
    public static final double SHOOTER_HEIGHT_INCHES = 12.00;

    public static final double LAUNCH_ANGLE_DEG = 30.0;

    // --- Flywheel Geometry ---
    // 72mm converted to inches = 72 / 25.4 = 2.83464567 inches
    public static final double WHEEL_DIAMETER_INCHES = 72.0 / 25.4;
    public static final double WHEEL_RADIUS_INCHES = WHEEL_DIAMETER_INCHES / 2.0; // ~1.4173 in

    // PLACEHOLDER: calibrate against measured shots; wheel slip depends on compression and game piece.
    public static double SLIP_FACTOR = 1.0;

    public static final Vector2d BLUE_HIVE_1_POSITION = new Vector2d(83.5, 55.0);
    public static final Vector2d BLUE_HIVE_2_POSITION = new Vector2d(83.5, 86.5);
    public static final Vector2d RED_HIVE_1_POSITION = new Vector2d(83.5, 86.5);
    public static final Vector2d RED_HIVE_2_POSITION = new Vector2d(83.5, 86.5);

    private DualShooterKinematics() {
    }

    /**
     * Trajectory Formula:
     * Vball = (x / cos(theta)) * sqrt( g / (2 * (x * tan(theta) - DeltaY)) )
     *
     * @param horizontalDistanceInches x: distance from shooter exit to target opening
     * @return Ball exit speed in inches/second
     */
    public static double calculateExitVelocity(double horizontalDistanceInches, double deltaYInInches, double launchAngleInDegrees) {
        return calculateExitVelocity(horizontalDistanceInches, deltaYInInches,
                launchAngleInDegrees, GRAVITY_IN_PER_SEC2);
    }

    private static double calculateExitVelocity(double horizontalDistanceInches, double deltaYInInches,
                                                double launchAngleInDegrees, double gravity) {
        double thetaRad = Math.toRadians(launchAngleInDegrees);

        // Denominator: 2 * (x * tan(theta) - deltaY)
        double denom = 2.0 * (horizontalDistanceInches * Math.tan(thetaRad) - deltaYInInches);

        // Denominator <= 0 means apex cannot physically reach target height
        if (denom <= 0.0) {
            return 0.0;
        }

        double insideSqrt = gravity / denom;
        return (horizontalDistanceInches / Math.cos(thetaRad)) * Math.sqrt(insideSqrt);
    }

    /**
     * Converts ball linear velocity (in/s) to flywheel RPM.
     * With 2 powered wheels, linear ball speed closely tracks wheel surface speed:
     * v_surface = (2 * PI * r) * (RPM / 60)
     * v_ball = v_surface * SLIP_FACTOR
     */
    public static double velocityToRPM(double vBallInchesPerSec) {
        return velocityToRPM(vBallInchesPerSec, SLIP_FACTOR);
    }

    private static double velocityToRPM(double vBallInchesPerSec, double slipFactor) {
        if (vBallInchesPerSec <= 0.0) return 0.0;

        double circumferenceInches = 2.0 * Math.PI * WHEEL_RADIUS_INCHES;
        double wheelSurfaceSpeedNeeded = vBallInchesPerSec / slipFactor;

        // RPM = (surface_speed * 60) / circumference
        return (wheelSurfaceSpeedNeeded * 60.0) / circumferenceInches;
    }


    public static double getTargetRPM(KinematicsModel kinematicsModel) {
        return getTargetRPM(kinematicsModel, GRAVITY_IN_PER_SEC2, SLIP_FACTOR);
    }

    static double getTargetRPM(KinematicsModel kinematicsModel, double gravity, double slipFactor) {
        double heightDifference = kinematicsModel.getTargetHiveHeightInInches()
                - kinematicsModel.getShooterHeightInInches();
        double ballSpeed = calculateExitVelocity(kinematicsModel.getHorizontalDistanceInches(),
                heightDifference, kinematicsModel.getLaunchAngleInDegree(), gravity);
        return velocityToRPM(ballSpeed, slipFactor);
    }

    public static double getTargetRPM(double horizontalDistanceInches, double deltaYInInches, double launchAngleInDegrees) {
        double vBall = calculateExitVelocity(horizontalDistanceInches, deltaYInInches, launchAngleInDegrees);
        return velocityToRPM(vBall);
    }
}