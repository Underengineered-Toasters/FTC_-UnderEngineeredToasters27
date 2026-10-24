package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Straight-drive heading-hold test with live IMU angle telemetry.
 * Distances and turns are estimates and require calibration on the robot.
 */
@Autonomous(name = "Basic Out and Back Auto", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {

    private final Robot robot = new Robot();
    private IMU imu;
    private double previousYaw;
    private double turnAngle;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();

    // Initial heading-hold settings; tune on the robot.
    public static final double STRAIGHT_TEST_INCHES = 24.0;
    public static final double STRAIGHT_TEST_POWER = 0.35;
    public static final double HEADING_GAIN = 0.03;
    public static final double HEADING_RATE_GAIN = 0.002;
    public static final double MAX_HEADING_CORRECTION = 0.12;

    // Calibration: Estimated driving speed in inches per second at 0.5 power
    public static final double ESTIMATED_INCHES_PER_SECOND = 20.0;
    // Starting estimate from the previous right turn; calibrate both directions.
    public static final double TURN_POWER = 0.565;
    public static final double ESTIMATED_LEFT_SECONDS_PER_90_DEGREES = 0.60;
    public static final double ESTIMATED_RIGHT_SECONDS_PER_90_DEGREES = 0.56;

    @Override
    public void runOpMode() {

        // Initialize drive motors and vision
        robot.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        if (!imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)))) {
            telemetry.addLine("IMU initialization failed; autonomous cancelled.");
            telemetry.update();
            robot.close();
            return;
        }

        telemetry.addLine("Basic Out and Back Auto");
        telemetry.addLine("Straight test: forward 24 inches at 0.35 power; no planned turns.");
        telemetry.addLine("Hub mounting: logo UP, USB FORWARD.");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }
        imu.resetYaw();
        previousYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        turnAngle = 0.0;
        runtime.reset();
        driveInches(STRAIGHT_TEST_INCHES, STRAIGHT_TEST_POWER);

        // Full route retained for restoration after heading-hold calibration.
        /*
        driveInches(12, 0.5);
        turnLeft(TURN_POWER, ESTIMATED_LEFT_SECONDS_PER_90_DEGREES);
        driveInches(20, 0.5);
        turnRight(TURN_POWER, ESTIMATED_RIGHT_SECONDS_PER_90_DEGREES);
        if (!opModeIsActive()) {
            return;
        }
        sleep(2000);
        turnLeft(TURN_POWER, ESTIMATED_LEFT_SECONDS_PER_90_DEGREES);
        driveInches(30,0.5);
        sleep(1000);
        turnRight(TURN_POWER, 2 * ESTIMATED_RIGHT_SECONDS_PER_90_DEGREES);

        // A half-turn retraces the 46-inch leg; turn right to retrace the 7-inch leg.
        // The final half-turn restores the starting heading.
        driveInches(30, 0.5);
        turnLeft(TURN_POWER, ESTIMATED_LEFT_SECONDS_PER_90_DEGREES);
        driveInches(80, 0.5);
        turnLeft(TURN_POWER, 2 * ESTIMATED_LEFT_SECONDS_PER_90_DEGREES);
        sleep(2000);
        turnLeft(TURN_POWER, ESTIMATED_LEFT_SECONDS_PER_90_DEGREES);
        driveInches(-80, 0.5);
        turnLeft(TURN_POWER, 2 * ESTIMATED_LEFT_SECONDS_PER_90_DEGREES);
        driveInches(-7, 0.3);
        */
        stopRobot();

        // Continue measuring after motor stop so coasting/overshoot is visible.
        while (opModeIsActive()) {
            telemetry.addLine("Straight test complete. Motors stopped. Press STOP when finished.");
            addAngleTelemetry();
            telemetry.update();
            sleep(50);
        }
        robot.close();
    }

    private void addAngleTelemetry() {
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        turnAngle += AngleUnit.normalizeDegrees(yaw - previousYaw);
        previousYaw = yaw;
        telemetry.addData("Turn angle", "%.1f deg (+left / -right)", turnAngle);
        telemetry.addData("Angle magnitude", "%.1f deg", Math.abs(turnAngle));
    }

    // =========================================================================
    // TIME & POWER DRIVING HELPER METHODS
    // =========================================================================

    /**
     * Drives forward or backward based on target inches converted to seconds.
     */
    private void driveInches(double targetInches, double power) {
        double seconds = Math.abs(targetInches) / (ESTIMATED_INCHES_PER_SECOND * (Math.abs(power) / 0.5));
        double direction = targetInches >= 0 ? 1.0 : -1.0;
        double motorPower = Math.abs(power) * direction;

        if (targetInches == 0 || !opModeIsActive()) {
            stopRobot();
            return;
        }
        double targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        stepTimer.reset();
        try {
            while (opModeIsActive() && stepTimer.seconds() < seconds) {
                double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double error = AngleUnit.normalizeDegrees(targetHeading - currentHeading);
                double yawRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
                // Raw positive turn is physical left on this reversed-drive chassis.
                // Steering responds to heading error with the same sign in reverse.
                double correctionLimit = Math.min(MAX_HEADING_CORRECTION, Math.abs(motorPower));
                // Oppose angular velocity to reduce repeated overshoot of the target.
                double requestedCorrection = error * HEADING_GAIN - yawRate * HEADING_RATE_GAIN;
                double correction = Math.max(-correctionLimit,
                        Math.min(correctionLimit, requestedCorrection));
                double scale = Math.max(1.0, Math.abs(motorPower) + Math.abs(correction));
                robot.drive(-motorPower / scale, correction / scale);
                addAngleTelemetry();
                telemetry.addData("Target heading", "%.1f deg", targetHeading);
                telemetry.addData("Current heading", "%.1f deg", currentHeading);
                telemetry.addData("Heading error", "%.1f deg", error);
                telemetry.addData("Yaw rate", "%.1f deg/sec", yawRate);
                telemetry.addData("Steering correction", "%.3f", correction / scale);
                telemetry.addData("Drive powers", "Left %.3f / Right %.3f",
                        robot.getLeftPower(), robot.getRightPower());
                telemetry.addData("Drive time", "%.2f / %.2f sec", stepTimer.seconds(), seconds);
                telemetry.update();
                idle();
            }
        } finally {
            stopRobot();
        }
    }

    /**
     * Turns Left for a specified duration in seconds.
     */
    private void turnLeft(double power, double seconds) {
        driveForSeconds(0.0, -Math.abs(power), seconds);
    }

    /**
     * Turns Right for a specified duration in seconds.
     */
    private void turnRight(double power, double seconds) {
        driveForSeconds(0.0, Math.abs(power), seconds);
    }

    /**
     * Drives the robot with given forward and turn powers for exact duration in seconds.
     */
    private void driveForSeconds(double forward, double turn, double seconds) {
        stepTimer.reset();
        while (opModeIsActive() && stepTimer.seconds() < seconds) {
            // This chassis moves backward for positive drive power.
            // Reverse both axes so route distances and turn names stay physical.
            robot.drive(-forward, -turn);
            addAngleTelemetry();
            telemetry.addData("Executing Step", "Time: %.2fs / %.2fs", stepTimer.seconds(), seconds);
            telemetry.update();
        }
        stopRobot();
    }

    /**
     * Stop all drive motors.
     */
    private void stopRobot() {
        robot.drive(0.0, 0.0);
    }
}
