package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Single timed right-turn calibration with measured IMU angle telemetry.
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

    // Calibration: Estimated driving speed in inches per second at 0.5 power
    public static final double ESTIMATED_INCHES_PER_SECOND = 20.0;
    // Starting estimate from the previous right turn; calibrate both directions.
    public static final double TURN_POWER = 0.565;
    public static final double ESTIMATED_SECONDS_PER_90_DEGREES = 0.60;

    @Override
    public void runOpMode() {

        // Initialize drive motors and vision
        robot.init(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        if (!imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)))) {
            telemetry.addLine("IMU initialization failed; turn test cancelled.");
            telemetry.update();
            robot.close();
            return;
        }

        telemetry.addLine("Basic Out and Back Auto");
        telemetry.addLine("Right-turn angle test; yaw resets at START.");
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
        turnRight(TURN_POWER, ESTIMATED_SECONDS_PER_90_DEGREES);
        /*driveInches(12, 1);
        turnLeft(TURN_POWER, ESTIMATED_SECONDS_PER_90_DEGREES);
        driveInches(20, 1);
        turnRight(TURN_POWER, ESTIMATED_SECONDS_PER_90_DEGREES);
        if (!opModeIsActive()) {
            return;
        }
        sleep(2000);
        turnLeft(TURN_POWER, ESTIMATED_SECONDS_PER_90_DEGREES);
        driveInches(46,1);
        sleep(1000);
        turnRight(TURN_POWER, 2*ESTIMATED_SECONDS_PER_90_DEGREES);

        // A half-turn retraces the 46-inch leg; turn right to retrace the 7-inch leg.
        // The final half-turn restores the starting heading.
        driveInches(30, 1);
        turnLeft(TURN_POWER, ESTIMATED_SECONDS_PER_90_DEGREES);
        driveInches(80, 1);
        turnLeft(TURN_POWER, 2 * ESTIMATED_SECONDS_PER_90_DEGREES);
        sleep(2000);
        turnLeft(TURN_POWER, 1.6 * ESTIMATED_SECONDS_PER_90_DEGREES);
        driveInches(-80, 0.5);
        turnLeft(TURN_POWER, 2 * ESTIMATED_SECONDS_PER_90_DEGREES);
        driveInches(-7, 0.3);*/
        stopRobot();

        // Continue measuring after motor stop so coasting/overshoot is visible.
        while (opModeIsActive()) {
            telemetry.addLine("Turn complete. Motors stopped. Press STOP when finished.");
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

        driveForSeconds(motorPower, 0.0, seconds);
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
