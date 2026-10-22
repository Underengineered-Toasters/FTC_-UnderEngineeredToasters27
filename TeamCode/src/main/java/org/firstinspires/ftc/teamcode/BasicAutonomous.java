package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Timed out-and-back route with a tuned turnaround duration.
 * Distances and turns are estimates and require calibration on the robot.
 */
@Autonomous(name = "Basic Out and Back Auto", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {

    private final Robot robot = new Robot();
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

        telemetry.addLine("Basic Out and Back Auto");
        telemetry.addLine("Forward 6 in, left 90, forward 36 in, wait 2 sec, turn 195, return");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }
        runtime.reset();

        driveInches(6, 0.3);
        turnLeft(TURN_POWER, ESTIMATED_SECONDS_PER_90_DEGREES);
        driveInches(36, 0.5);
        if (!opModeIsActive()) {
            return;
        }
        sleep(2000);
        turnLeft(TURN_POWER, (195.0 / 90.0) * ESTIMATED_SECONDS_PER_90_DEGREES);

        // Return legs; endpoint accuracy depends on turnaround calibration.
        driveInches(36, 0.5);
        turnRight(TURN_POWER, ESTIMATED_SECONDS_PER_90_DEGREES);
        driveInches(6, 0.3);
        turnLeft(TURN_POWER, 2 * ESTIMATED_SECONDS_PER_90_DEGREES);
        stopRobot();

        if (opModeIsActive()) {
            telemetry.addLine("Out and back route completed");
            telemetry.addData("Total Time Elapsed", "%.2f sec", runtime.seconds());
            telemetry.update();
        }
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
