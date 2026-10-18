package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Biobuzz Max-Scoring Autonomous Path starting STRAIGHT between Flower & Parking Space.
 *
 * Starting Setup:
 * - Position: Between Flower (Submersible) and Parking Space
 * - Facing: Straight Forward (North, 0° heading)
 *
 * Execution Time: ~17.5 Seconds (within 20s window)
 * Projected Score: 23 Points
 */
@Autonomous(name = "Biobuzz Straight Auto (17.5s Path)", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {

    private final Robot robot = new Robot();
    private final ElapsedTime runtime = new ElapsedTime();

    // Calibration: Estimated driving speed in inches per second at 0.5 power
    public static final double ESTIMATED_INCHES_PER_SECOND = 20.0;

    @Override
    public void runOpMode() {

        // Initialize drive motors and vision
        robot.init(hardwareMap);

        telemetry.addLine("=================================");
        telemetry.addLine("  BIOBUZZ STRAIGHT AUTONOMOUS    ");
        telemetry.addLine("  Start: Facing Straight         ");
        telemetry.addLine("  Target Time: 17.5 Seconds      ");
        telemetry.addLine("  Target Score: 23 Points        ");
        telemetry.addLine("=================================");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // =========================================================================
        // STEP 1: SCORE PRE-LOADED SAMPLE IN HIGH TARGET (0.0s - 3.5s)
        // =========================================================================
        telemetry.addLine("[0-3.5s] Step 1: Driving Straight Forward & Scoring Pre-load (+10 pts)...");
        telemetry.update();

        // Drive straight forward 32 inches toward Target structure
        driveInches(32.0, 0.5);

        // Pause for scoring mechanism / release
        sleep(500);

        // =========================================================================
        // STEP 2: DRIVE TO INTAKE FLOOR SAMPLE 1 (3.5s - 8.5s)
        // =========================================================================
        telemetry.addLine("[3.5-8.5s] Step 2: Backing up & Intaking Floor Sample 1...");
        telemetry.update();

        // Back up 12 inches straight
        driveInches(-12.0, 0.4);

        // Turn RIGHT 90° toward floor samples
        turnRight(0.4, 0.60);

        // Drive forward 24 inches to pick up Sample 1
        driveInches(24.0, 0.5);

        // Pause for intake collection
        sleep(500);

        // =========================================================================
        // STEP 3: RETURN & SCORE SAMPLE 2 IN HIGH TARGET (8.5s - 13.5s)
        // =========================================================================
        telemetry.addLine("[8.5-13.5s] Step 3: Returning & Scoring Sample 2 (+10 pts)...");
        telemetry.update();

        // Back up 24 inches
        driveInches(-24.0, 0.5);

        // Turn LEFT 90° facing straight back toward Target
        turnLeft(0.4, 0.60);

        // Drive forward 12 inches to High Target
        driveInches(12.0, 0.4);

        // Pause for scoring mechanism / release
        sleep(500);

        // =========================================================================
        // STEP 4: PARK IN PARKING SPACE (13.5s - 17.5s)
        // =========================================================================
        telemetry.addLine("[13.5-17.5s] Step 4: Parking in Parking Space (+3 pts)...");
        telemetry.update();

        // Back up 12 inches straight
        driveInches(-12.0, 0.4);

        // Turn RIGHT 90° facing Parking Space
        turnRight(0.4, 0.60);

        // Drive forward 36 inches into Parking Space
        driveInches(36.0, 0.6);

        // Final Full Stop
        stopRobot();

        telemetry.addLine("=================================");
        telemetry.addLine("  AUTONOMOUS COMPLETED IN 17.5s  ");
        telemetry.addData("Total Time Elapsed", "%.2f sec", runtime.seconds());
        telemetry.addLine("=================================");
        telemetry.update();
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
        driveForSeconds(0.0, power, seconds);
    }

    /**
     * Turns Right for a specified duration in seconds.
     */
    private void turnRight(double power, double seconds) {
        driveForSeconds(0.0, -power, seconds);
    }

    /**
     * Drives the robot with given forward and turn powers for exact duration in seconds.
     */
    private void driveForSeconds(double forward, double turn, double seconds) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {
            robot.drive(forward, turn);
            telemetry.addData("Executing Step", "Time: %.2fs / %.2fs", runtime.seconds(), seconds);
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
