package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * BLUE ALLIANCE AUTONOMOUS PATH (High-Speed Performance)
 * 
 * Tuning Adjustments:
 * - Increased driving velocity across all steps (0.75 - 1.00 power).
 * - Calibrated turn duration for snappy 90° rotations.
 */
@Autonomous(name = "Blue Alliance Autonomous", group = "Blue")
public class BlueAutonomous extends LinearOpMode {

    private final Robot robot = new Robot();
    private final ElapsedTime runtime = new ElapsedTime();

    // Calibration: Estimated driving speed in inches per second at 0.5 power
    public static final double ESTIMATED_INCHES_PER_SECOND = 28.0;

    @Override
    public void runOpMode() {

        // Initialize drive motors and vision
        robot.init(hardwareMap);

        telemetry.addLine("=================================");
        telemetry.addLine("  BLUE ALLIANCE AUTONOMOUS READY ");
        telemetry.addLine("  Mode: High Speed Performance    ");
        telemetry.addLine("=================================");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // =========================================================================
        // BLUE ALLIANCE AUTONOMOUS7 PATH
        // =========================================================================

        // STEP 1: Drive forward 20 inches into Blue Hive Bucket opening (Higher speed)
        telemetry.addLine("Shooting...");
        telemetry.update();
        sleep(3000);

        // PAUSE AFTER STEP 1: Wait 1 second
        telemetry.addLine("Flower Intake");
        telemetry.update();
        driveInches(30, 0.70);
        sleep(500);
        turnRight(0.42, 0.56);
        sleep(500);
        driveInches(8, 0.5);
        sleep(500);
        turnRight(0.7, 0.47);
        driveInches(18, 0.75);
        sleep(2000);

        // STEP 2: Back up 12 inches & Turn LEFT 90° toward Blue Floor Sample Area
        telemetry.addLine("Intaking from flower");
        telemetry.update();
        driveInches(-15, 0.75);
        sleep(500);
        turnLeft(0.5, 0.7);
        driveInches(15, 0.7);
        sleep(500);
        turnLeft(0.4, 0.45);
        sleep(500);
        driveInches(50, 0.7);


        telemetry.addLine("test...");
        telemetry.update();
        turnLeft(0.4, 0.4);
        driveInches(27, 0.6);
        turnRight(0.4, 0.50);
        driveInches(8, 0.2);
        sleep(3000);


        telemetry.addLine("Park");
        telemetry.update();
        driveInches(-8, 0.3);
        turnRight(0.5, 0.7);
        driveInches(20, 0.5);
        turnRight(0., 0.50);
        driveInches(50, 0.7);





        stopRobot();

        telemetry.addLine("=================================");
        telemetry.addLine("  BLUE ALLIANCE AUTO COMPLETED!  ");
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
     * Turns Left (Spins Left: positive turn input).
     */
    private void turnRight(double power, double seconds) {
        driveForSeconds(0.0, power, seconds);
    }

    /**
     * Turns Right (Spins Right: negative turn input).
     */
    private void turnLeft(double power, double seconds) {
        driveForSeconds(0.0, -power, seconds);
    }

    /**
     * Drives the robot with given forward and turn powers for exact duration in seconds.
     */
    private void driveForSeconds(double forward, double turn, double seconds) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {
            double elapsedTime = runtime.seconds();
            robot.drive(forward, turn);
            telemetry.addData("Executing Step", "Time: %.2fs / %.2fs", elapsedTime, seconds);
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
