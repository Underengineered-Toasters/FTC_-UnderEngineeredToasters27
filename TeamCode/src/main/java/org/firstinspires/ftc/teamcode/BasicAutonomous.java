package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Basic Autonomous Template for FTC.
 * Includes a hardcoded motion sequence and commented helper methods for:
 * - Driving Forward (driveForward)
 * - Driving Backward (driveBackward)
 * - Turning Left (turnLeft)
 * - Turning Right (turnRight)
 * - Adjusting Motor Power & Speed (setMotorPowers)
 * - Stopping the Robot (stopRobot)
 */
@Autonomous(name = "Basic Auto - Hardcoded Sequence", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {

    // Drive Motors (HardwareMap names must match Driver Station configuration)
    private DcMotor left;
    private DcMotor right;

    // Timer for time-based autonomous movements
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ===============================================
        // 1. HARDWARE INITIALIZATION
        // ===============================================
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        // Reverse left motor so positive power drives both wheels forward
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        // Set motors to brake mode so the robot stops quickly when power is 0
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized - Ready for Start");
        telemetry.update();

        // Wait for the driver to press PLAY on the Driver Station
        waitForStart();
        runtime.reset();

        // ===============================================
        // 2. HARDCODED AUTONOMOUS SEQUENCE
        // ===============================================

        // STEP 1: Move Forward for 1.5 seconds at 50% power
        telemetry.addLine("Step 1: Driving Forward...");
        telemetry.update();
        driveForward(0.5, 1.5);
        // STEP 2: Turn Right for 0.8 seconds at 40% power
        telemetry.addLine("Step 2: Turning Right...");
        telemetry.update();
        turnRight(0.4, 0.8);

        // STEP 3: Move Forward again for 1.0 seconds at 60% power (Adjusting Power/Speed)
        telemetry.addLine("Step 3: Driving Forward (Higher Power)...");
        telemetry.update();
        driveForward(0.6, 1.0);

        // STEP 4: Turn Left for 0.8 seconds at 40% power
        telemetry.addLine("Step 4: Turning Left...");
        telemetry.update();
        turnLeft(0.4, 0.8);

        // STEP 5: Drive Backward for 1.0 seconds at 40% power
        telemetry.addLine("Step 5: Driving Backward...");
        telemetry.update();
        driveBackward(0.4, 1.0);

        // STEP 6: Stop the Robot
        telemetry.addLine("Step 6: Stopping Robot.");
        telemetry.update();
        stopRobot();

        telemetry.addData("Status", "Autonomous Sequence Complete!");
        telemetry.update();
    }

    // =========================================================================
    // HELPER COMMANDS FOR AUTONOMOUS MOVEMENTS
    // Use these methods to build custom autonomous paths.
    // =========================================================================

    /**
     * COMMAND: MOVE FORWARD
     * Both motors move forward at the specified power for a set duration.
     * 
     * @param power Speed level from 0.0 to 1.0 (e.g., 0.5 = 50% max speed)
     * @param seconds Duration in seconds to execute the movement
     */
    private void driveForward(double power, double seconds) {
        setMotorPowers(power, power, seconds);
    }

    /**
     * COMMAND: MOVE BACKWARD
     * Both motors move backward (negative power) for a set duration.
     * 
     * @param power Speed level from 0.0 to 1.0
     * @param seconds Duration in seconds
     */
    private void driveBackward(double power, double seconds) {
        setMotorPowers(-power, -power, seconds);
    }

    /**
     * COMMAND: MOVE / TURN LEFT
     * Left motor moves backward, Right motor moves forward to spin left in place.
     * 
     * @param power Turn speed level from 0.0 to 1.0
     * @param seconds Duration in seconds
     */
    private void turnLeft(double power, double seconds) {
        setMotorPowers(-power, power, seconds);
    }

    /**
     * COMMAND: MOVE / TURN RIGHT
     * Left motor moves forward, Right motor moves backward to spin right in place.
     * 
     * @param power Turn speed level from 0.0 to 1.0
     * @param seconds Duration in seconds
     */
    private void turnRight(double power, double seconds) {
        setMotorPowers(power, -power, seconds);
    }

    /**
     * COMMAND: ADJUST POWER / CUSTOM DIFFERENTIAL POWER
     * Set specific left and right motor powers for custom speeds or curved turns.
     * 
     * Examples:
     * - setMotorPowers(0.3, 0.6, 2.0); // Curve left (right motor faster)
     * - setMotorPowers(0.8, 0.8, 1.0); // High speed forward sprint
     * - setMotorPowers(0.2, 0.2, 3.0); // Low speed precision crawl
     * 
     * @param leftPower Power for left drive motor (-1.0 to 1.0)
     * @param rightPower Power for right drive motor (-1.0 to 1.0)
     * @param seconds Duration to run before stopping
     */
    private void setMotorPowers(double leftPower, double rightPower, double seconds) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {
            left.setPower(leftPower);
            right.setPower(rightPower);

            telemetry.addData("Time Elapsed", "%.1f / %.1f sec", runtime.seconds(), seconds);
            telemetry.addData("Left Motor Power", "%.2f", leftPower);
            telemetry.addData("Right Motor Power", "%.2f", rightPower);
            telemetry.update();
        }
        stopRobot();
    }

    /**
     * COMMAND: STOP ROBOT
     * Sets power on all motors to 0.0.
     */
    private void stopRobot() {
        left.setPower(0.0);
        right.setPower(0.0);
    }
}
