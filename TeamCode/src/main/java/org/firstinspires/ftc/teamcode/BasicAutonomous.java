package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * Flower to Hive Autonomous for FTC BIOBUZZ.
 * Routine: Starting position -> lightly bump Flower with intake running ->
 * navigate to Hive -> camera AprilTag align -> pause 3s (pretend shoot) -> park.
 * Helper methods:
 * - Driving Forward (driveForward)
 * - Driving Backward (driveBackward)
 * - Turning Left (turnLeft)
 * - Turning Right (turnRight)
 * - Custom Drive Input (driveFor)
 * - AprilTag Alignment (alignToCluster)
 * - Stopping the Robot (stopRobot)
 */
@Autonomous(name = "Basic Auto - Flower to Hive", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {

    // Shared robot subsystem (drive, shooter, intake, AprilTag vision)
    private final Robot robot = new Robot();

    // Alliance selected during init ("RED" or "BLUE")
    private String alliance = "RED";

    @Override
    public void runOpMode() {

        // ===============================================
        // 1. HARDWARE + SUBSYSTEM INITIALIZATION
        // ===============================================
        robot.init(hardwareMap);

        // ===============================================
        // 2. ALLIANCE SELECTION
        // Press X for BLUE, B for RED while in INIT.
        // ===============================================
        while (opModeInInit()) {
            if (gamepad1.x) {
                alliance = "BLUE";
            } else if (gamepad1.b) {
                alliance = "RED";
            }

            telemetry.addData("Status", "Initialized - Ready for Start");
            telemetry.addData("Alliance", alliance);
            telemetry.addLine("Press X for BLUE, B for RED");
            telemetry.update();
        }

        // Wait for the driver to press PLAY on the Driver Station
        waitForStart();

        if (!opModeIsActive()) {
            robot.close();
            return;
        }

        try {
            // ===============================================
            // 3. FLOWER TO HIVE AUTONOMOUS SEQUENCE
            // ===============================================

            // STEP 1: Start intake and lightly bump Flower to collect piece
            telemetry.addLine("Step 1: Starting intake & approaching Flower...");
            telemetry.update();
            robot.intakeSystem.intakeIn();
            driveForward(0.35, 1.2);
            sleep(400);
            robot.intakeSystem.stopIntake();

            // STEP 2: Back away from Flower and face towards Hive
            telemetry.addLine("Step 2: Backing away and turning toward Hive...");
            telemetry.update();
            driveBackward(0.4, 0.8);
            turnRight(0.4, 0.6);

            // STEP 3: Camera line up with Hive AprilTag cluster
            String targetCluster = "RED".equals(alliance) ? AprilTagVision.RED_SCORING : AprilTagVision.BLUE_SCORING;
            telemetry.addLine("Step 3: Aligning to Hive (" + targetCluster + ")...");
            telemetry.update();
            alignToCluster(targetCluster, 4.0);

            // STEP 4: Pause 3s (pretend shoot)
            telemetry.addLine("Step 4: Pausing 3s (pretend shoot)...");
            telemetry.update();
            robot.shooter.setTargetPower(Shooter.POWER_MID_SHOT);
            sleep(3000);
            robot.shooter.spinDown();

            // STEP 5: Drive to parking space
            telemetry.addLine("Step 5: Driving to parking space...");
            telemetry.update();
            driveForward(0.5, 1.5);

            // STEP 6: Stop the Robot
            telemetry.addLine("Step 6: Stopping Robot.");
            telemetry.update();
            stopRobot();
            telemetry.addData("Status", "Autonomous Sequence Complete!");
            telemetry.update();
        } finally {
            robot.close();
        }
    }

    // =========================================================================
    // HELPER COMMANDS FOR AUTONOMOUS MOVEMENTS
    // Use these methods to build custom autonomous paths.
    // =========================================================================

    /**
     * COMMAND: MOVE FORWARD
     * Drives both wheels forward at the specified power for a set duration.
     * 
     * @param power Speed level from 0.0 to 1.0 (e.g., 0.5 = 50% max speed)
     * @param seconds Duration in seconds to execute the movement
     */
    private void driveForward(double power, double seconds) {
        driveFor(power, 0.0, seconds);
    }

    /**
     * COMMAND: MOVE BACKWARD
     * Drives both wheels backward for a set duration.
     * 
     * @param power Speed level from 0.0 to 1.0
     * @param seconds Duration in seconds
     */
    private void driveBackward(double power, double seconds) {
        driveFor(-power, 0.0, seconds);
    }

    /**
     * COMMAND: MOVE / TURN LEFT
     * Spins left in place by applying a negative turn command.
     * 
     * @param power Turn speed level from 0.0 to 1.0
     * @param seconds Duration in seconds
     */
    private void turnLeft(double power, double seconds) {
        driveFor(0.0, -power, seconds);
    }

    /**
     * COMMAND: MOVE / TURN RIGHT
     * Spins right (clockwise) in place by applying a positive turn command.
     * 
     * @param power Turn speed level from 0.0 to 1.0
     * @param seconds Duration in seconds
     */
    private void turnRight(double power, double seconds) {
        driveFor(0.0, power, seconds);
    }

    /**
     * COMMAND: DRIVE WITH FORWARD / TURN INPUT
     * Applies a forward and turn command to the drivetrain for a set duration.
     * 
     * @param forward Forward power (-1.0 to 1.0), positive drives forward
     * @param turn    Turn power (-1.0 to 1.0), positive turns right (clockwise)
     * @param seconds Duration to run before stopping
     */
    private void driveFor(double forward, double turn, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            robot.drive(forward, turn);

            telemetry.addData("Time Elapsed", "%.1f / %.1f sec", timer.seconds(), seconds);
            telemetry.addData("Forward Power", "%.2f", forward);
            telemetry.addData("Turn Power", "%.2f", turn);
            telemetry.update();

            sleep(20);
        }
        robot.stopDrive();
    }

    /**
     * COMMAND: ALIGN TO APRILTAG CLUSTER
     * Rotates toward the named AprilTag cluster until aligned within tolerance or
     * the timeout elapses.
     * 
     * @param clusterName    Cluster constant from {@link AprilTagVision}
     * @param timeoutSeconds Maximum time to spend aligning
     * @return true if the robot aligned within tolerance, false otherwise
     */
    private boolean alignToCluster(String clusterName, double timeoutSeconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            Robot.AlignResult result = robot.calculateClusterAlign(clusterName);
            if (result.targetFound) {
                if (result.aligned) {
                    robot.stopDrive();
                    return true;
                }
                robot.drive(0, result.turnPower);
            } else {
                robot.stopDrive();
            }
            sleep(20);
        }
        robot.stopDrive();
        return false;
    }

    /**
     * COMMAND: STOP ROBOT
     * Stops the drivetrain.
     */
    private void stopRobot() {
        robot.stopDrive();
    }
}
