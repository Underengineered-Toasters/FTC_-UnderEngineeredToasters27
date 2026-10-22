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
    private boolean xLast = false;
    private boolean bLast = false;
    private final AutoLog logger = new AutoLog();
    private String currentStep = "INIT";

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
            // Edge-detect so a held or bouncing button doesn't flip the choice back
            boolean xNow = gamepad1.x;
            boolean bNow = gamepad1.b;
            if (xNow && !xLast) {
                alliance = "BLUE";
            } else if (bNow && !bLast) {
                alliance = "RED";
            }
            xLast = xNow;
            bLast = bNow;

            telemetry.addData("Status", "Initialized - Ready for Start");
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Camera", robot.getCameraState());
            telemetry.addLine("Press X for BLUE, B for RED");
            telemetry.update();
        }

        // Wait for the driver to press PLAY on the Driver Station
        waitForStart();
        logger.open("auto_flower_hive");
        if (!opModeIsActive()) {
            robot.close();
            return;
        }

        try {
            // ===============================================
            // 3. FLOWER TO HIVE AUTONOMOUS SEQUENCE
            // ===============================================

            // STEP 1: Start intake and lightly bump Flower to collect piece
            currentStep = "Step 1: Bump Flower";
            telemetry.addLine("Step 1: Starting intake & approaching Flower...");
            telemetry.update();
            robot.intakeSystem.intakeIn();
            driveForward(0.35, 1.2);
            sleep(400);
            robot.intakeSystem.stopIntake();

            // STEP 2: Back away from Flower and face towards Hive
            currentStep = "Step 2: Back out & Turn";
            telemetry.addLine("Step 2: Backing away and turning toward Hive...");
            telemetry.update();
            driveBackward(0.4, 0.8);
            if ("BLUE".equals(alliance)) {
                turnLeft(0.4, 1.1);
            } else {
                turnRight(0.4, 1.1);
            }
            // STEP 3: Camera line up with Hive AprilTag cluster
            currentStep = "Step 3: Align Hive";
            String targetCluster = "RED".equals(alliance) ? AprilTagVision.RED_SCORING : AprilTagVision.BLUE_SCORING;
            telemetry.addLine("Step 3: Aligning to Hive (" + targetCluster + ")...");
            telemetry.update();
            alignToCluster(targetCluster, 4.0);

            // STEP 4: Pause 3s (pretend shoot)
            currentStep = "Step 4: Pause Shoot";
            logSnapshot();
            telemetry.addLine("Step 4: Pausing 3s (pretend shoot)...");
            telemetry.update();
            robot.shooter.setTargetPower(Shooter.POWER_MID_SHOT);
            sleep(3000);
            robot.shooter.spinDown();

            // STEP 5: Turn and drive to parking space
            currentStep = "Step 5: Park";
            telemetry.addLine("Step 5: Driving to parking space...");
            telemetry.update();
            if ("BLUE".equals(alliance)) {
                turnLeft(0.4, 0.8);
            } else {
                turnRight(0.4, 0.8);
            }
            driveForward(0.5, 1.3);
            // STEP 6: Stop the Robot
            currentStep = "Step 6: Stopped";
            logSnapshot();
            telemetry.addLine("Step 6: Stopping Robot.");
            telemetry.update();
            stopRobot();
            telemetry.addData("Status", "Autonomous Sequence Complete!");
            telemetry.update();
        } finally {
            logger.close();
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
            logSnapshot();

            telemetry.addData("Time Elapsed", "%.1f / %.1f sec", timer.seconds(), seconds);
            telemetry.addData("Camera", robot.getCameraState());
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
        if (!robot.isVisionReady()) {
            telemetry.addLine("CAMERA OFFLINE - skipping alignment");
            telemetry.update();
            return false;
        }
        ElapsedTime timer = new ElapsedTime();
        double searchTurn = "BLUE".equals(alliance) ? -0.22 : 0.22;
        while (opModeIsActive() && timer.seconds() < timeoutSeconds) {
            Robot.AlignResult result = robot.calculateClusterAlign(clusterName);
            if (result.targetFound) {
                if (result.aligned) {
                    robot.stopDrive();
                    logAlign(clusterName, result, 0.0);
                    return true;
                }
                robot.drive(0, result.turnPower);
                logAlign(clusterName, result, result.turnPower);
            } else {
                // Sweep slowly until camera acquires the AprilTag cluster
                robot.drive(0, searchTurn);
                logAlign(clusterName, result, searchTurn);
            }
            sleep(20);
        }
        robot.stopDrive();
        return false;
    }
    private void logSnapshot() {
        logger.log(
                currentStep,
                alliance,
                robot.getLeftPower(),
                robot.getRightPower(),
                robot.intakeSystem.getPower(),
                robot.shooter.getCurrentPower(),
                "none",
                false,
                false,
                null,
                null,
                0.0,
                robot.getAprilTagDetections().size()
        );
    }

    private void logAlign(String clusterName, Robot.AlignResult res, double turnCmd) {
        logger.log(
                currentStep,
                alliance,
                robot.getLeftPower(),
                robot.getRightPower(),
                robot.intakeSystem.getPower(),
                robot.shooter.getCurrentPower(),
                clusterName,
                res != null && res.targetFound,
                res != null && res.aligned,
                res != null && res.targetFound ? res.bearingDeg : null,
                res != null && res.targetFound ? res.rangeInches : null,
                turnCmd,
                robot.getAprilTagDetections().size()
        );
    }

    /**
     * COMMAND: STOP ROBOT
     * Stops the drivetrain.
     */
    private void stopRobot() {
        robot.stopDrive();
    }
}
