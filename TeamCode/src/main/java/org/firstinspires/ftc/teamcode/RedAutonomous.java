package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * RED ALLIANCE AUTONOMOUS PATH (High-Speed Performance)
 *
 * Includes tagAlign() method for camera turret & chassis alignment.
 */
@Autonomous(name = "Red Alliance Autonomous", group = "Red")
public class RedAutonomous extends LinearOpMode {

    private final Robot robot = new Robot();
    private final ElapsedTime runtime = new ElapsedTime();

    // Calibration: Estimated driving speed in inches per second at 0.5 power
    public static final double ESTIMATED_INCHES_PER_SECOND = 28.0;

    @Override
    public void runOpMode() {

        // Initialize drive motors and vision
        robot.init(hardwareMap);

        telemetry.addLine("=================================");
        telemetry.addLine("  RED ALLIANCE AUTONOMOUS READY ");
        telemetry.addLine("  tagAlign() Vision Active      ");
        telemetry.addLine("=================================");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // =========================================================================
        // RED ALLIANCE AUTONOMOUS PATH
        // =========================================================================

        // STEP 1: Align to Red AprilTags (30-33) before shooting
        telemetry.addLine("Auto-Aligning to Red AprilTags...");
        telemetry.update();
        tagAlign(AprilTagVision.RED_TAGS_PRIMARY); // Automatically locks onto Red AprilTags!

        // STEP 2: Drive into position and intake
        driveInches(30, 0.70);
        sleep(500);
        turnRight(0.42, 0.56);
        sleep(500);
        driveInches(10, 0.5);
        sleep(500);

        // Align again at second waypoint
        tagAlign(AprilTagVision.RED_TAGS_SECONDARY);

        turnRight(0.7, 0.42);
        driveInches(18, 0.75);
        sleep(2000);

        driveInches(-15, 0.75);
        sleep(500);
        turnLeft(0.5, 0.7);
        driveInches(15, 0.7);
        sleep(500);
        turnLeft(0.4, 0.45);
        sleep(500);
        driveInches(50, 0.7);

        stopRobot();

        telemetry.addLine("=================================");
        telemetry.addLine("  RED ALLIANCE AUTO COMPLETED!  ");
        telemetry.addData("Total Time Elapsed", "%.2f sec", runtime.seconds());
        telemetry.addLine("=================================");
        telemetry.update();
    }

    // =========================================================================
    // AUTONOMOUS APRILTAG ALIGNMENT METHOD
    // =========================================================================

    /**
     * Automatically locks onto target AprilTags, aligns turret, and drives chassis into position.
     * Usage: tagAlign(AprilTagVision.RED_TAGS_PRIMARY);
     */
    public boolean tagAlign(int[] targetTagIds) {
        return tagAlign(targetTagIds, 12.0, 3.5);
    }

    public boolean tagAlign(int[] targetTagIds, double targetDistanceInches, double maxTimeoutSeconds) {
        ElapsedTime alignTimer = new ElapsedTime();
        alignTimer.reset();
        boolean aligned = false;

        while (opModeIsActive() && alignTimer.seconds() < maxTimeoutSeconds && !aligned) {
            List<AprilTagDetection> detections = robot.getAprilTagDetections();
            AprilTagDetection targetTag = robot.vision.findTagInGroup(targetTagIds);

            if (targetTag == null && detections != null && !detections.isEmpty()) {
                targetTag = detections.get(0);
            }

            if (targetTag != null && targetTag.ftcPose != null) {
                // Apply +45° Camera Offset Correction
                double rawBearing = targetTag.ftcPose.bearing;
                double adjustedBearing = rawBearing + 45.0; 
                double rangeError = targetTag.ftcPose.range - targetDistanceInches;

                // Turret alignment power
                double turretPower = adjustedBearing * 0.020;
                turretPower = Math.max(-0.45, Math.min(0.45, turretPower));
                robot.turretSystem.setPower(turretPower);

                // Chassis drive correction
                double drivePower = rangeError * 0.03;
                double turnPower = adjustedBearing * 0.02;
                drivePower = Math.max(-0.35, Math.min(0.35, drivePower));
                turnPower = Math.max(-0.25, Math.min(0.25, turnPower));

                robot.drive(drivePower, turnPower);

                // Check alignment tolerance (1.5 inches range, 2.0 degrees bearing)
                if (Math.abs(rangeError) < 1.5 && Math.abs(adjustedBearing) < 2.0) {
                    aligned = true;
                    robot.drive(0, 0);
                    robot.turretSystem.stopTurret();
                    telemetry.addData("tagAlign()", "ALIGNED to Tag " + targetTag.id);
                } else {
                    telemetry.addData("tagAlign()", "Aligning Tag %d (Range Err: %.1fin, Bearing: %.1fdeg)",
                            targetTag.id, rangeError, adjustedBearing);
                }
            } else {
                robot.drive(0.0, 0.15); // Scan slowly for tag
                robot.turretSystem.stopTurret();
                telemetry.addData("tagAlign()", "Searching for AprilTags...");
            }

            telemetry.update();
        }

        robot.drive(0, 0);
        robot.turretSystem.stopTurret();
        return aligned;
    }

    // =========================================================================
    // TIME & POWER DRIVING HELPER METHODS
    // =========================================================================

    private void driveInches(double targetInches, double power) {
        double seconds = Math.abs(targetInches) / (ESTIMATED_INCHES_PER_SECOND * (Math.abs(power) / 0.5));
        double direction = targetInches >= 0 ? 1.0 : -1.0;
        double motorPower = Math.abs(power) * direction;

        driveForSeconds(motorPower, 0.0, seconds);
    }

    private void turnRight(double power, double seconds) {
        driveForSeconds(0.0, power, seconds);
    }

    private void turnLeft(double power, double seconds) {
        driveForSeconds(0.0, -power, seconds);
    }

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

    private void stopRobot() {
        robot.drive(0.0, 0.0);
    }
}
