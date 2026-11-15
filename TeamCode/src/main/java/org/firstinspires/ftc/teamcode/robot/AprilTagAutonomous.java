package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Autonomous OpMode for FTC testing.
 * Navigates a hardcoded waypoint path, using SDK 12 BIOBUZZ clusters
 * for relative closed-loop alignment (distance and heading), not absolute field localization.
 */
@Autonomous(name = "AprilTag Autonomous Path", group = "Autonomous")
public class AprilTagAutonomous extends LinearOpMode {

    private final Robot robot = new Robot();
    private final ElapsedTime timer = new ElapsedTime();

    // Alignment tuning constants (Proportional control)
    private static final double DESIRED_DISTANCE_INCHES = 12.0; // Target distance to tag
    private static final double P_DRIVE_GAIN = 0.03;            // Proportional gain for driving forward/back
    private static final double P_TURN_GAIN  = 0.02;            // Proportional gain for turning
    private static final double MAX_AUTO_SPEED = 0.40;           // Max drive power during alignment
    private static final double MAX_AUTO_TURN  = 0.30;           // Max turn power during alignment

    // Tolerances
    private static final double DISTANCE_TOLERANCE_INCHES = 1.5;
    private static final double BEARING_TOLERANCE_DEGREES = 2.0;

    // Target Tag IDs based on Game Map
    // Red Alliance: 34-37 (Audience), 30-33 (Scoring)
    // Blue Alliance: 38-41 (Audience), 42-45 (Scoring)
    private static final int TARGET_TAG_WAYPOINT_1 = 34; // Red Audience Tag
    private static final int TARGET_TAG_WAYPOINT_2 = 30; // Red Scoring Tag

    @Override
    public void runOpMode() {

        // Initialize robot hardware and AprilTag vision
        robot.init(hardwareMap);

        telemetry.addLine("=================================");
        telemetry.addLine("  APRILTAG AUTONOMOUS INITIALIZED");
        telemetry.addLine("  Target Tag 1: ID " + TARGET_TAG_WAYPOINT_1 + " (Red Audience)");
        telemetry.addLine("  Target Tag 2: ID " + TARGET_TAG_WAYPOINT_2 + " (Red Scoring)");
        telemetry.addLine("=================================");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            robot.close();
            return;
        }

        // ===============================================
        // STEP 1: Drive from Start to Waypoint 1 Area
        // ===============================================
        telemetry.addLine("Step 1: Driving forward toward Waypoint 1...");
        telemetry.update();
        driveTime(0.4, 0.0, 1.5); // Drive forward 1.5 seconds

        // Turn towards Waypoint 1 (Red Audience Area)
        telemetry.addLine("Step 1b: Turning toward Tag " + TARGET_TAG_WAYPOINT_1 + "...");
        telemetry.update();
        driveTime(0.0, 0.3, 0.8); // Turn right ~45 degrees

        // ===============================================
        // STEP 2: Verify Position & Align using Red Audience
        // ===============================================
        telemetry.addLine("Step 2: Searching & Aligning to AprilTag " + TARGET_TAG_WAYPOINT_1 + "...");
        telemetry.update();
        alignToTag(TARGET_TAG_WAYPOINT_1, 4.0); // Try aligning for up to 4 seconds

        sleep(500); // Pause briefly to observe telemetry

        // ===============================================
        // STEP 3: Navigate from Waypoint 1 to Waypoint 2
        // ===============================================
        telemetry.addLine("Step 3: Repositioning toward Waypoint 2...");
        telemetry.update();
        // Back up slightly from Tag 1
        driveTime(-0.3, 0.0, 1.0);
        // Turn left toward Scoring Area (Red Scoring)
        driveTime(0.0, -0.3, 1.2);
        // Drive forward across field
        driveTime(0.4, 0.0, 2.0);

        // ===============================================
        // STEP 4: Verify Position & Align using Red Scoring
        // ===============================================
        telemetry.addLine("Step 4: Searching & Aligning to AprilTag " + TARGET_TAG_WAYPOINT_2 + "...");
        telemetry.update();
        alignToTag(TARGET_TAG_WAYPOINT_2, 4.0); // Try aligning for up to 4 seconds

        sleep(500);

        // ===============================================
        // STEP 5: Final Park Step
        // ===============================================
        telemetry.addLine("Step 5: Parking robot...");
        telemetry.update();
        driveTime(-0.3, 0.0, 0.8); // Back up safely
        robot.drive(0, 0);         // Stop all motors

        telemetry.addLine("=================================");
        telemetry.addLine("    AUTONOMOUS PATH COMPLETED    ");
        telemetry.addLine("=================================");
        telemetry.update();

        // Keep vision active while idle to display final tags
        while (opModeIsActive()) {
            List<AprilTagDetection> current = robot.getAprilTagDetections();
            telemetry.addData("Final Visible AprilTags", current != null ? current.size() : 0);
            telemetry.update();
            sleep(100);
        }

        robot.close();
    }

    /**
     * Drives the robot with given forward and turn powers for specified duration.
     */
    private void driveTime(double forward, double turn, double durationSeconds) {
        timer.reset();
        while (opModeIsActive() && timer.seconds() < durationSeconds) {
            robot.drive(forward, turn);
            telemetry.addData("Path Execution", "Driving (Fwd: %.2f, Turn: %.2f) - %.1fs / %.1fs",
                    forward, turn, timer.seconds(), durationSeconds);
            telemetry.update();
        }
        robot.drive(0, 0);
    }

    /**
     * Uses AprilTag feedback to verify robot position and dynamically align to a target tag.
     * Adjusts heading (bearing) and distance (range) using proportional control.
     */
    private void alignToTag(int targetTagId, double maxTimeoutSeconds) {
        timer.reset();
        boolean aligned = false;

        while (opModeIsActive() && timer.seconds() < maxTimeoutSeconds && !aligned) {
            List<AprilTagDetection> detections = robot.getAprilTagDetections();
            AprilTagDetection targetTag = findTagById(detections, targetTagId);

            if (targetTag != null && targetTag.ftcPose != null) {
                double rangeError = targetTag.ftcPose.range - DESIRED_DISTANCE_INCHES;
                double bearingError = targetTag.ftcPose.bearing;

                // Calculate drive and turn corrections
                double drivePower = rangeError * P_DRIVE_GAIN;
                double turnPower = bearingError * P_TURN_GAIN;

                // Clip power to safe limits
                drivePower = Math.max(-MAX_AUTO_SPEED, Math.min(MAX_AUTO_SPEED, drivePower));
                turnPower = Math.max(-MAX_AUTO_TURN, Math.min(MAX_AUTO_TURN, turnPower));

                robot.drive(drivePower, turnPower);

                // Check if aligned within tolerance
                if (Math.abs(rangeError) < DISTANCE_TOLERANCE_INCHES &&
                        Math.abs(bearingError) < BEARING_TOLERANCE_DEGREES) {
                    aligned = true;
                    robot.drive(0, 0);
                    telemetry.addLine(">> VERIFIED & ALIGNED TO TAG " + targetTagId + " <<");
                } else {
                    telemetry.addData("Tag " + targetTagId + " Verified", "Range Error: %.1fin, Bearing Error: %.1fdeg",
                            rangeError, bearingError);
                }
            } else {
                // If tag is not in view yet, slowly rotate to search for it
                robot.drive(0, 0.15);
                telemetry.addData("Tag " + targetTagId, "Not in view - Scanning...");
            }

            telemetry.update();
        }

        robot.drive(0, 0);
    }

    /**
     * Helper to locate a specific AprilTag by ID from current detections.
     */
    private AprilTagDetection findTagById(List<AprilTagDetection> detections, int id) {
        if (detections == null) return null;
        for (AprilTagDetection detection : detections) {
            if (AprilTagTargets.matchesId(detection, id)) {
                return detection;
            }
        }
        return null;
    }
}
