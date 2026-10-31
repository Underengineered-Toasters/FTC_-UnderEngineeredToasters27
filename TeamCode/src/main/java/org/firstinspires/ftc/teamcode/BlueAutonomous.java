package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * BLUE ALLIANCE AUTONOMOUS PATH (High-Speed Performance)
 *
 * Includes blueTagAlign(), deployRamp(), pullRamp(), intake(seconds), and shoot(seconds).
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
        telemetry.addLine("  blueTagAlign() Vision Active   ");
        telemetry.addLine("=================================");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // =========================================================================
        // BLUE ALLIANCE AUTONOMOUS PATH EXAMPLE
        // =========================================================================

        // STEP 1: Align to Blue AprilTags (38-41 / 42-45)
        telemetry.addLine("Auto-Aligning to Blue AprilTags...");
        telemetry.update();
        blueTagAlign();

        // STEP 2: Deploy Ramp & Shoot for 2.5 seconds
        deployRamp();
        shoot(2.5);
        pullRamp();

        // STEP 3: Drive forward & Intake for 3.0 seconds
        driveInches(25, 0.60);
        intake(3.0);

        // STEP 4: Secondary Alignment & Final Shoot
        blueTagAlign(12.0, 3.0);
        deployRamp();
        shoot(2.0);
        pullRamp();

        stopRobot();

        telemetry.addLine("=================================");
        telemetry.addLine("  BLUE ALLIANCE AUTO COMPLETED! ");
        telemetry.addData("Total Time Elapsed", "%.2f sec", runtime.seconds());
        telemetry.addLine("=================================");
        telemetry.update();
    }

    // =========================================================================
    // BLUE ALLIANCE AUTONOMOUS APRILTAG ALIGNMENT
    // =========================================================================

    /**
     * Automatically locks onto target Blue AprilTags (38-41 primary / 42-45 secondary).
     * Usage: blueTagAlign();
     */
    public boolean blueTagAlign() {
        return blueTagAlign(12.0, 3.5);
    }

    public boolean blueTagAlign(double targetDistanceInches, double maxTimeoutSeconds) {
        int[] targetTagIds = AprilTagVision.BLUE_TAGS_PRIMARY;
        ElapsedTime alignTimer = new ElapsedTime();
        alignTimer.reset();
        boolean aligned = false;

        while (opModeIsActive() && alignTimer.seconds() < maxTimeoutSeconds && !aligned) {
            List<AprilTagDetection> detections = robot.getAprilTagDetections();
            AprilTagDetection targetTag = robot.vision.findTagInGroup(targetTagIds);

            if (targetTag == null) {
                targetTag = robot.vision.findTagInGroup(AprilTagVision.BLUE_TAGS_SECONDARY);
            }

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
                    telemetry.addData("blueTagAlign()", "ALIGNED to Blue Tag " + targetTag.id);
                } else {
                    telemetry.addData("blueTagAlign()", "Aligning Blue Tag %d (Range Err: %.1fin, Bearing: %.1fdeg)",
                            targetTag.id, rangeError, adjustedBearing);
                }
            } else {
                robot.drive(0.0, 0.15); // Scan slowly for tag
                robot.turretSystem.stopTurret();
                telemetry.addData("blueTagAlign()", "Searching for Blue AprilTags...");
            }

            telemetry.update();
        }

        robot.drive(0, 0);
        robot.turretSystem.stopTurret();
        return aligned;
    }

    // =========================================================================
    // RAMP, INTAKE, AND SHOOTING AUTONOMOUS HELPER METHODS
    // =========================================================================

    public void deployRamp() {
        robot.setMatchLoaderDown(true);
        telemetry.addLine("Ramp Deployed (110°)");
        telemetry.update();
    }

    public void pullRamp() {
        robot.setMatchLoaderDown(false);
        telemetry.addLine("Ramp Retracted (UP)");
        telemetry.update();
    }

    public void intake(double seconds) {
        ElapsedTime intakeTimer = new ElapsedTime();
        intakeTimer.reset();
        robot.intakeSystem.intakeIn();

        while (opModeIsActive() && intakeTimer.seconds() < seconds) {
            telemetry.addData("Intaking", "%.1fs / %.1fs", intakeTimer.seconds(), seconds);
            telemetry.update();
        }

        robot.intakeSystem.stopIntake();
    }

    public void shoot(double seconds) {
        ElapsedTime shootTimer = new ElapsedTime();
        shootTimer.reset();
        robot.shooter1System.spinForwardMaxSpeed();
        robot.shooter2System.spinForwardMaxSpeed();

        while (opModeIsActive() && shootTimer.seconds() < seconds) {
            telemetry.addData("Shooting", "%.1fs / %.1fs", shootTimer.seconds(), seconds);
            telemetry.update();
        }

        robot.shooter1System.spinDown();
        robot.shooter2System.spinDown();
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
