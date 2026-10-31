package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * RED ALLIANCE AUTONOMOUS PATH (High-Speed Chassis AprilTag Alignment)
 *
 * Uses quick chassis redTagAlign() for lightning-fast autonomous positioning.
 */
@Autonomous(name = "Red Alliance Autonomous", group = "Red")
public class RedAutonomous extends LinearOpMode {

    private final Robot robot = new Robot();
    private final ElapsedTime runtime = new ElapsedTime();

    public static final double ESTIMATED_INCHES_PER_SECOND = 28.0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addLine("=================================");
        telemetry.addLine("  RED ALLIANCE AUTONOMOUS READY ");
        telemetry.addLine("  Quick Chassis redTagAlign()   ");
        telemetry.addLine("=================================");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // =========================================================================
        // RED ALLIANCE AUTONOMOUS PATH
        // =========================================================================

        // STEP 1: Quick Chassis Alignment to Red AprilTags
        telemetry.addLine("Quick Chassis Aligning to Red AprilTags...");
        telemetry.update();
        redTagAlign(12.0, 2.0); // Fast 2.0 second max timeout

        // STEP 2: Deploy Ramp & Shoot for 2.5 seconds
        deployRamp();
        shoot(2.5);
        pullRamp();

        // STEP 3: Drive forward & Intake
        driveInches(25, 0.60);
        intake(3.0);

        // STEP 4: Secondary Quick Alignment & Final Shoot
        redTagAlign(12.0, 2.0);
        deployRamp();
        shoot(2.0);
        pullRamp();

        stopRobot();

        telemetry.addLine("=================================");
        telemetry.addLine("  RED ALLIANCE AUTO COMPLETED!  ");
        telemetry.addData("Total Time Elapsed", "%.2f sec", runtime.seconds());
        telemetry.addLine("=================================");
        telemetry.update();
    }

    // =========================================================================
    // QUICK CHASSIS RED ALLIANCE APRILTAG ALIGNMENT
    // =========================================================================

    public boolean redTagAlign() {
        return redTagAlign(12.0, 2.0); // Fast 2.0s timeout by default
    }

    public boolean redTagAlign(double targetDistanceInches, double maxTimeoutSeconds) {
        int[] targetTagIds = AprilTagVision.RED_TAGS_PRIMARY;
        ElapsedTime alignTimer = new ElapsedTime();
        alignTimer.reset();
        boolean aligned = false;

        while (opModeIsActive() && alignTimer.seconds() < maxTimeoutSeconds && !aligned) {
            List<AprilTagDetection> detections = robot.getAprilTagDetections();
            AprilTagDetection targetTag = robot.vision.findTagInGroup(targetTagIds);

            if (targetTag == null) {
                targetTag = robot.vision.findTagInGroup(AprilTagVision.RED_TAGS_SECONDARY);
            }

            if (targetTag == null && detections != null && !detections.isEmpty()) {
                targetTag = detections.get(0);
            }

            if (targetTag != null && targetTag.ftcPose != null) {
                // Apply +45° Camera Offset Correction
                double rawBearing = targetTag.ftcPose.bearing;
                double adjustedBearing = rawBearing + 45.0; 
                double rangeError = targetTag.ftcPose.range - targetDistanceInches;

                // Quick, snappy chassis drive & turn alignment
                double drivePower = rangeError * 0.05;
                double turnPower = adjustedBearing * 0.035;
                drivePower = Math.max(-0.50, Math.min(0.50, drivePower));
                turnPower = Math.max(-0.40, Math.min(0.40, turnPower));

                robot.drive(drivePower, turnPower);

                // Quick tolerance check (2.0 inches range, 3.0 degrees bearing)
                if (Math.abs(rangeError) < 2.0 && Math.abs(adjustedBearing) < 3.0) {
                    aligned = true;
                    robot.drive(0, 0);
                    telemetry.addData("redTagAlign()", "QUICK ALIGNED to Red Tag " + targetTag.id);
                } else {
                    telemetry.addData("redTagAlign()", "Quick Aligning Tag %d (Err: %.1fin, Turn: %.1fdeg)",
                            targetTag.id, rangeError, adjustedBearing);
                }
            } else {
                robot.drive(0.0, 0.25); // Fast scan speed
                telemetry.addData("redTagAlign()", "Scanning for Red AprilTags...");
            }

            telemetry.update();
        }

        robot.drive(0, 0);
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
