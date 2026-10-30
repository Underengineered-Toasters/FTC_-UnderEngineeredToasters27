package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * RED ALLIANCE TELEOP OPMODE
 * 
 * Camera Turret Auto-Alignment:
 * - LB (Hold): Auto-aligns Turret to Red Tags [30, 31, 32, 33]
 * - RB (Hold): Auto-aligns Turret to Red Tags [34, 35, 36, 37]
 * 
 * Manual Controls:
 * - Left Stick Y / Right Stick X: Manual Drive & Turn
 * - D-Pad Left / Right: Manual 360° Turret Rotation
 * - Right Trigger: Shoot (Shooter 1 & 2 at 100% Speed)
 * - Left Trigger: Run Intake
 * - Button Y: Toggle Intake Direction
 * - Button A: Toggle Match Loader Servo DOWN 90°
 * - Button X: Direct Intake OUT (Reverse)
 */
@TeleOp(name = "Red Alliance TeleOp", group = "Red")
public class RedTeleOp extends LinearOpMode {

    private final Robot robot = new Robot();

    // Intake direction state
    private boolean intakeInDirection = true;
    private boolean previousY = false;

    // Match loader servo state
    private boolean matchLoaderDown = false;
    private boolean previousA = false;

    // Proportional gains for camera alignment
    private static final double P_TURRET_GAIN = 0.025; // Gain for turret auto-alignment
    private static final double P_DRIVE_GAIN  = 0.03;
    private static final double P_CHASSIS_TURN_GAIN = 0.02;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addLine("=================================");
        telemetry.addLine("   RED ALLIANCE TELEOP READY     ");
        telemetry.addLine("  LB: Turret Align Tags 30-33    ");
        telemetry.addLine("  RB: Turret Align Tags 34-37    ");
        telemetry.addLine("=================================");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            robot.close();
            return;
        }

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = robot.getAprilTagDetections();

            double drive = -gamepad1.left_stick_y;
            double turn  = -gamepad1.right_stick_x;

            // =========================================================================
            // TURRET CAMERA AUTO-ALIGNMENT TO RED APRILTAGS (LB / RB)
            // =========================================================================

            boolean leftBumper  = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            boolean isAutoAligningTurret = false;

            if (leftBumper || rightBumper) {
                int[] targetGroup = leftBumper ? AprilTagVision.RED_TAGS_PRIMARY : AprilTagVision.RED_TAGS_SECONDARY;
                AprilTagDetection targetTag = robot.vision.findTagInGroup(targetGroup);

                // Fallback: If group tag not found, use any visible tag
                if (targetTag == null && detections != null && !detections.isEmpty()) {
                    targetTag = detections.get(0);
                }

                if (targetTag != null && targetTag.ftcPose != null) {
                    isAutoAligningTurret = true;

                    double bearingError = targetTag.ftcPose.bearing; // Camera offset angle
                    double rangeError   = targetTag.ftcPose.range - 12.0;

                    // Calculate proportional turret alignment power
                    double turretPower = bearingError * P_TURRET_GAIN;
                    turretPower = Math.max(-0.6, Math.min(0.6, turretPower));

                    // Rotate turret automatically to lock onto target
                    robot.turretSystem.setPower(turretPower);

                    // Chassis assistance drive
                    drive = Math.max(-0.3, Math.min(0.3, rangeError * P_DRIVE_GAIN));
                    turn  = Math.max(-0.2, Math.min(0.2, bearingError * P_CHASSIS_TURN_GAIN));

                    telemetry.addData("Turret Auto-Aligning Tag", targetTag.id);
                    telemetry.addData("Bearing Error (Deg)", "%.1f", bearingError);
                    telemetry.addData("Turret Power", "%.2f", turretPower);
                } else {
                    telemetry.addData("Turret Auto-Align", "No Target Red Tags in View!");
                }
            }

            // Apply chassis drive
            robot.drive(drive, turn);

            // =========================================================================
            // MANUAL TURRET CONTROL (D-Pad Left / Right - Active when not auto-aligning)
            // =========================================================================

            if (!isAutoAligningTurret) {
                boolean dpadLeft  = gamepad1.dpad_left || gamepad2.dpad_left;
                boolean dpadRight = gamepad1.dpad_right || gamepad2.dpad_right;

                if (dpadLeft) {
                    robot.turretSystem.rotateLeft();
                } else if (dpadRight) {
                    robot.turretSystem.rotateRight();
                } else {
                    robot.turretSystem.stopTurret();
                }
            }

            // =========================================================================
            // BUTTON Y: TOGGLE INTAKE DIRECTION
            // =========================================================================

            boolean currentY = gamepad1.y || gamepad2.y;

            if (currentY && !previousY) {
                intakeInDirection = !intakeInDirection;
            }

            previousY = currentY;

            // =========================================================================
            // BUTTON A: MATCH LOADER SERVO DOWN 90°
            // =========================================================================

            boolean currentA = gamepad1.a || gamepad2.a;

            if (currentA && !previousA) {
                matchLoaderDown = !matchLoaderDown;
                robot.setMatchLoaderDown(matchLoaderDown);
            }

            previousA = currentA;

            // =========================================================================
            // TRIGGERS & INTAKE / SHOOTER CONTROLS
            // =========================================================================

            double intakeTrigger  = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
            double shooterTrigger = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
            boolean buttonX       = gamepad1.x || gamepad2.x;

            // SHOOTER CONTROL (Right Trigger)
            if (shooterTrigger > 0.05) {
                robot.shooter1System.spinForwardMaxSpeed();
                robot.shooter2System.spinForwardMaxSpeed();
            } else {
                robot.shooter1System.spinDown();
                robot.shooter2System.spinDown();
            }

            // INTAKE CONTROL (Left Trigger / Button X)
            if (buttonX) {
                // Direct Intake OUT
                robot.intakeSystem.intakeOut();

            } else if (intakeTrigger > 0.05) {
                // Run Intake according to current toggle direction
                if (intakeInDirection) {
                    robot.intakeSystem.intakeIn();
                } else {
                    robot.intakeSystem.intakeOut();
                }

            } else {
                robot.intakeSystem.stopIntake();
            }

            robot.update();

            // =========================================================================
            // TELEMETRY
            // =========================================================================

            telemetry.addLine("------------------------------");
            telemetry.addData("Drive Left Power", "%.2f", robot.getLeftPower());
            telemetry.addData("Drive Right Power", "%.2f", robot.getRightPower());

            telemetry.addLine("------------------------------");
            telemetry.addData("Turret Power", "%.2f", robot.turretSystem.getPower());
            telemetry.addData("Turret Limit Switch", robot.turretSystem.isLimitReached() ? "ACTIVATED (mag)" : "OPEN");

            telemetry.addLine("------------------------------");
            telemetry.addData("Intake Direction (Y)", intakeInDirection ? "IN" : "OUT");
            telemetry.addData("Intake Power", "%.2f", robot.intakeSystem.getPower());

            telemetry.addLine("------------------------------");
            telemetry.addData("Shooter 1 Power", "%.2f", robot.shooter1System.getCurrentPower());
            telemetry.addData("Shooter 2 Power", "%.2f", robot.shooter2System.getCurrentPower());

            telemetry.addLine("------------------------------");
            telemetry.addData("Match Loader Servo (A)", matchLoaderDown ? "DOWN (90°)" : "UP (Rest)");

            telemetry.addLine("------------------------------");
            telemetry.addData("Visible AprilTags", detections != null ? detections.size() : 0);

            telemetry.update();
        }

        robot.intakeSystem.stopIntake();
        robot.shooter1System.spinDown();
        robot.shooter2System.spinDown();
        robot.turretSystem.stopTurret();
        robot.close();
    }
}
