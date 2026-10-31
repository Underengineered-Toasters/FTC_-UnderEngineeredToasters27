package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * BLUE ALLIANCE TELEOP OPMODE (Single Controller)
 * 
 * Match Loader Servo:
 * - Rotates 110° to the left on Button A toggle.
 * - Auto-halts / retracts if Touch Sensor ("matchLoaderTouch") is triggered.
 * 
 * Controls (Gamepad 1):
 * - Left Stick Y / Right Stick X: Manual Drive & Turn
 * - D-Pad Left / Right: Manual 360° Turret Rotation
 * - Right Trigger: Shoot (Shooter 1 & 2 at 100% Speed)
 * - Left Trigger: Run Intake
 * - Button Y: Toggle Intake Direction
 * - Button A: Toggle Match Loader Servo DOWN 110°
 * - Button X: Direct Intake OUT (Reverse)
 */
@TeleOp(name = "Blue Alliance TeleOp", group = "Blue")
public class BlueTeleOp extends LinearOpMode {

    private final Robot robot = new Robot();

    // Intake direction state
    private boolean intakeInDirection = true;
    private boolean previousY = false;

    // Match loader servo state
    private boolean matchLoaderDown = false;
    private boolean previousA = false;

    // Camera mounting offset correction for Blue Alliance (+45° Left Offset Correction)
    private static final double CAMERA_MOUNT_OFFSET_DEGREES = +45.0;

    // Proportional gains for camera turret alignment
    private static final double P_TURRET_GAIN = 0.020;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addLine("=================================");
        telemetry.addLine("  BLUE ALLIANCE TELEOP READY     ");
        telemetry.addLine("  Match Loader: 110° + Touch Halt");
        telemetry.addLine("  LB: Turret Align Tags 38-41    ");
        telemetry.addLine("  RB: Turret Align Tags 42-45    ");
        telemetry.addLine("  Right Trigger = Shooter 1 & 2  ");
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
            // TURRET CAMERA AUTO-ALIGNMENT TO BLUE APRILTAGS (LB / RB)
            // =========================================================================

            boolean leftBumper  = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            boolean isAutoAligningTurret = false;

            if (leftBumper || rightBumper) {
                int[] targetGroup = leftBumper ? AprilTagVision.BLUE_TAGS_PRIMARY : AprilTagVision.BLUE_TAGS_SECONDARY;
                AprilTagDetection targetTag = robot.vision.findTagInGroup(targetGroup);

                if (targetTag == null && detections != null && !detections.isEmpty()) {
                    targetTag = detections.get(0);
                }

                if (targetTag != null && targetTag.ftcPose != null) {
                    isAutoAligningTurret = true;

                    double rawBearing = targetTag.ftcPose.bearing;
                    double adjustedBearing = rawBearing + CAMERA_MOUNT_OFFSET_DEGREES;

                    double turretPower = adjustedBearing * P_TURRET_GAIN;
                    turretPower = Math.max(-0.45, Math.min(0.45, turretPower));

                    robot.turretSystem.setPower(turretPower);

                    telemetry.addData("Turret Auto-Aligning Tag", targetTag.id);
                    telemetry.addData("Raw Bearing (Deg)", "%.1f", rawBearing);
                    telemetry.addData("Adjusted Bearing", "%.1f", adjustedBearing);
                    telemetry.addData("Turret Power", "%.2f", turretPower);
                } else {
                    robot.turretSystem.stopTurret();
                    telemetry.addData("Turret Auto-Align", "No Target Blue Tags in View!");
                }
            }

            // Apply chassis drive
            robot.drive(drive, turn);

            // =========================================================================
            // MANUAL TURRET CONTROL (D-Pad Left / Right)
            // =========================================================================

            if (!isAutoAligningTurret) {
                if (gamepad1.dpad_left) {
                    robot.turretSystem.rotateLeft();
                } else if (gamepad1.dpad_right) {
                    robot.turretSystem.rotateRight();
                } else {
                    robot.turretSystem.stopTurret();
                }
            }

            // =========================================================================
            // BUTTON Y: TOGGLE INTAKE DIRECTION
            // =========================================================================

            boolean currentY = gamepad1.y;

            if (currentY && !previousY) {
                intakeInDirection = !intakeInDirection;
            }

            previousY = currentY;

            // =========================================================================
            // BUTTON A: MATCH LOADER SERVO DOWN 110° (WITH TOUCH SENSOR HALT)
            // =========================================================================

            boolean currentA = gamepad1.a;

            if (currentA && !previousA) {
                matchLoaderDown = !matchLoaderDown;
                robot.setMatchLoaderDown(matchLoaderDown);
            }

            previousA = currentA;

            // Update robot safety checks (auto-retract match loader if touch sensor pressed)
            robot.update();
            if (robot.isMatchLoaderTouchPressed()) {
                matchLoaderDown = false; // Reset toggle state if touch sensor triggered
            }

            // =========================================================================
            // TRIGGERS & INTAKE / SHOOTER CONTROLS
            // =========================================================================

            double intakeTrigger  = gamepad1.left_trigger;
            double shooterTrigger = gamepad1.right_trigger;
            boolean buttonX       = gamepad1.x;

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
                if (intakeInDirection) {
                    robot.intakeSystem.intakeIn();
                } else {
                    robot.intakeSystem.intakeOut();
                }

            } else {
                robot.intakeSystem.stopIntake();
            }

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
            telemetry.addData("Match Loader Servo (A)", matchLoaderDown ? "DOWN (110°)" : "UP (Rest)");
            telemetry.addData("Match Loader Touch Sensor", robot.isMatchLoaderTouchPressed() ? "TOUCHED (HALTED)" : "OPEN");

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
