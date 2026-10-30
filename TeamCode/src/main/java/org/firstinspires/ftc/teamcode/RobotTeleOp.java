package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Robot TeleOp", group = "Main")
public class RobotTeleOp extends LinearOpMode {

    private final Robot robot = new Robot();

    // Intake direction state
    private boolean intakeInDirection = true;
    private boolean previousY = false;

    // Match loader servo state
    private boolean matchLoaderDown = false;
    private boolean previousA = false;

    private static final double DESIRED_DISTANCE_INCHES = 12.0;
    private static final double P_DRIVE_GAIN = 0.03;
    private static final double P_TURN_GAIN  = 0.02;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addLine("==============================");
        telemetry.addLine("       ROBOT INITIALIZED");
        telemetry.addLine("==============================");
        telemetry.addLine("Left Stick Y  = Manual Drive");
        telemetry.addLine("Right Stick X = Manual Turn");
        telemetry.addLine("LB / RB       = Camera AprilTag Auto-Align");
        telemetry.addLine("D-Pad Left/Right = Rotate Turret 360°");
        telemetry.addLine("Left Trigger  = Run Intake (Port 2)");
        telemetry.addLine("Right Trigger = Shoot (Shooter 1 & 2 at 100%)");
        telemetry.addLine("Y Button      = Toggle Intake Direction");
        telemetry.addLine("A Button      = Match Loader Servo DOWN 90°");
        telemetry.addLine("X Button      = Direct Intake OUT (Reverse)");
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

            // ==================================
            // CAMERA APRILTAG AUTO-ALIGNMENT (LB / RB)
            // ==================================

            boolean leftBumper  = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;

            if (leftBumper || rightBumper) {

                String targetTagGroup = leftBumper
                        ? AprilTagVision.RED_AUDIENCE
                        : AprilTagVision.RED_SCORING;

                AprilTagDetection targetTag = robot.vision.findTagByName(targetTagGroup);

                if (targetTag == null && detections != null && !detections.isEmpty()) {
                    targetTag = detections.get(0);
                }

                if (targetTag != null && targetTag.ftcPose != null) {

                    double rangeError   = targetTag.ftcPose.range - DESIRED_DISTANCE_INCHES;
                    double bearingError = targetTag.ftcPose.bearing;

                    drive = rangeError * P_DRIVE_GAIN;
                    turn  = bearingError * P_TURN_GAIN;

                    drive = Math.max(-0.4, Math.min(0.4, drive));
                    turn  = Math.max(-0.3, Math.min(0.3, turn));

                    telemetry.addData("Auto-Align Target",
                            targetTag.metadata != null ? targetTag.metadata.name : "Tag " + targetTag.id);
                    telemetry.addData("Align Distance Error", "%.1f in", rangeError);
                    telemetry.addData("Align Heading Error", "%.1f deg", bearingError);

                } else {
                    drive = 0.0;
                    turn  = 0.0;
                    telemetry.addData("Auto-Align Status", "Target tag not in view!");
                }
            }

            robot.drive(drive, turn);

            // ==================================
            // TURRET ROTATION CONTROL (D-Pad Left / Right)
            // ==================================

            boolean dpadLeft  = gamepad1.dpad_left || gamepad2.dpad_left;
            boolean dpadRight = gamepad1.dpad_right || gamepad2.dpad_right;

            if (dpadLeft) {
                robot.turretSystem.rotateLeft();
            } else if (dpadRight) {
                robot.turretSystem.rotateRight();
            } else {
                robot.turretSystem.stopTurret();
            }

            // ==================================
            // BUTTON Y: TOGGLE INTAKE DIRECTION
            // ==================================

            boolean currentY = gamepad1.y || gamepad2.y;

            if (currentY && !previousY) {
                intakeInDirection = !intakeInDirection;
            }

            previousY = currentY;

            // ==================================
            // BUTTON A: MATCH LOADER SERVO DOWN 90°
            // ==================================

            boolean currentA = gamepad1.a || gamepad2.a;

            if (currentA && !previousA) {
                matchLoaderDown = !matchLoaderDown;
                robot.setMatchLoaderDown(matchLoaderDown);
            }

            previousA = currentA;

            // ==================================
            // TRIGGERS & INTAKE / SHOOTER CONTROLS
            // ==================================

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

            // ==================================
            // TELEMETRY
            // ==================================

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
            telemetry.addData("Match Loader Pos", "%.2f", robot.getMatchLoaderPosition());

            telemetry.addLine("------------------------------");
            telemetry.addData("Parallel Odom Ticks", robot.getParallelEncoderTicks());
            telemetry.addData("Perpendicular Odom Ticks", robot.getPerpendicularEncoderTicks());

            telemetry.addLine("------------------------------");
            telemetry.addData("AprilTags Visible", detections != null ? detections.size() : 0);

            telemetry.update();
        }

        robot.intakeSystem.stopIntake();
        robot.shooter1System.spinDown();
        robot.shooter2System.spinDown();
        robot.turretSystem.stopTurret();
        robot.close();
    }
}
