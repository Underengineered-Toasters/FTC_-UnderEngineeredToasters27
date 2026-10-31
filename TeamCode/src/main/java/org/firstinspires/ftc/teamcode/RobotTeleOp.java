
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;

@TeleOp(name = "Robot TeleOp", group = "Main")
public class RobotTeleOp extends LinearOpMode {

    private final Robot robot = new Robot();
    private final FlowerArm flowerArm = new FlowerArm();

    private static final double DESIRED_DISTANCE_INCHES = 12.0;
    private static final double P_DRIVE_GAIN = 0.03;
    private static final double P_TURN_GAIN = 0.02;

    private boolean shooterReversed = false;
    private boolean previousY = false;

    @Override
    public void runOpMode() {
        boolean robotInitialized = false;
        try {
            flowerArm.init(hardwareMap);
            robot.init(hardwareMap);
            robotInitialized = true;

            telemetry.addLine("==============================");
            telemetry.addLine("       ROBOT INITIALIZED");
            telemetry.addLine("==============================");
            telemetry.addLine("Left Stick Y  = Manual Drive");
            telemetry.addLine("Right Stick X = Manual Turn");
            telemetry.addLine("L1 (Hold) = Intake In; L2 (Hold) = Intake Out");
            telemetry.addLine("RB (Hold)     = Align Scoring Tag");
            telemetry.addLine("Controller 1 D-pad Up/Down = Flower Arm (hold)");
            telemetry.addLine("Right Trigger = Run Shooter");
            telemetry.addLine("Y             = Toggle Shooter Direction");
            telemetry.update();

            while (!isStarted() && !isStopRequested()) {
                telemetry.addLine("Robot initialized — Controller 1 D-pad Up/Down: Flower Arm");
                telemetry.addLine("Controller 1 L1: Intake In; L2: Intake Out (hold)");
                telemetry.addData("Flower upper limit", flowerArm.isUpperLimitPressed() ? "PRESSED" : "RELEASED");
                telemetry.addLine("Verify touch switch changes state before moving the arm");
                telemetry.update();
                sleep(50);
            }

            if (isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {

                List<AprilTagDetection> detections =
                        robot.getAprilTagDetections();

                // Stick forward is negative Y; negative drive power moves this chassis forward.
                double drive = gamepad1.left_stick_y;
                double turn = -gamepad1.right_stick_x;

                boolean rightBumper = gamepad1.right_bumper;

                if (rightBumper) {

                    String targetTagGroup = AprilTagVision.RED_SCORING;

                    AprilTagClusterDetection targetTag =
                            robot.vision.findCluster(targetTagGroup);

                    if (targetTag == null && detections != null) {
                        for (AprilTagDetection detection : detections) {
                            if (detection instanceof AprilTagClusterDetection
                                    && detection.ftcPose != null) {
                                targetTag = (AprilTagClusterDetection) detection;
                                break;
                            }
                        }
                    }

                    if (targetTag != null
                            && targetTag.ftcPose != null) {

                        double rangeError =
                                targetTag.ftcPose.range
                                        - DESIRED_DISTANCE_INCHES;

                        double bearingError =
                                targetTag.ftcPose.bearing;

                        drive = rangeError * P_DRIVE_GAIN;
                        turn = bearingError * P_TURN_GAIN;

                        drive = Math.max(
                                -0.4, Math.min(0.4, drive));

                        turn = Math.max(
                                -0.3, Math.min(0.3, turn));

                        telemetry.addData(
                                "Auto-Align Target",
                                targetTag.metadata != null
                                        ? targetTag.metadata.name
                                        : "Visible cluster");

                        telemetry.addData(
                                "Align Distance Error",
                                "%.1f in", rangeError);

                        telemetry.addData(
                                "Align Heading Error",
                                "%.1f deg", bearingError);

                    } else {
                        drive = 0.0;
                        turn = 0.0;

                        telemetry.addData(
                                "Auto-Align Status",
                                "Target tag not in view!");
                    }
                }

                robot.drive(drive, turn);
                flowerArm.update(gamepad1.dpad_up, gamepad1.dpad_down);

                boolean intakeIn = gamepad1.left_bumper;
                boolean intakeOut = gamepad1.left_trigger > 0.05;
                if (intakeIn == intakeOut) {
                    robot.intakeSystem.stopIntake();
                } else if (intakeIn) {
                    robot.intakeSystem.intakeIn();
                } else {
                    robot.intakeSystem.intakeOut();
                }

                // ==================================
                // SHOOTER CONTROL
                // Y toggles direction.
                // Right trigger runs at maximum power.
                // ==================================

                boolean yPressed = gamepad1.y || gamepad2.y;

                if (yPressed && !previousY) {
                    shooterReversed = !shooterReversed;
                }

                previousY = yPressed;

                double rightTrigger = Math.max(
                        gamepad1.right_trigger,
                        gamepad2.right_trigger);

                if (rightTrigger > 0.05) {
                    robot.shooter.setTargetPower(
                            shooterReversed ? -1.0 : 1.0
                    );
                } else {
                    robot.shooter.spinDown();
                }

                robot.update();

                // ==================================
                // TELEMETRY
                // ==================================

                telemetry.addLine("------------------------------");
                telemetry.addData("Drive Left Power",
                        "%.2f", robot.getLeftPower());
                telemetry.addData("Drive Right Power",
                        "%.2f", robot.getRightPower());

                telemetry.addLine("------------------------------");
                telemetry.addData("Shooter Direction",
                        shooterReversed ? "REVERSE" : "FORWARD");
                telemetry.addData("Shooter Power", "%.2f",
                        robot.shooter.getCurrentPower());
                telemetry.addData("Shooter Hardware", robot.shooter.getHardwareDescription());
                if (robot.shooter.hasVelocityFeedback()) {
                    telemetry.addData("Shooter Velocity", "%.0f RPM", robot.shooter.getCurrentVelocityRPM());
                } else {
                    telemetry.addData("Shooter Velocity", "Unavailable (CR servos)");
                }
                telemetry.addData("Shooter active (measured/commanded)",
                        robot.shooter.isSpinning());

                telemetry.addLine("------------------------------");
                telemetry.addData("AprilTags Visible",
                        detections != null ? detections.size() : 0);

                telemetry.addData("Flower upper limit", flowerArm.isUpperLimitPressed() ? "PRESSED" : "RELEASED");
                telemetry.addData("Flower arm power", "%.2f", flowerArm.getPower());
                telemetry.addData("Intake power", "%.2f", robot.intakeSystem.getPower());
                telemetry.update();
            }

        } finally {
            flowerArm.stop();
            robot.shooter.spinDown();
            if (robotInitialized) robot.intakeSystem.stopIntake();
            robot.close();
        }
    }
}
