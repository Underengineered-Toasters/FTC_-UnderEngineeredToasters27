
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
    private final HoldToAlignController alignment = new HoldToAlignController();
    private String alignmentTarget = AprilTagVision.RED_SCORING;


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
            telemetry.addLine("L1 (Hold) = Intake In; R1 (Hold) = Intake Out");
            telemetry.addLine("Controller 1 D-pad Up/Down = Flower Arm (hold)");
            telemetry.addLine("R2 (Hold) = Intake In + Both Shooters");
            telemetry.addLine("Y             = Toggle Shooter Direction");
            telemetry.addLine("Hold L2 = Camera Aim");
            telemetry.update();

            while (!isStarted() && !isStopRequested()) {
                if (gamepad1.x) alignmentTarget = AprilTagVision.BLUE_SCORING;
                if (gamepad1.b) alignmentTarget = AprilTagVision.RED_SCORING;
                telemetry.addLine("Square: Blue / Circle: Red during INIT; hold L2 to aim after START");
                telemetry.addData("Aim target", alignmentTarget);
                telemetry.addData("Camera", robot.vision.getCameraState());
                telemetry.addLine("Robot initialized — Controller 1 D-pad Up/Down: Flower Arm");
                telemetry.addLine("Controller 1 L1: Intake In; R1: Intake Out (hold)");
                telemetry.addLine("Controller 1 R2: Intake In + Both Shooters (hold)");
                telemetry.addData("Flower upper limit", flowerArm.isUpperLimitPressed() ? "PRESSED" : "RELEASED");
                telemetry.addLine("Arm holds position on release; INIT commands raised target (0.0)");
                telemetry.addData("Flower arm target (not measured)", "%.3f", flowerArm.getTargetPosition());
                telemetry.update();
                sleep(50);
            }

            if (isStopRequested()) {
                return;
            }

            while (opModeIsActive()) {

                telemetry.addData("Arm D-pad up/down", "%s / %s", gamepad1.dpad_up, gamepad1.dpad_down);

                List<AprilTagDetection> detections =
                        robot.getAprilTagDetections();

                // Stick forward is negative Y; negative drive power moves this chassis forward.
                double drive = gamepad1.left_stick_y;
                double turn = -gamepad1.right_stick_x;

                // Controller 1 L2 controls alignment independently of the intake bumpers.
                boolean aimHeld = gamepad1.left_trigger > 0.05;
                List<AprilTagDetection> fresh = aimHeld ? robot.vision.getFreshDetections() : null;
                ClusterAlignmentController.Sample sample = null;
                if (fresh != null) {
                    for (AprilTagDetection detection : fresh) {
                        if (!(detection instanceof AprilTagClusterDetection)) continue;
                        AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                        if (cluster.metadata != null && alignmentTarget.equals(cluster.metadata.name)
                                && cluster.ftcPose != null) {
                            sample = new ClusterAlignmentController.Sample(cluster.ftcPose.bearing,
                                    cluster.ftcPose.range, cluster.frameAcquisitionNanoTime);
                            break;
                        }
                    }
                }
                ClusterAlignmentController.Output aim = alignment.update(aimHeld, System.nanoTime(),
                        robot.vision.isReady(), robot.vision.hasFailed(), fresh != null, sample);
                if (aim != null) {
                    drive = 0;
                    turn = aim.turnPower;
                    telemetry.addData("Alignment", aim.state);
                    telemetry.addData("Stable frames", "%d / %d", aim.stableFrames,
                            ClusterAlignmentController.REQUIRED_FRAMES);
                    if (aim.target != null) {
                        telemetry.addData("Aim bearing", "%.1f deg", aim.target.bearingDeg);
                    }
                    telemetry.addLine("Release L2 for manual driving; release/repress to retry timeout");
                } else {
                    telemetry.addData("Alignment", "MANUAL - hold L2 to aim");
                }
                telemetry.addData("Aim target", alignmentTarget);
                telemetry.addData("Camera", robot.vision.getCameraState());

                robot.drive(drive, turn);
                flowerArm.update(gamepad1.dpad_up, gamepad1.dpad_down);

                boolean intakeIn = gamepad1.left_bumper;
                boolean intakeOut = gamepad1.right_bumper;
                // R2 takes priority so intake always feeds inward while shooting.
                boolean shootAndIntake = gamepad1.right_trigger > 0.05;
                if (shootAndIntake) {
                    robot.intakeSystem.intakeIn();
                } else if (intakeIn == intakeOut) {
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
                telemetry.addData("Flower arm target (not measured)", "%.3f", flowerArm.getTargetPosition());
                telemetry.addData("Intake power", "%.2f", robot.intakeSystem.getPower());
                telemetry.update();
            }

        } finally {
            flowerArm.stop();
            robot.shooter.spinDown();
            if (robotInitialized) {
                robot.drive(0, 0);
                robot.intakeSystem.stopIntake();
            }
            robot.close();
        }
    }
}
