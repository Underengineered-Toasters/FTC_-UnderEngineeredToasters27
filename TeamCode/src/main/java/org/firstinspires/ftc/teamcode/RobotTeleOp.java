package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;

@TeleOp(name = "Robot TeleOp", group = "Main")
public class RobotTeleOp extends LinearOpMode {
    private final Robot robot = new Robot();
    private final AlignLog alignLog = new AlignLog();
    private static final double ALIGN_KP = 0.05;
    private static final double ALIGN_MAX_TURN = 0.6;
    // Smallest turn power that actually rotates the chassis (overcomes friction)
    private static final double ALIGN_MIN_TURN = 0.15;
    // Deadband wide enough that the chassis settles inside it instead of hunting
    private static final double ALIGN_TOLERANCE_DEG = 2.0;
    // Alliance selected during INIT; defaults to RED
    private String alliance = "RED";




    @Override
    public void runOpMode() {

        // =========================
        // Initialize Robot
        // =========================
        robot.init(hardwareMap);

        alignLog.open();

        // Alliance select: X = BLUE, B = RED (default RED) before START
        while (opModeInInit()) {

            if (gamepad1.x) {
                alliance = "BLUE";
            } else if (gamepad1.b) {
                alliance = "RED";
            }

            telemetry.addLine("==============================");
            telemetry.addLine("       ROBOT INITIALIZED");
            telemetry.addLine("==============================");
            telemetry.addData("Alliance", alliance + " (X = Blue, B = Red)");
            telemetry.addLine("");
            telemetry.addLine("Left Stick Y  = Drive");
            telemetry.addLine("Right Stick X = Turn");
            telemetry.addLine("");
            telemetry.addLine("LB (hold)     = Align audience CELL");
            telemetry.addLine("RB (hold)     = Align far CELL");
            telemetry.addLine("");
            telemetry.addLine("Left Trigger  = Feeder only");
            telemetry.addLine("Right Trigger = Shooter + Feeder");
            telemetry.update();
        }

        if (isStopRequested()) {
            robot.close();
            return;
        }

        // =========================
        // Main Loop
        // =========================

        while (opModeIsActive()) {

            // ==================================
            // DRIVE
            // ==================================

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Hold-to-align on the CELL AprilTag cluster.
            // The camera must face the shooter direction; if the camera moves
            // to the turret (issue #39) this aims the chassis, not the turret.
            String alignTarget = null;
            if (gamepad1.left_bumper) {
                alignTarget = "RED".equals(alliance)
                        ? AprilTagVision.RED_AUDIENCE
                        : AprilTagVision.BLUE_AUDIENCE;
            } else if (gamepad1.right_bumper) {
                alignTarget = "RED".equals(alliance)
                        ? AprilTagVision.RED_SCORING
                        : AprilTagVision.BLUE_SCORING;
            }

            AprilTagClusterDetection target =
                    alignTarget == null ? null : robot.findAprilTagCluster(alignTarget);
            boolean aligned = false;

            // Positive bearing = target LEFT; positive turn rotates RIGHT.
            if (target != null && target.ftcPose != null) {
                double bearing = target.ftcPose.bearing;
                if (Math.abs(bearing) < ALIGN_TOLERANCE_DEG) {
                    turn = 0;
                    aligned = true;
                } else {
                    double power = Math.max(ALIGN_MIN_TURN,
                            Math.min(ALIGN_MAX_TURN, Math.abs(ALIGN_KP * bearing)));
                    turn = -Math.signum(bearing) * power;
                }
            }

            robot.drive(drive, turn);

            alignLog.row(
                    alliance,
                    alignTarget,
                    target != null && target.ftcPose != null
                            ? target.ftcPose.bearing : null,
                    turn,
                    -gamepad1.left_stick_y,
                    gamepad1.right_stick_x,
                    drive,
                    robot.getLeftPower(),
                    robot.getRightPower(),
                    robot.getLeftVelocity(),
                    robot.getRightVelocity()
            );




            // ==================================
            // TRIGGERS
            // ==================================

            double intakeTrigger = gamepad1.left_trigger;
            double shooterTrigger = gamepad1.right_trigger;


            // ==================================
            // FEEDER (left trigger = feeder only, issue #46)
            // ==================================

            if (intakeTrigger > 0.05) {

                // Left trigger runs only the feeder into the shooter;
                // the intake motor stays where it is.
                robot.intakeSystem.pushBall();

            } else if (shooterTrigger > 0.05) {

                // Right trigger runs the feeder while the shooter spins.
                robot.intakeSystem.pushBall();

            } else {

                robot.intakeSystem.restFeeder();
            }

            // ==================================
            // INTAKE
            // ==================================

            // The intake motor has no trigger binding; keep it stopped.
            robot.intakeSystem.stopIntake();




            // ==================================
            // SHOOTER
            // ==================================

            if (shooterTrigger > 0.05) {

                // Use the shooter trigger as the shooter command.
                //
                // This currently uses the MID shot velocity.
                // Change this constant later after tuning.
                robot.shooter.setTargetVelocity(
                        Shooter.VELOCITY_MID_SHOT
                );
                // Feeder runs from the FEEDER block above on the same trigger.

            } else {

                robot.shooter.spinDown();
            }


            // ==================================
            // UPDATE SHOOTER
            // ==================================

            robot.update();


            // ==================================
            // APRILTAG VISION
            // ==================================

            List<AprilTagDetection> detections =
                    robot.getAprilTagDetections();


            // ==================================
            // TELEMETRY
            // ==================================

            telemetry.addLine("------------------------------");

            telemetry.addData(
                    "Drive Left",
                    "%.2f",
                    robot.getLeftPower()
            );

            telemetry.addData(
                    "Drive Right",
                    "%.2f",
                    robot.getRightPower()
            );

            telemetry.addLine("------------------------------");


            telemetry.addData(
                    "Intake Power",
                    "%.2f",
                    robot.intakeSystem.getPower()
            );

            telemetry.addData(
                    "Feeder Power",
                    "%.2f",
                    robot.intakeSystem.getFeederPower()
            );

            telemetry.addLine("------------------------------");

            telemetry.addData(
                    "Shooter Target",
                    "%.0f ticks/s",
                    robot.shooter.getTargetVelocity()
            );

            telemetry.addData(
                    "Shooter Actual",
                    "%.0f ticks/s",
                    robot.shooter.getCurrentVelocity()
            );

            telemetry.addData(
                    "Shooter Ready",
                    robot.shooter.isReadyToFire()
            );

            telemetry.addLine("------------------------------");

            telemetry.addData("Alliance", alliance);
            telemetry.addData(
                    "Align Target",
                    alignTarget == null ? "off" : alignTarget
            );
            telemetry.addData(
                    "Align Bearing",
                    target != null && target.ftcPose != null
                            ? String.format("%.1f deg", target.ftcPose.bearing)
                            : "not visible"
            );
            telemetry.addData("Aligned", aligned);

            telemetry.addData(
                    "AprilTag Targets",
                    detections.size()
            );

            telemetry.update();
        }

        // =========================
        // Cleanup
        // =========================

        alignLog.close();
        robot.close();
    }
}
