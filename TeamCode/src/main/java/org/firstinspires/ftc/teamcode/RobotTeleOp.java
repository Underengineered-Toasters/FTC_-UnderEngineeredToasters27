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



    // Tracks intake direction
    // true = intake in
    // false = intake out
    private boolean intakeInDirection = true;

    // Used so holding Y doesn't repeatedly toggle
    private boolean previousY = false;

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
            telemetry.addLine("Left Trigger  = Intake + Feeder");
            telemetry.addLine("Right Trigger = Shooter + Feeder");
            telemetry.addLine("Y             = Toggle Intake Direction");
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
            // LB = audience end, RB = far end. If the chosen cluster is not
            // visible, fall back to the alliance's other cluster so a held
            // bumper never dead-ends. The camera must face the shooter
            // direction; if the camera moves to the turret (issue #39) this
            // aims the chassis, not the turret.
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

            // Fallback: chosen cluster not visible, try the other end.
            if (target == null && alignTarget != null) {
                String fallback = alignTarget.equals(AprilTagVision.RED_SCORING)
                        ? AprilTagVision.RED_AUDIENCE
                        : alignTarget.equals(AprilTagVision.RED_AUDIENCE)
                        ? AprilTagVision.RED_SCORING
                        : alignTarget.equals(AprilTagVision.BLUE_SCORING)
                        ? AprilTagVision.BLUE_AUDIENCE
                        : AprilTagVision.BLUE_SCORING;
                target = robot.findAprilTagCluster(fallback);
                if (target != null) {
                    alignTarget = fallback;
                }
            }
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
            // INTAKE DIRECTION TOGGLE
            // ==================================

            boolean currentY = gamepad1.y;

            // Only toggle when Y is initially pressed
            if (currentY && !previousY) {
                intakeInDirection = !intakeInDirection;
            }

            previousY = currentY;


            // ==================================
            // TRIGGERS
            // ==================================

            double intakeTrigger = gamepad1.left_trigger;
            double shooterTrigger = gamepad1.right_trigger;


            // ==================================
            // INTAKE + FEEDER
            // ==================================

            if (intakeTrigger > 0.05) {

                // Intake direction depends on Y toggle
                if (intakeInDirection) {
                    robot.intakeSystem.intakeIn();
                } else {
                    robot.intakeSystem.intakeOut();
                }

                // Push feeder while intake trigger is held
                robot.intakeSystem.pushBall();

            } else if (shooterTrigger > 0.05) {

                // Shooter trigger does not run intake motor,
                // but DOES run the feeder.
                robot.intakeSystem.restFeeder();

            } else {

                robot.intakeSystem.stopIntake();
                robot.intakeSystem.restFeeder();
            }


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

                // Push ball into shooter
                robot.intakeSystem.pushBall();

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
                    "Intake Direction",
                    intakeInDirection ? "IN" : "OUT"
            );

            telemetry.addData(
                    "Intake Power",
                    "%.2f",
                    robot.intakeSystem.getPower()
            );

            telemetry.addData(
                    "Feeder Position",
                    "%.2f",
                    robot.intakeSystem.getFeederPosition()
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
