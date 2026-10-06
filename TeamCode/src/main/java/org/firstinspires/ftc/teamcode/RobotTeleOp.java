package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Robot TeleOp", group = "Main")
public class RobotTeleOp extends LinearOpMode {

    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {

        // =========================
        // Initialize Robot
        // =========================

        robot.init(hardwareMap);

        telemetry.addLine("==============================");
        telemetry.addLine("       ROBOT INITIALIZED");
        telemetry.addLine("==============================");
        telemetry.addLine("");
        telemetry.addLine("Left Stick Y  = Drive");
        telemetry.addLine("Right Stick X = Turn");
        telemetry.addLine("");
        telemetry.addLine("Left Trigger  = Feeder");
        telemetry.addLine("Right Trigger = Shooter + Feeder");
        telemetry.update();

        waitForStart();

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

            robot.drive(drive, turn);


            // ==================================
            // TRIGGERS
            // ==================================

            double feederTrigger = gamepad1.left_trigger;
            double shooterTrigger = gamepad1.right_trigger;


            // ==================================
            // FEEDER
            // ==================================

            robot.intakeSystem.stopIntake();

            if (feederTrigger > 0.05 || shooterTrigger > 0.05) {
                robot.intakeSystem.pushBall();
            } else {
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

            telemetry.addData(
                    "AprilTags Visible",
                    detections.size()
            );

            telemetry.update();
        }

        // =========================
        // Cleanup
        // =========================

        robot.close();
    }
}
