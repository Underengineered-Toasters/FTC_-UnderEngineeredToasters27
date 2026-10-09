package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Robot TeleOp", group = "Main")
public class RobotTeleOp extends LinearOpMode {

    private final Robot robot = new Robot();

    // Tracks intake direction: true = IN, false = OUT
    private boolean intakeInDirection = true;

    // Edge detector for the Y toggle
    private boolean previousY = false;

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
        telemetry.addLine("Left Trigger  = Run Intake System");
        telemetry.addLine("Right Trigger = Spin Shooter & Feed");
        telemetry.addLine("Y             = Toggle Intake Direction");
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
            // DRIVE (Inverted both signs to fix backward forward & swapped turns)
            // ==================================

            double drive = gamepad1.left_stick_y;
            double turn  = -gamepad1.right_stick_x;

            robot.drive(drive, turn);


            // ==================================
            // INTAKE DIRECTION TOGGLE
            // ==================================

            boolean currentY = gamepad1.y;

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
            // SHOOTER CONTROL
            // ==================================

            boolean isShooting = shooterTrigger > 0.05;

            if (isShooting) {
                robot.shooter.setTargetPower(Shooter.POWER_MID_SHOT);
            } else {
                robot.shooter.spinDown();
            }


            // ==================================
            // INTAKE & FEEDER CONTROL
            // ==================================

            if (isShooting) {
                robot.intakeSystem.stopIntake();
                robot.intakeSystem.pushBall();

            } else if (intakeTrigger > 0.05) {
                if (intakeInDirection) {
                    robot.intakeSystem.intakeIn();
                } else {
                    robot.intakeSystem.intakeOut();
                }

            } else {
                robot.intakeSystem.stopIntake();
                robot.intakeSystem.restFeeder();
            }


            // ==================================
            // UPDATE SUBSYSTEMS
            // ==================================

            robot.update();


            // ==================================
            // APRILTAG VISION
            // ==================================

            List<AprilTagDetection> detections = robot.getAprilTagDetections();


            // ==================================
            // TELEMETRY
            // ==================================

            telemetry.addLine("------------------------------");
            telemetry.addData("Drive Left", "%.2f", robot.getLeftPower());
            telemetry.addData("Drive Right", "%.2f", robot.getRightPower());

            telemetry.addLine("------------------------------");
            telemetry.addData("Intake Direction", intakeInDirection ? "IN" : "OUT");
            telemetry.addData("Intake Power", "%.2f", robot.intakeSystem.getPower());
            telemetry.addData("Feeder Position", "%.2f", robot.intakeSystem.getFeederPosition());

            telemetry.addLine("------------------------------");
            telemetry.addData("Shooter Power", "%.2f", robot.shooter.getCurrentPower());
            telemetry.addData("Shooter Spinning", robot.shooter.isSpinning());

            telemetry.addLine("------------------------------");
            telemetry.addData("AprilTags Visible", detections != null ? detections.size() : 0);

            telemetry.update();
        }

        // =========================
        // Cleanup
        // =========================

        robot.close();
    }
}
