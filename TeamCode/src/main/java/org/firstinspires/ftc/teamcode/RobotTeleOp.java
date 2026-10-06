package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.io.File;
import java.util.List;

@TeleOp(name = "Robot TeleOp", group = "Main")
public class RobotTeleOp extends LinearOpMode {
    private static final double TRIGGER_DEADBAND = 0.05;
    private static final long LOG_INTERVAL_MS = 100;
    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        RobotActionLog actionLog = new RobotActionLog(new File(AppUtil.FIRST_FOLDER, "debug_logs"));
        long startNanos = System.nanoTime();
        String feederPwm = robot.intakeSystem.getFeederPwmStatus();
        try {
            recordActions(actionLog, startNanos, "init", 0, 0, 0, feederPwm);
            telemetry.addLine("ROBOT INITIALIZED");
            telemetry.addLine("Left Stick Y = Drive; Right Stick X = Turn");
            telemetry.addLine("Left Trigger = Feeder");
            telemetry.addLine("Right Trigger = Shooter + Feeder");
            telemetry.addData("Feeder Connection", robot.intakeSystem.getFeederConnection());
            telemetry.addData("CSV Log", actionLog.getStatus());
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;

            long nextLogMs = 0;
            boolean previousLeftActive = false;
            boolean previousRightActive = false;
            boolean firstSample = true;
            while (opModeIsActive()) {
                double drive = -gamepad1.left_stick_y;
                double turn = gamepad1.right_stick_x;
                robot.drive(drive, turn);
                double leftTrigger = gamepad1.left_trigger;
                double rightTrigger = gamepad1.right_trigger;
                boolean leftActive = leftTrigger > TRIGGER_DEADBAND;
                boolean rightActive = rightTrigger > TRIGGER_DEADBAND;
                double feederRequested = (leftActive || rightActive) ? intake.FEEDER_PUSH : intake.FEEDER_REST;

                robot.intakeSystem.stopIntake();
                // The right trigger always commands the shooter and feeder together.
                if (rightActive) {
                    robot.shooter.setTargetVelocity(Shooter.VELOCITY_MID_SHOT);
                    robot.intakeSystem.pushBall();
                } else {
                    robot.shooter.spinDown();
                    if (leftActive) {
                        robot.intakeSystem.pushBall();
                    } else {
                        robot.intakeSystem.restFeeder();
                    }
                }
                robot.update();

                long elapsedMs = (System.nanoTime() - startNanos) / 1_000_000;
                boolean changed = leftActive != previousLeftActive || rightActive != previousRightActive;
                if (firstSample || changed || elapsedMs >= nextLogMs) {
                    feederPwm = robot.intakeSystem.getFeederPwmStatus();
                    String event = firstSample ? "start" : (changed ? "trigger_change" : "sample");
                    recordActions(actionLog, startNanos, event, leftTrigger, rightTrigger,
                            feederRequested, feederPwm);
                    nextLogMs = elapsedMs + LOG_INTERVAL_MS;
                    previousLeftActive = leftActive;
                    previousRightActive = rightActive;
                    firstSample = false;
                }

                List<AprilTagDetection> detections = robot.getAprilTagDetections();
                telemetry.addData("Drive Left", "%.2f", robot.getLeftPower());
                telemetry.addData("Drive Right", "%.2f", robot.getRightPower());
                telemetry.addData("Left Trigger", "%.2f", leftTrigger);
                telemetry.addData("Right Trigger", "%.2f", rightTrigger);
                telemetry.addData("Intake Power", "%.2f", robot.intakeSystem.getPower());
                telemetry.addData("Feeder Requested Power", "%.2f", feederRequested);
                telemetry.addData("Feeder Power", "%.2f", robot.intakeSystem.getFeederPower());
                telemetry.addData("Feeder PWM Enabled", feederPwm);
                telemetry.addData("Shooter Target", "%.0f ticks/s", robot.shooter.getTargetVelocity());
                telemetry.addData("Shooter Actual", "%.0f ticks/s", robot.shooter.getCurrentVelocity());
                telemetry.addData("Shooter Ready", robot.shooter.isReadyToFire());
                telemetry.addData("AprilTags Visible", detections.size());
                telemetry.addData("CSV Log", actionLog.getStatus());
                telemetry.update();
            }
        } finally {
            try {
                robot.close();
                recordActions(actionLog, startNanos, "stop", gamepad1.left_trigger,
                        gamepad1.right_trigger, 0, robot.intakeSystem.getFeederPwmStatus());
            } finally {
                actionLog.close();
            }
        }
    }

    private void recordActions(RobotActionLog log, long startNanos, String event,
                               double leftTrigger, double rightTrigger,
                               double feederRequested, String feederPwm) {
        log.record((System.nanoTime() - startNanos) / 1_000_000, event, leftTrigger, rightTrigger,
                feederRequested, robot.intakeSystem.getFeederPower(), feederPwm,
                robot.intakeSystem.getFeederPort(), robot.intakeSystem.getFeederConnection(),
                robot.shooter.getTargetVelocity(), robot.shooter.getCurrentVelocity(),
                robot.intakeSystem.getPower(), robot.getLeftPower(), robot.getRightPower());
    }
}