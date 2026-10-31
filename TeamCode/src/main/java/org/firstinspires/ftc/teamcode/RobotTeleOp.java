
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Robot TeleOp", group = "Main")
public class RobotTeleOp extends LinearOpMode {

    private final Robot robot = new Robot();
    private final FlowerArm flowerArm = new FlowerArm();


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
            telemetry.update();

            while (!isStarted() && !isStopRequested()) {
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

                List<AprilTagDetection> detections =
                        robot.getAprilTagDetections();

                // Stick forward is negative Y; negative drive power moves this chassis forward.
                double drive = gamepad1.left_stick_y;
                double turn = -gamepad1.right_stick_x;

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
            if (robotInitialized) robot.intakeSystem.stopIntake();
            robot.close();
        }
    }
}
