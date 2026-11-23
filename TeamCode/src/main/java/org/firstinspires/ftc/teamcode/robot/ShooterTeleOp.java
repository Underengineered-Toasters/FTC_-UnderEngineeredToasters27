package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Drive plus shooter check. Keeps its own motors so it does not need the intake, turret or camera
 * to be configured, but uses the shared {@link Robot} wheel names, directions and mixing.
 */
@TeleOp(name = "Drive + Shooter", group = "TeleOp")
public class ShooterTeleOp extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // Two separate flywheel shooter instances
    private final shooter1 shooter1System = new shooter1(); // ("shooter1")
    private final shooter2 shooter2System = new shooter2(); // ("shooter2")

    @Override
    public void runOpMode() {

        try {
            leftFront = hardwareMap.get(DcMotor.class, Robot.LEFT_FRONT_MOTOR);
            leftBack = hardwareMap.get(DcMotor.class, Robot.LEFT_BACK_MOTOR);
            rightFront = hardwareMap.get(DcMotor.class, Robot.RIGHT_FRONT_MOTOR);
            rightBack = hardwareMap.get(DcMotor.class, Robot.RIGHT_BACK_MOTOR);

            Robot.applyMecanumDirections(leftFront, leftBack, rightFront, rightBack);

            for (DcMotor motor : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
                motor.setPower(0.0);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // Initialize both shooter subsystems
            shooter1System.init(hardwareMap);
            shooter2System.init(hardwareMap);

            telemetry.addLine("Ready.");
            telemetry.addLine("Left Stick Y = Drive, Left Stick X = Strafe");
            telemetry.addLine("Right Stick X = Turn");
            telemetry.addLine("Right Trigger = Run Shooter 1 & 2 FORWARD (100% Speed)");
            telemetry.addLine("Left Trigger  = Run Shooter 1 & 2 REVERSE (100% Speed)");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {

                double drive = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = -gamepad1.right_stick_x;

                Robot.driveMecanum(leftFront, leftBack, rightFront, rightBack, drive, strafe, turn);

                // Dual-Trigger Shooter Controls
                double rightTrigger = gamepad1.right_trigger;
                double leftTrigger = gamepad1.left_trigger;

                if (rightTrigger > 0.05 || gamepad1.y || gamepad1.b || gamepad1.a) {
                    // Right Trigger: Both motors spin FORWARD at 100% max speed
                    shooter1System.spinForwardMaxSpeed();
                    shooter2System.spinForwardMaxSpeed();

                } else if (leftTrigger > 0.05) {
                    // Left Trigger: Both motors spin REVERSE at 100% max speed
                    shooter1System.spinReverseMaxSpeed();
                    shooter2System.spinReverseMaxSpeed();

                } else {
                    // Stop both flywheel motors
                    shooter1System.spinDown();
                    shooter2System.spinDown();
                }

                telemetry.addData("Shooter 1 Power", "%.2f", shooter1System.getCurrentPower());
                telemetry.addData("Shooter 2 Power", "%.2f", shooter2System.getCurrentPower());
                telemetry.addData("Shooter 1 RPM", "%.0f RPM", shooter1System.getCurrentVelocityRPM());
                telemetry.addData("Shooter 2 RPM", "%.0f RPM", shooter2System.getCurrentVelocityRPM());
                telemetry.addData("Drive / strafe / turn", "%.2f / %.2f / %.2f",
                        drive, strafe, turn);

                telemetry.update();
            }
        } finally {
            shooter1System.spinDown();
            shooter2System.spinDown();
            for (DcMotor motor : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
                if (motor != null) {
                    motor.setPower(0.0);
                }
            }
        }
    }
}
