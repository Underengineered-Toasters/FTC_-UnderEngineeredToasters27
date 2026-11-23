package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Mecanum drivetrain bring-up test.
 *
 * <p>Left stick forward/back drives, left stick left/right strafes, right stick left/right rotates.
 * Uses the same wheel names, direction normalization and mixing as {@link Robot}, so a wheel that
 * turns the wrong way here is a wiring/mounting problem, not a sign bug in an OpMode.
 */
@TeleOp(name = "Mecanum Drive Test", group = "Testing")
public class MecanumDriveTest extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    private static final double DRIVE_SPEED = 1.0;
    private static final double STICK_DEADBAND = 0.05;

    @Override
    public void runOpMode() {
        try {
            leftFront = hardwareMap.get(DcMotor.class, Robot.LEFT_FRONT_MOTOR);
            leftBack = hardwareMap.get(DcMotor.class, Robot.LEFT_BACK_MOTOR);
            rightFront = hardwareMap.get(DcMotor.class, Robot.RIGHT_FRONT_MOTOR);
            rightBack = hardwareMap.get(DcMotor.class, Robot.RIGHT_BACK_MOTOR);

            Robot.applyMecanumDirections(leftFront, leftBack, rightFront, rightBack);

            for (DcMotor motor : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
                motor.setPower(0);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            telemetry.addLine("Left stick up/down: forward/backward");
            telemetry.addLine("Left stick left/right: strafe");
            telemetry.addLine("Right stick left/right: rotate");
            telemetry.addData("Maximum power", DRIVE_SPEED);
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                double forward = applyDeadband(-gamepad1.left_stick_y);
                double strafe = applyDeadband(gamepad1.left_stick_x);
                double rotation = applyDeadband(-gamepad1.right_stick_x);

                double[] powers = Robot.mecanumWheelPowers(
                        forward * DRIVE_SPEED, strafe * DRIVE_SPEED, rotation * DRIVE_SPEED);

                leftFront.setPower(powers[0]);
                leftBack.setPower(powers[1]);
                rightFront.setPower(powers[2]);
                rightBack.setPower(powers[3]);

                telemetry.addData("Forward input", "%.2f", forward);
                telemetry.addData("Strafe input", "%.2f", strafe);
                telemetry.addData("Rotation input", "%.2f", rotation);
                telemetry.addLine("--------------------------------");
                telemetry.addData("Left front / back", "%.2f / %.2f", powers[0], powers[1]);
                telemetry.addData("Right front / back", "%.2f / %.2f", powers[2], powers[3]);
                telemetry.update();

                idle();
            }
        } finally {
            for (DcMotor motor : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
                if (motor != null) {
                    motor.setPower(0);
                }
            }
        }
    }

    private double applyDeadband(double input) {
        return Math.abs(input) < STICK_DEADBAND ? 0.0 : input;
    }
}
