package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum Drive Test", group = "Testing")
public class MecanumDriveTest extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;

    private static final double DRIVE_SPEED = 1.0;
    private static final double STICK_DEADBAND = 0.05;

    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");

        // Retains your latest direction settings.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] motors = {
                frontLeft, frontRight, backLeft, backRight
        };

        for (DcMotor motor : motors) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addLine("Left stick up/down: forward/backward");
        telemetry.addLine("Left stick left/right: rotate");
        telemetry.addLine("Right stick left/right: strafe");
        telemetry.addData("Maximum power", DRIVE_SPEED);
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                double forward = applyDeadband(-gamepad1.left_stick_y);
                double strafe = applyDeadband(gamepad1.left_stick_x);
                double rotation = applyDeadband(gamepad1.right_stick_x);

                // Start each loop with no requested movement.
                resetRequestedPowers();

                // Add the contribution from each movement function.
                moveForwardBackward(forward);
                strafeLeftRight(strafe);
                rotate(rotation);

                // Normalize, limit speed, and command the motors once.
                applyMotorPowers();

                telemetry.addData("Forward input", "%.2f", forward);
                telemetry.addData("Strafe input", "%.2f", strafe);
                telemetry.addData("Rotation input", "%.2f", rotation);

                telemetry.addData("Front left power", "%.2f", frontLeftPower);
                telemetry.addData("Front right power", "%.2f", frontRightPower);
                telemetry.addData("Back left power", "%.2f", backLeftPower);
                telemetry.addData("Back right power", "%.2f", backRightPower);
                telemetry.update();

                idle();
            }
        } finally {
            for (DcMotor motor : motors) {
                motor.setPower(0);
            }
        }
    }

    // Positive input requests forward; negative requests backward.
    private void moveForwardBackward(double power) {
        frontLeftPower += power;
        frontRightPower += power;
        backLeftPower += power;
        backRightPower += power;
    }

    // Positive input requests right strafe; negative requests left strafe.
    private void strafeLeftRight(double power) {
        frontLeftPower += power;
        frontRightPower -= power;
        backLeftPower -= power;
        backRightPower += power;
    }

    // Positive input requests clockwise rotation; negative requests counterclockwise.
    private void rotate(double power) {
        frontLeftPower += power;
        frontRightPower -= power;
        backLeftPower += power;
        backRightPower -= power;
    }

    private void resetRequestedPowers() {
        frontLeftPower = 0;
        frontRightPower = 0;
        backLeftPower = 0;
        backRightPower = 0;
    }

    private void applyMotorPowers() {
        double largestPower = Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        double denominator = Math.max(1.0, largestPower);

        frontLeftPower = frontLeftPower / denominator * DRIVE_SPEED;
        frontRightPower = frontRightPower / denominator * DRIVE_SPEED;
        backLeftPower = backLeftPower / denominator * DRIVE_SPEED;
        backRightPower = backRightPower / denominator * DRIVE_SPEED;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    private double applyDeadband(double input) {
        return Math.abs(input) < STICK_DEADBAND ? 0.0 : input;
    }
}