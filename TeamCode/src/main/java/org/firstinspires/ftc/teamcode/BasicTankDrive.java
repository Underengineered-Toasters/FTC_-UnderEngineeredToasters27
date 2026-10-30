package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Basic Tank Drive", group = "TeleOp")
public class BasicTankDrive extends LinearOpMode {

    private DcMotor left;
    private DcMotor right;

    @Override
    public void runOpMode() {

        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Inverted signs so forward is forward, turn left is left
            double drive = gamepad1.left_stick_y;
            double turn  = -gamepad1.right_stick_x;

            double leftPower  = drive + turn;
            double rightPower = drive - turn;

            // Clip to [-1, 1] range
            leftPower  = Math.max(-1.0, Math.min(1.0, leftPower));
            rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

            left.setPower(leftPower);
            right.setPower(rightPower);

            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.update();
        }
    }
}