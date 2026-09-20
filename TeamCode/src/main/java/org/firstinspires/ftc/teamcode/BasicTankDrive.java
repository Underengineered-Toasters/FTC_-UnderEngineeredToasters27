//imports and packages
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Basic Tank Drive", group = "TeleOp")
public class BasicTankDrive extends LinearOpMode {

    // Make the names match what we have in Driver Station Menu
    private DcMotor left;
    private DcMotor right;

    @Override
    public void runOpMode() {

        // Map the hardware — grabs the motors by the config names
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        //Keep opposite as the motors are oriented the opposite way in tank
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);

        // Optional but recommended: motors actively resist being pushed when stopped
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Use PLAY for driver station
        waitForStart();

        // Runs Teleop till disable
        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;  // stick is inverted: up = negative
            double turn  =  gamepad1.right_stick_x;

            double leftPower  = drive + turn;
            double rightPower = drive - turn;

            // Clip to [-1, 1] in case drive+turn exceeds motor range
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
