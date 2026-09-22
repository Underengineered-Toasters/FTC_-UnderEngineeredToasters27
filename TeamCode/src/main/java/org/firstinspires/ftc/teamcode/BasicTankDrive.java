package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Basic Tank Drive", group = "TeleOp")
public class BasicTankDrive extends LinearOpMode {

    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;  // stick is inverted: up = negative
            double turn  =  gamepad1.right_stick_x;

            robot.drivetrain.drive(drive, turn);

            telemetry.addData("Stick Y (drive)", "%.2f", drive);
            telemetry.addData("Stick X (turn)", "%.2f", turn);
            telemetry.update();
        }
    }
}
