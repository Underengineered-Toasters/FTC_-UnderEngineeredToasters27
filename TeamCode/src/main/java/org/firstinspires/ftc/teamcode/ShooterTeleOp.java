package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Drive + Shooter", group = "TeleOp")
public class ShooterTeleOp extends LinearOpMode {

    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            robot.drivetrain.drive(drive, turn);

            // Shooter (gamepad1 face buttons — swap to gamepad2 if you add an operator)
            // Y = far shot, B = mid shot, A = close shot, X = spin down/off
            // you want the SAME repeatable speed every time you pick a distance
            if (gamepad1.y) {
                robot.shooter.setTargetVelocity(Shooter.VELOCITY_FAR_SHOT);
            } else if (gamepad1.b) {
                robot.shooter.setTargetVelocity(Shooter.VELOCITY_MID_SHOT);
            } else if (gamepad1.a) {
                robot.shooter.setTargetVelocity(Shooter.VELOCITY_CLOSE_SHOT);
            } else if (gamepad1.x) {
                robot.shooter.spinDown();
            }

            robot.update(); // actually commands the flywheel every loop

            telemetry.addData("Shooter target (ticks/s)", robot.shooter.getTargetVelocity());
            telemetry.addData("Shooter actual (ticks/s)", robot.shooter.getCurrentVelocity());
            telemetry.addData("Ready to fire", robot.shooter.isReadyToFire());
            telemetry.update();
        }
    }
}
