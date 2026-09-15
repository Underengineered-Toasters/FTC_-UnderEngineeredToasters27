package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive + Shooter", group = "TeleOp")
public class ShooterTeleOp extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private Shooter shooter = new Shooter();

    @Override
    public void runOpMode() {

        leftDrive  = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.init(hardwareMap);

        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            double leftPower  = Math.max(-1.0, Math.min(1.0, drive + turn));
            double rightPower = Math.max(-1.0, Math.min(1.0, drive - turn));

            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Shooter (gamepad1 face buttons — swap to gamepad2 if you add an operator)
            // Y = far shot, B = mid shot, A = close shot, X = spin down/off
            // you want the SAME repeatable speed every time you pick a distance
            if (gamepad1.y) {
                shooter.setTargetVelocity(Shooter.VELOCITY_FAR_SHOT);
            } else if (gamepad1.b) {
                shooter.setTargetVelocity(Shooter.VELOCITY_MID_SHOT);
            } else if (gamepad1.a) {
                shooter.setTargetVelocity(Shooter.VELOCITY_CLOSE_SHOT);
            } else if (gamepad1.x) {
                shooter.spinDown();
            }

            shooter.update(); // actually commands the motor every loop


            telemetry.addData("Shooter target (ticks/s)", shooter.getTargetVelocity());
            telemetry.addData("Shooter actual (ticks/s)", shooter.getCurrentVelocity());
            telemetry.addData("Ready to fire", shooter.isReadyToFire());
            telemetry.update();
        }
    }
}
