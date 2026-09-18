package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive + Shooter", group = "TeleOp")
public class ShooterTeleOp extends LinearOpMode {

    private DcMotor left;
    private DcMotor right;
    private Shooter shooter = new Shooter();

    @Override
    public void runOpMode() {

        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter.init(hardwareMap);

        telemetry.addLine("Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            double leftPower  = Math.max(-1.0, Math.min(1.0, drive + turn));
            double rightPower = Math.max(-1.0, Math.min(1.0, drive - turn));

            left.setPower(leftPower);
            right.setPower(rightPower);

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
