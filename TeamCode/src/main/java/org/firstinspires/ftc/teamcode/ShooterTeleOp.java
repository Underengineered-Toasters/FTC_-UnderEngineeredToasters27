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
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);
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

            // Use setTargetPower and POWER_* constants
            if (gamepad1.y) {
                shooter.setTargetPower(Shooter.POWER_FAR_SHOT);
            } else if (gamepad1.b) {
                shooter.setTargetPower(Shooter.POWER_MID_SHOT);
            } else if (gamepad1.a) {
                shooter.setTargetPower(Shooter.POWER_CLOSE_SHOT);
            } else if (gamepad1.x) {
                shooter.spinDown();
            }

            shooter.update();

            telemetry.addData("Shooter Power", "%.2f", shooter.getCurrentPower());
            telemetry.addData("Drive Left", "%.2f", leftPower);
            telemetry.addData("Drive Right", "%.2f", rightPower);
            telemetry.update();
        }
    }
}
