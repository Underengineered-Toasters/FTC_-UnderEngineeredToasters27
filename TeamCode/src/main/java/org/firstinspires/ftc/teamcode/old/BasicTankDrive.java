package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Four-wheel drive check: left stick drives and strafes, right stick turns.
 *
 * <p>Kept from the two-motor tank days for quick manual checks, but the chassis is mecanum now, so
 * it drives the four {@link Robot} wheel names through the shared {@code Robot.driveMecanum}
 * mixing. Forward is forward for every wheel and positive turn is clockwise, the same convention
 * as {@link Robot#drive}.
 */
@TeleOp(name = "Basic Tank Drive", group = "TeleOp")
public class BasicTankDrive extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

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

            telemetry.addLine("Left stick Y = drive, X = strafe");
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                double drive = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = -gamepad1.right_stick_x;

                Robot.driveMecanum(leftFront, leftBack, rightFront, rightBack, drive, strafe, turn);

                telemetry.addData("Drive / strafe / turn", "%.2f / %.2f / %.2f", drive, strafe, turn);
                telemetry.update();
            }
        } finally {
            for (DcMotor motor : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
                if (motor != null) {
                    motor.setPower(0.0);
                }
            }
        }
    }
}
