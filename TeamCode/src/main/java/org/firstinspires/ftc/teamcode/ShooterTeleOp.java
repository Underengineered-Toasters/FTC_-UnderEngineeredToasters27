package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Drive + Shooter", group = "TeleOp")
public class ShooterTeleOp extends LinearOpMode {

    private DcMotor left;
    private DcMotor right;

    // Two separate flywheel shooter instances
    private final shooter1 shooter1System = new shooter1(); // Motor Port 2 ("shooter1")
    private final shooter2 shooter2System = new shooter2(); // Motor Port 3 ("shooter2")

    @Override
    public void runOpMode() {

        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize both shooter subsystems
        shooter1System.init(hardwareMap);
        shooter2System.init(hardwareMap);

        telemetry.addLine("Ready.");
        telemetry.addLine("Left Stick Y = Drive");
        telemetry.addLine("Right Stick X = Turn");
        telemetry.addLine("Right Trigger = Run Shooter 1 & 2 FORWARD (100% Speed)");
        telemetry.addLine("Left Trigger  = Run Shooter 1 & 2 REVERSE (100% Speed)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double leftPower = Math.max(-1.0, Math.min(1.0, drive + turn));
            double rightPower = Math.max(-1.0, Math.min(1.0, drive - turn));

            left.setPower(leftPower);
            right.setPower(rightPower);

            // Dual-Trigger Shooter Controls
            double rightTrigger = gamepad1.right_trigger;
            double leftTrigger  = gamepad1.left_trigger;

            if (rightTrigger > 0.05 || gamepad1.y || gamepad1.b || gamepad1.a) {
                // Right Trigger: Both motors spin FORWARD at 100% max speed
                shooter1System.spinForwardMaxSpeed();
                shooter2System.spinForwardMaxSpeed();

            } else if (leftTrigger > 0.05) {
                // Left Trigger: Both motors spin REVERSE at 100% max speed
                shooter1System.spinReverseMaxSpeed();
                shooter2System.spinReverseMaxSpeed();

            } else {
                // Stop both flywheel motors
                shooter1System.spinDown();
                shooter2System.spinDown();
            }

            telemetry.addData("Shooter 1 Power", "%.2f", shooter1System.getCurrentPower());
            telemetry.addData("Shooter 2 Power", "%.2f", shooter2System.getCurrentPower());
            telemetry.addData("Shooter 1 RPM", "%.0f RPM", shooter1System.getCurrentVelocityRPM());
            telemetry.addData("Shooter 2 RPM", "%.0f RPM", shooter2System.getCurrentVelocityRPM());
            telemetry.addData("Drive Left", "%.2f", leftPower);
            telemetry.addData("Drive Right", "%.2f", rightPower);

            telemetry.update();
        }

        shooter1System.spinDown();
        shooter2System.spinDown();
    }
}
