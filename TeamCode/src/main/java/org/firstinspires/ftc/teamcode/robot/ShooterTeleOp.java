package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.subsystems.shooter1;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter2;

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
        DualShooterSubsystem shooter = DualShooterSubsystem.tryCreate(hardwareMap, false);
        if (shooter == null) {
            shooter1System.init(hardwareMap);
            shooter2System.init(hardwareMap);
        }

        telemetry.addLine("Ready.");
        telemetry.addLine("Left Stick Y = Drive");
        telemetry.addLine("Right Stick X = Turn");
        telemetry.addLine("Right Trigger = Run both shooters forward");
        telemetry.addLine("Left Trigger  = Run both shooters in reverse");
        telemetry.addData("Shooter Control", shooter != null ? "Encoder PIDF" : "Open-loop power");
        telemetry.update();

        waitForStart();

        boolean reversed = false;
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

            boolean forwardRequested = rightTrigger > 0.05 || gamepad1.y || gamepad1.b || gamepad1.a;
            boolean reverseRequested = !forwardRequested && leftTrigger > 0.05;
            if (shooter != null) {
                boolean running = forwardRequested || reverseRequested;
                if (running && reversed != reverseRequested) {
                    shooter.stop();
                    shooter.setReverse(reverseRequested);
                    reversed = reverseRequested;
                }
                double maxWheelRPM = DualShooterSubsystem.MAX_MOTOR_RPM
                        * DualShooterSubsystem.GEAR_RATIO_WHEEL_OVER_MOTOR;
                shooter.setTargetRPM(running ? maxWheelRPM : 0.0);
                shooter.update();
            } else if (forwardRequested) {
                // Right Trigger: Both motors spin FORWARD at 100% max speed
                shooter1System.spinForwardMaxSpeed();
                shooter2System.spinForwardMaxSpeed();

            } else if (reverseRequested) {
                // Left Trigger: Both motors spin REVERSE at 100% max speed
                shooter1System.spinReverseMaxSpeed();
                shooter2System.spinReverseMaxSpeed();

            } else {
                // Stop both flywheel motors
                shooter1System.spinDown();
                shooter2System.spinDown();
            }

            if (shooter != null) {
                telemetry.addData("Shooter Target", "%.0f RPM", shooter.getTargetWheelRPM());
                telemetry.addData("Shooter 1 / 2 RPM", "%.0f / %.0f",
                        shooter.getMasterWheelRPM(), shooter.getFollowerWheelRPM());
                telemetry.addData("Shooter 1 / 2 Power", "%.2f / %.2f",
                        shooter.getMasterPower(), shooter.getFollowerPower());
            } else {
                telemetry.addData("Shooter 1 / 2 Power", "%.2f / %.2f",
                        shooter1System.getCurrentPower(), shooter2System.getCurrentPower());
            }
            telemetry.addData("Drive Left", "%.2f", leftPower);
            telemetry.addData("Drive Right", "%.2f", rightPower);

            telemetry.update();
        }

        if (shooter != null) {
            shooter.stop();
        } else {
            shooter1System.spinDown();
            shooter2System.spinDown();
        }
    }
}