
package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive + Shooter", group = "TeleOp")
public class ShooterTeleOp extends LinearOpMode {

    private DcMotor left;
    private DcMotor right;
    private final Shooter shooter = new Shooter();

    private boolean shooterReversed = false;
    private boolean previousY = false;

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
        telemetry.addLine("Left Stick Y = Drive");
        telemetry.addLine("Right Stick X = Turn");
        telemetry.addLine("Right Trigger = Run Shooter");
        telemetry.addLine("Y = Toggle Shooter Direction");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double leftPower = Math.max(
                    -1.0, Math.min(1.0, drive + turn));

            double rightPower = Math.max(
                    -1.0, Math.min(1.0, drive - turn));

            left.setPower(leftPower);
            right.setPower(rightPower);

            // Toggle direction on a new Y press.
            boolean yPressed = gamepad1.y;

            if (yPressed && !previousY) {
                shooterReversed = !shooterReversed;
            }

            previousY = yPressed;

            if (gamepad1.right_trigger > 0.05) {
                if (shooterReversed) {
                    shooter.spinReverseMaxSpeed();
                } else {
                    shooter.spinForwardMaxSpeed();
                }
            } else {
                shooter.spinDown();
            }

            shooter.update();

            telemetry.addData("Shooter Direction",
                    shooterReversed ? "REVERSE" : "FORWARD");
            telemetry.addData("Shooter Power", "%.2f",
                    shooter.getCurrentPower());
            telemetry.addData("Shooter Hardware", shooter.getHardwareDescription());
            if (shooter.hasVelocityFeedback()) {
                telemetry.addData("Shooter Velocity", "%.0f RPM", shooter.getCurrentVelocityRPM());
            } else {
                telemetry.addData("Shooter Velocity", "Unavailable (CR servos)");
                telemetry.addData("Shooter 1 / 2 power", "%.2f / %.2f",
                        shooter.getShooter1Power(), shooter.getShooter2Power());
            }
            telemetry.addData("Shooter active (measured/commanded)",
                    shooter.isSpinning());
            telemetry.addData("Drive Left", "%.2f", leftPower);
            telemetry.addData("Drive Right", "%.2f", rightPower);

            telemetry.update();
        }

        shooter.spinDown();
    }
}
