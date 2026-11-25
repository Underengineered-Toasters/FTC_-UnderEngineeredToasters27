package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.robot.subsystems.shooter1;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter2;

/** Safe manual test for the turret motor. */
@TeleOp(name = "Turret Test", group = "Testing")
public class TurretTest extends LinearOpMode {
    private static final double TURRET_POWER = 0.20;
    // 20T motor pinion driving a 110T turret gear = 5.5 motor turns for one turret turn.
    private static final double MOTOR_TURNS_PER_TURRET_TURN = 110.0 / 20.0;

    @Override
    public void runOpMode() {
        DcMotor turret = hardwareMap.get(DcMotor.class, "turret");
        TouchSensor magneticLimit = hardwareMap.tryGet(TouchSensor.class, "mag");
        // These wrappers support SPARK Minis configured as CRServos on servo ports.
        shooter1 shooterOne = new shooter1();
        shooter2 shooterTwo = new shooter2();
        shooterOne.init(hardwareMap);
        shooterTwo.init(hardwareMap);

        turret.setPower(0.0);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("D-pad LEFT: rotate left | D-pad RIGHT: rotate right");
        telemetry.addLine("Left trigger: run both shooter motors | Y: reverse shooter direction");
        telemetry.addLine("Release D-pad to stop. Power is limited to 0.20.");
        telemetry.addLine("Gear reduction: 5.5 motor turns per turret revolution.");
        telemetry.update();
        waitForStart();

        boolean shooterReversed = false;
        boolean previousY = false;

        try {
            while (opModeIsActive()) {
                double power = gamepad1.dpad_left ? -TURRET_POWER
                        : gamepad1.dpad_right ? TURRET_POWER : 0.0;
                turret.setPower(power);

                if (gamepad1.y && !previousY) {
                    shooterReversed = !shooterReversed;
                }
                previousY = gamepad1.y;

                if (gamepad1.left_trigger > 0.05) {
                    if (shooterReversed) {
                        shooterOne.spinReverseMaxSpeed();
                        shooterTwo.spinReverseMaxSpeed();
                    } else {
                        shooterOne.spinForwardMaxSpeed();
                        shooterTwo.spinForwardMaxSpeed();
                    }
                } else {
                    shooterOne.spinDown();
                    shooterTwo.spinDown();
                }

                telemetry.addData("Commanded power", "%.2f", power);
                telemetry.addData("Motor encoder ticks", turret.getCurrentPosition());
                telemetry.addData("Motor turns / turret turn", "%.2f", MOTOR_TURNS_PER_TURRET_TURN);
                telemetry.addData("Magnetic limit", magneticLimit != null && magneticLimit.isPressed()
                        ? "PRESSED" : "open / not configured");
                telemetry.addData("Shooter direction (Y)", shooterReversed ? "REVERSE" : "FORWARD");
                telemetry.addData("Shooter 1 power (LT)", "%.2f", shooterOne.getCurrentPower());
                telemetry.addData("Shooter 2 power (LT)", "%.2f", shooterTwo.getCurrentPower());
                telemetry.update();
                idle();
            }
        } finally {
            turret.setPower(0.0);
            shooterOne.spinDown();
            shooterTwo.spinDown();
        }
    }
}
