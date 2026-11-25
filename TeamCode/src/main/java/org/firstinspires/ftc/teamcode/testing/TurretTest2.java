package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.subsystems.turret;

/**
 * TurretTest2 OpMode to test the new rotateTo(deg) function on the turret subsystem.
 * 
 * Gamepad Controls:
 * - X Button: Rotate to 0°
 * - A Button: Rotate to 90°
 * - B Button: Rotate to 180°
 * - Y Button: Rotate to 270°
 * - D-Pad Left / Right: Manual override
 */
@TeleOp(name = "Turret Test 2 (rotateTo)", group = "Testing")
public class TurretTest2 extends LinearOpMode {

    private final turret turretSubsystem = new turret();

    @Override
    public void runOpMode() {
        turretSubsystem.init(hardwareMap);

        telemetry.addLine("=================================");
        telemetry.addLine("   TURRET TEST 2 (rotateTo)      ");
        telemetry.addLine("  X Button: Rotate to 0°         ");
        telemetry.addLine("  A Button: Rotate to 90°        ");
        telemetry.addLine("  B Button: Rotate to 180°       ");
        telemetry.addLine("  Y Button: Rotate to 270°       ");
        telemetry.addLine("  D-Pad Left/Right: Manual       ");
        telemetry.addLine("=================================");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                turretSubsystem.rotateTo(0.0);
                telemetry.addData("Commanded Action", "Rotate to 0°");
            } else if (gamepad1.a) {
                turretSubsystem.rotateTo(90.0);
                telemetry.addData("Commanded Action", "Rotate to 90°");
            } else if (gamepad1.b) {
                turretSubsystem.rotateTo(180.0);
                telemetry.addData("Commanded Action", "Rotate to 180°");
            } else if (gamepad1.y) {
                turretSubsystem.rotateTo(270.0);
                telemetry.addData("Commanded Action", "Rotate to 270°");
            } else if (gamepad1.dpad_left) {
                turretSubsystem.rotateLeft();
                telemetry.addData("Commanded Action", "Manual Rotate Left");
            } else if (gamepad1.dpad_right) {
                turretSubsystem.rotateRight();
                telemetry.addData("Commanded Action", "Manual Rotate Right");
            } else {
                // If no button is pressed, hold position
                turretSubsystem.stopTurret();
                telemetry.addData("Commanded Action", "Holding / Stopped");
            }

            telemetry.addData("Current Turret Ticks", turretSubsystem.getCurrentPosition());
            telemetry.addData("Limit Switch", turretSubsystem.isLimitReached() ? "PRESSED" : "OPEN");
            telemetry.update();
            idle();
        }

        turretSubsystem.stopTurret();
    }
}
