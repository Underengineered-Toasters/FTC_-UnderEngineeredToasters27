package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * RED ALLIANCE TELEOP OPMODE (Static Turret Hold)
 *
 * Camera alignment commented out. Turret holds static initial position:
 * - Automatically returns to initial position if drifted > 5° to 30° equivalent ticks.
 * - Manual override via D-Pad Left / Right.
 */
@TeleOp(name = "Red Alliance TeleOp", group = "Red")
public class RedTeleOp extends LinearOpMode {

    private final Robot robot = new Robot();
    private final FlowerArm flowerArm = new FlowerArm();

    // Intake direction state
    private boolean intakeInDirection = true;
    private boolean previousY = false;

    // Match loader servo state
    private boolean matchLoaderDown = false;
    private boolean previousA = false;

    @Override
    public void runOpMode() {

        flowerArm.init(hardwareMap);
        robot.init(hardwareMap);
        telemetry.addLine("Flower arm: hold B = UP, D-pad down = DOWN; release = HOLD");
        telemetry.addData("Flower upper limit", flowerArm.isUpperLimitPressed());

        // Record initial static turret position
        final int initialTurretPos = robot.turretSystem.getCurrentPosition();

        telemetry.addLine("=================================");
        telemetry.addLine("   RED ALLIANCE TELEOP READY     ");
        telemetry.addLine("  Turret Mode: Static Position ");
        telemetry.addLine("  Right Trigger = Shooter 1 & 2  ");
        telemetry.addLine("=================================");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            robot.close();
            return;
        }

        while (opModeIsActive()) {

            flowerArm.update(gamepad1.b, gamepad1.dpad_down);
            telemetry.addData("Flower upper limit", flowerArm.isUpperLimitPressed());
            telemetry.addData("Flower arm target", flowerArm.getTargetPosition());
            List<AprilTagDetection> detections = robot.getAprilTagDetections();

            double drive = -gamepad1.left_stick_y;
            double turn  = -gamepad1.right_stick_x;

            // Apply chassis drive
            robot.drive(drive, turn);

            // =========================================================================
            // STATIC TURRET POSITION HOLD & MANUAL D-PAD OVERRIDE
            // =========================================================================

            boolean dpadLeft  = gamepad1.dpad_left;
            boolean dpadRight = gamepad1.dpad_right;

            if (dpadLeft) {
                robot.turretSystem.rotateLeft();
            } else if (dpadRight) {
                robot.turretSystem.rotateRight();
            } else {
                // Turret is static - check drift from initial position
                int currentTurretPos = robot.turretSystem.getCurrentPosition();
                int errorTicks = currentTurretPos - initialTurretPos;

                // Threshold: ~5 degrees (~150 ticks) to 30 degrees (~900 ticks) drift correction
                int driftThresholdTicks = 150;

                if (Math.abs(errorTicks) > driftThresholdTicks) {
                    // Automatically return to initial static position
                    double returnPower = -Math.signum(errorTicks) * 0.30;
                    robot.turretSystem.setPower(returnPower);
                    telemetry.addData("Turret Hold", "Drift detected (error: %d ticks). Returning to initial pos...", errorTicks);
                } else {
                    robot.turretSystem.stopTurret();
                    telemetry.addData("Turret Hold", "Static (Locked at initial position)");
                }
            }

            // =========================================================================
            // BUTTON Y: TOGGLE INTAKE DIRECTION
            // =========================================================================

            boolean currentY = gamepad1.y;

            if (currentY && !previousY) {
                intakeInDirection = !intakeInDirection;
            }

            previousY = currentY;

            // =========================================================================
            // BUTTON A: MATCH LOADER SERVO DOWN 110° (WITH TOUCH SENSOR HALT)
            // =========================================================================

            boolean currentA = gamepad1.a;

            if (currentA && !previousA) {
                matchLoaderDown = !matchLoaderDown;
                robot.setMatchLoaderDown(matchLoaderDown);
            }

            previousA = currentA;

            robot.update();
            if (robot.isMatchLoaderTouchPressed()) {
                matchLoaderDown = false;
            }

            // =========================================================================
            // TRIGGERS & INTAKE / SHOOTER CONTROLS
            // =========================================================================

            double intakeTrigger  = gamepad1.left_trigger;
            double shooterTrigger = gamepad1.right_trigger;
            boolean buttonX       = gamepad1.x;

            // SHOOTER CONTROL (Right Trigger)
            if (shooterTrigger > 0.05) {
                robot.shooter1System.spinForwardMaxSpeed();
                robot.shooter2System.spinForwardMaxSpeed();
            } else {
                robot.shooter1System.spinDown();
                robot.shooter2System.spinDown();
            }

            // INTAKE CONTROL (Left Trigger / Button X)
            if (buttonX) {
                robot.intakeSystem.intakeOut();
            } else if (intakeTrigger > 0.05) {
                if (intakeInDirection) {
                    robot.intakeSystem.intakeIn();
                } else {
                    robot.intakeSystem.intakeOut();
                }
            } else {
                robot.intakeSystem.stopIntake();
            }

            // =========================================================================
            // TELEMETRY
            // =========================================================================

            telemetry.addLine("------------------------------");
            telemetry.addData("Drive Left Power", "%.2f", robot.getLeftPower());
            telemetry.addData("Drive Right Power", "%.2f", robot.getRightPower());

            telemetry.addLine("------------------------------");
            telemetry.addData("Turret Ticks", "%d (Initial: %d)", robot.turretSystem.getCurrentPosition(), initialTurretPos);
            telemetry.addData("Turret Limit Switch", robot.turretSystem.isLimitReached() ? "ACTIVATED (mag)" : "OPEN");

            telemetry.addLine("------------------------------");
            telemetry.addData("Intake Direction (Y)", intakeInDirection ? "IN" : "OUT");
            telemetry.addData("Shooter Power", "S1: %.2f | S2: %.2f", robot.shooter1System.getCurrentPower(), robot.shooter2System.getCurrentPower());

            telemetry.addLine("------------------------------");
            telemetry.addData("Match Loader Servo (A)", matchLoaderDown ? "DOWN (110°)" : "UP (Rest)");
            telemetry.addData("Touch Sensor", robot.isMatchLoaderTouchPressed() ? "TOUCHED (HALTED)" : "OPEN");

            telemetry.update();
        }

        flowerArm.stop();
        robot.intakeSystem.stopIntake();
        robot.shooter1System.spinDown();
        robot.shooter2System.spinDown();
        robot.turretSystem.stopTurret();
        robot.close();
    }
}