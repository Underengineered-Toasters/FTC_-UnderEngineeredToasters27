package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Dual-Wheel Auto-Aim Shooter", group="TeleOp")
@SuppressWarnings("unused") // FTC discovers OpModes through their annotations.
public class TeleOpDualShooterAuto extends LinearOpMode {

    // Field is 144" x 144". Center HIVE coordinate is (72, 72)
    public static final double HIVE_X_INCHES = 72.0;
    public static final double HIVE_Y_INCHES = 72.0;

    @Override
    public void runOpMode() {
        DualShooterSubsystem shooter;

        // Configure these two motor names in your Driver Station phone configuration:
        // "leftShooter" and "rightShooter"
        shooter = new DualShooterSubsystem(hardwareMap, "leftShooter", "rightShooter");

        telemetry.addLine("Dual 6000 RPM Shooter Initialized.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // ==========================================================
            // 1. READ ODOMETRY POSITION (INCHES)
            // Replace with your real odometry estimate:
            // e.g. Pose2d currentPose = drive.getPoseEstimate();
            // ==========================================================
            double robotX = 36.0; // Current robot X
            double robotY = 36.0; // Current robot Y

            // ==========================================================
            // 2. COMPUTE DISTANCE (x) TO GOAL
            // ==========================================================
            double dx = HIVE_X_INCHES - robotX;
            double dy = HIVE_Y_INCHES - robotY;
            double horizontalDistance = Math.hypot(dx, dy); // x in inches

            // ==========================================================
            // 3. CONTROLS
            // ==========================================================
            if (gamepad1.right_bumper) {
                // Calculate required RPM dynamically based on position
                double targetRPM = DualShooterKinematics.getTargetRPM(horizontalDistance);
                shooter.setTargetRPM(targetRPM);
            } else if (gamepad1.left_bumper) {
                // Turn off
                shooter.setTargetRPM(0.0);
            }

            // ==========================================================
            // 4. RUN PIDF CYCLE
            // ==========================================================
            shooter.update();

            // ==========================================================
            // 5. TELEMETRY
            // ==========================================================
            telemetry.addData("Horizontal Dist (in)", "%.2f in", horizontalDistance);
            telemetry.addData("Target RPM", "%.0f RPM", shooter.getTargetWheelRPM());
            telemetry.addData("Actual RPM", "%.0f RPM", shooter.getCurrentWheelRPM());

            if (shooter.isReadyToShoot(80.0)) {
                telemetry.addData("STATUS", ">>> LOCKED ON SPEED - FIRE! <<<");
            } else if (shooter.getTargetWheelRPM() > 0) {
                telemetry.addData("STATUS", "Ramping up to speed...");
            } else {
                telemetry.addData("STATUS", "Idle (Off)");
            }

            telemetry.update();
        }
    }
}