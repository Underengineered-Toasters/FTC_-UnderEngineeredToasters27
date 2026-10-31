
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;

/** Shared controls for the two fixed-alliance TeleOps. */
public abstract class RobotTeleOp extends LinearOpMode {

    private final Robot robot = new Robot();
    private final FlowerArm flowerArm = new FlowerArm();
    private HoldToAlignController alignment = new HoldToAlignController();
    private String alignmentTarget;
    private boolean aimCalibrated;
    private boolean previousCalibrationButton;
    private double learnedBearing;


    private boolean shooterReversed = false;
    private boolean previousY = false;
    private boolean previousPowerDown = false;
    private boolean previousPowerUp = false;
    private long nextAlignmentLog;

    protected abstract String getAllianceTarget();

    @Override
    public void runOpMode() {
        alignmentTarget = getAllianceTarget();
        boolean robotInitialized = false;
        try {
            flowerArm.init(hardwareMap);
            robot.init(hardwareMap);
            robotInitialized = true;

            telemetry.addLine("==============================");
            telemetry.addLine("       ROBOT INITIALIZED");
            telemetry.addLine("==============================");
            telemetry.addLine("Left Stick Y  = Manual Drive");
            telemetry.addLine("Right Stick X = Manual Turn");
            telemetry.addLine("L1 (Hold) = Intake In; R1 (Hold) = Intake Out");
            telemetry.addLine("Controller 1 D-pad Up/Down = Flower Arm (hold)");
            telemetry.addLine("L2 (Hold) = Both Shooters Only");
            telemetry.addLine("R2 (Hold) = Intake In + Both Shooters");
            telemetry.addLine("Y             = Toggle Shooter Direction");
            telemetry.addLine("Hold M2/Cross = Turret Search + Camera Aim");
            telemetry.addLine("D-pad Left/Right = Shooter Power -/+ 1 percentage point");
            telemetry.update();

            while (!isStarted() && !isStopRequested()) {
                AprilTagClusterDetection visible = robot.vision.findCluster(alignmentTarget);
                boolean freshReference = visible != null && visible.ftcPose != null
                        && Double.isFinite(visible.ftcPose.bearing) && visible.frameAcquisitionNanoTime <= System.nanoTime()
                        && System.nanoTime() - visible.frameAcquisitionNanoTime <= ClusterAlignmentController.MAX_FRAME_AGE_NANOS;
                if (gamepad1.y && !previousCalibrationButton && freshReference) {
                    learnedBearing = visible.ftcPose.bearing;
                    alignment = new HoldToAlignController(learnedBearing);
                    aimCalibrated = true;
                }
                previousCalibrationButton = gamepad1.y;
                telemetry.addLine("Alliance audience target is fixed by the selected TeleOp");
                telemetry.addLine("Hold M2/Cross after START to align automatically; Triangle optionally records an aim offset");
                telemetry.addData("Selected hive tags", freshReference ? "DETECTED" : "NOT DETECTED");
                telemetry.addData("Hive aim reference", aimCalibrated ? String.format(java.util.Locale.US,
                        "SAVED %.1f deg", learnedBearing) : "DEFAULT 0 deg - no setup required");
                telemetry.addData("Aim target", alignmentTarget);
                telemetry.addData("Camera", robot.vision.getCameraState());
                telemetry.addLine("D-pad Left/Right tunes shooter power after START; resets on INIT");
                telemetry.addLine("Robot initialized — Controller 1 D-pad Up/Down: Flower Arm");
                telemetry.addLine("Controller 1 L1: Intake In; R1: Intake Out (hold)");
                telemetry.addLine("Controller 1 L2: Both Shooters Only (hold)");
                telemetry.addLine("Controller 1 R2: Intake In + Both Shooters (hold)");
                telemetry.addData("Flower upper limit", flowerArm.isUpperLimitPressed() ? "PRESSED" : "RELEASED");
                telemetry.addLine("Arm holds position on release; INIT commands opposite startup target (1.0)");
                telemetry.addData("Flower arm target (not measured)", "%.3f", flowerArm.getTargetPosition());
                telemetry.update();
                sleep(50);
            }

            if (isStopRequested()) {
                return;
            }
            previousY = gamepad1.y || gamepad2.y;

            while (opModeIsActive()) {

                telemetry.addData("Arm D-pad up/down", "%s / %s", gamepad1.dpad_up, gamepad1.dpad_down);

                List<AprilTagDetection> detections =
                        robot.getAprilTagDetections();

                // Stick forward is negative Y; negative drive power moves this chassis forward.
                double drive = gamepad1.left_stick_y;
                double turn = -gamepad1.right_stick_x;

                // Controller 1 M2/Cross controls alignment independently of the intake bumpers.
                boolean aimHeld = gamepad1.a; // REV M2 defaults to Cross (FTC a).
                List<AprilTagDetection> fresh = aimHeld ? robot.vision.getFreshDetections() : null;
                ClusterAlignmentController.Sample sample = null;
                if (fresh != null) {
                    for (AprilTagDetection detection : fresh) {
                        if (!(detection instanceof AprilTagClusterDetection)) continue;
                        AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                        if (cluster.metadata != null && alignmentTarget.equals(cluster.metadata.name)
                                && cluster.ftcPose != null) {
                            sample = new ClusterAlignmentController.Sample(cluster.ftcPose.bearing,
                                    cluster.ftcPose.range, cluster.frameAcquisitionNanoTime);
                            break;
                        }
                    }
                }
                ClusterAlignmentController.Output aim = alignment.update(aimHeld, System.nanoTime(),
                        robot.vision.isReady(), robot.vision.hasFailed(), fresh != null, sample);
                if (aim != null) {
                    drive = 0;
                    turn = 0;
                    telemetry.addData("Alignment", aim.state);
                    if (aim.state == ClusterAlignmentController.State.SEARCHING) {
                        telemetry.addLine("Searching side-to-side for hive tags; release M2/Cross to stop");
                    }
                    telemetry.addData("Stable frames", "%d / %d", aim.stableFrames,
                            ClusterAlignmentController.REQUIRED_FRAMES);
                    if (aim.target != null) {
                        telemetry.addData("Aim bearing", "%.1f deg", aim.target.bearingDeg);
                    }
                    telemetry.addLine("Release M2/Cross for manual driving; release/repress to retry timeout");
                } else {
                    telemetry.addData("Alignment", "MANUAL - hold M2/Cross to aim");
                }
                telemetry.addData("Aim target", alignmentTarget);
                telemetry.addData("Hive aim reference", aimCalibrated ? learnedBearing : "DEFAULT 0 deg");
                telemetry.addData("Camera", robot.vision.getCameraState());
                robot.turret.applyAlignment(aim);
                telemetry.addData("Turret power", "%.3f", robot.turret.getPower());

                if (System.nanoTime() >= nextAlignmentLog) {
                    StringBuilder observed = new StringBuilder();
                    for (AprilTagDetection detection : detections) {
                        if (detection instanceof AprilTagClusterDetection) {
                            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                            observed.append(cluster.metadata == null ? "unknown-cluster" : cluster.metadata.name);
                            observed.append(cluster.ftcPose == null ? ":no-pose;" :
                                    String.format(java.util.Locale.US, ":bearing=%.1f,range=%.1f;",
                                            cluster.ftcPose.bearing, cluster.ftcPose.range));
                        } else {
                            observed.append(detection.getClass().getSimpleName()).append(';');
                        }
                    }
                    RobotLog.ii("[DEBUG-camera-align]",
                            "M2/Cross=%s state=%s selected=%s camera=%s observed=%s turret=%.3f reference=%s aimBearing=%.1f",
                            aimHeld, aim == null ? "MANUAL" : aim.state,
                            alignmentTarget, robot.vision.getCameraState(), observed.toString(), robot.turret.getPower(),
                            aimCalibrated, learnedBearing);
                    nextAlignmentLog = System.nanoTime() + 500_000_000L;
                }

                robot.drive(drive, turn);
                flowerArm.update(gamepad1.dpad_up, gamepad1.dpad_down);

                boolean intakeIn = gamepad1.left_bumper;
                boolean intakeOut = gamepad1.right_bumper;
                // R2 takes priority so intake always feeds inward while shooting.
                boolean shootAndIntake = gamepad1.right_trigger > 0.05;
                if (shootAndIntake) {
                    robot.intakeSystem.intakeIn();
                } else if (intakeIn == intakeOut) {
                    robot.intakeSystem.stopIntake();
                } else if (intakeIn) {
                    robot.intakeSystem.intakeIn();
                } else {
                    robot.intakeSystem.intakeOut();
                }

                // ==================================
                // SHOOTER CONTROL
                // Y toggles direction.
                // L2 runs shooters only; R2 also commands intake above.
                // ==================================

                boolean powerDown = gamepad1.dpad_left;
                boolean powerUp = gamepad1.dpad_right;
                if (powerUp && !previousPowerUp && !powerDown) robot.shooter.adjustPower(0.01);
                if (powerDown && !previousPowerDown && !powerUp) robot.shooter.adjustPower(-0.01);
                previousPowerUp = powerUp;
                previousPowerDown = powerDown;

                boolean yPressed = gamepad1.y || gamepad2.y;

                if (yPressed && !previousY) {
                    shooterReversed = !shooterReversed;
                }

                previousY = yPressed;

                double shooterTrigger = Math.max(gamepad1.left_trigger, Math.max(
                        gamepad1.right_trigger,
                        gamepad2.right_trigger));

                if (shooterTrigger > 0.05) {
                    robot.shooter.setTargetPower(
                            shooterReversed ? -1.0 : 1.0
                    );
                } else {
                    robot.shooter.spinDown();
                }

                robot.update();

                // ==================================
                // TELEMETRY
                // ==================================

                telemetry.addLine("------------------------------");
                telemetry.addData("Drive Left Power",
                        "%.2f", robot.getLeftPower());
                telemetry.addData("Drive Right Power",
                        "%.2f", robot.getRightPower());

                telemetry.addLine("------------------------------");
                telemetry.addData("Shooter Direction",
                        shooterReversed ? "REVERSE" : "FORWARD");
                telemetry.addData("Shooter Power", "%.2f",
                        robot.shooter.getCurrentPower());
                telemetry.addData("Shooter Hardware", robot.shooter.getHardwareDescription());
                telemetry.addData("Shooter 1 / 2 setting", "%.0f%% / %.0f%%",
                        100 * robot.shooter.getShooter1Limit(), 100 * robot.shooter.getShooter2Limit());
                if (robot.shooter.hasVelocityFeedback()) {
                    telemetry.addData("Shooter Velocity", "%.0f RPM", robot.shooter.getCurrentVelocityRPM());
                } else {
                    telemetry.addData("Shooter Velocity", "Unavailable (CR servos)");
                    telemetry.addData("Shooter 1 / 2 power", "%.2f / %.2f",
                            robot.shooter.getShooter1Power(), robot.shooter.getShooter2Power());
                }
                telemetry.addData("Shooter active (measured/commanded)",
                        robot.shooter.isSpinning());

                telemetry.addLine("------------------------------");
                telemetry.addData("AprilTags Visible",
                        detections != null ? detections.size() : 0);

                telemetry.addData("Flower upper limit", flowerArm.isUpperLimitPressed() ? "PRESSED" : "RELEASED");
                telemetry.addData("Flower arm target (not measured)", "%.3f", flowerArm.getTargetPosition());
                telemetry.addData("Intake power", "%.2f", robot.intakeSystem.getPower());
                telemetry.update();
            }

        } finally {
            flowerArm.stop();
            robot.shooter.spinDown();
            if (robotInitialized) {
                robot.drive(0, 0);
                robot.intakeSystem.stopIntake();
            }
            robot.close();
        }
    }
}
