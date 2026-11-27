package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.robot.DualShooterKinematics.BLUE_HIVE_1_POSITION;
import static org.firstinspires.ftc.teamcode.robot.DualShooterKinematics.BLUE_HIVE_2_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.messages.KinematicsModel;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
@TeleOp(name = "Robot Incremental Shooter TeleOp", group = "Main")
public class RobotIncrementalShooterTeleOp extends LinearOpMode {

    public static final double TRANSFER_MOTOR_POWER = 0.80;
    private final Robot robot = new Robot();

    // Intake direction state
    private boolean intakeInDirection = true;
    private boolean previousY = false;

    // Match loader servo state
    private boolean matchLoaderDown = false;
    private boolean previousA = false;

    private static final double DESIRED_DISTANCE_INCHES = 12.0;
    private static final double P_DRIVE_GAIN = 0.03;
    private static final double P_TURN_GAIN  = 0.02;

    private static final double SHOT_SECONDS = 30.0;
    private static final double COOLDOWN_SECONDS = 60.0;

    // The sheet's 120 multiplier corresponds to a 0.50 slip factor.
    private static final double SHEET_GRAVITY_IN_PER_SEC2 = 2.0 * 193.044;
    private static final double SHEET_SLIP_FACTOR = 0.50;
    private static final double SHEET_TARGET_HEIGHT_INCHES = 59.55;

    // Robot and hive coordinates in inches from Shot Calculator, rows 5–8.
    private static final KinematicsModel[] SHOT_PRESETS = {
            createShotPreset(new Vector2d(83.5, 131.5), BLUE_HIVE_2_POSITION),
            createShotPreset(new Vector2d(129.0, 131.5), BLUE_HIVE_2_POSITION),
            createShotPreset(new Vector2d(80.5, 10.3), BLUE_HIVE_1_POSITION),
            createShotPreset(new Vector2d(94.5, 15.0), BLUE_HIVE_1_POSITION)
    };

    private static KinematicsModel createShotPreset(Vector2d botPose, Vector2d hivePose) {
        return new KinematicsModel(botPose, hivePose, SHEET_TARGET_HEIGHT_INCHES,
                DualShooterKinematics.SHOOTER_HEIGHT_INCHES, DualShooterKinematics.LAUNCH_ANGLE_DEG);
    }

    private enum ShotState {
        READY, SHOOTING, COOLDOWN
    }

    // Estimated flywheel RPM at full power. Calibrate using a tachometer.
    public static double SERVO_SHOOTER_FULL_POWER_RPM = 6000.0;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        boolean servoShooters = (hardwareMap.tryGet(CRServo.class, "shooter1") != null
                || hardwareMap.tryGet(CRServo.class, "shooter") != null)
                && hardwareMap.tryGet(CRServo.class, "shooter2") != null;
        DualShooterSubsystem shooter = DualShooterSubsystem.tryCreate(hardwareMap, true);
        String shooterStatus = shooter != null ? "Encoder PIDF"
                : servoShooters ? "Servo-port power (no RPM feedback)"
                : "Open-loop power (check shooter port configuration)";

        telemetry.addLine("==============================");
        telemetry.addLine("       ROBOT INITIALIZED");
        telemetry.addLine("==============================");
        telemetry.addLine("Left Stick Y  = Manual Drive");
        telemetry.addLine("Right Stick X = Manual Turn");
        telemetry.addLine("LB / RB       = Camera AprilTag Auto-Align");
        telemetry.addLine("D-Pad Left/Right = Rotate Turret 360°");
        telemetry.addLine("Left Trigger  = Run Intake (Port 2)");
        telemetry.addLine("D-Pad Up/Down = Select Shot Preset");
        telemetry.addLine("Hold Right Trigger = Shoot for up to 30 s, then cool down for 60 s");
        telemetry.addData("Shooter Status", shooterStatus);
        telemetry.addLine("Y Button      = Toggle Intake Direction");
        telemetry.addLine("A Button      = Match Loader Servo DOWN 90°");
        telemetry.addLine("X Button      = Reverse Intake");
        telemetry.update();

        DcMotor transfer = hardwareMap.tryGet(DcMotor.class, "transfer");
        if (transfer != null) {
            transfer.setPower(0.0);
            transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        waitForStart();

        if (isStopRequested()) {
            stopShooter(shooter);
            robot.close();
            return;
        }

        ElapsedTime shotTimer = new ElapsedTime();
        ShotState shotState = ShotState.READY;
        int shotIndex = 0;
        boolean wasShootPressed = false;
        boolean previousUp = false;
        boolean previousDown = false;
        while (opModeIsActive()) {

            List<AprilTagDetection> detections = robot.getAprilTagDetections();

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // ==================================
            // CAMERA APRILTAG AUTO-ALIGNMENT (LB / RB)
            // ==================================

            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;

            if (leftBumper || rightBumper) {

                String targetTagGroup = leftBumper
                        ? AprilTagVision.RED_AUDIENCE
                        : AprilTagVision.RED_SCORING;

                AprilTagDetection targetTag = robot.vision.findTagByName(targetTagGroup);

                if (targetTag == null && detections != null && !detections.isEmpty()) {
                    targetTag = detections.get(0);
                }

                if (targetTag != null && targetTag.ftcPose != null) {

                    double rangeError = targetTag.ftcPose.range - DESIRED_DISTANCE_INCHES;
                    double bearingError = targetTag.ftcPose.bearing;

                    drive = rangeError * P_DRIVE_GAIN;
                    turn = bearingError * P_TURN_GAIN;
                    strafe = 0.0;

                    drive = Math.max(-0.4, Math.min(0.4, drive));
                    turn = Math.max(-0.3, Math.min(0.3, turn));

                    telemetry.addData("Auto-Align Target",
                            AprilTagTargets.nameOf(targetTag));
                    telemetry.addData("Align Distance Error", "%.1f in", rangeError);
                    telemetry.addData("Align Heading Error", "%.1f deg", bearingError);

                } else {
                    drive = 0.0;
                    turn = 0.0;
                    strafe = 0.0;
                    telemetry.addData("Auto-Align Status", "Target tag not in view!");
                }
            }

            robot.drive(drive, strafe, turn);

            // ==================================
            // TURRET ROTATION CONTROL (D-Pad Left / Right)
            // ==================================

            boolean dpadLeft = gamepad1.dpad_left || gamepad2.dpad_left;
            boolean dpadRight = gamepad1.dpad_right || gamepad2.dpad_right;

            if (dpadLeft) {
                robot.turretSystem.rotateLeft();
            } else if (dpadRight) {
                robot.turretSystem.rotateRight();
            } else {
                robot.turretSystem.stopTurret();
            }

            // ==================================
            // BUTTON Y: TOGGLE INTAKE DIRECTION
            // ==================================

            boolean currentY = gamepad1.y || gamepad2.y;

            if (currentY && !previousY) {
                intakeInDirection = !intakeInDirection;
            }

            previousY = currentY;

            // ==================================
            // BUTTON A: MATCH LOADER SERVO DOWN 90°
            // ==================================

            boolean currentA = gamepad1.a || gamepad2.a;

            if (currentA && !previousA) {
                matchLoaderDown = !matchLoaderDown;
                robot.setMatchLoaderDown(matchLoaderDown);
            }

            previousA = currentA;

            // ==================================
            // TRIGGERS & INTAKE / SHOOTER CONTROLS
            // ==================================

            double intakeTrigger = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
            double shooterTrigger = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
            boolean buttonX = gamepad1.x || gamepad2.x;

            // SHOOTER AND TRANSFER CONTROL (Right Trigger)
            boolean up = gamepad1.dpad_up || gamepad2.dpad_up;
            boolean down = gamepad1.dpad_down || gamepad2.dpad_down;
            boolean shootPressed = shooterTrigger > 0.05;
            if (shotState == ShotState.COOLDOWN && shotTimer.seconds() >= COOLDOWN_SECONDS) {
                shotState = ShotState.READY;
            }
            if (shotState == ShotState.READY) {
                if (up && !previousUp) {
                    shotIndex = (shotIndex + 1) % SHOT_PRESETS.length;
                } else if (down && !previousDown) {
                    shotIndex = (shotIndex + SHOT_PRESETS.length - 1) % SHOT_PRESETS.length;
                }
            }
            previousUp = up;
            previousDown = down;

            KinematicsModel preset = SHOT_PRESETS[shotIndex];
            final double calculatedRPM = DualShooterKinematics.getTargetRPM(
                    preset, SHEET_GRAVITY_IN_PER_SEC2, SHEET_SLIP_FACTOR);
            double fullPowerRPM = shooter == null ? SERVO_SHOOTER_FULL_POWER_RPM
                    : DualShooterSubsystem.MAX_MOTOR_RPM * DualShooterSubsystem.GEAR_RATIO_WHEEL_OVER_MOTOR;
            boolean validTarget = Double.isFinite(calculatedRPM) && calculatedRPM > 0.0
                    && Double.isFinite(fullPowerRPM) && calculatedRPM <= fullPowerRPM;
            if (shotState == ShotState.SHOOTING
                    && (!shootPressed || shotTimer.seconds() >= SHOT_SECONDS || !validTarget)) {
                shotState = ShotState.COOLDOWN;
                shotTimer.reset();
            } else if (shotState == ShotState.READY && shootPressed && !wasShootPressed
                    && validTarget) {
                shotState = ShotState.SHOOTING;
                shotTimer.reset();
            }
            wasShootPressed = shootPressed;

            boolean shooting = shotState == ShotState.SHOOTING;
            double targetRPM = shooting ? calculatedRPM : 0.0;
            double finalShooterPower = 0;
            if (shooter != null) {
                shooter.setTargetRPM(targetRPM);
                shooter.update();
            } else {
                finalShooterPower = shooting ? targetRPM / fullPowerRPM : 0.0;
                robot.shooter1System.setReversePower(finalShooterPower);
                robot.shooter2System.setReversePower(finalShooterPower);
            }

            if (transfer != null) {
                transfer.setPower(shooting ? TRANSFER_MOTOR_POWER : 0.0);
            }

            // INTAKE CONTROL (Left Trigger / Button X)
            if (buttonX) {
                // Direct Intake OUT
                robot.intakeSystem.intakeOut();

            } else if (intakeTrigger > 0.05) {
                // Run Intake according to current toggle direction
                if (intakeInDirection) {
                    robot.intakeSystem.intakeIn();
                } else {
                    robot.intakeSystem.intakeOut();
                }

            } else {
                robot.intakeSystem.stopIntake();
            }

            robot.update();
            robot.updateOdometry();

            // ==================================
            // TELEMETRY
            // ==================================

            telemetry.addLine("------------------------------");
            telemetry.addData("Drive Left Power", "%.2f", robot.getLeftPower());
            telemetry.addData("Drive Right Power", "%.2f", robot.getRightPower());

            telemetry.addLine("------------------------------");
            telemetry.addData("Turret Power", "%.2f", robot.turretSystem.getPower());
            telemetry.addData("Turret Limit Switch", robot.turretSystem.isLimitReached() ? "ACTIVATED (mag)" : "OPEN");

            telemetry.addLine("------------------------------");
            telemetry.addData("Intake Direction (Y)", intakeInDirection ? "IN" : "OUT");
            telemetry.addData("Intake Power", "%.2f", robot.intakeSystem.getPower());

            telemetry.addLine("------------------------------");
            telemetry.addData("Shooter Status", shooterStatus);
            telemetry.addData("Shot Preset", "%d/%d", shotIndex + 1, SHOT_PRESETS.length);
            telemetry.addData("Calculated Shooter RPM", "%.2f RPM", calculatedRPM);
            Vector2d botPose = preset.getBotPose();
            Vector2d hivePose = preset.getHivePose();
            telemetry.addData("Bot Position (in)", "(%.1f, %.1f)", botPose.x, botPose.y);
            telemetry.addData("Hive Position (in)", "(%.1f, %.1f)", hivePose.x, hivePose.y);
            telemetry.addData("Horizontal Hive Distance", "%.2f in", preset.getHorizontalDistanceInches());
            telemetry.addData("Shot State", shotState);
            telemetry.addData("Final Shooter Power", "%.2f", finalShooterPower);
            if (shotState != ShotState.READY) {
                double phaseSeconds = shooting ? SHOT_SECONDS : COOLDOWN_SECONDS;
                telemetry.addData("Time Remaining", "%.1f s",
                        Math.max(0.0, phaseSeconds - shotTimer.seconds()));
            }
            if (!validTarget) {
                telemetry.addLine("Calculated RPM is outside the configured wheel speed range.");
            }
            if (shooter != null) {
                telemetry.addData("Shooter Target", "%.2f RPM", shooter.getTargetWheelRPM());
                telemetry.addData("Shooter 1 / 2 RPM", "%.0f / %.0f",
                        shooter.getMasterWheelRPM(), shooter.getFollowerWheelRPM());
                telemetry.addData("Shooter Power", "1: %.2f | 2: %.2f",
                        shooter.getMasterPower(), shooter.getFollowerPower());
            } else {
                telemetry.addData("Shooter Target (open loop)", "%.2f RPM", targetRPM);
                telemetry.addData("Shooter Power", "1: %.2f | 2: %.2f",
                        robot.shooter1System.getCurrentPower(), robot.shooter2System.getCurrentPower());
            }

            telemetry.addLine("------------------------------");
            telemetry.addData("Match Loader Servo (A)", matchLoaderDown ? "DOWN (90°)" : "UP (Rest)");
            telemetry.addData("Match Loader Pos", "%.2f", robot.getMatchLoaderPosition());

            telemetry.addLine("------------------------------");
            telemetry.addData("IMU", robot.isImuReady()
                    ? "OK (" + robot.getImuName() + ")"
                    : "NOT FOUND - check robot config");
            telemetry.addData("X Odom", robot.isStrafePodMoving() ? "in motion" : "stopped");
            telemetry.addData("Y Odom", robot.isForwardPodMoving() ? "in motion" : "stopped");
            telemetry.addData("Parallel Odom Ticks", robot.getParallelEncoderTicks());
            telemetry.addData("Perpendicular Odom Ticks", robot.getPerpendicularEncoderTicks());
            telemetry.addData("Field X (in)", "%.1f", robot.getFieldX());
            telemetry.addData("Field Y (in)", "%.1f", robot.getFieldY());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(robot.getHeadingRadians()));
            telemetry.addData("Distance Traveled (in)", "%.1f", robot.getDistanceTraveled());

            telemetry.addLine("------------------------------");
            telemetry.addData("AprilTags Visible", detections != null ? detections.size() : 0);

            telemetry.update();
        }

        robot.intakeSystem.stopIntake();
        stopShooter(shooter);
        if (transfer != null) {
            transfer.setPower(0.0);
        }
        robot.turretSystem.stopTurret();
        robot.close();
    }

    private void stopShooter(DualShooterSubsystem shooter) {
        if (shooter != null) {
            shooter.stop();
        } else {
            robot.shooter1System.spinDown();
            robot.shooter2System.spinDown();
        }
    }
}