package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.messages.KinematicsModel;

@Config
@TeleOp(name = "Dual Shooter Odometry Test", group = "Testing")
@SuppressWarnings("unused") // FTC discovers OpModes through their annotations.
public class DualShooterTestTeleOp extends LinearOpMode {
    private static final double FIELD_SIZE_INCHES = 144.0;
    private static final double FIELD_CENTER_INCHES = FIELD_SIZE_INCHES / 2.0;
    private static final double ROBOT_SIZE_INCHES = 18.0;
    private static final double ROBOT_HALF_SIZE_INCHES = ROBOT_SIZE_INCHES / 2.0;
    private static final double HIVE_TARGET_INSET_FROM_EDGE_INCHES = 18.571914;
    private static final double SHOOTER_OFFSET_FROM_ROBOT_CENTER_INCHES = 3.6;
    private static final double STARTING_HEADING_RADIANS = -Math.PI / 2.0;
    private static final double TRIGGER_DEADBAND = 0.05;

    public static int BURST_BALL_COUNT = 3;
    public static double FEED_PULSE_SECONDS = 0.20;
    public static double READY_TOLERANCE_RPM = 80.0;
    public static double READY_TIMEOUT_SECONDS = 4.0;

    private enum BurstState {
        IDLE,
        SPINUP,
        FEEDING,
        RECOVERING
    }

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Robot robot = new Robot();
        robot.initDriveAndOdometry(hardwareMap);
        robot.initMatchLoader(hardwareMap);
        boolean feederAvailable = robot.matchLoaderServo != null;

        DualShooterSubsystem shooter =
                new DualShooterSubsystem(hardwareMap, "shooter1", "shooter2");

        // Pedro coordinates use the field corner as (0,0). With the robot flush in that
        // corner, the centered IMU is 9 inches from both walls. Forward faces the lower wall.
        robot.setPose(
                ROBOT_HALF_SIZE_INCHES,
                ROBOT_HALF_SIZE_INCHES,
                STARTING_HEADING_RADIANS
        );

        double hiveYInches = HIVE_TARGET_INSET_FROM_EDGE_INCHES;
        boolean previousDpadUp = false;
        boolean previousDpadDown = false;
        boolean previousBurstButton = false;
        boolean previousCancelButton = false;

        BurstState burstState = BurstState.IDLE;
        ElapsedTime burstTimer = new ElapsedTime();
        double burstTargetRPM = 0.0;
        int burstBallsRemaining = 0;
        String burstStatus = "Idle";

        telemetry.addLine("Dual shooter + odometry burst test");
        telemetry.addLine("D-pad up/down during INIT: choose far/near field-edge HIVE");
        telemetry.addLine("Right trigger: manual RPM");
        telemetry.addLine("Left trigger: RPM from odometry distance");
        telemetry.addLine("Drive: left stick; turn: right stick");
        telemetry.addLine("Hold A to start a burst at the selected RPM; B cancels");
        telemetry.addLine("Release both triggers to stop shooter after a burst");
        telemetry.addData("Feeder", feederAvailable ? "matchLoader ready" : "matchLoader NOT FOUND");
        telemetry.addData("IMU", robot.isImuReady() ? robot.getImuName() : "NOT FOUND");
        telemetry.update();

        try {
            while (!isStarted() && !isStopRequested()) {
                boolean dpadUp = gamepad1.dpad_up;
                boolean dpadDown = gamepad1.dpad_down;

                if (dpadUp && !previousDpadUp) {
                    hiveYInches = FIELD_SIZE_INCHES - HIVE_TARGET_INSET_FROM_EDGE_INCHES;
                } else if (dpadDown && !previousDpadDown) {
                    hiveYInches = HIVE_TARGET_INSET_FROM_EDGE_INCHES;
                }

                previousDpadUp = dpadUp;
                previousDpadDown = dpadDown;

                telemetry.addData("Selected HIVE Y", "%.0f in", hiveYInches);
                telemetry.update();
                idle();
            }

            waitForStart();

            while (opModeIsActive()) {
                double drive = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = -gamepad1.right_stick_x;
                robot.drive(drive, strafe, turn);
                robot.updateOdometry();

                double hiveXInches = FIELD_CENTER_INCHES;
                double robotCenterXInches = robot.getFieldX();
                double robotCenterYInches = robot.getFieldY();
                double headingRadians = robot.getHeadingRadians();

                // The shooter is 3.6 inches from center, opposite the intake-facing direction.
                // At the starting heading, that places it toward +Y and closer to the upper hive.
                double shooterXInches = robotCenterXInches
                        - SHOOTER_OFFSET_FROM_ROBOT_CENTER_INCHES * Math.cos(headingRadians);
                double shooterYInches = robotCenterYInches
                        - SHOOTER_OFFSET_FROM_ROBOT_CENTER_INCHES * Math.sin(headingRadians);

                double robotCenterDistanceToHiveInches = robot.getDistanceTo(hiveXInches, hiveYInches);
                KinematicsModel kinematicsModel = new KinematicsModel(
                        new Vector2d(shooterXInches, shooterYInches),
                        new Vector2d(hiveXInches, hiveYInches));
                double shooterDistanceInches = kinematicsModel.getHorizontalDistanceInches();

                double requestedRPM = 0.0;
                String controlMode = "Stopped";

                if (gamepad1.right_trigger > TRIGGER_DEADBAND) {
                    double maxWheelRPM = DualShooterSubsystem.MAX_MOTOR_RPM
                            * DualShooterSubsystem.GEAR_RATIO_WHEEL_OVER_MOTOR;
                    requestedRPM = maxWheelRPM * gamepad1.right_trigger;
                    controlMode = "Manual trigger";
                } else if (gamepad1.left_trigger > TRIGGER_DEADBAND) {
                    requestedRPM = DualShooterKinematics.getTargetRPM(kinematicsModel);
                    controlMode = "Odometry distance";
                }

                boolean burstButton = gamepad1.a;
                boolean cancelButton = gamepad1.b;
                if (burstButton && !previousBurstButton && burstState == BurstState.IDLE) {
                    if (!feederAvailable) {
                        burstStatus = "Cannot start: matchLoader not configured";
                    } else if (requestedRPM <= 0.0) {
                        burstStatus = "Select RPM with a trigger first";
                    } else if (BURST_BALL_COUNT <= 0) {
                        burstStatus = "Set burst count above zero";
                    } else {
                        burstTargetRPM = requestedRPM;
                        burstBallsRemaining = BURST_BALL_COUNT;
                        burstState = BurstState.SPINUP;
                        burstStatus = "Waiting for shooter speed";
                        burstTimer.reset();
                    }
                }

                if (cancelButton && !previousCancelButton && burstState != BurstState.IDLE) {
                    robot.setMatchLoaderDown(false);
                    burstState = BurstState.IDLE;
                    burstBallsRemaining = 0;
                    burstStatus = "Canceled";
                }
                previousBurstButton = burstButton;
                previousCancelButton = cancelButton;

                boolean burstActive = burstState != BurstState.IDLE;
                double targetRPM = burstActive ? burstTargetRPM : requestedRPM;
                if (burstActive) {
                    controlMode = "Burst (RPM held)";
                }

                shooter.setTargetRPM(targetRPM);
                shooter.update();

                boolean shooterReady = shooter.isReadyToShoot(READY_TOLERANCE_RPM);
                switch (burstState) {
                    case SPINUP:
                    case RECOVERING:
                        robot.setMatchLoaderDown(false);
                        if (shooterReady) {
                            robot.setMatchLoaderDown(true);
                            if (robot.getMatchLoaderPosition()
                                    < Robot.MATCH_LOADER_DOWN_110 - 0.001) {
                                burstState = BurstState.IDLE;
                                burstBallsRemaining = 0;
                                burstStatus = "Feed blocked by limit switch";
                            } else {
                                burstState = BurstState.FEEDING;
                                burstStatus = "Feeding ball";
                                burstTimer.reset();
                            }
                        } else if (burstTimer.seconds() >= READY_TIMEOUT_SECONDS) {
                            robot.setMatchLoaderDown(false);
                            burstState = BurstState.IDLE;
                            burstBallsRemaining = 0;
                            burstStatus = "Aborted: shooter did not reach speed";
                        }
                        break;

                    case FEEDING:
                        if (burstTimer.seconds() >= Math.max(0.05, FEED_PULSE_SECONDS)) {
                            robot.setMatchLoaderDown(false);
                            burstBallsRemaining--;
                            if (burstBallsRemaining <= 0) {
                                burstState = BurstState.IDLE;
                                burstStatus = "Burst complete";
                            } else {
                                burstState = BurstState.RECOVERING;
                                burstStatus = "Waiting for RPM recovery";
                                burstTimer.reset();
                            }
                        }
                        break;

                    case IDLE:
                    default:
                        robot.setMatchLoaderDown(false);
                        break;
                }

                telemetry.addData("HIVE Y", "%.1f in", hiveYInches);
                telemetry.addData("Field X", "%.1f in", robot.getFieldX());
                telemetry.addData("Field Y", "%.1f in", robot.getFieldY());
                telemetry.addData("Heading", "%.1f deg", Math.toDegrees(robot.getHeadingRadians()));
                telemetry.addData("Robot center to HIVE", "%.1f in", robotCenterDistanceToHiveInches);
                telemetry.addData("Shooter field X", "%.1f in", shooterXInches);
                telemetry.addData("Shooter field Y", "%.1f in", shooterYInches);
                telemetry.addData("Shooter to HIVE", "%.1f in", shooterDistanceInches);
                telemetry.addData("Mode", controlMode);
                telemetry.addData("Burst", "%s (%d left)", burstStatus, burstBallsRemaining);
                telemetry.addData("Feeder position", "%.2f", robot.getMatchLoaderPosition());
                telemetry.addData("Target speed", "%.0f RPM", shooter.getTargetWheelRPM());
                telemetry.addData("Measured speed", "%.0f RPM", shooter.getCurrentWheelRPM());
                telemetry.addData("Master speed", "%.0f RPM", shooter.getMasterWheelRPM());
                telemetry.addData("Follower speed", "%.0f RPM", shooter.getFollowerWheelRPM());
                telemetry.addData("kF", "%.7f", DualShooterSubsystem.kF);
                telemetry.addData("kP", "%.7f", DualShooterSubsystem.kP);
                telemetry.addData("kI", "%.7f", DualShooterSubsystem.kI);
                telemetry.addData("kD", "%.7f", DualShooterSubsystem.kD);
                telemetry.addData("Master power", "%.3f", shooter.getMasterPower());
                telemetry.addData("Follower power", "%.3f", shooter.getFollowerPower());
                telemetry.addData("Ready", shooterReady ? "YES" : "NO");
                telemetry.update();

                if (dashboard != null) {
                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("targetRPM", shooter.getTargetWheelRPM());
                    packet.put("averageRPM", shooter.getCurrentWheelRPM());
                    packet.put("masterRPM", shooter.getMasterWheelRPM());
                    packet.put("followerRPM", shooter.getFollowerWheelRPM());
                    packet.put("masterPower", shooter.getMasterPower());
                    packet.put("followerPower", shooter.getFollowerPower());
                    packet.put("burstStatus", burstStatus);
                    packet.put("burstBallsRemaining", burstBallsRemaining);
                    packet.put("kF", DualShooterSubsystem.kF);
                    packet.put("kP", DualShooterSubsystem.kP);
                    packet.put("kI", DualShooterSubsystem.kI);
                    packet.put("kD", DualShooterSubsystem.kD);
                    dashboard.sendTelemetryPacket(packet);
                }

                idle();
            }
        } finally {
            robot.setMatchLoaderDown(false);
            shooter.stop();
            robot.drive(0.0, 0.0, 0.0);
        }
    }
}
