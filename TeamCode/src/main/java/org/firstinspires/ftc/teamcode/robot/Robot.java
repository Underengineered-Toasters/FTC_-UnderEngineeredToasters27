package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystems.intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter1;
import org.firstinspires.ftc.teamcode.robot.subsystems.shooter2;
import org.firstinspires.ftc.teamcode.robot.subsystems.turret;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Collections;
import java.util.List;

public class Robot {

    public intake intakeSystem = new intake();

    // Dual shooter motor subsystems
    public shooter1 shooter1System = new shooter1(); // Motor Port 2 / Servo Port ("shooter1")
    public shooter2 shooter2System = new shooter2(); // Motor Port 3 / Servo Port ("shooter2")
    public Shooter shooter = new Shooter(); // Unified shooter wrapper

    // 360-Degree Turret Motor & Limit Switch
    public turret turretSystem = new turret(); // Motor ("turret") & Limit Switch ("mag")

    // Match Loader Servo ("matchLoader") & Touch Sensor ("matchLoaderTouch")
    public Servo matchLoaderServo;
    public TouchSensor matchLoaderTouch;

    public static final double MATCH_LOADER_UP       = 0.00; // Rest position
    public static final double MATCH_LOADER_DOWN_110  = 0.61; // 110 degrees rotation to the left

    public AprilTagVision vision = new AprilTagVision();

    // 2 Odometry pods (dead wheels plugged into the frontleft / frontright motor encoder ports)
    public DcMotor parallelEncoder;     // forward pod: parallel to the drive direction (forward/backward)
    public DcMotor perpendicularEncoder;// strafe pod: perpendicular to the drive direction (side-to-side)

    // Mecanum drivetrain (names/directions mirror MecanumDriveTest.java)
    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;

    private double leftPower = 0.0;
    private double rightPower = 0.0;

    // ------------------------------------------------------------------
    // Odometry -> field position tracking
    // ------------------------------------------------------------------
    // Pedro Pathing uses a right-hand field frame whose origin (0,0) is the BOTTOM-LEFT corner of
    // the 144" x 144" field. +x points to the robot's right, +y points up the field, and heading is
    // counterclockwise with 0 rad = facing +x. We integrate the odometry pods in that same frame so
    // this code reports X/Y position and distance traveled in the coordinates Pedro's paths use.
    // Heading IMU is the one built into the REV Control Hub. It shows up in the Driver Hub robot
    // configuration as an "IMU" device (default name "imu"); the code just looks it up by that name.
    // A few common spellings are tried so this works even if the config used a slightly different
    // label. isImuReady() reports whether it was actually found.
    public IMU imu;
    private static final String[] IMU_NAMES = {"imu", "IMU", "imu1"};
    private String imuName = null;

    // Inches per encoder tick for each pod. Replace after running the Pedro "Two Wheel Tuner".
    public static final double FORWARD_TICKS_TO_INCHES = 1.0;
    public static final double STRAFE_TICKS_TO_INCHES  = 1.0;

    private double fieldX = 0.0;                // inches, Pedro frame (0 = left edge of the field)
    private double fieldY = 0.0;                // inches, Pedro frame (0 = bottom edge of the field)
    private double headingOffsetRadians = 0.0;  // aligns raw IMU yaw with a chosen field heading
    private double distanceTraveled = 0.0;      // total odometry path length, inches

    private int previousForwardTicks = 0;
    private int previousStrafeTicks = 0;

    // Pod motion indicators. A pod counts as "in motion" while its ticks keep changing, and stays
    // "in motion" for a short grace period afterwards so the Driver Station telemetry is readable
    // instead of flickering on and off every loop.
    public static final double ODOM_MOTION_THRESHOLD_INCHES = 0.001; // smallest change that counts as movement
    public static final long ODOM_MOTION_TIMEOUT_MS = 250;           // how long "in motion" lingers

    private boolean forwardPodMoving = false;
    private boolean strafePodMoving = false;
    private long lastForwardMoveMs = 0;
    private long lastStrafeMoveMs = 0;

    public void init(HardwareMap hardwareMap) {
        // Mecanum drivetrain. Names and directions mirror MecanumDriveTest.java so that a positive
        // forward/strafe/turn input produces the expected motion.
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontleft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontright");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "backleft");
        backRightDrive  = hardwareMap.get(DcMotor.class, "backright");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // IMU for heading: look up the Control Hub's built-in IMU by its config name. Orientation
        // matches OdometryTest.java (logo UP, USB FORWARD).
        imuName = null;
        for (String name : IMU_NAMES) {
            try {
                IMU candidate = hardwareMap.tryGet(IMU.class, name);
                if (candidate == null) {
                    continue;
                }
                candidate.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )));
                imu = candidate;
                imuName = name;
                break;
            } catch (Exception e) {
                // Try the next candidate name; teleop still runs if no IMU is configured.
            }
        }
        if (imuName == null) {
            imu = null;
        }

        // Initialize Match Loader Servo ("matchLoader")
        try {
            matchLoaderServo = hardwareMap.tryGet(Servo.class, "matchLoader");
            if (matchLoaderServo != null) {
                matchLoaderServo.setPosition(MATCH_LOADER_UP);
            }
        } catch (Exception e) {
            matchLoaderServo = null;
        }

        // Initialize Match Loader Touch Sensor ("matchLoaderTouch" or "touch")
        try {
            matchLoaderTouch = hardwareMap.tryGet(TouchSensor.class, "matchLoaderTouch");
            if (matchLoaderTouch == null) {
                matchLoaderTouch = hardwareMap.tryGet(TouchSensor.class, "touch");
            }
        } catch (Exception e) {
            matchLoaderTouch = null;
        }

        // Initialize 2 Odometry Encoders ("frontleft" = Y/forward pod, "frontright" = X/sideways pod)
        try {
            parallelEncoder = hardwareMap.get(DcMotor.class, "frontleft");
            perpendicularEncoder = hardwareMap.get(DcMotor.class, "frontright");

            resetOdomEncoders();
        } catch (Exception e) {
            // Allows OpMode to run if odom pods are not plugged in or configured differently
        }

        // Initialize intake, shooter, and turret subsystems
        intakeSystem.init(hardwareMap);
        shooter1System.init(hardwareMap);
        shooter2System.init(hardwareMap);
        shooter.init(hardwareMap);
        turretSystem.init(hardwareMap);

        try {
            vision.init(hardwareMap);
        } catch (Exception e) {
            // Allows teleop to run if camera is disconnected
        }
    }

    public void resetOdomEncoders() {
        if (parallelEncoder != null) {
            parallelEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            parallelEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (perpendicularEncoder != null) {
            perpendicularEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            perpendicularEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public int getParallelEncoderTicks() {
        return parallelEncoder != null ? parallelEncoder.getCurrentPosition() : 0;
    }

    public int getPerpendicularEncoderTicks() {
        return perpendicularEncoder != null ? perpendicularEncoder.getCurrentPosition() : 0;
    }

    /** Y odometry data: forward/backward pod ticks ("frontleft"). */
    public int getYOdometerTicks() {
        return getParallelEncoderTicks();
    }

    /** X odometry data: sideways/strafe pod ticks ("frontright"). */
    public int getXOdometerTicks() {
        return getPerpendicularEncoderTicks();
    }

    /** Forward pod ticks (the pod that measures forward/backward motion). */
    public int getForwardPodTicks() {
        return getParallelEncoderTicks();
    }

    /** Strafe pod ticks (the pod that measures side-to-side motion). */
    public int getStrafePodTicks() {
        return getPerpendicularEncoderTicks();
    }

    /** True while the forward (Y) odometry pod is reporting movement. */
    public boolean isForwardPodMoving() {
        return forwardPodMoving;
    }

    /** True while the strafe (X) odometry pod is reporting movement. */
    public boolean isStrafePodMoving() {
        return strafePodMoving;
    }

    // ------------------------------------------------------------------
    // Odometry-based field position (Pedro frame)
    // ------------------------------------------------------------------

    /** True if the hub's built-in IMU was found and initialized; use this to verify heading works. */
    public boolean isImuReady() {
        return imu != null;
    }

    /** HardwareMap name the IMU was found under, or null if no IMU was found. */
    public String getImuName() {
        return imuName;
    }

    /** Current heading in radians, Pedro convention (0 = facing +x, counterclockwise positive). */
    public double getHeadingRadians() {
        if (imu == null) {
            return normalizeRadians(headingOffsetRadians);
        }
        double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        return normalizeRadians(yaw + headingOffsetRadians);
    }

    /** Current field X position in inches (Pedro frame: 0 at the left edge of the field). */
    public double getFieldX() {
        return fieldX;
    }

    /** Current field Y position in inches (Pedro frame: 0 at the bottom edge of the field). */
    public double getFieldY() {
        return fieldY;
    }

    /** Total distance the robot has driven, in inches, according to the odometry pods. */
    public double getDistanceTraveled() {
        return distanceTraveled;
    }

    /** Straight-line distance in inches from the robot to a field point (Pedro frame). */
    public double getDistanceTo(double x, double y) {
        return Math.hypot(x - fieldX, y - fieldY);
    }

    /**
     * Places the robot at a known field pose and re-zeroes the odometry deltas.
     *
     * Call this at the start of autonomous with the robot's real starting coordinates so the
     * tracked position lines up with Pedro's corner-origin frame (and, if desired, matches the
     * pose you pass to follower.setPose(...)). Coordinates are inches; heading is radians.
     */
    public void setPose(double x, double y, double headingRadians) {
        fieldX = x;
        fieldY = y;
        distanceTraveled = 0.0;

        if (imu != null) {
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            headingOffsetRadians = normalizeRadians(headingRadians - yaw);
        } else {
            headingOffsetRadians = normalizeRadians(headingRadians);
        }

        previousForwardTicks = getForwardPodTicks();
        previousStrafeTicks = getStrafePodTicks();
    }

    /**
     * Reads the odometry pods and integrates the movement into the tracked field position.
     * Call once per loop so field X/Y and the distance traveled stay current.
     */
    public void updateOdometry() {
        int forwardTicks = getForwardPodTicks();
        int strafeTicks = getStrafePodTicks();

        double deltaForward = FORWARD_TICKS_TO_INCHES * (forwardTicks - previousForwardTicks);
        double deltaStrafe = STRAFE_TICKS_TO_INCHES * (strafeTicks - previousStrafeTicks);
        previousForwardTicks = forwardTicks;
        previousStrafeTicks = strafeTicks;

        // Update the "in motion" indicators used by the Driver Station telemetry.
        long now = System.currentTimeMillis();
        if (Math.abs(deltaForward) > ODOM_MOTION_THRESHOLD_INCHES) {
            lastForwardMoveMs = now;
        }
        if (Math.abs(deltaStrafe) > ODOM_MOTION_THRESHOLD_INCHES) {
            lastStrafeMoveMs = now;
        }
        forwardPodMoving = (now - lastForwardMoveMs) < ODOM_MOTION_TIMEOUT_MS;
        strafePodMoving = (now - lastStrafeMoveMs) < ODOM_MOTION_TIMEOUT_MS;

        // Rotate the robot-frame deltas (forward, strafe) into the field frame. This mirrors the
        // math inside Pedro's TwoWheelLocalizer so the numbers agree with Pedro's pose.
        double heading = getHeadingRadians();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        fieldX += deltaForward * cos - deltaStrafe * sin;
        fieldY += deltaForward * sin + deltaStrafe * cos;

        distanceTraveled += Math.hypot(deltaForward, deltaStrafe);
    }

    private static double normalizeRadians(double radians) {
        double twoPi = 2.0 * Math.PI;
        double wrapped = radians % twoPi;
        if (wrapped > Math.PI) {
            wrapped -= twoPi;
        } else if (wrapped < -Math.PI) {
            wrapped += twoPi;
        }
        return wrapped;
    }

    /** Tank-style convenience (no strafe). Kept so existing OpModes compile unchanged. */
    public void drive(double forward, double turn) {
        drive(forward, 0.0, turn);
    }

    /** Mecanum drive. forward/strafe/turn are each in [-1, 1]; positive strafe drives right. */
    public void drive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Normalize so no motor command exceeds 1.0 while keeping the requested ratios.
        double max = Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        double denominator = Math.max(1.0, max);
        fl /= denominator;
        fr /= denominator;
        bl /= denominator;
        br /= denominator;

        if (frontLeftDrive != null) frontLeftDrive.setPower(fl);
        if (frontRightDrive != null) frontRightDrive.setPower(fr);
        if (backLeftDrive != null) backLeftDrive.setPower(bl);
        if (backRightDrive != null) backRightDrive.setPower(br);

        leftPower = (fl + bl) / 2.0;
        rightPower = (fr + br) / 2.0;
    }

    public void setMatchLoaderDown(boolean down) {
        if (matchLoaderServo != null) {
            // Touch sensor safety safeguard: If touch sensor is triggered while driving down, halt / retract servo
            if (down && isMatchLoaderTouchPressed()) {
                matchLoaderServo.setPosition(MATCH_LOADER_UP);
            } else {
                matchLoaderServo.setPosition(down ? MATCH_LOADER_DOWN_110 : MATCH_LOADER_UP);
            }
        }
    }

    public boolean isMatchLoaderTouchPressed() {
        return matchLoaderTouch != null && matchLoaderTouch.isPressed();
    }

    public double getMatchLoaderPosition() {
        return matchLoaderServo != null ? matchLoaderServo.getPosition() : Double.NaN;
    }

    public void update() {
        // If match loader is down and touches the sensor, automatically halt/retract to protect servo
        if (isMatchLoaderTouchPressed()) {
            if (matchLoaderServo != null && matchLoaderServo.getPosition() > 0.1) {
                matchLoaderServo.setPosition(MATCH_LOADER_UP);
            }
        }
    }

    public List<AprilTagDetection> getAprilTagDetections() {
        if (vision != null) {
            return vision.getDetections();
        }
        return Collections.emptyList();
    }

    public double getLeftPower() {
        return leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }

    public void close() {
        if (vision != null) {
            vision.close();
        }
    }
}