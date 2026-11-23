package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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

    /**
     * Sign that converts an SDK-style yaw command (positive = counter-clockwise) into the
     * {@link #drive(double, double)} turn argument.
     *
     * {@code drive} gives a positive turn to the left wheels and the opposite to the right
     * wheels, so the chassis rotates CLOCKWISE (to the robot's right). The SDK vision/pose
     * convention is the opposite (positive yaw/bearing is to the left), so SDK-derived headings
     * must be multiplied by this constant before reaching {@code drive}. Autonomous alignment uses
     * it; AprilTag-bearing TeleOp code must use it too.
     */
    public static final double SDK_YAW_TO_TURN = -1.0;

    // Mecanum drivetrain hardware names. They must match the Driver Hub robot configuration and
    // the names MecanumDrive uses, so every OpMode and Road Runner share one configuration.
    public static final String LEFT_FRONT_MOTOR = "leftFront";
    public static final String LEFT_BACK_MOTOR = "leftBack";
    public static final String RIGHT_FRONT_MOTOR = "rightFront";
    public static final String RIGHT_BACK_MOTOR = "rightBack";

    public AprilTagVision vision = new AprilTagVision();

    private VoltageSensor batterySensor;

    // 2 Odometry Encoders (unpowered dead wheels plugged into motor encoder ports)
    public DcMotor parallelEncoder;     // Parallel to drive direction (forward/backward)
    public DcMotor perpendicularEncoder;// Perpendicular to drive direction (strafing/side-to-side)

    private DcMotor leftFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;

    private double leftFrontPower = 0.0;
    private double leftBackPower = 0.0;
    private double rightFrontPower = 0.0;
    private double rightBackPower = 0.0;

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, true);
    }

    /**
     * @param enableAprilTagVision false when the OpMode drives its own camera wrapper (for example
     *        {@link YellowBallVision}). Only one {@code VisionPortal} can hold a camera at a time,
     *        so the AprilTag portal must stay closed in that case; {@code robot.vision} is then
     *        left uninitialized and reports no detections.
     */
    public void init(HardwareMap hardwareMap, boolean enableAprilTagVision) {
        leftFrontDrive = hardwareMap.get(DcMotor.class, LEFT_FRONT_MOTOR);
        leftBackDrive = hardwareMap.get(DcMotor.class, LEFT_BACK_MOTOR);
        rightFrontDrive = hardwareMap.get(DcMotor.class, RIGHT_FRONT_MOTOR);
        rightBackDrive = hardwareMap.get(DcMotor.class, RIGHT_BACK_MOTOR);

        applyMecanumDirections(leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive);

        for (DcMotor motor : new DcMotor[]{
                leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive}) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        // Initialize 2 Odometry Pod Encoders ("parOdom" and "perpOdom")
        try {
            parallelEncoder = hardwareMap.get(DcMotor.class, "parOdom");
            perpendicularEncoder = hardwareMap.get(DcMotor.class, "perpOdom");

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
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                batterySensor = sensor;
                break;
            }
        } catch (Exception e) {
            batterySensor = null;
        }

        if (enableAprilTagVision) {
            try {
                vision.init(hardwareMap);
            } catch (Exception e) {
                // Allows teleop to run if camera is disconnected
            }
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

    /**
     * Points every drive motor forward for positive power. The mecanum mixing in
     * {@link #drive(double, double, double)} assumes this normalization.
     *
     * <p>The left and right motors are mounted mirrored, so the right pair is reversed. If one
     * wheel turns the wrong way on the bench, flip that single line here - never the mixing.
     */
    public static void applyMecanumDirections(DcMotor leftFront, DcMotor leftBack,
                                              DcMotor rightFront, DcMotor rightBack) {
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    /** Mecanum drive with no strafe; the timed autonomous routes use this. */
    public void drive(double forward, double turn) {
        drive(forward, 0.0, turn);
    }

    /**
     * Mecanum drive.
     *
     * @param forward positive = away from the driver (all four wheels forward)
     * @param strafe  positive = to the robot's right
     * @param turn    positive = clockwise, to the robot's right
     */
    public void drive(double forward, double strafe, double turn) {
        double[] powers = mecanumWheelPowers(forward, strafe, turn);

        leftFrontPower = powers[0];
        leftBackPower = powers[1];
        rightFrontPower = powers[2];
        rightBackPower = powers[3];

        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Mecanum wheel mixing for one 3-axis command, already normalized to [-1, 1].
     *
     * <p>Positive forward drives all four wheels forward, positive strafe moves the robot right,
     * and positive turn rotates it clockwise. Clipping each wheel separately would bend the
     * requested arc whenever two axes are near full power, so all four are scaled by the same
     * factor instead.
     *
     * <p>Powers follow {@code MecanumDrive}'s wheel order: left front, left back, right front,
     * right back. This is the single mix used by every raw-motor OpMode as well as {@link #drive}.
     */
    public static double[] mecanumWheelPowers(double forward, double strafe, double turn) {
        double f = clip(forward);
        double s = clip(strafe);
        double t = clip(turn);

        double leftFront = f + s + t;
        double leftBack = f - s + t;
        double rightFront = f - s - t;
        double rightBack = f + s - t;

        double largest = Math.max(
                Math.max(Math.abs(leftFront), Math.abs(leftBack)),
                Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (largest > 1.0) {
            double scale = 1.0 / largest;
            leftFront *= scale;
            leftBack *= scale;
            rightFront *= scale;
            rightBack *= scale;
        }

        return new double[]{leftFront, leftBack, rightFront, rightBack};
    }

    /**
     * Mixes one 3-axis command and writes it to four raw drive motors. Used by the OpModes that
     * keep their own motors instead of building the whole {@link Robot} hub.
     */
    public static void driveMecanum(DcMotor leftFront, DcMotor leftBack,
                                    DcMotor rightFront, DcMotor rightBack,
                                    double forward, double strafe, double turn) {
        double[] powers = mecanumWheelPowers(forward, strafe, turn);
        leftFront.setPower(powers[0]);
        leftBack.setPower(powers[1]);
        rightFront.setPower(powers[2]);
        rightBack.setPower(powers[3]);
    }

    /** Clamps a requested axis to the legal power range. */
    public static double clip(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
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

    /**
     * Battery voltage used for autonomous time compensation.
     * Returns 12.0 (nominal) when no voltage sensor is configured, which makes the
     * compensation a no-op instead of dividing by zero.
     */
    public double getBatteryVolts() {
        if (batterySensor == null) {
            return 12.0;
        }
        double volts;
        try {
            volts = batterySensor.getVoltage();
        } catch (Exception e) {
            return 12.0;
        }
        return volts > 1.0 ? volts : 12.0;
    }

    /** Spins both flywheels up to full power. */
    public void spinUpShooters() {
        shooter1System.spinForwardMaxSpeed();
        shooter2System.spinForwardMaxSpeed();
    }

    /** Stops both flywheels. */
    public void spinDownShooters() {
        shooter1System.spinDown();
        shooter2System.spinDown();
    }

    public double getLeftFrontPower() {
        return leftFrontPower;
    }

    public double getLeftBackPower() {
        return leftBackPower;
    }

    public double getRightFrontPower() {
        return rightFrontPower;
    }

    public double getRightBackPower() {
        return rightBackPower;
    }

    public void close() {
        if (vision != null) {
            vision.close();
        }
    }
}