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
     * {@code drive} mixes {@code left = forward + turn} and {@code right = forward - turn} with
     * the left motor FORWARD and the right motor REVERSE, so a positive turn drives the left
     * wheel forward and the right wheel backward: the chassis rotates CLOCKWISE (to the robot's
     * right). The SDK vision/pose convention is the opposite (positive yaw/bearing is to the
     * left), so SDK-derived headings must be multiplied by this constant before reaching
     * {@code drive}. Autonomous alignment uses it; AprilTag-bearing TeleOp code must use it too.
     */
    public static final double SDK_YAW_TO_TURN = -1.0;

    public AprilTagVision vision = new AprilTagVision();

    private VoltageSensor batterySensor;

    // 2 Odometry Encoders (unpowered dead wheels plugged into motor encoder ports)
    public DcMotor parallelEncoder;     // Parallel to drive direction (forward/backward)
    public DcMotor perpendicularEncoder;// Perpendicular to drive direction (strafing/side-to-side)

    private DcMotor leftDrive;
    private DcMotor rightDrive;

    private double leftPower = 0.0;
    private double rightPower = 0.0;

    public void init(HardwareMap hardwareMap) {
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        // Corrected motor directions so positive power drives the robot FORWARD
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void drive(double forward, double turn) {
        leftPower = Math.max(-1.0, Math.min(1.0, forward + turn));
        rightPower = Math.max(-1.0, Math.min(1.0, forward - turn));

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
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