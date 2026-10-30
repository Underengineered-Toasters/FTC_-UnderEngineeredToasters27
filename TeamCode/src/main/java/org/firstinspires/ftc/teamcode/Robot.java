package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Collections;
import java.util.List;

public class Robot {

    public intake intakeSystem = new intake();

    // Dual shooter motor subsystems
    public shooter1 shooter1System = new shooter1(); // Motor Port 2 ("shooter1")
    public shooter2 shooter2System = new shooter2(); // Motor Port 3 ("shooter2")
    public Shooter shooter = new Shooter(); // Unified shooter wrapper

    // 360-Degree Turret Motor & Limit Switch
    public turret turretSystem = new turret(); // Motor ("turret") & Limit Switch ("mag")

    // Match Loader Servo ("matchLoader")
    public Servo matchLoaderServo;
    public static final double MATCH_LOADER_UP      = 0.0;
    public static final double MATCH_LOADER_DOWN_90 = 0.50; // 90 degrees down position

    public AprilTagVision vision = new AprilTagVision();

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
            matchLoaderServo.setPosition(down ? MATCH_LOADER_DOWN_90 : MATCH_LOADER_UP);
        }
    }

    public double getMatchLoaderPosition() {
        return matchLoaderServo != null ? matchLoaderServo.getPosition() : Double.NaN;
    }

    public void update() {
        shooter.update();
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
