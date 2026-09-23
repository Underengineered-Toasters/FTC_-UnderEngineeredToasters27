package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Collections;
import java.util.List;

/**
 * Main robot container.
 *
 * This class owns all of the robot's subsystems so that one OpMode
 * can initialize and control the entire robot.
 *
 * Subsystems:
 * - Drivetrain
 * - Intake
 * - Shooter
 * - AprilTag Vision
 */
public class Robot {

    // =========================
    // Drivetrain
    // =========================

    private DcMotor left;
    private DcMotor right;

    // =========================
    // Subsystems
    // =========================

    public final intake intakeSystem = new intake();
    public final Shooter shooter = new Shooter();
    public final AprilTagVision vision = new AprilTagVision();

    // =========================
    // Initialization
    // =========================

    public void init(HardwareMap hardwareMap) {

        // -------------------------
        // Drivetrain
        // -------------------------

        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // -------------------------
        // Intake
        // -------------------------

        intakeSystem.init(hardwareMap);

        // -------------------------
        // Shooter
        // -------------------------

        shooter.init(hardwareMap);

        // -------------------------
        // AprilTag Vision
        // -------------------------

        vision.init(hardwareMap);
    }

    // =========================
    // Drivetrain
    // =========================

    public void drive(double drive, double turn) {

        double leftPower = drive + turn;
        double rightPower = drive - turn;

        // Prevent values outside [-1, 1]
        leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
        rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

        left.setPower(leftPower);
        right.setPower(rightPower);
    }

    public void stopDrive() {
        left.setPower(0);
        right.setPower(0);
    }

    public double getLeftPower() {
        return left.getPower();
    }

    public double getRightPower() {
        return right.getPower();
    }

    // =========================
    // Shooter Update
    // =========================

    /**
     * Must be called repeatedly while the OpMode is running.
     * This actually sends the target velocity to the shooter motor.
     */
    public void update() {
        shooter.update();
    }

    // =========================
    // Vision
    // =========================

    public List<AprilTagDetection> getAprilTagDetections() {

        if (vision == null) {
            return Collections.emptyList();
        }

        return vision.getDetections();
    }

    /*public CameraStreamSource getCameraSource() {
        return vision.getCameraSource();
    }
    */
    // =========================
    // Stop Everything
    // =========================

    public void stopAll() {

        stopDrive();

        intakeSystem.stopIntake();

        shooter.spinDown();
        shooter.update();
    }

    // =========================
    // Close
    // =========================

    public void close() {

        stopAll();

        vision.close();
    }
}
