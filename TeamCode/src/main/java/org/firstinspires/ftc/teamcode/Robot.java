package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Collections;
import java.util.List;

public class Robot {

    public intake intakeSystem = new intake();
    public Shooter shooter = new Shooter();
    public AprilTagVision vision = new AprilTagVision();
    public final Turret turret = new Turret();

    private DcMotor leftDrive;
    private DcMotor rightDrive;

    private double leftPower = 0.0;
    private double rightPower = 0.0;

    public void init(HardwareMap hardwareMap) {
        turret.init(hardwareMap);
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeSystem.init(hardwareMap);
        shooter.init(hardwareMap);

        try {
            vision.init(hardwareMap);
        } catch (Exception e) {
            // Allows teleop to run if camera is disconnected
        }
    }

    public void drive(double forward, double turn) {
        leftPower = Math.max(-1.0, Math.min(1.0, forward + turn));
        rightPower = Math.max(-1.0, Math.min(1.0, forward - turn));

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
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
        turret.stop();
        if (vision != null) {
            vision.close();
        }
    }
}
