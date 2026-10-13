package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import java.util.Collections;
import java.util.List;
public class Robot {

    public static final double ALIGN_KP = 0.05;
    public static final double ALIGN_MAX_TURN = 0.6;
    public static final double ALIGN_MIN_TURN = 0.15;
    public static final double ALIGN_TOLERANCE_DEG = 2.0;

    public static class AlignResult {
        public final boolean targetFound;
        public final boolean aligned;
        public final double turnPower;
        public final double bearingDeg;
        public final Double rangeInches;

        public AlignResult(boolean targetFound, boolean aligned, double turnPower, double bearingDeg, Double rangeInches) {
            this.targetFound = targetFound;
            this.aligned = aligned;
            this.turnPower = turnPower;
            this.bearingDeg = bearingDeg;
            this.rangeInches = rangeInches;
        }
    }

    public intake intakeSystem = new intake();
    public Shooter shooter = new Shooter();
    public AprilTagVision vision = new AprilTagVision();

    private DcMotor leftDrive;
    private DcMotor rightDrive;

    private double leftPower = 0.0;
    private double rightPower = 0.0;

    public void init(HardwareMap hardwareMap) {
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeSystem.init(hardwareMap);
        shooter.init(hardwareMap);

        try {
            vision.init(hardwareMap);
        } catch (Exception e) {
            // Allows teleop to run if camera is disconnected; vision.isReady()
            // stays false and telemetry/CSV report the camera as offline.
        }
    }

    public void drive(double forward, double turn) {
        leftPower = Math.max(-1.0, Math.min(1.0, forward + turn));
        rightPower = Math.max(-1.0, Math.min(1.0, forward - turn));

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    public void stopDrive() {
        drive(0, 0);
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

    public AprilTagClusterDetection findAprilTagCluster(String name) {
        if (vision != null) {
            return vision.findCluster(name);
        }
        return null;
    }

    public AlignResult calculateClusterAlign(String clusterName) {
        AprilTagClusterDetection target = findAprilTagCluster(clusterName);
        if (target == null || target.ftcPose == null) {
            return new AlignResult(false, false, 0.0, 0.0, null);
        }

        double bearing = target.ftcPose.bearing;
        double range = target.ftcPose.range;

        if (Math.abs(bearing) <= ALIGN_TOLERANCE_DEG) {
            return new AlignResult(true, true, 0.0, bearing, range);
        }

        double power = Math.max(ALIGN_MIN_TURN, Math.min(ALIGN_MAX_TURN, Math.abs(ALIGN_KP * bearing)));
        double turn = -Math.signum(bearing) * power;
        return new AlignResult(true, false, turn, bearing, range);
    }

    public double getLeftPower() {
        return leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }

    public String getCameraState() {
        return vision.getCameraState();
    }

    public boolean isVisionReady() {
        return vision.isReady();
    }

    public void close() {
        if (vision != null) {
            vision.close();
        }
    }
}
