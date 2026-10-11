package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

/** Rear-camera solo route; FIRST BIOBUZZ TU04 section 9, setup guide V1.0 p20. */
@Autonomous(name = "Basic Auto - Flower to Hive", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {
    // Required robot measurements. Zero values block motion, rather than guessing.
    // Gear ratio: motor output shaft revolutions per wheel revolution.
    private static final double WHEEL_DIAMETER_INCHES = 0.0;
    private static final double TRACK_WIDTH_INCHES = 0.0;
    private static final double ROBOT_LENGTH_INCHES = 0.0;
    private static final double ROBOT_WIDTH_INCHES = 0.0;
    private static final double DRIVE_GEAR_RATIO = 0.0;
    private static final double START_GAP_INCHES = 12.0;
    private static final double SLOW_APPROACH_INCHES = 2.0;
    private static final double FLOWER_RETRIEVAL_DEPTH_INCHES = 3.57;
    // Center at x=96, y=36 from audience-left inside corner; outside hive frame.
    private static final double VIEWING_Y_INCHES = 36.0;
    private static final double FRAME_NEAR_Y_INCHES = 72.0 - 38.95 / 2.0;
    private static final double MATCH_LIMIT_SECONDS = 29.0;
    private DcMotor left;
    private DcMotor right;
    private final AprilTagVision vision = new AprilTagVision();
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopDrive();
        try {
            if (!calibrationValid()) {
                report("Motion blocked: set drivetrain dimensions/gearing; check frame clearance");
                waitForStart();
                return;
            }
            vision.init(hardwareMap);
            telemetry.addLine("SOLO: audience-side FLOWER, 48 in from right corner");
            telemetry.addLine("Rear bumper 12 in from bottom opening; rear camera faces FLOWER");
            telemetry.addLine("Front faces into field. Final turn: 180 degrees clockwise");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;
            matchTimer.reset();
            if (!move("Approach rear first", -(START_GAP_INCHES - SLOW_APPROACH_INCHES),
                    -(START_GAP_INCHES - SLOW_APPROACH_INCHES), 0.25, 4.0)) return;
            if (!move("Light rear tap", -SLOW_APPROACH_INCHES, -SLOW_APPROACH_INCHES, 0.10, 2.0)) return;
            double pullAway = VIEWING_Y_INCHES
                    - (FLOWER_RETRIEVAL_DEPTH_INCHES + ROBOT_LENGTH_INCHES / 2.0);
            if (!move("Pull away to hive viewing position", pullAway, pullAway, 0.25, 7.0)) return;
            if (!turnClockwise("Face hive with rear camera", 180.0)) return;
            if (!alignRearCamera()) return;
            if (!turnClockwise("Turn camera away from hive", 180.0)) return;
            report("Sequence complete; stopped");
        } finally {
            stopDrive();
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            vision.close();
        }
    }

    private boolean calibrationValid() {
        return positive(WHEEL_DIAMETER_INCHES) && positive(TRACK_WIDTH_INCHES)
                && positive(ROBOT_LENGTH_INCHES) && positive(ROBOT_WIDTH_INCHES)
                && positive(DRIVE_GEAR_RATIO)
                && positive(left.getMotorType().getTicksPerRev())
                && positive(right.getMotorType().getTicksPerRev())
                && FLOWER_RETRIEVAL_DEPTH_INCHES + ROBOT_LENGTH_INCHES / 2.0
                    + START_GAP_INCHES < VIEWING_Y_INCHES
                && VIEWING_Y_INCHES + Math.hypot(ROBOT_LENGTH_INCHES, ROBOT_WIDTH_INCHES) / 2.0
                    + 3.0 < FRAME_NEAR_Y_INCHES;
    }

    private boolean positive(double value) {
        return value > 0.0 && !Double.isInfinite(value) && !Double.isNaN(value);
    }

    private boolean active() {
        return opModeIsActive() && matchTimer.seconds() < MATCH_LIMIT_SECONDS;
    }

    private double ticksPerInch(DcMotor motor) {
        return motor.getMotorType().getTicksPerRev() * DRIVE_GEAR_RATIO
                / (Math.PI * WHEEL_DIAMETER_INCHES);
    }

    private boolean turnClockwise(String label, double degrees) {
        double arc = Math.PI * TRACK_WIDTH_INCHES * degrees / 360.0;
        return move(label, arc, -arc, 0.20, 5.0);
    }

    private boolean move(String label, double leftInches, double rightInches,
                         double power, double timeout) {
        if (!active()) return false;
        int leftTarget = left.getCurrentPosition() + (int) Math.round(leftInches * ticksPerInch(left));
        int rightTarget = right.getCurrentPosition() + (int) Math.round(rightInches * ticksPerInch(right));
        left.setTargetPosition(leftTarget);
        right.setTargetPosition(rightTarget);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stepTimer.reset();
        boolean reached = false;
        while (active() && stepTimer.seconds() < timeout) {
            boolean leftReached = Math.abs(leftTarget - left.getCurrentPosition()) <= 0.25 * ticksPerInch(left);
            boolean rightReached = Math.abs(rightTarget - right.getCurrentPosition()) <= 0.25 * ticksPerInch(right);
            left.setPower(leftReached ? 0.0 : power);
            right.setPower(rightReached ? 0.0 : power);
            if (leftReached && rightReached) {
                reached = true;
                break;
            }
            report(label);
            idle();
        }
        stopDrive();
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!reached) report("Aborted: " + label + " timeout/STOP");
        return reached && active();
    }

    private boolean alignRearCamera() {
        stepTimer.reset();
        while (active() && stepTimer.seconds() < 5.0) {
            AprilTagClusterDetection cluster = vision.findCluster(AprilTagVision.BLUE_AUDIENCE);
            if (cluster == null || Double.isNaN(cluster.ftcPose.bearing)
                    || Double.isInfinite(cluster.ftcPose.bearing)) {
                stopDrive();
                report("Waiting for BLUE AUDIENCE cluster; stopped");
            } else {
                double bearing = cluster.ftcPose.bearing;
                if (Math.abs(bearing) <= 2.0) {
                    stopDrive();
                    return true;
                }
                // Camera positive bearing is left; positive motor turn is clockwise.
                // Rear mounting reverses translation, but preserves yaw correction.
                double clockwise = Math.max(-0.15, Math.min(0.15, -bearing * 0.015));
                left.setPower(clockwise);
                right.setPower(-clockwise);
                report("Rear camera aligning to BLUE AUDIENCE");
            }
            sleep(50);
        }
        stopDrive();
        report("Aborted: hive cluster unavailable or alignment timed out");
        return false;
    }

    private void stopDrive() {
        left.setPower(0.0);
        right.setPower(0.0);
    }

    private void report(String message) {
        telemetry.addData("Autonomous", message);
        telemetry.addData("Elapsed", "%.1f s", matchTimer.seconds());
        telemetry.update();
    }
}
