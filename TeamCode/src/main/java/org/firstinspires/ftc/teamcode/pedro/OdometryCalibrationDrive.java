package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/** Manual mecanum drive with live two-wheel Pedro odometry. */
@TeleOp(name = "Odometry Calibration Drive", group = "Calibration")
public class OdometryCalibrationDrive extends LinearOpMode {
    private static final double DRIVE_SPEED = 0.75;
    private static final double SLOW_SPEED = 0.35;

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        // The external pods are connected to the encoder pins of these configured motor ports.
        DcMotorEx forwardPod = hardwareMap.get(DcMotorEx.class, "frontleft");
        DcMotorEx strafePod = hardwareMap.get(DcMotorEx.class, "frontright");

        telemetry.addLine("Start sets the current robot pose to (0, 0, 0).");
        telemetry.addLine("Left stick: drive/strafe. Right stick X: turn. Left bumper: slow.");
        telemetry.addLine("A: reset the current pose to (0, 0, 0).");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) {
            follower.stop();
            return;
        }

        follower.setPose(Pose.zero());
        boolean previousA = false;

        try {
            while (opModeIsActive()) {
                boolean resetRequested = gamepad1.a && !previousA;
                previousA = gamepad1.a;
                if (resetRequested) follower.setPose(Pose.zero());

                double speed = gamepad1.left_bumper ? SLOW_SPEED : DRIVE_SPEED;
                double forward = -gamepad1.left_stick_y * speed;
                double strafe = -gamepad1.left_stick_x * speed;
                double turn = -gamepad1.right_stick_x * speed;
                follower.manual(forward, strafe, turn);
                follower.update();

                Pose pose = follower.pose();
                telemetry.addData("X (forward, in)", "%.2f", pose.x());
                telemetry.addData("Y (left, in)", "%.2f", pose.y());
                telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
                telemetry.addData("Forward pod raw ticks", forwardPod.getCurrentPosition());
                telemetry.addData("Strafe pod raw ticks", strafePod.getCurrentPosition());
                telemetry.addLine("If a pod's raw ticks stay at 0 while its wheel rolls, check that pod's cable/port.");
                telemetry.addLine("A resets pose to 0, 0, 0");
                telemetry.update();
                idle();
            }
        } finally {
            follower.stop();
        }
    }
}
