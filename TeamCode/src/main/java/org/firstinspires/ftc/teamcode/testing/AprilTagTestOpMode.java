package org.firstinspires.ftc.teamcode.testing;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.AprilTagTargets;
import org.firstinspires.ftc.teamcode.robot.AprilTagVision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.List;

@Config
@TeleOp(name = "AprilTag Test", group = "Vision")
public class AprilTagTestOpMode extends LinearOpMode {

    // Set before INIT. Use 640 x 480 to compare with the LifeCam's 720p mode.
    public static int CAMERA_WIDTH = 1280;
    public static int CAMERA_HEIGHT = 720;

    private final AprilTagVision vision = new AprilTagVision();

    @Override
    public void runOpMode() {
        Size resolution = new Size(CAMERA_WIDTH, CAMERA_HEIGHT);
        vision.init(hardwareMap, resolution);
        try {
            while (!isStarted() && !isStopRequested()) {
                showDetections(resolution);
                sleep(50);
            }

            waitForStart();
            while (opModeIsActive()) {
                showDetections(resolution);
                sleep(50);
            }
        } finally {
            vision.close();
        }
    }

    private void showDetections(Size resolution) {
        List<AprilTagDetection> detections = vision.getDetections();
        telemetry.addData("Requested Resolution", "%d x %d",
                resolution.getWidth(), resolution.getHeight());
        telemetry.addData("Camera State", vision.getCameraState());
        telemetry.addData("Camera FPS", "%.1f", vision.getFps());
        telemetry.addData("AprilTag Detections", detections.size());

        for (AprilTagDetection detection : detections) {
            telemetry.addLine(String.format("%s (%s)",
                    AprilTagTargets.nameOf(detection), AprilTagTargets.identityOf(detection)));
            AprilTagPoseFtc pose = detection.ftcPose;
            if (pose == null) {
                telemetry.addLine("Pose unavailable");
                continue;
            }

            // The FTC pose is relative to the camera. X and Y lie in its horizontal plane
            // when the camera is mounted level.
            telemetry.addData("Camera X / Y / Z (in)", "%.1f / %.1f / %.1f", pose.x, pose.y, pose.z);
            telemetry.addData("Camera X–Y Distance (in)", "%.1f", Math.hypot(pose.x, pose.y));
            telemetry.addData("Bearing (deg)", "%.1f", pose.bearing);
            telemetry.addData("Yaw / Pitch / Roll (deg)", "%.1f / %.1f / %.1f",
                    pose.yaw, pose.pitch, pose.roll);
        }

        telemetry.update();
    }
}
