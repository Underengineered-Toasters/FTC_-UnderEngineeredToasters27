package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.ds.DriverStation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * End-to-end test for the driver station web UI: attaches diagnostics, streams the
 * camera, and pushes per-detection AprilTag detail plus a heartbeat to both the
 * Driver Hub telemetry and the web UI.
 */
@TeleOp(name = "Driver Station Test", group = "ds")
public class DriverStationTestOpMode extends LinearOpMode {

    private static final String NOT_VISIBLE = "not visible";

    private final AprilTagVision vision = new AprilTagVision();
    private final Set<String> shownTags = new HashSet<>();

    @Override
    public void runOpMode() {
        DriverStation.attach(this);
        try {
            try {
                vision.init(hardwareMap);
                CameraStreamSource source = vision.getCameraSource();
                if (source == null) {
                    DriverStation.log("camera unavailable: no stream source");
                } else {
                    DriverStation.setCameraSource(source);
                }
            } catch (Exception e) {
                DriverStation.log("vision init failed: " + e.getMessage());
            }

            DriverStation.log("Driver Station Test initialized");
            telemetry.addLine("Driver Station Test initialized.");
            telemetry.addLine("Waiting for start...");
            telemetry.update();

            waitForStart();

            int heartbeat = 0;
            while (opModeIsActive()) {
                List<AprilTagDetection> detections = vision.getDetections();

                telemetry.addData("Driver Station Connection", "Active");
                telemetry.addData("Total Detections", detections.size());
                telemetry.addLine("--------------------------------");
                DriverStation.telemetry("AprilTag Targets", detections.size());
                DriverStation.telemetry("Heartbeat", heartbeat);

                Set<String> seen = new HashSet<>();
                for (AprilTagDetection detection : detections) {
                    String identity = AprilTagTargets.identityOf(detection);
                    String name = AprilTagTargets.nameOf(detection);
                    String key = "Tag " + identity;
                    String detail;

                    telemetry.addLine(String.format("Tag ID/group %s (%s):", identity, name));
                    if (detection.ftcPose != null) {
                        telemetry.addLine(String.format(
                                "   X: %.1fin | Y: %.1fin | Z: %.1fin",
                                detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        telemetry.addLine(String.format(
                                "   Range: %.1fin | Bearing: %.1fdeg | Yaw: %.1fdeg",
                                detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.yaw));
                        detail = String.format("%s | range %.1fin, bearing %.1fdeg, yaw %.1fdeg",
                                name, detection.ftcPose.range, detection.ftcPose.bearing,
                                detection.ftcPose.yaw);
                    } else if (AprilTagTargets.centerOf(detection) != null) {
                        telemetry.addLine(String.format(
                                "   2D Center: X %.0f, Y %.0f",
                                AprilTagTargets.centerOf(detection).x,
                                AprilTagTargets.centerOf(detection).y));
                        detail = String.format("%s | 2D center X %.0f, Y %.0f", name,
                                AprilTagTargets.centerOf(detection).x,
                                AprilTagTargets.centerOf(detection).y);
                    } else {
                        detail = name;
                    }

                    DriverStation.telemetry(key, detail);
                    seen.add(key);
                }

                // The web UI keeps the last value of every key, so mark lost tags explicitly.
                for (String key : shownTags) {
                    if (!seen.contains(key)) {
                        DriverStation.telemetry(key, NOT_VISIBLE);
                    }
                }
                shownTags.addAll(seen);

                if (heartbeat % 10 == 0) {
                    DriverStation.log("heartbeat " + heartbeat + ", targets=" + detections.size());
                }
                heartbeat++;

                telemetry.update();
                sleep(100);
            }

            DriverStation.log("Driver Station Test stopping");
        } finally {
            DriverStation.detach();
            vision.close();
        }
    }
}
