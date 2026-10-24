package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "AprilTag Test", group = "Vision")
public class AprilTagTestOpMode extends LinearOpMode {

    private AprilTagVision vision = new AprilTagVision();
    private static final String DESIRED_CLUSTER = AprilTagVision.RED_AUDIENCE;

    @Override
    public void runOpMode() {

        vision.init(hardwareMap);

        telemetry.addLine("Vision initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = vision.getDetections();
            boolean targetFound = false;

            telemetry.addData("AprilTag Targets", detections.size());
            telemetry.addData("Looking for Cluster", DESIRED_CLUSTER);
            telemetry.addLine("--------------------------------");

            for (AprilTagDetection tag : detections) {
                if (tag instanceof AprilTagClusterDetection) {
                    AprilTagClusterDetection cluster = (AprilTagClusterDetection) tag;
                    String clusterName = (cluster.metadata != null) ? cluster.metadata.name : "unknown";
                    if (DESIRED_CLUSTER.equals(clusterName)) {
                        targetFound = true;
                        telemetry.addLine(String.format(
                                ">> TARGET CLUSTER (%s) FOUND: %d%% of tags <<",
                                clusterName,
                                cluster.percentClusterFound
                        ));
                    } else {
                        telemetry.addLine(String.format(
                                "Cluster %s: %d%% of tags",
                                clusterName,
                                cluster.percentClusterFound
                        ));
                    }
                } else if (tag instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection single = (AprilTagSingleDetection) tag;
                    String tagName = (single.metadata != null) ? single.metadata.name : "unknown tag";
                    telemetry.addLine(String.format("Tag (ID %d, %s):", single.id, tagName));
                } else {
                    telemetry.addLine("Unrecognized detection:");
                }

                // Null check prevents the NullPointerException crash
                if (tag.ftcPose != null) {
                    telemetry.addLine(String.format(
                            "   Range: %.1fin | Bearing: %.1fdeg | Yaw: %.1fdeg",
                            tag.ftcPose.range,
                            tag.ftcPose.bearing,
                            tag.ftcPose.yaw
                    ));
                } else if (tag instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection single = (AprilTagSingleDetection) tag;
                    telemetry.addLine(String.format(
                            "   Pose unavailable (Center at X: %.0f, Y: %.0f)",
                            single.center.x,
                            single.center.y
                    ));
                } else {
                    telemetry.addLine("   Pose unavailable");
                }

                // BIOBUZZ tags ride on moving CELLs, so robotPose is relative
                // to the tag, not an absolute field position.
                if (tag.robotPose != null) {
                    String poseLabel = (tag instanceof AprilTagClusterDetection)
                            ? "   Pose vs cluster (CELL moves; not field-absolute)"
                            : "   Robot Pose (sample tag)";
                    telemetry.addLine(String.format(
                            poseLabel + ": X %.1f  Y %.1f  Z %.1f in | Yaw %.1f deg",
                            tag.robotPose.getPosition().x,
                            tag.robotPose.getPosition().y,
                            tag.robotPose.getPosition().z,
                            tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)
                    ));
                }
                telemetry.addLine("");
            }

            if (!targetFound) {
                telemetry.addLine("Status: Target cluster \"" + DESIRED_CLUSTER + "\" not in view.");
            }

            telemetry.update();
        }

        vision.close();
    }
}
