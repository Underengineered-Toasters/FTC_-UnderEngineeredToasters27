package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "AprilTag Test", group = "Vision")
public class AprilTagTestOpMode extends LinearOpMode {

    private AprilTagVision vision = new AprilTagVision();
    private static final int DESIRED_TAG_ID = 34;

    @Override
    public void runOpMode() {

        vision.init(hardwareMap);

        telemetry.addLine("Vision initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = vision.getDetections();
            boolean targetFound = false;

            telemetry.addData("Tags Visible", detections.size());
            telemetry.addData("Looking for ID", DESIRED_TAG_ID);
            telemetry.addLine("--------------------------------");

            for (AprilTagDetection tag : detections) {
                if (tag instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection single = (AprilTagSingleDetection) tag;
                    if (single.id == DESIRED_TAG_ID) {
                        targetFound = true;
                        telemetry.addLine(String.format(">> TARGET TAG (ID %d) FOUND! <<", single.id));
                    } else {
                        telemetry.addLine(String.format("Other Tag (ID %d):", single.id));
                    }
                } else {
                    telemetry.addLine("Tag cluster detected:");
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

                // Field pose only exists for tags with real field positions
                // (official DECODE tags) — custom test tags 30-45 have none.
                if (tag.robotPose != null) {
                    telemetry.addLine(String.format(
                            "   Robot Pose: X %.1f  Y %.1f  Z %.1f in | Yaw %.1f deg",
                            tag.robotPose.getPosition().x,
                            tag.robotPose.getPosition().y,
                            tag.robotPose.getPosition().z,
                            tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)
                    ));
                }
                telemetry.addLine("");
            }

            if (!targetFound) {
                telemetry.addLine("Status: Target tag 34 not in view.");
            }

            telemetry.update();
        }

        vision.close();
    }
}
