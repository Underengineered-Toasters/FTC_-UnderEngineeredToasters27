package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "AprilTag Test", group = "Vision")
public class AprilTagTestOpMode extends LinearOpMode {

    private final AprilTagVision vision = new AprilTagVision();
    private static final String DESIRED_TARGET = AprilTagVision.RED_AUDIENCE;

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
            telemetry.addData("Looking for Target", DESIRED_TARGET);
            telemetry.addLine("--------------------------------");

            for (AprilTagDetection tag : detections) {
                String tagName = AprilTagTargets.nameOf(tag);
                if (DESIRED_TARGET.equals(tagName)) {
                    targetFound = true;
                    telemetry.addLine(String.format(
                            ">> TARGET FOUND (%s, ID/group %s) <<",
                            tagName,
                            AprilTagTargets.identityOf(tag)
                    ));
                } else {
                    telemetry.addLine(String.format(
                            "Tag (ID/group %s, %s):",
                            AprilTagTargets.identityOf(tag),
                            tagName
                    ));
                }

                // Null check prevents NullPointerException
                if (tag.ftcPose != null) {
                    telemetry.addLine(String.format(
                            "   Range: %.1fin | Bearing: %.1fdeg | Yaw: %.1fdeg",
                            tag.ftcPose.range,
                            tag.ftcPose.bearing,
                            tag.ftcPose.yaw
                    ));
                    // BIOBUZZ tags ride on moving CELLs, so ftcPose is relative
                    // to the tag, not an absolute field position.
                    telemetry.addLine(String.format(
                            "   Relative Pose: X %.1f  Y %.1f  Z %.1f in | Yaw %.1f deg",
                            tag.ftcPose.x,
                            tag.ftcPose.y,
                            tag.ftcPose.z,
                            tag.ftcPose.yaw
                    ));
                } else if (AprilTagTargets.centerOf(tag) != null) {
                    telemetry.addLine(String.format(
                            "   Pose unavailable (Center at X: %.0f, Y: %.0f)",
                            AprilTagTargets.centerOf(tag).x,
                            AprilTagTargets.centerOf(tag).y
                    ));
                } else {
                    telemetry.addLine("   Pose unavailable");
                }

                telemetry.addLine("");
            }

            if (!targetFound) {
                telemetry.addLine("Status: Target \"" + DESIRED_TARGET + "\" not in view.");
            }

            telemetry.update();
            sleep(20);
        }

        vision.close();
    }
}
