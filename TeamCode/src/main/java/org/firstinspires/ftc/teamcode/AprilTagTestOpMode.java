package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "AprilTag Test", group = "Vision")
public class AprilTagTestOpMode extends LinearOpMode {

    private AprilTagVision vision = new AprilTagVision();

    @Override
    public void runOpMode() {

        vision.init(hardwareMap);

        telemetry.addLine("Vision initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = vision.getDetections();

            telemetry.addData("Tags visible", detections.size());

            for (AprilTagDetection tag : detections) {
                telemetry.addLine(String.format(
                        "ID %d (%s): range=%.1fin, bearing=%.1fdeg, yaw=%.1fdeg",
                        tag.id,
                        tag.metadata != null ? tag.metadata.name : "unknown",
                        tag.ftcPose.range,
                        tag.ftcPose.bearing,
                        tag.ftcPose.yaw
                ));
            }

            telemetry.update();
        }

        vision.close();
    }
}
