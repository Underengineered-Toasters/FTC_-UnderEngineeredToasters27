package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Driver Station Test", group = "Vision")
public class DriverStationTestOpMode extends LinearOpMode {

    private final AprilTagVision vision = new AprilTagVision();

    @Override
    public void runOpMode() {

        vision.init(hardwareMap);

        telemetry.addLine("Driver Station Test initialized.");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = vision.getDetections();

            telemetry.addData("Driver Station Connection", "Active");
            telemetry.addData("Total Detections", detections.size());
            telemetry.addLine("--------------------------------");

            for (AprilTagDetection detection : detections) {
                String name = AprilTagTargets.nameOf(detection);
                telemetry.addLine(String.format("Tag ID/group %s (%s):", AprilTagTargets.identityOf(detection), name));

                if (detection.ftcPose != null) {
                    telemetry.addLine(String.format(
                            "   X: %.1fin | Y: %.1fin | Z: %.1fin",
                            detection.ftcPose.x,
                            detection.ftcPose.y,
                            detection.ftcPose.z
                    ));
                    telemetry.addLine(String.format(
                            "   Range: %.1fin | Bearing: %.1fdeg | Yaw: %.1fdeg",
                            detection.ftcPose.range,
                            detection.ftcPose.bearing,
                            detection.ftcPose.yaw
                    ));
                } else if (AprilTagTargets.centerOf(detection) != null) {
                    telemetry.addLine(String.format(
                            "   2D Center: X %.0f, Y %.0f",
                            AprilTagTargets.centerOf(detection).x,
                            AprilTagTargets.centerOf(detection).y
                    ));
                }
            }

            telemetry.update();
            sleep(20);
        }

        vision.close();
    }
}