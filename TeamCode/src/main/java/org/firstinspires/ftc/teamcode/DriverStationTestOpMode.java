package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ds.DriverStation;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * End-to-end test for the driver station web UI: attaches diagnostics,
 * streams the camera, and pushes telemetry plus a heartbeat log line.
 */
@TeleOp(name = "Driver Station Test", group = "ds")
public class DriverStationTestOpMode extends LinearOpMode {

    private final AprilTagVision vision = new AprilTagVision();

    @Override
    public void runOpMode() {
        DriverStation.attach(this);
        try {
            vision.init(hardwareMap);
            DriverStation.setCameraSource(vision.getCameraSource());
        } catch (Exception e) {
            DriverStation.log("vision init failed: " + e.getMessage());
        }

        DriverStation.log("Driver Station Test initialized");
        telemetry.addLine("Driver Station Test: waiting for start");
        telemetry.update();

        waitForStart();

        int heartbeat = 0;
        while (opModeIsActive()) {
            List<AprilTagDetection> detections = vision.getDetections();
            DriverStation.telemetry("AprilTag Targets", detections.size());
            DriverStation.telemetry("Heartbeat", heartbeat);
            if (heartbeat % 4 == 0) {
                DriverStation.log("heartbeat " + heartbeat + ", targets=" + detections.size());
            }
            heartbeat++;
            sleep(250);
        }

        DriverStation.log("Driver Station Test stopping");
        DriverStation.detach();
        vision.close();
    }
}
