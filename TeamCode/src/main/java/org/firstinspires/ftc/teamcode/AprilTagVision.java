package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

public class AprilTagVision {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Physical tag width in inches (adjust if your printed tag is different)
    private static final double TAG_SIZE_INCHES = 2.0;

    public void init(HardwareMap hardwareMap) {

        // Register Tag 34 so the SDK knows its dimensions to calculate range/bearing/yaw
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTag(34, "Tag 34", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .build();

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .build();

        // "Vision" must match the name in your Driver Station configuration
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "vision"))
                .addProcessor(aprilTag)
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTag == null) return Collections.emptyList();
        return aprilTag.getDetections();
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
