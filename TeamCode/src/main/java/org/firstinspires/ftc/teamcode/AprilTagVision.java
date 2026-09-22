package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
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

        // Official DECODE field tags carry real field positions, so detection.robotPose
        // gives field X/Y/Z + orientation. Custom tags 30-45 are merged in for ID/range
        // testing but have no field position, so they cannot produce a robot pose.
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .addTag(30, "Tag 30", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(31, "Tag 31", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(32, "Tag 32", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(33, "Tag 33", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(34, "Tag 34", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(35, "Tag 35", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(36, "Tag 36", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(37, "Tag 37", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(38, "Tag 38", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(39, "Tag 39", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(40, "Tag 40", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(41, "Tag 41", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(42, "Tag 42", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(43, "Tag 43", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(44, "Tag 44", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(45, "Tag 45", TAG_SIZE_INCHES, DistanceUnit.INCH)
                .build();

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();


        // "Webcam 1" must match the name in the robot configuration
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTag == null) return Collections.emptyList();
        return aprilTag.getDetections();
    }

    public CameraStreamSource getCameraSource() {
        return visionPortal;
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
