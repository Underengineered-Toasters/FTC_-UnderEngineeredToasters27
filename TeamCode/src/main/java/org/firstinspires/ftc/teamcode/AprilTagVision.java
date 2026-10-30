package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

public class AprilTagVision {

    public static final String RED_SCORING  = "RED SCORING";
    public static final String RED_AUDIENCE = "RED AUDIENCE";
    public static final String BLUE_AUDIENCE = "BLUE AUDIENCE";
    public static final String BLUE_SCORING  = "BLUE SCORING";

    // Red Alliance Target Tag IDs
    public static final int[] RED_TAGS_PRIMARY   = {30, 31, 32, 33};
    public static final int[] RED_TAGS_SECONDARY = {34, 35, 36, 37};

    // Blue Alliance Target Tag IDs
    public static final int[] BLUE_TAGS_PRIMARY   = {38, 39, 40, 41};
    public static final int[] BLUE_TAGS_SECONDARY = {42, 43, 44, 45};

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Physical tag width in inches
    private static final double TAG_SIZE_INCHES = 3.25;

    public void init(HardwareMap hardwareMap) {

        // Register tags 30–45 for BIOBUZZ
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTag(30, RED_AUDIENCE, TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(31, RED_AUDIENCE, TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(32, RED_SCORING, TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(33, RED_SCORING, TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(34, BLUE_AUDIENCE, TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(35, BLUE_AUDIENCE, TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(36, BLUE_SCORING, TAG_SIZE_INCHES, DistanceUnit.INCH)
                .addTag(37, BLUE_SCORING, TAG_SIZE_INCHES, DistanceUnit.INCH)
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

        // "Webcam 1" must match the name in Driver Station configuration
        try {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception e) {
            visionPortal = null;
        }
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTag == null) return Collections.emptyList();
        return aprilTag.getDetections();
    }

    public AprilTagDetection findTagByName(String name) {
        for (AprilTagDetection detection : getDetections()) {
            if (detection.metadata != null && name != null
                    && name.equals(detection.metadata.name) && detection.ftcPose != null) {
                return detection;
            }
        }
        return null;
    }

    public AprilTagDetection findTagInGroup(int[] tagIds) {
        for (AprilTagDetection detection : getDetections()) {
            if (detection.ftcPose != null) {
                for (int id : tagIds) {
                    if (detection.id == id) {
                        return detection;
                    }
                }
            }
        }
        return null;
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
