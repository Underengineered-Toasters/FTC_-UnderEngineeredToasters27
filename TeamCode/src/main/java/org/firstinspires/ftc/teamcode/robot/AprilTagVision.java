package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

public class AprilTagVision {

    public static final String RED_SCORING  = "RED SCORING";
    public static final String RED_AUDIENCE = "RED AUDIENCE";
    public static final String BLUE_AUDIENCE = "BLUE AUDIENCE";
    public static final String BLUE_SCORING  = "BLUE SCORING";

    // SDK BIOBUZZ cluster members: primary = scoring, secondary = audience for red.
    public static final int[] RED_TAGS_PRIMARY   = {30, 31, 32, 33};
    public static final int[] RED_TAGS_SECONDARY = {34, 35, 36, 37};

    // SDK BIOBUZZ cluster members: primary = audience, secondary = scoring for blue.
    public static final int[] BLUE_TAGS_PRIMARY   = {38, 39, 40, 41};
    public static final int[] BLUE_TAGS_SECONDARY = {42, 43, 44, 45};

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public void init(HardwareMap hardwareMap) {

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
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
            if (name != null
                    && name.equals(AprilTagTargets.nameOf(detection)) && detection.ftcPose != null) {
                return detection;
            }
        }
        return null;
    }

    public AprilTagDetection findTagInGroup(int[] tagIds) {
        for (AprilTagDetection detection : getDetections()) {
            if (detection.ftcPose != null) {
                for (int id : tagIds) {
                    if (AprilTagTargets.matchesId(detection, id)) {
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
