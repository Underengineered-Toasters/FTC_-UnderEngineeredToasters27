package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

public class AprilTagVision {
    public static final String RED_SCORING = "RED SCORING";
    public static final String RED_AUDIENCE = "RED AUDIENCE";
    public static final String BLUE_AUDIENCE = "BLUE AUDIENCE";
    public static final String BLUE_SCORING = "BLUE SCORING";

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private String initError = null;

    /** True when the vision portal was created without throwing. */
    public boolean isReady() {
        return visionPortal != null && initError == null;
    }

    /** Null when vision is ready, otherwise a short description of the failure. */
    public String getInitError() {
        return initError;
    }

    /** Camera state string for telemetry and logs. */
    public String getCameraState() {
        if (initError != null) {
            return "INIT FAILED: " + initError;
        }
        if (visionPortal == null) {
            return "OFFLINE";
        }
        try {
            return visionPortal.getCameraState().toString();
        } catch (Exception e) {
            return "UNKNOWN";
        }
    }


    public void init(HardwareMap hardwareMap) {
        try {
            // Current game library includes BIOBUZZ clusters (IDs 30-45, 3.25 in).
            // Detections may be AprilTagClusterDetection or AprilTagSingleDetection.
            // CELL tags move: robotPose is for relative targeting, not absolute field localization.
            aprilTag = new AprilTagProcessor.Builder()
                    .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .build();

            // "Webcam 1" must match the name in the robot configuration
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception e) {
            initError = e.getClass().getSimpleName();
        }
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTag == null || !isReady()) return Collections.emptyList();
        return aprilTag.getDetections();
    }
    public AprilTagClusterDetection findCluster(String name) {
        for (AprilTagDetection detection : getDetections()) {
            if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if (cluster.metadata != null && name != null
                        && name.equals(cluster.metadata.name) && cluster.ftcPose != null) {
                    return cluster;
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
