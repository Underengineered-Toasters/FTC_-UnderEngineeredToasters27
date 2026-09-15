package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Wraps AprilTag detection using a USB webcam.
 * Handles setup once; call getDetections() each loop to read current tags in view.
 */
public class AprilTagVision {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public void init(HardwareMap hardwareMap) {

        // Build the AprilTag processor. Defaults are fine to start —
        // it auto-detects tag size/family for the standard FTC tag library.
        aprilTag = new AprilTagProcessor.Builder().build();

        // VisionPortal owns the camera stream and feeds frames to the processor.
        // "Webcam 1" must match EXACTLY what you named it in the Driver Station config.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /** Returns every AprilTag currently visible in the camera frame. */
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    /** Convenience: returns the first detection matching a specific tag ID, or null if not seen. */
    public AprilTagDetection getDetectionById(int tagId) {
        for (AprilTagDetection detection : getDetections()) {
            if (detection.id == tagId) {
                return detection;
            }
        }
        return null;
    }

    /** Call when the OpMode ends to free the camera cleanly. */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
