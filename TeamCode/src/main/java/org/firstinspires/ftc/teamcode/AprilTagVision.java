package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
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
import java.util.Locale;
import java.util.concurrent.TimeUnit;

public class AprilTagVision {
    public static final String RED_SCORING = "RED SCORING";
    public static final String RED_AUDIENCE = "RED AUDIENCE";
    public static final String BLUE_AUDIENCE = "BLUE AUDIENCE";
    public static final String BLUE_SCORING = "BLUE SCORING";

    // Starting values for the Arducam OV9281 (USB 0c45:6366); tune with a sweep at shooting range.
    private static final Size CAMERA_RESOLUTION = new Size(640, 480);
    private static final long EXPOSURE_MS = 6;
    private static final int GAIN = 50;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private String initError = null;
    private String controlsError = null;
    private boolean controlsApplied = false;

    /** True when the vision portal was created without throwing. */
    public boolean isReady() {
        return visionPortal != null && initError == null;
    }

    /** Null when vision is ready, otherwise a short description of the failure. */
    public String getInitError() {
        return initError;
    }

    /** Camera state string for telemetry and logs, with measured fps while streaming. */
    public String getCameraState() {
        if (initError != null) {
            return "INIT FAILED: " + initError;
        }
        if (visionPortal == null) {
            return "OFFLINE";
        }
        try {
            VisionPortal.CameraState state = visionPortal.getCameraState();
            if (state != VisionPortal.CameraState.STREAMING) {
                return state.toString();
            }
            applyCameraControls();
            String text = String.format(Locale.US, "STREAMING %.0f fps", visionPortal.getFps());
            return controlsError == null ? text : text + " (controls: " + controlsError + ")";
        } catch (Exception e) {
            return "UNKNOWN";
        }
    }

    /**
     * Locks exposure and gain once the camera is streaming. Exposure and gain can only be set
     * on an open camera, so this runs from getDetections() and getCameraState() instead of
     * init(), which must not block.
     */
    private void applyCameraControls() {
        if (controlsApplied || visionPortal == null
                || visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return;
        }
        controlsApplied = true;
        try {
            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            if (exposure.getMode() != ExposureControl.Mode.Manual) {
                exposure.setMode(ExposureControl.Mode.Manual);
            }
            long minMs = exposure.getMinExposure(TimeUnit.MILLISECONDS);
            long maxMs = exposure.getMaxExposure(TimeUnit.MILLISECONDS);
            exposure.setExposure(Math.max(minMs, Math.min(maxMs, EXPOSURE_MS)), TimeUnit.MILLISECONDS);

            GainControl gain = visionPortal.getCameraControl(GainControl.class);
            gain.setGain(Math.max(gain.getMinGain(), Math.min(gain.getMaxGain(), GAIN)));
        } catch (Exception e) {
            controlsError = e.getClass().getSimpleName();
            return;
        }
        try {
            // Without this a dark scene stretches the exposure and drops the frame rate.
            visionPortal.getCameraControl(ExposureControl.class).setAePriority(false);
        } catch (Exception e) {
            // Not every camera exposes this control; exposure and gain are already locked.
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

            // "Webcam 1" must match the name in the robot configuration.
            // The Arducam OV9281 only exceeds 30 fps over MJPEG; the SDK default is YUY2.
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .setCameraResolution(CAMERA_RESOLUTION)
                    .addProcessor(aprilTag)
                    .build();
        } catch (Exception e) {
            initError = e.getClass().getSimpleName();
        }
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTag == null || !isReady()) return Collections.emptyList();
        applyCameraControls();
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
