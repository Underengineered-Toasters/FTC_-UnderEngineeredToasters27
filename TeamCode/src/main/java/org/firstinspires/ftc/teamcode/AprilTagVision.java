package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
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
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

public class AprilTagVision {
    public static final String RED_SCORING = "RED SCORING";
    public static final String RED_AUDIENCE = "RED AUDIENCE";
    public static final String BLUE_AUDIENCE = "BLUE AUDIENCE";
    public static final String BLUE_SCORING = "BLUE SCORING";

    // Arducam OV9281 (USB 0c45:6366). Exposure and gain are adjusted at runtime to hold the
    // frame brightness near BRIGHTNESS_TARGET, so tag detection survives different venue lighting.
    private static final Size CAMERA_RESOLUTION = new Size(640, 480);
    private static final long EXPOSURE_START_MS = 6;
    private static final long EXPOSURE_MIN_MS = 1;
    private static final long EXPOSURE_MAX_MS = 10;   // longer blurs tags while the robot moves
    private static final int GAIN_START = 50;
    private static final int GAIN_MIN = 0;
    private static final int GAIN_MAX = 200;
    private static final double BRIGHTNESS_TARGET = 110;  // mean 8-bit gray
    private static final double BRIGHTNESS_DEADBAND = 25;
    private static final double STEP = 1.25;
    private static final long ADJUST_INTERVAL_MS = 300;

    // Hardware configuration names. The Logitech C270 needs a "Webcam 2" entry to be used.
    private static final String PRIMARY_CAMERA = "Webcam 1";
    private static final String FALLBACK_CAMERA = "Webcam 2";

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private String initError = null;
    private String controlsError = null;
    private boolean controlsApplied = false;
    private final BrightnessMeter brightness = new BrightnessMeter();
    private long exposureMs = EXPOSURE_START_MS;
    private int gainValue = GAIN_START;
    private long lastAdjustMs = 0;

    /** Measures the mean gray level of each frame on a small downscaled copy. */
    private static class BrightnessMeter implements VisionProcessor {
        private final Mat small = new Mat();
        volatile double mean = -1;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {}

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.resize(frame, small, new org.opencv.core.Size(80, 60), 0, 0, Imgproc.INTER_AREA);
            mean = Core.mean(small).val[0];
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
    }

    private boolean usingFallback = false;

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
            adjustExposure();
            String text = String.format(Locale.US, "%sSTREAMING %.0f fps exp=%d ms gain=%d lum=%.0f",
                    usingFallback ? "FALLBACK Logitech " : "", visionPortal.getFps(),
                    exposureMs, gainValue, brightness.mean);
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
        if (usingFallback || controlsApplied || visionPortal == null
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
            exposureMs = Math.max(minMs, Math.min(maxMs, EXPOSURE_START_MS));
            exposure.setExposure(exposureMs, TimeUnit.MILLISECONDS);

            GainControl gain = visionPortal.getCameraControl(GainControl.class);
            gainValue = Math.max(gain.getMinGain(), Math.min(gain.getMaxGain(), GAIN_START));
            gain.setGain(gainValue);
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

    /**
     * Nudges exposure and gain toward BRIGHTNESS_TARGET. When dark it raises exposure to
     * EXPOSURE_MAX_MS first, then gain; when bright it lowers gain first, then exposure.
     * Throttled so each change takes effect before the next measurement.
     */
    private void adjustExposure() {
        if (!controlsApplied || controlsError != null || usingFallback || brightness.mean < 0) {
            return;
        }
        long now = System.currentTimeMillis();
        if (now - lastAdjustMs < ADJUST_INTERVAL_MS) {
            return;
        }
        lastAdjustMs = now;
        double error = BRIGHTNESS_TARGET - brightness.mean;
        if (Math.abs(error) < BRIGHTNESS_DEADBAND) {
            return;
        }
        try {
            ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
            GainControl gain = visionPortal.getCameraControl(GainControl.class);
            long maxMs = Math.min(EXPOSURE_MAX_MS, exposure.getMaxExposure(TimeUnit.MILLISECONDS));
            long minMs = Math.max(EXPOSURE_MIN_MS, exposure.getMinExposure(TimeUnit.MILLISECONDS));
            int maxGain = Math.min(GAIN_MAX, gain.getMaxGain());
            int minGain = Math.max(GAIN_MIN, gain.getMinGain());
            if (error > 0) {
                if (exposureMs < maxMs) {
                    exposureMs = Math.min(maxMs, Math.max(exposureMs + 1, Math.round(exposureMs * STEP)));
                } else if (gainValue < maxGain) {
                    gainValue = Math.min(maxGain, Math.max(gainValue + 1, (int) Math.round(gainValue * STEP)));
                } else {
                    return;
                }
            } else {
                if (gainValue > minGain) {
                    // The -1 keeps every step moving: at small values rounding 2/1.25 lands back on 2.
                    gainValue = Math.max(minGain, Math.min(gainValue - 1, (int) Math.round(gainValue / STEP)));
                } else if (exposureMs > minMs) {
                    exposureMs = Math.max(minMs, Math.min(exposureMs - 1, Math.round(exposureMs / STEP)));
                } else {
                    return;
                }
            }
            exposure.setExposure(exposureMs, TimeUnit.MILLISECONDS);
            gain.setGain(gainValue);
        } catch (Exception e) {
            controlsError = e.getClass().getSimpleName();
        }
    }

    private AprilTagProcessor newProcessor() {
        // Current game library includes BIOBUZZ clusters (IDs 30-45, 3.25 in).
        // Detections may be AprilTagClusterDetection or AprilTagSingleDetection.
        // CELL tags move: robotPose is for relative targeting, not absolute field localization.
        return new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
    }

    /**
     * Opens the Arducam, or the Logitech C270 when the Arducam is not configured, not plugged in,
     * or fails to open. Both names must match the robot configuration.
     */
    public void init(HardwareMap hardwareMap) {
        WebcamName primary = hardwareMap.tryGet(WebcamName.class, PRIMARY_CAMERA);
        if (primary != null && primary.isAttached()) {
            try {
                // MJPEG would allow 120 fps, but on this robot the SDK delivers only black frames
                // in MJPEG (measured brightness 0.0 vs about 210 in YUY2). Requesting YUY2
                // explicitly also suppresses the SDK's "increase your FPS" warning.
                aprilTag = newProcessor();
                visionPortal = new VisionPortal.Builder()
                        .setCamera(primary)
                        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                        .setCameraResolution(CAMERA_RESOLUTION)
                        .addProcessor(aprilTag)
                        .addProcessor(brightness)
                        .build();
                return;
            } catch (Exception e) {
                initError = e.getClass().getSimpleName();
            }
        }

        WebcamName fallback = hardwareMap.tryGet(WebcamName.class, FALLBACK_CAMERA);
        if (fallback == null || !fallback.isAttached()) {
            if (initError == null) {
                initError = "NoCamera";
            }
            return;
        }
        try {
            // The Logitech has a built-in SDK calibration and works at the SDK defaults.
            aprilTag = newProcessor();
            visionPortal = new VisionPortal.Builder()
                    .setCamera(fallback)
                    .addProcessor(aprilTag)
                    .build();
            usingFallback = true;
            initError = null;
        } catch (Exception e) {
            initError = e.getClass().getSimpleName();
        }
    }

    public List<AprilTagDetection> getDetections() {
        if (aprilTag == null || !isReady()) return Collections.emptyList();
        applyCameraControls();
        adjustExposure();
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
