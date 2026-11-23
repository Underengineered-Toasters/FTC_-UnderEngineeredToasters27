package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Yellow-ball detection on the external webcam (Logitech C270 or any other UVC webcam).
 *
 * <p>The C270 needs no vendor SDK on Android: it is a standard USB UVC camera, and the FTC SDK
 * opens it through {@link WebcamName} + {@link VisionPortal}. Configure it in the Driver Station
 * robot configuration under the name in {@link #cameraName} ("Webcam 1"), plug it into the Control
 * Hub, and it streams at {@link #imageWidth} x {@link #imageHeight}.
 *
 * <p>Detection uses the SDK's own {@link ColorBlobLocatorProcessor} with the predefined
 * {@link ColorRange#YELLOW} (a YCrCb range, which is far more stable under gym lighting than raw
 * RGB). Each blob is filtered for "ball-ness" (radius, area, circularity, aspect ratio) and then
 * converted to a distance and a bearing:
 *
 * <pre>
 *   focalPixels = 0.5 * hypot(width, height) / tan(diagonalFov / 2)
 *   distance    = focalPixels * ballDiameter / measuredDiameter      (similar triangles)
 *   bearing     = atan((x - width / 2) / focalPixels)                (positive = ball is right)
 * </pre>
 *
 * <p>{@code bearing} is positive to the robot's right, which is the same sign {@link Robot#drive}
 * uses for a positive turn - so a chase controller can use it directly. This is the OPPOSITE
 * convention from {@code ftcPose.bearing} (SDK vision is positive to the left), so do not multiply
 * this value by {@link Robot#SDK_YAW_TO_TURN}.
 *
 * <p>Only one OpMode may own the camera at a time. A robot that needs AprilTag and yellow-ball
 * vision at the same time must add this processor to a shared portal instead of building a second
 * one; see the shared-portal note in {@code DESIGN.md}.
 */
public class YellowBallVision {

    // ============================================================
    // Camera - must match the Driver Station robot configuration
    // ============================================================

    /** Robot-configuration name of the C270. */
    public static String cameraName = "Webcam 1";

    /** Capture resolution; must match the portal's {@code setCameraResolution}. 640x480 measures
     *  small/far balls better, 320x240 is faster. */
    public static int imageWidth = 640;
    public static int imageHeight = 480;

    /** C270 datasheet diagonal field of view. CALIBRATE if distances read consistently short/long. */
    public static double diagonalFovDegrees = 60.0;

    // ============================================================
    // Target - CALIBRATE
    // ============================================================

    /** Real outside diameter of the yellow ball, in inches. */
    public static double ballDiameterInches = 3.0;

    /** Colour match. Replace with a custom range if the field lighting needs it, e.g.
     *  {@code new ColorRange(ColorSpace.HSV, new Scalar(20, 120, 120), new Scalar(35, 255, 255))}
     *  (OpenCV HSV: H 0-180, S/V 0-255). */
    public static ColorRange targetColorRange = ColorRange.YELLOW;

    /**
     * Mounting correction: subtract this from every measured bearing when the camera is not
     * pointed exactly along the robot's forward axis. Point the robot at a ball, read the
     * reported bearing in {@code Yellow Ball Camera Test}, and set this to that value so a
     * centred ball reads 0.
     */
    public static double bearingOffsetDegrees = 0.0;

    /** Blobs smaller than this radius (pixels) are ignored. */
    public static double minRadiusPixels = 4.0;

    /** Blobs with less painted area (pixels^2) than this are ignored. */
    public static double minContourArea = 80.0;

    /** 1.0 is a perfect circle. Field lighting and partial occlusion lower this. */
    public static double minCircularity = 0.52;

    /** Rejects long yellow streaks (tape, shirts) - a ball's box is close to square. */
    public static double maxAspectRatio = 1.70;

    /** Detections returned per frame, nearest first. */
    public static int maxDetections = 5;

    private ColorBlobLocatorProcessor colorLocator;
    private VisionPortal visionPortal;
    private String failure;

    // ============================================================
    // Lifecycle
    // ============================================================

    /** Opens the camera and starts streaming. Never throws: a missing camera becomes {@link #getStatus}. */
    public void init(HardwareMap hardwareMap) {
        failure = null;
        try {
            colorLocator = new ColorBlobLocatorProcessor.Builder()
                    .setTargetColorRange(targetColorRange)
                    .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                    .setRoi(ImageRegion.entireFrame())
                    .setDrawContours(true)
                    .setBoxFitColor(0)
                    .setCircleFitColor(Color.rgb(255, 214, 0))
                    .setBlurSize(5)
                    .build();

            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, cameraName))
                    .setCameraResolution(new Size(imageWidth, imageHeight))
                    .addProcessor(colorLocator)
                    .setShowStatsOverlay(true)
                    .build();
        } catch (Exception e) {
            colorLocator = null;
            visionPortal = null;
            failure = e.getClass().getSimpleName() + ": " + e.getMessage();
        }
    }

    /** True once frames are actually flowing; a freshly built portal is still opening. */
    public boolean isStreaming() {
        return visionPortal != null
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING;
    }

    /** Human-readable camera state for telemetry and autonomous pre-flight checks. */
    public String getStatus() {
        if (visionPortal == null) {
            return failure == null ? "not initialized" : "camera unavailable (" + failure + ")";
        }
        String state = visionPortal.getCameraState() == null
                ? "unknown" : visionPortal.getCameraState().toString();
        if (visionPortal.getCameraState() == VisionPortal.CameraState.ERROR && failure != null) {
            return state + " (" + failure + ")";
        }
        return state;
    }

    public float getFps() {
        return visionPortal != null ? visionPortal.getFps() : 0.0f;
    }

    /** Exposes the portal to FTC Dashboard / the Driver Station camera stream. */
    public CameraStreamSource getCameraSource() {
        return visionPortal;
    }

    public void close() {
        if (visionPortal != null) {
            try {
                visionPortal.close();
            } catch (Exception ignored) {
                // shutting down anyway
            }
            visionPortal = null;
        }
        colorLocator = null;
    }

    // ============================================================
    // Detections
    // ============================================================

    /** Yellow-ball detections for the current frame, nearest (largest) first. */
    public List<Detection> getDetections() {
        if (colorLocator == null) {
            return Collections.emptyList();
        }

        List<ColorBlobLocatorProcessor.Blob> blobs;
        try {
            blobs = colorLocator.getBlobs();
        } catch (Exception e) {
            return Collections.emptyList();
        }
        if (blobs == null || blobs.isEmpty()) {
            return Collections.emptyList();
        }

        double focal = focalPixels();
        List<Detection> detections = new ArrayList<>();
        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            Circle circle;
            try {
                circle = blob.getCircle();
            } catch (Exception e) {
                continue;
            }
            if (circle == null) {
                continue;
            }

            double radius = circle.getRadius();
            double area = blob.getContourArea();
            double circularity = blob.getCircularity();
            double aspectRatio = blob.getAspectRatio();

            if (radius < minRadiusPixels || area < minContourArea) {
                continue;
            }
            if (circularity < minCircularity || aspectRatio > maxAspectRatio) {
                continue;
            }

            detections.add(new Detection(
                    circle.getX(),
                    circle.getY(),
                    radius,
                    area,
                    circularity,
                    bearingDegrees(circle.getX(), imageWidth, focal) - bearingOffsetDegrees,
                    distanceInches(radius, focal, ballDiameterInches)));
        }

        // Nearest first: a bigger apparent radius means a closer ball.
        Collections.sort(detections, (a, b) -> Double.compare(b.radiusPixels, a.radiusPixels));
        if (detections.size() > maxDetections) {
            return new ArrayList<>(detections.subList(0, maxDetections));
        }
        return detections;
    }

    /** Closest usable ball, or null when none is visible. */
    public Detection bestTarget() {
        List<Detection> detections = getDetections();
        return detections.isEmpty() ? null : detections.get(0);
    }

    // ============================================================
    // Geometry (pure - unit tested)
    // ============================================================

    /** Focal length in pixels implied by the configured resolution and field of view. */
    public static double focalPixels() {
        return focalPixels(imageWidth, imageHeight, diagonalFovDegrees);
    }

    public static double focalPixels(int width, int height, double diagonalFovDegrees) {
        if (width <= 0 || height <= 0) {
            throw new IllegalArgumentException("image size must be positive");
        }
        if (diagonalFovDegrees <= 0.0 || diagonalFovDegrees >= 180.0) {
            throw new IllegalArgumentException(
                    "diagonalFovDegrees must be between 0 and 180, was " + diagonalFovDegrees);
        }
        double diagonalPixels = Math.hypot(width, height);
        return 0.5 * diagonalPixels / Math.tan(Math.toRadians(diagonalFovDegrees) / 2.0);
    }

    /** Horizontal field of view in degrees, derived from the focal length. */
    public static double horizontalFovDegrees(int width, double focalPixels) {
        return Math.toDegrees(2.0 * Math.atan(0.5 * width / focalPixels));
    }

    /**
     * Ball bearing in degrees, positive to the robot's right (the same sign as a positive
     * {@link Robot#drive} turn). The image centre is 0 degrees.
     */
    public static double bearingDegrees(double centerXPixels, int width, double focalPixels) {
        if (focalPixels <= 0.0) {
            throw new IllegalArgumentException("focalPixels must be positive");
        }
        double offset = centerXPixels - 0.5 * width;
        return Math.toDegrees(Math.atan2(offset, focalPixels));
    }

    /**
     * Distance in inches from the apparent radius of the ball (similar triangles).
     * Returns {@link Double#POSITIVE_INFINITY} when the blob has no usable radius.
     */
    public static double distanceInches(double radiusPixels, double focalPixels,
                                        double ballDiameterInches) {
        if (radiusPixels <= 0.0 || focalPixels <= 0.0 || ballDiameterInches <= 0.0) {
            return Double.POSITIVE_INFINITY;
        }
        return focalPixels * ballDiameterInches / (2.0 * radiusPixels);
    }

    // ============================================================
    // Value type
    // ============================================================

    /** One yellow ball seen in the current frame. */
    public static final class Detection {
        /** Blob centre, pixels, origin top-left. */
        public final double centerXPixels;
        public final double centerYPixels;

        /** Enclosing-circle radius in pixels. */
        public final double radiusPixels;

        /** Painted area in pixels^2. */
        public final double contourArea;

        /** 1.0 for a perfect circle. */
        public final double circularity;

        /** Positive = ball is to the robot's right. */
        public final double bearingDegrees;

        /** Estimated range in inches; {@link Double#POSITIVE_INFINITY} when unmeasurable. */
        public final double distanceInches;

        /** Detection built from measured blob and geometry values (also used by tests/simulation). */
        public Detection(double centerXPixels, double centerYPixels, double radiusPixels,
                         double contourArea, double circularity, double bearingDegrees,
                         double distanceInches) {
            this.centerXPixels = centerXPixels;
            this.centerYPixels = centerYPixels;
            this.radiusPixels = radiusPixels;
            this.contourArea = contourArea;
            this.circularity = circularity;
            this.bearingDegrees = bearingDegrees;
            this.distanceInches = distanceInches;
        }

        @Override
        public String toString() {
            return String.format(java.util.Locale.US,
                    "ball %.1f in at %.1f deg (r=%.1f px, area=%.0f)",
                    distanceInches, bearingDegrees, radiusPixels, contourArea);
        }
    }
}
