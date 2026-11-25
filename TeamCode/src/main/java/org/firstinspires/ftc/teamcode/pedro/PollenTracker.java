package org.firstinspires.ftc.teamcode.pedro;

import android.graphics.Color;
import android.util.Size;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.opencv.core.Scalar;

import java.util.List;

/** Tracks the largest circular yellow object in the webcam view. */
@TeleOp(name = "Pollen Tracker", group = "Vision")
public class PollenTracker extends LinearOpMode {
    private static final String WEBCAM_NAME = "Webcam 1";
    private static final int FRAME_WIDTH = 320;
    private static final int FRAME_HEIGHT = 240;
    // Replace with the radius reported by telemetry when the pollen is exactly 12 inches away.
    private static final double TARGET_RADIUS_AT_12_INCHES = 30.0;
    private static final double RADIUS_DEADBAND = 3.0;
    private static final double MIN_BLOB_AREA = 100.0;
    private static final double MIN_CIRCULARITY = 0.55;
    private static final double TURN_KP = 0.85;
    private static final double MAX_TURN = 0.35;
    private static final double FORWARD_POWER = 0.22;
    private static final double REVERSE_POWER = 0.16;

    /*
     * OpenCV HSV: hue is 0-180, saturation and brightness are 0-255.
     * This deliberately accepts only vivid yellow (about 36-76 degrees), excluding red and blue.
     */
    private static final ColorRange VIVID_YELLOW = new ColorRange(
            ColorSpace.HSV,
            new Scalar(18, 130, 100),
            new Scalar(38, 255, 255)
    );

    @Override
    public void runOpMode() {
        Follower follower = Constants.create(hardwareMap);
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(VIVID_YELLOW)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.YELLOW)
                .setBlurSize(5)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(new Size(FRAME_WIDTH, FRAME_HEIGHT))
                .addProcessor(colorLocator)
                .build();
        boolean dashboardStreamStarted = false;
        try {
            FtcDashboard.getInstance().startCameraStream(portal, 30);
            dashboardStreamStarted = true;
        } catch (Exception ignored) {
            // Keep pollen tracking available if Dashboard streaming is unavailable.
        }

        telemetry.addLine("Pollen Tracker is ready.");
        telemetry.addLine("A enables/disables tracking. B stops it immediately.");
        telemetry.addLine("Robot stops whenever it loses the yellow ball.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            stopDashboardCameraStream(dashboardStreamStarted);
            portal.close();
            follower.stop();
            return;
        }

        follower.setPose(Pose.zero());
        boolean enabled = false;
        boolean previousA = false;

        try {
            while (opModeIsActive()) {
                if (gamepad1.a && !previousA) enabled = !enabled;
                previousA = gamepad1.a;
                if (gamepad1.b) enabled = false;

                List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
                ColorBlobLocatorProcessor.Util.filterByCriteria(
                        ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                        MIN_BLOB_AREA, Double.MAX_VALUE, blobs);
                ColorBlobLocatorProcessor.Util.filterByCriteria(
                        ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                        MIN_CIRCULARITY, 1.0, blobs);

                ColorBlobLocatorProcessor.Blob target = blobs.isEmpty() ? null : blobs.get(0);
                if (enabled && target != null) {
                    Circle circle = target.getCircle();
                    double horizontalError = (circle.getX() - FRAME_WIDTH / 2.0) / (FRAME_WIDTH / 2.0);
                    double turn = Range.clip(-horizontalError * TURN_KP, -MAX_TURN, MAX_TURN);
                    double radiusError = TARGET_RADIUS_AT_12_INCHES - circle.getRadius();
                    double forward = 0.0;

                    if (Math.abs(horizontalError) < 0.18) {
                        if (radiusError > RADIUS_DEADBAND) forward = FORWARD_POWER;
                        if (radiusError < -RADIUS_DEADBAND) forward = -REVERSE_POWER;
                    }

                    follower.manual(forward, 0.0, turn);
                    telemetry.addData("Pollen center", "%.0f, %.0f", circle.getX(), circle.getY());
                    telemetry.addData("Pollen radius (px)", "%.1f", circle.getRadius());
                    telemetry.addData("Estimated range (in)", "%.1f",
                            TARGET_RADIUS_AT_12_INCHES * 12.0 / circle.getRadius());
                    telemetry.addData("Drive", "%.2f  Turn %.2f", forward, turn);
                } else {
                    follower.manual(0.0, 0.0, 0.0);
                    telemetry.addData("Pollen", target == null ? "not found - stopped" : "tracking disabled");
                }

                follower.update();
                telemetry.addData("Tracking", enabled ? "ENABLED" : "DISABLED");
                telemetry.addData("Camera", portal.getCameraState());
                telemetry.addLine("A toggle | B emergency stop");
                telemetry.update();
                idle();
            }
        } finally {
            follower.stop();
            stopDashboardCameraStream(dashboardStreamStarted);
            portal.close();
        }
    }

    private static void stopDashboardCameraStream(boolean started) {
        if (started) {
            try {
                FtcDashboard.getInstance().stopCameraStream();
            } catch (Exception ignored) {
                // The camera portal must still close if Dashboard shutdown fails.
            }
        }
    }
}
