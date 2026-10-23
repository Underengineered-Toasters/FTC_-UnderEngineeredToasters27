package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.Locale;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Saves checkerboard frames for lens calibration of the Arducam OV9281.
 * Frames use the same format and resolution as AprilTagVision, because the calibration is only
 * valid at the resolution it was captured at. Press START, then move the camera in front of a
 * checkerboard. Frames are written to /sdcard/FIRST/calib on the Control Hub as grayscale PNGs.
 */
@TeleOp(name = "Camera Calibration Capture", group = "Utility")
public class CameraCalibrationCapture extends LinearOpMode {

    private static final Size RESOLUTION = new Size(640, 480);
    private static final int MAX_FRAMES = 40;
    private static final long CAPTURE_PERIOD_MS = 2000;
    private static final long EXPOSURE_MS = 10;
    private static final int GAIN = 50;
    private static final String OUT_DIR = "/sdcard/FIRST/calib";

    /** Reads every frame's brightness and writes a PNG when asked. */
    private static class FrameSaver implements VisionProcessor {
        final AtomicBoolean saveNext = new AtomicBoolean(false);
        volatile double brightness = -1;
        volatile int saved = 0;
        volatile String lastError = null;
        private final Mat gray = new Mat();

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGBA2GRAY);
            brightness = Core.mean(gray).val[0];
            if (saveNext.getAndSet(false)) {
                File dir = new File(OUT_DIR);
                dir.mkdirs();
                String path = new File(dir, String.format(Locale.US, "calib-%03d.png", saved)).getPath();
                if (Imgcodecs.imwrite(path, gray)) {
                    saved++;
                } else {
                    lastError = "imwrite failed";
                }
            }
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        }
    }

    @Override
    public void runOpMode() {
        FrameSaver saver = new FrameSaver();
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(RESOLUTION)
                .addProcessor(saver)
                .build();

        while (!isStarted() && !isStopRequested()) {
            if (portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                break;
            }
            sleep(20);
        }
        if (portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            try {
                ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
                exposure.setMode(ExposureControl.Mode.Manual);
                exposure.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
                portal.getCameraControl(GainControl.class).setGain(GAIN);
            } catch (Exception e) {
                telemetry.addData("Exposure error", e.getClass().getSimpleName());
            }
        }

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Point the camera at the checkerboard, then press START.");
            telemetry.addData("Camera", portal.getCameraState());
            telemetry.addData("Frame brightness", "%.1f / 255 (0 means a black image)", saver.brightness);
            telemetry.update();
            sleep(100);
        }

        long lastCapture = System.currentTimeMillis();
        while (opModeIsActive() && saver.saved < MAX_FRAMES) {
            long now = System.currentTimeMillis();
            if (now - lastCapture >= CAPTURE_PERIOD_MS) {
                saver.saveNext.set(true);
                lastCapture = now;
            }
            telemetry.addData("Camera", portal.getCameraState());
            telemetry.addData("Frame brightness", "%.1f / 255 (0 means a black image)", saver.brightness);
            telemetry.addData("Frames saved", "%d / %d", saver.saved, MAX_FRAMES);
            telemetry.addData("Next frame in", "%.1f s",
                    Math.max(0, CAPTURE_PERIOD_MS - (now - lastCapture)) / 1000.0);
            if (saver.lastError != null) {
                telemetry.addData("Error", saver.lastError);
            }
            telemetry.update();
            sleep(50);
        }

        telemetry.addData("Frames saved", "%d / %d", saver.saved, MAX_FRAMES);
        telemetry.addLine("Done. Stop the OpMode.");
        telemetry.update();
        while (opModeIsActive()) {
            sleep(50);
        }
        portal.close();
    }
}
