package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Log;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * Captures chessboard frames for offline camera calibration.
 *
 * Press A to save the newest frame to /sdcard/FIRST/calib/ and report whether the board was
 * detected. Press B to delete the frame saved most recently in this session. The frames are
 * solved on a PC with TeamCode/tools/calibrate_camera.py.
 */
@TeleOp(name = "Mono Calibration Capture", group = "Calibration")
public class MonoCalibrationCapture extends LinearOpMode {
    private static final String TAG = "MonoCalib";

    private static final VisionPortal.StreamFormat STREAM_FORMAT = VisionPortal.StreamFormat.YUY2;
    private static final int WIDTH = 640;
    private static final int HEIGHT = 480;
    private static final long EXPOSURE_MS = 10;
    private static final int GAIN = 100;
    private static final int BOARD_COLS = 9;
    private static final int BOARD_ROWS = 6;

    private static final String CALIB_DIR = "/sdcard/FIRST/calib";
    private static final String FILE_PREFIX = "calib_" + WIDTH + "x" + HEIGHT + "_";

    private final Object frameLock = new Object();
    private Mat latestGray;
    private long frameCount;

    private ExposureControl exposureControl;
    private GainControl gainControl;
    private long appliedExposureMs = -1;
    private int appliedGain = -1;
    private String controlStatus = "waiting for stream";
    private boolean controlsApplied;

    private int savedCount;
    private String lastSavedPath;
    private String saveStatus = "none";

    private final VisionProcessor processor = new VisionProcessor() {
        @Override
        public void init(int width, int height, CameraCalibration cameraCalibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            synchronized (frameLock) {
                if (latestGray == null || latestGray.rows() != frame.rows()
                        || latestGray.cols() != frame.cols()) {
                    if (latestGray != null) latestGray.release();
                    latestGray = new Mat();
                }
                Imgproc.cvtColor(frame, latestGray, Imgproc.COLOR_RGBA2GRAY);
                frameCount++;
            }
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
        }
    };

    @Override
    public void runOpMode() {
        File dir = new File(CALIB_DIR);
        if (!dir.isDirectory() && !dir.mkdirs()) {
            Log.e(TAG, "could not create " + CALIB_DIR);
        }
        savedCount = countSavedFrames(dir);

        VisionPortal portal = null;
        String buildError = null;
        long nextControlAttempt = 0;
        boolean prevA = false;
        boolean prevB = false;
        try {
            try {
                portal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .setStreamFormat(STREAM_FORMAT)
                        .setCameraResolution(new Size(WIDTH, HEIGHT))
                        .enableLiveView(true)
                        .addProcessor(processor)
                        .build();
            } catch (Throwable t) {
                buildError = String.valueOf(t);
                Log.e(TAG, "could not open camera", t);
            }

            waitForStart();

            if (portal == null) {
                while (opModeIsActive()) {
                    telemetry.addData("ERROR", buildError);
                    telemetry.update();
                    sleep(100);
                }
                return;
            }

            while (opModeIsActive()) {
                if (!controlsApplied && portal.getCameraState() == VisionPortal.CameraState.STREAMING
                        && System.currentTimeMillis() >= nextControlAttempt) {
                    controlsApplied = applyControls(portal);
                    if (!controlsApplied) nextControlAttempt = System.currentTimeMillis() + 1000;
                }

                boolean a = gamepad1.a;
                boolean b = gamepad1.b;
                if (a && !prevA) saveFrame();
                if (b && !prevB) deleteLastSaved();
                prevA = a;
                prevB = b;

                showTelemetry(portal);
                telemetry.update();
                sleep(20);
            }
        } finally {
            if (portal != null) portal.close();
            synchronized (frameLock) {
                if (latestGray != null) {
                    latestGray.release();
                    latestGray = null;
                }
            }
        }
    }

    private boolean applyControls(VisionPortal portal) {
        boolean applied = false;
        try {
            exposureControl = portal.getCameraControl(ExposureControl.class);
        } catch (Throwable t) {
            exposureControl = null;
        }
        try {
            gainControl = portal.getCameraControl(GainControl.class);
        } catch (Throwable t) {
            gainControl = null;
        }

        if (exposureControl != null && exposureControl.isExposureSupported()) {
            try {
                if (exposureControl.isModeSupported(ExposureControl.Mode.Manual)) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                }
                long min = exposureControl.getMinExposure(TimeUnit.MILLISECONDS);
                long max = exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
                long target = EXPOSURE_MS;
                if (min > 0) target = Math.max(target, min);
                if (max > 0) target = Math.min(target, max);
                exposureControl.setExposure(target, TimeUnit.MILLISECONDS);
                appliedExposureMs = target;
                applied = true;
            } catch (Throwable t) {
                Log.e(TAG, "exposure setup failed", t);
            }
        }

        if (gainControl != null) {
            try {
                int min = gainControl.getMinGain();
                int max = gainControl.getMaxGain();
                int target = Math.max(GAIN, min);
                if (max > 0) target = Math.min(target, max);
                gainControl.setGain(target);
                appliedGain = target;
                applied = true;
            } catch (Throwable t) {
                Log.e(TAG, "gain setup failed", t);
            }
        }

        controlStatus = applied
                ? String.format(Locale.US, "exp %d ms, gain %d", appliedExposureMs, appliedGain)
                : "no controls";
        return applied;
    }

    private void saveFrame() {
        Mat snapshot;
        synchronized (frameLock) {
            if (latestGray == null) {
                saveStatus = "no frame yet";
                return;
            }
            snapshot = latestGray.clone();
        }

        MatOfPoint2f corners = new MatOfPoint2f();
        try {
            int index = nextFreeIndex();
            if (index < 0) {
                saveStatus = "no free index";
                return;
            }
            String path = String.format(Locale.US, "%s/%s%02d.png", CALIB_DIR, FILE_PREFIX, index);
            boolean written = Imgcodecs.imwrite(path, snapshot);
            boolean found = Calib3d.findChessboardCorners(
                    snapshot,
                    new org.opencv.core.Size(BOARD_COLS, BOARD_ROWS),
                    corners,
                    Calib3d.CALIB_CB_ADAPTIVE_THRESH | Calib3d.CALIB_CB_NORMALIZE_IMAGE);

            if (written) {
                savedCount++;
                lastSavedPath = path;
            }
            saveStatus = String.format(Locale.US, "%s  %s  (%d saved)",
                    found ? "FOUND" : "NOT FOUND",
                    written ? "saved " + new File(path).getName() : "write failed",
                    savedCount);
            Log.i(TAG, saveStatus);
        } catch (Throwable t) {
            saveStatus = "save failed: " + t;
            Log.e(TAG, "save failed", t);
        } finally {
            snapshot.release();
            corners.release();
        }
    }

    private void deleteLastSaved() {
        File file = lastSavedPath != null ? new File(lastSavedPath) : newestSavedFile();
        if (file == null) {
            saveStatus = "nothing to delete";
            return;
        }
        boolean deleted = file.delete();
        if (deleted) savedCount = Math.max(0, savedCount - 1);
        saveStatus = deleted
                ? String.format(Locale.US, "deleted %s  (%d saved)", file.getName(), savedCount)
                : "could not delete " + file.getName();
        lastSavedPath = null;
    }

    private File newestSavedFile() {
        String[] names = new File(CALIB_DIR).list();
        if (names == null) return null;
        int highest = -1;
        for (String name : names) {
            if (!name.startsWith(FILE_PREFIX) || !name.endsWith(".png")) continue;
            String digits = name.substring(FILE_PREFIX.length(), name.length() - 4);
            try {
                highest = Math.max(highest, Integer.parseInt(digits));
            } catch (NumberFormatException ignored) {
            }
        }
        if (highest < 0) return null;
        return new File(String.format(Locale.US, "%s/%s%02d.png", CALIB_DIR, FILE_PREFIX, highest));
    }

    private void showTelemetry(VisionPortal portal) {
        double mean = -1;
        long frames;
        synchronized (frameLock) {
            frames = frameCount;
            if (latestGray != null) mean = Core.mean(latestGray).val[0];
        }

        long exposure = appliedExposureMs;
        if (exposureControl != null) {
            try {
                if (exposureControl.isExposureSupported()) {
                    long live = exposureControl.getExposure(TimeUnit.MILLISECONDS);
                    if (live > 0) exposure = live;
                }
            } catch (Throwable ignored) {
            }
        }
        int gain = appliedGain;
        if (gainControl != null) {
            try {
                gain = gainControl.getGain();
            } catch (Throwable ignored) {
            }
        }

        telemetry.addData("Camera", portal.getCameraState());
        telemetry.addData("FPS", String.format(Locale.US, "%.1f", portal.getFps()));
        telemetry.addData("Frames", frames);
        telemetry.addData("Mean brightness", String.format(Locale.US, "%.1f", mean));
        telemetry.addData("Exposure (ms)", exposure < 0 ? "?" : String.valueOf(exposure));
        telemetry.addData("Gain", gain < 0 ? "?" : String.valueOf(gain));
        telemetry.addData("Controls", controlStatus);
        telemetry.addData("Board", BOARD_COLS + " x " + BOARD_ROWS + " inner corners");
        telemetry.addData("Last frame", saveStatus);
        telemetry.addData("Saved frames", savedCount);
        telemetry.addData("A", "save frame      B: delete last frame");
    }

    private int nextFreeIndex() {
        for (int i = 0; i < 100; i++) {
            if (!new File(String.format(Locale.US, "%s/%s%02d.png", CALIB_DIR, FILE_PREFIX, i)).exists()) {
                return i;
            }
        }
        return -1;
    }

    private int countSavedFrames(File dir) {
        String[] names = dir.list();
        if (names == null) return 0;
        int count = 0;
        for (String name : names) {
            if (name.startsWith(FILE_PREFIX) && name.endsWith(".png")) count++;
        }
        return count;
    }
}
