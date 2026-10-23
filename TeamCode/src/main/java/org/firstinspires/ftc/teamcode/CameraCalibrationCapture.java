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
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.TimeUnit;

/**
 * Saves checkerboard frames for lens calibration of the Arducam OV9281.
 * Frames use the same resolution and format as AprilTagVision, because a calibration is only
 * valid at the resolution it was captured at.
 *
 * The OpMode finds the board itself and only saves a frame when the board is fully visible and
 * its pose differs from every saved frame. Focal length cannot be solved from frames that all
 * show the board at the same angle, so it also counts tilted poses and shows which parts of the
 * image the board has not reached. Frames go to /sdcard/FIRST/calib as grayscale PNGs.
 * The board is 9x6 inner corners, which is what TeamCode/tools/calibrate_camera.py makes.
 */
@TeleOp(name = "Camera Calibration Capture", group = "Utility")
public class CameraCalibrationCapture extends LinearOpMode {

    private static final Size RESOLUTION = new Size(640, 480);
    private static final int COLS = 9;
    private static final int ROWS = 6;
    private static final int MAX_FRAMES = 40;
    private static final int ENOUGH_FRAMES = 25;
    private static final int ENOUGH_TILTED = 8;
    /** A frame is new when its corners are on average this many pixels from every saved frame. */
    private static final double NEW_POSE_PX = 40;
    /** A frame is tilted when opposite board edges differ in length by this fraction. */
    private static final double TILT_RATIO = 0.12;
    private static final long EXPOSURE_MS = 10;
    private static final int GAIN = 50;
    private static final String OUT_DIR = "/sdcard/FIRST/calib";

    /** Finds the board in every few frames and saves the ones that show a new pose. */
    private static class BoardSaver implements VisionProcessor {
        volatile boolean saving = false;
        volatile boolean boardFound = false;
        volatile double brightness = -1;
        volatile int saved = 0;
        volatile int tilted = 0;
        volatile String error = null;
        final boolean[] cells = new boolean[9];

        private final Mat gray = new Mat();
        private final List<Point[]> savedCorners = new ArrayList<>();
        private long lastTry = 0;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGBA2GRAY);
            brightness = Core.mean(gray).val[0];

            long now = System.currentTimeMillis();
            if (now - lastTry < 300) {
                return null;
            }
            lastTry = now;

            MatOfPoint2f corners = new MatOfPoint2f();
            // FAST_CHECK returns quickly when no board is in view. Without it a miss took about
            // 780 ms on the Control Hub, which dropped the live view to 1.3 fps.
            boolean found = Calib3d.findChessboardCorners(gray, new org.opencv.core.Size(COLS, ROWS), corners,
                    Calib3d.CALIB_CB_ADAPTIVE_THRESH | Calib3d.CALIB_CB_NORMALIZE_IMAGE
                            | Calib3d.CALIB_CB_FAST_CHECK);
            boardFound = found;
            if (!found || !saving || saved >= MAX_FRAMES) {
                return null;
            }

            Point[] pts = corners.toArray();
            if (!isNewPose(pts)) {
                return null;
            }
            File dir = new File(OUT_DIR);
            dir.mkdirs();
            String path = new File(dir, String.format(Locale.US, "calib-%03d.png", saved)).getPath();
            if (!Imgcodecs.imwrite(path, gray)) {
                error = "could not write " + path;
                return null;
            }
            savedCorners.add(pts);
            saved++;
            if (isTilted(pts)) {
                tilted++;
            }
            double cx = 0;
            double cy = 0;
            for (Point p : pts) {
                cx += p.x;
                cy += p.y;
            }
            cx /= pts.length;
            cy /= pts.length;
            int col = (int) Math.min(2, Math.max(0, cx / (RESOLUTION.getWidth() / 3.0)));
            int row = (int) Math.min(2, Math.max(0, cy / (RESOLUTION.getHeight() / 3.0)));
            cells[row * 3 + col] = true;
            return null;
        }

        private boolean isNewPose(Point[] pts) {
            for (Point[] old : savedCorners) {
                double same = 0;
                double flipped = 0;
                for (int i = 0; i < pts.length; i++) {
                    same += dist(pts[i], old[i]);
                    flipped += dist(pts[i], old[pts.length - 1 - i]);
                }
                if (Math.min(same, flipped) / pts.length < NEW_POSE_PX) {
                    return false;
                }
            }
            return true;
        }

        private static boolean isTilted(Point[] p) {
            double top = dist(p[COLS - 1], p[0]);
            double bottom = dist(p[ROWS * COLS - 1], p[(ROWS - 1) * COLS]);
            double left = dist(p[(ROWS - 1) * COLS], p[0]);
            double right = dist(p[ROWS * COLS - 1], p[COLS - 1]);
            return Math.abs(Math.log(top / bottom)) > TILT_RATIO
                    || Math.abs(Math.log(left / right)) > TILT_RATIO;
        }

        private static double dist(Point a, Point b) {
            return Math.hypot(a.x - b.x, a.y - b.y);
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        }
    }

    private void showStatus(VisionPortal portal, BoardSaver saver, String instruction) {
        telemetry.addLine(instruction);
        telemetry.addData("Camera", "%s, brightness %.0f", portal.getCameraState(), saver.brightness);
        telemetry.addData("Board", saver.boardFound ? "FOUND" : "not found (show the whole board)");
        telemetry.addData("Poses saved", "%d (need %d)", saver.saved, ENOUGH_FRAMES);
        telemetry.addData("Tilted poses", "%d (need %d, tilt the camera 30 to 45 degrees)",
                saver.tilted, ENOUGH_TILTED);
        StringBuilder grid = new StringBuilder("Board centre reached:");
        for (int r = 0; r < 3; r++) {
            grid.append("\n  ");
            for (int c = 0; c < 3; c++) {
                grid.append(saver.cells[r * 3 + c] ? "[X]" : "[ ]");
            }
        }
        telemetry.addLine(grid.toString());
        if (saver.error != null) {
            telemetry.addData("Error", saver.error);
        }
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        File[] old = new File(OUT_DIR).listFiles();
        if (old != null) {
            for (File f : old) {
                f.delete();
            }
        }

        BoardSaver saver = new BoardSaver();
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setCameraResolution(RESOLUTION)
                .addProcessor(saver)
                .build();

        while (!isStarted() && !isStopRequested()
                && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
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
            showStatus(portal, saver, "Aim at the board until it says FOUND, then press START.");
            sleep(100);
        }

        saver.saving = true;
        while (opModeIsActive()) {
            boolean done = saver.saved >= ENOUGH_FRAMES && saver.tilted >= ENOUGH_TILTED;
            showStatus(portal, saver, done
                    ? "DONE. Stop the OpMode."
                    : "Move closer and farther, tilt, and rotate the camera. Fill every [ ].");
            sleep(100);
        }
        portal.close();
    }
}
