package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Measures the real MJPEG path: frames per second the camera delivers, JPEG decode cost, and
 * AprilTag detections per second when the decoded frames are fed to AprilTagProcessor by hand.
 * VisionPortal cannot decode this camera's MJPEG, so this uses the low-level camera API.
 * Results go to telemetry, logcat tag MjpegTest and /sdcard/FIRST/probe/mjpeg.csv.
 */
@TeleOp(name = "MJPEG FPS Test", group = "Calibration")
public class MjpegFpsTest extends LinearOpMode {
    private static final String TAG = "MjpegTest";
    private static final int JPEG = 256;
    private static final int[][] SIZES = {{1280, 800}, {800, 600}, {640, 480}};
    private static final long RUN_MS = 6000;
    private static final long EXPOSURE_MS = 2;
    private static final int GAIN = 50;
    private static final String DIR = "/sdcard/FIRST/probe";

    private final Object lock = new Object();
    private byte[] newestJpeg;
    private long newestSeq;
    private volatile long delivered;

    @Override
    public void runOpMode() {
        new File(DIR).mkdirs();
        telemetry.addLine("Ready. START runs each size for " + RUN_MS / 1000 + " s.");
        telemetry.update();
        waitForStart();

        StringBuilder csv = new StringBuilder(
                "width,height,camera_fps,processed_fps,decode_ms,detect_ms,detections_per_frame\n");
        for (int[] s : SIZES) {
            if (!opModeIsActive()) break;
            try {
                csv.append(runSize(s[0], s[1]));
            } catch (Throwable e) {
                Log.e(TAG, s[0] + "x" + s[1] + " failed", e);
                telemetry.addLine(s[0] + "x" + s[1] + " FAILED " + e);
                telemetry.update();
            }
        }
        try (FileWriter w = new FileWriter(new File(DIR, "mjpeg.csv"))) {
            w.write(csv.toString());
        } catch (IOException e) {
            Log.e(TAG, "csv write failed", e);
        }
        telemetry.addLine("DONE");
        telemetry.update();
        while (opModeIsActive()) sleep(50);
    }

    private String runSize(final int w, final int h) throws Exception {
        Camera camera = null;
        CameraCaptureSession session = null;
        AprilTagProcessor tags = null;
        Thread worker = null;
        final boolean[] run = {true};
        synchronized (lock) {
            newestJpeg = null;
            newestSeq = 0;
        }
        delivered = 0;
        try {
            camera = ClassFactory.getInstance().getCameraManager().requestPermissionAndOpenCamera(
                    new Deadline(5, TimeUnit.SECONDS), hardwareMap.get(WebcamName.class, "Webcam 1"),
                    Continuation.createTrivial(new Camera.StateCallback() {
                        public void onOpened(Camera c) {}
                        public void onOpenFailed(CameraName n, Camera.OpenFailure f) { Log.e(TAG, "open failed " + f); }
                        public void onClosed(Camera c) {}
                        public void onError(Camera c, Camera.Error e) { Log.e(TAG, "error " + e); }
                    }));
            if (camera == null) throw new IllegalStateException("camera did not open");

            // Rough intrinsics: detection only. Pose is meaningless until the lens is calibrated.
            double f = w * 0.6;
            tags = new AprilTagProcessor.Builder()
                    .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                    .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                    .setLensIntrinsics(f, f, w / 2.0, h / 2.0)
                    .setSuppressCalibrationWarnings(true)
                    .build();
            tags.init(w, h, null);
            final AprilTagProcessor processor = tags;

            CameraCaptureRequest request = camera.createCaptureRequest(JPEG, new Size(w, h), 120);
            session = camera.createCaptureSession(Continuation.createTrivial(
                    new CameraCaptureSession.StateCallback() {
                        public void onConfigured(CameraCaptureSession s) {}
                        public void onClosed(CameraCaptureSession s) {}
                    }));
            session.startCapture(request, Continuation.createTrivial(
                    new CameraCaptureSession.CaptureCallback() {
                        public void onNewFrame(CameraCaptureSession s, CameraCaptureRequest r, CameraFrame fr) {
                            try {
                                byte[] d = fr.getImageData();
                                if (d != null && d.length > 100) {
                                    synchronized (lock) {
                                        newestJpeg = d;
                                        newestSeq++;
                                    }
                                    delivered++;
                                }
                            } finally {
                                fr.releaseRef();
                            }
                        }
                    }),
                    Continuation.createTrivial(new CameraCaptureSession.StatusCallback() {
                        public void onCaptureSequenceCompleted(CameraCaptureSession s,
                                CameraCaptureSequenceId id, long ts) {}
                    }));

            try {
                ExposureControl exposure = camera.getControl(ExposureControl.class);
                if (exposure != null && exposure.isModeSupported(ExposureControl.Mode.Manual)) {
                    exposure.setMode(ExposureControl.Mode.Manual);
                    exposure.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
                }
                GainControl gain = camera.getControl(GainControl.class);
                if (gain != null) gain.setGain(GAIN);
            } catch (Throwable e) {
                Log.w(TAG, "exposure/gain not applied", e);
            }

            final long[] processed = {0};
            final double[] decodeNs = {0};
            final double[] detectNs = {0};
            final long[] detections = {0};
            final Mat[] lastGray = {null};
            worker = new Thread(new Runnable() {
                public void run() {
                    long seen = 0;
                    Mat gray = new Mat();
                    Mat rgba = new Mat();
                    while (run[0]) {
                        byte[] jpeg;
                        synchronized (lock) {
                            if (newestSeq == seen || newestJpeg == null) jpeg = null;
                            else { jpeg = newestJpeg; seen = newestSeq; }
                        }
                        if (jpeg == null) {
                            try { Thread.sleep(1); } catch (InterruptedException e) { return; }
                            continue;
                        }
                        long t0 = System.nanoTime();
                        MatOfByte buf = new MatOfByte(jpeg);
                        Mat dec = Imgcodecs.imdecode(buf, Imgcodecs.IMREAD_GRAYSCALE);
                        buf.release();
                        if (dec.empty()) { dec.release(); continue; }
                        dec.copyTo(gray);
                        dec.release();
                        Imgproc.cvtColor(gray, rgba, Imgproc.COLOR_GRAY2RGBA);
                        long t1 = System.nanoTime();
                        Object out = processor.processFrame(rgba, t0);
                        long t2 = System.nanoTime();
                        decodeNs[0] += t1 - t0;
                        detectNs[0] += t2 - t1;
                        processed[0]++;
                        List<AprilTagDetection> d = processor.getDetections();
                        detections[0] += d == null ? 0 : d.size();
                        synchronized (lastGray) {
                            if (lastGray[0] == null) lastGray[0] = gray.clone();
                        }
                    }
                    gray.release();
                    rgba.release();
                }
            }, "MjpegWorker");
            worker.start();

            long start = System.currentTimeMillis();
            while (opModeIsActive() && System.currentTimeMillis() - start < RUN_MS) {
                double secs = (System.currentTimeMillis() - start) / 1000.0;
                telemetry.addData(w + "x" + h, "camera %.1f fps, processed %.1f fps",
                        delivered / Math.max(secs, 0.001), processed[0] / Math.max(secs, 0.001));
                telemetry.update();
                sleep(200);
            }
            double secs = (System.currentTimeMillis() - start) / 1000.0;
            run[0] = false;
            worker.join(2000);
            long p = Math.max(processed[0], 1);
            String row = String.format("%d,%d,%.1f,%.1f,%.1f,%.1f,%.2f\n", w, h,
                    delivered / secs, processed[0] / secs,
                    decodeNs[0] / p / 1e6, detectNs[0] / p / 1e6, detections[0] / (double) p);
            Log.i(TAG, row.trim());
            telemetry.addLine(row.trim());
            telemetry.update();
            synchronized (lastGray) {
                if (lastGray[0] != null) {
                    Imgcodecs.imwrite(DIR + "/mjpeg_" + w + "x" + h + ".png", lastGray[0]);
                    lastGray[0].release();
                }
            }
            return row;
        } finally {
            run[0] = false;
            if (worker != null) {
                try { worker.join(2000); } catch (InterruptedException ignored) {}
            }
            try { if (session != null) session.stopCapture(); } catch (Throwable ignored) {}
            try { if (session != null) session.close(); } catch (Throwable ignored) {}
            try { if (camera != null) camera.close(); } catch (Throwable ignored) {}
            sleep(500);
        }
    }
}
