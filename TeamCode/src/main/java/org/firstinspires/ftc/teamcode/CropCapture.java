package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.util.Log;
import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.TimeUnit;

/** Throwaway: saves one frame at 640x480 YUY2 and one at 1280x800 MJPEG for the crop check. */
@TeleOp(name = "Crop Capture", group = "Calibration")
public class CropCapture extends LinearOpMode {
    private static final String DIR = "/sdcard/FIRST/probe";
    private volatile Mat yuy2;

    @Override
    public void runOpMode() {
        waitForStart();
        new java.io.File(DIR).mkdirs();

        // Phase 1: the full view, which VisionPortal cannot decode from MJPEG.
        Camera camera = null;
        CameraCaptureSession session = null;
        final byte[][] jpeg = new byte[1][];
        final int[] frames = new int[1];
        try {
            CameraManager manager = ClassFactory.getInstance().getCameraManager();
            camera = manager.requestPermissionAndOpenCamera(
                    new Deadline(5, TimeUnit.SECONDS), hardwareMap.get(WebcamName.class, "Webcam 1"),
                    Continuation.createTrivial(new Camera.StateCallback() {
                        public void onOpened(Camera c) {}
                        public void onOpenFailed(org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName n, Camera.OpenFailure f) { Log.e("CropCap", "open failed " + f); }
                        public void onClosed(Camera c) {}
                        public void onError(Camera c, Camera.Error e) { Log.e("CropCap", "error " + e); }
                    }));
            CameraCaptureRequest request = camera.createCaptureRequest(
                    256, new org.firstinspires.ftc.robotcore.external.android.util.Size(1280, 800), 30);
            Log.i("CropCap", "request format=" + request.getAndroidFormat()
                    + " size=" + request.getSize() + " fps=" + request.getFramesPerSecond());
            session = camera.createCaptureSession(Continuation.createTrivial(
                    new CameraCaptureSession.StateCallback() {
                        public void onConfigured(CameraCaptureSession s) { Log.i("CropCap", "session configured"); }
                        public void onClosed(CameraCaptureSession s) {}
                    }));
            session.startCapture(request, Continuation.createTrivial(
                    new CameraCaptureSession.CaptureCallback() {
                        public void onNewFrame(CameraCaptureSession s, CameraCaptureRequest r, CameraFrame f) {
                            try {
                                byte[] d = f.getImageData();
                                if (frames[0] < 5) {
                                    frames[0]++;
                                    Log.i("CropCap", "fmt=" + f.getUvcFrameFormat() + " size=" + f.getSize()
                                            + " imageSize=" + f.getImageSize() + " buf=" + (d == null ? -1 : d.length)
                                            + " head=" + (d == null || d.length < 4 ? "?" :
                                            String.format("%02X%02X%02X%02X", d[0], d[1], d[2], d[3])));
                                }
                                // Keep the newest frame that starts like a JPEG.
                                if (d != null && d.length > 10000 && (d[0] & 0xFF) == 0xFF && (d[1] & 0xFF) == 0xD8) {
                                    jpeg[0] = d;
                                }
                            } finally {
                                f.releaseRef();
                            }
                        }
                    }),
                    Continuation.createTrivial(new CameraCaptureSession.StatusCallback() {
                        public void onCaptureSequenceCompleted(CameraCaptureSession s,
                                org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId id,
                                long ts) {}
                    }));
            sleep(3000);
            byte[] data = jpeg[0];
            Bitmap bmp = data == null ? null : BitmapFactory.decodeByteArray(data, 0, data.length);
            if (bmp != null) {
                Mat big = new Mat();
                Utils.bitmapToMat(bmp, big);
                Mat gray = new Mat();
                Imgproc.cvtColor(big, gray, Imgproc.COLOR_RGBA2GRAY);
                Imgcodecs.imwrite(DIR + "/big_1280x800.png", gray);
                Log.i("CropCap", "saved 1280x800 " + gray.cols() + "x" + gray.rows());
                gray.release();
                big.release();
            } else {
                Log.e("CropCap", "no 1280x800 frame");
            }
        } catch (Throwable e) {
            Log.e("CropCap", "capture failed", e);
        } finally {
            try { if (session != null) session.stopCapture(); } catch (Throwable ignored) {}
            try { if (session != null) session.close(); } catch (Throwable ignored) {}
            try { if (camera != null) camera.close(); } catch (Throwable ignored) {}
        }
        // Phase 2: the mode the calibration is written for, through VisionPortal.
        VisionPortal portal = null;
        try {
            portal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                    .setCameraResolution(new Size(640, 480))
                    .addProcessor(new VisionProcessor() {
                        public void init(int w, int h, CameraCalibration c) {}
                        public Object processFrame(Mat f, long t) {
                            if (yuy2 == null) {
                                yuy2 = new Mat();
                                Imgproc.cvtColor(f, yuy2, Imgproc.COLOR_RGBA2GRAY);
                            }
                            return null;
                        }
                        public void onDrawFrame(Canvas c, int a, int b, float s, float d, Object u) {}
                    })
                    .build();
            long end = System.currentTimeMillis() + 10000;
            while (opModeIsActive() && yuy2 == null && System.currentTimeMillis() < end) sleep(50);
            if (yuy2 != null) {
                Imgcodecs.imwrite(DIR + "/small_640x480.png", yuy2);
                Log.i("CropCap", "saved 640x480 " + yuy2.cols() + "x" + yuy2.rows());
            } else {
                Log.e("CropCap", "no 640x480 frame");
            }
        } catch (Throwable e) {
            Log.e("CropCap", "portal failed", e);
        } finally {
            if (portal != null) portal.close();
        }
        sleep(4000);

        telemetry.addLine("DONE, check logcat");
        telemetry.update();
    }
}