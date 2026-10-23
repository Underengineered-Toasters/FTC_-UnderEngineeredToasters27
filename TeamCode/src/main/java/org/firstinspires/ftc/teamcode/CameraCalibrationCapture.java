package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

/**
 * Saves checkerboard frames for lens calibration of the Arducam OV9281.
 * Frames use the same format and resolution as AprilTagVision, because the calibration is only
 * valid at the resolution it was captured at. Press START, then move the camera in front of a
 * checkerboard. Frames are written to /sdcard/FIRST/data on the Control Hub.
 */
@TeleOp(name = "Camera Calibration Capture", group = "Utility")
public class CameraCalibrationCapture extends LinearOpMode {

    private static final Size RESOLUTION = new Size(640, 480);
    private static final int MAX_FRAMES = 40;
    private static final long CAPTURE_PERIOD_MS = 2000;

    @Override
    public void runOpMode() {
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(RESOLUTION)
                .build();

        telemetry.addLine("Press START, then slowly move the camera in front of the checkerboard.");
        telemetry.update();
        waitForStart();

        int frames = 0;
        long lastCapture = System.currentTimeMillis();
        while (opModeIsActive() && frames < MAX_FRAMES) {
            long now = System.currentTimeMillis();
            if (now - lastCapture >= CAPTURE_PERIOD_MS
                    && portal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                portal.saveNextFrameRaw(String.format(Locale.US, "CalibFrame-%03d", frames++));
                lastCapture = now;
            }
            telemetry.addData("Camera", portal.getCameraState());
            telemetry.addData("Frames saved", "%d / %d", frames, MAX_FRAMES);
            telemetry.addData("Next frame in", "%.1f s",
                    Math.max(0, CAPTURE_PERIOD_MS - (now - lastCapture)) / 1000.0);
            telemetry.update();
            sleep(50);
        }

        telemetry.addData("Frames saved", "%d / %d", frames, MAX_FRAMES);
        telemetry.addLine("Done. Stop the OpMode.");
        telemetry.update();
        while (opModeIsActive()) {
            sleep(50);
        }
        portal.close();
    }
}
