package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.util.Log;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Deque;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.concurrent.TimeUnit;

/**
 * Unattended probe for the Arducam OV9281 monochrome camera.
 *
 * Dumps UVC capabilities, tries every YUY2 size up to 1280x800 plus MJPEG 1280x800 through
 * VisionPortal, saves one auto-exposure frame per working mode (for field-of-view comparison),
 * then sweeps exposure/gain manually in the best working mode.
 *
 * Outputs: /sdcard/FIRST/probe/{modes.txt,controls.txt,summary.csv,auto_*.png,sweep_*.png}
 */
@TeleOp(name = "Mono Camera Probe", group = "Calibration")
public class MonoCameraProbe extends LinearOpMode {

    private static final String TAG = "MonoProbe";
    private static final String DIR = "/sdcard/FIRST/probe";
    private static final String CAMERA_NAME = "Webcam 1";

    private static final long STREAM_TIMEOUT_MS = 5000;
    private static final long FRAME_TIMEOUT_MS = 2500;
    private static final long PORTAL_CLOSE_SETTLE_MS = 700;
    private static final long SENSOR_SETTLE_MS = 600;
    private static final long FPS_WINDOW_MS = 2000;

    private static final long[] EXPOSURES_MS = {1, 2, 5, 10, 20, 33};

    private static final int FORMAT_YUY2 = 20;

    private final Deque<String> statusLines = new ArrayDeque<>();

    // ---------------------------------------------------------------------------------------
    // OpMode entry point
    // ---------------------------------------------------------------------------------------

    @Override
    public void runOpMode() {
        telemetry.addLine("Mono Camera Probe");
        telemetry.addLine("output: " + DIR);
        telemetry.addLine("waiting for START");
        telemetry.update();

        waitForStart();
        new File(DIR).mkdirs();
        writeSummaryHeader();

        WebcamName webcam = null;
        try {
            webcam = hardwareMap.get(WebcamName.class, CAMERA_NAME);
        } catch (Throwable e) {
            Log.e(TAG, "webcam '" + CAMERA_NAME + "' not in hardware map", e);
            status("camera '" + CAMERA_NAME + "' missing from hardware map");
        }

        CameraCharacteristics characteristics = null;
        if (webcam != null) {
            characteristics = dumpCapabilities(webcam);
        }

        List<ModeCandidate> candidates = buildCandidates(characteristics);
        List<WorkingMode> working = new ArrayList<>();
        if (webcam != null) {
            for (ModeCandidate candidate : candidates) {
                if (!opModeIsActive()) break;
                WorkingMode mode = testMode(webcam, candidate);
                if (mode != null) working.add(mode);
            }
        }

        WorkingMode best = pickBest(working);
        if (best == null) {
            status("no working VisionPortal mode");
            status("DONE (nothing to sweep)");
        } else {
            status("best mode: " + best.label() + " fps=" + fmt1(best.fps));
            String bestSweep = runSweep(webcam, best);
            if (bestSweep == null) {
                status("sweep: no usable row");
            } else {
                status("best sweep: " + bestSweep);
            }
            status("DONE");
        }

        while (opModeIsActive()) {
            sleep(50);
        }
    }

    // ---------------------------------------------------------------------------------------
    // Phase 1: capabilities
    // ---------------------------------------------------------------------------------------

    private CameraCharacteristics dumpCapabilities(WebcamName webcam) {
        CameraManager manager = null;
        try {
            manager = ClassFactory.getInstance().getCameraManager();
        } catch (Throwable e) {
            Log.e(TAG, "CameraManager unavailable", e);
        }

        final boolean[] opened = {false};
        Camera camera = null;
        CameraCharacteristics characteristics = null;
        try {
            if (manager != null) {
                camera = manager.requestPermissionAndOpenCamera(
                        new Deadline(5, TimeUnit.SECONDS),
                        webcam,
                        Continuation.createTrivial(new Camera.StateCallback() {
                            @Override
                            public void onOpened(Camera c) {
                                opened[0] = true;
                            }

                            @Override
                            public void onOpenFailed(CameraName name, Camera.OpenFailure failure) {
                                Log.e(TAG, "camera open failed: " + failure);
                            }

                            @Override
                            public void onClosed(Camera c) {
                            }

                            @Override
                            public void onError(Camera c, Camera.Error error) {
                                Log.e(TAG, "camera error: " + error);
                            }
                        }));
                ElapsedTime openTimer = new ElapsedTime();
                while (!opened[0] && openTimer.milliseconds() < 2000 && opModeIsActive()) {
                    sleep(50);
                }
            }
            characteristics = webcam.getCameraCharacteristics();
        } catch (Throwable e) {
            Log.e(TAG, "capability query failed", e);
            status("capabilities: " + e.getClass().getSimpleName());
        } finally {
            try {
                if (camera != null) camera.close();
            } catch (Throwable ignored) {
            }
        }

        // Let the UVC device finish closing before a VisionPortal opens it again.
        sleep(PORTAL_CLOSE_SETTLE_MS);

        if (characteristics == null) {
            status("capabilities: none reported");
            return null;
        }

        List<String> lines = new ArrayList<>();
        lines.add("camera: " + CAMERA_NAME);
        lines.add("androidFormats:");
        int[] formats = null;
        try {
            formats = characteristics.getAndroidFormats();
        } catch (Throwable e) {
            lines.add("  query failed: " + e);
        }
        if (formats != null) {
            for (int format : formats) {
                lines.add("  " + format + " (" + formatName(format) + ")");
                try {
                    org.firstinspires.ftc.robotcore.external.android.util.Size[] sizes =
                            characteristics.getSizes(format);
                    if (sizes == null) {
                        lines.add("    sizes: none");
                        continue;
                    }
                    for (org.firstinspires.ftc.robotcore.external.android.util.Size size : sizes) {
                        if (size == null) continue;
                        int w = size.getWidth();
                        int h = size.getHeight();
                        int maxFps = -1;
                        try {
                            maxFps = characteristics.getMaxFramesPerSecond(format, size);
                        } catch (Throwable e) {
                            lines.add("    " + w + "x" + h + " maxFps=error(" + e + ")");
                            continue;
                        }
                        lines.add("    " + w + "x" + h + " maxFps=" + maxFps);
                    }
                } catch (Throwable e) {
                    lines.add("    sizes: query failed " + e);
                }
            }
        }

        lines.add("allCameraModes:");
        try {
            List<CameraCharacteristics.CameraMode> modes = characteristics.getAllCameraModes();
            if (modes == null) {
                lines.add("  none");
            } else {
                for (CameraCharacteristics.CameraMode mode : modes) {
                    lines.add("  " + mode);
                }
            }
        } catch (Throwable e) {
            lines.add("  query failed: " + e);
        }

        for (String line : lines) {
            Log.i(TAG, line);
        }
        appendLines("modes.txt", lines);
        status("capabilities dumped (" + (formats == null ? 0 : formats.length) + " formats)");
        return characteristics;
    }

    private static String formatName(int format) {
        switch (format) {
            case 20:
                return "YUY2";
            case 256:
                return "JPEG/MJPEG";
            case 17:
                return "NV21";
            case 16:
                return "NV16";
            case 35:
                return "YUV_420_888";
            case 842094169:
                return "YV12";
            case 32:
                return "RAW_SENSOR";
            case 34:
                return "PRIVATE";
            case 1:
                return "RGBA_8888";
            case 2:
                return "RGBX_8888";
            case 3:
                return "RGB_888";
            case 4:
                return "RGB_565";
            default:
                return "unknown";
        }
    }

    // ---------------------------------------------------------------------------------------
    // Phase 2: mode sweep
    // ---------------------------------------------------------------------------------------

    private List<ModeCandidate> buildCandidates(CameraCharacteristics characteristics) {
        List<int[]> yuy2Sizes = new ArrayList<>();
        Set<String> seen = new LinkedHashSet<>();
        if (characteristics != null) {
            try {
                org.firstinspires.ftc.robotcore.external.android.util.Size[] sizes =
                        characteristics.getSizes(FORMAT_YUY2);
                if (sizes != null) {
                    for (org.firstinspires.ftc.robotcore.external.android.util.Size size : sizes) {
                        if (size == null) continue;
                        int w = size.getWidth();
                        int h = size.getHeight();
                        if (w > 0 && h > 0 && w <= 1280 && h <= 800 && seen.add(w + "x" + h)) {
                            yuy2Sizes.add(new int[]{w, h});
                        }
                    }
                }
            } catch (Throwable e) {
                Log.w(TAG, "YUY2 size query failed", e);
            }
        }
        if (yuy2Sizes.isEmpty()) {
            yuy2Sizes.add(new int[]{1280, 800});
            yuy2Sizes.add(new int[]{640, 480});
        }
        Collections.sort(yuy2Sizes, new Comparator<int[]>() {
            @Override
            public int compare(int[] a, int[] b) {
                return Integer.compare(b[0] * b[1], a[0] * a[1]);
            }
        });

        List<ModeCandidate> candidates = new ArrayList<>();
        for (int[] size : yuy2Sizes) {
            candidates.add(new ModeCandidate(VisionPortal.StreamFormat.YUY2, size[0], size[1]));
        }
        candidates.add(new ModeCandidate(VisionPortal.StreamFormat.MJPEG, 1280, 800));
        return candidates;
    }

    private WorkingMode testMode(WebcamName webcam, ModeCandidate candidate) {
        String label = candidate.label();
        status("testing " + label);
        VisionPortal portal = null;
        LatestGray processor = new LatestGray();
        try {
            portal = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .setStreamFormat(candidate.format)
                    .setCameraResolution(new Size(candidate.width, candidate.height))
                    .enableLiveView(false)
                    .addProcessor(processor)
                    .build();

            if (!waitForStreaming(portal, STREAM_TIMEOUT_MS)) {
                Log.w(TAG, label + ": no stream, state=" + stateOf(portal));
                status(label + ": FAILED (state " + stateOf(portal) + ")");
                return null;
            }

            Mat first = waitForNewFrame(processor, FRAME_TIMEOUT_MS);
            if (first == null) {
                Log.w(TAG, label + ": streaming but no frames");
                status(label + ": FAILED (no frames)");
                return null;
            }
            int actualW = first.cols();
            int actualH = first.rows();
            first.release();
            if (actualW != candidate.width || actualH != candidate.height) {
                Log.w(TAG, label + ": requested " + candidate.width + "x" + candidate.height
                        + " but frames arrive as " + actualW + "x" + actualH);
            }

            sleep(FPS_WINDOW_MS);
            float fps = portal.getFps();
            recordControls(portal, label + " fps=" + fmt1(fps));

            Mat gray = waitForNewFrame(processor, FRAME_TIMEOUT_MS);
            if (gray == null) {
                status(label + ": no frame for auto capture");
                return null;
            }
            int w = gray.cols();
            int h = gray.rows();
            String file = "auto_" + candidate.formatName() + "_" + w + "x" + h + ".png";
            boolean saved = Imgcodecs.imwrite(DIR + "/" + file, gray);
            if (!saved) {
                Log.e(TAG, label + ": failed to write " + file);
            }
            Scalar mean = Core.mean(gray);
            Core.MinMaxLocResult minMax = Core.minMaxLoc(gray);
            double meanValue = mean.val[0];
            appendSummary(candidate.formatName(), w, h, "auto", "auto",
                    meanValue, minMax.minVal, minMax.maxVal, fps);
            status(String.format(Locale.US, "%s: mean=%.1f min=%.0f max=%.0f fps=%.1f",
                    label, meanValue, minMax.minVal, minMax.maxVal, fps));
            Log.i(TAG, label + " auto frame " + w + "x" + h + " mean=" + fmt1(meanValue)
                    + " min=" + fmt1(minMax.minVal) + " max=" + fmt1(minMax.maxVal)
                    + " fps=" + fmt1(fps) + (saved ? "" : " (png write failed)"));
            gray.release();

            return new WorkingMode(candidate.format, candidate.width, candidate.height, fps);
        } catch (Throwable e) {
            Log.e(TAG, label + " portal failed", e);
            status(label + ": FAILED (" + e.getClass().getSimpleName() + ")");
            return null;
        } finally {
            if (portal != null) {
                try {
                    portal.close();
                } catch (Throwable e) {
                    Log.w(TAG, "portal close failed", e);
                }
            }
            sleep(PORTAL_CLOSE_SETTLE_MS);
        }
    }

    private void recordControls(VisionPortal portal, String label) {
        List<String> lines = new ArrayList<>();
        lines.add("== " + label);
        ExposureControl exposure = null;
        GainControl gain = null;
        try {
            exposure = portal.getCameraControl(ExposureControl.class);
        } catch (Throwable e) {
            lines.add("ExposureControl: unavailable (" + e + ")");
        }
        try {
            gain = portal.getCameraControl(GainControl.class);
        } catch (Throwable e) {
            lines.add("GainControl: unavailable (" + e + ")");
        }

        if (exposure != null) {
            try {
                lines.add("ExposureControl supported=" + exposure.isExposureSupported()
                        + " mode=" + exposure.getMode()
                        + " minMs=" + exposure.getMinExposure(TimeUnit.MILLISECONDS)
                        + " maxMs=" + exposure.getMaxExposure(TimeUnit.MILLISECONDS)
                        + " exposureMs=" + exposure.getExposure(TimeUnit.MILLISECONDS));
                ExposureControl.Mode[] modes = {
                        ExposureControl.Mode.Auto,
                        ExposureControl.Mode.Manual,
                        ExposureControl.Mode.ShutterPriority,
                        ExposureControl.Mode.AperturePriority};
                for (ExposureControl.Mode mode : modes) {
                    try {
                        lines.add("  isModeSupported(" + mode + ")=" + exposure.isModeSupported(mode));
                    } catch (Throwable e) {
                        lines.add("  isModeSupported(" + mode + ")=error(" + e + ")");
                    }
                }
            } catch (Throwable e) {
                lines.add("  exposure query failed: " + e);
            }
        } else {
            lines.add("ExposureControl: null");
        }

        if (gain != null) {
            try {
                lines.add("GainControl min=" + gain.getMinGain()
                        + " max=" + gain.getMaxGain()
                        + " gain=" + gain.getGain());
            } catch (Throwable e) {
                lines.add("  gain query failed: " + e);
            }
        } else {
            lines.add("GainControl: null");
        }

        for (String line : lines) {
            Log.i(TAG, line);
        }
        appendLines("controls.txt", lines);
    }

    private static WorkingMode pickBest(List<WorkingMode> working) {
        WorkingMode best = null;
        for (WorkingMode mode : working) {
            if (mode.width == 1280 && mode.height == 800) {
                return mode;
            }
            if (best == null || (long) mode.width * mode.height > (long) best.width * best.height) {
                best = mode;
            }
        }
        return best;
    }

    // ---------------------------------------------------------------------------------------
    // Phase 4: exposure / gain sweep
    // ---------------------------------------------------------------------------------------

    private String runSweep(WebcamName webcam, WorkingMode best) {
        status("sweep start " + best.label());
        VisionPortal portal = null;
        LatestGray processor = new LatestGray();
        try {
            portal = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .setStreamFormat(best.format)
                    .setCameraResolution(new Size(best.width, best.height))
                    .enableLiveView(false)
                    .addProcessor(processor)
                    .build();
            if (!waitForStreaming(portal, STREAM_TIMEOUT_MS)) {
                status("sweep: no stream (" + stateOf(portal) + ")");
                return null;
            }
            if (waitForNewFrame(processor, FRAME_TIMEOUT_MS) == null) {
                status("sweep: no frames");
                return null;
            }

            ExposureControl exposure = null;
            GainControl gain = null;
            try {
                exposure = portal.getCameraControl(ExposureControl.class);
            } catch (Throwable e) {
                Log.w(TAG, "sweep: no ExposureControl", e);
            }
            try {
                gain = portal.getCameraControl(GainControl.class);
            } catch (Throwable e) {
                Log.w(TAG, "sweep: no GainControl", e);
            }
            if (exposure == null) {
                status("sweep: ExposureControl unavailable");
                return null;
            }

            try {
                if (!exposure.setMode(ExposureControl.Mode.Manual)) {
                    status("sweep: manual mode rejected");
                }
            } catch (Throwable e) {
                Log.w(TAG, "sweep: setMode failed", e);
            }

            long minMs = 1;
            long maxMs = 33;
            try {
                minMs = Math.max(1, exposure.getMinExposure(TimeUnit.MILLISECONDS));
                long reportedMax = exposure.getMaxExposure(TimeUnit.MILLISECONDS);
                if (reportedMax > 0) maxMs = reportedMax;
            } catch (Throwable e) {
                Log.w(TAG, "sweep: exposure range query failed", e);
            }

            Set<Integer> gainValues = new LinkedHashSet<>();
            if (gain != null) {
                int minGain;
                int maxGain;
                try {
                    minGain = gain.getMinGain();
                    maxGain = gain.getMaxGain();
                } catch (Throwable e) {
                    minGain = -1;
                    maxGain = -1;
                }
                if (minGain >= 0 && maxGain >= 0) {
                    gainValues.add(minGain);
                    gainValues.add(minGain + (maxGain - minGain) / 2);
                    gainValues.add(maxGain);
                } else {
                    gainValues.add(-1);
                }
            } else {
                gainValues.add(-1);
            }
            int[] gains = new int[gainValues.size()];
            int gainIndex = 0;
            for (int value : gainValues) {
                gains[gainIndex++] = value;
            }

            Set<Long> exposures = new LinkedHashSet<>();
            for (long requested : EXPOSURES_MS) {
                exposures.add(Math.max(minMs, Math.min(maxMs, requested)));
            }

            String bestName = null;
            double bestMean = 0;
            double bestMin = 0;
            double bestMax = 0;
            double bestDistance = Double.MAX_VALUE;

            for (long exposureMs : exposures) {
                if (!opModeIsActive()) break;
                for (int gainValue : gains) {
                    if (!opModeIsActive()) break;
                    boolean exposureSet = false;
                    try {
                        exposureSet = exposure.setExposure(exposureMs, TimeUnit.MILLISECONDS);
                    } catch (Throwable e) {
                        Log.w(TAG, "sweep: setExposure failed", e);
                    }
                    boolean gainSet = gainValue < 0;
                    if (gainValue >= 0 && gain != null) {
                        try {
                            gainSet = gain.setGain(gainValue);
                        } catch (Throwable e) {
                            Log.w(TAG, "sweep: setGain failed", e);
                        }
                    }
                    sleep(SENSOR_SETTLE_MS);

                    Mat gray = waitForNewFrame(processor, FRAME_TIMEOUT_MS);
                    if (gray == null) {
                        status("sweep e" + exposureMs + " g" + gainValue + ": no frame");
                        continue;
                    }
                    int w = gray.cols();
                    int h = gray.rows();
                    String file = String.format(Locale.US,
                            "sweep_%dx%d_e%d_g%d.png", w, h, exposureMs, gainValue);
                    boolean saved = Imgcodecs.imwrite(DIR + "/" + file, gray);
                    Scalar mean = Core.mean(gray);
                    Core.MinMaxLocResult minMax = Core.minMaxLoc(gray);
                    double meanValue = mean.val[0];
                    float fps = portal.getFps();
                    appendSummary(best.formatName() + "_manual", w, h,
                            String.valueOf(exposureMs), String.valueOf(gainValue),
                            meanValue, minMax.minVal, minMax.maxVal, fps);
                    Log.i(TAG, "sweep " + file + " mean=" + fmt1(meanValue)
                            + " min=" + fmt1(minMax.minVal) + " max=" + fmt1(minMax.maxVal)
                            + " exposureSet=" + exposureSet + " gainSet=" + gainSet
                            + (saved ? "" : " (png write failed)"));
                    status(String.format(Locale.US, "sweep e%d g%d mean=%.1f max=%.0f",
                            exposureMs, gainValue, meanValue, minMax.maxVal));

                    if (minMax.maxVal < 255 && Math.abs(meanValue - 110) < bestDistance) {
                        bestDistance = Math.abs(meanValue - 110);
                        bestName = file;
                        bestMean = meanValue;
                        bestMin = minMax.minVal;
                        bestMax = minMax.maxVal;
                    }
                    gray.release();
                }
            }

            if (bestName == null) {
                return null;
            }
            return String.format(Locale.US, "%s mean=%.1f min=%.0f max=%.0f",
                    bestName, bestMean, bestMin, bestMax);
        } catch (Throwable e) {
            Log.e(TAG, "sweep failed", e);
            status("sweep failed: " + e.getClass().getSimpleName());
            return null;
        } finally {
            if (portal != null) {
                try {
                    portal.close();
                } catch (Throwable e) {
                    Log.w(TAG, "sweep portal close failed", e);
                }
            }
        }
    }

    // ---------------------------------------------------------------------------------------
    // Shared helpers
    // ---------------------------------------------------------------------------------------

    private boolean waitForStreaming(VisionPortal portal, long timeoutMs) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < timeoutMs) {
            VisionPortal.CameraState state;
            try {
                state = portal.getCameraState();
            } catch (Throwable e) {
                Log.w(TAG, "getCameraState failed", e);
                return false;
            }
            if (state == VisionPortal.CameraState.STREAMING) {
                return true;
            }
            if (state == VisionPortal.CameraState.ERROR
                    || state == VisionPortal.CameraState.CAMERA_DEVICE_CLOSED) {
                return false;
            }
            sleep(50);
        }
        return false;
    }

    private Mat waitForNewFrame(LatestGray processor, long timeoutMs) {
        long base = processor.frameCount();
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < timeoutMs) {
            if (processor.frameCount() > base) {
                return processor.snapshot();
            }
            sleep(20);
        }
        return null;
    }

    private static String stateOf(VisionPortal portal) {
        try {
            return String.valueOf(portal.getCameraState());
        } catch (Throwable e) {
            return "unknown";
        }
    }

    private void status(String line) {
        Log.i(TAG, line);
        statusLines.addLast(line);
        while (statusLines.size() > 12) {
            statusLines.removeFirst();
        }
        telemetry.clear();
        for (String statusLine : statusLines) {
            telemetry.addLine(statusLine);
        }
        telemetry.update();
    }

    private static String fmt1(double value) {
        return String.format(Locale.US, "%.1f", value);
    }

    private void writeSummaryHeader() {
        File file = new File(DIR, "summary.csv");
        if (!file.exists() || file.length() == 0) {
            appendLines("summary.csv",
                    Collections.singletonList("mode,width,height,exposure_ms,gain,mean,min,max,fps"));
        }
    }

    private void appendSummary(String mode, int width, int height,
                               String exposureMs, String gain,
                               double mean, double min, double max, float fps) {
        String row = String.format(Locale.US, "%s,%d,%d,%s,%s,%.1f,%.0f,%.0f,%.1f",
                mode, width, height, exposureMs, gain, mean, min, max, fps);
        appendLines("summary.csv", Collections.singletonList(row));
    }

    private void appendLines(String name, List<String> lines) {
        BufferedWriter writer = null;
        try {
            writer = new BufferedWriter(new FileWriter(new File(DIR, name), true));
            for (String line : lines) {
                writer.write(line);
                writer.write("\n");
            }
            writer.flush();
        } catch (Throwable e) {
            Log.e(TAG, "write " + name + " failed", e);
        } finally {
            if (writer != null) {
                try {
                    writer.close();
                } catch (IOException ignored) {
                }
            }
        }
    }

    // ---------------------------------------------------------------------------------------
    // Vision processor: keeps the newest grayscale frame
    // ---------------------------------------------------------------------------------------

    private static final class LatestGray implements VisionProcessor {
        private final Object lock = new Object();
        private Mat gray;
        private long frames;

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            Mat converted = new Mat();
            try {
                Imgproc.cvtColor(frame, converted, Imgproc.COLOR_RGBA2GRAY);
            } catch (Throwable e) {
                converted.release();
                return null;
            }
            synchronized (lock) {
                if (gray != null) {
                    gray.release();
                }
                gray = converted;
                frames++;
            }
            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                                Object userContext) {
        }

        long frameCount() {
            synchronized (lock) {
                return frames;
            }
        }

        Mat snapshot() {
            synchronized (lock) {
                return gray == null ? null : gray.clone();
            }
        }
    }

    // ---------------------------------------------------------------------------------------
    // Small value types
    // ---------------------------------------------------------------------------------------

    private static final class ModeCandidate {
        final VisionPortal.StreamFormat format;
        final int width;
        final int height;

        ModeCandidate(VisionPortal.StreamFormat format, int width, int height) {
            this.format = format;
            this.width = width;
            this.height = height;
        }

        String formatName() {
            return format.name().toLowerCase(Locale.US);
        }

        String label() {
            return format + " " + width + "x" + height;
        }
    }

    private static final class WorkingMode {
        final VisionPortal.StreamFormat format;
        final int width;
        final int height;
        final float fps;

        WorkingMode(VisionPortal.StreamFormat format, int width, int height, float fps) {
            this.format = format;
            this.width = width;
            this.height = height;
            this.fps = fps;
        }

        String formatName() {
            return format.name().toLowerCase(Locale.US);
        }

        String label() {
            return format + " " + width + "x" + height;
        }
    }
}
