package org.firstinspires.ftc.teamcode.ds;

import android.content.Context;
import android.content.res.AssetManager;
import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.WebSocketManager;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.lang.ref.WeakReference;
import java.nio.charset.StandardCharsets;
import java.util.Collections;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.regex.Pattern;

import fi.iki.elonen.NanoHTTPD;

/**
 * Registers the driver station web UI on the Robot Controller's built-in
 * web server. The SDK invokes {@link #registerWebHandlers} at startup via
 * the {@link WebHandlerRegistrar} annotation scan.
 *
 * Routes (exact-match dispatch in RobotWebHandlerManager):
 *   GET /ds and /ds/        -> ds/index.html
 *   GET /ds/<file>          -> assets/ds/<file>, read once at registration
 *   GET /ds/camera.mjpeg    -> MJPEG stream pulled by the HTTP thread, or 503 without a camera
 * Websocket namespace "ds"  -> DsSocketHandler
 *
 * Failures here are logged and never thrown into the SDK: this code runs inside the
 * Robot Controller app, and an uncaught exception on a background thread ends the app.
 */
public final class DriverStationServer {

    private static final String TAG = "DriverStationServer";
    private static final String ASSET_ROOT = "ds";
    private static final String URI_PREFIX = "/ds/";
    static final String MJPEG_BOUNDARY = "frame";
    static final String MJPEG_MIME = "multipart/x-mixed-replace; boundary=" + MJPEG_BOUNDARY;
    private static final String OCTET_STREAM = "application/octet-stream";
    private static final String TEXT_UTF8 = "; charset=utf-8";
    /** Pacing between camera frames, about 15 fps. */
    private static final long FRAME_INTERVAL_MS = 66;
    /** A camera that delivers nothing for this long ends the stream; the client shows its empty state. */
    private static final long FRAME_TIMEOUT_MS = 2000;
    private static final long POLL_PERIOD_MS = 500;
    private static final int JPEG_QUALITY = 75;
    private static final Pattern SAFE_SEGMENT = Pattern.compile("[A-Za-z0-9._-]+");

    /** Guards one-time startup work; the registrar can run again after a robot restart. */
    private static final AtomicBoolean started = new AtomicBoolean(false);
    private static final Object REGISTRATION_LOCK = new Object();
    /** The web server whose websocket manager already has the "ds" namespace. */
    private static WebSocketManager namespaceManager; // guarded by REGISTRATION_LOCK
    /**
     * Strong reference to the opmode listener: OpModeManagerImpl keeps listeners in a
     * WeakReferenceSet, so an unreferenced anonymous listener is garbage-collected and
     * state tracking silently freezes.
     */
    private static final OpModeManagerNotifier.Notifications opModeListener =
            new OpModeManagerNotifier.Notifications() {
                @Override
                public void onOpModePreInit(OpMode opMode) {
                    if (!isUserOpModeActive()) return;
                    DriverStation.attach(opMode);
                    if (!(opMode.telemetry instanceof DsTelemetry)) {
                        opMode.telemetry = new DsTelemetry(opMode.telemetry);
                    }
                    DriverStation.setOpModeState("INIT");
                }

                @Override
                public void onOpModePreStart(OpMode opMode) {
                    if (isUserOpModeActive()) DriverStation.setOpModeState("LOOPING");
                }

                @Override
                public void onOpModePostStop(OpMode opMode) {
                    DriverStation.setOpModeState(null);
                    DriverStation.detach();
                }
            };
    /**
     * Manager the listener is registered on. Weak, so a replaced manager (robot restart)
     * and its Activity are not kept alive by this class.
     */
    private static volatile WeakReference<OpModeManagerImpl> listenerManager = new WeakReference<>(null);

    private static final Map<String, String> MIME_TYPES = new HashMap<>();
    static {
        MIME_TYPES.put("html", "text/html" + TEXT_UTF8);
        MIME_TYPES.put("css", "text/css" + TEXT_UTF8);
        MIME_TYPES.put("js", "application/javascript" + TEXT_UTF8);
        MIME_TYPES.put("json", "application/json");
        MIME_TYPES.put("png", "image/png");
        MIME_TYPES.put("jpg", "image/jpeg");
        MIME_TYPES.put("jpeg", "image/jpeg");
        MIME_TYPES.put("svg", "image/svg+xml");
        MIME_TYPES.put("ico", "image/x-icon");
        MIME_TYPES.put("woff2", "font/woff2");
    }

    private DriverStationServer() {}

    @WebHandlerRegistrar
    public static void registerWebHandlers(Context context, WebHandlerManager manager) {
        try {
            registerNamespace(manager.getWebServer().getWebSocketManager());
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "driver station websocket namespace not registered");
        }
        try {
            registerRoutes(context.getAssets(), manager);
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "driver station web routes not registered");
        }
        if (started.compareAndSet(false, true)) {
            startDiagnosticsPoller();
        }
    }

    private static void registerNamespace(WebSocketManager wsManager) {
        synchronized (REGISTRATION_LOCK) {
            DriverStation.setWebSocketManager(wsManager);
            if (wsManager == namespaceManager) return; // this web server already has the handler
            wsManager.registerNamespaceHandler(new DsSocketHandler());
            namespaceManager = wsManager;
        }
    }

    private static void registerRoutes(AssetManager assets, WebHandlerManager manager) {
        WebHandler index = staticAsset(assets, ASSET_ROOT + "/index.html");
        manager.register("/ds", index);
        manager.register("/ds/", index);
        manager.register("/ds/camera.mjpeg", DriverStationServer::cameraResponse);
        // Dispatch is exact-match, so register every file under assets/ds/.
        registerAssetTree(assets, manager, ASSET_ROOT);
    }

    // ---------- static assets ----------

    private static void registerAssetTree(AssetManager assets, WebHandlerManager manager, String dir) {
        for (String child : list(assets, dir)) {
            String path = dir + "/" + child;
            if (list(assets, path).length > 0) {
                registerAssetTree(assets, manager, path);
            } else {
                manager.register(URI_PREFIX + path.substring(ASSET_ROOT.length() + 1),
                        staticAsset(assets, path));
            }
        }
    }

    private static String[] list(AssetManager assets, String dir) {
        try {
            String[] names = assets.list(dir);
            return names == null ? new String[0] : names;
        } catch (IOException e) {
            return new String[0];
        }
    }

    /** Reads the asset once; requests then serve the bytes from memory with a fixed length. */
    private static WebHandler staticAsset(AssetManager assets, String assetPath) {
        byte[] body = readAsset(assets, assetPath);
        if (body == null) {
            RobotLog.ee(TAG, "driver station asset missing: %s", assetPath);
            return session -> notFound();
        }
        String mime = contentTypeFor(assetPath);
        return session -> {
            if (!isSafeRequestPath(session.getUri())) return notFound();
            NanoHTTPD.Response response = NanoHTTPD.newFixedLengthResponse(
                    NanoHTTPD.Response.Status.OK, mime, new ByteArrayInputStream(body), body.length);
            response.addHeader("Cache-Control", "no-cache");
            return response;
        };
    }

    private static byte[] readAsset(AssetManager assets, String path) {
        try (InputStream in = assets.open(path)) {
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buffer = new byte[8192];
            int n;
            while ((n = in.read(buffer)) != -1) {
                out.write(buffer, 0, n);
            }
            return out.toByteArray();
        } catch (IOException e) {
            return null;
        }
    }

    private static NanoHTTPD.Response notFound() {
        return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
                NanoHTTPD.MIME_PLAINTEXT, "Not found");
    }

    /**
     * True when the request path is /ds or below it and every segment is a plain file name.
     * Asset handlers only ever serve fixed registered files, so this is defence in depth.
     */
    static boolean isSafeRequestPath(String uri) {
        if (uri == null || !(uri.equals("/ds") || uri.startsWith("/ds/"))) return false;
        for (String segment : uri.split("/")) {
            if (segment.isEmpty()) continue;
            if (segment.equals(".") || segment.equals("..")) return false;
            if (!SAFE_SEGMENT.matcher(segment).matches()) return false;
        }
        return true;
    }

    /** Content-Type for a file name. Text types carry an explicit UTF-8 charset; unknown types are octet-stream. */
    static String contentTypeFor(String path) {
        int dot = path.lastIndexOf('.');
        if (dot < 0 || dot < path.lastIndexOf('/')) return OCTET_STREAM;
        String mime = MIME_TYPES.get(path.substring(dot + 1).toLowerCase(Locale.ROOT));
        return mime != null ? mime : OCTET_STREAM;
    }

    // ---------- camera MJPEG ----------

    private static NanoHTTPD.Response cameraResponse(NanoHTTPD.IHTTPSession session) {
        if (DriverStation.getCameraSource() == null) {
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.SERVICE_UNAVAILABLE,
                    NanoHTTPD.MIME_PLAINTEXT, "no camera source");
        }
        // Frames are pulled by the HTTP thread as the client accepts them. A client that
        // disconnects fails the socket write and the stream is simply dropped: no thread outlives it.
        NanoHTTPD.Response response = NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                MJPEG_MIME, new MjpegStream(DriverStationServer::nextCameraJpeg, FRAME_INTERVAL_MS));
        response.addHeader("Cache-Control", "no-cache");
        return response;
    }

    /**
     * Next camera frame as JPEG. Returns null (end the stream) when there is no camera,
     * the camera stalls, or something fails; an empty array skips one undecodable frame.
     * The frame is compressed inside the callback, because the SDK recycles the bitmap as
     * soon as the callback returns and it must not be used from another thread afterwards.
     */
    private static byte[] nextCameraJpeg() throws InterruptedException {
        try {
            CameraStreamSource source = DriverStation.getCameraSource();
            if (source == null) return null;
            FrameSlot<byte[]> slot = new FrameSlot<>();
            source.getFrameBitmap(Continuation.createTrivial(
                    (Consumer<Bitmap>) bitmap -> slot.deliver(compressJpeg(bitmap))));
            return slot.await(FRAME_TIMEOUT_MS);
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "camera frame failed; ending the stream");
            return null;
        }
    }

    /** Runs on the SDK's thread inside the frame callback, so it must never throw. */
    private static byte[] compressJpeg(Bitmap bitmap) {
        if (bitmap == null) return new byte[0];
        try {
            ByteArrayOutputStream bos = new ByteArrayOutputStream(32 * 1024);
            if (!bitmap.compress(Bitmap.CompressFormat.JPEG, JPEG_QUALITY, bos)) {
                return new byte[0];
            }
            return bos.toByteArray();
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "could not compress a camera frame; skipping it");
            return new byte[0];
        }
    }

    /** One multipart/x-mixed-replace part: header with the JPEG length, the JPEG, and CRLF. */
    static byte[] mjpegPart(byte[] jpeg) {
        byte[] header = ("--" + MJPEG_BOUNDARY + "\r\nContent-Type: image/jpeg\r\nContent-Length: "
                + jpeg.length + "\r\n\r\n").getBytes(StandardCharsets.US_ASCII);
        byte[] part = new byte[header.length + jpeg.length + 2];
        System.arraycopy(header, 0, part, 0, header.length);
        System.arraycopy(jpeg, 0, part, header.length, jpeg.length);
        part[part.length - 2] = '\r';
        part[part.length - 1] = '\n';
        return part;
    }

    /**
     * Hands one value from a callback thread to a reader that waits with a timeout.
     * The first value wins; a value that arrives after the reader gave up, or a
     * second value, is dropped. Callers hand over finished data, not camera bitmaps,
     * because the SDK recycles a bitmap when its callback returns.
     */
    static final class FrameSlot<T> {

        private T frame;
        private boolean closed;

        /** Called on the camera's thread. */
        void deliver(T value) {
            if (value == null) return;
            synchronized (this) {
                if (closed || frame != null) return;
                frame = value;
                notifyAll();
            }
        }

        /** Waits up to timeoutMs. Returns the value, or null on timeout; the slot is closed either way. */
        synchronized T await(long timeoutMs) throws InterruptedException {
            long deadline = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(timeoutMs);
            while (frame == null) {
                long left = deadline - System.nanoTime();
                if (left <= 0) break;
                TimeUnit.NANOSECONDS.timedWait(this, left);
            }
            T taken = frame;
            frame = null;
            closed = true;
            return taken;
        }
    }

    /**
     * Pull-based multipart JPEG body. Reads block on the next frame, so the HTTP
     * thread's socket writes set the pace and a slow client slows the camera
     * requests instead of queueing frames. Ends on a null frame, an interrupt,
     * or an unexpected failure.
     */
    static final class MjpegStream extends InputStream {

        interface FrameSource {
            /** Next encoded JPEG, an empty array to skip it, or null to end the stream. */
            byte[] nextJpeg() throws InterruptedException;
        }

        private final FrameSource source;
        private final long frameIntervalMs;
        private byte[] part;
        private int offset;
        private boolean sentAny;
        private boolean ended;

        MjpegStream(FrameSource source, long frameIntervalMs) {
            this.source = source;
            this.frameIntervalMs = frameIntervalMs;
        }

        @Override
        public int read() {
            byte[] one = new byte[1];
            int n = read(one, 0, 1);
            return n == -1 ? -1 : one[0] & 0xff;
        }

        @Override
        public int read(byte[] b, int off, int len) {
            if (len == 0) return 0;
            while (part == null || offset >= part.length) {
                if (ended || !nextPart()) {
                    ended = true;
                    return -1;
                }
            }
            int n = Math.min(len, part.length - offset);
            System.arraycopy(part, offset, b, off, n);
            offset += n;
            return n;
        }

        @Override
        public void close() {
            ended = true;
            part = null;
        }

        private boolean nextPart() {
            try {
                while (true) {
                    if (sentAny && frameIntervalMs > 0) Thread.sleep(frameIntervalMs);
                    byte[] jpeg = source.nextJpeg();
                    if (jpeg == null) return false;
                    if (jpeg.length > 0) {
                        part = mjpegPart(jpeg);
                        offset = 0;
                        sentAny = true;
                        return true;
                    }
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return false;
            } catch (RuntimeException e) {
                return false;
            }
        }
    }

    // ---------- opmode state tracking ----------

    /** True while a user opmode (not the SDK's default "$Stop$Robot$") is the active one. */
    private static boolean isUserOpModeActive() {
        OpModeManagerImpl manager = listenerManager.get();
        return manager == null || DsSocketHandler.activeOpModeName(manager) != null;
    }

    /**
     * Registers {@link #opModeListener} on the current OpModeManagerImpl. The SDK
     * replaces the manager when the event loop is recreated (robot restart /
     * configuration change), so the poller re-registers whenever the manager
     * instance changes.
     */
    private static void ensureListenerRegistered() {
        try {
            OpModeManagerImpl manager = OpModeManagerImpl.getOpModeManagerOfActivity(
                    AppUtil.getInstance().getActivity());
            if (manager == null || manager == listenerManager.get()) return;
            manager.registerListener(opModeListener);
            listenerManager = new WeakReference<>(manager);
        } catch (RuntimeException e) {
            // State stays null until the next poll; diagnostics still report the active name.
        }
    }

    // ---------- diagnostics poller ----------

    private static void startDiagnosticsPoller() {
        Thread poller = new Thread(() -> {
            while (true) {
                try {
                    Thread.sleep(POLL_PERIOD_MS);
                    ensureListenerRegistered();
                    // Reading hardware costs USB round trips on the hub. Do it only while a browser watches.
                    if (DriverStation.hasSubscribers()) {
                        DriverStation.publish("diag", collectDiagnostics());
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                } catch (Throwable t) {
                    // Diagnostics are optional. An uncaught throwable on a background thread
                    // ends the whole Robot Controller app, so log it and keep polling.
                    RobotLog.ee(TAG, t, "diagnostics poll failed");
                }
            }
        }, "ds-diagnostics");
        poller.setDaemon(true);
        poller.start();
    }

    private static JSONObject collectDiagnostics() {
        JSONObject json = new JSONObject();
        try {
            json.put("type", "diag");
            HardwareMap map = DriverStation.getHardwareMap();
            json.put("battery", map == null ? JSONObject.NULL : batteryOf(map));
            json.put("motors", map == null ? new JSONArray() : motorsOf(map));
            json.put("servos", map == null ? new JSONArray() : servosOf(map));

            String active = DsSocketHandler.activeOpModeName(DsSocketHandler.opModeManager());
            json.put("active", active == null ? JSONObject.NULL : active);
            String state = DsSocketHandler.stateFor(active);
            json.put("opmodeState", state == null ? JSONObject.NULL : state);
        } catch (JSONException | RuntimeException e) {
            // Keys are literals; keep whatever was built.
        }
        return json;
    }

    /** Highest live voltage sensor reading, or null when none responds. */
    private static Object batteryOf(HardwareMap map) {
        double battery = Double.NaN;
        try {
            for (VoltageSensor sensor : map.voltageSensor) {
                try {
                    double v = sensor.getVoltage();
                    if (!Double.isNaN(v) && (Double.isNaN(battery) || v > battery)) {
                        battery = v;
                    }
                } catch (RuntimeException e) {
                    // An unplugged or failed sensor must not hide the others.
                }
            }
        } catch (RuntimeException e) {
            return JSONObject.NULL;
        }
        return finite(battery);
    }

    private static JSONArray motorsOf(HardwareMap map) {
        JSONArray motors = new JSONArray();
        for (String name : namesOf(map, DcMotor.class)) {
            try {
                DcMotor motor = map.get(DcMotor.class, name);
                if (motor == null) continue;
                Object current = motor instanceof DcMotorEx
                        ? finite(((DcMotorEx) motor).getCurrent(CurrentUnit.MILLIAMPS))
                        : JSONObject.NULL;
                motors.put(new JSONObject()
                        .put("name", name)
                        .put("power", finite(motor.getPower()))
                        .put("position", motor.getCurrentPosition())
                        .put("busy", motor.isBusy())
                        .put("current", current));
            } catch (RuntimeException | JSONException e) {
                // One missing or unplugged motor must not hide the rest.
            }
        }
        return motors;
    }

    private static JSONArray servosOf(HardwareMap map) {
        JSONArray servos = new JSONArray();
        for (String name : namesOf(map, Servo.class)) {
            try {
                Servo servo = map.get(Servo.class, name);
                if (servo == null) continue;
                servos.put(new JSONObject()
                        .put("name", name)
                        .put("position", finite(servo.getPosition())));
            } catch (RuntimeException | JSONException e) {
                // One missing or unplugged servo must not hide the rest.
            }
        }
        return servos;
    }

    private static Set<String> namesOf(HardwareMap map, Class<? extends HardwareDevice> type) {
        try {
            return map.getAllNames(type);
        } catch (RuntimeException e) {
            return Collections.emptySet();
        }
    }

    /** JSON cannot carry NaN or infinity; those become null. */
    private static Object finite(double v) {
        return Double.isNaN(v) || Double.isInfinite(v) ? JSONObject.NULL : Double.valueOf(v);
    }
}
