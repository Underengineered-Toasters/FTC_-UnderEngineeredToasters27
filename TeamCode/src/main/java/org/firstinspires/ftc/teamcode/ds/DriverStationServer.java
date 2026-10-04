package org.firstinspires.ftc.teamcode.ds;

import android.content.Context;
import android.content.res.AssetManager;
import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.FtcWebSocketMessage;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.WebSocketManager;
import org.json.JSONArray;
import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.PipedInputStream;
import java.io.PipedOutputStream;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import fi.iki.elonen.NanoHTTPD;

/**
 * Registers the driver station web UI on the Robot Controller's built-in
 * web server. The SDK invokes {@link #registerWebHandlers} at startup via
 * the {@link WebHandlerRegistrar} annotation scan.
 *
 * Routes (exact-match dispatch in RobotWebHandlerManager):
 *   GET /ds and /ds/        -> ds/index.html
 *   GET /ds/<file>          -> assets/ds/<file> (one handler per asset file,
 *                            registered at startup; anything else falls
 *                            through to the SDK's 404 asset fallback)
 *   GET /ds/camera.mjpeg    -> MJPEG stream, or 503 when no camera source
 * Websocket namespace "ds"  -> DsSocketHandler
 */
public final class DriverStationServer {

    private static final String ASSET_ROOT = "ds";
    private static final String URI_PREFIX = "/ds/";
    private static final long FRAME_INTERVAL_MS = 66; // ~15 fps
    private static final long FRAME_TIMEOUT_MS = 2000;
    private static final long POLL_PERIOD_MS = 500;
    /** Guards one-time startup work; the registrar can run again after a robot restart. */
    private static final AtomicBoolean started = new AtomicBoolean(false);
    /**
     * Strong reference to the opmode listener: OpModeManagerImpl keeps listeners in a
     * WeakReferenceSet, so an unreferenced anonymous listener is garbage-collected and
     * state tracking silently freezes.
     */
    private static final OpModeManagerNotifier.Notifications opModeListener =
            new OpModeManagerNotifier.Notifications() {
                @Override
                public void onOpModePreInit(OpMode opMode) {
                    DriverStation.attach(opMode);
                    if (!(opMode.telemetry instanceof DsTelemetry)) {
                        opMode.telemetry = new DsTelemetry(opMode.telemetry);
                    }
                    DriverStation.setOpModeState("INIT");
                }
                @Override
                public void onOpModePreStart(OpMode opMode) {
                    DriverStation.setOpModeState("LOOPING");
                }

                @Override
                public void onOpModePostStop(OpMode opMode) {
                    DriverStation.setOpModeState(null);
                    DriverStation.detach();
                }
            };
    /** Last manager the listener was registered on; replaced on robot restart. */
    private static volatile OpModeManagerImpl listenerManager;

    private static final Map<String, String> MIME_TYPES = new HashMap<>();
    static {
        MIME_TYPES.put("html", "text/html");
        MIME_TYPES.put("css", "text/css");
        MIME_TYPES.put("js", "application/javascript");
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
        WebSocketManager wsManager = manager.getWebServer().getWebSocketManager();
        DriverStation.setWebSocketManager(wsManager);
        wsManager.registerNamespaceHandler(new DsSocketHandler());

        WebHandler index = assetHandler(context.getAssets(), ASSET_ROOT + "/index.html");
        manager.register("/ds", index);
        manager.register("/ds/", index);
        manager.register("/ds/camera.mjpeg", DriverStationServer::cameraResponse);

        // Dispatch is exact-match, so register every file under assets/ds/.
        registerAssetTree(context.getAssets(), manager, ASSET_ROOT);
        if (started.compareAndSet(false, true)) {
            startDiagnosticsPoller(wsManager);
        }
    }

    // ---------- static assets ----------

    private static void registerAssetTree(AssetManager assets, WebHandlerManager manager, String dir) {
        String[] children;
        try {
            children = assets.list(dir);
        } catch (IOException e) {
            return;
        }
        if (children == null) return;
        for (String child : children) {
            String path = dir + "/" + child;
            String[] grandChildren;
            try {
                grandChildren = assets.list(path);
            } catch (IOException e) {
                grandChildren = null;
            }
            if (grandChildren != null && grandChildren.length > 0) {
                registerAssetTree(assets, manager, path);
            } else {
                manager.register(URI_PREFIX + path.substring(ASSET_ROOT.length() + 1),
                        assetHandler(assets, path));
            }
        }
    }

    private static WebHandler assetHandler(AssetManager assets, String assetPath) {
        return session -> {
            String uri = session.getUri();
            // Defensive: every URI segment must be a plain filename.
            for (String segment : uri.split("/")) {
                if (!segment.isEmpty() && !segment.matches("[A-Za-z0-9._-]+")) {
                    return NanoHTTPD.newFixedLengthResponse(
                            NanoHTTPD.Response.Status.NOT_FOUND, NanoHTTPD.MIME_PLAINTEXT, "");
                }
            }
            try {
                InputStream stream = assets.open(assetPath);
                return NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                        mimeType(assetPath), stream);
            } catch (IOException e) {
                return NanoHTTPD.newFixedLengthResponse(
                        NanoHTTPD.Response.Status.NOT_FOUND, NanoHTTPD.MIME_PLAINTEXT, "");
            }
        };
    }

    private static String mimeType(String path) {
        int dot = path.lastIndexOf('.');
        String mime = dot < 0 ? null : MIME_TYPES.get(path.substring(dot + 1).toLowerCase());
        return mime != null ? mime : "application/octet-stream";
    }

    // ---------- camera MJPEG ----------

    private static NanoHTTPD.Response cameraResponse(NanoHTTPD.IHTTPSession session) {
        if (DriverStation.getCameraSource() == null) {
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.SERVICE_UNAVAILABLE,
                    NanoHTTPD.MIME_PLAINTEXT, "no camera source");
        }
        try {
            PipedInputStream in = new PipedInputStream(64 * 1024);
            PipedOutputStream out = new PipedOutputStream(in);
            Thread writer = new Thread(() -> writeFrames(out), "ds-mjpeg");
            writer.setDaemon(true);
            writer.start();
            return NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                    "multipart/x-mixed-replace; boundary=frame", in);
        } catch (IOException e) {
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR,
                    NanoHTTPD.MIME_PLAINTEXT, "stream setup failed");
        }
    }

    private static void writeFrames(PipedOutputStream out) {
        try {
            while (true) {
                CameraStreamSource source = DriverStation.getCameraSource();
                if (source == null) break;
                Bitmap frame = awaitFrame(source);
                if (frame == null) {
                    Thread.sleep(FRAME_INTERVAL_MS);
                    continue;
                }
                byte[] jpeg = compressJpeg(frame);
                frame.recycle();
                out.write("--frame\r\nContent-Type: image/jpeg\r\n\r\n".getBytes(java.nio.charset.StandardCharsets.UTF_8));
                out.write(jpeg);
                out.write("\r\n".getBytes(java.nio.charset.StandardCharsets.UTF_8));
                out.flush();
                Thread.sleep(FRAME_INTERVAL_MS);
            }
        } catch (IOException e) {
            // Client disconnected; normal exit.
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } finally {
            try {
                out.close();
            } catch (IOException ignored) {
            }
        }
    }

    private static Bitmap awaitFrame(CameraStreamSource source) throws InterruptedException {
        ArrayBlockingQueue<Bitmap> queue = new ArrayBlockingQueue<>(1);
        source.getFrameBitmap(Continuation.createTrivial((Consumer<Bitmap>) queue::offer));
        return queue.poll(FRAME_TIMEOUT_MS, TimeUnit.MILLISECONDS);
    }

    private static byte[] compressJpeg(Bitmap bitmap) throws IOException {
        ByteArrayOutputStream bos = new ByteArrayOutputStream(32 * 1024);
        bitmap.compress(Bitmap.CompressFormat.JPEG, 75, bos);
        bos.close();
        return bos.toByteArray();
    }

    // ---------- opmode state tracking ----------

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
            if (manager == null || manager == listenerManager) return;
            manager.registerListener(opModeListener);
            listenerManager = manager;
        } catch (Exception e) {
            // State stays null; diagnostics still report the active name.
        }
    }

    // ---------- diagnostics poller ----------

    private static void startDiagnosticsPoller(WebSocketManager wsManager) {
        Thread poller = new Thread(() -> {
            while (true) {
                try {
                    Thread.sleep(POLL_PERIOD_MS);
                    ensureListenerRegistered();
                    String json = collectDiagnostics();
                    wsManager.broadcastToNamespace(DriverStation.NAMESPACE,
                            new FtcWebSocketMessage(DriverStation.NAMESPACE, "diag",
                                    DriverStation.encodePayload(json)));
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                } catch (Exception e) {
                    // Keep polling; a bad cycle must not kill the thread.
                }
            }
        }, "ds-diagnostics");
        poller.setDaemon(true);
        poller.start();
    }

    private static String collectDiagnostics() {
        JSONObject json = new JSONObject();
        try {
            json.put("type", "diag");

            HardwareMap map = DriverStation.getHardwareMap();
            if (map != null) {
                double battery = Double.NaN;
                for (VoltageSensor sensor : map.voltageSensor) {
                    double v = sensor.getVoltage();
                    if (!Double.isNaN(v) && (Double.isNaN(battery) || v > battery)) {
                        battery = v;
                    }
                }
                json.put("battery", Double.isNaN(battery) ? JSONObject.NULL : battery);

                JSONArray motors = new JSONArray();
                for (String name : map.getAllNames(DcMotor.class)) {
                    DcMotor motor = map.get(DcMotor.class, name);
                    if (motor == null) continue;
                    JSONObject m = new JSONObject();
                    m.put("name", name);
                    m.put("power", motor.getPower());
                    m.put("position", motor.getCurrentPosition());
                    m.put("busy", motor.isBusy());
                    if (motor instanceof DcMotorEx) {
                        m.put("current", ((DcMotorEx) motor).getCurrent(CurrentUnit.MILLIAMPS));
                    } else {
                        m.put("current", JSONObject.NULL);
                    }
                    motors.put(m);
                }
                json.put("motors", motors);

                JSONArray servos = new JSONArray();
                for (String name : map.getAllNames(Servo.class)) {
                    Servo servo = map.get(Servo.class, name);
                    if (servo == null) continue;
                    JSONObject s = new JSONObject();
                    s.put("name", name);
                    s.put("position", servo.getPosition());
                    servos.put(s);
                }
                json.put("servos", servos);
            } else {
                json.put("battery", JSONObject.NULL);
                json.put("motors", new JSONArray());
                json.put("servos", new JSONArray());
            }

            String active = null;
            try {
                OpModeManagerImpl manager = OpModeManagerImpl.getOpModeManagerOfActivity(
                        AppUtil.getInstance().getActivity());
                active = DsSocketHandler.activeOpModeName(manager);
            } catch (Exception ignored) {
            }
            json.put("active", active == null ? JSONObject.NULL : active);
            String state = active == null ? null : DriverStation.getOpModeState();
            json.put("opmodeState", state == null ? JSONObject.NULL : state);
        } catch (Exception e) {
            // Return whatever was accumulated.
        }
        return json.toString();
    }
}
