package org.firstinspires.ftc.teamcode.ds;

import android.util.Base64;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.FtcWebSocketMessage;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.WebSocketManager;
import org.json.JSONObject;

import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Static facade opmodes use to feed the driver station web UI.
 * All public methods are safe to call unconditionally: they no-op when
 * nothing is attached or the websocket manager is not registered yet.
 * Thread-safe: called from the opmode thread, the diagnostics poller,
 * and websocket threads.
 */
public final class DriverStation {

    static final String NAMESPACE = "ds";
    private static final int LOG_CAPACITY = 200;

    private static volatile HardwareMap hardwareMap;
    private static volatile CameraStreamSource cameraSource;
    private static volatile WebSocketManager webSocketManager;
    /** "INIT" | "LOOPING" | null — tracked via OpModeManagerNotifier. */
    private static volatile String opModeState;

    private static final Deque<String> logBuffer = new ArrayDeque<>(LOG_CAPACITY);
    private static final Map<String, String> telemetrySnapshot = new LinkedHashMap<>();

    private DriverStation() {}

    /** Called once by DriverStationServer when the web handlers are registered. */
    static void setWebSocketManager(WebSocketManager manager) {
        webSocketManager = manager;
    }

    public static void attach(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    public static void detach() {
        hardwareMap = null;
        cameraSource = null;
        synchronized (telemetrySnapshot) {
            telemetrySnapshot.clear();
        }
    }

    public static HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public static void setCameraSource(CameraStreamSource source) {
        cameraSource = source;
    }

    public static CameraStreamSource getCameraSource() {
        return cameraSource;
    }

    static void setOpModeState(String state) {
        opModeState = state;
    }

    static String getOpModeState() {
        return opModeState;
    }

    /** Append a line to the debug log stream and ring buffer. */
    public static void log(String line) {
        if (line == null) return;
        synchronized (logBuffer) {
            if (logBuffer.size() >= LOG_CAPACITY) {
                logBuffer.pollFirst();
            }
            logBuffer.addLast(line);
        }
        JSONObject json = new JSONObject();
        try {
            json.put("type", "log");
            json.put("line", line);
        } catch (org.json.JSONException e) {
            return; // keys are literals; unreachable
        }
        broadcast(json);
    }

    /** Push one telemetry key/value pair; also updates the replay snapshot. */
    public static void telemetry(String key, Object value) {
        if (key == null) return;
        String v = String.valueOf(value);
        synchronized (telemetrySnapshot) {
            telemetrySnapshot.put(key, v);
        }
        JSONObject json = new JSONObject();
        try {
            json.put("type", "telemetry");
            json.put("key", key);
            json.put("value", v);
        } catch (org.json.JSONException e) {
            return;
        }
        broadcast(json);
    }

    static void putTelemetry(String key, String value) {
        synchronized (telemetrySnapshot) {
            telemetrySnapshot.put(key, value);
        }
    }

    static void removeTelemetry(String key) {
        synchronized (telemetrySnapshot) {
            telemetrySnapshot.remove(key);
        }
    }

    static void clearTelemetry() {
        synchronized (telemetrySnapshot) {
            telemetrySnapshot.clear();
        }
    }

    static List<String> logSnapshot() {
        synchronized (logBuffer) {
            return new ArrayList<>(logBuffer);
        }
    }

    static Map<String, String> telemetrySnapshot() {
        synchronized (telemetrySnapshot) {
            return new LinkedHashMap<>(telemetrySnapshot);
        }
    }

    /** Base64-encode a payload the way the SDK websocket protocol expects. */
    static String encodePayload(String s) {
        return Base64.encodeToString(s.getBytes(StandardCharsets.UTF_8), Base64.NO_WRAP);
    }

    static void broadcast(JSONObject json) {
        WebSocketManager manager = webSocketManager;
        if (manager == null) return;
        try {
            manager.broadcastToNamespace(NAMESPACE,
                    new FtcWebSocketMessage(NAMESPACE, json.getString("type"), encodePayload(json.toString())));
        } catch (Exception e) {
            // Never let a broadcast failure reach the opmode thread.
        }
    }
}
