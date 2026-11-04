package org.firstinspires.ftc.teamcode.ds;

import android.util.Base64;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.FtcWebSocketMessage;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.WebSocketManager;
import org.json.JSONException;
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
 *
 * Nothing here blocks the opmode thread on the network. Messages are queued
 * into a bounded outbox and sent by one daemon thread; when the browser falls
 * behind, the oldest queued messages are dropped. Work is skipped entirely
 * while no browser is subscribed.
 */
public final class DriverStation {

    static final String NAMESPACE = "ds";
    private static final int LOG_CAPACITY = 200;
    /** Upper bound on telemetry keys per map; oldest keys are evicted first. */
    static final int TELEMETRY_CAPACITY = 256;
    /** Messages waiting for the sender thread; oldest dropped when full. */
    static final int OUTBOX_CAPACITY = 512;
    private static final String TYPE_TELEMETRY = "telemetry";
    private static final String TYPE_TELEMETRY_DUMP = "telemetryDump";
    private static final String TYPE_LOG = "log";

    private static volatile HardwareMap hardwareMap;
    private static volatile CameraStreamSource cameraSource;
    private static volatile WebSocketManager webSocketManager;
    /** "INIT" | "LOOPING" | null — tracked via OpModeManagerNotifier. */
    private static volatile String opModeState;
    /** Number of browsers subscribed to the "ds" namespace; guarded by the class lock. */
    private static volatile int subscriberCount;
    private static Thread sender; // guarded by the class lock

    private static final Deque<String> logBuffer = new ArrayDeque<>(LOG_CAPACITY);
    private static final Object TELEMETRY_LOCK = new Object();
    /** Values set through {@link #telemetry}; they persist until detach. */
    private static final Map<String, String> pinnedTelemetry = boundedMap();
    /** What the SDK telemetry currently shows; cleared whenever the SDK auto-clears. */
    private static final Map<String, String> frameTelemetry = boundedMap();
    private static final BoundedOutbox<Outgoing> outbox = new BoundedOutbox<>(OUTBOX_CAPACITY);

    private DriverStation() {}

    /** Called once by DriverStationServer when the web handlers are registered. */
    static void setWebSocketManager(WebSocketManager manager) {
        webSocketManager = manager;
    }

    public static void attach(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    /**
     * Forget the opmode's hardware, camera and telemetry. Connected browsers get
     * an empty telemetry dump so they clear their table too.
     */
    public static void detach() {
        hardwareMap = null;
        cameraSource = null;
        synchronized (TELEMETRY_LOCK) {
            pinnedTelemetry.clear();
            frameTelemetry.clear();
        }
        publishTelemetryDump();
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
        if (!hasSubscribers()) return;
        try {
            publish(TYPE_LOG, new JSONObject().put("type", TYPE_LOG).put("line", line));
        } catch (JSONException e) {
            // Keys are literals; unreachable.
        }
    }

    /**
     * Push one telemetry key/value pair. The value is pinned: it stays in the
     * replay snapshot until {@link #detach} and is not removed by the SDK's
     * telemetry auto-clear.
     */
    public static void telemetry(String key, Object value) {
        if (key == null) return;
        String v = String.valueOf(value);
        synchronized (TELEMETRY_LOCK) {
            pinnedTelemetry.put(key, v);
        }
        if (!hasSubscribers()) return;
        try {
            publish(TYPE_TELEMETRY, new JSONObject().put("type", TYPE_TELEMETRY)
                    .put("key", key).put("value", v));
        } catch (JSONException e) {
            // Keys are literals; unreachable.
        }
    }

    /** Sets one SDK-mirrored telemetry row. Null keys are ignored. */
    static void putTelemetry(String key, String value) {
        if (key == null) return;
        synchronized (TELEMETRY_LOCK) {
            frameTelemetry.put(key, value == null ? "" : value);
        }
    }

    static void removeTelemetry(String key) {
        if (key == null) return;
        synchronized (TELEMETRY_LOCK) {
            frameTelemetry.remove(key);
        }
    }

    /** Drops the SDK-mirrored rows; pinned values are kept. */
    static void clearFrameTelemetry() {
        synchronized (TELEMETRY_LOCK) {
            frameTelemetry.clear();
        }
    }

    static List<String> logSnapshot() {
        synchronized (logBuffer) {
            return new ArrayList<>(logBuffer);
        }
    }

    /** Pinned values, overridden by the current SDK frame where both set a key. */
    static Map<String, String> telemetrySnapshot() {
        synchronized (TELEMETRY_LOCK) {
            Map<String, String> merged = new LinkedHashMap<>(pinnedTelemetry);
            merged.putAll(frameTelemetry);
            return merged;
        }
    }

    /** Sends the full telemetry snapshot; the UI replaces its table with it. */
    static void publishTelemetryDump() {
        if (!hasSubscribers()) return;
        try {
            JSONObject entries = new JSONObject();
            for (Map.Entry<String, String> e : telemetrySnapshot().entrySet()) {
                entries.put(e.getKey(), e.getValue());
            }
            publish(TYPE_TELEMETRY_DUMP, new JSONObject().put("type", TYPE_TELEMETRY_DUMP)
                    .put("entries", entries));
        } catch (JSONException e) {
            // Keys are null-checked on insert; unreachable.
        }
    }

    /** Base64-encode a payload the way the SDK websocket protocol expects. */
    static String encodePayload(String s) {
        return Base64.encodeToString(s.getBytes(StandardCharsets.UTF_8), Base64.NO_WRAP);
    }

    static synchronized void subscriberJoined() {
        subscriberCount++;
    }

    static synchronized void subscriberLeft() {
        if (subscriberCount > 0) subscriberCount--;
    }

    static boolean hasSubscribers() {
        return subscriberCount > 0;
    }

    /**
     * Queues one message for the "ds" namespace. Never blocks; drops the oldest
     * queued message when the outbox is full. No-op without subscribers.
     */
    static void publish(String type, JSONObject json) {
        if (!hasSubscribers()) return;
        ensureSender();
        outbox.offer(new Outgoing(type, json.toString()));
    }

    private static synchronized void ensureSender() {
        if (sender != null) return;
        sender = new Thread(DriverStation::senderLoop, "ds-sender");
        sender.setDaemon(true);
        sender.start();
    }

    private static void senderLoop() {
        while (true) {
            Outgoing next;
            try {
                next = outbox.take();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
            WebSocketManager manager = webSocketManager;
            if (manager == null) continue;
            try {
                manager.broadcastToNamespace(NAMESPACE,
                        new FtcWebSocketMessage(NAMESPACE, next.type, encodePayload(next.json)));
            } catch (RuntimeException e) {
                // A socket that died mid-send or a manager being replaced must not stop the sender.
            }
        }
    }

    private static <V> Map<String, V> boundedMap() {
        return new LinkedHashMap<String, V>(16, 0.75f, false) {
            @Override
            protected boolean removeEldestEntry(Map.Entry<String, V> eldest) {
                return size() > TELEMETRY_CAPACITY;
            }
        };
    }

    /** One queued outbound message; the sender thread encodes it. */
    private static final class Outgoing {
        final String type;
        final String json;

        Outgoing(String type, String json) {
            this.type = type;
            this.json = json;
        }
    }

    /**
     * Bounded FIFO between the opmode thread and the sender. offer never blocks;
     * when full the oldest item is dropped, so a stalled browser cannot grow memory
     * or slow the opmode.
     */
    static final class BoundedOutbox<T> {
        private final ArrayDeque<T> items = new ArrayDeque<>();
        private final int capacity;
        private long dropped;

        BoundedOutbox(int capacity) {
            if (capacity < 1) throw new IllegalArgumentException("capacity must be positive");
            this.capacity = capacity;
        }

        synchronized void offer(T item) {
            if (item == null) throw new NullPointerException("item");
            if (items.size() >= capacity) {
                items.pollFirst();
                dropped++;
            }
            items.addLast(item);
            notifyAll();
        }

        /** Blocks until an item is available. */
        synchronized T take() throws InterruptedException {
            while (items.isEmpty()) {
                wait();
            }
            return items.pollFirst();
        }

        synchronized int size() {
            return items.size();
        }

        synchronized long dropped() {
            return dropped;
        }
    }
}
