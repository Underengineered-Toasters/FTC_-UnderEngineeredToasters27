package org.firstinspires.ftc.teamcode.ds;

import android.app.Activity;
import android.util.Base64;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.robotcore.internal.opmode.RegisteredOpModes;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.FtcWebSocket;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.FtcWebSocketMessage;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.WebSocketMessageTypeHandler;
import org.firstinspires.ftc.robotcore.internal.webserver.websockets.WebSocketNamespaceHandler;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.nio.ByteBuffer;
import java.nio.charset.CharacterCodingException;
import java.nio.charset.CodingErrorAction;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;

/**
 * Websocket namespace handler for the driver station UI ("ds" namespace).
 * Inbound payloads are base64-encoded UTF-8 strings; outbound payloads are
 * base64-encoded JSON. Every inbound message gets a reply or is deliberately
 * ignored, and nothing here throws into the SDK.
 *
 * Commands are checked against the robot's current state before they are sent,
 * so a double-click or a command for the wrong state cannot restart an opmode.
 * Control goes through the same asynchronous OpModeManagerImpl calls the SDK's
 * own command path uses; the event loop performs the actual swap.
 */
public class DsSocketHandler extends WebSocketNamespaceHandler {

    /** Largest accepted base64 payload, in characters. */
    static final int MAX_PAYLOAD_CHARS = 1024;
    /** Largest accepted opmode name, in characters. */
    static final int MAX_NAME_CHARS = 128;

    private static final String TAG = "DsSocketHandler";
    private static final String NOT_READY = "robot controller is not ready";
    /** Serialises command checks so two sockets cannot both pass the same check. */
    private static final Object COMMAND_LOCK = new Object();

    /** Outcome of checking one control command against the robot's snapshot. */
    enum Action { SEND, IGNORE, REJECT }

    /** A checked command: what to do, and the reason when rejecting. */
    static final class Verdict {
        final Action action;
        final String message;

        private Verdict(Action action, String message) {
            this.action = action;
            this.message = message;
        }

        static Verdict send() { return new Verdict(Action.SEND, null); }
        static Verdict ignore() { return new Verdict(Action.IGNORE, null); }
        static Verdict reject(String message) { return new Verdict(Action.REJECT, message); }
    }

    public DsSocketHandler() {
        super(DriverStation.NAMESPACE);
    }

    @Override
    protected void registerMessageTypeHandlers(Map<String, WebSocketMessageTypeHandler> map) {
        map.put("initOpMode", (msg, socket) -> onInitOpMode(msg, socket));
        map.put("startOpMode", (msg, socket) -> onStartOpMode(socket));
        map.put("stopOpMode", (msg, socket) -> onStopOpMode(socket));
        map.put("getState", (msg, socket) -> sendState(socket));
    }

    /**
     * Answers every message, including unknown ones. Without this override the SDK
     * falls through to its own command parser, which logs and does not reply.
     */
    @Override
    public boolean onMessage(FtcWebSocketMessage msg, FtcWebSocket socket) {
        String type = msg.getType();
        if (type == null) {
            sendError(socket, "message has no type");
            return true;
        }
        try {
            if (!super.onMessage(msg, socket)) {
                sendError(socket, "unknown message type '" + clip(type, 32) + "'");
            }
        } catch (RuntimeException e) {
            RobotLog.ee(TAG, e, "message handling failed");
            sendError(socket, "internal error handling '" + clip(type, 32) + "'");
        }
        return true;
    }

    @Override
    public void onSubscribe(FtcWebSocket socket) {
        DriverStation.subscriberJoined();
        try {
            sendState(socket);
            for (String line : DriverStation.logSnapshot()) {
                send(socket, new JSONObject().put("type", "log").put("line", line));
            }
            for (Map.Entry<String, String> e : DriverStation.telemetrySnapshot().entrySet()) {
                send(socket, new JSONObject().put("type", "telemetry")
                        .put("key", e.getKey()).put("value", e.getValue()));
            }
        } catch (Exception e) {
            // A dead socket mid-replay is not fatal.
        }
    }

    @Override
    public void onUnsubscribe(FtcWebSocket socket) {
        DriverStation.subscriberLeft();
    }

    private void onInitOpMode(FtcWebSocketMessage msg, FtcWebSocket socket) {
        String name;
        try {
            name = nameFromPayload(msg.getPayload());
        } catch (IllegalArgumentException e) {
            sendError(socket, "initOpMode: " + e.getMessage());
            return;
        }
        synchronized (COMMAND_LOCK) {
            if (!isUserOpMode(name)) {
                sendError(socket, "initOpMode: unknown opmode '" + name + "'");
                return;
            }
            OpModeManagerImpl manager = opModeManager();
            if (manager == null) {
                sendError(socket, "initOpMode: " + NOT_READY);
                return;
            }
            String active = activeOpModeName(manager);
            apply(socket, "initOpMode", checkInit(name, active, stateFor(active)),
                    () -> manager.initOpMode(name));
        }
    }

    private void onStartOpMode(FtcWebSocket socket) {
        synchronized (COMMAND_LOCK) {
            OpModeManagerImpl manager = opModeManager();
            if (manager == null) {
                sendError(socket, "startOpMode: " + NOT_READY);
                return;
            }
            String active = activeOpModeName(manager);
            apply(socket, "startOpMode", checkStart(active, stateFor(active)),
                    manager::startActiveOpMode);
        }
    }

    private void onStopOpMode(FtcWebSocket socket) {
        synchronized (COMMAND_LOCK) {
            OpModeManagerImpl manager = opModeManager();
            if (manager == null) {
                sendError(socket, "stopOpMode: " + NOT_READY);
                return;
            }
            String active = activeOpModeName(manager);
            // Same request the SDK's own stop command makes. stopActiveOpMode() would run the
            // opmode's stop() and the listeners on this websocket thread, racing the event loop.
            apply(socket, "stopOpMode", checkStop(active),
                    () -> manager.initOpMode(OpModeManagerImpl.DEFAULT_OP_MODE_NAME));
        }
    }

    private static void apply(FtcWebSocket socket, String command, Verdict verdict, Runnable action) {
        switch (verdict.action) {
            case SEND:
                runControl(socket, action);
                break;
            case REJECT:
                sendError(socket, command + ": " + verdict.message);
                break;
            default:
                // Already in the requested state: a repeated command is a no-op.
                break;
        }
    }

    // ---------- command policy (pure; unit tested) ----------

    /**
     * Init is a no-op for the opmode already initialised, and refused for the one
     * that is running (stop it first). Any other name is sent, which swaps opmodes.
     */
    static Verdict checkInit(String name, String active, String state) {
        if (name.equals(active)) {
            if ("INIT".equals(state)) return Verdict.ignore();
            if ("LOOPING".equals(state)) {
                return Verdict.reject("'" + name + "' is running; stop it before initialising it again");
            }
        }
        return Verdict.send();
    }

    /** Start needs an initialised opmode. Starting a running one is a no-op. */
    static Verdict checkStart(String active, String state) {
        if ("LOOPING".equals(state)) return Verdict.ignore();
        if ("INIT".equals(state) && active != null) return Verdict.send();
        return Verdict.reject("no initialised opmode to start; initialise one first");
    }

    /** Stop with nothing active is a no-op. */
    static Verdict checkStop(String active) {
        return active == null ? Verdict.ignore() : Verdict.send();
    }

    // ---------- payload validation (pure; unit tested) ----------

    /** Null when the encoded payload is acceptable, otherwise the reason. Null payload is empty. */
    static String payloadProblem(String encoded) {
        if (encoded == null) return null;
        if (encoded.length() > MAX_PAYLOAD_CHARS) return "payload is too large";
        int firstPad = encoded.indexOf('=');
        int dataEnd = firstPad < 0 ? encoded.length() : firstPad;
        for (int i = 0; i < dataEnd; i++) {
            if (!isBase64Char(encoded.charAt(i))) return "payload is not base64";
        }
        if (firstPad >= 0) {
            if (encoded.length() - firstPad > 2) return "payload is not base64";
            for (int i = firstPad; i < encoded.length(); i++) {
                if (encoded.charAt(i) != '=') return "payload is not base64";
            }
        }
        return null;
    }

    /** Null when the name is usable as an opmode name, otherwise the reason. */
    static String opModeNameProblem(String name) {
        if (name == null || name.isEmpty()) return "missing opmode name";
        if (name.length() > MAX_NAME_CHARS) return "opmode name is too long";
        for (int i = 0; i < name.length(); i++) {
            char c = name.charAt(i);
            if (c < 0x20 || c == 0x7f) return "opmode name has control characters";
        }
        return null;
    }

    /** Strict UTF-8 decode; null when the bytes are not valid UTF-8. */
    static String utf8Strict(byte[] bytes) {
        try {
            return StandardCharsets.UTF_8.newDecoder()
                    .onMalformedInput(CodingErrorAction.REPORT)
                    .onUnmappableCharacter(CodingErrorAction.REPORT)
                    .decode(ByteBuffer.wrap(bytes)).toString();
        } catch (CharacterCodingException e) {
            return null;
        }
    }

    /** Decodes a command payload into a validated opmode name. Throws IllegalArgumentException with the reason. */
    static String nameFromPayload(String encoded) {
        String problem = payloadProblem(encoded);
        if (problem != null) throw new IllegalArgumentException(problem);
        byte[] bytes;
        try {
            bytes = encoded == null ? new byte[0] : Base64.decode(encoded, Base64.DEFAULT);
        } catch (RuntimeException e) {
            throw new IllegalArgumentException("payload is not base64");
        }
        String name = utf8Strict(bytes);
        if (name == null) throw new IllegalArgumentException("payload is not UTF-8");
        problem = opModeNameProblem(name);
        if (problem != null) throw new IllegalArgumentException(problem);
        return name;
    }

    private static boolean isBase64Char(char c) {
        return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9')
                || c == '+' || c == '/';
    }

    private static String clip(String s, int max) {
        return s.length() <= max ? s : s.substring(0, max);
    }

    // ---------- SDK access ----------

    private static void runControl(FtcWebSocket socket, Runnable action) {
        try {
            action.run();
        } catch (RuntimeException e) {
            sendError(socket, e.getClass().getSimpleName() + ": " + e.getMessage());
        }
    }

    static OpModeManagerImpl opModeManager() {
        Activity activity = AppUtil.getInstance().getActivity();
        return activity == null ? null : OpModeManagerImpl.getOpModeManagerOfActivity(activity);
    }

    /** True for a registered, non-system opmode name. */
    private static boolean isUserOpMode(String name) {
        return !OpModeMeta.isSystemName(name)
                && RegisteredOpModes.getInstance().getOpModeMetadata(name) != null;
    }

    /** The SDK reports "$Stop$Robot$" when nothing is running; the UI wants null. */
    static String activeOpModeName(OpModeManagerImpl manager) {
        if (manager == null) return null;
        String name = manager.getActiveOpModeName();
        if (name == null || OpModeManagerImpl.DEFAULT_OP_MODE_NAME.equals(name)) return null;
        return name;
    }

    /** The notifier state only means something while an opmode is active. */
    static String stateFor(String active) {
        return active == null ? null : DriverStation.getOpModeState();
    }

    static void sendState(FtcWebSocket socket) {
        try {
            JSONArray names = new JSONArray();
            List<OpModeMeta> metas = RegisteredOpModes.getInstance().getOpModes();
            for (OpModeMeta meta : metas) {
                if (!OpModeMeta.isSystemName(meta.name)) {
                    names.put(meta.name);
                }
            }
            OpModeManagerImpl manager = opModeManager();
            String active = activeOpModeName(manager);
            String state = stateFor(active);

            JSONObject json = new JSONObject();
            json.put("type", "state");
            json.put("opmodes", names);
            json.put("active", active == null ? JSONObject.NULL : active);
            json.put("opmodeState", state == null ? JSONObject.NULL : state);
            send(socket, json);
        } catch (Exception e) {
            // Never throw into the SDK.
        }
    }

    static void sendError(FtcWebSocket socket, String message) {
        try {
            send(socket, new JSONObject().put("type", "error").put("message", message));
        } catch (JSONException | RuntimeException e) {
            // Keys are literals. A socket that died mid-send must not throw into the SDK's websocket thread.
        }
    }

    private static void send(FtcWebSocket socket, JSONObject json) throws JSONException {
        socket.send(new FtcWebSocketMessage(DriverStation.NAMESPACE,
                json.getString("type"), DriverStation.encodePayload(json.toString())));
    }
}
