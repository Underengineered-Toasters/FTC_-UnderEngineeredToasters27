package org.firstinspires.ftc.teamcode.ds;

import android.app.Activity;
import android.util.Base64;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;

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

import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;

/**
 * Websocket namespace handler for the driver station UI ("ds" namespace).
 * Inbound payloads are base64-encoded plain strings; outbound payloads are
 * base64-encoded JSON. Malformed input gets an "error" reply on the
 * offending socket and is never allowed to throw into the SDK.
 */
public class DsSocketHandler extends WebSocketNamespaceHandler {

    public DsSocketHandler() {
        super(DriverStation.NAMESPACE);
    }

    @Override
    protected void registerMessageTypeHandlers(Map<String, WebSocketMessageTypeHandler> map) {
        map.put("initOpMode", this::onInitOpMode);
        map.put("startOpMode", (msg, socket) -> runControl(socket, () -> opModeManager().startActiveOpMode()));
        map.put("stopOpMode", (msg, socket) -> runControl(socket, () -> opModeManager().stopActiveOpMode()));
        map.put("getState", (msg, socket) -> sendState(socket));
    }

    @Override
    public void onSubscribe(FtcWebSocket socket) {
        try {
            sendState(socket);
            for (String line : DriverStation.logSnapshot()) {
                JSONObject json = new JSONObject();
                json.put("type", "log");
                json.put("line", line);
                send(socket, json);
            }
            for (Map.Entry<String, String> e : DriverStation.telemetrySnapshot().entrySet()) {
                JSONObject json = new JSONObject();
                json.put("type", "telemetry");
                json.put("key", e.getKey());
                json.put("value", e.getValue());
                send(socket, json);
            }
        } catch (Exception e) {
            // A dead socket mid-replay is not fatal.
        }
    }

    private void onInitOpMode(FtcWebSocketMessage msg, FtcWebSocket socket) {
        String name = decodePayload(msg);
        if (name == null) {
            sendError(socket, "initOpMode: payload is not valid base64");
            return;
        }
        name = name.trim();
        if (!isRegistered(name)) {
            sendError(socket, "initOpMode: unknown opmode '" + name + "'");
            return;
        }
        final String opModeName = name;
        runControl(socket, () -> opModeManager().initOpMode(opModeName));
    }

    private void runControl(FtcWebSocket socket, Runnable action) {
        try {
            action.run();
        } catch (Exception e) {
            sendError(socket, e.getClass().getSimpleName() + ": " + e.getMessage());
        }
    }

    private static OpModeManagerImpl opModeManager() {
        Activity activity = AppUtil.getInstance().getActivity();
        return OpModeManagerImpl.getOpModeManagerOfActivity(activity);
    }

    private static boolean isRegistered(String name) {
        List<OpModeMeta> metas = RegisteredOpModes.getInstance().getOpModes();
        for (OpModeMeta meta : metas) {
            if (meta.name.equals(name)) {
                return true;
            }
        }
        return false;
    }

    /** The SDK reports "$Stop$Robot$" when nothing is running; the UI wants null. */
    static String activeOpModeName(OpModeManagerImpl manager) {
        if (manager == null) return null;
        String name = manager.getActiveOpModeName();
        if (name == null || OpModeManagerImpl.DEFAULT_OP_MODE_NAME.equals(name)) return null;
        return name;
    }


    static void sendState(FtcWebSocket socket) {
        try {
            JSONArray names = new JSONArray();
            for (OpModeMeta meta : RegisteredOpModes.getInstance().getOpModes()) {
                if (!OpModeMeta.isSystemName(meta.name)) {
                    names.put(meta.name);
                }
            }
            OpModeManagerImpl manager = opModeManager();
            String active = activeOpModeName(manager);
            String state = active == null ? null : DriverStation.getOpModeState();

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
            JSONObject json = new JSONObject();
            json.put("type", "error");
            json.put("message", message);
            send(socket, json);
        } catch (JSONException e) {
            // unreachable
        }
    }

    private static void send(FtcWebSocket socket, JSONObject json) throws JSONException {
        socket.send(new FtcWebSocketMessage(DriverStation.NAMESPACE,
                json.getString("type"), DriverStation.encodePayload(json.toString())));
    }

    private static String decodePayload(FtcWebSocketMessage msg) {
        try {
            // getPayload() can be JSON null; hasPayload() would NPE inside the SDK.
            String payload = msg.getPayload();
            if (payload == null || payload.isEmpty()) return "";
            return new String(Base64.decode(payload, Base64.DEFAULT), StandardCharsets.UTF_8);
        } catch (RuntimeException e) {
            return null;
        }
    }
}
