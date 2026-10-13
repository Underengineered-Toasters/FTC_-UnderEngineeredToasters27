"use strict";
/*
 * Driver station client for the FTC driver station web UI.
 *
 * Transport: the SDK websocket on ws://<robot>:8081 (wss when the page is
 * served over TLS). We subscribe to the "ds" namespace with the system
 * message type "subscribeToNamespace" and the RAW namespace name (not base64).
 *
 * Wire format (both directions), matching DsSocketHandler / DriverStation:
 *   { namespace, type, payload }
 * The robot base64-encodes its JSON payloads (UTF-8). Commands to the robot
 * carry a base64-encoded UTF-8 string payload; start/stop use an empty string.
 *
 * Inbound robot message types handled here:
 *   state         { opmodes[], active|null, opmodeState("INIT"|"LOOPING"|null) }
 *   diag          { battery, motors[], servos[], active|null, opmodeState|null }
 *   telemetry     { key, value }          (single pair, live + replay)
 *   telemetryDump { entries{ key: value } } (full snapshot per telemetry.update())
 *   log           { line }
 *   error         { message }
 *
 * Outbound command types: initOpMode (payload = opmode name), startOpMode,
 * stopOpMode, getState.
 *
 * Safety: control buttons are enabled only when the socket is open AND the
 * robot-reported state permits the action. All robot-supplied text is written
 * with textContent; this file never assigns innerHTML. No network access
 * beyond the robot websocket and the MJPEG camera, no external dependencies.
 *
 * The pure helpers are exported for unit tests (see tests/app.test.js) and the
 * DOM layer only runs when a document exists.
 */
(function (root, factory) {
  var api = factory();
  if (typeof module !== "undefined" && module.exports) module.exports = api;
  if (typeof document !== "undefined") {
    root.DriverStationUI = api;
    api.start();
  }
})(typeof globalThis !== "undefined" ? globalThis : this, function () {

  var MAX_LOG_LINES = 400;
  var SPARK_MAX = 120;
  var CAMERA_URL = "/ds/camera.mjpeg";
  var CAMERA_COOLDOWN_MS = 4000;
  var WS_PORT = 8081;
  var RECONNECT_BASE_MS = 1000;
  var RECONNECT_MAX_MS = 10000;
  var PENDING_TIMEOUT_MS = 4000;
  var ROBOT_NAMESPACE = "ds";

  // Every element id the DOM layer uses. Kept in one place so the retained-id
  // contract with index.html is auditable and testable.
  var ELEMENT_IDS = [
    "banner", "conn", "opmode", "opstate", "active", "battery", "batterySpark",
    "motors", "motorsEmpty", "servos", "servosEmpty", "telemetry", "telemetryEmpty",
    "log", "logPause", "logClear", "logCopy", "cam", "camEmpty",
    "btnInit", "btnStart", "btnStop",
  ];

  // ---------------------------------------------------------------------------
  // Pure helpers (no DOM, unit-tested)
  // ---------------------------------------------------------------------------

  /** Base64-encode a UTF-8 string (btoa alone throws on non-ASCII). */
  function b64encodeUtf8(str) {
    var text = str === null || str === undefined ? "" : String(str);
    var bytes = new TextEncoder().encode(text);
    var bin = "";
    for (var i = 0; i < bytes.length; i++) bin += String.fromCharCode(bytes[i]);
    return btoa(bin);
  }

  /** Base64-decode to a UTF-8 string. Returns null on invalid base64. */
  function b64decodeUtf8(b64) {
    if (b64 === null || b64 === undefined || b64 === "") return "";
    var bin;
    try {
      bin = atob(b64);
    } catch (e) {
      return null;
    }
    var bytes = new Uint8Array(bin.length);
    for (var i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    return new TextDecoder("utf-8").decode(bytes);
  }

  /** Build the websocket URL from a Location-like object (hostname + port). */
  function wsUrl(loc) {
    var proto = loc && loc.protocol === "https:" ? "wss:" : "ws:";
    return proto + "//" + (loc && loc.hostname ? loc.hostname : "") + ":" + WS_PORT;
  }

  /**
   * Parse one websocket frame. Returns { namespace, type, data } where `data`
   * is the decoded inner JSON object (or null). Returns null when the outer
   * frame is not valid JSON. A malformed inner payload yields data === null
   * rather than throwing.
   */
  function parseRobotFrame(raw) {
    var outer;
    try {
      outer = JSON.parse(raw);
    } catch (e) {
      return null;
    }
    if (!outer || typeof outer !== "object") return null;
    var data = null;
    if (outer.payload && typeof outer.payload === "object") {
      data = outer.payload; // tolerate an already-decoded object payload
    } else if (typeof outer.payload === "string" && outer.payload.length > 0) {
      var text = b64decodeUtf8(outer.payload);
      if (text !== null && text !== "") {
        try {
          data = JSON.parse(text);
        } catch (e) {
          data = null;
        }
      }
    }
    return {
      namespace: typeof outer.namespace === "string" ? outer.namespace : null,
      type: typeof outer.type === "string" ? outer.type : null,
      data: data && typeof data === "object" ? data : null,
    };
  }

  /** Human label for the robot-reported opmode state. */
  function humanState(state) {
    switch (state) {
      case "INIT": return "Init";
      case "LOOPING": return "Running";
      case "STOPPED": return "Stopped";
      default: return "Idle";
    }
  }

  /** Coarse tone used for badge styling/attributes: idle | init | run. */
  function stateTone(state) {
    if (state === "LOOPING") return "run";
    if (state === "INIT") return "init";
    return "idle";
  }

  /** Exponential reconnect delay in ms, capped. attempt is 0-based. */
  function reconnectDelay(attempt) {
    var n = Math.max(0, attempt | 0);
    var delay = RECONNECT_BASE_MS * Math.pow(2, Math.min(n, 20));
    return Math.min(delay, RECONNECT_MAX_MS);
  }

  /**
   * Decide which control buttons are enabled. Pure so the safety rules are
   * directly testable.
   *   INIT  : connected, a selection exists, not currently LOOPING, no pending cmd
   *   START : connected, robot is initialized (state INIT), no pending cmd
   *   STOP  : connected, an opmode is active (state INIT or LOOPING), no pending cmd
   */
  function controlsFor(input) {
    var connected = !!(input && input.connected);
    var state = input ? input.state : null;
    var selection = input && input.selection ? String(input.selection) : "";
    var pending = !!(input && input.pending);
    var canAct = connected && !pending;
    return {
      init: canAct && selection !== "" && state !== "LOOPING",
      start: canAct && state === "INIT",
      stop: canAct && (state === "INIT" || state === "LOOPING"),
    };
  }

  /** Replace the whole telemetry map from a telemetryDump {entries} object. */
  function mergeTelemetryDump(entries) {
    var next = new Map();
    if (entries && typeof entries === "object") {
      for (var key of Object.keys(entries)) next.set(key, String(entries[key]));
    }
    return next;
  }

  /** Immutably set one telemetry key (type "telemetry"). */
  function applyTelemetryEntry(prev, key, value) {
    var next = new Map(prev || []);
    if (key === null || key === undefined || key === "") return next;
    next.set(String(key), String(value));
    return next;
  }

  /** Camera liveness + retry decision. Pure, so retry cadence is testable. */
  function cameraDecision(input) {
    var naturalWidth = input ? input.naturalWidth : 0;
    var live = typeof naturalWidth === "number" && naturalWidth > 0;
    var elapsed = (input && input.nowMs ? input.nowMs : 0) - (input && input.lastAttemptMs ? input.lastAttemptMs : 0);
    var cooldown = input && input.cooldownMs ? input.cooldownMs : CAMERA_COOLDOWN_MS;
    return { live: live, retry: !live && elapsed >= cooldown };
  }

  var core = {
    MAX_LOG_LINES: MAX_LOG_LINES,
    ELEMENT_IDS: ELEMENT_IDS,
    ROBOT_NAMESPACE: ROBOT_NAMESPACE,
    CAMERA_URL: CAMERA_URL,
    b64encodeUtf8: b64encodeUtf8,
    b64decodeUtf8: b64decodeUtf8,
    wsUrl: wsUrl,
    parseRobotFrame: parseRobotFrame,
    humanState: humanState,
    stateTone: stateTone,
    reconnectDelay: reconnectDelay,
    controlsFor: controlsFor,
    mergeTelemetryDump: mergeTelemetryDump,
    applyTelemetryEntry: applyTelemetryEntry,
    cameraDecision: cameraDecision,
  };

  // ---------------------------------------------------------------------------
  // DOM layer — only active in a browser
  // ---------------------------------------------------------------------------

  // Class contract from app.css section "JS CONTRACT": a base `badge` plus a
  // state modifier for #conn and #opstate.
  var CONN = {
    on: "badge badge--on",
    off: "badge badge--off",
    wait: "badge badge--wait",
  };
  var STATE_BADGE = {
    INIT: "badge badge--init",
    LOOPING: "badge badge--loop",
    none: "badge badge--none",
  };

  core.start = function start() {
    if (typeof document === "undefined") return;

    var els = {};
    for (var id of ELEMENT_IDS) els[id] = document.getElementById(id);

    // ---- state ----
    var ws = null;
    var connected = false;
    var opState = null;
    var telemetry = new Map();
    var telemetryRows = new Map();
    var sparkData = new Map();
    var pending = false;
    var reconnectAttempt = 0;
    var reconnectTimer = null;
    var pendingTimer = null;
    var logPinned = true;
    var logPaused = false;
    var camLastAttemptMs = 0;
    var camWatchdog = null;

    function el(tag, cls, text) {
      var e = document.createElement(tag);
      if (cls) e.className = cls;
      if (text !== undefined && text !== null) e.textContent = text;
      return e;
    }

    // ---- controls ----
    function updateControls() {
      var c = controlsFor({
        connected: connected,
        state: opState,
        selection: els.opmode ? els.opmode.value : "",
        pending: pending,
      });
      if (els.btnInit) els.btnInit.disabled = !c.init;
      if (els.btnStart) els.btnStart.disabled = !c.start;
      if (els.btnStop) els.btnStop.disabled = !c.stop;
    }

    function setConnected(next) {
      connected = next;
      if (els.conn) {
        els.conn.textContent = next ? "Connected" : "Disconnected";
        els.conn.className = next ? CONN.on : CONN.off;
        els.conn.setAttribute("data-state", next ? "on" : "off");
      }
      if (els.banner) els.banner.classList.toggle("hidden", next);
      updateControls();
    }

    function setStatus(text, tone) {
      if (els.conn) {
        els.conn.textContent = text;
        els.conn.className = tone === "wait" ? CONN.wait : CONN.off;
        els.conn.setAttribute("data-state", tone || "off");
      }
    }

    function setActive(name, state) {
      if (els.active) els.active.textContent = name ? String(name) : "None";
      if (els.opstate) {
        els.opstate.textContent = humanState(state);
        els.opstate.className = STATE_BADGE[state] || STATE_BADGE.none;
        els.opstate.setAttribute("data-state", stateTone(state));
      }
    }

    function setPending(next) {
      pending = next;
      if (pendingTimer) { clearTimeout(pendingTimer); pendingTimer = null; }
      if (pending) {
        pendingTimer = setTimeout(function () {
          pending = false;
          pendingTimer = null;
          updateControls();
        }, PENDING_TIMEOUT_MS);
      }
      updateControls();
    }

    // ---- opmode list ----
    function setOpmodes(names, active, state) {
      if (!els.opmode) return;
      var prev = els.opmode.value;
      var list = Array.isArray(names) ? names : [];
      els.opmode.textContent = "";
      if (list.length === 0) {
        els.opmode.append(el("option", "", "No opmodes"));
        els.opmode.disabled = true;
      } else {
        for (var n of list) els.opmode.append(el("option", "", n));
        els.opmode.disabled = false;
        if (list.indexOf(prev) !== -1) els.opmode.value = prev;
        else if (active && list.indexOf(active) !== -1) els.opmode.value = active;
      }
      setActive(active, state);
      updateControls();
    }

    // ---- battery sparkline ----
    function pushSpark(canvas, v, color) {
      if (!canvas || typeof canvas.getContext !== "function") return;
      var h = sparkData.get(canvas);
      if (!h) { h = []; sparkData.set(canvas, h); }
      h.push(v);
      if (h.length > SPARK_MAX) h.shift();
      var ctx = canvas.getContext("2d");
      if (!ctx) return;
      var w = canvas.width, hh = canvas.height;
      ctx.clearRect(0, 0, w, hh);
      if (h.length < 2) return;
      var min = Math.min.apply(null, h);
      var max = Math.max.apply(null, h);
      if (max - min < 1e-9) { min -= 1; max += 1; }
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      h.forEach(function (p, i) {
        var x = (i / (h.length - 1)) * w;
        var y = hh - 2 - ((p - min) / (max - min)) * (hh - 4);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.stroke();
    }

    function setBattery(v) {
      var n = v === null || v === undefined ? NaN : Number(v);
      if (els.battery) els.battery.textContent = isFinite(n) ? n.toFixed(2) : "n/a";
      if (isFinite(n) && els.batterySpark) pushSpark(els.batterySpark, n, "#facc15");
    }

    // ---- tables ----
    function setMotors(motors) {
      if (!els.motors) return;
      var list = Array.isArray(motors) ? motors : [];
      els.motors.textContent = "";
      if (els.motorsEmpty) els.motorsEmpty.classList.toggle("hidden", list.length > 0);
      for (var m of list) {
        var tr = el("tr", "");
        var name = el("th", "", m && m.name);
        name.scope = "row";
        tr.append(name);
        tr.append(el("td", "",
          m && m.power !== null && m.power !== undefined ? Number(m.power).toFixed(2) : "n/a"));
        tr.append(el("td", "",
          m && m.position !== null && m.position !== undefined ? String(m.position) : "n/a"));
        tr.append(el("td", "",
          m && m.current !== null && m.current !== undefined ? String(Math.round(m.current)) : "n/a"));
        var busy = el("td", "");
        var dot = el("span", "dot " + (m && m.busy ? "dot--busy" : "dot--idle"));
        dot.title = m && m.busy ? "Busy" : "Idle";
        busy.append(dot);
        tr.append(busy);
        els.motors.append(tr);
      }
    }

    function setServos(servos) {
      if (!els.servos) return;
      var list = Array.isArray(servos) ? servos : [];
      els.servos.textContent = "";
      if (els.servosEmpty) els.servosEmpty.classList.toggle("hidden", list.length > 0);
      for (var s of list) {
        var tr = el("tr", "");
        var name = el("th", "", s && s.name);
        name.scope = "row";
        tr.append(name);
        tr.append(el("td", "",
          s && s.position !== null && s.position !== undefined ? Number(s.position).toFixed(2) : "n/a"));
        els.servos.append(tr);
      }
    }

    function renderTelemetry() {
      if (!els.telemetry) return;
      // Drop rows whose keys are gone, add/update the rest. Ordered by key.
      var keys = Array.from(telemetry.keys()).sort();
      for (var oldKey of Array.from(telemetryRows.keys())) {
        if (!telemetry.has(oldKey)) {
          var r = telemetryRows.get(oldKey);
          if (r && r.tr) r.tr.remove();
          telemetryRows.delete(oldKey);
        }
      }
      els.telemetry.textContent = "";
      for (var key of keys) {
        var row = telemetryRows.get(key);
        if (!row) {
          var tr = el("tr", "");
          var k = el("th", "", key);
          k.scope = "row";
          var td = el("td", "");
          var wrap = el("span", "tvalwrap");
          var v = el("span", "tval");
          wrap.append(v);
          td.append(wrap);
          tr.append(k, td);
          row = { tr: tr, v: v, wrap: wrap, spark: null };
          telemetryRows.set(key, row);
        }
        var value = telemetry.get(key);
        row.v.textContent = value === undefined ? "" : String(value);
        var n = Number(value);
        if (value !== "" && isFinite(n) && els.telemetry) {
          if (!row.spark) {
            row.spark = el("canvas", "tval__spark");
            row.spark.width = 64;
            row.spark.height = 16;
            row.spark.setAttribute("aria-hidden", "true");
            row.wrap.append(row.spark);
          }
          pushSpark(row.spark, n, "#a1a1aa");
        }
        els.telemetry.append(row.tr);
      }
      if (els.telemetryEmpty) els.telemetryEmpty.classList.toggle("hidden", telemetryRows.size > 0);
    }

    // ---- log ----
    function addLog(text, isError) {
      if (!els.log) return;
      els.log.append(el("div", isError ? "log-line--error" : "", text));
      while (els.log.childElementCount > MAX_LOG_LINES) {
        if (els.log.firstChild) els.log.firstChild.remove(); else break;
      }
      if (logPinned && !logPaused) els.log.scrollTop = els.log.scrollHeight;
    }

    // ---- outbound ----
    function sendCommand(type, payloadStr) {
      if (!ws || ws.readyState !== 1 /* OPEN */) return false;
      var message = {
        namespace: ROBOT_NAMESPACE,
        type: type,
        payload: b64encodeUtf8(payloadStr === null || payloadStr === undefined ? "" : payloadStr),
      };
      try {
        ws.send(JSON.stringify(message));
      } catch (e) {
        return false;
      }
      return true;
    }

    function sendControl(type, payloadStr) {
      if (!sendCommand(type, payloadStr)) return;
      setPending(true);
    }

    // ---- inbound dispatch ----
    function onMessage(raw) {
      var frame = parseRobotFrame(raw);
      if (!frame) return;
      var data = frame.data;
      var kind = (data && typeof data.type === "string") ? data.type : frame.type;
      switch (kind) {
        case "state":
          opState = data && data.opmodeState ? data.opmodeState : null;
          setPending(false);
          setOpmodes(data && data.opmodes, data && data.active, opState);
          break;
        case "diag":
          if (data) {
            opState = data.opmodeState ? data.opmodeState : null;
            setPending(false);
            setBattery(data.battery);
            setMotors(data.motors);
            setServos(data.servos);
            setActive(data.active, opState);
          }
          break;
        case "telemetry":
          if (data) {
            telemetry = applyTelemetryEntry(telemetry, data.key, data.value);
            renderTelemetry();
          }
          break;
        case "telemetryDump":
          if (data && data.entries) {
            telemetry = mergeTelemetryDump(data.entries);
            renderTelemetry();
          }
          break;
        case "log":
          addLog(data ? String(data.line === undefined ? "" : data.line) : "", false);
          break;
        case "error":
          addLog("Error: " + (data && data.message !== undefined ? String(data.message) : "unknown"), true);
          break;
        default:
          break;
      }
    }

    // ---- connection lifecycle ----
    function clearReconnect() {
      if (reconnectTimer) { clearTimeout(reconnectTimer); reconnectTimer = null; }
    }

    function scheduleReconnect() {
      clearReconnect();
      var delay = reconnectDelay(reconnectAttempt);
      reconnectAttempt += 1;
      reconnectTimer = setTimeout(connect, delay);
    }

    function teardownSocket() {
      if (!ws) return;
      try {
        ws.onopen = null;
        ws.onmessage = null;
        ws.onclose = null;
        ws.onerror = null;
      } catch (e) { /* ignore */ }
      try { ws.close(); } catch (e) { /* ignore */ }
      ws = null;
    }

    function connect() {
      if (typeof WebSocket === "undefined") return;
      if (ws && (ws.readyState === 0 /* CONNECTING */ || ws.readyState === 1 /* OPEN */)) return;
      clearReconnect();
      teardownSocket();
      setConnected(false);
      setStatus("Connecting", "wait");
      var socket;
      try {
        socket = new WebSocket(wsUrl(window.location));
      } catch (e) {
        scheduleReconnect();
        return;
      }
      ws = socket;
      socket.onopen = function () {
        if (ws !== socket) return;
        reconnectAttempt = 0;
        clearReconnect();
        setConnected(true);
        // SDK protocol: the subscribe payload is the RAW namespace name, not base64.
        try {
          socket.send(JSON.stringify({ namespace: "system", type: "subscribeToNamespace", payload: ROBOT_NAMESPACE }));
        } catch (e) { /* close will follow */ }
      };
      socket.onmessage = function (ev) {
        if (ws !== socket) return;
        onMessage(ev && ev.data);
      };
      socket.onerror = function () { /* onclose handles retry */ };
      socket.onclose = function () {
        if (ws === socket) ws = null;
        setConnected(false);
        scheduleReconnect();
      };
    }

    // ---- camera ----
    function reloadCamera() {
      if (!els.cam) return;
      camLastAttemptMs = Date.now();
      try { els.cam.removeAttribute("src"); } catch (e) { /* ignore */ }
      els.cam.src = CAMERA_URL;
    }

    function tickCamera() {
      if (!els.cam) return;
      var decision = cameraDecision({
        naturalWidth: els.cam.naturalWidth,
        nowMs: Date.now(),
        lastAttemptMs: camLastAttemptMs,
        cooldownMs: CAMERA_COOLDOWN_MS,
      });
      els.cam.classList.toggle("hidden", !decision.live);
      if (els.camEmpty) els.camEmpty.classList.toggle("hidden", decision.live);
      if (decision.retry) reloadCamera();
    }

    // ---- wiring ----
    if (els.log) {
      els.log.addEventListener("scroll", function () {
        var l = els.log;
        logPinned = l.scrollTop + l.clientHeight >= l.scrollHeight - 8;
      });
    }
    if (els.logPause) {
      els.logPause.addEventListener("click", function () {
        logPaused = !logPaused;
        els.logPause.textContent = logPaused ? "Resume" : "Pause";
      });
    }
    if (els.logClear) {
      els.logClear.addEventListener("click", function () { if (els.log) els.log.textContent = ""; });
    }
    if (els.logCopy) {
      els.logCopy.addEventListener("click", function () {
        var text = els.log ? Array.prototype.map.call(els.log.children, function (d) { return d.textContent; }).join("\n") : "";
        if (navigator.clipboard && navigator.clipboard.writeText) {
          navigator.clipboard.writeText(text).catch(function () { downloadLog(text); });
        } else {
          downloadLog(text);
        }
      });
    }
    if (els.opmode) {
      els.opmode.addEventListener("change", updateControls);
    }
    if (els.btnInit) {
      els.btnInit.addEventListener("click", function () {
        if (els.btnInit.disabled) return;
        sendControl("initOpMode", els.opmode ? els.opmode.value : "");
      });
    }
    if (els.btnStart) {
      els.btnStart.addEventListener("click", function () {
        if (els.btnStart.disabled) return;
        sendControl("startOpMode", "");
      });
    }
    if (els.btnStop) {
      els.btnStop.addEventListener("click", function () {
        if (els.btnStop.disabled) return;
        sendControl("stopOpMode", "");
      });
    }
    window.addEventListener("beforeunload", function () {
      clearReconnect();
      if (pendingTimer) clearTimeout(pendingTimer);
      if (camWatchdog) clearInterval(camWatchdog);
    });

    function downloadLog(text) {
      var a = el("a");
      a.href = URL.createObjectURL(new Blob([text], { type: "text/plain" }));
      a.download = "driver-station.log";
      a.click();
      URL.revokeObjectURL(a.href);
    }

    // ---- boot ----
    updateControls();
    connect();
    // Let the initial <img src> stream have a full cooldown before we consider
    // it dead and force a reload.
    camLastAttemptMs = Date.now();
    camWatchdog = setInterval(tickCamera, 1000);
  };

  return core;
});
