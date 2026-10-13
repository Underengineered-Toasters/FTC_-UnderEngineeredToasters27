"use strict";
/*
 * Unit + invariant tests for the driver station client.
 *
 * Zero dependencies: run with the Node test runner (Node 18+).
 *   node --test TeamCode/ds-ui/tests/app.test.js
 *
 * The protocol expectations here are taken directly from the Java side
 * (DsSocketHandler.java / DriverStation.java / DsTelemetry.java): the robot
 * base64-encodes its JSON payloads, the browser subscribes with the system
 * type "subscribeToNamespace" and the RAW namespace name "ds".
 */
const test = require("node:test");
const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");

const APP_PATH = path.join(__dirname, "..", "..", "src", "main", "assets", "ds", "app.js");
const APP_SRC = fs.readFileSync(APP_PATH, "utf8");
const core = require(APP_PATH);
const CSS_PATH = path.join(__dirname, "..", "..", "src", "main", "assets", "ds", "app.css");
const CSS_SRC = fs.existsSync(CSS_PATH) ? fs.readFileSync(CSS_PATH, "utf8") : "";

// Non-ASCII samples built without escape sequences.
const OMEGA = String.fromCharCode(0x3a9);   // Greek capital omega
const EACUTE = String.fromCharCode(0xe9);   // e with acute accent
const EMDASH = String.fromCharCode(0x2014); // em dash
const EMOJI = String.fromCodePoint(0x1f600);

// Mirror DriverStation.encodePayload (Base64 NO_WRAP of UTF-8 bytes) so tests
// build frames exactly the way the robot does.
function robotPayload(objOrString) {
  const text = typeof objOrString === "string" ? objOrString : JSON.stringify(objOrString);
  return core.b64encodeUtf8(text);
}

function robotFrame(type, objOrString, namespace) {
  return JSON.stringify({
    namespace: namespace === undefined ? "ds" : namespace,
    type,
    payload: robotPayload(objOrString),
  });
}

// ---------------------------------------------------------------------------
// base64 / UTF-8
// ---------------------------------------------------------------------------

test("base64 UTF-8 round-trips ASCII, accented and emoji text", () => {
  const samples = ["", "abc", "Omega" + OMEGA + "pMode", "caf" + EACUTE, EMOJI + " robot " + EMDASH];
  for (const s of samples) {
    assert.equal(core.b64decodeUtf8(core.b64encodeUtf8(s)), s);
  }
});

test("b64decodeUtf8 returns null on invalid base64 and '' for empty", () => {
  assert.equal(core.b64decodeUtf8(""), "");
  assert.equal(core.b64decodeUtf8(null), "");
  assert.equal(core.b64decodeUtf8("!!!not-base64!!!"), null);
});

test("empty command payload encodes to an empty string (start/stop)", () => {
  assert.equal(core.b64encodeUtf8(""), "");
});

// ---------------------------------------------------------------------------
// ws url
// ---------------------------------------------------------------------------

test("wsUrl uses wss on TLS pages and the SDK websocket port", () => {
  assert.equal(core.wsUrl({ protocol: "http:", hostname: "192.168.43.1" }), "ws://192.168.43.1:8081");
  assert.equal(core.wsUrl({ protocol: "https:", hostname: "192.168.43.1" }), "wss://192.168.43.1:8081");
});

// ---------------------------------------------------------------------------
// inbound frame decoding
// ---------------------------------------------------------------------------

test("parseRobotFrame decodes a state frame", () => {
  const frame = core.parseRobotFrame(robotFrame("state", {
    opmodes: ["RobotTeleOp", "Auto"], active: "RobotTeleOp", opmodeState: "INIT",
  }));
  assert.equal(frame.namespace, "ds");
  assert.equal(frame.type, "state");
  assert.deepEqual(frame.data.opmodes, ["RobotTeleOp", "Auto"]);
  assert.equal(frame.data.active, "RobotTeleOp");
  assert.equal(frame.data.opmodeState, "INIT");
});

test("parseRobotFrame decodes a diag frame with motors and servos", () => {
  const frame = core.parseRobotFrame(robotFrame("diag", {
    battery: 12.7,
    motors: [{ name: "leftFront", power: 0.5, position: 100, current: 1200, busy: true }],
    servos: [{ name: "claw", position: 0.4 }],
    active: "RobotTeleOp",
    opmodeState: "LOOPING",
  }));
  assert.equal(frame.data.battery, 12.7);
  assert.equal(frame.data.motors[0].name, "leftFront");
  assert.equal(frame.data.servos[0].position, 0.4);
});

test("parseRobotFrame decodes a single telemetry pair", () => {
  const frame = core.parseRobotFrame(robotFrame("telemetry", { key: "Heading", value: "42.0" }));
  assert.equal(frame.type, "telemetry");
  assert.equal(frame.data.key, "Heading");
  assert.equal(frame.data.value, "42.0");
});

test("parseRobotFrame decodes a telemetryDump", () => {
  const frame = core.parseRobotFrame(robotFrame("telemetryDump", { entries: { A: "1", B: "two" } }));
  assert.equal(frame.type, "telemetryDump");
  assert.deepEqual(frame.data.entries, { A: "1", B: "two" });
});

test("parseRobotFrame decodes log and error frames", () => {
  const log = core.parseRobotFrame(robotFrame("log", { type: "log", line: "hello" }));
  assert.equal(log.data.line, "hello");
  const err = core.parseRobotFrame(robotFrame("error", { type: "error", message: "bad opmode" }));
  assert.equal(err.data.message, "bad opmode");
});

test("parseRobotFrame tolerates non-ASCII opmode names", () => {
  const name = OMEGA + "pMode";
  const frame = core.parseRobotFrame(robotFrame("state", {
    opmodes: [name], active: null, opmodeState: null,
  }));
  assert.deepEqual(frame.data.opmodes, [name]);
});

test("parseRobotFrame returns null for a malformed outer envelope", () => {
  assert.equal(core.parseRobotFrame("not json"), null);
  assert.equal(core.parseRobotFrame(""), null);
  assert.equal(core.parseRobotFrame("42"), null);
  assert.equal(core.parseRobotFrame(null), null);
});

test("parseRobotFrame keeps the frame but nulls data for a broken payload", () => {
  const bad64 = core.parseRobotFrame(JSON.stringify({ namespace: "ds", type: "log", payload: "###" }));
  assert.equal(bad64.type, "log");
  assert.equal(bad64.data, null);

  const notJson = core.parseRobotFrame(JSON.stringify({
    namespace: "ds", type: "log", payload: core.b64encodeUtf8("not json"),
  }));
  assert.equal(notJson.data, null);
});

test("parseRobotFrame accepts an already-decoded object payload", () => {
  const frame = core.parseRobotFrame(JSON.stringify({ namespace: "ds", type: "log", payload: { line: "x" } }));
  assert.equal(frame.data.line, "x");
});

// ---------------------------------------------------------------------------
// outbound command encoding (mirrors DsSocketHandler.decodePayload)
// ---------------------------------------------------------------------------

function buildCommand(type, payloadStr) {
  return { namespace: "ds", type, payload: core.b64encodeUtf8(payloadStr || "") };
}

test("initOpMode command decodes back to the opmode name on the robot", () => {
  const msg = buildCommand("initOpMode", "RobotTeleOp");
  assert.equal(msg.namespace, "ds");
  assert.equal(msg.type, "initOpMode");
  // Robot side: new String(Base64.decode(payload, DEFAULT), UTF_8)
  assert.equal(core.b64decodeUtf8(msg.payload), "RobotTeleOp");
});

test("start/stop commands carry an empty payload the robot reads as ''", () => {
  for (const type of ["startOpMode", "stopOpMode"]) {
    const msg = buildCommand(type, "");
    assert.equal(msg.payload, "");
    assert.equal(core.b64decodeUtf8(msg.payload), "");
  }
});

// ---------------------------------------------------------------------------
// state labels + safety gating
// ---------------------------------------------------------------------------

test("humanState maps robot states to explicit labels", () => {
  assert.equal(core.humanState("INIT"), "Init");
  assert.equal(core.humanState("LOOPING"), "Running");
  assert.equal(core.humanState("STOPPED"), "Stopped");
  assert.equal(core.humanState(null), "Idle");
  assert.equal(core.humanState(undefined), "Idle");
  assert.equal(core.humanState("garbage"), "Idle");
});

test("stateTone distinguishes idle / init / run", () => {
  assert.equal(core.stateTone("LOOPING"), "run");
  assert.equal(core.stateTone("INIT"), "init");
  assert.equal(core.stateTone(null), "idle");
});

test("controlsFor: everything disabled while disconnected", () => {
  for (const state of [null, "INIT", "LOOPING"]) {
    const c = core.controlsFor({ connected: false, state, selection: "X" });
    assert.deepEqual(c, { init: false, start: false, stop: false });
  }
});

test("controlsFor: INIT requires a selection and forbids LOOPING", () => {
  assert.equal(core.controlsFor({ connected: true, state: null, selection: "X" }).init, true);
  assert.equal(core.controlsFor({ connected: true, state: "INIT", selection: "X" }).init, true);
  assert.equal(core.controlsFor({ connected: true, state: null, selection: "" }).init, false);
  assert.equal(core.controlsFor({ connected: true, state: "LOOPING", selection: "X" }).init, false);
});

test("controlsFor: START only when the robot reports INIT", () => {
  assert.equal(core.controlsFor({ connected: true, state: "INIT", selection: "X" }).start, true);
  assert.equal(core.controlsFor({ connected: true, state: null, selection: "X" }).start, false);
  assert.equal(core.controlsFor({ connected: true, state: "LOOPING", selection: "X" }).start, false);
});

test("controlsFor: STOP only when an opmode is active", () => {
  assert.equal(core.controlsFor({ connected: true, state: "INIT", selection: "" }).stop, true);
  assert.equal(core.controlsFor({ connected: true, state: "LOOPING", selection: "X" }).stop, true);
  assert.equal(core.controlsFor({ connected: true, state: null, selection: "X" }).stop, false);
});

test("controlsFor: a pending command disables every control", () => {
  const c = core.controlsFor({ connected: true, state: "INIT", selection: "X", pending: true });
  assert.deepEqual(c, { init: false, start: false, stop: false });
});

// ---------------------------------------------------------------------------
// reconnect backoff
// ---------------------------------------------------------------------------

test("reconnectDelay grows exponentially and stays capped", () => {
  assert.equal(core.reconnectDelay(0), 1000);
  assert.equal(core.reconnectDelay(1), 2000);
  assert.equal(core.reconnectDelay(2), 4000);
  assert.equal(core.reconnectDelay(3), 8000);
  assert.equal(core.reconnectDelay(4), 10000);
  assert.equal(core.reconnectDelay(50), 10000);
  assert.equal(core.reconnectDelay(-3), 1000);
  for (let i = 0; i < 12; i++) {
    assert.ok(core.reconnectDelay(i + 1) >= core.reconnectDelay(i));
  }
});

// ---------------------------------------------------------------------------
// telemetry accumulation
// ---------------------------------------------------------------------------

test("mergeTelemetryDump replaces the whole set and drops missing keys", () => {
  let m = core.applyTelemetryEntry(new Map(), "Stale", "1");
  m = core.mergeTelemetryDump({ A: "1", B: "2" });
  assert.deepEqual([...m.keys()].sort(), ["A", "B"]);
  assert.equal(m.has("Stale"), false);

  m = core.mergeTelemetryDump({ A: "9" });
  assert.deepEqual([...m.entries()], [["A", "9"]]);
});

test("applyTelemetryEntry adds/updates a key without dropping others", () => {
  let m = core.mergeTelemetryDump({ A: "1" });
  m = core.applyTelemetryEntry(m, "B", 2);
  assert.equal(m.get("A"), "1");
  assert.equal(m.get("B"), "2");
  m = core.applyTelemetryEntry(m, "A", "7");
  assert.equal(m.get("A"), "7");
  // Does not mutate the previous map.
  assert.equal(core.applyTelemetryEntry(new Map(), "k", "v").get("k"), "v");
});

test("applyTelemetryEntry ignores empty keys and stringifies values", () => {
  const m = core.applyTelemetryEntry(new Map(), "", "x");
  assert.equal(m.size, 0);
  assert.equal(core.applyTelemetryEntry(new Map(), "n", 3.5).get("n"), "3.5");
  assert.equal(core.applyTelemetryEntry(new Map(), "b", null).get("b"), "null");
});

// ---------------------------------------------------------------------------
// camera retry
// ---------------------------------------------------------------------------

test("cameraDecision reports live once frames arrive", () => {
  const d = core.cameraDecision({ naturalWidth: 320, nowMs: 10000, lastAttemptMs: 0, cooldownMs: 4000 });
  assert.equal(d.live, true);
  assert.equal(d.retry, false);
});

test("cameraDecision retries only after the cooldown when dead", () => {
  const early = core.cameraDecision({ naturalWidth: 0, nowMs: 2000, lastAttemptMs: 0, cooldownMs: 4000 });
  assert.equal(early.live, false);
  assert.equal(early.retry, false);
  const due = core.cameraDecision({ naturalWidth: 0, nowMs: 5000, lastAttemptMs: 0, cooldownMs: 4000 });
  assert.equal(due.retry, true);
});

// ---------------------------------------------------------------------------
// source invariants: retained ids, no HTML injection, no external deps
// ---------------------------------------------------------------------------

const RETAINED_IDS = [
  "banner", "conn", "opmode", "opstate", "active", "battery", "motors", "motorsEmpty",
  "servos", "servosEmpty", "telemetry", "telemetryEmpty", "log", "logPause", "logClear",
  "logCopy", "cam", "camEmpty", "btnInit", "btnStart", "btnStop",
];

test("app.js references every retained element id", () => {
  for (const id of RETAINED_IDS) {
    assert.ok(core.ELEMENT_IDS.includes(id), `ELEMENT_IDS missing ${id}`);
    assert.ok(APP_SRC.includes('"' + id + '"'), `app.js missing id literal ${id}`);
  }
});

test("app.js never injects robot text as HTML", () => {
  assert.equal(/\.innerHTML\s*=/.test(APP_SRC), false, "assigns innerHTML");
  assert.equal(/insertAdjacentHTML/.test(APP_SRC), false, "uses insertAdjacentHTML");
  assert.equal(/\.outerHTML\s*=/.test(APP_SRC), false, "assigns outerHTML");
  assert.ok(APP_SRC.includes("textContent"), "expected textContent usage");
});

test("app.js has a single camera watchdog interval (no duplicate timers)", () => {
  assert.equal((APP_SRC.match(/setInterval\(/g) || []).length, 1);
});

test("app.js subscribes with the raw ds namespace, not base64", () => {
  assert.ok(APP_SRC.includes("subscribeToNamespace"));
  assert.ok(APP_SRC.includes("payload: ROBOT_NAMESPACE"));
  assert.equal(core.ROBOT_NAMESPACE, "ds");
});

test("app.js handles every documented message type", () => {
  for (const kind of ["state", "diag", "telemetry", "telemetryDump", "log", "error"]) {
    assert.ok(APP_SRC.includes('case "' + kind + '":'), `missing handler for ${kind}`);
  }
});

test("app.js sends the documented command types", () => {
  for (const cmd of ["initOpMode", "startOpMode", "stopOpMode"]) {
    assert.ok(APP_SRC.includes('"' + cmd + '"'), `missing command ${cmd}`);
  }
});

test("app.js has no external network origin beyond the robot and camera", () => {
  assert.equal(/https?:\/\//.test(APP_SRC), false, "found an absolute http(s) URL");
  assert.ok(APP_SRC.includes('"/ds/camera.mjpeg"'));
  assert.ok(APP_SRC.includes(":8081") || APP_SRC.includes("WS_PORT"));
});

test("app.js bounds the log buffer", () => {
  assert.ok(core.MAX_LOG_LINES > 0 && core.MAX_LOG_LINES <= 1000);
  assert.ok(APP_SRC.includes("MAX_LOG_LINES"));
});

test("app.js is a classic script that also exports its pure core to Node", () => {
  assert.equal(typeof core.start, "function");
  for (const fn of ["b64encodeUtf8", "b64decodeUtf8", "parseRobotFrame", "humanState",
    "stateTone", "controlsFor", "reconnectDelay", "mergeTelemetryDump",
    "applyTelemetryEntry", "cameraDecision", "wsUrl"]) {
    assert.equal(typeof core[fn], "function", `missing export ${fn}`);
  }
});

test("app.js uses the app.css JS-contract class names", () => {
  const inJs = [
    "badge badge--on", "badge badge--off", "badge badge--wait",
    "badge badge--init", "badge badge--loop", "badge badge--none",
    "dot--busy", "dot--idle", "log-line--error", "tvalwrap", "tval__spark",
  ];
  for (const cls of inJs) {
    assert.ok(APP_SRC.includes(cls), `app.js missing class from the JS contract: ${cls}`);
  }
  // The stylesheet shipped alongside must define the same class names.
  for (const cls of ["badge--on", "badge--off", "badge--wait", "badge--init",
    "badge--loop", "badge--none", "dot--busy", "log-line--error", "tvalwrap"]) {
    assert.ok(CSS_SRC.includes("." + cls), `app.css missing .${cls}`);
  }
});