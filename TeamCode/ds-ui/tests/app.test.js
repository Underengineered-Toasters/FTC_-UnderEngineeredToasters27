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
const HELLIP = String.fromCharCode(0x2026); // horizontal ellipsis
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

test("controlsFor: a pending command disables INIT and START but never STOP", () => {
  // A command in flight must not be repeatable...
  assert.deepEqual(
    core.controlsFor({ connected: true, state: "INIT", selection: "X", pending: true }),
    { init: false, start: false, stop: true });
  assert.deepEqual(
    core.controlsFor({ connected: true, state: "LOOPING", selection: "X", pending: true }),
    { init: false, start: false, stop: true });
  // ...but with nothing active there is nothing to stop.
  assert.deepEqual(
    core.controlsFor({ connected: true, state: null, selection: "X", pending: true }),
    { init: false, start: false, stop: false });
});

// ---------------------------------------------------------------------------
// command guard release rule (mirrors DsSocketHandler + the diag poller)
// ---------------------------------------------------------------------------

test("commandSatisfied only accepts the snapshot the command produces", () => {
  const init = { type: "initOpMode", name: "RobotTeleOp" };
  assert.equal(core.commandSatisfied(init, "RobotTeleOp", "INIT"), true);
  assert.equal(core.commandSatisfied(init, "Auto", "INIT"), false);
  assert.equal(core.commandSatisfied(init, null, null), false);
  // A running opmode that is not the requested one must not release the guard.
  assert.equal(core.commandSatisfied(init, "Auto", "LOOPING"), false);
  // The requested name alone is not proof either: DriverStationServer reports
  // opmodeState as null until the opmode listener has recorded one
  // (onOpModePreInit -> "INIT"), so a name with no INIT state is a snapshot
  // that has not shown the init finished.
  assert.equal(core.commandSatisfied(init, "RobotTeleOp", null), false);
  assert.equal(core.commandSatisfied(init, "RobotTeleOp", "LOOPING"), false);

  const start = { type: "startOpMode" };
  assert.equal(core.commandSatisfied(start, "RobotTeleOp", "LOOPING"), true);
  assert.equal(core.commandSatisfied(start, "RobotTeleOp", "INIT"), false);
  assert.equal(core.commandSatisfied(start, null, null), false);

  const stop = { type: "stopOpMode" };
  assert.equal(core.commandSatisfied(stop, null, null), true);
  assert.equal(core.commandSatisfied(stop, "RobotTeleOp", "LOOPING"), false);

  // No command in flight is always "satisfied" (nothing to release).
  assert.equal(core.commandSatisfied(null, "RobotTeleOp", "LOOPING"), true);
  assert.equal(core.PENDING_TIMEOUT_MS > 0 && core.PENDING_TIMEOUT_MS <= 15000, true);
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
  "logCopy", "cam", "camEmpty", "btnInit", "btnStart", "btnStop", "batterySpark",
  "cmdStatus",
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
// ---------------------------------------------------------------------------
// Browser-boundary harness: fake DOM + fake WebSocket + fake clock
// ---------------------------------------------------------------------------
//
// app.js boots itself when a document exists, so the DOM layer can only be
// exercised by running the real file inside a vm context that provides
// browser-shaped globals, then driving it with real events: clicks, socket
// frames and timers. Everything below is simulated. There is no browser, no
// network and no robot in this file.

const vm = require("node:vm");

/** Virtual clock so backoff / watchdog / pending timers are deterministic. */
function makeClock() {
  let nextId = 1;
  let now = 0;
  const timers = new Map();
  return {
    now: () => now,
    setTimeout(fn, ms) {
      const id = nextId++;
      timers.set(id, { fn, at: now + (Number(ms) || 0), ms: Number(ms) || 0, repeat: false });
      return id;
    },
    clearTimeout(id) { timers.delete(id); },
    setInterval(fn, ms) {
      const id = nextId++;
      timers.set(id, { fn, at: now + (Number(ms) || 0), ms: Number(ms) || 0, repeat: true });
      return id;
    },
    clearInterval(id) { timers.delete(id); },
    /** Run every timer due within `ms` of the virtual clock, cascades included. */
    advance(ms) {
      const target = now + ms;
      for (let guard = 0; guard < 500; guard++) {
        const due = [...timers.entries()]
          .filter(([, t]) => t.at <= target)
          .sort((a, b) => a[1].at - b[1].at || a[0] - b[0])[0];
        if (!due) break;
        const [id, timer] = due;
        now = Math.max(now, timer.at);
        if (timer.repeat) {
          timer.at = now + timer.ms;
          timer.fn();
        } else {
          timers.delete(id);
          timer.fn();
        }
      }
      now = target;
    },
    tickIntervals(times) {
      for (const timer of [...timers.values()]) {
        if (!timer.repeat) continue;
        for (let i = 0; i < times; i++) timer.fn();
      }
    },
    intervalCount() { return [...timers.values()].filter((t) => t.repeat).length; },
    size() { return timers.size; },
  };
}

class FakeClassList {
  constructor(el) { this.el = el; }
  _list() { return String(this.el.className || "").split(/\s+/).filter(Boolean); }
  _set(list) { this.el.className = list.join(" "); }
  add(...names) {
    const list = this._list();
    for (const n of names) if (!list.includes(n)) list.push(n);
    this._set(list);
  }
  remove(...names) { this._set(this._list().filter((n) => !names.includes(n))); }
  contains(name) { return this._list().includes(name); }
  toggle(name, force) {
    const on = force === undefined ? !this.contains(name) : Boolean(force);
    if (on) this.add(name); else this.remove(name);
    return on;
  }
}

const makeFakeCtx = () => ({
  clears: 0,
  clearRect() { this.clears += 1; },
  beginPath() {}, moveTo() {}, lineTo() {}, stroke() {},
  strokeStyle: "", lineWidth: 0,
});

// A real <option> reports its value attribute when it has one and its text
// content otherwise. That distinction is the whole point of a placeholder
// option: `<option>No opmodes</option>` reports "No opmodes", which reads as a
// real opmode name, while `<option value="">No opmodes</option>` reports "".
const optionValue = (o) => ("value" in o.attributes ? o.attributes.value : o.textContent);

class FakeElement {
  constructor(tag) {
    this.tagName = String(tag).toUpperCase();
    this.children = [];
    this.parentNode = null;
    this.className = "";
    this.id = "";
    this.disabled = false;
    this.title = "";
    this.scope = "";
    this.src = "";
    this.width = 0;
    this.height = 0;
    this.naturalWidth = 0;
    this.scrollTop = 0;
    this.scrollHeight = 0;
    this.clientHeight = 0;
    this.attributes = Object.create(null);
    this.listeners = Object.create(null);
    this.classList = new FakeClassList(this);
    this.style = {};
    this._text = "";
    this._value = "";
  }

  // An action button is inert when either the native property or aria-disabled
  // says so. app.js only ever uses aria-disabled on its action buttons: a native
  // disabled button loses keyboard focus when it is pressed.
  get disabled() { return this._disabled || this.attributes["aria-disabled"] === "true"; }
  set disabled(value) { this._disabled = !!value; }
  get nativeDisabled() { return this._disabled; }
  select() {}
  click() {}
  get childElementCount() { return this.children.length; }
  get firstChild() { return this.children.length ? this.children[0] : null; }
  get textContent() {
    if (this.children.length) return this.children.map((c) => c.textContent).join("");
    return this._text;
  }
  set textContent(value) {
    this._text = String(value);
    for (const c of this.children) c.parentNode = null;
    this.children = [];
  }
  // A real <select> reports the first option when the assigned value matches
  // none of them; the INIT payload depends on that.
  get value() {
    if (this.tagName !== "SELECT") return this._value;
    const opts = this.children.filter((c) => c.tagName === "OPTION");
    if (!opts.length) return this._value;
    if (opts.some((o) => optionValue(o) === this._value)) return this._value;
    return optionValue(opts[0]);
  }
  set value(v) {
    this._value = String(v);
    // <option>.value reflects the value attribute in the real DOM.
    if (this.tagName === "OPTION") this.attributes.value = String(v);
  }
  append(...nodes) {
    for (const n of nodes) {
      if (!n || typeof n !== "object") continue;
      n.parentNode = this;
      this.children.push(n);
    }
  }
  appendChild(node) { this.append(node); return node; }
  remove() {
    if (!this.parentNode) return;
    const i = this.parentNode.children.indexOf(this);
    if (i >= 0) this.parentNode.children.splice(i, 1);
    this.parentNode = null;
  }
  setAttribute(name, value) { this.attributes[name] = String(value); }
  getAttribute(name) { return name in this.attributes ? this.attributes[name] : null; }
  removeAttribute(name) { delete this.attributes[name]; }
  addEventListener(type, fn) { (this.listeners[type] = this.listeners[type] || []).push(fn); }
  dispatchEvent(type, event) {
    for (const fn of (this.listeners[type] || []).slice()) fn(Object.assign({ type, target: this }, event));
  }
  getContext() {
    if (!this._ctx) this._ctx = makeFakeCtx();
    return this._ctx;
  }
  // Simple tag selector only ("span"): the app never uses anything richer.
  querySelector(selector) {
    const tag = String(selector).toUpperCase();
    const walk = (node) => {
      for (const child of node.children) {
        if (child.tagName === tag) return child;
        const found = walk(child);
        if (found) return found;
      }
      return null;
    };
    return walk(this);
  }
}

function makeFakeWebSocket() {
  const instances = [];
  class FakeWebSocket {
    constructor(url) {
      this.url = url;
      this.readyState = 0;
      this.sent = [];
      this.onopen = null;
      this.onmessage = null;
      this.onclose = null;
      this.onerror = null;
      instances.push(this);
    }
    send(data) {
      if (this.readyState !== 1) throw new Error("send() on a socket that is not OPEN");
      this.sent.push(String(data));
    }
    close() { this.readyState = 3; }
    // ---- test-side event sources ----
    open() { this.readyState = 1; if (this.onopen) this.onopen({ type: "open" }); }
    deliver(raw) { if (this.onmessage) this.onmessage({ data: raw }); }
    drop() { this.readyState = 3; if (this.onclose) this.onclose({ type: "close" }); }
  }
  FakeWebSocket.CONNECTING = 0;
  FakeWebSocket.OPEN = 1;
  FakeWebSocket.CLOSING = 2;
  FakeWebSocket.CLOSED = 3;
  FakeWebSocket.instances = instances;
  return FakeWebSocket;
}

/**
 * Boot the real app.js against browser-shaped globals. `noWebSocket: true`
 * omits the WebSocket constructor, which is what a browser without support
 * looks like to the client.
 */
function loadUI(options = {}) {
  const els = new Map();
  for (const id of RETAINED_IDS) {
    const tag = id === "opmode" ? "select" : id === "cam" ? "img" : id === "batterySpark" ? "canvas" : "div";
    const el = new FakeElement(tag);
    el.id = id;
    els.set(id, el);
  }
  // Mirror the initial states index.html ships.
  els.get("conn").className = "badge badge--wait";
  els.get("conn").textContent = "Connecting";
  els.get("opstate").className = "badge badge--none";
  els.get("opstate").textContent = "Idle";
  els.get("active").textContent = "None";
  els.get("battery").textContent = "n/a";
  els.get("banner").className = "banner hidden";
  // index.html ships the banner as an icon plus one text span.
  const bannerText = new FakeElement("span");
  bannerText.textContent = "Disconnected from the robot. Reconnecting" + HELLIP;
  els.get("banner").append(bannerText);
  // index.html ships the empty-list placeholder with an explicit empty value.
  const placeholder = new FakeElement("option");
  placeholder.textContent = "No opmodes";
  placeholder.value = "";
  els.get("opmode").append(placeholder);
  els.get("logPause").textContent = "Pause";

  const copies = [];
  const downloads = [];
  const document = {
    getElementById: (id) => els.get(id) || null,
    createElement: (tag) => new FakeElement(tag),
    body: new FakeElement("body"),
    // Record what a copy would have copied: the proxy textarea is the last child.
    execCommand: (command) => {
      const ta = document.body.children[document.body.children.length - 1];
      copies.push({ command, value: ta ? ta.value : null });
      return options.execCommandResult !== undefined ? options.execCommandResult : true;
    },
  };
  const clock = makeClock();
  const window = {
    location: { protocol: "http:", hostname: "192.168.43.1" },
    listeners: Object.create(null),
    addEventListener(type, fn) { (this.listeners[type] = this.listeners[type] || []).push(fn); },
    dispatch(type) { for (const fn of (this.listeners[type] || []).slice()) fn({ type }); },
  };
  const sandbox = {
    console,
    document,
    window,
    // Plain http on the robot network is not a secure context: no clipboard API.
    navigator: options.insecure ? {} : {
      clipboard: { writeText: () => (options.clipboardRejects ? Promise.reject(new Error("denied")) : Promise.resolve()) },
    },
    URL: { createObjectURL: () => { downloads.push(1); return "blob:test"; }, revokeObjectURL() {} },
    Blob: class { constructor(parts) { this.parts = parts; } },
    setTimeout: (fn, ms) => clock.setTimeout(fn, ms),
    clearTimeout: (id) => clock.clearTimeout(id),
    setInterval: (fn, ms) => clock.setInterval(fn, ms),
    clearInterval: (id) => clock.clearInterval(id),
    TextEncoder,
    TextDecoder,
    atob,
    btoa,
    module: { exports: {} },
  };
  if (!options.noWebSocket) sandbox.WebSocket = makeFakeWebSocket();
  vm.createContext(sandbox);
  vm.runInContext(APP_SRC, sandbox, { filename: "app.js" });
  return {
    ui: sandbox.module.exports,
    els,
    clock,
    window,
    sockets: sandbox.WebSocket ? sandbox.WebSocket.instances : [],
    copies,
    downloads,
  };
}

const logLines = (h) => h.els.get("log").children.map((c) => c.textContent);
const telemetryKeys = (h) => h.els.get("telemetry").children.map((tr) => tr.children[0].textContent);
const telemetryValue = (h, key) => {
  const row = h.els.get("telemetry").children.find((tr) => tr.children[0].textContent === key);
  return row ? row.children[1].textContent : null;
};
const isHidden = (el) => el.classList.contains("hidden");
const hasTag = (el, tag) => el.tagName === tag || el.children.some((c) => hasTag(c, tag));
const click = (h, id) => h.els.get(id).dispatchEvent("click");

/** Open a harness, connect, and hand back the live socket. */
function connectedUI(options) {
  const h = loadUI(options);
  const ws = h.sockets[0];
  ws.open();
  return { h, ws };
}

// ---------------------------------------------------------------------------
// harness: handshake + command encoding
// ---------------------------------------------------------------------------

test("[harness] boots, opens one socket and subscribes with the raw ds namespace", () => {
  const h = loadUI();
  assert.equal(h.sockets.length, 1, "exactly one socket on boot");
  assert.equal(h.sockets[0].url, "ws://192.168.43.1:8081");
  assert.equal(h.sockets[0].readyState, 0);
  assert.equal(h.els.get("conn").textContent, "Connecting");
  assert.equal(isHidden(h.els.get("banner")), false, "banner shows while not connected");

  h.sockets[0].open();
  assert.equal(h.els.get("conn").textContent, "Connected");
  assert.equal(h.els.get("conn").getAttribute("data-state"), "on");
  assert.equal(isHidden(h.els.get("banner")), true, "banner hides once connected");
  assert.equal(h.sockets[0].sent.length, 1, "subscribe is the only frame on open");
  assert.deepEqual(JSON.parse(h.sockets[0].sent[0]), {
    namespace: "system", type: "subscribeToNamespace", payload: "ds",
  });
});

test("[harness] INIT/START/STOP send base64 payloads on the ds namespace", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp", "Auto"], active: null, opmodeState: null }));
  assert.equal(h.els.get("opmode").value, "RobotTeleOp");
  assert.equal(h.els.get("btnInit").disabled, false);

  click(h, "btnInit");
  assert.equal(ws.sent.length, 2);
  const init = JSON.parse(ws.sent[1]);
  assert.equal(init.namespace, "ds");
  assert.equal(init.type, "initOpMode");
  assert.equal(core.b64decodeUtf8(init.payload), "RobotTeleOp");

  // The guard holds until the robot reports the transition: a second click is
  // swallowed by the disabled button, so no duplicate command is sent.
  assert.equal(h.els.get("btnInit").disabled, true);
  click(h, "btnInit");
  assert.equal(ws.sent.length, 2, "no repeat command while pending");

  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp", "Auto"], active: "RobotTeleOp", opmodeState: "INIT" }));
  assert.equal(h.els.get("btnStart").disabled, false);
  click(h, "btnStart");
  const start = JSON.parse(ws.sent[2]);
  assert.equal(start.type, "startOpMode");
  assert.equal(start.payload, "", "start carries an empty payload");
  assert.equal(start.namespace, "ds");

  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp", "Auto"], active: "RobotTeleOp", opmodeState: "LOOPING" }));
  assert.equal(h.els.get("btnStop").disabled, false);
  click(h, "btnStop");
  const stop = JSON.parse(ws.sent[3]);
  assert.equal(stop.type, "stopOpMode");
  assert.equal(stop.payload, "");
});

// ---------------------------------------------------------------------------
// harness: telemetry across a reconnect
// ---------------------------------------------------------------------------

test("[harness] telemetry is dropped on disconnect and an empty replay cannot resurrect it", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("telemetryDump", { entries: { Heading: "42", LeftPower: "0.5" } }));
  assert.deepEqual(telemetryKeys(h), ["Heading", "LeftPower"]);
  assert.equal(telemetryValue(h, "Heading"), "42");
  assert.equal(isHidden(h.els.get("telemetryEmpty")), true);

  ws.drop();
  assert.equal(h.els.get("conn").textContent, "Disconnected");
  assert.deepEqual(telemetryKeys(h), [], "stale rows are gone once the link drops");
  assert.equal(isHidden(h.els.get("telemetryEmpty")), false, "empty state is shown again");

  // Reconnect: the robot's snapshot is empty (opmode stopped -> detach()).
  h.clock.advance(1000);
  assert.equal(h.sockets.length, 2, "one reconnect attempt");
  const ws2 = h.sockets[1];
  ws2.open();
  ws2.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: null, opmodeState: null }));
  assert.deepEqual(telemetryKeys(h), [], "nothing to show, nothing invented");

  // Live data after the reconnect renders normally.
  ws2.deliver(robotFrame("telemetry", { key: "Heading", value: "7" }));
  assert.deepEqual(telemetryKeys(h), ["Heading"]);
  assert.equal(telemetryValue(h, "Heading"), "7");
});

test("[harness] a replayed live snapshot replaces the old one key-for-key", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("telemetryDump", { entries: { Stale: "1" } }));
  assert.deepEqual(telemetryKeys(h), ["Stale"]);

  ws.drop();
  h.clock.advance(1000);
  const ws2 = h.sockets[1];
  ws2.open();
  // DsSocketHandler.onSubscribe replays the snapshot as individual pairs.
  ws2.deliver(robotFrame("telemetry", { key: "Fresh", value: "2" }));
  assert.deepEqual(telemetryKeys(h), ["Fresh"], "the stale key did not survive");
  assert.equal(telemetryValue(h, "Fresh"), "2");
});

// ---------------------------------------------------------------------------
// harness: log pause / resume / clear
// ---------------------------------------------------------------------------

test("[harness] Pause freezes the displayed log and Resume flushes it in order", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("log", { line: "first" }));
  ws.deliver(robotFrame("log", { line: "second" }));
  assert.deepEqual(logLines(h), ["first", "second"]);

  click(h, "logPause");
  assert.equal(h.els.get("logPause").textContent, "Resume");
  ws.deliver(robotFrame("log", { line: "third" }));
  ws.deliver(robotFrame("error", { message: "boom" }));
  assert.deepEqual(logLines(h), ["first", "second"], "the paused view does not move");

  click(h, "logPause");
  assert.equal(h.els.get("logPause").textContent, "Pause");
  assert.deepEqual(logLines(h), ["first", "second", "third", "Error: boom"]);
});

test("[harness] a paused log queues a bounded number of lines, newest kept", () => {
  const { h, ws } = connectedUI();
  click(h, "logPause");
  const flood = core.MAX_LOG_LINES + 25;
  for (let i = 0; i < flood; i++) ws.deliver(robotFrame("log", { line: "line" + i }));
  assert.deepEqual(logLines(h), [], "still frozen under a flood");

  click(h, "logPause");
  const lines = logLines(h);
  assert.equal(lines.length, core.MAX_LOG_LINES, "queue is capped at the log buffer size");
  assert.equal(lines[0], "line" + (flood - core.MAX_LOG_LINES), "oldest queued lines were dropped");
  assert.equal(lines[lines.length - 1], "line" + (flood - 1));
});

test("[harness] Clear empties the log and drops anything queued while paused", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("log", { line: "kept" }));
  click(h, "logPause");
  ws.deliver(robotFrame("log", { line: "queued" }));
  click(h, "logClear");
  assert.deepEqual(logLines(h), []);
  click(h, "logPause");
  assert.deepEqual(logLines(h), [], "resume does not resurrect cleared lines");
});

// ---------------------------------------------------------------------------
// harness: pending guard
// ---------------------------------------------------------------------------

test("[harness] a diag that shows no transition does not release pending", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: null, opmodeState: null }));
  click(h, "btnInit");
  assert.equal(ws.sent.length, 2);
  assert.equal(h.els.get("btnInit").disabled, true);

  // Three 500 ms poller cycles with no state change: still waiting.
  for (let i = 0; i < 3; i++) {
    ws.deliver(robotFrame("diag", { battery: 12.6, motors: [], servos: [], active: null, opmodeState: null }));
  }
  assert.equal(h.els.get("btnInit").disabled, true, "diag alone is not proof the command finished");
  click(h, "btnInit");
  assert.equal(ws.sent.length, 2, "still no repeat command");

  // The robot finally reports the requested opmode in INIT.
  ws.deliver(robotFrame("diag", { battery: 12.6, motors: [], servos: [], active: "RobotTeleOp", opmodeState: "INIT" }));
  assert.equal(h.els.get("btnInit").disabled, false);
  assert.equal(h.els.get("btnStart").disabled, false);
});

test("[harness] pending is released by the bounded timeout when the robot never answers", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: null, opmodeState: null }));
  click(h, "btnInit");
  assert.equal(h.els.get("btnInit").disabled, true);

  h.clock.advance(3999);
  assert.equal(h.els.get("btnInit").disabled, true, "guard holds for the whole timeout window");
  h.clock.advance(1);
  assert.equal(h.els.get("btnInit").disabled, false, "guard releases at the bound");
});

test("[harness] an error reply releases pending and surfaces the message", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: null, opmodeState: null }));
  click(h, "btnInit");
  assert.equal(h.els.get("btnInit").disabled, true);

  ws.deliver(robotFrame("error", { message: "initOpMode: unknown opmode 'RobotTeleOp'" }));
  assert.equal(h.els.get("btnInit").disabled, false);
  assert.equal(logLines(h).some((l) => l.includes("unknown opmode")), true);
});

test("[harness] STOP stays available while a command is pending", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: "RobotTeleOp", opmodeState: "INIT" }));
  click(h, "btnStart");
  assert.equal(h.els.get("btnStart").disabled, true, "START is guarded");
  assert.equal(h.els.get("btnInit").disabled, true, "INIT is guarded");
  assert.equal(h.els.get("btnStop").disabled, false, "the stop button is never taken away");

  click(h, "btnStop");
  const last = JSON.parse(ws.sent[ws.sent.length - 1]);
  assert.equal(last.type, "stopOpMode");
});

test("[harness] a disconnect releases pending so a reconnect starts clean", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: null, opmodeState: null }));
  click(h, "btnInit");
  assert.equal(h.els.get("btnInit").disabled, true);

  ws.drop();
  assert.equal(h.els.get("btnInit").disabled, true, "disconnected: nothing is clickable");
  h.clock.advance(1000);
  const ws2 = h.sockets[1];
  ws2.open();
  ws2.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: null, opmodeState: null }));
  assert.equal(h.els.get("btnInit").disabled, false, "no command is stuck pending after a reconnect");
});

// ---------------------------------------------------------------------------
// harness: opmode list + INIT gating
// ---------------------------------------------------------------------------

test("[harness] an empty opmode list cannot enable INIT", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("state", { opmodes: [], active: null, opmodeState: null }));

  assert.equal(h.els.get("opmode").disabled, true);
  assert.equal(h.els.get("opmode").value, "", "the placeholder must not read as an opmode name");
  assert.equal(h.els.get("btnInit").disabled, true, "nothing is selected, so INIT is not offered");
  assert.equal(h.els.get("btnStart").disabled, true);
  click(h, "btnInit");
  assert.deepEqual(ws.sent.slice(1), [], "an empty list sends no command");

  // A list arriving later restores the control.
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: null, opmodeState: null }));
  assert.equal(h.els.get("opmode").disabled, false);
  assert.equal(h.els.get("opmode").value, "RobotTeleOp");
  assert.equal(h.els.get("btnInit").disabled, false);
});

test("[harness] frames outside the ds namespace never drive the UI", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: null, opmodeState: null }));
  assert.equal(h.els.get("opmode").value, "RobotTeleOp");

  const spoof = JSON.stringify({ opmodes: ["Spoof"], active: "Spoof", opmodeState: "INIT" });

  // The SDK multiplexes its own namespaces over the same socket; only frames on
  // the "ds" namespace are driver station state.
  ws.deliver(JSON.stringify({ namespace: "system", type: "state", payload: core.b64encodeUtf8(spoof) }));
  assert.equal(h.els.get("opmode").value, "RobotTeleOp", "a foreign state frame cannot replace the list");
  assert.equal(h.els.get("active").textContent, "None");
  assert.equal(h.els.get("btnStart").disabled, true, "a foreign frame cannot enable START");

  // A foreign envelope whose body claims a robot type is not a robot frame.
  ws.deliver(JSON.stringify({ namespace: "system", type: "whatever", payload: core.b64encodeUtf8(
    JSON.stringify(Object.assign({ type: "state" }, JSON.parse(spoof)))) }));
  assert.equal(h.els.get("opmode").value, "RobotTeleOp");
  assert.equal(h.els.get("active").textContent, "None");
  assert.equal(h.els.get("btnStart").disabled, true);

  // A body that disagrees with its own envelope is not a frame we recognise:
  // DsSocketHandler builds both from the same "type" field.
  ws.deliver(robotFrame("log", Object.assign({ type: "state" }, JSON.parse(spoof))));
  assert.equal(h.els.get("opmode").value, "RobotTeleOp");
  assert.equal(h.els.get("active").textContent, "None");
  assert.deepEqual(logLines(h), [], "the mismatched frame was not treated as a log line either");

  // The real thing still drives the UI.
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: "RobotTeleOp", opmodeState: "INIT" }));
  assert.equal(h.els.get("active").textContent, "RobotTeleOp");
  assert.equal(h.els.get("btnStart").disabled, false);
});

// ---------------------------------------------------------------------------
// harness: stale state after a disconnect
// ---------------------------------------------------------------------------

test("[harness] a disconnect stops presenting the lost session's state", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp", "Auto"], active: "RobotTeleOp", opmodeState: "INIT" }));
  ws.deliver(robotFrame("diag", {
    battery: 12.7,
    motors: [{ name: "leftFront", power: 0.5, position: 100, current: 1200, busy: true }],
    servos: [{ name: "claw", position: 0.4 }],
    active: "RobotTeleOp",
    opmodeState: "INIT",
  }));
  assert.equal(h.els.get("active").textContent, "RobotTeleOp");
  assert.equal(h.els.get("battery").textContent, "12.70");
  assert.equal(h.els.get("motors").children.length, 1);
  assert.equal(h.els.get("servos").children.length, 1);
  assert.equal(h.els.get("btnStart").disabled, false);
  const spark = h.els.get("batterySpark");
  assert.equal(spark.getContext("2d").clears > 0, true, "the battery trace was drawn while connected");
  const clearsWhileConnected = spark.getContext("2d").clears;

  ws.drop();

  assert.equal(h.els.get("conn").textContent, "Disconnected");
  assert.equal(h.els.get("opstate").textContent, "Idle", "the old opmode state is not current any more");
  assert.equal(h.els.get("opstate").getAttribute("data-state"), "idle");
  assert.equal(h.els.get("active").textContent, "None", "nothing is active as far as we know");
  assert.equal(h.els.get("battery").textContent, "n/a");
  assert.equal(h.els.get("motors").children.length, 0);
  assert.equal(h.els.get("servos").children.length, 0);
  assert.equal(isHidden(h.els.get("motorsEmpty")), false);
  assert.equal(isHidden(h.els.get("servosEmpty")), false);
  assert.equal(spark.getContext("2d").clears > clearsWhileConnected, true,
    "the lost session's battery trace is wiped");
  assert.equal(h.els.get("opmode").disabled, true, "the list came from the session we lost");
  assert.equal(h.els.get("opmode").value, "");
  assert.equal(h.els.get("btnInit").disabled, true);
  assert.equal(h.els.get("btnStart").disabled, true);
  assert.equal(h.els.get("btnStop").disabled, true, "no robot is reachable to stop");
});

test("[harness] a snapshot naming the opmode without INIT does not release the guard", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: null, opmodeState: null }));
  click(h, "btnInit");
  assert.equal(ws.sent.length, 2);
  assert.equal(h.els.get("btnInit").disabled, true);

  // DriverStationServer reports the active name as soon as OpModeManagerImpl has
  // it, and opmodeState as null until the opmode listener has recorded "INIT".
  // The name alone is not proof the init finished.
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: "RobotTeleOp", opmodeState: null }));
  assert.equal(h.els.get("btnInit").disabled, true, "the guard is still held");
  click(h, "btnInit");
  assert.equal(ws.sent.length, 2, "no duplicate init while the outcome is unknown");

  // The INIT state is the proof.
  ws.deliver(robotFrame("diag", { battery: 12.6, motors: [], servos: [], active: "RobotTeleOp", opmodeState: "INIT" }));
  assert.equal(h.els.get("btnInit").disabled, false);
  assert.equal(h.els.get("btnStart").disabled, false);
});

// ---------------------------------------------------------------------------
// harness: missing transport, injection, teardown
// ---------------------------------------------------------------------------

test("[harness] a browser without WebSocket is told so instead of sitting on Connecting", () => {
  const h = loadUI({ noWebSocket: true });
  assert.equal(h.sockets.length, 0);
  const conn = h.els.get("conn");
  assert.notEqual(conn.textContent, "Connecting", "must not claim it is connecting");
  assert.equal(conn.textContent, "Unsupported");
  assert.equal(conn.getAttribute("data-state"), "off");
  assert.equal(isHidden(h.els.get("banner")), false, "banner stays visible");
  assert.equal(h.els.get("btnInit").disabled, true);
  assert.equal(h.els.get("btnStart").disabled, true);
  assert.equal(h.els.get("btnStop").disabled, true);
  const lines = logLines(h);
  assert.equal(lines.length >= 1, true, "the failure is logged");
  assert.match(lines[lines.length - 1], /WebSocket/i);
  assert.equal(h.clock.intervalCount(), 1, "only the camera watchdog");
  assert.equal(h.clock.size(), 1, "no reconnect storm against a transport that cannot exist");
});

test("[harness] the no-WebSocket banner does not claim it is reconnecting", () => {
  const h = loadUI({ noWebSocket: true });
  const banner = h.els.get("banner");
  assert.equal(isHidden(banner), false, "the banner stays visible");
  assert.equal(/reconnect/i.test(banner.textContent), false,
    "nothing is reconnecting when the browser has no transport");
  assert.match(banner.textContent, /WebSocket/i, "the banner names what is missing");
  // The rest of the banner markup (the icon) is left alone.
  assert.ok(banner.children.length >= 1);
});

test("[harness] robot-supplied text is rendered as text, never as HTML", () => {
  const { h, ws } = connectedUI();
  const payload = '<img src=x onerror="alert(1)">';
  ws.deliver(robotFrame("log", { line: payload }));
  ws.deliver(robotFrame("telemetryDump", { entries: { [payload]: payload } }));

  const line = h.els.get("log").children[0];
  assert.equal(line.textContent, payload);
  assert.equal(line.children.length, 0, "no child element was parsed out of the text");
  assert.equal(h.els.get("log").children.length, 1);

  const row = h.els.get("telemetry").children[0];
  assert.equal(row.children[0].textContent, payload);
  assert.equal(row.children[1].textContent, payload);
  assert.equal(hasTag(h.els.get("log"), "IMG"), false);
  assert.equal(hasTag(h.els.get("telemetry"), "IMG"), false);
});

test("[harness] a stale socket is inert and reconnects never stack watchdogs", () => {
  const { h, ws } = connectedUI();
  assert.equal(h.clock.intervalCount(), 1);
  ws.deliver(robotFrame("log", { line: "before" }));

  ws.drop();
  h.clock.advance(1000);
  assert.equal(h.sockets.length, 2, "exactly one new socket");
  // The dead socket keeps its listeners but they are guarded: it can no longer
  // write to the DOM or reopen the connection.
  ws.deliver(robotFrame("log", { line: "zombie" }));
  assert.deepEqual(logLines(h), ["before"]);
  const connBefore = h.els.get("conn").textContent;
  ws.open();
  assert.equal(h.els.get("conn").textContent, connBefore, "a stale open is ignored");
  assert.equal(h.clock.intervalCount(), 1, "still one camera watchdog");

  h.sockets[1].open();
  // Several drop/reconnect rounds must not stack intervals or sockets.
  for (let i = 0; i < 4; i++) {
    h.sockets[h.sockets.length - 1].drop();
    h.clock.advance(10000);
  }
  assert.equal(h.sockets.length, 6, "one socket per attempt");
  assert.equal(h.clock.intervalCount(), 1, "no duplicate intervals after repeated reconnects");

  h.window.dispatch("beforeunload");
  assert.equal(h.clock.size(), 0, "unload clears the reconnect timer and the watchdog");
});

// ---------------------------------------------------------------------------
// harness: regression tests for the operator-audit fixes
// ---------------------------------------------------------------------------

test("[harness] a pressed action keeps keyboard focus: buttons are aria-disabled, never natively disabled", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: null, opmodeState: null }));
  click(h, "btnInit");
  // A native disabled button drops focus the moment it is pressed, so the guard
  // must be aria-disabled: the focused button stays focusable and inert.
  assert.equal(h.els.get("btnInit").nativeDisabled, false, "no native disabled on a pressed action");
  assert.equal(h.els.get("btnInit").getAttribute("aria-disabled"), "true");
  assert.equal(ws.sent.length, 2);
  click(h, "btnInit");
  assert.equal(ws.sent.length, 2, "an aria-disabled action is inert");

  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: "RobotTeleOp", opmodeState: "INIT" }));
  assert.equal(h.els.get("btnStart").getAttribute("aria-disabled"), "false");
  assert.equal(h.els.get("btnStart").nativeDisabled, false);
});

test("[harness] telemetry is dropped when the active opmode stops or is replaced", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp", "Auto"], active: "RobotTeleOp", opmodeState: "LOOPING" }));
  ws.deliver(robotFrame("telemetryDump", { entries: { Heading: "42" } }));
  assert.deepEqual(telemetryKeys(h), ["Heading"]);

  // The robot clears its snapshot on stop and sends nothing for it.
  ws.deliver(robotFrame("diag", { battery: 12.6, motors: [], servos: [], active: null, opmodeState: null }));
  assert.deepEqual(telemetryKeys(h), [], "a stopped opmode's values are not presented as live");

  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp", "Auto"], active: "Auto", opmodeState: "INIT" }));
  ws.deliver(robotFrame("telemetryDump", { entries: { Mode: "auto" } }));
  ws.deliver(robotFrame("diag", { battery: 12.6, motors: [], servos: [], active: "Auto", opmodeState: "LOOPING" }));
  assert.deepEqual(telemetryKeys(h), ["Mode"]);
  ws.deliver(robotFrame("diag", { battery: 12.6, motors: [], servos: [], active: "RobotTeleOp", opmodeState: "INIT" }));
  assert.deepEqual(telemetryKeys(h), [], "a replaced opmode's values are dropped too");
});

test("[harness] a reconnect replaces the log view instead of stacking the robot's replay on it", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("log", { line: "first" }));
  ws.deliver(robotFrame("log", { line: "second" }));
  ws.drop();
  h.clock.advance(1000);
  const ws2 = h.sockets[1];
  ws2.open();
  // The robot replays its log ring on every subscribe.
  ws2.deliver(robotFrame("log", { line: "first" }));
  ws2.deliver(robotFrame("log", { line: "second" }));
  assert.deepEqual(logLines(h), ["first", "second"], "each replayed line appears once");
});

test("[harness] a telemetry update does not rebuild the table, so a driver's text selection survives", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("telemetryDump", { entries: { A: "1", B: "2" } }));
  // A real browser drops a selection when the rows it sits in are removed. Count
  // every wipe of the table body: a rebuild is a textContent = "" on it.
  const body = h.els.get("telemetry");
  const rowsBefore = body.children.slice();
  let wipes = 0;
  const proto = Object.getOwnPropertyDescriptor(FakeElement.prototype, "textContent");
  Object.defineProperty(body, "textContent", {
    get() { return proto.get.call(this); },
    set(v) { if (v === "") wipes += 1; proto.set.call(this, v); },
  });
  ws.deliver(robotFrame("telemetryDump", { entries: { A: "5", B: "2" } }));
  assert.equal(wipes, 0, "an update with the same keys must not rebuild the table");
  assert.deepEqual(body.children, rowsBefore, "the same row elements stay attached");
  assert.equal(telemetryValue(h, "A"), "5");
});

test("[harness] a robot command error is shown beside the controls and cleared by the next command", () => {
  const { h, ws } = connectedUI();
  ws.deliver(robotFrame("state", { opmodes: ["RobotTeleOp"], active: null, opmodeState: null }));
  assert.equal(h.els.get("cmdStatus").textContent, "", "nothing shown before a command fails");
  click(h, "btnInit");
  ws.deliver(robotFrame("error", { message: "initOpMode: unknown opmode 'RobotTeleOp'" }));
  assert.match(h.els.get("cmdStatus").textContent, /unknown opmode/, "the failure is visible without the log");
  click(h, "btnInit");
  assert.equal(h.els.get("cmdStatus").textContent, "", "the next command clears the old error");
});

test("[harness] Copy works on plain http, where navigator.clipboard is unavailable", async () => {
  const h = loadUI({ insecure: true });
  h.sockets[0].open();
  h.sockets[0].deliver(robotFrame("log", { line: "one" }));
  h.sockets[0].deliver(robotFrame("log", { line: "two" }));
  click(h, "logCopy");
  await new Promise((resolve) => setImmediate(resolve));
  assert.deepEqual(h.copies, [{ command: "copy", value: "one\ntwo" }]);
  assert.equal(h.downloads.length, 0, "the text is copied, not saved as a file");
});

test("[harness] Copy saves the log only when no copy route works", async () => {
  const h = loadUI({ insecure: true, execCommandResult: false });
  h.sockets[0].open();
  h.sockets[0].deliver(robotFrame("log", { line: "one" }));
  click(h, "logCopy");
  await new Promise((resolve) => setImmediate(resolve));
  assert.equal(h.downloads.length, 1, "falls back to a file download when copying is refused");
});

test("[harness] Copy falls back to the textarea when the clipboard API rejects", async () => {
  const h = loadUI({ clipboardRejects: true });
  h.sockets[0].open();
  h.sockets[0].deliver(robotFrame("log", { line: "one" }));
  click(h, "logCopy");
  await new Promise((resolve) => setImmediate(resolve));
  assert.deepEqual(h.copies, [{ command: "copy", value: "one" }]);
  assert.equal(h.downloads.length, 0);
});
