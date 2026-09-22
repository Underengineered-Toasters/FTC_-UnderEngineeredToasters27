"use strict";

const $ = (id) => document.getElementById(id);
const els = {
  banner: $("banner"), conn: $("conn"), opmode: $("opmode"), opstate: $("opstate"),
  active: $("active"), battery: $("battery"), batterySpark: $("batterySpark"),
  motors: $("motors"), motorsEmpty: $("motorsEmpty"), servos: $("servos"),
  servosEmpty: $("servosEmpty"), telemetry: $("telemetry"), telemetryEmpty: $("telemetryEmpty"),
  log: $("log"), logPause: $("logPause"), logClear: $("logClear"), logCopy: $("logCopy"),
  cam: $("cam"), camEmpty: $("camEmpty"),
  btnInit: $("btnInit"), btnStart: $("btnStart"), btnStop: $("btnStop"),
};

// Base64 that survives non-ASCII opmode names (btoa alone throws on them).
function b64(s) {
  const bytes = new TextEncoder().encode(s);
  let bin = "";
  for (const b of bytes) bin += String.fromCharCode(b);
  return btoa(bin);
}

function el(tag, cls, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}

const BADGE = "inline-flex w-fit shrink-0 items-center justify-center gap-1 overflow-hidden rounded-full border px-2 py-0.5 text-xs font-medium whitespace-nowrap ";
const CONN = {
  on: BADGE + "border-emerald-500/30 bg-emerald-500/10 text-emerald-400",
  off: BADGE + "border-transparent bg-destructive/60 text-white",
  wait: BADGE + "border-transparent bg-secondary text-secondary-foreground",
};
const STATE_BADGE = {
  INIT: BADGE + "border-yellow-500/30 bg-yellow-500/10 text-yellow-400",
  LOOPING: BADGE + "border-emerald-500/30 bg-emerald-500/10 text-emerald-400",
  none: BADGE + "border-transparent bg-secondary text-secondary-foreground",
};
const TR = "border-b transition-colors hover:bg-muted/50";
const TH_ROW = "p-2 align-middle font-medium whitespace-nowrap text-foreground";
const TD = "p-2 align-middle text-right whitespace-nowrap tabular-nums";

let ws = null;
let telemetryRows = new Map();

// Rolling sparkline history per canvas, ~120 samples.
const SPARK_MAX = 120;
const sparkData = new Map();

function pushSpark(canvas, v, color) {
  let h = sparkData.get(canvas);
  if (!h) { h = []; sparkData.set(canvas, h); }
  h.push(v);
  if (h.length > SPARK_MAX) h.shift();
  const ctx = canvas.getContext("2d");
  const w = canvas.width, hh = canvas.height;
  ctx.clearRect(0, 0, w, hh);
  if (h.length < 2) return;
  let min = Math.min(...h), max = Math.max(...h);
  if (max - min < 1e-9) { min -= 1; max += 1; }
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  h.forEach((p, i) => {
    const x = (i / (h.length - 1)) * w;
    const y = hh - 2 - ((p - min) / (max - min)) * (hh - 4);
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.stroke();
}

function setConn(connected) {
  els.conn.textContent = connected ? "Connected" : "Disconnected";
  els.conn.className = connected ? CONN.on : CONN.off;
  els.banner.classList.toggle("hidden", connected);
  els.btnInit.disabled = !connected || !els.opmode.value;
  els.btnStart.disabled = !connected;
  els.btnStop.disabled = !connected;
}

function setOpmodes(names, active, state) {
  const prev = els.opmode.value;
  els.opmode.textContent = "";
  if (!names || names.length === 0) {
    els.opmode.append(el("option", "", "No opmodes"));
    els.opmode.disabled = true;
  } else {
    for (const n of names) els.opmode.append(el("option", "", n));
    els.opmode.disabled = false;
    if (names.includes(prev)) els.opmode.value = prev;
    else if (active && names.includes(active)) els.opmode.value = active;
  }
  setActive(active, state);
  els.btnInit.disabled = !(ws && ws.readyState === WebSocket.OPEN) || !els.opmode.value;
}

function setActive(active, state) {
  els.active.textContent = active || "None";
  els.opstate.textContent = state || "Idle";
  els.opstate.className = STATE_BADGE[state] || STATE_BADGE.none;
}

function setBattery(v) {
  const n = (v === null || v === undefined) ? NaN : Number(v);
  els.battery.textContent = Number.isFinite(n) ? n.toFixed(2) : "n/a";
  if (Number.isFinite(n)) pushSpark(els.batterySpark, n, "#facc15");
}

function setMotors(motors) {
  els.motors.textContent = "";
  const list = Array.isArray(motors) ? motors : [];
  els.motorsEmpty.classList.toggle("hidden", list.length > 0);
  for (const m of list) {
    const tr = el("tr", TR);
    const name = el("th", TH_ROW, m.name);
    name.scope = "row";
    tr.append(name);
    tr.append(el("td", TD, m.power === null || m.power === undefined ? "n/a" : Number(m.power).toFixed(2)));
    tr.append(el("td", TD, m.position === null || m.position === undefined ? "n/a" : String(m.position)));
    tr.append(el("td", TD, m.current === null || m.current === undefined ? "n/a" : String(Math.round(m.current))));
    const busy = el("td", TD);
    const dot = el("span", "inline-block h-2 w-2 rounded-full " + (m.busy ? "bg-yellow-400" : "bg-zinc-700"));
    dot.title = m.busy ? "Busy" : "Idle";
    busy.append(dot);
    tr.append(busy);
    els.motors.append(tr);
  }
}

function setServos(servos) {
  els.servos.textContent = "";
  const list = Array.isArray(servos) ? servos : [];
  els.servosEmpty.classList.toggle("hidden", list.length > 0);
  for (const s of list) {
    const tr = el("tr", TR);
    const name = el("th", TH_ROW, s.name);
    name.scope = "row";
    tr.append(name);
    tr.append(el("td", TD + " text-muted-foreground",
      s.position === null || s.position === undefined ? "n/a" : Number(s.position).toFixed(2)));
    els.servos.append(tr);
  }
}

function setTelemetry(key, value) {
  let row = telemetryRows.get(key);
  if (!row) {
    const tr = el("tr", TR);
    const k = el("th", TH_ROW, key);
    k.scope = "row";
    const td = el("td", "p-2 align-middle text-right text-foreground");
    const wrap = el("span", "inline-flex items-center justify-end gap-2");
    const v = el("span", "break-all");
    wrap.append(v);
    td.append(wrap);
    tr.append(k, td);
    telemetryRows.set(key, { tr, v, wrap, spark: null });
    els.telemetry.append(tr);
    row = telemetryRows.get(key);
  }
  row.v.textContent = String(value);
  const n = Number(value);
  if (value !== "" && Number.isFinite(n)) {
    if (!row.spark) {
      row.spark = el("canvas", "h-4 w-16 shrink-0");
      row.spark.width = 64;
      row.spark.height = 16;
      row.spark.setAttribute("aria-hidden", "true");
      row.wrap.append(row.spark);
    }
    pushSpark(row.spark, n, "#a1a1aa");
  }
  els.telemetryEmpty.classList.add("hidden");
}

let logPinned = true;
let logPaused = false;
els.log.addEventListener("scroll", () => {
  const l = els.log;
  logPinned = l.scrollTop + l.clientHeight >= l.scrollHeight - 8;
});

els.logPause.addEventListener("click", () => {
  logPaused = !logPaused;
  els.logPause.textContent = logPaused ? "Resume" : "Pause";
});
els.logClear.addEventListener("click", () => { els.log.textContent = ""; });
els.logCopy.addEventListener("click", async () => {
  const text = [...els.log.children].map(d => d.textContent).join("\n");
  try {
    await navigator.clipboard.writeText(text);
  } catch {
    // Plain HTTP page: clipboard API may be unavailable. Fall back to a download.
    const a = el("a");
    a.href = URL.createObjectURL(new Blob([text], { type: "text/plain" }));
    a.download = "driver-station.log";
    a.click();
    URL.revokeObjectURL(a.href);
  }
});

function addLog(text, isError) {
  els.log.append(el("div", isError ? "text-destructive" : "", text));
  while (els.log.childElementCount > 500) els.log.firstChild.remove();
  if (logPinned && !logPaused) els.log.scrollTop = els.log.scrollHeight;
}

function send(type, payload) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ namespace: "ds", type, payload: b64(payload || "") }));
  }
}

els.btnInit.addEventListener("click", () => send("initOpMode", els.opmode.value));
els.btnStart.addEventListener("click", () => send("startOpMode", ""));
els.btnStop.addEventListener("click", () => send("stopOpMode", ""));
els.opmode.addEventListener("change", () => {
  els.btnInit.disabled = !els.opmode.value || !ws || ws.readyState !== WebSocket.OPEN;
});

function handle(msg) {
  let data;
  try {
    data = JSON.parse(atob(msg.payload || ""));
  } catch {
    return; // malformed frame: ignore, never throw into the page
  }
  switch (data.type) {
    case "state":
      setOpmodes(data.opmodes, data.active, data.opmodeState);
      break;
    case "diag":
      setBattery(data.battery);
      setMotors(data.motors);
      setServos(data.servos);
      setActive(data.active, data.opmodeState);
      break;
    case "log":
      addLog(String(data.line));
      break;
    case "error":
      addLog("Error: " + String(data.message), true);
      break;
  }
}

function connect() {
  setConn(false);
  els.conn.textContent = "Connecting";
  els.conn.className = CONN.wait;
  try {
    ws = new WebSocket("ws://" + location.hostname + ":8081");
  } catch {
    setTimeout(connect, 2000);
    return;
  }
  ws.onopen = () => {
    setConn(true);
    // SDK protocol: the subscribe payload is the raw namespace string, not base64.
    ws.send(JSON.stringify({
      namespace: "system",
      type: "subscribeToNamespace",
      payload: "ds",
    }));
  };
  ws.onmessage = (ev) => {
    try {
      handle(JSON.parse(ev.data));
    } catch {
      // ignore unparseable frames
    }
  };
  ws.onclose = () => {
    setConn(false);
    setTimeout(connect, 2000);
  };
  ws.onerror = () => {
    // onclose follows and handles the retry
  };
}

// Camera feed: MJPEG <img> gives no per-frame events. naturalWidth > 0 means
// frames are arriving; 'load'/'error' mean the stream ended or 503'd, so we
// show the placeholder and retry by re-setting src every 5 s while dead.
let camDead = false;
let camRetry = 0;
els.cam.addEventListener("load", () => { camDead = true; });
els.cam.addEventListener("error", () => { camDead = true; });
setInterval(() => {
  const live = els.cam.naturalWidth > 0 && !camDead;
  els.cam.classList.toggle("hidden", !live);
  els.camEmpty.classList.toggle("hidden", live);
  if (camDead) {
    camRetry += 1;
    if (camRetry >= 5) {
      camRetry = 0;
      camDead = false;
      els.cam.src = "/ds/camera.mjpeg";
    }
  }
}, 1000);

connect();
