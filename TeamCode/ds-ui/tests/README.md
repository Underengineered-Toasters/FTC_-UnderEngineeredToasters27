# Driver station client tests

Zero-dependency tests for `TeamCode/src/main/assets/ds/app.js` (the browser
client for the `ds` websocket namespace). They run on any Node 18+ install;
nothing is added to the Android build.

## Run

```sh
# from the repository root
node --test TeamCode/ds-ui/tests/app.test.js
# or
node --test TeamCode/ds-ui/tests/*.test.js
```

(Node 22's test runner no longer accepts a bare directory, so pass the file or
a glob.)

## What is covered

`app.js` is a classic browser script that also exports its pure core through
`module.exports`. The tests load it in Node (no DOM) and cover:

- base64/UTF-8 encode + decode round-trips (ASCII, accented, emoji)
- `parseRobotFrame` for every robot message type: `state`, `diag`, `telemetry`,
  `telemetryDump`, `log`, `error`, including malformed frames
- outbound command encoding that mirrors `DsSocketHandler.decodePayload`
  (`initOpMode` name, empty `startOpMode`/`stopOpMode` payloads)
- `controlsFor` safety gating (disconnected / idle / INIT / LOOPING / pending)
- `commandSatisfied` guard release: `initOpMode` needs the requested name
  reported in state `INIT` (the name alone can appear before the state does),
  `startOpMode` needs `LOOPING`, `stopOpMode` needs no active opmode
- `humanState` explicit labels
- `reconnectDelay` exponential backoff cap
- telemetry accumulation (`telemetry` single pair vs `telemetryDump` snapshot)
- `cameraDecision` retry cadence
- source invariants: retained element ids present, no `innerHTML`, a single
  `setInterval`, the raw-`ds` `subscribeToNamespace` handshake, no external
  `http(s)` origin, a bounded log buffer
- the class names `app.js` emits match the JS-contract classes defined in
  `app.css` (`badge--on/off/wait/init/loop/none`, `dot--busy/--idle`,
  `log-line--error`, `tvalwrap`, `tval__spark`)

## Browser-boundary harness

The DOM wiring is covered here too. `app.js` boots itself when a document
exists, so the harness runs the real file inside a `node:vm` context that
provides browser-shaped globals, then drives it with real events: clicks on the
real buttons, frames delivered through the fake `WebSocket`, and a virtual clock
for the backoff, watchdog and pending timers. Nothing inside `app.js` is stubbed;
there is no browser, no network and no robot.

The fakes deliberately mirror the browser details the client depends on:

- `<select>` reports the first option when the assigned value matches none of
  them, and `<option>` reports its value attribute when it has one and its text
  content otherwise (that difference is what makes a placeholder option without
  a value attribute impersonate a real opmode name)
- the markup the harness starts from is the markup `index.html` ships (banner
  copy, `value=""` opmode placeholder, `None`/`n/a` initial values)

Harness coverage:

- boot: one socket, the raw-`ds` subscribe frame, banner and badge transitions
- INIT/START/STOP command encoding and the pending guard (no repeat commands)
- the guard releases only on the snapshot that proves the effect, on an error
  reply, on a disconnect, or at the bounded timeout; a robot snapshot naming the
  opmode without `INIT` is not proof
- telemetry dropped on disconnect, and an empty replay cannot resurrect it
- a disconnect clears the robot snapshot (state badge, active opmode, battery,
  motor/servo rows, battery sparkline, opmode list) instead of presenting the
  lost session's values as current
- an empty opmode list cannot enable INIT
- frames outside the `ds` namespace, and bodies whose `type` disagrees with their
  envelope, never drive the UI
- log pause / resume / clear, including the bounded queue while paused
- no WebSocket support: honest badge, log line and banner copy, no reconnect storm
- robot-supplied text rendered as text; stale sockets inert; no stacked watchdogs
- action buttons use `aria-disabled`, so a pressed control keeps keyboard focus
- telemetry is dropped when the active opmode stops or is replaced
- a reconnect replaces the log view, so the robot's log replay is not doubled
- a telemetry update keeps the table rows, so a driver's text selection survives
- a robot command error shows beside the controls until the next command
- Copy works without the clipboard API (plain http) and saves a file only when no copy route works
