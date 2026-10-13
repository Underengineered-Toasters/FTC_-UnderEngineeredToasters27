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

The DOM wiring itself is exercised by a throwaway fake-DOM harness during
development; it is not committed here because it stubs browser globals.
