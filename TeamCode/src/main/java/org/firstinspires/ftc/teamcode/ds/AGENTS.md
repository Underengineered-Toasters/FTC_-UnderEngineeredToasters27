# Driver Station Server

## Purpose
Java side of the Driver Station web UI: an HTTP page, an MJPEG camera feed and a websocket namespace served from the Robot Controller.

## Ownership
Team programming owns these classes. The browser client lives in `TeamCode/src/main/assets/ds/` and its tests in `TeamCode/ds-ui/`.

## Local Contracts
- `DriverStationServer` registers the handlers through the SDK `@WebHandlerRegistrar` hook. The page is `/ds` on port 8080 and the websocket namespace is `ds` on port 8081.
- Routes: `/ds` and `/ds/` serve `index.html`, `/ds/<file>` serves each fixed file under `assets/ds/` (read once at registration), and `/ds/camera.mjpeg` streams the camera. Unknown files get 404. No other routes exist.
- `DriverStation` is the static facade for OpModes. `attach`, `setCameraSource`, `log`, `telemetry` and `detach` are safe to call with no browser connected. Outbound work happens only while a browser is subscribed.
- `detach` clears the hardware map, the camera source and the telemetry, then sends an empty `telemetryDump` so connected browsers clear their tables.
- Command policy in `DsSocketHandler`: `initOpMode` is a no-op for the opmode already initialised and is refused for the running one (stop it first). `startOpMode` needs an initialised opmode and is a no-op while one is looping. `stopOpMode` requests the SDK's default stop opmode, the same request the SDK's own stop command makes, and is a no-op when nothing is active.
- Input limits: base64 payloads over 1024 characters, names over 128 characters and names with control characters are refused. Unknown message types and malformed payloads get an `error` reply. Opmode names are validated before they reach the SDK.
- `DsSocketHandler` and `app.js` share one wire protocol. Change both in the same commit, and update the tests in `TeamCode/ds-ui/tests/`.
- Telemetry has two sources. `DriverStation.telemetry` values are pinned until `detach`. Rows set through `DsTelemetry` (the opmode's `addData` and `addLine(caption)`) mirror the SDK frame and clear when the SDK auto-clears. `addLine()` without a caption is not mirrored. Each `update()` sends one `telemetryDump` that replaces the browser table.
- Opmode state (`INIT`, `LOOPING` or null) comes from `OpModeManagerNotifier`. The listener is held by a strong static reference because the SDK keeps listeners weakly. The post-stop callback clears the state and detaches.
- MJPEG: the HTTP thread pulls one frame per request, paced at about 15 fps. The stream ends when the camera stalls for 2 seconds, when the camera source is removed, or when the client disconnects. Each frame is compressed to JPEG inside the SDK's frame callback, because the SDK recycles the bitmap when the callback returns. Never keep the bitmap or use it from another thread. This was found on the robot, where handing the bitmap to the HTTP thread threw "Can't compress a recycled bitmap".
- Diagnostics run on one daemon thread every 500 ms and read hardware only while a browser is subscribed. A missing or failing device is skipped and does not hide the others.
- Failures are logged with `RobotLog` and never thrown into the SDK. An uncaught exception on any thread ends the Robot Controller app.
- Validate all websocket input. There is no authentication.
- Practice and debugging only. Game Manual rule R704.D prohibits third party streaming tools and any continuous video stream, so the server and camera stream must be off for matches and inspection.

## Work Guidance
- Keep OpMode hookups in the OpMode. Always call `DriverStation.detach()` in a `finally`.
- Do not add dependencies or network calls to external hosts.
- Pure logic (payload and name validation, command policy, path checks, MIME types, MJPEG framing, frame hand-off, outbox) stays free of Android and SDK types so it can be unit tested. Tests live in `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/ds/` (JUnit 4).

## Verification
- Run `JAVA_HOME=<JDK 25> ./gradlew :TeamCode:compileDebugJavaWithJavac` and `node --test TeamCode/ds-ui/tests/app.test.js`.
- Hardware behavior (camera stream, opmode control, battery and motor readings, the diagnostics thread) needs a robot and stays unverified until tried on one.

## Child DOX Index
None.
