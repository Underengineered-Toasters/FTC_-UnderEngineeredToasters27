# Product

<!-- impeccable:product-schema 1 -->

## Platform

web

## Stack

Static HTML/CSS/JS served by the FTC Robot Controller's built-in web server (NanoHTTPD on port 8080, websocket on 8081) through the SDK `@WebHandlerRegistrar` hook in TeamCode. The stylesheet is hand-written and committed; there is no CSS build, CDN or runtime build step, because the robot Wi-Fi network has no internet access. Backend glue is Java in `org.firstinspires.ftc.teamcode.ds`.

## Users

The drive team and student programmers of FTC team 36424 (Underengineered Toasters). Used on a laptop or the Driver Hub browser while connected to the robot Wi-Fi network (`36424-RC`) during practice and debugging sessions.

## Product Purpose

A driver station web UI for monitoring and controlling the robot without opening the official Driver Station app. It shows live diagnostics (opmode state, battery voltage, motor and servo status, telemetry), the camera feed, and a debug log stream, and it can init, start, and stop opmodes. Success means the team can debug and run practice sessions from a browser.

## Positioning

Served directly by the Robot Controller web server with no external library dependency, unlike FTC Dashboard or Panels which are added to the project as dependencies. The layout is built for this team: diagnostics on one side, camera on the other.

## Operating Context

Runs on the robot's Wi-Fi network at `http://192.168.43.1:8080/ds`. Practice and debugging only: competition rules (RS09) ban browser dashboards during matches, and gamepads stay on the official Driver Station. Deployed with the TeamCode APK over wireless ADB.

## Capabilities and Constraints

- Opmode list, init, start, and stop through `OpModeManagerImpl`.
- Live diagnostics: battery voltage, motor power/position/busy state, servo positions, active opmode state.
- Camera feed as MJPEG from a `CameraStreamSource` (the `VisionPortal` in `AprilTagVision`).
- Telemetry and debug log streaming over the SDK websocket protocol (`FtcWebSocketMessage`, base64 payloads, `subscribeToNamespace`).
- No authentication: the robot Wi-Fi password is the trust boundary. All websocket input is still validated.
- No virtual gamepad; driving stays on physical gamepads.
- Everything works offline; no external fonts, CDNs, or services.

## Brand Commitments

Team name Underengineered Toasters, team number 36424. No other binding assets.

## Evidence on Hand

Robot code in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` (opmodes, `AprilTagVision`, `Shooter`, `intake`). No logos or imagery; do not fabricate any.

## Product Principles

- Glanceable under pressure: the driver needs state in under a second.
- Offline first: every asset ships inside the APK.
- Monitoring always works; control actions are explicit buttons.
- Plain data over decoration: numbers and states before visual flourish.
