# Driver Station Server

## Purpose
Java side of the Driver Station web UI: an HTTP page, an MJPEG camera feed and a websocket namespace served from the Robot Controller.

## Ownership
Team programming owns these classes. The browser client lives in `TeamCode/src/main/assets/ds/` and its tests in `TeamCode/ds-ui/`.

## Local Contracts
- `DriverStationServer` registers the handlers through the SDK `@WebHandlerRegistrar` hook. The page is `/ds` on port 8080 and the websocket namespace is `ds` on port 8081.
- `DriverStation` is the static facade for OpModes. `attach`, `setCameraSource`, `log`, `telemetry` and `detach` are safe to call with no browser connected.
- `DsSocketHandler` and `app.js` share one wire protocol. Change both in the same commit, and update the tests in `TeamCode/ds-ui/tests/`.
- Validate all websocket input. There is no authentication.
- Practice and debugging only (RS09 bans browser dashboards in matches).

## Work Guidance
- Keep OpMode hookups in the OpMode. Always call `DriverStation.detach()` in a `finally`.
- Do not add dependencies or network calls to external hosts.

## Verification
- Run `JAVA_HOME=<JDK 25> ./gradlew :TeamCode:compileDebugJavaWithJavac` and `node --test TeamCode/ds-ui/tests/app.test.js`.
- Hardware behavior (camera stream, opmode control, battery and motor readings) needs a robot and stays unverified until tried on one.

## Child DOX Index
None.
