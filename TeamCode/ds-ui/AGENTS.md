# Driver Station UI Tests

## Purpose
Zero-dependency Node tests for the Driver Station browser client, `TeamCode/src/main/assets/ds/app.js`.

## Ownership
Team programming owns the tests. The client assets are in `TeamCode/src/main/assets/ds/` and are packaged into the APK; nothing in `ds-ui/` is part of the Android build.

## Local Contracts
- `app.js` exports its pure core through `module.exports` and boots itself when a document exists. The tests rely on both.
- There is no CSS or JS build step. `app.css` is committed as written.
- Tests mirror `DsSocketHandler`, so a protocol change in either file needs a matching test change.
- Assets must work offline: no external origins, fonts or CDNs.

## Work Guidance
Add a regression test with every behavior fix. See `tests/README.md` for what the suite covers.

## Verification
Run `node --test TeamCode/ds-ui/tests/app.test.js` from the repository root. CI runs the same command.

## Child DOX Index
None.
