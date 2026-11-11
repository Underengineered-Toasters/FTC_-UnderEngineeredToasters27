# Camera align robustness implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make camera cluster alignment robust. Today `Robot.calculateClusterAlign` trusts any cluster pose it finds, drops the command on one missing frame, and flips between turn and no turn as the bearing crosses 2 degrees. After this plan, a bad pose is treated as no detection, a detection gap of up to 150 ms keeps the last command, and the aligned latch has a 3 degree exit band.

**Architecture:** One new hardware-free class, `ClusterAligner`, holds the whole decision: validity gates, drop-out hold, hysteresis latch, and the turn law as it is now. It takes `nowMs` as a parameter, so JUnit drives it without a clock or a robot. `Robot.calculateClusterAlign` becomes an adapter that reads the real `AprilTagClusterDetection` and copies the aligner result into the existing `AlignResult`. `RobotTeleOp` shows the held state. `TeamCode/build.gradle` gains JUnit. `TeamCode/AGENTS.md` documents the class and the test command.

**Tech Stack:** Java 8 source target, Android Gradle module `TeamCode`, FTC SDK 12.0 (`Vision-12.0.0`), JUnit 4.13.2, Gradle 9.6.0, JDK 25.

**Spec:** no separate spec file. The binding brief is the approval message plus the director rulings sent with this task, and the values below are copied from it. The existing align law comes from `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java` at BASE `61823bf`.

## Global constraints

- Branch `feature/autonomous-camera-align`, work in place. No new branch, no worktree. Tasks run in order.
- Commit locally only. No push, no `adb`, no robot access. PR #56 is not edited.
- Stage by explicit path, one `git add <path>` per file. Never `git add -A`, `git add .`, `git commit -a`. `CropCapture.java` and every other pre-existing change stay out.
- Commit subject: imperative, capitalized, no trailing period. Body: one or two plain sentences on why, and the issue number in place of `Refs #N` when an issue exists (no gh login during this run, so none exists yet). No separator characters used as punctuation.
- Bearing is counterclockwise positive. Turn law unchanged. `turn = -signum(bearing) * clamp(|ALIGN_KP * bearing|, ALIGN_MIN_TURN, ALIGN_MAX_TURN)`, `ALIGN_KP = 0.05`, `ALIGN_MIN_TURN = 0.15`, `ALIGN_MAX_TURN = 0.6`, `ALIGN_TOLERANCE_DEG = 2.0`.
- New values in the aligner only: `ALIGN_EXIT_TOLERANCE_DEG = 3.0`, `HOLD_TIMEOUT_MS = 150`, `MIN_PERCENT_FOUND = 50`, `MAX_ABS_ROLL_DEG = 90.0`.
- `Robot.AlignResult` gains exactly one boolean, `held`, at the end, from `Robot`, never `false` when the result came from a hold. Its signature becomes `AlignResult(boolean targetFound, boolean aligned, double turnPower, double bearingDeg, Double rangeInches, boolean held)`. `Robot` is the only construction site.
- No new production dependency. JUnit 4.13.2 is `testImplementation` only.
- SDK field names are from `Vision-12.0.0`, read with `javap`: `AprilTagClusterDetection.percentClusterFound` (`int`), `AprilTagClusterDetection.metadata` (`AprilTagClusterMetadata.name`), `AprilTagDetection.ftcPose` (`AprilTagPoseFtc.roll`, `range`, `bearing`).
- No smoothing, no gain change, no deployment.

Commands, run once per task, from the repo root:

```bash
export JAVA_HOME=/usr/lib/jvm/java-25-openjdk ANDROID_HOME=$HOME/Android/Sdk
./gradlew :TeamCode:test --console=plain
./gradlew :TeamCode:assembleDebug --console=plain
```

## Review focus

These fail modes matter most when a person runs the robot. Each has its test in the task that owns the code.

- A cluster seen from behind: `roll` near or past 90 degrees, bearing reads a confident but mirrored angle. Expected: `targetFound` false for that frame, exactly like no detection, so the held case runs only from the previous valid sample.
- Only part of the cluster visible: `percentClusterFound` under 50 solves neither bearing nor range. Expected: rejected as no detection. Exactly 50 is accepted.
- Camera misses one or two frames while the robot moves toward the tag: the turn command must survive up to 150 ms, `aligned` must read false during the hold, and 151 ms must read not found with state cleared.
- Bearing sits near the tolerance edge and jitters: a fresh aligner must not report aligned at 2.5; an aligned one must stay aligned at 2.5 and release at 3.0.
- `System.nanoTime` steps backwards between frames (`d < 0`): it is a discontinuity, so it counts as expired. Fresh detection is required, never an infinite hold.
- The caller switches cluster from one bumper to the other: the latch the old cluster earned must not report aligned for the new one, and the old last turn power must not be held over onto the new cluster.
- The first detection for a cluster arrives during a hold from another cluster: it is a fresh valid sample, not a continuation. Hold clock restarts at it.

---

### Task 1: `ClusterAligner` and the JUnit test suite

**Files:**
- Create: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ClusterAligner.java`
- Create: `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/ClusterAlignerTest.java`
- Modify: `TeamCode/build.gradle` (dependencies block)
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java` (the four `ALIGN_*` constant initialisers only; `calculateClusterAlign` and teleop are Task 2)

**Interfaces:**
- Consumes: nothing. `Robot.java` at BASE for the constants and law.
- Produces for Task 2:
  - `public final class ClusterAligner` in `org.firstinspires.ftc.teamcode`, no Android or FTC imports.
  - `public static final double ALIGN_KP = 0.05, ALIGN_MAX_TURN = 0.6, ALIGN_MIN_TURN = 0.15, ALIGN_TOLERANCE_DEG = 2.0, ALIGN_EXIT_TOLERANCE_DEG = 3.0, MAX_ABS_ROLL_DEG = 90.0; public static final long HOLD_TIMEOUT_MS = 150; public static final int MIN_PERCENT_FOUND = 50;`
  - `public static final class Result { public final boolean targetFound; public final boolean aligned; public final boolean held; public final double turnPower; public final double bearingDeg; public final Double rangeInches; }` with a constructor in that field order, and `public static final Result NO_TARGET` (`targetFound` false, `aligned` false, `held` false, `turnPower` 0.0, `bearingDeg` 0.0, `rangeInches` null).
  - `public static boolean isPoseUsable(double rollDeg, int percentFound)`: `abs(rollDeg) < MAX_ABS_ROLL_DEG && percentFound >= MIN_PERCENT_FOUND`.
  - `public static double turnForBearing(double bearingDeg)`: the turn law above, for a fresh valid bearing.
  - `public Result update(String clusterName, Boolean detectionValid, double bearingDeg, double rangeInches, long nowMs)`: `null` means no detection this frame; `Boolean.TRUE` means a detection whose pose is usable; `Boolean.FALSE` means a detection the caller rejected after `isPoseUsable` returned false. `FALSE` and `null` behave the same way, and both behave the same way as no sample ever seen on the first frame.
  - A hold result is built per call from the stored fields; there is no shared mutable PENDING. No state getter is added, since Task 2 reads only the returned Result.

- [ ] **Step 1: Add the test dependency**

In `TeamCode/build.gradle`:

```gradle
dependencies {
    implementation project(':FtcRobotController')
    testImplementation 'junit:junit:4.13.2'
}
```

- [ ] **Step 2: Write the failing test**

Create `TeamCode/src/test/java/org/firstinspires/ftc/teamcode/ClusterAlignerTest.java`. Test class helpers:

```java
private static ClusterAligner.Result hit(ClusterAligner a, String name, double bearing, double range, long ms) {
    return a.update(name, Boolean.TRUE, bearing, range, ms);
}
private static ClusterAligner.Result miss(ClusterAligner a, String name, long ms) {
    return a.update(name, null, 0.0, 0.0, ms);
}
```

Passing `Boolean.TRUE` does not run the gates; the caller runs them after `isPoseUsable`, as Task 2 does. Do not assert object identity, since the contract does not promise object reuse.

Tests, one `@Test` each:

- `bearingLeftIsNegativeTurn`: `hit(a, "RED", 10.0, 30.0, 0L)` gives `turnPower == -0.5`, `bearingDeg == 10.0`, `rangeInches == 30.0`, `targetFound`, `!aligned`, `!held`.
- `bearingRightIsPositiveTurn`: `hit(a, "RED", -10.0, 30.0, 0L)` gives `turnPower == 0.5`.
- `turnClampsToMin`: `hit(a, "RED", 2.5, 30.0, 0L)` gives `turnPower == -0.15` and not aligned.
- `turnClampsToMax`: `hit(a, "RED", 40.0, 30.0, 0L)` gives `turnPower == -0.6`.
- `alignedAtTolerance`: `hit(a, "RED", 2.0, 30.0, 0L)` gives `aligned`, `turnPower == 0.0`, `targetFound`.
- `alignedIsLatchInsideBand`: `hit(a, "RED", 2.0, 30.0, 0L)` then `hit(a, "RED", 2.5, 30.0, 20L)` gives `aligned`, `turnPower == 0.0`, `bearingDeg == 2.5`.
- `latchReleasesAtThree`: `hit(a, "RED", 2.0, 30.0, 0L)` then `hit(a, "RED", 3.0, 30.0, 20L)` gives `!aligned`, `turnPower == -0.15`.
- `latchReentersAtTwo`: `hit(a, "RED", 2.0, 30.0, 0L)` then `hit(a, "RED", 3.0, 30.0, 20L)` then `hit(a, "RED", 2.0, 30.0, 40L)` gives `aligned`, `turnPower == 0.0`.
- `freshNotAlignedAtTwoFive`: `hit(a, "RED", 2.5, 30.0, 0L)` gives `!aligned`, `turnPower == -0.15`.
- `holdKeepsLastTurn`: `hit(a, "RED", 10.0, 30.0, 0L)` then `miss(a, "RED", 100L)` gives `targetFound`, `held`, `!aligned`, `turnPower == -0.5`, `bearingDeg == 10.0`, `rangeInches == 30.0`.
- `holdFromAlignedStateHasNoTurn`: `hit(a, "RED", 1.0, 30.0, 0L)` then `miss(a, "RED", 50L)` gives `held`, `!aligned`, `turnPower == 0.0`.
- `holdLastsExactly150`: `hit(a, "RED", 10.0, 30.0, 0L)` then `miss(a, "RED", 150L)` gives `targetFound`, `held`.
- `holdExpiresAt151`: same then `miss(a, "RED", 151L)` gives `!targetFound`, `!held`, `!aligned`, `turnPower == 0.0`, `rangeInches == null`.
- `expiryClearsLatch`: `hit(a, "RED", 1.0, 30.0, 0L)`, `miss(a, "RED", 152L)`, `hit(a, "RED", 2.5, 30.0, 160L)` gives `!aligned`.
- `rejectedDetectionHoldsLikeMiss`: `hit(a, "RED", 10.0, 30.0, 0L)` then `a.update("RED", Boolean.FALSE, 0.0, 0.0, 100L)` gives `held`, `turnPower == -0.5`, `bearingDeg == 10.0`.
- `missingAtFirstSightingIsNotFound`: fresh aligner, `miss(a, "RED", 0L)` gives `!targetFound`.
- `clockStepBackCountsAsExpiredDOT`: `hit(a, "RED", 10.0, 30.0, 1000L)` then `miss(a, "RED", 500L)` gives `!targetFound` (`d < 0`).
- `switchToOtherClusterClearsState`: `hit(a, "RED SCORING", 1.0, 30.0, 0L)` then `hit(a, "BLUE SCORING", 2.5, 30.0, 20L)` gives `!aligned`, `turnPower == -0.15`.
- `switchDoesNotHoldOldCommand`: `hit(a, "RED SCORING", 10.0, 30.0, 0L)` then `miss(a, "BLUE SCORING", 20L)` gives `!targetFound`, `!held`.
- `switchBackIsFresh`: after the previous switch, `hit(a, "RED SCORING", -10.0, 30.0, 40L)` gives `turnPower == 0.5` with its own sign, not carried from RED.
- `firstHitAfterOtherClusterLatchIsNotAligned`: `hit(a, "RED", 1.0, 30.0, 0L)` then `hit(a, "BLUE", 2.0, 30.0, 1L)`: BLUE's own bearing is inside tolerance, so `aligned`; then `hit(a, "RED", 2.5, 30.0, 200L)` must be `!aligned` because RED's latch died in its own reset.
- `resetForgetsEverything`: `hit(a, "RED", 10.0, 30.0, 0L)`, `reset()`, `miss(a, "RED", 10L)` gives `!targetFound`.
- `poseUsableAcceptsUprightStrongCluster`: `isPoseUsable(0.0, 100)` true.
- `poseUsableAcceptsRollJustUnderLimit`: `isPoseUsable(89.9, 50)` true.
- `poseUsableRejectsRollAtLimit`: `isPoseUsable(90.0, 100)` false.
- `poseUsableRejectsMirroredRoll`: `isPoseUsable(-90.0, 100)` false.
- `poseUsableRejectsPartialCluster`: `isPoseUsable(0.0, 49)` false.
- `poseUsableRejectsNegativePercent`: `isPoseUsable(0.0, -1)` false.
- `turnForBearingKeepsSignConvention`: `turnForBearing(10.0) == -turnForBearing(-10.0)` and `turnForBearing(10.0) == -0.5`.

Do not add a test that only asserts `turnForBearing` equals the expression it wraps.

Each test builds its own `new ClusterAligner()` and calls `hit` or `miss`. Do not share one aligner between methods, or state leaks.

- [ ] **Step 3: Create `ClusterAligner` and move the constants**

`ClusterAligner.java`, package `org.firstinspires.ftc.teamcode`, no imports at all:

State fields: `String lastClusterName` (null means unknown), `boolean hasSample`, `long lastValidMs`, `double lastTurnPower`, `double lastBearingDeg`, `double lastRangeInches`, `boolean alignedLatch`.

`update`:

1. Normalise the name: `clusterName` null is tracked as null, and `Objects.equals` decides the cluster change.
2. If the normalised name differs from `lastClusterName`, set `lastClusterName` and clear state (`hasSample = false`, `lastValidMs = 0`, `lastTurnPower = 0`, `lastBearingDeg = 0`, `lastRangeInches = 0`, `alignedLatch = false`).
3. If `Boolean.TRUE.equals(detectionValid)`, take the sample: `alignedLatch` becomes true when `abs(bearingDeg) <= ALIGN_TOLERANCE_DEG`, stays true when it already was and `abs(bearingDeg) <= ALIGN_EXIT_TOLERANCE_DEG`, and becomes false otherwise. Then `lastTurnPower = alignedLatch ? 0.0 : turnForBearing(bearingDeg)`, store bearing and range, `lastValidMs = nowMs`, `hasSample = true`, and return `Result(true, alignedLatch, false, lastTurnPower, bearingDeg, rangeInches)`.
4. Otherwise, when `hasSample` and `nowMs - lastValidMs >= 0` and that gap is `<= HOLD_TIMEOUT_MS`, return `Result(true, false, true, lastTurnPower, lastBearingDeg, lastRangeInches)`.
5. Otherwise (no sample, negative gap, or gap past the timeout), clear state including `alignedLatch` and return a fresh `Result` with the `NO_TARGET` values.

`turnForBearing` keeps the existing expression. `isPoseUsable` is the two comparisons.

In `Robot.java`, replace the four initialisers. Keep each name, type and javadoc-free form:

```java
public static final double ALIGN_KP = ClusterAligner.ALIGN_KP;
public static final double ALIGN_MAX_TURN = ClusterAligner.ALIGN_MAX_TURN;
public static final double ALIGN_MIN_TURN = ClusterAligner.ALIGN_MIN_TURN;
public static final double ALIGN_TOLERANCE_DEG = ClusterAligner.ALIGN_TOLERANCE_DEG;
```

`java.util.Objects` joins the existing `java.util.Collections` import in the top block. No other change in the file.

- [ ] **Step 4: Run the suite and the build**

```bash
export JAVA_HOME=/usr/lib/jvm/java-25-openjdk ANDROID_HOME=$HOME/Android/Sdk
./gradlew :TeamCode:test --console=plain
./gradlew :TeamCode:assembleDebug --console=plain
```

Expected: `ClusterAlignerTest` reports every `@Test` method, all passing, 0 failures, in the report or the XML at `TeamCode/build/test-results/testDebugUnitTest/TEST-org.firstinspires.ftc.teamcode.ClusterAlignerTest.xml`. Paste the count line. `assembleDebug` ends `BUILD SUCCESSFUL`.

- [ ] **Step 5: Commit**

```bash
git add TeamCode/build.gradle
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ClusterAligner.java
git add TeamCode/src/test/java/org/firstinspires/ftc/teamcode/ClusterAlignerTest.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java
git commit -m "Add a hardware-free cluster aligner with unit tests"
```

---

### Task 2: Wire `Robot.calculateClusterAlign` and the teleop telemetry

**Review:** per-task, public API and the robot-facing entry point

**Files:**
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java`
- Modify: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotTeleOp.java:95-104,197-207`

**Interfaces:**
- Consumes: Task 1's `ClusterAligner.update`, `ClusterAligner.Result` and its fields, `ClusterAligner.isPoseUsable`, `ClusterAligner.NO_TARGET`, and the constants. `findAprilTagCluster(String)` stays as it is.
- Produces: `Robot.calculateClusterAlign(String clusterName)` keeps its signature and returns `AlignResult` with a new `public final boolean held` field as the sixth constructor parameter. Both existing callers (`RobotTeleOp`, `BasicAutonomous.alignToCluster`) keep compiling unchanged.

- [ ] **Step 1: Regression pins, no new production code**

There is no red-first test for the adapter. Constructing `Robot` runs hardware field initialisers (`new intake()`, `new Shooter()`, `new AprilTagVision()`) and needs an Android runtime, which this module's JVM unit tests do not have. Task 1 covered the aligner behavior red to green. This step pins the two gate shapes the adapter relies on, so a later aligner change cannot break the adapter quietly. Write them, run the suite, and expect green: a red here means the aligner contract drifted, not that the adapter is missing.

- `gatedPoseReadsAsMissing`: a usable sample at 0 ms, then a `Boolean.FALSE` frame at 80 ms, gives `held` true, and the same shape at 160 ms gives `!targetFound` and no latch.
- `heldFrameCarriesLastBearingToCaller`: a valid sample at 2.5 deg (not aligned, turn -0.15) at 0 ms, then a `Boolean.FALSE` frame at 80 ms, gives `held` true with `bearingDeg` 2.5 and `turnPower` -0.15, the values a teleop running controller reads during the hold.

Run the suite. Expected: the two new names pass along with the earlier methods, no production change.

- [ ] **Step 2: Implement the adapter**

In `Robot.java`, add

```java
private final ClusterAligner aligner = new ClusterAligner();
```

and replace the body of `calculateClusterAlign`:

```java
public AlignResult calculateClusterAlign(String clusterName) {
    AprilTagClusterDetection target = findAprilTagCluster(clusterName);
    Boolean detectionValid = null;
    double bearing = 0.0;
    double range = 0.0;
    if (target != null && target.ftcPose != null) {
        detectionValid = ClusterAligner.isPoseUsable(target.ftcPose.roll, target.percentClusterFound);
        bearing = target.ftcPose.bearing;
        range = target.ftcPose.range;
    }
    ClusterAligner.Result r = aligner.update(clusterName, detectionValid, bearing, range,
            System.nanoTime() / 1_000_000L);
    return new AlignResult(r.targetFound, r.aligned, r.turnPower, r.bearingDeg, r.rangeInches, r.held);
}
```

`System.nanoTime` is monotonic and available on the same JVM the unit tests run on, unlike `android.os.SystemClock`. `findCluster` already drops a cluster with no `ftcPose`, so the `target.ftcPose != null` check is about a future caller, not about today's `findCluster`.

`AlignResult` gains `public final boolean held;` after `rangeInches` and a sixth constructor parameter, assigned in the constructor.

- [ ] **Step 3: Teleop telemetry**

In `RobotTeleOp`, the `Aligned` line reads

```java
telemetry.addData("Aligned",
        align != null && align.targetFound ? (align.held ? "HELD" : (align.aligned ? "YES" : "NO")) : "N/A");
```

The `Align Bearing` line is untouched. The bumper branch already drives `turn = align.turnPower` whenever `targetFound`, so a hold repeats the last command with no change there.

- [ ] **Step 4: Run the suite and the build**

```bash
export JAVA_HOME=/usr/lib/jvm/java-25-openjdk ANDROID_HOME=$HOME/Android/Sdk
./gradlew :TeamCode:test --console=plain
./gradlew :TeamCode:assembleDebug --console=plain
```

Expected: every `@Test` method passing; `BUILD SUCCESSFUL`; `CropCapture.java` still untracked and unmodified.

- [ ] **Step 5: Commit**

```bash
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Robot.java
git add TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotTeleOp.java
git add TeamCode/src/test/java/org/firstinspires/ftc/teamcode/ClusterAlignerTest.java
git commit -m "Route cluster alignment through the aligner and show held frames"
```

---

### Task 3: TeamCode DOX

**Files:**
- Modify: `TeamCode/AGENTS.md`

**Interfaces:**
- Consumes: the class and test command from Tasks 1 and 2.

- [ ] **Step 1: Local contracts**

Add one bullet: `ClusterAligner` owns the alignment rules, which are roll and percent gates, a 150 ms drop-out hold, 2 degree entry and 3 degree exit hysteresis, and the turn law. `Robot.calculateClusterAlign` only reads the detection and passes the result through, and its constants point at `ClusterAligner`, so a threshold belongs there.

- [ ] **Step 2: Verification**

Add `./gradlew :TeamCode:test` as the unit test command above `assembleDebug`, and note that the tests are JVM JUnit 4 with no robot and no camera.

- [ ] **Step 3: Run both commands, commit**

```bash
./gradlew :TeamCode:test --console=plain
./gradlew :TeamCode:assembleDebug --console=plain
git add TeamCode/AGENTS.md
git commit -m "Document the cluster aligner and the TeamCode test command"
```

Root `AGENTS.md` needs no change: no new module, workflow or child index entry.
