package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Autonomous route that positions with the BIOBUZZ AprilTag clusters, then scores the preload.
 *
 * <p>Route: drive to the audience side, align to the alliance audience cluster, reposition to the
 * scoring side, align to the alliance scoring cluster, score the preload, park.
 *
 * <p>Fixed relative to the previous version:
 * <ul>
 *   <li>Alliance is selectable before start (A = RED, B = BLUE) instead of hardcoded Red.</li>
 *   <li>It actually scores: {@link AutoSequence#shootBalls} settles the turret, spins the
 *       flywheels up and feeds the balls. The old route only drove around.</li>
 *   <li>Alignment is bounded and time-guarded, and uses {@link Robot#SDK_YAW_TO_TURN} so it turns
 *       toward the tag instead of away from it.</li>
 *   <li>Every step is wrapped in a {@code finally} that stops all mechanisms and closes vision, so
 *       a stopped OpMode no longer leaves the camera open or a flywheel running.</li>
 * </ul>
 *
 * <p>Distances are the previous timed legs converted to inches at the shared calibration; they are
 * still estimates and must be measured on the field.
 */
@Autonomous(name = "AprilTag Autonomous Path", group = "Autonomous")
public class AprilTagAutonomous extends LinearOpMode {

    // ============================================================
    // Route - CALIBRATE on the field
    // ============================================================

    /** First leg toward the audience wall (was 1.5 s at 0.4 power). */
    public static double audienceLegInches = 24.0;

    /** Turn from the start heading toward the audience cluster (was 0.8 s at 0.3 power). */
    public static double audienceTurnDegrees = 80.0;

    /** Back away from the audience wall before crossing (was 1.0 s at 0.3 power). */
    public static double audienceBackoffInches = 12.0;

    /** Turn from the audience side toward the scoring side (was 1.2 s at 0.3 power). */
    public static double scoringTurnDegrees = -120.0;

    /** Cross-field leg to the scoring side (was 2.0 s at 0.4 power). */
    public static double scoringLegInches = 32.0;

    /** Final park nudge away from the goal (was 0.8 s at 0.3 power). */
    public static double parkInches = -10.0;

    /** Travel power for the timed legs. */
    public static double routePower = 0.40;

    /** Approach range while aligning to the audience cluster. */
    public static double audienceRangeInches = 12.0;

    /** Shooting range to the scoring cluster; must match what the flywheel can actually reach. */
    public static double scoringRangeInches = 12.0;

    /** Longest time any single alignment may take. */
    public static double alignTimeoutSeconds = 4.0;

    /** Time budget for settling the turret, spinning up and feeding the preload. */
    public static double shootBudgetSeconds = 4.0;

    // Match guard
    public static double lastActionSeconds = 27.0;
    public static double matchSeconds = 30.0;

    /** BIOBUZZ cluster member ids; any member selects the whole cluster's pose. */
    public static int redAudienceTag = 34;
    public static int redScoringTag = 30;
    public static int blueAudienceTag = 38;
    public static int blueScoringTag = 42;

    private final Robot robot = new Robot();
    private TimedMotion motion;
    private AutoSequence sequence;
    private boolean redAlliance = true;

    @Override
    public void runOpMode() {
        try {
            robot.init(hardwareMap);
            motion = TimedMotion.forRobot(robot, this);
            sequence = new AutoSequence(robot, motion, this::opModeIsActive);

            selectAlliance();
            waitForStart();

            if (opModeIsActive()) {
                runRoute();
            }
        } finally {
            if (sequence != null) {
                sequence.stopAll();
            }
            robot.close();
        }
    }

    /** Alliance is chosen on the Driver Hub before START; defaults to RED. */
    private void selectAlliance() {
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.a) {
                redAlliance = true;
            }
            if (gamepad1.b) {
                redAlliance = false;
            }

            telemetry.addLine("APRILTAG AUTONOMOUS READY");
            telemetry.addData("A = RED / B = BLUE", redAlliance ? "RED selected" : "BLUE selected");
            telemetry.addData("Audience tag", redAlliance ? redAudienceTag : blueAudienceTag);
            telemetry.addData("Scoring tag", redAlliance ? redScoringTag : blueScoringTag);
            telemetry.addLine("Press START to run.");
            telemetry.update();
            sleep(20);
        }
    }

    private void runRoute() {
        ElapsedTime match = new ElapsedTime();
        int audienceTag = redAlliance ? redAudienceTag : blueAudienceTag;
        int scoringTag = redAlliance ? redScoringTag : blueScoringTag;

        // 1. Drive toward the audience wall and align on its cluster.
        if (!outOfTime(match)) {
            motion.driveInches(audienceLegInches, routePower);
        }
        if (!outOfTime(match)) {
            motion.turnDegrees(audienceTurnDegrees);
        }
        if (!outOfTime(match)) {
            sequence.alignToTag(audienceTag, audienceRangeInches, alignTimeoutSeconds);
        }

        // 2. Reposition across the field toward the scoring side.
        if (!outOfTime(match)) {
            motion.driveInches(-audienceBackoffInches, routePower);
        }
        if (!outOfTime(match)) {
            motion.turnDegrees(scoringTurnDegrees);
        }
        if (!outOfTime(match)) {
            motion.driveInches(scoringLegInches, routePower);
        }

        // 3. Align on the scoring cluster at shooting range.
        boolean aligned = false;
        if (!outOfTime(match)) {
            aligned = sequence.alignToTag(scoringTag, scoringRangeInches, alignTimeoutSeconds);
        }

        // 4. Score the preload.
        if (!outOfTime(match)) {
            double budget = Math.min(shootBudgetSeconds, matchSeconds - match.seconds());
            sequence.shootBalls(AutoSequence.preloadBalls, Math.max(0.0, budget));
        }

        // 5. Park clear of the goal.
        if (!outOfTime(match)) {
            motion.driveInches(parkInches, routePower);
        }

        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Scoring align", aligned ? "ON TARGET" : "TIMED OUT");
        telemetry.addData("AUTO elapsed", "%.1fs / %.0fs", match.seconds(), matchSeconds);
        telemetry.update();
    }

    private boolean outOfTime(ElapsedTime match) {
        return match.seconds() >= lastActionSeconds || !opModeIsActive();
    }
}
