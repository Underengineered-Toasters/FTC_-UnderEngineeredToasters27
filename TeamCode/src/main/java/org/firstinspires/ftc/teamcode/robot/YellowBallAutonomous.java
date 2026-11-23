package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Autonomous: find the nearest yellow ball with the C270 webcam and drive to it.
 *
 * <p>Flow: open the camera, wait for the stream, sweep for a ball, chase it with live range and
 * bearing updates from {@link YellowBallChase}, run the intake once it is grabbable, and repeat
 * once if the ball is lost. Every phase is time-guarded and the whole route stays inside the match
 * window.
 *
 * <p>{@code Robot.init(hardwareMap, false)} keeps the AprilTag portal closed so this OpMode owns
 * the webcam; two portals cannot hold the same camera.
 */
@Autonomous(name = "Yellow Ball Autonomous", group = "Autonomous")
public class YellowBallAutonomous extends LinearOpMode {

    // ============================================================
    // Route - CALIBRATE on the field
    // ============================================================

    /** How long to wait for the camera to start streaming before giving up. */
    public static double streamTimeoutSeconds = 4.0;

    /** Sweep time when no ball is visible. */
    public static double searchSeconds = 3.0;

    /** Longest single chase. */
    public static double chaseSeconds = 12.0;

    /** How long the intake runs to pull in a grabbable ball. */
    public static double captureSeconds = 1.2;

    /** Search/chase rounds before the OpMode stops. */
    public static int chaseAttempts = 2;

    // Match guard
    public static double matchSeconds = 30.0;

    /** No new phase starts at or after this elapsed time. */
    public static double lastActionSeconds = 25.0;

    /** Minimum time between Driver Hub telemetry pushes while chasing. */
    public static double telemetryIntervalSeconds = 0.10;

    private final Robot robot = new Robot();
    private final YellowBallVision vision = new YellowBallVision();

    private YellowBallChase chase;
    private ElapsedTime match;
    private ElapsedTime lastTelemetry;

    @Override
    public void runOpMode() {
        try {
            robot.init(hardwareMap, false);
            vision.init(hardwareMap);

            chase = new YellowBallChase(robot, vision, this::opModeIsActive,
                    this::tick, this::publishChase);

            telemetry.addLine("YELLOW BALL AUTONOMOUS");
            telemetry.addData("Camera", YellowBallVision.cameraName);
            telemetry.addData("Status", vision.getStatus());
            telemetry.addData("Ball diameter", "%.2f in", YellowBallVision.ballDiameterInches);
            telemetry.addData("Focal length", "%.1f px", YellowBallVision.focalPixels());
            telemetry.update();

            waitForStart();

            if (opModeIsActive()) {
                runRoute();
            }
        } finally {
            finish();
        }
    }

    private void runRoute() {
        match = new ElapsedTime();

        if (!waitForStream()) {
            telemetry.addLine("Camera never started streaming; stopping.");
            telemetry.update();
            return;
        }

        int attempt = 0;
        boolean collected = false;

        while (opModeIsActive() && attempt < chaseAttempts && !outOfTime()) {
            attempt++;

            if (vision.bestTarget() == null
                    && !chase.searchForBall(Math.min(searchSeconds, remainingSeconds()))) {
                telemetry.addData("Search " + attempt, "no ball found");
                telemetry.update();
                continue;
            }

            double budget = Math.min(chaseSeconds, remainingSeconds());
            YellowBallChase.Result result = chase.chase(budget);

            switch (result) {
                case REACHED:
                    collected = captureBall();
                    if (collected) {
                        attempt = chaseAttempts;
                    }
                    break;
                case LOST:
                    telemetry.addData("Chase " + attempt, "ball lost");
                    break;
                case TIMEOUT:
                    telemetry.addData("Chase " + attempt, "timed out");
                    break;
                default:
                    break;
            }
            telemetry.update();
        }

        robot.intakeSystem.stopIntake();
        telemetry.addData("Result", collected ? "BALL COLLECTED" : "NO BALL");
        telemetry.addData("Elapsed", "%.1fs / %.0fs", match.seconds(), matchSeconds);
        telemetry.update();
    }

    /** Runs the intake while creeping the last few inches so the roller sweeps the ball up. */
    private boolean captureBall() {
        if (outOfTime()) {
            return false;
        }

        robot.intakeSystem.intakeIn();
        YellowBallChase.Result result = chase.chase(
                Math.min(captureSeconds, remainingSeconds()),
                YellowBallChase.finalApproachDistanceInches);
        robot.intakeSystem.stopIntake();

        // Losing the ball during the final creep is the normal "it went into the intake" signature.
        return result == YellowBallChase.Result.REACHED
                || result == YellowBallChase.Result.LOST;
    }

    /** Blocks until frames are flowing, so the first sweep does not chase an empty frame list. */
    private boolean waitForStream() {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < streamTimeoutSeconds) {
            if (vision.isStreaming()) {
                return true;
            }
            telemetry.addData("Status", vision.getStatus());
            telemetry.update();
            tick();
        }
        return vision.isStreaming();
    }

    private void tick() {
        robot.update();
        idle();
    }

    /** Live chase telemetry: the measured range/bearing and the powers they produced. */
    private void publishChase(YellowBallVision.Detection target, double forward, double turn) {
        // Roughly 10 Hz: the chase loop runs far faster than the Driver Hub can display.
        if (lastTelemetry != null && lastTelemetry.seconds() < telemetryIntervalSeconds) {
            return;
        }
        lastTelemetry = new ElapsedTime();

        if (target != null) {
            telemetry.addData("Target", "%.1f in at %.1f deg", target.distanceInches,
                    target.bearingDegrees);
            telemetry.addData("Blob", "r=%.1f px, area=%.0f", target.radiusPixels,
                    target.contourArea);
        } else {
            telemetry.addData("Target", "lost - coasting");
        }
        telemetry.addData("Drive / turn", "%.2f / %.2f", forward, turn);
        telemetry.addData("FPS", "%.1f", vision.getFps());
        telemetry.update();
    }

    private boolean outOfTime() {
        return (match != null && match.seconds() >= lastActionSeconds) || !opModeIsActive();
    }

    private double remainingSeconds() {
        if (match == null) {
            return chaseSeconds;
        }
        return Math.max(0.0, Math.min(matchSeconds, lastActionSeconds) - match.seconds());
    }

    private void finish() {
        if (chase != null) {
            chase.stop();
        }
        robot.intakeSystem.stopIntake();
        robot.spinDownShooters();
        vision.close();
        robot.close();
        telemetry.addLine("Yellow ball autonomous finished; camera closed.");
        telemetry.update();
    }
}
