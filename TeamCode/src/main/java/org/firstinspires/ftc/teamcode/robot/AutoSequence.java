package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.BooleanSupplier;

/**
 * Autonomous-only actions shared by every timed route: settling the turret, feeding the
 * flywheels, intaking at the flower, AprilTag alignment and a safe stop.
 *
 * <p>Two things every auto used to get wrong live here:
 * <ul>
 *   <li>{@link #shootBalls} spins the flywheels up <em>before</em> feeding, then feeds one ball at
 *       a time. The old {@code shoot()} only ran the flywheels and never released a ball, so the
 *       autonomous routines could not score.</li>
 *   <li>{@link #alignToTag} bounds the search sweep and converts the SDK bearing with
 *       {@link Robot#SDK_YAW_TO_TURN}. {@code ftcPose.bearing} is positive to the LEFT of the
 *       camera (the SDK computes {@code atan2(-x, z)}), while this chassis turns clockwise for a
 *       positive turn, so feeding the raw bearing into {@code drive} would steer away from the
 *       tag.</li>
 * </ul>
 *
 * <p>Everything runs through {@link TimedMotion}, so {@code Robot.update()} keeps running
 * (match-loader touch safety) and every action stays interruptible.
 */
public final class AutoSequence {

    // ============================================================
    // CALIBRATION - verify against the real mechanism before competing
    // ============================================================

    /**
     * True when the match loader is the feed: pulling it down pushes one ball into the
     * flywheels and raising it re-arms. If the mechanism feeds in the opposite position, flip
     * the two positions below instead of editing the sequence.
     */
    public static boolean loaderIsFeed = true;

    /** Balls a full load holds (preload). */
    public static int preloadBalls = 4;

    /** Time for the flywheels to reach speed before the first ball is released. */
    public static double spinUpSeconds = 0.70;

    /** Time the loader holds the feed position to release one ball. */
    public static double feedSeconds = 0.45;

    /** Time to re-arm between balls. */
    public static double retractSeconds = 0.30;

    /** Loader travel time when deploying/retracting around the intake. */
    public static double loaderSettleSeconds = 0.35;

    /** Turret settle time and tolerance before shooting. */
    public static double turretSettleSeconds = 1.20;

    // Alignment
    public static double desiredRangeInches = 12.0;
    public static double rangeGain = 0.03;
    public static double bearingGain = 0.02;
    public static double maxAlignDrive = 0.40;
    public static double maxAlignTurn = 0.30;
    public static double rangeToleranceInches = 1.5;
    public static double bearingToleranceDegrees = 2.0;
    public static double scanTurnPower = 0.15;
    public static double scanSweepSeconds = 1.50;

    private final Robot robot;
    private final TimedMotion motion;
    private final BooleanSupplier opModeActive;

    public AutoSequence(Robot robot, TimedMotion motion, BooleanSupplier opModeActive) {
        this.robot = robot;
        this.motion = motion;
        this.opModeActive = opModeActive;
    }

    // ============================================================
    // Turret
    // ============================================================

    /**
     * Returns the turret to its init heading (encoder zero) so the shot goes where the chassis
     * points. Returns true once it is settled or the budget expires.
     */
    public boolean settleTurret(double maxSeconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeActive.getAsBoolean() && timer.seconds() < maxSeconds) {
            if (robot.turretSystem.holdInitPosition()) {
                return true;
            }
            motion.tick();
        }
        robot.turretSystem.stopTurret();
        return robot.turretSystem.holdInitPosition();
    }

    // ============================================================
    // Scoring
    // ============================================================

    /**
     * Scores up to {@code balls} from the hopper: settles the turret, spins the flywheels up,
     * then feeds one ball per cycle. Stops early when {@code maxSeconds} runs out so a long
     * alignment cannot push the shot past the end of the match.
     *
     * <p>The budget covers spin-up plus feeding; the bounded turret settle happens first and is
     * not counted against it.
     *
     * @return true when the OpMode is still active at the end
     */
    public boolean shootBalls(int balls, double maxSeconds) {
        if (balls <= 0 || !opModeActive.getAsBoolean()) {
            return opModeActive.getAsBoolean();
        }

        settleTurret(turretSettleSeconds);
        if (!opModeActive.getAsBoolean()) {
            return false;
        }

        ElapsedTime budget = new ElapsedTime();

        robot.spinUpShooters();
        try {
            if (!motion.waitSeconds(spinUpSeconds)) {
                return false;
            }

            for (int i = 0; i < balls; i++) {
                if (!opModeActive.getAsBoolean() || budget.seconds() >= maxSeconds) {
                    break;
                }
                robot.setMatchLoaderDown(loaderIsFeed);
                if (!motion.waitSeconds(feedSeconds)) {
                    return false;
                }
                robot.setMatchLoaderDown(!loaderIsFeed);
                if (!motion.waitSeconds(retractSeconds)) {
                    return false;
                }
            }

            return opModeActive.getAsBoolean();
        } finally {
            // Never leave a flywheel spinning, whatever ended the sequence.
            robot.spinDownShooters();
            robot.setMatchLoaderDown(false);
        }
    }

    /**
     * Deploys the loader, intakes for {@code seconds}, then retracts. The touch-sensor safeguard
     * runs because {@link TimedMotion} calls {@code Robot.update()} every iteration.
     */
    public boolean intakeAtFlower(double seconds) {
        robot.setMatchLoaderDown(true);
        if (!motion.waitSeconds(loaderSettleSeconds)) {
            return false;
        }

        robot.intakeSystem.intakeIn();
        boolean stillRunning = motion.waitSeconds(seconds);
        robot.intakeSystem.stopIntake();

        robot.setMatchLoaderDown(false);
        motion.waitSeconds(loaderSettleSeconds);
        return stillRunning;
    }

    // ============================================================
    // Vision
    // ============================================================

    /**
     * Closest visible detection that belongs to {@code tagId}'s cluster, with a usable pose.
     * Because {@link AprilTagTargets#matchesId} matches the containing BIOBUZZ cluster, passing an
     * alliance-specific id also filters out the other alliance's clusters.
     */
    public AprilTagDetection nearestForTag(int tagId) {
        AprilTagDetection best = null;
        for (AprilTagDetection detection : robot.getAprilTagDetections()) {
            if (detection == null || detection.ftcPose == null) {
                continue;
            }
            if (!AprilTagTargets.matchesId(detection, tagId)) {
                continue;
            }
            if (best == null || detection.ftcPose.range < best.ftcPose.range) {
                best = detection;
            }
        }
        return best;
    }

    /**
     * Closed-loop alignment on range and bearing at {@link #desiredRangeInches}.
     *
     * @return true when aligned within {@link #rangeToleranceInches} and
     *         {@link #bearingToleranceDegrees}
     */
    public boolean alignToTag(int tagId, double timeoutSeconds) {
        return alignToTag(tagId, desiredRangeInches, timeoutSeconds);
    }

    /**
     * Closed-loop alignment on range and bearing. Searches with a bounded sweep (camera-side, then
     * the other side) and never spins for the whole timeout. Always stops the drivetrain.
     *
     * @param desiredRange target range in inches; use the shooting range here, not just the
     *                     approach range, so the robot does not park inside the goal
     * @return true when aligned within {@link #rangeToleranceInches} of {@code desiredRange} and
     *         {@link #bearingToleranceDegrees} of the target bearing
     */
    public boolean alignToTag(int tagId, double desiredRange, double timeoutSeconds) {
        ElapsedTime timeout = new ElapsedTime();
        ElapsedTime sweep = new ElapsedTime();

        try {
            while (opModeActive.getAsBoolean() && timeout.seconds() < timeoutSeconds) {
                AprilTagDetection target = nearestForTag(tagId);

                if (target != null) {
                    double rangeError = target.ftcPose.range - desiredRange;
                    double bearingError = target.ftcPose.bearing;

                    if (Math.abs(rangeError) <= rangeToleranceInches
                            && Math.abs(bearingError) <= bearingToleranceDegrees) {
                        return true;
                    }

                    double forward = clamp(rangeError * rangeGain, maxAlignDrive);
                    double turn = clamp(Robot.SDK_YAW_TO_TURN * bearingError * bearingGain,
                            maxAlignTurn);
                    robot.drive(forward, turn);
                } else if (sweep.seconds() < scanSweepSeconds) {
                    // Unknown side: sweep one way first.
                    robot.drive(0.0, scanTurnPower);
                } else if (sweep.seconds() < 2.0 * scanSweepSeconds) {
                    // Then sweep back the other way before giving up.
                    robot.drive(0.0, -scanTurnPower);
                } else {
                    return false;
                }

                motion.tick();
            }
            return false;
        } finally {
            robot.drive(0.0, 0.0);
        }
    }

    // ============================================================
    // Shutdown
    // ============================================================

    /** Stops every mechanism. Safe when the OpMode is already stopping. */
    public void stopAll() {
        robot.drive(0.0, 0.0);
        robot.intakeSystem.stopIntake();
        robot.spinDownShooters();
        robot.turretSystem.stopTurret();
        robot.setMatchLoaderDown(false);
    }

    private static double clamp(double value, double limit) {
        return Math.max(-limit, Math.min(limit, value));
    }
}
