package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

/**
 * Drives the robot toward the nearest yellow ball using live camera measurements.
 *
 * <p>Every loop iteration re-reads a {@link YellowBallVision.Detection} and feeds its range and
 * bearing straight into {@link Robot#drive}: range produces forward power (slowing to
 * {@link #grabDistanceInches}), bearing produces turn power. Because the bearing sign from
 * {@link YellowBallVision} is positive to the robot's right - the same sign as a positive turn in
 * {@code drive} - no sign conversion is needed here.
 *
 * <p>When the ball disappears the controller coasts toward the last heading for
 * {@link #lostGraceSeconds} before giving up; {@link #searchForBall} sweeps a bounded arc to
 * reacquire it. Both phases always stop the drivetrain before returning.
 */
public final class YellowBallChase {

    /** Receives one update per control-loop iteration, for live telemetry. */
    public interface Listener {
        void onLoop(YellowBallVision.Detection target, double forward, double turn);
    }

    /** How a chase ended. */
    public enum Result {
        /** Close enough and pointed at the ball. */
        REACHED,
        /** Ball out of view beyond the grace period. */
        LOST,
        /** Ran out of time. */
        TIMEOUT,
        /** OpMode stopped. */
        STOPPED
    }

    // ============================================================
    // Control gains - CALIBRATE on the robot
    // ============================================================

    /** Range at which the robot stops so the intake can grab the ball. */
    public static double grabDistanceInches = 8.0;

    /** Final approach range once the intake is running; the ball is eaten here. */
    public static double finalApproachDistanceInches = 3.0;

    /** Acceptable range error around the target distance. */
    public static double grabToleranceInches = 2.0;

    /** Acceptable heading error before the target counts as reached. */
    public static double grabBearingDegrees = 8.0;

    /** Forward power per inch of range error. */
    public static double driveGain = 0.07;

    /** Turn power per degree of bearing error. */
    public static double turnGain = 0.02;

    public static double maxDrivePower = 0.50;
    public static double maxTurnPower = 0.30;

    /** Floor on forward power so the robot does not stall on carpet short of the ball. */
    public static double minDrivePower = 0.14;

    /** Keep driving this long after the ball is lost, aiming along the last heading. */
    public static double lostGraceSeconds = 0.60;

    /** Cruise power while coasting after a lost ball. */
    public static double lostDrivePower = 0.15;

    /** Turn power used by {@link #searchForBall}. */
    public static double sweepPower = 0.18;

    /** Seconds per sweep leg before reversing direction. */
    public static double sweepLegSeconds = 1.20;

    private final Robot robot;
    private final YellowBallVision vision;
    private final BooleanSupplier opModeActive;
    private final Runnable tick;
    private final Listener listener;

    private YellowBallVision.Detection lastTarget;
    private double lastForward;
    private double lastTurn;

    /**
     * @param robot        drivetrain and mechanisms
     * @param vision       yellow-ball camera wrapper
     * @param opModeActive usually {@code this::opModeIsActive}
     * @param tick         per-iteration housekeeping ({@code Robot.update()} plus {@code idle()})
     * @param listener     optional live-telemetry callback, may be null
     */
    public YellowBallChase(Robot robot, YellowBallVision vision, BooleanSupplier opModeActive,
                           Runnable tick, Listener listener) {
        this.robot = robot;
        this.vision = vision;
        this.opModeActive = opModeActive;
        this.tick = tick;
        this.listener = listener;
    }

    /**
     * Sweeps left and right until a ball appears.
     *
     * @return true when a ball is visible
     */
    public boolean searchForBall(double maxSeconds) {
        ElapsedTime timer = new ElapsedTime();
        double legEndsAt = sweepLegSeconds;
        double direction = 1.0;

        try {
            while (opModeActive.getAsBoolean() && timer.seconds() < maxSeconds) {
                if (vision.bestTarget() != null) {
                    return true;
                }

                robot.drive(0.0, sweepPower * direction);
                tick.run();

                if (timer.seconds() >= legEndsAt) {
                    direction = -direction;
                    legEndsAt = timer.seconds() + sweepLegSeconds;
                }
            }
            return vision.bestTarget() != null;
        } finally {
            stop();
        }
    }

    /** Chase the nearest ball until it is grabbable, lost or out of time. */
    public Result chase(double maxSeconds) {
        return chase(maxSeconds, grabDistanceInches);
    }

    /**
     * Chase the nearest ball down to {@code targetDistanceInches}.
     *
     * <p>Pass {@link #finalApproachDistanceInches} for the last creep with the intake running, so
     * the intake sweeps the ball up instead of stopping short of it.
     */
    public Result chase(double maxSeconds, double targetDistanceInches) {
        ElapsedTime total = new ElapsedTime();
        ElapsedTime sinceSeen = new ElapsedTime();
        sinceSeen.reset();

        try {
            while (opModeActive.getAsBoolean() && total.seconds() < maxSeconds) {
                YellowBallVision.Detection target = vision.bestTarget();

                if (target != null) {
                    sinceSeen.reset();
                    lastTarget = target;

                    if (isGrabbable(target, targetDistanceInches)) {
                        lastForward = 0.0;
                        lastTurn = 0.0;
                        notifyListener(target, 0.0, 0.0);
                        return Result.REACHED;
                    }

                    double forward = forwardFor(target.distanceInches, targetDistanceInches);
                    double turn = turnFor(target.bearingDegrees);

                    lastForward = forward;
                    lastTurn = turn;
                    robot.drive(forward, turn);
                    notifyListener(target, forward, turn);
                } else if (sinceSeen.seconds() < lostGraceSeconds) {
                    // Coast toward where the ball was instead of spinning in place.
                    lastForward = lostDrivePower;
                    lastTurn = 0.0;
                    robot.drive(lastForward, lastTurn);
                    notifyListener(null, lastForward, lastTurn);
                } else {
                    return Result.LOST;
                }

                tick.run();
            }
            return opModeActive.getAsBoolean() ? Result.TIMEOUT : Result.STOPPED;
        } finally {
            stop();
        }
    }

    /** Stops the drivetrain. Safe to call when the OpMode is already stopping. */
    public void stop() {
        robot.drive(0.0, 0.0);
        lastForward = 0.0;
        lastTurn = 0.0;
    }

    public YellowBallVision.Detection getLastTarget() {
        return lastTarget;
    }

    public double getLastForward() {
        return lastForward;
    }

    public double getLastTurn() {
        return lastTurn;
    }

    // ============================================================
    // Control law (pure - unit testable)
    // ============================================================

    /** True when the ball is inside the grab window for {@link #grabDistanceInches}. */
    public static boolean isGrabbable(YellowBallVision.Detection target) {
        return isGrabbable(target, grabDistanceInches);
    }

    /**
     * True when the ball is inside the grab window for {@code targetDistanceInches}.
     * A ball closer than half the target is treated as reached even if the bearing is off - it is
     * under the camera and the bearing no longer means anything.
     */
    public static boolean isGrabbable(YellowBallVision.Detection target, double targetDistanceInches) {
        if (target == null) {
            return false;
        }
        boolean rangeOk = target.distanceInches <= targetDistanceInches + grabToleranceInches;
        boolean onTopOfIt = target.distanceInches <= 0.5 * targetDistanceInches;
        return onTopOfIt || (rangeOk && Math.abs(target.bearingDegrees) <= grabBearingDegrees);
    }

    /** Forward power for a measured range, aiming at {@link #grabDistanceInches}. */
    public static double forwardFor(double distanceInches) {
        return forwardFor(distanceInches, grabDistanceInches);
    }

    /**
     * Forward power for a measured range: proportional, with a floor so the robot keeps closing
     * instead of stalling on carpet short of the ball.
     */
    public static double forwardFor(double distanceInches, double targetDistanceInches) {
        if (distanceInches <= targetDistanceInches) {
            return 0.0;
        }
        double power = (distanceInches - targetDistanceInches) * driveGain;
        power = Math.min(power, maxDrivePower);
        return Math.max(power, Math.min(minDrivePower, maxDrivePower));
    }

    /** Turn power for a measured bearing (positive bearing = ball to the right). */
    public static double turnFor(double bearingDegrees) {
        double power = bearingDegrees * turnGain;
        return Math.max(-maxTurnPower, Math.min(maxTurnPower, power));
    }

    private void notifyListener(YellowBallVision.Detection target, double forward, double turn) {
        if (listener != null) {
            listener.onLoop(target, forward, turn);
        }
    }
}
