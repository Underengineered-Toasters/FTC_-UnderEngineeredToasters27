package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.AutoSequence;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.TimedMotion;

/**
 * BIOBUZZ timed autonomous template.
 *
 * <p>Distance, turn and battery handling live in {@link TimedMotion}; feeding the preload lives in
 * {@link AutoSequence#shootBalls}. Previously this class only spun the flywheels for 2.8 s and
 * never released a ball, and it never called {@code Robot.update()}, so the match-loader touch
 * safeguard could not fire. Both are fixed here.
 *
 * <p>Route shape is unchanged: optional 7 s wait, score the preload, drive to the flower and
 * intake, then either return and score again or cross and park.
 */
public abstract class BioBuzzTimedAuto extends LinearOpMode {
    protected final Robot robot = new Robot();

    // ============================================================
    // Route shape - CALIBRATE on the field
    // ============================================================

    /** Travel power for the 28 in flower leg. */
    public static double MOVE_POWER = 0.42;

    /** Cross/park legs use this fraction of {@link #MOVE_POWER}. */
    public static double PARK_POWER_SCALE = 0.80;

    /** Distance from the start to the alliance FLOWER, in inches. */
    public static double FLOWER_LEG_INCHES = 28.0;

    /** Cross-field park leg, in inches. */
    public static double PARK_LEG_INCHES = 26.0;

    /** How long the intake runs at the flower. */
    public static double INTAKE_SECONDS = 2.4;

    /** Delay before the first shot for the WAIT routes. */
    public static double WAIT_BEFORE_SHOOT_SECONDS = 7.0;

    /** Extra settle time for the SHOOT_RETURN_WAIT_SHOOT route. */
    public static double WAIT_AFTER_RETURN_SECONDS = 2.0;

    /** Balls in a full preload. */
    public static int PRELOAD_BALLS = 4;

    /** Time budget for spin-up plus feeding every preload ball. */
    public static double PRELOAD_SHOOT_SECONDS = 4.0;

    /** Time budget for the second scoring pass after returning. */
    public static double SECOND_SHOT_SECONDS = 3.0;

    // ============================================================
    // Match guard - never start a shot that would run past the buzzer
    // ============================================================

    /** Legal match length. */
    public static double MATCH_SECONDS = 30.0;

    /** Do not start a second scoring pass at or after this elapsed time. */
    public static double SECOND_SHOT_EARLIEST = 27.0;

    /** Hard stop for any shot still running at this elapsed time. */
    public static double LAST_SHOT_DEADLINE = 29.0;

    private TimedMotion motion;
    private AutoSequence sequence;

    protected enum Plan {
        WAIT_SHOOT_RETURN_SHOOT,
        SHOOT_RETURN_WAIT_SHOOT,
        WAIT_SHOOT_CROSS_PARK,
        SHOOT_CROSS_PARK
    }

    protected abstract boolean isRed();
    protected abstract Plan plan();

    @Override public final void runOpMode() {
        try {
            robot.init(hardwareMap);
            motion = TimedMotion.forRobot(robot, this);
            sequence = new AutoSequence(robot, motion, this::opModeIsActive);

            telemetry.addLine("BIOBUZZ timed auto - calibrate distance and turns before use");
            telemetry.addData("Alliance", isRed() ? "RED" : "BLUE");
            telemetry.addData("Route", plan());
            telemetry.addLine("Start aimed toward the alliance FLOWER");
            telemetry.update();

            waitForStart();
            if (!opModeIsActive()) {
                return;
            }

            Plan selected = plan();
            ElapsedTime match = new ElapsedTime();

            if (startsByWaiting(selected)) {
                motion.waitSeconds(WAIT_BEFORE_SHOOT_SECONDS);
            }

            // Up to four preloaded POLLEN.
            sequence.shootBalls(PRELOAD_BALLS, PRELOAD_SHOOT_SECONDS);
            sequence.intakeAtFlower(INTAKE_SECONDS);

            if (returnsToShoot(selected)) {
                motion.turnDegrees(180.0);
                motion.driveInches(FLOWER_LEG_INCHES, MOVE_POWER);
                motion.turnDegrees(180.0);

                if (selected == Plan.SHOOT_RETURN_WAIT_SHOOT) {
                    motion.waitSeconds(WAIT_AFTER_RETURN_SECONDS);
                }

                if (match.seconds() < SECOND_SHOT_EARLIEST) {
                    double budget = Math.min(SECOND_SHOT_SECONDS,
                            LAST_SHOT_DEADLINE - match.seconds());
                    sequence.shootBalls(PRELOAD_BALLS, Math.max(0.0, budget));
                }
            } else {
                crossAndPark();
            }

            telemetry.addData("AUTO elapsed", "%.1fs / %.0fs", match.seconds(), MATCH_SECONDS);
            telemetry.update();
        } finally {
            if (sequence != null) {
                sequence.stopAll();
            }
            robot.close();
        }
    }

    private static boolean startsByWaiting(Plan plan) {
        return plan == Plan.WAIT_SHOOT_RETURN_SHOOT || plan == Plan.WAIT_SHOOT_CROSS_PARK;
    }

    private static boolean returnsToShoot(Plan plan) {
        return plan == Plan.WAIT_SHOOT_RETURN_SHOOT || plan == Plan.SHOOT_RETURN_WAIT_SHOOT;
    }

    private void crossAndPark() {
        motion.turnDegrees(isRed() ? -90.0 : 90.0);
        motion.driveInches(PARK_LEG_INCHES, MOVE_POWER * PARK_POWER_SCALE);
        motion.turnDegrees(isRed() ? 90.0 : -90.0);
    }
}
