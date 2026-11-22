package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Time-based motion primitives shared by every timed autonomous routine.
 *
 * <p>Distances and headings are converted to run time from ONE set of calibration constants, so
 * the autos cannot drift apart the way they did when each file carried its own copy
 * (0.45 power vs 0.565 power for the same "0.6 s per 90 degrees"). Calibrate
 * {@link #inchesPerSecondAtHalfPower}, {@link #turnPower} and
 * {@link #degreesPerSecondAtTurnPower} once on the field and every route follows.
 *
 * <p>All movement goes through {@link #driveInches}, {@link #turnDegrees} and
 * {@link #driveFor}: they apply battery compensation, always stop the drivetrain on the way out,
 * keep {@code Robot.update()} running (match-loader touch safety) and stay interruptible when the
 * OpMode is stopped.
 *
 * <p>Sign convention: {@code forward > 0} is forward and {@code turn > 0} is clockwise/to the
 * robot's right, matching {@link Robot#drive(double, double)}. SDK yaw/bearing values are the
 * opposite sign; convert them with {@link Robot#SDK_YAW_TO_TURN}.
 */
public final class TimedMotion {

    /** Writes forward/turn powers to whichever drivetrain the OpMode owns. */
    public interface DriveSink {
        void drive(double forward, double turn);
    }

    // ============================================================
    // CALIBRATION - measure these on the field, then update here
    // ============================================================

    /** Ground speed in inches per second at 0.5 power on a fully charged pack. */
    public static double inchesPerSecondAtHalfPower = 20.0;

    /** Power used for every calibrated turn. Keep turn rate and turn power together. */
    public static double turnPower = 0.45;

    /** Heading rate in degrees per second at {@link #turnPower}. */
    public static double degreesPerSecondAtTurnPower = 150.0;

    /** Pack voltage the two constants above were measured at. */
    public static double nominalBatteryVolts = 12.0;

    /** Anything below this is treated as "no reading" rather than a real pack voltage. */
    public static double minBatteryVolts = 1.0;

    private final DriveSink driveSink;
    private final BooleanSupplier opModeActive;
    private final DoubleSupplier batteryVolts;
    private final Runnable periodic;

    /**
     * @param driveSink      writes the mixed forward/turn powers to the motors
     * @param opModeActive   usually {@code this::opModeIsActive}
     * @param batteryVolts   pack voltage supplier; values at or below {@link #minBatteryVolts} fall
     *                       back to {@link #nominalBatteryVolts}
     * @param periodic       per-iteration housekeeping ({@code Robot.update()} plus {@code idle()})
     */
    public TimedMotion(DriveSink driveSink, BooleanSupplier opModeActive,
                       DoubleSupplier batteryVolts, Runnable periodic) {
        this.driveSink = driveSink;
        this.opModeActive = opModeActive;
        this.batteryVolts = batteryVolts;
        this.periodic = periodic;
    }

    /** Adapter for the production {@link Robot} chassis. */
    public static TimedMotion forRobot(Robot robot, LinearOpMode opMode) {
        return new TimedMotion(
                robot::drive,
                opMode::opModeIsActive,
                robot::getBatteryVolts,
                () -> {
                    robot.update();
                    opMode.idle();
                });
    }

    /**
     * Adapter for legacy OpModes that drive raw {@code left}/{@code right} motors. The sink is
     * responsible for its own clipping and motor directions.
     */
    public static TimedMotion forRawMotors(TimedMotion.DriveSink sink, LinearOpMode opMode,
                                           HardwareMap hardwareMap) {
        return new TimedMotion(
                sink,
                opMode::opModeIsActive,
                () -> readBatteryVolts(hardwareMap),
                opMode::idle);
    }

    /** First configured voltage sensor, or {@link #nominalBatteryVolts} when there is none. */
    public static double readBatteryVolts(HardwareMap hardwareMap) {
        try {
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double volts = sensor.getVoltage();
                if (volts > minBatteryVolts) {
                    return volts;
                }
            }
        } catch (Exception ignored) {
            // fall through to nominal
        }
        return nominalBatteryVolts;
    }

    // ============================================================
    // Pure conversion helpers (unit tested, no hardware)
    // ============================================================

    /** Run time needed to cover {@code inches} at {@code power} on a nominal pack. */
    public static double secondsForInches(double inches, double power) {
        if (power <= 0.0) {
            throw new IllegalArgumentException("power must be positive, was " + power);
        }
        double scale = power / 0.5;
        return Math.abs(inches) / (inchesPerSecondAtHalfPower * scale);
    }

    /** Run time needed to turn {@code degrees} at {@code power} on a nominal pack. */
    public static double secondsForDegrees(double degrees, double power) {
        if (power <= 0.0) {
            throw new IllegalArgumentException("power must be positive, was " + power);
        }
        if (turnPower <= 0.0) {
            throw new IllegalStateException("turnPower must be positive, was " + turnPower);
        }
        if (degreesPerSecondAtTurnPower <= 0.0) {
            throw new IllegalStateException("degreesPerSecondAtTurnPower must be positive, was "
                    + degreesPerSecondAtTurnPower);
        }
        double degreesPerSecond = degreesPerSecondAtTurnPower * (power / turnPower);
        return Math.abs(degrees) / degreesPerSecond;
    }

    /**
     * A fuller pack drives faster, so the same distance needs less time.
     * Readings at or below {@link #minBatteryVolts} (including NaN) are ignored.
     */
    public static double compensateForBattery(double seconds, double batteryVolts) {
        double volts = batteryVolts > minBatteryVolts ? batteryVolts : nominalBatteryVolts;
        return seconds * nominalBatteryVolts / volts;
    }

    // ============================================================
    // Motion
    // ============================================================

    /** Current pack voltage, already sanitized. */
    public double batteryVolts() {
        return batteryVolts.getAsDouble();
    }

    /** Runs one housekeeping tick ({@code Robot.update()} + {@code idle()}). */
    public void tick() {
        periodic.run();
    }

    /** Stops the drivetrain. Safe to call when the OpMode is already stopping. */
    public void stop() {
        driveSink.drive(0.0, 0.0);
    }

    /**
     * Raw timed move. Returns false when the OpMode stopped before the time elapsed.
     * The drivetrain is always stopped before returning.
     */
    public boolean driveFor(double forward, double turn, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeActive.getAsBoolean() && timer.seconds() < seconds) {
            driveSink.drive(forward, turn);
            periodic.run();
        }
        stop();
        return opModeActive.getAsBoolean();
    }

    /** Drives {@code inches} (negative = backward) at {@code power}. */
    public boolean driveInches(double inches, double power) {
        double seconds = compensateForBattery(secondsForInches(inches, power), batteryVolts());
        return driveFor(Math.copySign(power, inches), 0.0, seconds);
    }

    /** Turns {@code degrees} at the calibrated {@link #turnPower}; negative = counter-clockwise. */
    public boolean turnDegrees(double degrees) {
        return turnDegrees(degrees, turnPower);
    }

    /** Turns {@code degrees} at an explicit power; negative = counter-clockwise. */
    public boolean turnDegrees(double degrees, double power) {
        double seconds = compensateForBattery(secondsForDegrees(degrees, power), batteryVolts());
        return driveFor(0.0, Math.copySign(power, degrees), seconds);
    }

    /** Waits while keeping {@code Robot.update()}/idle alive. False if the OpMode stopped. */
    public boolean waitSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeActive.getAsBoolean() && timer.seconds() < seconds) {
            periodic.run();
        }
        return opModeActive.getAsBoolean();
    }
}
