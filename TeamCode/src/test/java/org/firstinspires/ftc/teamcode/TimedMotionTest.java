package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.TimedMotion;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Locks the single calibration contract every autonomous route depends on: inches and degrees
 * convert to seconds in one place, and every move applies battery compensation and ends stopped.
 */
public class TimedMotionTest {

    /** Records what a fake drivetrain was commanded. */
    private static final class Recorder implements TimedMotion.DriveSink {
        final List<double[]> commands = new ArrayList<>();

        @Override
        public void drive(double forward, double turn) {
            commands.add(new double[]{forward, turn});
        }

        double[] last() {
            return commands.get(commands.size() - 1);
        }
    }

    private static TimedMotion motion(Recorder recorder, AtomicBoolean active, double volts) {
        return new TimedMotion(recorder, active::get, () -> volts, () -> { });
    }

    @Test
    public void inchesConvertToSecondsAtHalfPower() {
        assertEquals(1.0, TimedMotion.secondsForInches(20.0, 0.5), 1e-9);
        assertEquals(2.0, TimedMotion.secondsForInches(40.0, 0.5), 1e-9);
        // Sign of the distance must not change the duration; direction is applied to the power.
        assertEquals(1.0, TimedMotion.secondsForInches(-20.0, 0.5), 1e-9);
        // Full power is twice as fast as half power.
        assertEquals(0.5, TimedMotion.secondsForInches(20.0, 1.0), 1e-9);
    }

    @Test
    public void degreesConvertToSecondsAtCalibratedTurnPower() {
        assertEquals(0.6, TimedMotion.secondsForDegrees(90.0, TimedMotion.turnPower), 1e-9);
        assertEquals(1.2, TimedMotion.secondsForDegrees(180.0, TimedMotion.turnPower), 1e-9);
        assertEquals(1.2, TimedMotion.secondsForDegrees(-180.0, TimedMotion.turnPower), 1e-9);
        // Half the turn power takes twice as long for the same heading change.
        assertEquals(1.2, TimedMotion.secondsForDegrees(90.0, TimedMotion.turnPower / 2.0), 1e-9);
    }

    @Test
    public void batteryCompensationShortensRunsOnAFullerPack() {
        assertEquals(1.0, TimedMotion.compensateForBattery(1.0, 12.0), 1e-9);
        assertEquals(12.0 / 14.4, TimedMotion.compensateForBattery(1.0, 14.4), 1e-9);
        assertEquals(12.0 / 11.0, TimedMotion.compensateForBattery(1.0, 11.0), 1e-9);
    }

    @Test
    public void unusableVoltageFallsBackToNominal() {
        assertEquals(1.0, TimedMotion.compensateForBattery(1.0, 0.0), 1e-9);
        assertEquals(1.0, TimedMotion.compensateForBattery(1.0, Double.NaN), 1e-9);
        assertEquals(1.0, TimedMotion.compensateForBattery(1.0, -5.0), 1e-9);
    }

    @Test
    public void driveInchesAppliesDirectionToPowerAndStops() {
        Recorder recorder = new Recorder();
        TimedMotion motion = motion(recorder, new AtomicBoolean(true), 12.0);

        assertTrue(motion.driveInches(-4.0, 0.3));
        // Backward run is a negative forward power; the turn axis stays neutral.
        assertEquals(-0.3, recorder.commands.get(0)[0], 1e-9);
        assertEquals(0.0, recorder.commands.get(0)[1], 1e-9);
        assertEquals(0.0, recorder.last()[0], 1e-9);
        assertEquals(0.0, recorder.last()[1], 1e-9);
    }

    @Test
    public void turnDegreesAppliesDirectionToPower() {
        Recorder recorder = new Recorder();
        TimedMotion motion = motion(recorder, new AtomicBoolean(true), 12.0);

        assertTrue(motion.turnDegrees(-90.0));
        // Counter-clockwise is a negative turn on this chassis.
        assertEquals(0.0, recorder.commands.get(0)[0], 1e-9);
        assertEquals(-TimedMotion.turnPower, recorder.commands.get(0)[1], 1e-9);
    }

    @Test
    public void moveAbortsAndStopsWhenTheOpModeEnds() {
        Recorder recorder = new Recorder();
        AtomicBoolean active = new AtomicBoolean(false);
        TimedMotion motion = motion(recorder, active, 12.0);

        assertFalse(motion.driveInches(10.0, 0.4));
        assertEquals(0.0, recorder.last()[0], 1e-9);
        assertFalse(motion.waitSeconds(1.0));
    }
}
