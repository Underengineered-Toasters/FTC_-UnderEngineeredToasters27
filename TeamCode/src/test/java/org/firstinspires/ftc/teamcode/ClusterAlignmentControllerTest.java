package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class ClusterAlignmentControllerTest {
    private static long ns(long ms) { return ms * 1_000_000L; }
    private ClusterAlignmentController controller() { return new ClusterAlignmentController(0, 0); }
    private ClusterAlignmentController.Sample sample(long ms, double bearing) {
        return new ClusterAlignmentController.Sample(bearing, 48, ns(ms));
    }
    private ClusterAlignmentController.Output frame(ClusterAlignmentController c, long ms, double bearing) {
        return c.update(ns(ms), true, false, true, sample(ms, bearing));
    }

    @Test public void requiresDistinctFramesAndSettlingTime() {
        ClusterAlignmentController c = controller();
        for (int ms = 0; ms < 200; ms += 50) {
            assertEquals(ClusterAlignmentController.State.SETTLING, frame(c, ms, 0).state);
        }
        assertEquals(ClusterAlignmentController.State.ALIGNED, frame(c, 200, 0).state);
    }

    @Test public void repeatedFrameCannotManufactureLock() {
        ClusterAlignmentController c = controller();
        for (int ms = 0; ms <= 200; ms += 20) {
            ClusterAlignmentController.Output o = c.update(ns(ms), true, false, true, sample(0, 0));
            assertEquals(1, o.stableFrames);
            assertNotEquals(ClusterAlignmentController.State.ALIGNED, o.state);
        }
    }

    @Test public void fiveFastFramesAreNotEnough() {
        ClusterAlignmentController c = controller();
        for (int ms = 0; ms <= 40; ms += 10) assertNotEquals(
                ClusterAlignmentController.State.ALIGNED, frame(c, ms, 0).state);
    }

    @Test public void targetLossStopsAndRequiresNewStabilityWindow() {
        ClusterAlignmentController c = controller();
        frame(c, 0, 0); frame(c, 50, 0); frame(c, 100, 0);
        ClusterAlignmentController.Output lost = c.update(ns(120), true, false, true, null);
        assertEquals(0, lost.turnPower, 0);
        assertEquals(ClusterAlignmentController.State.TARGET_MISSING, lost.state);
        assertEquals(1, frame(c, 150, 0).stableFrames);
    }

    @Test public void stalledFrameStopsSteering() {
        ClusterAlignmentController c = controller();
        assertTrue(frame(c, 0, 15).turnPower > 0);
        ClusterAlignmentController.Output stale = c.update(ns(251), true, false, false, null);
        assertEquals(ClusterAlignmentController.State.STALE_FRAME, stale.state);
        assertEquals(0, stale.turnPower, 0);
    }

    @Test public void oldAndFutureCapturesAreRejected() {
        ClusterAlignmentController c = new ClusterAlignmentController(ns(100), 0);
        assertEquals(ClusterAlignmentController.State.STALE_FRAME,
                c.update(ns(100), true, false, true, sample(50, 0)).state);
        assertEquals(ClusterAlignmentController.State.STALE_FRAME,
                c.update(ns(100), true, false, true, sample(150, 0)).state);
    }

    @Test public void nonStreamingCameraNeverCommandsMotion() {
        ClusterAlignmentController c = controller();
        assertEquals(0, c.update(0, false, false, true, sample(0, 25)).turnPower, 0);
        assertEquals(ClusterAlignmentController.State.CAMERA_FAILED,
                c.update(0, false, true, false, null).state);
    }

    @Test public void timeoutCannotBeOverriddenByAGoodFrame() {
        assertEquals(ClusterAlignmentController.State.TIMED_OUT,
                frame(controller(), 5000, 0).state);
    }

    @Test public void steeringMatchesCurrentChassisMappingAndIsBounded() {
        assertEquals(0.25, frame(controller(), 0, 90).turnPower, 0);
        assertEquals(-0.25, frame(controller(), 0, -90).turnPower, 0);
        assertEquals(0.08, frame(controller(), 0, 3).turnPower, 0);
    }

    @Test public void singleInToleranceFrameStopsButDoesNotAlign() {
        ClusterAlignmentController.Output o = frame(controller(), 0, 1.9);
        assertEquals(0, o.turnPower, 0);
        assertEquals(ClusterAlignmentController.State.SETTLING, o.state);
    }

    @Test public void invalidMeasurementsCannotAlign() {
        assertEquals(ClusterAlignmentController.State.STALE_FRAME, frame(controller(), 0, Double.NaN).state);
        assertEquals(ClusterAlignmentController.State.STALE_FRAME, controller().update(0, true, false,
                true, new ClusterAlignmentController.Sample(0, -1, 0)).state);
    }

    @Test public void cameraAimOffsetAndAngleWrappingAreApplied() {
        ClusterAlignmentController c = new ClusterAlignmentController(0, 179);
        assertEquals(ClusterAlignmentController.State.SETTLING, frame(c, 0, -179).state);
    }

    @Test public void fastMotionInToleranceCannotCountAsStable() {
        ClusterAlignmentController c = controller();
        frame(c, 0, -1.5);
        assertEquals(0, frame(c, 50, 1.5).stableFrames);
    }
    @Test public void feedingInterlockRejectsLossStaleAimAndBadNumbers() {
        assertTrue(ClusterAlignmentController.isFreshAndAimed(ns(100), sample(100, 0), 0));
        assertFalse(ClusterAlignmentController.isFreshAndAimed(ns(100), null, 0));
        assertFalse(ClusterAlignmentController.isFreshAndAimed(ns(351), sample(100, 0), 0));
        assertFalse(ClusterAlignmentController.isFreshAndAimed(ns(100), sample(100, 3), 0));
        assertFalse(ClusterAlignmentController.isFreshAndAimed(ns(100), sample(101, 0), 0));
        assertFalse(ClusterAlignmentController.isFreshAndAimed(ns(100), sample(100, Double.NaN), 0));
    }

    @Test public void feedInterlockUsesTheSameOffsetAndToleranceAsAlignment() {
        assertTrue(ClusterAlignmentController.isFreshAndAimed(0, sample(0, -179), 179));
        assertFalse(ClusterAlignmentController.isFreshAndAimed(0, sample(0, -178), 179));
    }
}
