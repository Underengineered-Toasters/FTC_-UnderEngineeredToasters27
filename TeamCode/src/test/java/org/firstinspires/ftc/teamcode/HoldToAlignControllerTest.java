package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class HoldToAlignControllerTest {
    @Test public void learnedShooterBearingBecomesAlignmentReference() {
        HoldToAlignController controller = new HoldToAlignController(25.0);
        ClusterAlignmentController.Output output = controller.update(true, 100, true, false, true,
                new ClusterAlignmentController.Sample(25.0, 72, 100));
        assertEquals(0, output.turnPower, 0);
        assertEquals(ClusterAlignmentController.State.SETTLING, output.state);
        output = controller.update(true, 200, true, false, true,
                new ClusterAlignmentController.Sample(35.0, 72, 200));
        assertTrue(output.turnPower > 0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void invalidLearnedBearingIsRejected() { new HoldToAlignController(Double.NaN); }

    @Test public void releaseImmediatelyReturnsManualAndNextHoldStartsNewAttempt() {
        HoldToAlignController controller = new HoldToAlignController();
        assertNull(controller.update(false, 0, false, true, false, null));
        ClusterAlignmentController.Output turning = controller.update(true, 100, true, false, true,
                new ClusterAlignmentController.Sample(10, 24, 100));
        assertTrue(turning.turnPower > 0);
        assertNull(controller.update(false, 200, true, false, false, null));
        ClusterAlignmentController.Output retry = controller.update(true, 300, true, false, false, null);
        assertEquals(0, retry.turnPower, 0);
        assertEquals(ClusterAlignmentController.State.TARGET_MISSING, retry.state);
    }

    @Test public void timeoutDoesNotRestartUntilButtonIsReleased() {
        HoldToAlignController controller = new HoldToAlignController();
        controller.update(true, 0, true, false, false, null);
        assertEquals(ClusterAlignmentController.State.TIMED_OUT,
                controller.update(true, 6_000_000_000L, true, false, false, null).state);
        assertEquals(ClusterAlignmentController.State.TIMED_OUT,
                controller.update(true, 7_000_000_000L, true, false, false, null).state);
        controller.update(false, 8_000_000_000L, true, false, false, null);
        assertEquals(ClusterAlignmentController.State.TARGET_MISSING,
                controller.update(true, 9_000_000_000L, true, false, false, null).state);
    }

    @Test public void targetLossSearchesButCameraFailureStopsTurret() {
        HoldToAlignController controller = new HoldToAlignController();
        controller.update(true, 100, true, false, true,
                new ClusterAlignmentController.Sample(-10, 24, 100));
        assertEquals(0.10, controller.update(true, 200, true, false, true, null).turnPower, 0);
        assertEquals(0, controller.update(true, 300, false, true, false, null).turnPower, 0);
    }

    @Test public void missingTargetSearchesBothDirectionsThenSwitchesToFeedback() {
        HoldToAlignController controller = new HoldToAlignController();
        ClusterAlignmentController.Output output = controller.update(true, 0, true, false, true, null);
        assertEquals(ClusterAlignmentController.State.SEARCHING, output.state);
        assertEquals(0.10, output.turnPower, 0);
        assertEquals(-0.10, controller.update(true, 400_000_000L, true, false, true, null).turnPower, 0);
        assertEquals(0.10, controller.update(true, 1_200_000_000L, true, false, true, null).turnPower, 0);
        output = controller.update(true, 1_300_000_000L, true, false, true,
                new ClusterAlignmentController.Sample(-10, 72, 1_300_000_000L));
        assertEquals(ClusterAlignmentController.State.ALIGNING, output.state);
        assertTrue(output.turnPower < 0);
        assertNull(controller.update(false, 1_400_000_000L, true, false, false, null));
    }

    @Test public void searchRequiresFreshFramesAndStopsAtHoldTimeout() {
        HoldToAlignController controller = new HoldToAlignController();
        assertEquals(0, controller.update(true, 0, true, false, false, null).turnPower, 0);
        assertEquals(0.10, controller.update(true, 100, true, false, true, null).turnPower, 0);
        assertEquals(0, controller.update(true, 300_000_000L, true, false, false, null).turnPower, 0);
        assertEquals(0, controller.update(true, 400_000_000L, false, false, false, null).turnPower, 0);
        ClusterAlignmentController.Output output = controller.update(true, 5_000_000_000L, true, false, true, null);
        assertEquals(ClusterAlignmentController.State.TIMED_OUT, output.state);
        assertEquals(0, output.turnPower, 0);
    }
}
