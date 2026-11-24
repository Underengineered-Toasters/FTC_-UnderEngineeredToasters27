package org.firstinspires.ftc.underengineered.teleops;

import org.firstinspires.ftc.teamcode.robot.HiveTipDetector;
import org.junit.Test;
import static org.junit.Assert.*;

public class HiveTipDetectorTest {
    private static final long T0 = 1_000_000_000L;

    private static HiveTipDetector.Sample sample(double pitchDeg, double rollDeg, long nanos) {
        return new HiveTipDetector.Sample(pitchDeg, rollDeg, 0, nanos);
    }

    @Test public void uprightFramesConfirmAfterRequiredFrames() {
        HiveTipDetector detector = new HiveTipDetector();
        assertEquals(HiveTipDetector.State.UNKNOWN,
                detector.update(T0, true, false, true, sample(1, -2, T0)).state);
        detector.update(T0 + 1, true, false, true, sample(1, -2, T0 + 1));
        HiveTipDetector.Output output =
                detector.update(T0 + 2, true, false, true, sample(1, -2, T0 + 2));
        assertEquals(HiveTipDetector.State.UPRIGHT, output.state);
        assertEquals(HiveTipDetector.REQUIRED_FRAMES, output.stableFrames);
    }

    @Test public void tippedFramesReportTipped() {
        HiveTipDetector detector = new HiveTipDetector();
        HiveTipDetector.Output output = null;
        for (int i = 0; i < HiveTipDetector.REQUIRED_FRAMES; i++) {
            output = detector.update(T0 + i, true, false, true, sample(0, 80, T0 + i));
        }
        assertEquals(HiveTipDetector.State.TIPPED, output.state);
        assertEquals(80, output.tiltDeg, 0);
    }

    @Test public void inPlaneSpinIsNotATip() {
        HiveTipDetector detector = new HiveTipDetector();
        HiveTipDetector.Output output = null;
        for (int i = 0; i < HiveTipDetector.REQUIRED_FRAMES; i++) {
            // Large yaw with level pitch/roll: a tag spun in plane, not tipped.
            output = detector.update(T0 + i, true, false, true,
                    new HiveTipDetector.Sample(2, -1, 170, T0 + i));
        }
        assertEquals(HiveTipDetector.State.UPRIGHT, output.state);
    }

    @Test public void tiltUsesWorstOfPitchAndRoll() {
        assertEquals(40, HiveTipDetector.tilt(sample(-40, 5, 0)), 0);
        assertEquals(55, HiveTipDetector.tilt(sample(-40, 55, 0)), 0);
    }

    @Test public void tippingBackRestoresUpright() {
        HiveTipDetector detector = new HiveTipDetector();
        HiveTipDetector.Output tipped = null;
        for (int i = 0; i < HiveTipDetector.REQUIRED_FRAMES; i++) {
            tipped = detector.update(T0 + i, true, false, true, sample(-5, 88, T0 + i));
        }
        assertEquals(HiveTipDetector.State.TIPPED, tipped.state);
        HiveTipDetector.Output back = null;
        for (int i = 0; i < HiveTipDetector.REQUIRED_FRAMES; i++) {
            back = detector.update(T0 + 100 + i, true, false, true, sample(-5, 3, T0 + 100 + i));
        }
        assertEquals(HiveTipDetector.State.UPRIGHT, back.state);
    }

    @Test public void cameraFailureAndNoStreamReportUnknown() {
        HiveTipDetector detector = new HiveTipDetector();
        for (int i = 0; i < HiveTipDetector.REQUIRED_FRAMES; i++) {
            detector.update(T0 + i, true, false, true, sample(0, 90, T0 + i));
        }
        assertEquals(HiveTipDetector.State.UNKNOWN,
                detector.update(T0 + 10, true, true, true, sample(0, 90, T0 + 10)).state);
        assertEquals(HiveTipDetector.State.UNKNOWN,
                detector.update(T0 + 20, false, false, false, null).state);
    }

    @Test public void staleSampleIsRejected() {
        HiveTipDetector detector = new HiveTipDetector();
        long stale = T0 - HiveTipDetector.MAX_FRAME_AGE_NANOS - 1;
        assertEquals(HiveTipDetector.State.UNKNOWN,
                detector.update(T0, true, false, true, sample(0, 90, stale)).state);
    }

    @Test public void tagLossKeepsLastConfirmedState() {
        HiveTipDetector detector = new HiveTipDetector();
        for (int i = 0; i < HiveTipDetector.REQUIRED_FRAMES; i++) {
            detector.update(T0 + i, true, false, true, sample(0, 90, T0 + i));
        }
        HiveTipDetector.Output output =
                detector.update(T0 + 10, true, false, true, null);
        assertEquals(HiveTipDetector.State.TIPPED, output.state);
        assertEquals(0, output.stableFrames);
    }

    @Test public void repeatedFrameIsNotRecounted() {
        HiveTipDetector detector = new HiveTipDetector();
        for (int i = 0; i < HiveTipDetector.REQUIRED_FRAMES; i++) {
            detector.update(T0 + i, true, false, true, sample(0, 90, T0 + i));
        }
        // Same capture time again, now upright: it is not a new sample, so it cannot start
        // accumulating upright frames.
        HiveTipDetector.Output output =
                detector.update(T0 + 2, true, false, true, sample(0, 0, T0 + 2));
        assertEquals(HiveTipDetector.State.TIPPED, output.state);
        assertEquals(HiveTipDetector.REQUIRED_FRAMES, output.stableFrames);
    }
}
