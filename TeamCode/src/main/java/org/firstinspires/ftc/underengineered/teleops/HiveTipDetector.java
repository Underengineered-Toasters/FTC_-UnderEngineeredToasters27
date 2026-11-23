package org.firstinspires.ftc.underengineered.teleops;

/**
 * Pure logic: reports whether a tagged BIOBUZZ hive cell has tipped, judged from the AprilTag's
 * 3D orientation.
 *
 * The hive is a bistable structure and its cells carry the AprilTags that robots aim at, so
 * tipping the hive rotates those tags out of the camera's view plane. {@code pitch} and
 * {@code roll} stay near their upright values while the cell is up and swing past
 * {@link #TIP_THRESHOLD_DEG} once the structure flips. {@code yaw} is in-plane spin and is
 * ignored, so a tag merely rotated sideways is not reported as a tip.
 *
 * Frames are confirmed {@link #REQUIRED_FRAMES} at a time and stale or non-finite samples are
 * rejected, mirroring {@link ClusterAlignmentController}. Callers own the clock: pass
 * {@code System.nanoTime()} and the timestamp of the frame that produced the sample.
 */
public final class HiveTipDetector {
    /** Tilt at or above which one frame reads as tipped. Tune on the robot. */
    public static final double TIP_THRESHOLD_DEG = 30.0;
    /** Consecutive agreeing frames required before the reported state changes. */
    public static final int REQUIRED_FRAMES = 3;
    /** Samples older than this are treated as unusable. */
    public static final long MAX_FRAME_AGE_NANOS = 250_000_000L;

    public enum State { UNKNOWN, UPRIGHT, TIPPED }

    public static final class Sample {
        public final double pitchDeg;
        public final double rollDeg;
        public final double yawDeg;
        public final long captureNanos;
        public Sample(double pitchDeg, double rollDeg, double yawDeg, long captureNanos) {
            this.pitchDeg = pitchDeg;
            this.rollDeg = rollDeg;
            this.yawDeg = yawDeg;
            this.captureNanos = captureNanos;
        }
    }

    public static final class Output {
        public final State state;
        public final double tiltDeg;
        public final int stableFrames;
        Output(State state, double tiltDeg, int stableFrames) {
            this.state = state;
            this.tiltDeg = tiltDeg;
            this.stableFrames = stableFrames;
        }
    }

    private State confirmed = State.UNKNOWN;
    private int tippedFrames;
    private int uprightFrames;
    private double lastTiltDeg;
    private long lastCapture = Long.MIN_VALUE;

    /** hasNewFrame distinguishes a fresh tag-less frame from one still being processed. */
    public Output update(long now, boolean streaming, boolean cameraFailed,
                         boolean hasNewFrame, Sample sample) {
        if (cameraFailed || !streaming) return reset();
        if (hasNewFrame) {
            if (sample == null) {
                // Fresh frame with no visible tag. Drop the frame counters but keep the last
                // confirmed state so a brief occlusion does not flip the result.
                tippedFrames = 0;
                uprightFrames = 0;
                return new Output(confirmed, lastTiltDeg, 0);
            }
            if (!Double.isFinite(sample.pitchDeg) || !Double.isFinite(sample.rollDeg)
                    || sample.captureNanos > now
                    || now - sample.captureNanos > MAX_FRAME_AGE_NANOS) {
                return reset();
            }
            if (sample.captureNanos <= lastCapture) {
                // Repeated or out-of-order frame; report without re-counting it.
                return new Output(confirmed, lastTiltDeg, Math.max(tippedFrames, uprightFrames));
            }
            lastCapture = sample.captureNanos;
            lastTiltDeg = tilt(sample);
            if (lastTiltDeg >= TIP_THRESHOLD_DEG) {
                tippedFrames++;
                uprightFrames = 0;
            } else {
                uprightFrames++;
                tippedFrames = 0;
            }
            if (tippedFrames >= REQUIRED_FRAMES) {
                confirmed = State.TIPPED;
            } else if (uprightFrames >= REQUIRED_FRAMES) {
                confirmed = State.UPRIGHT;
            }
        }
        return new Output(confirmed, lastTiltDeg, Math.max(tippedFrames, uprightFrames));
    }

    /** How far the tag is rotated out of the camera's view plane; yaw (in-plane spin) is ignored. */
    public static double tilt(Sample sample) {
        return Math.max(Math.abs(sample.pitchDeg), Math.abs(sample.rollDeg));
    }

    private Output reset() {
        confirmed = State.UNKNOWN;
        tippedFrames = 0;
        uprightFrames = 0;
        lastTiltDeg = 0;
        lastCapture = Long.MIN_VALUE;
        return new Output(State.UNKNOWN, 0, 0);
    }
}
