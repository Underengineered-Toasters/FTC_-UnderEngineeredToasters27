package org.firstinspires.ftc.teamcode;

/** Pure feedback/state logic. All timestamps use System.nanoTime's clock. */
public final class ClusterAlignmentController {
    public static final double BEARING_TOLERANCE_DEG = 2.0;
    public static final double MAX_BEARING_RATE_DEG_PER_SEC = 8.0;
    public static final int REQUIRED_FRAMES = 5;
    public static final long SETTLE_NANOS = 200_000_000L;
    public static final long MAX_FRAME_AGE_NANOS = 250_000_000L;
    public static final long TIMEOUT_NANOS = 5_000_000_000L;
    private static final double KP = 0.02;
    private static final double MIN_POWER = 0.08;
    private static final double MAX_POWER = 0.25;

    public enum State { WAITING_FOR_CAMERA, TARGET_MISSING, ALIGNING, SETTLING,
        ALIGNED, STALE_FRAME, CAMERA_FAILED, TIMED_OUT }

    public static final class Sample {
        public final double bearingDeg;
        public final double rangeInches;
        public final long captureNanos;
        public Sample(double bearingDeg, double rangeInches, long captureNanos) {
            this.bearingDeg = bearingDeg;
            this.rangeInches = rangeInches;
            this.captureNanos = captureNanos;
        }
    }

    public static final class Output {
        public final State state;
        public final double turnPower;
        public final int stableFrames;
        public final Sample target;
        Output(State state, double turnPower, int stableFrames, Sample target) {
            this.state = state;
            this.turnPower = turnPower;
            this.stableFrames = stableFrames;
            this.target = target;
        }
    }

    private final long startNanos;
    private final double aimBearingDeg;
    private Sample lastTarget;
    private long lastCapture = Long.MIN_VALUE;
    private long settledSince;
    private int stableFrames;

    public ClusterAlignmentController(long startNanos, double aimBearingDeg) {
        if (!Double.isFinite(aimBearingDeg)) throw new IllegalArgumentException("Aim bearing must be finite");
        this.startNanos = startNanos;
        this.aimBearingDeg = aimBearingDeg;
    }

    /** hasNewFrame distinguishes target loss from a camera frame still being processed. */
    public Output update(long now, boolean streaming, boolean cameraFailed,
                         boolean hasNewFrame, Sample sample) {
        if (cameraFailed) return reset(State.CAMERA_FAILED);
        if (now - startNanos >= TIMEOUT_NANOS) return reset(State.TIMED_OUT);
        if (!streaming) return reset(State.WAITING_FOR_CAMERA);
        if (hasNewFrame) {
            if (sample == null) return reset(State.TARGET_MISSING);
            if (!Double.isFinite(sample.bearingDeg) || !Double.isFinite(sample.rangeInches)
                    || sample.rangeInches <= 0 || sample.captureNanos < startNanos
                    || sample.captureNanos > now
                    || now - sample.captureNanos > MAX_FRAME_AGE_NANOS) {
                return reset(State.STALE_FRAME);
            }
            if (sample.captureNanos > lastCapture) {
                boolean contiguous = lastTarget != null
                        && sample.captureNanos - lastTarget.captureNanos <= MAX_FRAME_AGE_NANOS;
                double rate = contiguous ? Math.abs(normalize(sample.bearingDeg - lastTarget.bearingDeg))
                        / ((sample.captureNanos - lastTarget.captureNanos) / 1e9) : 0.0;
                lastTarget = sample;
                lastCapture = sample.captureNanos;
                if (Math.abs(error(sample.bearingDeg)) <= BEARING_TOLERANCE_DEG
                        && rate <= MAX_BEARING_RATE_DEG_PER_SEC) {
                    if (!contiguous || stableFrames == 0) {
                        settledSince = sample.captureNanos;
                        stableFrames = 0;
                    }
                    stableFrames++;
                } else {
                    stableFrames = 0;
                }
                if (stableFrames >= REQUIRED_FRAMES
                        && sample.captureNanos - settledSince >= SETTLE_NANOS) {
                    return new Output(State.ALIGNED, 0, stableFrames, lastTarget);
                }
            }
        }
        if (lastTarget == null) return reset(State.TARGET_MISSING);
        if (now - lastTarget.captureNanos > MAX_FRAME_AGE_NANOS) return reset(State.STALE_FRAME);
        double error = error(lastTarget.bearingDeg);
        if (Math.abs(error) <= BEARING_TOLERANCE_DEG) {
            return new Output(State.SETTLING, 0, stableFrames, lastTarget);
        }
        // Current drivetrain mapping: raw positive turn is physical left.
        double power = Math.copySign(Math.min(MAX_POWER, Math.max(MIN_POWER, KP * Math.abs(error))), error);
        return new Output(State.ALIGNING, power, 0, lastTarget);
    }

    /** Live interlock after stable alignment; does not replace stable-lock acquisition. */
    public static boolean isFreshAndAimed(long now, Sample target, double aimBearingDeg) {
        return target != null && Double.isFinite(aimBearingDeg)
                && Double.isFinite(target.bearingDeg) && Double.isFinite(target.rangeInches)
                && target.rangeInches > 0 && target.captureNanos <= now
                && now - target.captureNanos <= MAX_FRAME_AGE_NANOS
                && Math.abs(normalize(target.bearingDeg - aimBearingDeg)) <= BEARING_TOLERANCE_DEG;
    }

    private double error(double bearing) {
        return normalize(bearing - aimBearingDeg);
    }

    private static double normalize(double degrees) {
        degrees %= 360;
        if (degrees >= 180) degrees -= 360;
        if (degrees < -180) degrees += 360;
        return degrees;
    }

    private Output reset(State state) {
        lastTarget = null;
        stableFrames = 0;
        return new Output(state, 0, 0, null);
    }
}
