package org.firstinspires.ftc.teamcode;

/** One alignment attempt per button hold; release returns control to the driver. */
public final class HoldToAlignController {
    private ClusterAlignmentController attempt;
    private final double aimBearingDeg;
    private long searchStart = Long.MIN_VALUE;
    private long lastFreshFrame = Long.MIN_VALUE;
    private static final double SEARCH_POWER = 0.10;
    private static final long FIRST_SWEEP_NANOS = 400_000_000L;
    private static final long FULL_SWEEP_NANOS = 800_000_000L;

    public HoldToAlignController() { this(AutonomousCameraAligner.AIM_BEARING_DEG); }

    public HoldToAlignController(double aimBearingDeg) {
        if (!Double.isFinite(aimBearingDeg)) throw new IllegalArgumentException("Aim bearing must be finite");
        this.aimBearingDeg = aimBearingDeg;
    }

    /** Null returns manual driving; non-null controls turret alignment or bounded timed search. */
    public ClusterAlignmentController.Output update(boolean held, long now, boolean streaming,
            boolean cameraFailed, boolean hasNewFrame, ClusterAlignmentController.Sample target) {
        if (!held) {
            attempt = null;
            searchStart = Long.MIN_VALUE;
            lastFreshFrame = Long.MIN_VALUE;
            return null;
        }
        if (attempt == null) {
            attempt = new ClusterAlignmentController(now, aimBearingDeg);
        }
        ClusterAlignmentController.Output output = attempt.update(now, streaming, cameraFailed, hasNewFrame, target);
        if (hasNewFrame && streaming && !cameraFailed) lastFreshFrame = now;
        if (!streaming || cameraFailed || output.state == ClusterAlignmentController.State.STALE_FRAME) {
            lastFreshFrame = Long.MIN_VALUE;
        }
        if (output.state == ClusterAlignmentController.State.TARGET_MISSING
                && lastFreshFrame != Long.MIN_VALUE
                && now - lastFreshFrame <= ClusterAlignmentController.MAX_FRAME_AGE_NANOS) {
            if (searchStart == Long.MIN_VALUE) searchStart = now;
            long elapsed = now - searchStart;
            double direction = elapsed < FIRST_SWEEP_NANOS ? 1
                    : (((elapsed - FIRST_SWEEP_NANOS) / FULL_SWEEP_NANOS) % 2 == 0 ? -1 : 1);
            return new ClusterAlignmentController.Output(ClusterAlignmentController.State.SEARCHING,
                    direction * SEARCH_POWER, 0, null);
        }
        searchStart = Long.MIN_VALUE;
        return output;
    }
}
