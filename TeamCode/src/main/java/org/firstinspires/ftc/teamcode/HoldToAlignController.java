package org.firstinspires.ftc.teamcode;

/** One alignment attempt per button hold; release returns control to the driver. */
public final class HoldToAlignController {
    private ClusterAlignmentController attempt;
    private final double aimBearingDeg;

    public HoldToAlignController() { this(AutonomousCameraAligner.AIM_BEARING_DEG); }

    public HoldToAlignController(double aimBearingDeg) {
        if (!Double.isFinite(aimBearingDeg)) throw new IllegalArgumentException("Aim bearing must be finite");
        this.aimBearingDeg = aimBearingDeg;
    }

    /** Null output means manual driving; non-null output exclusively controls chassis turning. */
    public ClusterAlignmentController.Output update(boolean held, long now, boolean streaming,
            boolean cameraFailed, boolean hasNewFrame, ClusterAlignmentController.Sample target) {
        if (!held) {
            attempt = null;
            return null;
        }
        if (attempt == null) {
            attempt = new ClusterAlignmentController(now, aimBearingDeg);
        }
        return attempt.update(now, streaming, cameraFailed, hasNewFrame, target);
    }
}
