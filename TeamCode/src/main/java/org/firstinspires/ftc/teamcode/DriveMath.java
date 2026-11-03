package org.firstinspires.ftc.teamcode;

/** Pure-Java tank drive stick shaping and mixing shared by all drive code. */
public final class DriveMath {
    public static final double DEADBAND = 0.05;
    public static final double SLOW_MODE_SCALE = 0.4;

    private DriveMath() {}

    /** Clamp, apply deadband with rescale, then square keeping sign. NaN maps to 0. */
    public static double shape(double stick) {
        if (Double.isNaN(stick)) return 0.0;
        double magnitude = Math.min(Math.abs(stick), 1.0);
        if (magnitude <= DEADBAND) return 0.0;
        double t = (magnitude - DEADBAND) / (1.0 - DEADBAND);
        return Math.copySign(t * t, stick);
    }

    /** forward + turn, scaled down (not clipped) when |forward| + |turn| exceeds 1. */
    public static double leftPower(double forward, double turn) {
        return mix(clamp(forward), clamp(turn), true);
    }

    /** forward - turn, scaled down (not clipped) when |forward| + |turn| exceeds 1. */
    public static double rightPower(double forward, double turn) {
        return mix(clamp(forward), clamp(turn), false);
    }

    private static double mix(double forward, double turn, boolean left) {
        double scale = Math.max(1.0, Math.abs(forward) + Math.abs(turn));
        return (left ? forward + turn : forward - turn) / scale;
    }

    private static double clamp(double x) {
        return Math.max(-1.0, Math.min(1.0, x));
    }
}
