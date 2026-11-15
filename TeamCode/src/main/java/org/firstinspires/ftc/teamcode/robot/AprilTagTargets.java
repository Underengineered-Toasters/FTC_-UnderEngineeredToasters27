package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.opencv.core.Point;

/** SDK 12 separates single-tag identity from cluster identity. */
public final class AprilTagTargets {
    private AprilTagTargets() {}

    public static String nameOf(AprilTagDetection detection) {
        if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            return cluster.metadata == null ? "Unknown cluster" : cluster.metadata.name;
        }
        if (detection instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection single = (AprilTagSingleDetection) detection;
            return single.metadata == null ? "Unknown tag" : single.metadata.name;
        }
        return "Unknown detection";
    }

    public static String identityOf(AprilTagDetection detection) {
        if (detection instanceof AprilTagSingleDetection) {
            return Integer.toString(((AprilTagSingleDetection) detection).id);
        }
        return nameOf(detection);
    }

    public static Point centerOf(AprilTagDetection detection) {
        return detection instanceof AprilTagSingleDetection
                ? ((AprilTagSingleDetection) detection).center : null;
    }

    /** BIOBUZZ ID lookups select the containing cluster's pose, not an individual tag pose. */
    public static boolean matchesId(AprilTagDetection detection, int id) {
        if (detection instanceof AprilTagSingleDetection) {
            return ((AprilTagSingleDetection) detection).id == id;
        }
        String cluster = bioBuzzClusterForId(id);
        return detection instanceof AprilTagClusterDetection && cluster != null
                && cluster.equals(nameOf(detection));
    }

    public static String bioBuzzClusterForId(int id) {
        if (id >= 30 && id <= 33) return AprilTagVision.RED_SCORING;
        if (id >= 34 && id <= 37) return AprilTagVision.RED_AUDIENCE;
        if (id >= 38 && id <= 41) return AprilTagVision.BLUE_AUDIENCE;
        if (id >= 42 && id <= 45) return AprilTagVision.BLUE_SCORING;
        return null;
    }
}
