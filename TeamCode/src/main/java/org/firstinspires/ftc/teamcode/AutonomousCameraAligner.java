package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

/** Chassis-only alignment to a selected scoring cluster; never drives toward it or fires. */
public final class AutonomousCameraAligner {
    // Camera bearing at which the shooter is aimed correctly. Calibrate on hardware.
    public static final double AIM_BEARING_DEG = 0.0;

    public static boolean align(LinearOpMode opMode, Robot robot, String targetName) {
        ClusterAlignmentController controller = new ClusterAlignmentController(System.nanoTime(), AIM_BEARING_DEG);
        try {
            while (opMode.opModeIsActive()) {
                boolean ready = robot.vision.isReady();
                List<AprilTagDetection> fresh = robot.vision.getFreshDetections();
                ClusterAlignmentController.Sample target = null;
                if (fresh != null) {
                    for (AprilTagDetection detection : fresh) {
                        if (!(detection instanceof AprilTagClusterDetection)) continue;
                        AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                        if (cluster.metadata != null && targetName.equals(cluster.metadata.name)
                                && cluster.ftcPose != null) {
                            target = new ClusterAlignmentController.Sample(cluster.ftcPose.bearing,
                                    cluster.ftcPose.range, cluster.frameAcquisitionNanoTime);
                            break;
                        }
                    }
                }
                ClusterAlignmentController.Output output = controller.update(System.nanoTime(), ready,
                        robot.vision.hasFailed(), fresh != null, target);
                robot.drive(0, output.turnPower);
                opMode.telemetry.addData("Camera", robot.vision.getCameraState());
                opMode.telemetry.addData("Align target", targetName);
                opMode.telemetry.addData("Alignment", output.state);
                opMode.telemetry.addData("Stable frames", "%d / %d", output.stableFrames,
                        ClusterAlignmentController.REQUIRED_FRAMES);
                opMode.telemetry.addData("Align turn power", "%.3f", output.turnPower);
                if (output.target != null) {
                    opMode.telemetry.addData("Target bearing", "%.1f deg", output.target.bearingDeg);
                    opMode.telemetry.addData("Target range", "%.1f in (display only)", output.target.rangeInches);
                }
                opMode.telemetry.update();
                if (output.state == ClusterAlignmentController.State.ALIGNED) return true;
                if (output.state == ClusterAlignmentController.State.CAMERA_FAILED
                        || output.state == ClusterAlignmentController.State.TIMED_OUT) return false;
                opMode.sleep(20);
            }
            return false;
        } finally {
            robot.drive(0, 0);
        }
    }

    public static boolean hasFreshAim(Robot robot, String targetName) {
        if (!robot.vision.isReady()) return false;
        AprilTagClusterDetection target = robot.vision.findCluster(targetName);
        ClusterAlignmentController.Sample sample = target == null || target.ftcPose == null ? null
                : new ClusterAlignmentController.Sample(target.ftcPose.bearing,
                        target.ftcPose.range, target.frameAcquisitionNanoTime);
        return ClusterAlignmentController.isFreshAndAimed(System.nanoTime(), sample, AIM_BEARING_DEG);
    }

    private AutonomousCameraAligner() {}
}
