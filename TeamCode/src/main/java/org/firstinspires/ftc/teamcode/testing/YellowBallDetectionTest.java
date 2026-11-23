package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.YellowBallVision;

import java.util.List;

/**
 * Live yellow-ball camera test: stream, detections, and the distance/bearing calibration readout.
 *
 * <p>Use it to bring up the C270 and to calibrate before trusting autonomous:
 * <ol>
 *   <li>Open the Driver Station camera stream (3-dot menu) - the C270 picture with the yellow
 *       circle drawn around each blob should appear.</li>
 *   <li>Hold a ball at a measured distance (12 in is convenient) and compare "measured radius"
 *       with the computed distance. If the distance is consistently high or low, adjust
 *       {@link YellowBallVision#ballDiameterInches}, or the effective
 *       {@link YellowBallVision#diagonalFovDegrees}.</li>
 *   <li>Slide the ball left/right: the bearing must go positive when the ball is to the robot's
 *       right (the sign a positive drive turn uses).</li>
 * </ol>
 *
 * <p>No drivetrain or mechanism hardware is used, so this can run with the robot on blocks.
 */
@TeleOp(name = "Yellow Ball Camera Test", group = "Vision")
public class YellowBallDetectionTest extends LinearOpMode {

    /** How many detections to list. */
    public static int listedDetections = 3;

    private final YellowBallVision vision = new YellowBallVision();

    @Override
    public void runOpMode() {
        vision.init(hardwareMap);

        telemetry.addLine("YELLOW BALL CAMERA TEST");
        telemetry.addData("Camera", YellowBallVision.cameraName);
        telemetry.addData("Status", vision.getStatus());
        telemetry.addLine("Open the Driver Station camera stream to see the blobs.");
        telemetry.update();

        try {
            waitForStart();

            while (opModeIsActive()) {
                List<YellowBallVision.Detection> detections = vision.getDetections();

                telemetry.addData("Status", vision.getStatus());
                telemetry.addData("FPS", "%.1f", vision.getFps());
                telemetry.addData("Detections", detections.size());
                telemetry.addLine("--------------------------------");

                int shown = 0;
                for (YellowBallVision.Detection detection : detections) {
                    if (shown++ >= listedDetections) {
                        break;
                    }
                    telemetry.addData("Ball " + shown,
                            "%.1f in @ %.1f deg", detection.distanceInches,
                            detection.bearingDegrees);
                    telemetry.addData("  blob", "r=%.1f px, area=%.0f px2, circ=%.2f",
                            detection.radiusPixels, detection.contourArea,
                            detection.circularity);
                }

                if (detections.isEmpty()) {
                    telemetry.addLine("No yellow ball in view.");
                }

                telemetry.addLine("--------------------------------");
                telemetry.addData("Ball diameter", "%.2f in",
                        YellowBallVision.ballDiameterInches);
                telemetry.addData("Focal length", "%.1f px", YellowBallVision.focalPixels());
                telemetry.addData("Horizontal FOV", "%.1f deg",
                        YellowBallVision.horizontalFovDegrees(
                                YellowBallVision.imageWidth, YellowBallVision.focalPixels()));
                telemetry.addData("Expected radius at 12 in", "%.1f px",
                        expectedRadiusPixels(12.0));
                telemetry.update();

                idle();
            }
        } finally {
            vision.close();
        }
    }

    /** Radius the configured ball should show at a given distance, for calibration checks. */
    private static double expectedRadiusPixels(double distanceInches) {
        return YellowBallVision.focalPixels() * YellowBallVision.ballDiameterInches
                / (2.0 * distanceInches);
    }
}
