package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.YellowBallChase;
import org.firstinspires.ftc.teamcode.robot.YellowBallVision;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Locks the pinhole geometry and chase control law that turn camera pixels into drive commands.
 *
 * <p>The numbers below are derived from the C270 defaults (640x480, 60 degree diagonal FOV) and
 * must be re-derived if the camera, resolution or ball size changes.
 */
public class YellowBallVisionTest {

    private static final double C270_FOCAL_PX = 692.82; // 0.5 * 800 px / tan(30 deg)
    private static final int WIDTH = 640;
    private static final int HEIGHT = 480;

    @Test
    public void focalLengthMatchesTheDatasheetFieldOfView() {
        assertEquals(C270_FOCAL_PX, YellowBallVision.focalPixels(WIDTH, HEIGHT, 60.0), 0.05);
    }

    @Test
    public void horizontalFieldOfViewFollowsFromTheFocalLength() {
        double fov = YellowBallVision.horizontalFovDegrees(WIDTH, C270_FOCAL_PX);
        // 640x480 at 60 deg diagonal gives a ~49.6 deg horizontal field of view.
        assertEquals(49.6, fov, 0.1);
    }

    @Test
    public void bearingIsZeroAtTheCentreAndPositiveToTheRight() {
        assertEquals(0.0, YellowBallVision.bearingDegrees(320.0, WIDTH, C270_FOCAL_PX), 1e-9);

        double right = YellowBallVision.bearingDegrees(493.0, WIDTH, C270_FOCAL_PX);
        double left = YellowBallVision.bearingDegrees(147.0, WIDTH, C270_FOCAL_PX);
        assertEquals(14.0, right, 0.05);
        assertEquals(-14.0, left, 0.05);
        assertTrue("ball to the right must give a positive bearing", right > 0.0);
    }

    @Test
    public void bearingAtTheFrameEdgeEqualsHalfTheHorizontalFov() {
        double edge = YellowBallVision.bearingDegrees(WIDTH, WIDTH, C270_FOCAL_PX);
        assertEquals(YellowBallVision.horizontalFovDegrees(WIDTH, C270_FOCAL_PX) / 2.0, edge, 1e-6);
    }

    @Test
    public void distanceUsesSimilarTriangles() {
        // A 3 in ball at 12 in spans radius = f * D / (2 * range) = 86.6 px.
        double radiusAt12 = C270_FOCAL_PX * 3.0 / (2.0 * 12.0);
        assertEquals(12.0, YellowBallVision.distanceInches(radiusAt12, C270_FOCAL_PX, 3.0),
                1e-9);

        // Twice the apparent radius means half the range.
        assertEquals(6.0, YellowBallVision.distanceInches(radiusAt12 * 2.0, C270_FOCAL_PX, 3.0),
                1e-9);

        // A farther ball is smaller: 24 in gives half the radius.
        assertEquals(radiusAt12 / 2.0, C270_FOCAL_PX * 3.0 / (2.0 * 24.0), 1e-9);
    }

    @Test
    public void unmeasurableBlobsReportInfiniteRange() {
        assertEquals(Double.POSITIVE_INFINITY,
                YellowBallVision.distanceInches(0.0, C270_FOCAL_PX, 3.0), 0.0);
        assertEquals(Double.POSITIVE_INFINITY,
                YellowBallVision.distanceInches(-2.0, C270_FOCAL_PX, 3.0), 0.0);
        assertEquals(Double.POSITIVE_INFINITY,
                YellowBallVision.distanceInches(10.0, 0.0, 3.0), 0.0);
    }

    @Test
    public void forwardPowerClosesRangeAndStopsAtTheGrabDistance() {
        assertEquals(0.0, YellowBallChase.forwardFor(YellowBallChase.grabDistanceInches), 1e-9);
        assertEquals(0.0, YellowBallChase.forwardFor(2.0), 1e-9);

        // Full power once the range error is large enough to saturate the gain.
        assertEquals(YellowBallChase.maxDrivePower,
                YellowBallChase.forwardFor(60.0), 1e-9);

        // Short of the grab distance the robot still creeps instead of stalling.
        double creep = YellowBallChase.forwardFor(YellowBallChase.grabDistanceInches + 2.0);
        assertTrue(creep >= YellowBallChase.minDrivePower - 1e-9);
        assertTrue(creep <= YellowBallChase.maxDrivePower);
    }

    @Test
    public void turnPowerFollowsBearingAndIsClipped() {
        assertEquals(0.0, YellowBallChase.turnFor(0.0), 1e-9);
        assertTrue("ball to the right turns right", YellowBallChase.turnFor(5.0) > 0.0);
        assertTrue("ball to the left turns left", YellowBallChase.turnFor(-5.0) < 0.0);
        assertEquals(YellowBallChase.maxTurnPower, YellowBallChase.turnFor(90.0), 1e-9);
        assertEquals(-YellowBallChase.maxTurnPower, YellowBallChase.turnFor(-90.0), 1e-9);
    }

    @Test
    public void grabbableNeedsRangeAndHeadingUnlessTheBallIsUnderTheCamera() {
        assertFalse(YellowBallChase.isGrabbable(null));

        // In range and centred.
        assertTrue(YellowBallChase.isGrabbable(ball(6.0, 2.0)));
        // In range but far off to the side: keep turning.
        assertFalse(YellowBallChase.isGrabbable(ball(6.0, 25.0)));
        // Centred but still far away: keep driving.
        assertFalse(YellowBallChase.isGrabbable(ball(30.0, 2.0)));
        // Basically on top of it: bearing no longer matters.
        assertTrue(YellowBallChase.isGrabbable(ball(3.0, 60.0)));
    }

    @Test
    public void finalApproachWindowIsCloserThanTheChaseWindow() {
        // Arriving at the chase window is not yet close enough for the intake creep.
        assertTrue(YellowBallChase.isGrabbable(ball(8.0, 1.0),
                YellowBallChase.grabDistanceInches));
        assertFalse(YellowBallChase.isGrabbable(ball(8.0, 1.0),
                YellowBallChase.finalApproachDistanceInches));

        // The final approach only completes right at the intake.
        assertTrue(YellowBallChase.isGrabbable(ball(3.0, 4.0),
                YellowBallChase.finalApproachDistanceInches));

        // Forward power keeps closing from 8 in toward a 3 in target, and stops at it.
        assertTrue(YellowBallChase.forwardFor(8.0, YellowBallChase.finalApproachDistanceInches)
                > 0.0);
        assertEquals(0.0,
                YellowBallChase.forwardFor(2.0, YellowBallChase.finalApproachDistanceInches),
                1e-9);
    }

    private static YellowBallVision.Detection ball(double distanceInches, double bearingDegrees) {
        return new YellowBallVision.Detection(320.0, 240.0, 10.0, 200.0, 0.9,
                bearingDegrees, distanceInches);
    }
}
