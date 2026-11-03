package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class DriveMathTest {
    private static final double EPS = 1e-9;

    @Test public void deadbandAndInsideReturnZero() {
        assertEquals(0.0, DriveMath.shape(0.0), EPS);
        assertEquals(0.0, DriveMath.shape(DriveMath.DEADBAND), EPS);
        assertEquals(0.0, DriveMath.shape(-DriveMath.DEADBAND), EPS);
        assertEquals(0.0, DriveMath.shape(0.03), EPS);
    }

    @Test public void shapeIsContinuousAtDeadbandEdge() {
        assertEquals(0.0, DriveMath.shape(DriveMath.DEADBAND + 1e-4), 1e-6);
        assertEquals(0.0, DriveMath.shape(-DriveMath.DEADBAND - 1e-4), 1e-6);
    }

    @Test public void fullStickIsFullPower() {
        assertEquals(1.0, DriveMath.shape(1.0), EPS);
        assertEquals(-1.0, DriveMath.shape(-1.0), EPS);
    }

    @Test public void shapeIsSignSymmetric() {
        for (double x = 0.0; x <= 1.0; x += 0.01) {
            assertEquals(-DriveMath.shape(x), DriveMath.shape(-x), EPS);
        }
    }

    @Test public void midStickFollowsRescaledSquare() {
        // t = (0.525 - 0.05) / 0.95 = 0.5, output 0.25
        assertEquals(0.25, DriveMath.shape(0.525), EPS);
        assertEquals(-0.25, DriveMath.shape(-0.525), EPS);
        assertTrue(DriveMath.shape(0.5) < 0.5);
    }

    @Test public void shapeIsMonotonic() {
        double previous = DriveMath.shape(-1.0);
        for (double x = -0.99; x <= 1.0; x += 0.01) {
            double current = DriveMath.shape(x);
            assertTrue("not monotonic at " + x, current >= previous);
            previous = current;
        }
    }

    @Test public void outOfRangeAndNaNAreSafe() {
        assertEquals(1.0, DriveMath.shape(5.0), EPS);
        assertEquals(-1.0, DriveMath.shape(-5.0), EPS);
        assertEquals(1.0, DriveMath.shape(Double.POSITIVE_INFINITY), EPS);
        assertEquals(0.0, DriveMath.shape(Double.NaN), EPS);
    }

    @Test public void unsaturatedInputsPassThroughUnchanged() {
        assertEquals(0.75, DriveMath.leftPower(0.5, 0.25), EPS);
        assertEquals(0.25, DriveMath.rightPower(0.5, 0.25), EPS);
        assertEquals(-0.3, DriveMath.leftPower(-0.5, 0.2), EPS);
        assertEquals(-0.7, DriveMath.rightPower(-0.5, 0.2), EPS);
    }

    @Test public void pureRotationSpinsWheelsOpposite() {
        assertEquals(1.0, DriveMath.leftPower(0.0, 1.0), EPS);
        assertEquals(-1.0, DriveMath.rightPower(0.0, 1.0), EPS);
    }

    @Test public void fullForwardAndFullTurnDesaturate() {
        // sum = 2: left = 2/2 = 1, right = 0/2 = 0
        assertEquals(1.0, DriveMath.leftPower(1.0, 1.0), EPS);
        assertEquals(0.0, DriveMath.rightPower(1.0, 1.0), EPS);
    }

    @Test public void desaturationPreservesLeftRightRatio() {
        // sum = 1.2: left = 1.2/1.2 = 1, right = 0.4/1.2 = 1/3 (hard clip would give 1 and 0.4)
        assertEquals(1.0, DriveMath.leftPower(0.8, 0.4), EPS);
        assertEquals(1.0 / 3.0, DriveMath.rightPower(0.8, 0.4), EPS);
        assertEquals(3.0, DriveMath.leftPower(0.8, 0.4) / DriveMath.rightPower(0.8, 0.4), 1e-9);
    }

    @Test public void outputsNeverExceedOneOverGrid() {
        for (double f = -1.5; f <= 1.5; f += 0.1) {
            for (double t = -1.5; t <= 1.5; t += 0.1) {
                assertTrue(Math.abs(DriveMath.leftPower(f, t)) <= 1.0 + EPS);
                assertTrue(Math.abs(DriveMath.rightPower(f, t)) <= 1.0 + EPS);
            }
        }
    }

    @Test public void mixInputsAreClampedFirst() {
        assertEquals(DriveMath.leftPower(1.0, 0.0), DriveMath.leftPower(9.0, 0.0), EPS);
        assertEquals(DriveMath.rightPower(1.0, -1.0), DriveMath.rightPower(9.0, -9.0), EPS);
    }
}
