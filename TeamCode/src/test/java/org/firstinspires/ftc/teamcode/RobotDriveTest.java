package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Locks the four-wheel mecanum mixing contract that every OpMode and every autonomous route
 * depends on: positive forward drives all wheels forward, positive strafe moves right, positive
 * turn rotates clockwise, and saturation scales all four wheels instead of clipping them.
 *
 * <p>Wheel order matches {@code MecanumDrive}: left front, left back, right front, right back.
 */
public class RobotDriveTest {

    private static final double EPS = 1e-9;

    @Test
    public void forwardDrivesEveryWheelForward() {
        double[] p = Robot.mecanumWheelPowers(0.5, 0.0, 0.0);
        for (double wheel : p) {
            assertEquals(0.5, wheel, EPS);
        }
    }

    @Test
    public void reverseDrivesEveryWheelBackward() {
        double[] p = Robot.mecanumWheelPowers(-0.5, 0.0, 0.0);
        for (double wheel : p) {
            assertEquals(-0.5, wheel, EPS);
        }
    }

    @Test
    public void positiveTurnRotatesClockwise() {
        double[] p = Robot.mecanumWheelPowers(0.0, 0.0, 0.5);
        // Left side forward, right side backward -> clockwise, to the robot's right.
        assertTrue("left front forward", p[0] > 0.0);
        assertTrue("left back forward", p[1] > 0.0);
        assertTrue("right front backward", p[2] < 0.0);
        assertTrue("right back backward", p[3] < 0.0);
        assertEquals(p[0], p[1], EPS);
        assertEquals(p[2], p[3], EPS);
    }

    @Test
    public void positiveStrafeMovesRight() {
        double[] p = Robot.mecanumWheelPowers(0.0, 0.5, 0.0);
        // Diagonal pattern for a right strafe: front-left and back-right forward.
        assertTrue("left front forward", p[0] > 0.0);
        assertTrue("left back backward", p[1] < 0.0);
        assertTrue("right front backward", p[2] < 0.0);
        assertTrue("right back forward", p[3] > 0.0);
    }

    @Test
    public void strafingLeftIsTheMirrorOfStrafingRight() {
        double[] right = Robot.mecanumWheelPowers(0.0, 0.4, 0.0);
        double[] left = Robot.mecanumWheelPowers(0.0, -0.4, 0.0);
        for (int i = 0; i < 4; i++) {
            assertEquals(-right[i], left[i], EPS);
        }
    }

    @Test
    public void saturationScalesAllWheelsAndPreservesTheArc() {
        // Full forward + full strafe + full turn requests 3.0 on the left front wheel.
        double[] p = Robot.mecanumWheelPowers(1.0, 1.0, 1.0);

        for (double wheel : p) {
            assertTrue("wheel power must stay legal: " + wheel,
                    Math.abs(wheel) <= 1.0 + EPS);
        }
        // Left front requested 3.0, so every wheel is divided by 3.
        assertEquals(1.0, p[0], EPS);
        // The ratio between the wheels is preserved instead of clipping them individually.
        assertEquals(3.0, p[0] / p[1], 1e-6);
        assertEquals(3.0, p[0] / p[3], 1e-6);
        assertEquals(-1.0 / 3.0, p[2], EPS);
    }

    @Test
    public void singleAxisCommandsAreNotScaled() {
        double[] p = Robot.mecanumWheelPowers(0.7, 0.0, 0.0);
        assertEquals(0.7, p[0], EPS);
        assertEquals(0.7, p[1], EPS);
        assertEquals(0.7, p[2], EPS);
        assertEquals(0.7, p[3], EPS);
    }

    @Test
    public void inputsOutsideTheLegalRangeAreClipped() {
        double[] p = Robot.mecanumWheelPowers(2.0, 0.0, 0.0);
        assertEquals(1.0, p[0], EPS);
        assertEquals(1.0, Robot.clip(5.0), EPS);
        assertEquals(-1.0, Robot.clip(-5.0), EPS);
        assertEquals(0.25, Robot.clip(0.25), EPS);
    }
}
