package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.messages.KinematicsModel;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;

import static org.junit.Assert.assertEquals;

@RunWith(Parameterized.class)
public class DualShooterKinematicsCustomInputsTest {
    private static final double TOLERANCE = 1e-6;
    private static final double SHEET_GRAVITY = 386.088;
    private static final double SHEET_SLIP = 0.50;

    @Parameterized.Parameters(name = "{index}: {0}")
    public static Object[][] shotCases() {
        KinematicsModel shotOne = new KinematicsModel(
                new Vector2d(83.5, 131.5), new Vector2d(83.5, 86.5), 59.55, 12, 60.0);
        KinematicsModel shotTwo = new KinematicsModel(
                new Vector2d(83.5, 131.5), new Vector2d(83.5, 86.5), 59.55, 12, 30.0);
        return new Object[][] {
                //{"original sheet shot", shotOne, SHEET_GRAVITY, SHEET_SLIP, 3395.456611219688},
                {"diagonal sheet shot", shotTwo, SHEET_GRAVITY, SHEET_SLIP, 3127.821289345818},
        };
    }

    private final String description;
    private final KinematicsModel shot;
    private final double gravity;
    private final double slipFactor;
    private final double expectedRPM;

    public DualShooterKinematicsCustomInputsTest(String description, KinematicsModel shot,
                                                double gravity, double slipFactor, double expectedRPM) {
        this.description = description;
        this.shot = shot;
        this.gravity = gravity;
        this.slipFactor = slipFactor;
        this.expectedRPM = expectedRPM;
    }

    @Test
    public void calculatesExpectedTargetRpm() {
        double rpmValue = DualShooterKinematics.getTargetRPM(shot, gravity, slipFactor);
        System.out.println(rpmValue);
        assertEquals(description, expectedRPM, rpmValue, TOLERANCE);
    }
}
