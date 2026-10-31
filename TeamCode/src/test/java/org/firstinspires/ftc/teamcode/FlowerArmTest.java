package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.junit.Test;
import java.lang.reflect.Proxy;
import static org.junit.Assert.*;

public class FlowerArmTest {
    private double position;
    private int commands;
    private boolean pressed;
    private final FlowerArm arm = new FlowerArm();

    private Servo servo() {
        return (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[]{Servo.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPosition")) {
                        position = (Double) args[0]; commands++;
                    }
                    return null;
                });
    }

    private void initialize() {
        TouchSensor sensor = (TouchSensor) Proxy.newProxyInstance(TouchSensor.class.getClassLoader(),
                new Class<?>[]{TouchSensor.class}, (proxy, method, args) ->
                        method.getName().equals("isPressed") ? pressed : null);
        arm.configure(servo(), sensor);
    }

    @Test public void releaseAndStopHoldLastPositionInsteadOfReturningToCenter() {
        initialize(); assertEquals(1.0, position, 0);
        arm.update(true, false, 0.04); assertEquals(0.98, position, 1e-9);
        arm.update(false, false, 0.04); assertEquals(0.98, position, 1e-9);
        arm.stop(); assertEquals(0.98, position, 1e-9);
        arm.update(false, true, 0.04); assertEquals(1.0, position, 1e-9);
    }

    @Test public void upperSwitchBlocksRaisingButAllowsLowering() {
        initialize(); arm.update(true, false, 0.04);
        pressed = true;
        arm.update(true, false, 0.04); assertEquals(0.98, position, 1e-9);
        arm.update(false, true, 0.04); assertEquals(1.0, position, 1e-9);
        pressed = false;
        arm.update(true, false, 0.04); assertEquals(0.98, position, 1e-9);
    }

    @Test public void bothButtonsHoldAndLargeOrInvalidTimeCannotJumpTheArm() {
        initialize(); arm.update(true, true, 0.04); assertEquals(1.0, position, 0);
        arm.update(true, false, 10); assertEquals(0.975, position, 1e-9);
        arm.update(true, false, Double.NaN); assertEquals(0.975, position, 1e-9);
        arm.update(true, false, -1); assertEquals(0.975, position, 1e-9);
    }

    @Test public void sustainedInputReachesConfiguredRangeWithoutExceedingIt() {
        initialize();
        for (int i = 0; i < 100; i++) arm.update(true, false, 0.05);
        assertEquals(0, position, 0);
        for (int i = 0; i < 200; i++) arm.update(false, true, 0.05);
        assertEquals(1, position, 0);
    }

    @Test public void missingSwitchFailsBeforeCommandingPosition() {
        try { arm.configure(servo(), null); fail("Missing switch must fail initialization"); }
        catch (IllegalArgumentException expected) { assertEquals(0, commands); }
    }
}
