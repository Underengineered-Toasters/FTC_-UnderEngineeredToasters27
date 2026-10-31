package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.junit.Test;
import java.lang.reflect.Proxy;
import static org.junit.Assert.*;

public class FlowerArmTest {
    private double power = 1;
    private boolean pressed;
    private final FlowerArm arm = new FlowerArm();

    private void initialize() {
        CRServo servo = (CRServo) Proxy.newProxyInstance(CRServo.class.getClassLoader(),
                new Class<?>[]{CRServo.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPower")) power = (Double) args[0];
                    if (method.getName().equals("getPower")) return power;
                    return null;
                });
        TouchSensor sensor = (TouchSensor) Proxy.newProxyInstance(TouchSensor.class.getClassLoader(),
                new Class<?>[]{TouchSensor.class}, (proxy, method, args) ->
                        method.getName().equals("isPressed") ? pressed : null);
        arm.configure(servo, sensor);
    }

    @Test public void initializationReleaseAndStopCommandZero() {
        initialize(); assertEquals(0, power, 0);
        arm.update(true, false); assertTrue(power > 0);
        arm.update(false, false); assertEquals(0, power, 0);
        arm.update(false, true); assertTrue(power < 0);
        arm.stop(); assertEquals(0, power, 0);
    }

    @Test public void switchStopsAnAlreadyRaisingArmAndAllowsDownwardEscape() {
        initialize(); arm.update(true, false); assertTrue(power > 0);
        pressed = true;
        arm.update(true, false); assertEquals(0, power, 0);
        arm.update(false, true); assertTrue(power < 0);
        pressed = false;
        arm.update(true, false); assertTrue(power > 0);
    }

    @Test public void conflictingButtonsStopInEitherSwitchState() {
        initialize(); arm.update(true, false);
        arm.update(true, true); assertEquals(0, power, 0);
        pressed = true;
        arm.update(true, true); assertEquals(0, power, 0);
    }

    @Test public void missingSwitchFailsWithServoStopped() {
        CRServo servo = (CRServo) Proxy.newProxyInstance(CRServo.class.getClassLoader(),
                new Class<?>[]{CRServo.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPower")) power = (Double) args[0];
                    return null;
                });
        try { arm.configure(servo, null); fail("Missing switch must fail initialization"); }
        catch (IllegalArgumentException expected) { assertEquals(0, power, 0); }
    }
}
