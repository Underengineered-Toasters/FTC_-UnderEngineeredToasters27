package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.lang.reflect.Proxy;
import org.junit.Test;
import static org.junit.Assert.*;

public class FlowerArmTest {
    private boolean pressed = true;
    private double position = Double.NaN;
    private final Servo servo = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
            new Class<?>[] {Servo.class}, (proxy, method, args) -> {
                if (method.getName().equals("setPosition")) position = (Double) args[0];
                return null;
            });
    private final TouchSensor sensor = (TouchSensor) Proxy.newProxyInstance(
            TouchSensor.class.getClassLoader(), new Class<?>[] {TouchSensor.class},
            (proxy, method, args) -> method.getName().equals("isPressed") ? pressed : null);

    @Test public void startsAtUpperLimitAndHoldingBDoesNotMove() {
        FlowerArm arm = new FlowerArm();
        arm.configure(servo, sensor);
        double initial = position;
        arm.update(true, false, 0.05);
        assertEquals(initial, position, 0);
    }

    @Test public void contactStopsRaisingAndBounceCannotResumeIt() {
        FlowerArm arm = new FlowerArm();
        arm.configure(servo, sensor);
        pressed = false;
        for (int i = 0; i < 10; i++) arm.update(false, true, 0.05);
        double lowered = position;
        arm.update(true, false, 0.05);
        assertTrue(position < lowered);
        pressed = true;
        double contact = position;
        arm.update(true, false, 0.05);
        pressed = false;
        arm.update(true, false, 0.05);
        assertEquals(contact, position, 0);
    }

    @Test public void canLowerAwayFromPressedSwitchAndReleaseHolds() {
        FlowerArm arm = new FlowerArm();
        arm.configure(servo, sensor);
        arm.update(false, true, 0.05);
        assertTrue(position > FlowerArm.UPPER_POSITION);
        double lowered = position;
        arm.update(false, false, 0.05);
        assertEquals(lowered, position, 0);
        arm.update(true, true, 0.05);
        assertEquals(lowered, position, 0);
    }

    @Test public void refusesUnhomedStartupBeforeCommandingServo() {
        pressed = false;
        try {
            new FlowerArm().configure(servo, sensor);
            fail("Unhomed startup must be rejected");
        } catch (IllegalStateException expected) {
            assertTrue(Double.isNaN(position));
        }
    }
}
