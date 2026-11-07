package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.TouchSensor;
import java.lang.reflect.Proxy;
import org.junit.Test;
import static org.junit.Assert.*;

public class FlowerArmTest {
    private boolean pressed = true;
    private double position = Double.NaN;
    private PwmControl.PwmRange pwmRange;
    private final Servo servo = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
            new Class<?>[] {Servo.class, PwmControl.class}, (proxy, method, args) -> {
                if (method.getName().equals("getPosition")) return Double.isNaN(position) ? 0.5 : position;
                if (method.getName().equals("setPosition")) position = (Double) args[0];
                if (method.getName().equals("setPwmRange")) pwmRange = (PwmControl.PwmRange) args[0];
                return null;
            });
    private final TouchSensor sensor = (TouchSensor) Proxy.newProxyInstance(
            TouchSensor.class.getClassLoader(), new Class<?>[] {TouchSensor.class},
            (proxy, method, args) -> method.getName().equals("isPressed") ? pressed : null);

    @Test public void extendsOnlyUpwardPwmEndpoint() {
        new FlowerArm().configure(servo, sensor);
        assertNotNull(pwmRange);
        assertEquals(600, pwmRange.usPulseLower, 0);
        assertEquals(2500, pwmRange.usPulseUpper, 0);
    }

    @Test public void startsAtUpperLimitAndHoldingBDoesNotMove() {
        FlowerArm arm = new FlowerArm();
        arm.configure(servo, sensor);
        double initial = arm.getTargetPosition();
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
        assertTrue(position > lowered);
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
        assertTrue(position < FlowerArm.UPPER_POSITION);
        double lowered = position;
        arm.update(false, false, 0.05);
        assertEquals(lowered, position, 0);
        arm.update(true, true, 0.05);
        assertEquals(lowered, position, 0);
    }

    @Test public void bPressTogglesAndReleaseContinuesWithoutRepeatedToggles() {
        FlowerArm arm = new FlowerArm();
        arm.configure(servo, sensor);
        arm.updateToggle(true, 0.05);
        double firstStep = position;
        pressed = false;
        arm.updateToggle(true, 0.05);
        assertTrue(position < firstStep);
        double heldStep = position;
        arm.updateToggle(false, 0.05);
        assertTrue(position < heldStep);
        double lowered = position;
        arm.updateToggle(true, 0.05);
        assertTrue(position > lowered);
        double raising = position;
        arm.updateToggle(false, 0.05);
        assertTrue(position > raising);
    }

    @Test public void toggleRaiseStopsAtContactUntilNextPressLowers() {
        FlowerArm arm = new FlowerArm();
        arm.configure(servo, sensor);
        arm.updateToggle(true, 0.05);
        pressed = false;
        for (int i = 0; i < 5; i++) arm.updateToggle(false, 0.05);
        arm.updateToggle(true, 0.05);
        pressed = true;
        double contact = position;
        arm.updateToggle(false, 0.05);
        pressed = false;
        arm.updateToggle(false, 0.05);
        assertEquals(contact, position, 0);
        arm.updateToggle(true, 0.05);
        assertTrue(position < contact);
    }

    @Test public void unpressedStartupRaisesUntilContactInsteadOfThrowing() {
        pressed = false;
        FlowerArm arm = new FlowerArm();
        arm.configure(servo, sensor);
        assertTrue(Double.isNaN(position)); // Configure itself never sweeps the arm.
        assertEquals(FlowerArm.HomingState.MOVING, arm.homeStep(0.1, 0.05, false));
        assertTrue(position > 0.5);
        double contact = position;
        pressed = true;
        assertEquals(FlowerArm.HomingState.READY, arm.homeStep(0.2, 0.05, false));
        assertEquals(contact, position, 0);
    }

    @Test public void homingTimeoutOrStopDoesNotIssueAnotherUpwardStep() {
        pressed = false;
        FlowerArm arm = new FlowerArm();
        arm.configure(servo, sensor);
        arm.homeStep(0.1, 0.05, false);
        double last = position;
        assertEquals(FlowerArm.HomingState.FAILED, arm.homeStep(4.0, 0.05, false));
        assertEquals(last, position, 0);
        assertEquals(FlowerArm.HomingState.FAILED, arm.homeStep(0.2, 0.05, true));
        assertEquals(last, position, 0);
    }
}
