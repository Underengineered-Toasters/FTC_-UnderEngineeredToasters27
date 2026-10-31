package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.junit.Test;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import static org.junit.Assert.*;

public class ShooterTest {
    private static class Actuator implements InvocationHandler {
        double power;
        double velocity;
        DcMotorSimple.Direction direction;
        DcMotor.RunMode mode;
        boolean enabled;
        @SuppressWarnings("unchecked") <T> T as(Class<T> type) {
            return (T) Proxy.newProxyInstance(type.getClassLoader(), new Class<?>[]{type}, this);
        }
        @Override public Object invoke(Object proxy, Method method, Object[] args) {
            switch (method.getName()) {
                case "setPower": power = (Double) args[0]; return null;
                case "getPower": return power;
                case "setDirection": direction = (DcMotorSimple.Direction) args[0]; return null;
                case "getDirection": return direction;
                case "setMode": mode = (DcMotor.RunMode) args[0]; return null;
                case "setMotorEnable": enabled = true; return null;
                case "getVelocity": return velocity;
                default: return null;
            }
        }
    }

    @Test public void compPairInitializesWithoutAShooterMotorAndStartsStopped() {
        Actuator a = new Actuator(), b = new Actuator();
        Shooter s = new Shooter();
        s.configureActuators(a.as(CRServo.class), b.as(CRServo.class), null);
        assertEquals(0, a.power, 0); assertEquals(0, b.power, 0);
        assertFalse(s.hasVelocityFeedback());
        assertTrue(s.getHardwareDescription().contains("shooter1 + shooter2"));
    }

    @Test public void forwardReverseAndStopReachBothServos() {
        Actuator a = new Actuator(), b = new Actuator();
        Shooter s = new Shooter();
        s.configureActuators(a.as(CRServo.class), b.as(CRServo.class), null);
        s.spinForwardMaxSpeed();
        assertEquals(0.3, a.power, 0); assertEquals(0.5, b.power, 0); assertTrue(s.isSpinning());
        s.spinReverseMaxSpeed();
        assertEquals(-0.3, a.power, 0); assertEquals(-0.5, b.power, 0);
        s.spinDown();
        assertEquals(0, a.power, 0); assertEquals(0, b.power, 0); assertFalse(s.isSpinning());
    }

    @Test public void directionToggleReachesBothServos() {
        Actuator a = new Actuator(), b = new Actuator();
        Shooter s = new Shooter();
        s.configureActuators(a.as(CRServo.class), b.as(CRServo.class), null);
        assertEquals(DcMotorSimple.Direction.REVERSE, a.direction);
        assertEquals(DcMotorSimple.Direction.REVERSE, b.direction);
        s.setDirection(true);
        assertEquals(DcMotorSimple.Direction.FORWARD, a.direction);
        assertEquals(DcMotorSimple.Direction.FORWARD, b.direction);
        s.setDirection(false);
        assertEquals(DcMotorSimple.Direction.REVERSE, a.direction);
        assertEquals(DcMotorSimple.Direction.REVERSE, b.direction);
    }

    @Test public void pairPowerIsClippedAndInvalidPowerStops() {
        Actuator a = new Actuator(), b = new Actuator();
        Shooter s = new Shooter();
        s.configureActuators(a.as(CRServo.class), b.as(CRServo.class), null);
        s.setTargetPower(3); assertEquals(0.3, a.power, 0); assertEquals(0.5, b.power, 0);
        s.setTargetPower(-3); assertEquals(-0.3, a.power, 0); assertEquals(-0.5, b.power, 0);
        s.setTargetPower(Double.NaN); assertEquals(0, a.power, 0); assertEquals(0, b.power, 0);
    }

    @Test public void partialPairReportsTheMissingConfigurationAndStopsPresentServo() {
        Actuator a = new Actuator(); a.power = 1;
        try {
            new Shooter().configureActuators(a.as(CRServo.class), null, null);
            fail("Partial pair must not be accepted");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("BOTH"));
            assertEquals(0, a.power, 0);
        }
    }

    @Test public void absentHardwareReportsBothSupportedConfigurations() {
        try {
            new Shooter().configureActuators(null, null, null);
            fail("Missing hardware must not be accepted");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("shooter1/shooter2"));
            assertTrue(expected.getMessage().contains("DcMotorEx"));
        }
    }

    @Test public void legacyMotorKeepsItsInitializationAndVelocityConversion() {
        Actuator motor = new Actuator(); motor.velocity = 56;
        Shooter s = new Shooter(); s.configureActuators(null, null, motor.as(DcMotorEx.class));
        assertTrue(motor.enabled);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motor.mode);
        assertTrue(s.hasVelocityFeedback()); assertEquals(120, s.getCurrentVelocityRPM(), 0);
        s.setTargetPower(.7); assertEquals(.4, motor.power, 0);
        s.spinDown(); assertEquals(0, motor.power, 0);
    }

    @Test public void servosNeverReportFictitiousMeasuredRpm() {
        Shooter s = new Shooter();
        s.configureActuators(new Actuator().as(CRServo.class), new Actuator().as(CRServo.class), null);
        s.spinUp6000RPM();
        assertEquals(0.3, s.getCurrentPower(), 0);
        assertTrue(Double.isNaN(s.getCurrentVelocityRPM()));
    }
}
