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

    @Test public void shooter2InitializesWithoutAShooterMotorAndStartsStopped() {
        Actuator a = new Actuator();
        Shooter s = new Shooter();
        s.configureActuators(a.as(CRServo.class), null);
        assertEquals(0, a.power, 0);
        assertFalse(s.hasVelocityFeedback());
        assertTrue(s.getHardwareDescription().contains("shooter2"));
    }

    @Test public void forwardReverseAndStopReachShooterServo() {
        Actuator a = new Actuator();
        Shooter s = new Shooter();
        s.configureActuators(a.as(CRServo.class), null);
        s.spinForwardMaxSpeed();
        assertEquals(1, a.power, 0); assertTrue(s.isSpinning());
        s.spinReverseMaxSpeed();
        assertEquals(-1, a.power, 0);
        s.spinDown();
        assertEquals(0, a.power, 0); assertFalse(s.isSpinning());
    }

    @Test public void directionToggleReachesShooterServo() {
        Actuator a = new Actuator();
        Shooter s = new Shooter();
        s.configureActuators(a.as(CRServo.class), null);
        s.setDirection(true);
        assertEquals(DcMotorSimple.Direction.REVERSE, a.direction);
    }

    @Test public void powerIsClippedAndInvalidPowerStops() {
        Actuator a = new Actuator();
        Shooter s = new Shooter();
        s.configureActuators(a.as(CRServo.class), null);
        s.setTargetPower(3); assertEquals(1, a.power, 0);
        s.setTargetPower(-3); assertEquals(-1, a.power, 0);
        s.setTargetPower(Double.NaN); assertEquals(0, a.power, 0);
    }

    @Test public void absentHardwareReportsBothSupportedConfigurations() {
        try {
            new Shooter().configureActuators(null, null);
            fail("Missing hardware must not be accepted");
        } catch (IllegalArgumentException expected) {
            assertTrue(expected.getMessage().contains("shooter2"));
            assertTrue(expected.getMessage().contains("DcMotorEx"));
        }
    }

    @Test public void legacyMotorKeepsItsInitializationAndVelocityConversion() {
        Actuator motor = new Actuator(); motor.velocity = 56;
        Shooter s = new Shooter(); s.configureActuators(null, motor.as(DcMotorEx.class));
        assertTrue(motor.enabled);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, motor.mode);
        assertTrue(s.hasVelocityFeedback()); assertEquals(120, s.getCurrentVelocityRPM(), 0);
        s.setTargetPower(.7); assertEquals(.7, motor.power, 0);
        s.spinDown(); assertEquals(0, motor.power, 0);
    }

    @Test public void servosNeverReportFictitiousMeasuredRpm() {
        Shooter s = new Shooter();
        s.configureActuators(new Actuator().as(CRServo.class), null);
        s.spinUp6000RPM();
        assertEquals(1, s.getCurrentPower(), 0);
        assertTrue(Double.isNaN(s.getCurrentVelocityRPM()));
    }
}
