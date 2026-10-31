package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotorEx flywheel;
    private CRServo shooter1;
    private CRServo shooter2;
    private static final double POWER_MAX = 1.0;
    private static final double SHOOTER1_MAX = 0.3;
    private static final double SHOOTER2_MAX = 0.5;
    private static final double LEGACY_MOTOR_MAX = 0.4;

    public void init(HardwareMap hardwareMap) {
        CRServo first = hardwareMap.tryGet(CRServo.class, "shooter1");
        CRServo second = hardwareMap.tryGet(CRServo.class, "shooter2");
        // Prefer the explicitly configured Comp pair; retain older motor configurations.
        configureActuators(first, second, first == null && second == null
                ? hardwareMap.tryGet(DcMotorEx.class, "shooter") : null);
    }

    // Shared initialization seam for hardware mapping and actuator-command unit tests.
    void configureActuators(CRServo first, CRServo second, DcMotorEx motor) {
        if (first != null) first.setPower(0);
        if (second != null) second.setPower(0);
        if ((first == null) != (second == null)) {
            throw new IllegalArgumentException("Shooter requires BOTH CR servos: shooter1 and shooter2");
        }
        if (first == null && motor == null) {
            throw new IllegalArgumentException("Configure CR servos shooter1/shooter2 or a DcMotorEx named shooter");
        }
        shooter1 = first;
        shooter2 = second;
        flywheel = first == null ? motor : null;
        if (flywheel != null) {
            flywheel.setMotorEnable();
            flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        setDirection(false);
        spinDown();
    }

    public void spinForwardMaxSpeed() { setTargetPower(POWER_MAX); }
    public void spinReverseMaxSpeed() { setTargetPower(-POWER_MAX); }
    public void spinUpFullSpeed() { spinForwardMaxSpeed(); }

    public void spinUp6000RPM() {
        // Compatibility alias: maximum power, not regulated or guaranteed RPM.
        spinForwardMaxSpeed();
    }

    public void setDirection(boolean reverse) {
        DcMotorSimple.Direction direction = reverse
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        if (flywheel != null) flywheel.setDirection(direction);
        // Both installed shooter servos need opposite polarity for physical shooting.
        DcMotorSimple.Direction servoDirection = reverse
                ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
        if (shooter1 != null) shooter1.setDirection(servoDirection);
        if (shooter2 != null) shooter2.setDirection(servoDirection);
    }

    public void setTargetPower(double power) {
        double clipped = Double.isFinite(power) ? Math.max(-POWER_MAX, Math.min(POWER_MAX, power)) : 0;
        if (flywheel != null) flywheel.setPower(limit(clipped, LEGACY_MOTOR_MAX));
        if (shooter1 != null) shooter1.setPower(limit(clipped, SHOOTER1_MAX));
        if (shooter2 != null) shooter2.setPower(limit(clipped, SHOOTER2_MAX));
    }

    private static double limit(double power, double maximum) {
        return Math.max(-maximum, Math.min(maximum, power));
    }

    public double getShooter1Power() { return shooter1 == null ? 0 : shooter1.getPower(); }
    public double getShooter2Power() { return shooter2 == null ? 0 : shooter2.getPower(); }

    public void spinDown() { setTargetPower(0); }
    public void update() { /* Open-loop actuators need no periodic velocity command. */ }

    public double getCurrentPower() {
        if (flywheel != null) return flywheel.getPower();
        return shooter1 != null ? shooter1.getPower() : 0;
    }

    public boolean hasVelocityFeedback() { return flywheel != null; }

    public double getCurrentVelocityRPM() {
        // Legacy motor conversion assumes 28 ticks/revolution. CR servos have no encoder feedback.
        return flywheel != null ? flywheel.getVelocity() * 60.0 / 28.0 : Double.NaN;
    }

    public String getHardwareDescription() {
        if (flywheel != null) return "DcMotorEx: shooter";
        return shooter1 != null ? "CR servos: shooter1 + shooter2" : "Not initialized";
    }

    public boolean isSpinning() {
        // For CR servos this indicates a commanded run, not measured wheel motion.
        return flywheel != null ? Math.abs(flywheel.getVelocity()) > 1.0
                : Math.abs(getCurrentPower()) > 0.05;
    }
}
