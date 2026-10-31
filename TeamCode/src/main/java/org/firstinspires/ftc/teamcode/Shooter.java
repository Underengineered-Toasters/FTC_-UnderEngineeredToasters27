package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotorEx flywheel;
    private CRServo shooter2;
    private static final double POWER_MAX = 1.0;

    public void init(HardwareMap hardwareMap) {
        CRServo servo = hardwareMap.tryGet(CRServo.class, "shooter2");
        configureActuators(servo, servo == null
                ? hardwareMap.tryGet(DcMotorEx.class, "shooter") : null);
    }

    // Port 0 belongs exclusively to FlowerArm; never look up shooter1 here.
    void configureActuators(CRServo servo, DcMotorEx motor) {
        if (servo != null) servo.setPower(0);
        if (servo == null && motor == null) {
            throw new IllegalArgumentException("Configure CR servo shooter2 or a DcMotorEx named shooter");
        }
        shooter2 = servo;
        flywheel = servo == null ? motor : null;
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
        if (shooter2 != null) shooter2.setDirection(direction);
    }

    public void setTargetPower(double power) {
        double clipped = Double.isFinite(power) ? Math.max(-POWER_MAX, Math.min(POWER_MAX, power)) : 0;
        if (flywheel != null) flywheel.setPower(clipped);
        if (shooter2 != null) shooter2.setPower(clipped);
    }

    public void spinDown() { setTargetPower(0); }
    public void update() { /* Open-loop actuators need no periodic velocity command. */ }

    public double getCurrentPower() {
        if (flywheel != null) return flywheel.getPower();
        return shooter2 != null ? shooter2.getPower() : 0;
    }

    public boolean hasVelocityFeedback() { return flywheel != null; }

    public double getCurrentVelocityRPM() {
        // Legacy motor conversion assumes 28 ticks/revolution. CR servos have no encoder feedback.
        return flywheel != null ? flywheel.getVelocity() * 60.0 / 28.0 : Double.NaN;
    }

    public String getHardwareDescription() {
        if (flywheel != null) return "DcMotorEx: shooter";
        return shooter2 != null ? "CR servo: shooter2" : "Not initialized";
    }

    public boolean isSpinning() {
        // For CR servos this indicates a commanded run, not measured wheel motion.
        return flywheel != null ? Math.abs(flywheel.getVelocity()) > 1.0
                : Math.abs(getCurrentPower()) > 0.05;
    }
}
