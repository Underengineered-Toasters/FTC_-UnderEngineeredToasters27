package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private DcMotorEx flywheel;

    // Velocity is in encoder ticks/second. These are PLACEHOLDER values —
    // you WILL need to tune these once the wheel is spun up on a real motor,
    // by testing actual shots at known distances from the HIVE.
    public static final double VELOCITY_OFF        = 0;
    public static final double VELOCITY_CLOSE_SHOT  = 1200;  // tune: short range
    public static final double VELOCITY_MID_SHOT    = 1600;  // tune: mid range
    public static final double VELOCITY_FAR_SHOT    = 2000;  // tune: long range

    // How close (in ticks/sec) the actual velocity needs to be to target
    // before we consider the wheel "ready to fire" — flywheels take a moment to spin up.
    private static final double VELOCITY_TOLERANCE = 50;

    private double targetVelocity = VELOCITY_OFF;

    public void init(HardwareMap hardwareMap) {
        flywheel = hardwareMap.get(DcMotorEx.class, "shooter");

        // Flywheels should coast, NOT brake, when off — braking a spinning
        // flywheel abruptly can strain the motor and gears unnecessarily.
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // RUN_USING_ENCODER lets us command a velocity (ticks/sec) instead of raw power,
        // so the flywheel spins at a consistent speed regardless of battery voltage.
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void update() {
        flywheel.setVelocity(targetVelocity);
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }

    public void spinDown() {
        targetVelocity = VELOCITY_OFF;
    }

    public double getCurrentVelocity() {
        return flywheel.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public boolean isReadyToFire() {
        if (targetVelocity == VELOCITY_OFF) return false;
        return Math.abs(getCurrentVelocity() - targetVelocity) < VELOCITY_TOLERANCE;
    }
}
