package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class DualShooterSubsystem {
    private final DcMotorEx masterMotor;
    private final DcMotorEx followerMotor;
    private final ElapsedTime timer;

    // --- goBILDA 5203 (1:1 6000 RPM bare motor) ---
    public static final double TICKS_PER_REV = 28.0;
    public static final double MAX_MOTOR_RPM = 6000.0;

    // --- PULLEY / BELT RATIO ---
    // If driving 1:1 (same size pulleys), keep at 1.0.
    // If motor pulley has 30 teeth and wheel pulley has 15 teeth, ratio is (30.0 / 15.0) = 2.0 (speed-up).
    public static double GEAR_RATIO_WHEEL_OVER_MOTOR = 1.0;

    // PLACEHOLDER PIDF gains: tune on the robot with the flywheel unloaded first.
    public static double kF = 1.0 / ((MAX_MOTOR_RPM / 60.0) * TICKS_PER_REV);
    public static double kP = 0.00005;
    public static double kI = 0.0;
    public static double kD = 0.0;

    private double targetWheelRPM = 0.0;
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastTime = 0.0;

    public DualShooterSubsystem(HardwareMap hardwareMap, String motor1Name, String motor2Name) {
        masterMotor = hardwareMap.get(DcMotorEx.class, motor1Name);
        followerMotor = hardwareMap.get(DcMotorEx.class, motor2Name);

        masterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        masterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        followerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        followerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        masterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        followerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // BOTH MOTORS SHARE A BELT: THEY MUST SPIN IN THE SAME DIRECTION!
        masterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        followerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        // Note: If the ball shoots backwards into the robot, change BOTH to REVERSE.

        timer = new ElapsedTime();
    }

    /**
     * @param wheelRPM The desired RPM of the 72mm shooter wheel.
     */
    public void setTargetRPM(double wheelRPM) {
        double gearRatio = GEAR_RATIO_WHEEL_OVER_MOTOR;
        if (!Double.isFinite(wheelRPM) || !Double.isFinite(gearRatio) || gearRatio <= 0.0) {
            targetWheelRPM = 0.0;
            return;
        }

        double maxWheelRPM = MAX_MOTOR_RPM * gearRatio;
        targetWheelRPM = Math.max(0.0, Math.min(maxWheelRPM, wheelRPM));
    }

    public void update() {
        double currentTime = timer.seconds();
        double dt = currentTime - lastTime;
        if (dt <= 0.001) return;

        // Target motor shaft RPM = wheel RPM / gear ratio
        double gearRatio = GEAR_RATIO_WHEEL_OVER_MOTOR;
        if (!Double.isFinite(gearRatio) || gearRatio <= 0.0) {
            stop();
            return;
        }

        double targetMotorRPM = Math.min(targetWheelRPM / gearRatio, MAX_MOTOR_RPM);

        // Current motor speed from master encoder
        double currentMotorTicksPerSec = Math.abs(masterMotor.getVelocity());
        double targetMotorTicksPerSec = (targetMotorRPM / 60.0) * TICKS_PER_REV;

        double error = targetMotorTicksPerSec - currentMotorTicksPerSec;

        // PID Loop
        integralSum += error * dt;
        integralSum = Math.max(-500.0, Math.min(500.0, integralSum));
        double derivative = (error - lastError) / dt;

        double feedforward = targetMotorTicksPerSec * kF;
        double pidOutput = (kP * error) + (kI * integralSum) + (kD * derivative);
        double motorPower = Math.max(0.0, Math.min(1.0, feedforward + pidOutput));

        if (targetWheelRPM == 0.0) {
            masterMotor.setPower(0.0);
            followerMotor.setPower(0.0);
            integralSum = 0.0;
        } else {
            // Send the exact same power to both belted motors simultaneously
            masterMotor.setPower(motorPower);
            followerMotor.setPower(motorPower);
        }

        lastError = error;
        lastTime = currentTime;
    }

    public void stop() {
        targetWheelRPM = 0.0;
        masterMotor.setPower(0.0);
        followerMotor.setPower(0.0);
        integralSum = 0.0;
        lastError = 0.0;
        lastTime = timer.seconds();
    }

    public double getMasterWheelRPM() {
        return motorVelocityToWheelRPM(masterMotor);
    }

    public double getFollowerWheelRPM() {
        return motorVelocityToWheelRPM(followerMotor);
    }

    public double getCurrentWheelRPM() {
        return (getMasterWheelRPM() + getFollowerWheelRPM()) / 2.0;
    }

    public double getMasterPower() {
        return masterMotor.getPower();
    }

    public double getFollowerPower() {
        return followerMotor.getPower();
    }

    private double motorVelocityToWheelRPM(DcMotorEx motor) {
        return (Math.abs(motor.getVelocity()) / TICKS_PER_REV)
                * 60.0
                * GEAR_RATIO_WHEEL_OVER_MOTOR;
    }

    public double getTargetWheelRPM() {
        return targetWheelRPM;
    }

    public boolean isReadyToShoot(double toleranceRPM) {
        return targetWheelRPM > 0.0
                && toleranceRPM >= 0.0
                && Math.abs(getMasterWheelRPM() - targetWheelRPM) <= toleranceRPM
                && Math.abs(getFollowerWheelRPM() - targetWheelRPM) <= toleranceRPM;
    }
}