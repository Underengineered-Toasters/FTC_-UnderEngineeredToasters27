package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Turret-mounted camera feedback actuator. Physical polarity/travel require verification. */
public final class Turret {
    private static final double MAX_POWER = 0.15;
    private DcMotor motor;

    public void init(HardwareMap hardwareMap) {
        configure(hardwareMap.get(DcMotor.class, "turret"));
    }

    void configure(DcMotor motor) {
        this.motor = motor;
        stop();
        motor.setDirection(DcMotor.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void applyAlignment(ClusterAlignmentController.Output output) {
        setPower(output == null ? 0 : output.turnPower);
    }

    public void setPower(double power) {
        double bounded = Double.isFinite(power) ? Math.max(-MAX_POWER, Math.min(MAX_POWER, power)) : 0;
        if (motor != null) motor.setPower(bounded);
    }

    public double getPower() { return motor == null ? 0 : motor.getPower(); }
    public void stop() { setPower(0); }
}
