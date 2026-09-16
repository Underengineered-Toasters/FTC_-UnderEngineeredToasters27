package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Wraps the motorized intake roller. Simple on/off/reverse control — no encoder needed. */
public class intake {

    private DcMotor intake;

    public static final double INTAKE_IN     = 1.0;
    public static final double INTAKE_OUT    = -1.0;
    public static final double INTAKE_OFF    = 0.0;

    public void init(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setPower(double power) {
        intake.setPower(power);
    }
}
