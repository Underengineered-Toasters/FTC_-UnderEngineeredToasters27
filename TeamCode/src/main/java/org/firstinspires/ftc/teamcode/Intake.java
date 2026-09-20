package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Controls the mirrored continuous-rotation servos at the front of the robot. */
public class Intake {

    private static final double INTAKE_POWER = 1.0;

    private CRServo frontLeftServo;
    private CRServo frontRightServo;

    public void init(HardwareMap hardwareMap) {
        frontLeftServo = hardwareMap.get(CRServo.class, "frontLeftServo");
        frontRightServo = hardwareMap.get(CRServo.class, "frontRightServo");

        frontLeftServo.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightServo.setDirection(DcMotorSimple.Direction.REVERSE);
        stop();
    }

    public void intake() {
        setPower(INTAKE_POWER);
    }

    public void eject() {
        setPower(-INTAKE_POWER);
    }

    public void stop() {
        setPower(0.0);
    }

    private void setPower(double power) {
        frontLeftServo.setPower(power);
        frontRightServo.setPower(power);
    }
}
