package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    private DcMotor left;
    private DcMotor right;

    public void init(HardwareMap hardwareMap) {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");

        // Motors face opposite directions on a tank drive
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** Arcade drive: forward/back on drive, rotation on turn. Inputs in [-1, 1]. */
    public void drive(double drive, double turn) {
        double leftPower = clamp(drive + turn);
        double rightPower = clamp(drive - turn);
        left.setPower(leftPower);
        right.setPower(rightPower);
    }

    public void stop() {
        left.setPower(0);
        right.setPower(0);
    }

    private static double clamp(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }
}
