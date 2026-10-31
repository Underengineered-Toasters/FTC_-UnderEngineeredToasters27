package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Controller for Secondary Flywheel Motor ("shooter2").
 * Supports REV SPARK Mini Motor Controllers plugged into Servo Ports (CRServo)
 * as well as standard Control Hub Motor Ports (DcMotorEx).
 * Direction: Opposite to shooter1.
 */
public class shooter2 {

    private DcMotorEx motorFlywheel;
    private CRServo sparkMiniFlywheel;

    public static final double POWER_OFF = 0.0;
    public static final double POWER_MAX = 1.00;

    public void init(HardwareMap hardwareMap) {
        // First try initializing as CRServo (REV SPARK Mini plugged into Servo Port)
        try {
            sparkMiniFlywheel = hardwareMap.tryGet(CRServo.class, "shooter2");
        } catch (Exception e) {
            sparkMiniFlywheel = null;
        }

        // Fallback: try initializing as DcMotorEx (Control Hub Motor Port)
        if (sparkMiniFlywheel == null) {
            try {
                motorFlywheel = hardwareMap.get(DcMotorEx.class, "shooter2");
            } catch (Exception e) {
                motorFlywheel = null;
            }

            if (motorFlywheel != null) {
                motorFlywheel.setMotorEnable();
                motorFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorFlywheel.setDirection(DcMotor.Direction.REVERSE); // Opposite direction
            }
        }

        spinDown();
    }

    public void spinForwardMaxSpeed() {
        if (sparkMiniFlywheel != null) {
            sparkMiniFlywheel.setPower(-POWER_MAX); // Opposite direction (-1.00) on SPARK Mini
        } else if (motorFlywheel != null) {
            motorFlywheel.setPower(POWER_MAX);
        }
    }

    public void spinReverseMaxSpeed() {
        if (sparkMiniFlywheel != null) {
            sparkMiniFlywheel.setPower(POWER_MAX); // Opposite direction (+1.00) on SPARK Mini
        } else if (motorFlywheel != null) {
            motorFlywheel.setPower(-POWER_MAX);
        }
    }

    public void spinDown() {
        if (sparkMiniFlywheel != null) {
            sparkMiniFlywheel.setPower(POWER_OFF);
        }
        if (motorFlywheel != null) {
            motorFlywheel.setPower(POWER_OFF);
        }
    }

    public double getCurrentPower() {
        if (sparkMiniFlywheel != null) return sparkMiniFlywheel.getPower();
        if (motorFlywheel != null) return motorFlywheel.getPower();
        return 0.0;
    }

    public double getCurrentVelocityRPM() {
        if (motorFlywheel != null) {
            return motorFlywheel.getVelocity() * 60.0 / 28.0;
        }
        return sparkMiniFlywheel != null ? sparkMiniFlywheel.getPower() * 6000.0 : 0.0;
    }

    public boolean isSpinning() {
        return Math.abs(getCurrentPower()) > 0.05;
    }
}