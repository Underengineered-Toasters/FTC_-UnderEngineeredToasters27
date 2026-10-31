package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Controller for Primary Flywheel Motor ("shooter1").
 * Supports REV SPARK Mini Motor Controllers plugged into Servo Ports (CRServo)
 * as well as standard Control Hub Motor Ports (DcMotorEx).
 */
public class shooter1 {

    private DcMotorEx motorFlywheel;
    private CRServo sparkMiniFlywheel;

    public static final double POWER_OFF = 0.0;
    public static final double POWER_MAX = 1.00;

    public void init(HardwareMap hardwareMap) {
        // First try initializing as CRServo (REV SPARK Mini plugged into Servo Port)
        try {
            sparkMiniFlywheel = hardwareMap.tryGet(CRServo.class, "shooter1");
            if (sparkMiniFlywheel == null) {
                sparkMiniFlywheel = hardwareMap.tryGet(CRServo.class, "shooter");
            }
        } catch (Exception e) {
            sparkMiniFlywheel = null;
        }

        // Fallback: try initializing as DcMotorEx (Control Hub Motor Port)
        if (sparkMiniFlywheel == null) {
            try {
                motorFlywheel = hardwareMap.get(DcMotorEx.class, "shooter1");
            } catch (Exception e) {
                try {
                    motorFlywheel = hardwareMap.get(DcMotorEx.class, "shooter");
                } catch (Exception ex) {
                    motorFlywheel = null;
                }
            }

            if (motorFlywheel != null) {
                motorFlywheel.setMotorEnable();
                motorFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motorFlywheel.setDirection(DcMotor.Direction.FORWARD);
            }
        }

        spinDown();
    }

    public void spinForwardMaxSpeed() {
        if (sparkMiniFlywheel != null) {
            sparkMiniFlywheel.setPower(POWER_MAX); // 100% Full Speed on SPARK Mini
        } else if (motorFlywheel != null) {
            motorFlywheel.setPower(POWER_MAX);
        }
    }

    public void spinReverseMaxSpeed() {
        if (sparkMiniFlywheel != null) {
            sparkMiniFlywheel.setPower(-POWER_MAX); // -100% Reverse Speed on SPARK Mini
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
