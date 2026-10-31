package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

/** Isolates the production arm controller from all other robot subsystems. */
@TeleOp(name = "Flower Arm Test", group = "Diagnostics")
public class FlowerArmDiagnostic extends LinearOpMode {
    @Override
    public void runOpMode() {
        FlowerArm arm = new FlowerArm();
        try {
            arm.init(hardwareMap);
            Servo servo = hardwareMap.get(Servo.class, "flower arm");
            long nextLog = 0;
            while (!isStopRequested()) {
                boolean up = gamepad1.dpad_up;
                boolean down = gamepad1.dpad_down;
                if (isStarted()) arm.update(up, down);
                telemetry.addData("State", isStarted() ? "RUNNING" : "INIT - press START for controls");
                telemetry.addData("Controller 1 ID", gamepad1.id);
                telemetry.addData("D-pad UP", up);
                telemetry.addData("D-pad DOWN", down);
                telemetry.addData("Upper touch", arm.isUpperLimitPressed());
                telemetry.addData("Target (not measured angle)", "%.3f", arm.getTargetPosition());
                telemetry.addData("SDK command (not feedback)", "%.3f", servo.getPosition());
                telemetry.addData("Servo port", servo.getPortNumber());
                telemetry.addData("Servo PWM", servo.getController().getPwmStatus());
                if (System.nanoTime() >= nextLog) {
                    RobotLog.ii("[DEBUG-flower-arm]", "started=%s up=%s down=%s touch=%s target=%.3f sdk=%.3f port=%d pwm=%s",
                            isStarted(), up, down, arm.isUpperLimitPressed(), arm.getTargetPosition(),
                            servo.getPosition(), servo.getPortNumber(), servo.getController().getPwmStatus());
                    nextLog = System.nanoTime() + 500_000_000L;
                }
                telemetry.addLine("Starts at 0.000: test DOWN first; release holds target");
                telemetry.update();
                sleep(20);
            }
        } finally {
            arm.stop();
        }
    }
}
