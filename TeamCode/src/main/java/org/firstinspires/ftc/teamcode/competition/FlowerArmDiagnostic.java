package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/** Isolates the production arm controller from all other robot subsystems. */
@TeleOp(name = "Flower Arm Test", group = "Diagnostics")
public class FlowerArmDiagnostic extends LinearOpMode {
    @Override
    public void runOpMode() {
        FlowerArm arm = new FlowerArm();
        try {
            arm.init(hardwareMap);
            Servo servo = hardwareMap.get(Servo.class, "flower arm");
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
                telemetry.addLine("Starts at 1.000: test UP first; release holds target");
                telemetry.update();
                sleep(20);
            }
        } finally {
            arm.stop();
        }
    }
}
