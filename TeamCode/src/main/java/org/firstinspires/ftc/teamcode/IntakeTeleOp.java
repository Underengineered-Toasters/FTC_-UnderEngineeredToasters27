package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Intake TeleOp", group = "TeleOp")
public class IntakeTeleOp extends LinearOpMode {

    private intake intakeSystem = new intake();

    @Override
    public void runOpMode() {

        intakeSystem.init(hardwareMap);

        telemetry.addLine("Intake Initialized");
        telemetry.addLine("A = Intake IN");
        telemetry.addLine("B = Intake OUT");
        telemetry.addLine("Release = Stop Intake");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                intakeSystem.setPower(intake.INTAKE_IN);
            } else if (gamepad1.b) {
                intakeSystem.setPower(intake.INTAKE_OUT);
            } else {
                intakeSystem.setPower(intake.INTAKE_STOP);
            }

            telemetry.addData("Intake Power", intakeSystem.getPower());
            telemetry.update();
        }
    }
}