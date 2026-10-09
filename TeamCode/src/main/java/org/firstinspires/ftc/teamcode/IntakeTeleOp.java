package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Intake TeleOp", group = "TeleOp")
public class IntakeTeleOp extends LinearOpMode {

    private intake intakeSystem = new intake();

    @Override
    public void runOpMode() {

        intakeSystem.init(hardwareMap);

        telemetry.addLine("Intake initialized");
        telemetry.addLine("A = Intake In + Feeder Push");
        telemetry.addLine("B = Intake Out");
        telemetry.addLine("Release = Everything Stops");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Running intake automatically runs the feeder
            if (gamepad1.a) {
                intakeSystem.setPower(intake.INTAKE_IN);
            } else if (gamepad1.b) {
                intakeSystem.setPower(intake.INTAKE_OUT);
            } else {
                intakeSystem.setPower(intake.INTAKE_STOP);
            }

            telemetry.addData("Intake Power", intakeSystem.getPower());
            telemetry.addData("Feeder Pos", intakeSystem.getFeederPosition());
            telemetry.update();
        }
    }
}
