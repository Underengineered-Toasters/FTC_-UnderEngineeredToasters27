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
        telemetry.addLine("A = Intake In (100%)");
        telemetry.addLine("B = Intake Out (100%)");
        telemetry.addLine("Y = Push Ball to Shooter");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Intake motor at 100% speed
            if (gamepad1.a) {
                intakeSystem.setPower(intake.INTAKE_IN);
            } else if (gamepad1.b) {
                intakeSystem.setPower(intake.INTAKE_OUT);
            } else {
                intakeSystem.setPower(intake.INTAKE_STOP);
            }

            // Feeder servo control: hold Y to push, release to reset
            if (gamepad1.y) {
                intakeSystem.pushBall();
            } else {
                intakeSystem.restFeeder();
            }

            telemetry.addData("Intake Power", intakeSystem.getPower());
            telemetry.addData("Feeder Pos", intakeSystem.getFeederPosition());
            telemetry.update();
        }
    }
}
