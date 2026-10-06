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
        telemetry.addLine("Left Trigger = Run Feeder");
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

            // Hold the left trigger to run the feeder; release to stop.
            if (gamepad1.left_trigger > 0.05) {
                intakeSystem.pushBall();
            } else {
                intakeSystem.restFeeder();
            }

            telemetry.addData("Intake Power", intakeSystem.getPower());
            telemetry.addData("Feeder Power", intakeSystem.getFeederPower());
            telemetry.update();
        }

        intakeSystem.restFeeder();
        intakeSystem.stopIntake();
    }
}
