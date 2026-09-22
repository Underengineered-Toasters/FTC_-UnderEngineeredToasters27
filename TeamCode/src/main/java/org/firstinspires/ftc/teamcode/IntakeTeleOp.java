package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Intake TeleOp", group = "TeleOp")
public class IntakeTeleOp extends LinearOpMode {

    private final Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addLine("Intake initialized");
        telemetry.addLine("A = Intake In (100%)");
        telemetry.addLine("B = Intake Out (100%)");
        telemetry.addLine("Y = Push Ball to Shooter");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Intake motor at 100% speed
            if (gamepad1.a) {
                robot.intake.setPower(Intake.INTAKE_IN);
            } else if (gamepad1.b) {
                robot.intake.setPower(Intake.INTAKE_OUT);
            } else {
                robot.intake.setPower(Intake.INTAKE_STOP);
            }

            // Feeder servo control: hold Y to push, release to reset
            if (gamepad1.y) {
                robot.intake.pushBall();
            } else {
                robot.intake.restFeeder();
            }

            telemetry.addData("Intake Power", robot.intake.getPower());
            telemetry.addData("Feeder Pos", robot.intake.getFeederPosition());
            telemetry.update();
        }
    }
}
