package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Basic Auto - Drive Forward", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {

    private final Robot robot = new Robot();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        robot.drivetrain.drive(0.5, 0);

        while (opModeIsActive() && runtime.seconds() < 2.0) {
            telemetry.addData("Auto", "Driving forward: %.1f sec", runtime.seconds());
            telemetry.update();
        }

        robot.drivetrain.stop();

        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}
