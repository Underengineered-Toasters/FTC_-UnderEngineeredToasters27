package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Basic Auto", group = "Competition")
public class BasicAuto extends LinearOpMode {

    private static final double AUTO_TIME_LIMIT_SECONDS = 29.0;

    @Override
    public void runOpMode() {
        waitForStart();

        if (isStopRequested()) {
            return;
        }

        long startTime = System.nanoTime();

        while (opModeIsActive()) {
            double elapsedSeconds =
                    (System.nanoTime() - startTime) / 1_000_000_000.0;

            if (elapsedSeconds >= AUTO_TIME_LIMIT_SECONDS) {
                break;
            }

            telemetry.addData("Autonomous", "Running");
            telemetry.addData("Elapsed time", "%.1f seconds", elapsedSeconds);
            telemetry.update();

            idle();
        }
    }
}