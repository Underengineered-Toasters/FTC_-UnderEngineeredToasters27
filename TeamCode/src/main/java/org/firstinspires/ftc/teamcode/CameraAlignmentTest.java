package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Camera Alignment Test", group = "Autonomous")
public class CameraAlignmentTest extends LinearOpMode {
    @Override public void runOpMode() {
        Robot robot = new Robot();
        String target = AprilTagVision.RED_SCORING;
        boolean initialized = false;
        try {
            robot.init(hardwareMap);
            initialized = true;
            robot.shooter.spinDown();
            robot.intakeSystem.stopIntake();
            robot.intakeSystem.restFeeder();
            while (opModeInInit()) {
                if (gamepad1.x) target = AprilTagVision.BLUE_SCORING;
                if (gamepad1.b) target = AprilTagVision.RED_SCORING;
                telemetry.addLine("X = Blue, B = Red. Turns in place only; no shooting.");
                telemetry.addData("Target", target);
                telemetry.addData("Camera", robot.vision.getCameraState());
                telemetry.update();
                sleep(20);
            }
            if (!opModeIsActive()) return;
            boolean aligned = AutonomousCameraAligner.align(this, robot, target);
            while (opModeIsActive()) {
                telemetry.addData("Final alignment", aligned ? "ALIGNED" : "FAILED / TIMED OUT");
                telemetry.addData("Camera", robot.vision.getCameraState());
                telemetry.addLine("Motors stopped. Press STOP, then INIT to repeat.");
                telemetry.update();
                sleep(50);
            }
        } finally {
            if (initialized) {
                robot.drive(0, 0);
                robot.shooter.spinDown();
            }
            robot.close();
        }
    }
}
