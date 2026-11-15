package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Odometry Sensor Test", group = "Testing")
public class OdometryTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Pod encoder plugged into motor encoder port 0.
        DcMotorEx forwardEncoder =
                hardwareMap.get(DcMotorEx.class, "frontleft");

        // Pod encoder plugged into motor encoder port 1.
        DcMotorEx sidewaysEncoder =
                hardwareMap.get(DcMotorEx.class, "frontright");

        IMU imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        // Ensure all four drive motors remain unpowered.
        DcMotor[] driveMotors = {
                hardwareMap.get(DcMotor.class, "frontleft"),
                hardwareMap.get(DcMotor.class, "frontright"),
                hardwareMap.get(DcMotor.class, "backleft"),
                hardwareMap.get(DcMotor.class, "backright")
        };

        for (DcMotor motor : driveMotors) {
            motor.setPower(0);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        telemetry.addLine("Sensor test only: motors remain unpowered.");
        telemetry.addLine("Press START, then push the robot by hand.");
        telemetry.addLine("A resets displayed ticks and heading.");
        telemetry.update();

        waitForStart();

        int forwardStart = forwardEncoder.getCurrentPosition();
        int sidewaysStart = sidewaysEncoder.getCurrentPosition();
        imu.resetYaw();

        boolean previousA = false;

        try {
            while (opModeIsActive()) {
                if (gamepad1.a && !previousA) {
                    forwardStart = forwardEncoder.getCurrentPosition();
                    sidewaysStart = sidewaysEncoder.getCurrentPosition();
                    imu.resetYaw();
                }
                previousA = gamepad1.a;

                int forwardTicks =
                        forwardEncoder.getCurrentPosition() - forwardStart;

                int sidewaysTicks =
                        sidewaysEncoder.getCurrentPosition() - sidewaysStart;

                double heading = imu.getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.DEGREES);

                telemetry.addData("Forward pod ticks", forwardTicks);
                telemetry.addData("Sideways pod ticks", sidewaysTicks);
                telemetry.addData("Heading degrees", "%.1f", heading);
                telemetry.addLine("Push forward, then sideways.");
                telemetry.addLine("Rotate counterclockwise to test heading.");
                telemetry.update();

                idle();
            }
        } finally {
            for (DcMotor motor : driveMotors) {
                motor.setPower(0);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
    }
}