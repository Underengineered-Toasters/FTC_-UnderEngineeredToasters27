//imports and packages
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Basic Auto - Drive Forward", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {
//Same as before, make sure names match driverStation
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");
        //Same as before, set motors opposite during tank drive
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Waits for PLAY
        waitForStart();
        runtime.reset();

        //Give motors 50% power = 50% max velocity
        leftDrive.setPower(0.5);
        rightDrive.setPower(0.5);

        while (opModeIsActive() && runtime.seconds() < 2.0) {
            telemetry.addData("Auto", "Driving forward: %.1f sec", runtime.seconds());
            telemetry.update();
        }

        leftDrive.setPower(0);
        rightDrive.setPower(0);

        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}
