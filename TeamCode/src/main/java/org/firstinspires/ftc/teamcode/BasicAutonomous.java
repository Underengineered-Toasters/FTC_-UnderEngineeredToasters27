//imports and packages
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Basic Auto - Drive Forward", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {
//Same as before, make sure names match driverStation
    private DcMotor left;
    private DcMotor right;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        //Same as before, set motors opposite during tank drive
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //Waits for PLAY
        waitForStart();
        runtime.reset();

        //Give motors 50% power = 50% max velocity
        left.setPower(0.5);
        right.setPower(0.5);

        while (opModeIsActive() && runtime.seconds() < 2.0) {
            telemetry.addData("Auto", "Driving forward: %.1f sec", runtime.seconds());
            telemetry.update();
        }

        left.setPower(0);
        right.setPower(0);

        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}
