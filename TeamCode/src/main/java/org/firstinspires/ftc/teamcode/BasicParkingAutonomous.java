package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Drive-only parking estimate; leaves the existing Basic Out and Back Auto intact. */
@Autonomous(name = "Basic Autonomous", group = "Autonomous")
public class BasicParkingAutonomous extends LinearOpMode {
    // Starting estimates from BasicAutonomous, not measured parking coordinates.
    public static final double FIRST_BACKWARD_INCHES = 7.0;
    public static final double SECOND_BACKWARD_INCHES = 46.0;
    private static final double INCHES_PER_SECOND_AT_HALF_POWER = 20.0;
    private static final double TURN_POWER = 0.565;
    private static final double SECONDS_PER_90_DEGREES = 0.60;

    private DcMotor left;
    private DcMotor right;

    @Override public void runOpMode() {
        try {
            // Do not initialize Robot: it also initializes shooters, intake, camera and loader.
            left = hardwareMap.get(DcMotor.class, "left");
            left.setPower(0);
            right = hardwareMap.get(DcMotor.class, "right");
            right.setPower(0);
            // Match main's Robot drive configuration: positive power is forward.
            left.setDirection(DcMotor.Direction.FORWARD);
            right.setDirection(DcMotor.Direction.REVERSE);
            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            telemetry.addLine("Basic Autonomous - drive only, estimated distances");
            telemetry.addLine("Backward 7 inches, right 90 degrees, backward 46 inches");
            telemetry.addLine("No shooter, intake, loader or camera actions");
            telemetry.update();
            waitForStart();
            if (!opModeIsActive()) return;

            driveInches(-FIRST_BACKWARD_INCHES, 0.3);
            turn90(true);
            driveInches(-SECOND_BACKWARD_INCHES, 0.5);
            if (opModeIsActive()) {
                telemetry.addLine("Parking route complete");
                telemetry.update();
            }
        } finally {
            if (left != null) left.setPower(0);
            if (right != null) right.setPower(0);
        }
    }

    private void driveInches(double inches, double power) {
        double seconds = Math.abs(inches) / (INCHES_PER_SECOND_AT_HALF_POWER * (power / 0.5));
        driveFor(Math.copySign(power, inches), 0, seconds);
    }

    private void turn90(boolean rightTurn) {
        driveFor(0, rightTurn ? TURN_POWER : -TURN_POWER, SECONDS_PER_90_DEGREES);
    }

    private void driveFor(double forward, double turn, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            left.setPower(forward + turn);
            right.setPower(forward - turn);
            telemetry.addData("Drive / turn", "%.2f / %.2f", forward, turn);
            telemetry.addData("Step time", "%.2f / %.2f seconds", timer.seconds(), seconds);
            telemetry.update();
            idle();
        }
        left.setPower(0);
        right.setPower(0);
    }
}
