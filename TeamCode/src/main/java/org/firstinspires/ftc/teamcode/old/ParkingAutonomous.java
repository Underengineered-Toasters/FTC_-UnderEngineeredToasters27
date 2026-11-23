package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.TimedMotion;

/**
 * Drive-only parking estimate: backward 7 in, right 90 degrees, backward 46 in.
 *
 * <p>Runs on the raw {@code left}/{@code right} motors and shares the
 * {@link TimedMotion} calibration with the other timed routes.
 */
@Autonomous(name = "Parking Autonomous", group = "Autonomous")
public class ParkingAutonomous extends LinearOpMode {
    // Starting estimates, not measured parking coordinates.
    public static final double FIRST_BACKWARD_INCHES = 7.0;
    public static final double SECOND_BACKWARD_INCHES = 46.0;
    public static final double SHORT_LEG_POWER = 0.30;
    public static final double LONG_LEG_POWER = 0.50;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private TimedMotion motion;

    @Override
    public void runOpMode() {
        try {
            // Do not initialize Robot: it also initializes shooters, intake, camera and loader.
            // Only the four mecanum drive motors, using Robot's shared names and directions.
            leftFront = hardwareMap.get(DcMotor.class, Robot.LEFT_FRONT_MOTOR);
            leftBack = hardwareMap.get(DcMotor.class, Robot.LEFT_BACK_MOTOR);
            rightFront = hardwareMap.get(DcMotor.class, Robot.RIGHT_FRONT_MOTOR);
            rightBack = hardwareMap.get(DcMotor.class, Robot.RIGHT_BACK_MOTOR);

            Robot.applyMecanumDirections(leftFront, leftBack, rightFront, rightBack);

            for (DcMotor motor : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            motion = TimedMotion.forRawMotors(
                    (forward, turn) -> applyDrive(forward, 0.0, turn), this, hardwareMap);

            telemetry.addLine("Parking Autonomous - drive only, estimated distances");
            telemetry.addLine("Backward 7 inches, right 90 degrees, backward 46 inches");
            telemetry.addLine("No shooter, intake, loader or camera actions");
            telemetry.update();

            waitForStart();
            if (!opModeIsActive()) {
                return;
            }

            motion.driveInches(-FIRST_BACKWARD_INCHES, SHORT_LEG_POWER);
            motion.turnDegrees(90.0);
            motion.driveInches(-SECOND_BACKWARD_INCHES, LONG_LEG_POWER);

            if (opModeIsActive()) {
                telemetry.addLine("Parking route complete");
                telemetry.update();
            }
        } finally {
            if (motion != null) {
                motion.stop();
            }
            for (DcMotor motor : new DcMotor[]{leftFront, leftBack, rightFront, rightBack}) {
                if (motor != null) {
                    motor.setPower(0);
                }
            }
        }
    }

    private void applyDrive(double forward, double strafe, double turn) {
        Robot.driveMecanum(leftFront, leftBack, rightFront, rightBack, forward, strafe, turn);
    }
}
