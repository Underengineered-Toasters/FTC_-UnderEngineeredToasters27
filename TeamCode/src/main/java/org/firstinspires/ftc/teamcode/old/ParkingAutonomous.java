package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    private DcMotor left;
    private DcMotor right;
    private TimedMotion motion;

    @Override
    public void runOpMode() {
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

            motion = TimedMotion.forRawMotors(this::applyDrive, this, hardwareMap);

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
            if (left != null) left.setPower(0);
            if (right != null) right.setPower(0);
        }
    }

    private void applyDrive(double forward, double turn) {
        left.setPower(clip(forward + turn));
        right.setPower(clip(forward - turn));
    }

    private static double clip(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }
}
