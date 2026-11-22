package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.TimedMotion;

/**
 * Drive-only parking estimate: forward 7, right 90, forward 46, left 90, forward 7, left 90,
 * forward 46, left 90, back 7.
 *
 * <p>Runs on the raw {@code left}/{@code right} motors so it does not initialize the shooter,
 * intake, loader or camera. Distances, headings and battery compensation now come from
 * {@link TimedMotion}, so this route and {@link ParkingAutonomous} share one calibration
 * instead of carrying their own copies.
 */
@Autonomous(name = "Basic Autonomous", group = "Autonomous")
public class BasicParkingAutonomous extends LinearOpMode {
    // Starting estimates, not measured parking coordinates.
    public static final double FIRST_FORWARD_INCHES = 7.0;
    public static final double SECOND_FORWARD_INCHES = 46.0;
    public static final double THIRD_FORWARD_INCHES = 7.0;
    public static final double FOURTH_FORWARD_INCHES = 46.0;
    public static final double BACKWARD_INCHES = 7.0;
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

            telemetry.addLine("Basic Autonomous - drive only, estimated distances");
            telemetry.addLine("Forward 7, right 90, forward 46, left 90");
            telemetry.addLine("Forward 7, left 90, forward 46, left 90, back 7");
            telemetry.addLine("No shooter, intake, loader or camera actions");
            telemetry.update();

            waitForStart();
            if (!opModeIsActive()) {
                return;
            }

            motion.driveInches(FIRST_FORWARD_INCHES, SHORT_LEG_POWER);
            motion.turnDegrees(90.0);
            motion.driveInches(SECOND_FORWARD_INCHES, LONG_LEG_POWER);
            motion.turnDegrees(-90.0);
            motion.driveInches(THIRD_FORWARD_INCHES, SHORT_LEG_POWER);
            motion.turnDegrees(-90.0);
            motion.driveInches(FOURTH_FORWARD_INCHES, LONG_LEG_POWER);
            motion.turnDegrees(-90.0);
            motion.driveInches(-BACKWARD_INCHES, SHORT_LEG_POWER);

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
