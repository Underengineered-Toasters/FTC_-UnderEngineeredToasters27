package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ds.DriverStation;

/**
 * Autonomous: drive the turret elevation motor to a commanded angle and confirm with the PID that
 * the angle was actually reached.
 *
 * <p>The desired angle is published to the Driver Station as soon as the OpMode initialises, so the
 * number on screen is the one the loop will chase before the match starts. Once running, the loop
 * reports the live angle, the error and whether the controller has settled inside tolerance, and it
 * ends with an explicit ACQUIRED / NOT ACQUIRED line rather than simply stopping.
 *
 * <p>Three ways the move can end, and none of them is "keep pushing forever":
 * <ul>
 *   <li><b>acquired</b> — inside {@link TurretElevation#TOLERANCE_DEGREES};</li>
 *   <li><b>timeout</b> — did not settle within {@link #MOVE_TIMEOUT_SECONDS};</li>
 *   <li><b>stall</b> — commanded near full power while the encoder did not move, which means a
 *       disconnected encoder or a binding mechanism, not a tuning problem.</li>
 * </ul>
 *
 * <p>Before this runs on the robot: the motor must exist in the Control Hub configuration under
 * {@link TurretElevation#MOTOR_NAME}, and {@link TurretElevation#TICKS_PER_DEGREE} must be measured.
 * Until the ticks-per-degree figure is real, the reported angle is not in degrees.
 */
@Autonomous(name = "Turret Elevation (PID)", group = "Autonomous")
public class TurretElevationAuto extends LinearOpMode {

    /** The elevation angle this autonomous run drives to, in degrees. */
    private static final double DESIRED_ANGLE_DEGREES = 45.0;

    /** Abort a move that has not settled within this many seconds. */
    private static final double MOVE_TIMEOUT_SECONDS = 5.0;

    /** Commanded power at or above this, with no encoder movement, means something is wrong. */
    private static final double STALL_POWER_THRESHOLD = 0.9;
    private static final double STALL_TIMEOUT_SECONDS = 0.5;
    private static final double STALL_DEGREE_EPSILON = 0.1;

    private final TurretElevation elevation = new TurretElevation();

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime moveTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        DriverStation.attach(this);

        elevation.init(hardwareMap);

        if (!elevation.isAvailable()) {
            telemetry.addLine("Motor \"" + TurretElevation.MOTOR_NAME + "\" is not in the robot configuration.");
            telemetry.addLine("Add it on the Control Hub, then run this OpMode again.");
            telemetry.update();
            DriverStation.log("Turret elevation auto: motor \"" + TurretElevation.MOTOR_NAME + "\" is missing");
            waitForStart();
            return;
        }

        // ---- the desired angle is on screen before the match starts ----
        telemetry.addLine("Turret elevation autonomous: ready");
        telemetry.addData("Desired angle (deg)", "%.1f", DESIRED_ANGLE_DEGREES);
        telemetry.addData("Current angle (deg)", "%.1f", elevation.getAngleDegrees());
        telemetry.addData("Tolerance (deg)", "%.1f", TurretElevation.TOLERANCE_DEGREES);
        telemetry.addData("Ticks per degree", "%.3f", TurretElevation.TICKS_PER_DEGREE);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            elevation.stop();
            return;
        }

        elevation.setTargetAngle(DESIRED_ANGLE_DEGREES);

        loopTimer.reset();
        moveTimer.reset();
        stallTimer.reset();

        double lastAngle = elevation.getAngleDegrees();
        boolean acquired = false;
        String stopReason = "opmode stopped";

        while (opModeIsActive()) {

            // OpMode loops are not fixed-rate: measure the period instead of assuming one.
            double dt = loopTimer.seconds();
            loopTimer.reset();

            elevation.update(dt);

            double angle = elevation.getAngleDegrees();
            double output = elevation.getLastOutput();

            if (elevation.atTarget()) {
                acquired = true;
                stopReason = String.format("acquired %.1f deg (target %.1f)", angle, DESIRED_ANGLE_DEGREES);
                break;
            }

            if (moveTimer.seconds() > MOVE_TIMEOUT_SECONDS) {
                stopReason = String.format("timeout after %.1fs at %.1f deg", MOVE_TIMEOUT_SECONDS, angle);
                break;
            }

            if (Math.abs(output) >= STALL_POWER_THRESHOLD
                    && Math.abs(angle - lastAngle) <= STALL_DEGREE_EPSILON) {
                if (stallTimer.seconds() > STALL_TIMEOUT_SECONDS) {
                    stopReason = "mechanism not moving under full power (encoder or binding)";
                    break;
                }
            } else {
                stallTimer.reset();
            }
            lastAngle = angle;

            telemetry.addData("Desired angle (deg)", "%.1f", DESIRED_ANGLE_DEGREES);
            telemetry.addData("Current angle (deg)", "%.1f", angle);
            telemetry.addData("Error (deg)", "%.1f", elevation.getErrorDegrees());
            telemetry.addData("Output (power)", "%.3f", output);
            telemetry.addData("Loop time (ms)", "%.1f", dt * 1000.0);
            telemetry.addData("Integral", "%.4f", elevation.getIntegralSum());
            telemetry.addData("At setpoint", elevation.atTarget());
            telemetry.update();

            DriverStation.telemetry("Elevation Desired", String.format("%.1f", DESIRED_ANGLE_DEGREES));
            DriverStation.telemetry("Elevation Actual", String.format("%.1f", angle));
            DriverStation.telemetry("Elevation Error", String.format("%.1f", elevation.getErrorDegrees()));
            DriverStation.telemetry("Elevation Output", String.format("%.3f", output));
            DriverStation.telemetry("Elevation AtSetpoint", String.valueOf(elevation.atTarget()));
        }

        // Always leave the mechanism at zero power, whatever the reason for stopping.
        elevation.stop();

        telemetry.addData("Desired angle (deg)", "%.1f", DESIRED_ANGLE_DEGREES);
        telemetry.addData("Final angle (deg)", "%.1f", elevation.getAngleDegrees());
        telemetry.addData("ACQUIRED", acquired);
        telemetry.addData("Stopped", stopReason);
        telemetry.update();

        DriverStation.log("Turret elevation auto: " + (acquired ? "ACQUIRED" : "NOT ACQUIRED") + " — " + stopReason);
    }
}
