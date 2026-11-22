package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.TimedMotion;

/**
 * Timed out-and-back route: forward 7 in, left 90, forward 46 in, wait 2 s, left 180,
 * forward 46 in, left 90, back 7 in.
 *
 * <p>This route used to negate both drive axes because an older chassis drove backward for
 * positive power. {@link Robot} now normalizes its motors so positive power is forward, so that
 * negation made every leg and every turn run mirrored (the "forward" legs reversed and
 * {@code turnLeft} turned right). The axes are no longer negated.
 *
 * <p>Distances and headings are estimates and must be calibrated on the field
 * ({@link TimedMotion} holds the calibration).
 */
@Autonomous(name = "Basic Out and Back Auto", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {

    /** Travel power for the short legs. */
    public static double SHORT_LEG_POWER = 0.30;

    /** Travel power for the long legs. */
    public static double LONG_LEG_POWER = 0.50;

    /** Pause at the far end of the route. */
    public static double PAUSE_SECONDS = 2.0;

    private final Robot robot = new Robot();
    private TimedMotion motion;

    @Override
    public void runOpMode() {
        try {
            robot.init(hardwareMap);
            motion = TimedMotion.forRobot(robot, this);

            telemetry.addLine("Basic Out and Back Auto");
            telemetry.addLine("Forward 7 in, left 90, forward 46 in, wait 2 s");
            telemetry.addLine("Turn 180, forward 46 in, left 90, back 7 in");
            telemetry.update();

            waitForStart();
            if (!opModeIsActive()) {
                return;
            }

            ElapsedTime runtime = new ElapsedTime();

            motion.driveInches(7.0, SHORT_LEG_POWER);
            motion.turnDegrees(-90.0);
            motion.driveInches(46.0, LONG_LEG_POWER);

            if (!motion.waitSeconds(PAUSE_SECONDS)) {
                return;
            }

            // Half turn, retrace the 46 in leg, restore the starting heading, back up the last leg.
            motion.turnDegrees(-180.0);
            motion.driveInches(46.0, LONG_LEG_POWER);
            motion.turnDegrees(-90.0);
            motion.driveInches(-7.0, SHORT_LEG_POWER);

            if (opModeIsActive()) {
                telemetry.addLine("Out and back route completed");
                telemetry.addData("Total Time Elapsed", "%.2f sec", runtime.seconds());
                telemetry.update();
            }
        } finally {
            if (motion != null) {
                motion.stop();
            }
            robot.close();
        }
    }
}
