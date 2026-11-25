package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import static com.pedropathing.ivy.Scheduler.schedule;

/** Shared safe runner for the generated Pedro path-only autonomous routines. */
public abstract class PedroAutonomousBase extends LinearOpMode {
    protected Follower follower;

    protected abstract Pose startPose();
    protected abstract Command autoRoutine();

    @Override
    public final void runOpMode() {
        if (!Constants.AUTONOMOUS_ARMED) {
            telemetry.addLine("AUTO DISARMED: set Constants.AUTONOMOUS_ARMED to true after calibration.");
            telemetry.update();
            waitForStart();
            return;
        }

        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose());
        follower.update();
        waitForStart();
        if (isStopRequested()) return;

        schedule(autoRoutine());
        try {
            while (opModeIsActive()) {
                follower.update();
                Scheduler.execute();
                telemetry.addData("x", "%.2f", follower.pose().x());
                telemetry.addData("y", "%.2f", follower.pose().y());
                telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(follower.pose().heading()));
                if (follower.currentPath() != null) {
                    telemetry.addData("Path number", follower.pathIndex());
                    telemetry.addData("Distance remaining", "%.2f", follower.distanceToEndpoint());
                }
                telemetry.update();
            }
        } finally {
            follower.stop();
            Scheduler.reset();
        }
    }
}
