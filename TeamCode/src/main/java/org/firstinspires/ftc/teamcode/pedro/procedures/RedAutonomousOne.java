package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/** Paths transcribed from the supplied Red Auton 1 Pedro planner JSON. */
@Autonomous(name = "RedAutonomousOne", group = "Autonomous")
public class RedAutonomousOne extends PedroAutonomousBase {
    private final PoseFactory poses = PoseFactory.degrees();
    private final Pose start = poses.of(58.326532250287755, 132.08412605692226, 90);
    private final Pose highTarget1 = poses.of(57.852151135292175, 132.12861985831424, 90);
    private final Pose intake1 = poses.of(46.47072740078343, 129.3126253498385, 90);
    private final Pose intake1Control = poses.of(56.14902441329717, 119.98668050018598, 0);
    private final Pose highTarget2 = poses.of(46.927206739167524, 23.901061185876863, 0);
    private final Pose highTarget2Control = poses.of(0, 36.59482420200838, 0);
    private final Pose intake2 = poses.of(12.284334947364943, 46.54265206903628, 180);
    private final Pose intake2Control = poses.of(39.588936306466124, 52.41562737491811, 0);
    private final Pose highTarget3 = poses.of(47, 24.564263322884013, 0);
    private final Pose highTarget3Control = poses.of(21.328491497936664, 24.98797087235884, 0);
    private final Pose parkEnd = poses.of(7.742999070431882, 96.28483653201997, 0);
    private final Pose parkControl = poses.of(6.54271159874608, 60.252090992225, 0);

    @Override protected Pose startPose() { return start; }
    @Override protected Command autoRoutine() {
        // Planner pauses are not included: no mechanism commands were supplied for those waits.
        return sequential(follow(follower, startToHighTarget1()), follow(follower, highTarget1ToIntake1()),
                follow(follower, intake1ToHighTarget2()), follow(follower, highTarget2ToIntake2()),
                follow(follower, intake2ToHighTarget3()), follow(follower, park()));
    }
    private Path startToHighTarget1() { return Paths.line(start, highTarget1).linear(start, highTarget1); }
    private Path highTarget1ToIntake1() { return Paths.curve(highTarget1, intake1Control, intake1).linear(highTarget1, intake1); }
    private Path intake1ToHighTarget2() { return Paths.curve(intake1, highTarget2Control, highTarget2).linear(intake1, highTarget2); }
    private Path highTarget2ToIntake2() { return Paths.curve(highTarget2, intake2Control, intake2).linear(highTarget2, intake2); }
    private Path intake2ToHighTarget3() { return Paths.curve(intake2, highTarget3Control, highTarget3).linear(intake2, highTarget3); }
    private Path park() { return Paths.curve(highTarget3, parkControl, parkEnd).linear(highTarget3, parkEnd); }
}
