package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Blue_W5_S1_f1.I_S2_f2.I_S2_P", group = "Autonomous")
public class Blue_W5_S1_f1_I_S2_f2_I_S2_P extends PedroAutonomousBase {
    private final PoseFactory poses = PoseFactory.degrees();
    private final Pose start = poses.of(80.6244, 10.1776, 270);
    private final Pose path1End = poses.of(93.7389, 14.7398, 270);
    private final Pose path1Control = poses.of(91.7906, 36.6428, 0);
    private final Pose path2End = poses.of(109.6234, 111.8087, 270);
    private final Pose path2Control = poses.of(141.5, 97.4009, 0);
    private final Pose path3End = poses.of(123.6903, 94.2741, 270);
    private final Pose path3Control = poses.of(99.8339, 101.5041, 0);
    private final Pose path4End = poses.of(102.1576, 106.8947, 270);
    private final Pose path5End = poses.of(129.591, 45.6753, 270);
    private final Pose path5Control = poses.of(124.7685, 95.7913, 0);

    @Override protected Pose startPose() { return start; }
    @Override protected Command autoRoutine() {
        return sequential(follow(follower, path1()), follow(follower, path2()), follow(follower, path3()),
                follow(follower, path4()), follow(follower, path5()));
    }
    private Path path1() { return Paths.curve(start, path1Control, path1End).linear(start, path1End); }
    private Path path2() { return Paths.curve(path1End, path2Control, path2End).linear(path1End, path2End); }
    private Path path3() { return Paths.curve(path2End, path3Control, path3End).linear(path2End, path3End); }
    private Path path4() { return Paths.line(path3End, path4End).linear(path3End, path4End); }
    private Path path5() { return Paths.curve(path4End, path5Control, path5End).linear(path4End, path5End); }
}
