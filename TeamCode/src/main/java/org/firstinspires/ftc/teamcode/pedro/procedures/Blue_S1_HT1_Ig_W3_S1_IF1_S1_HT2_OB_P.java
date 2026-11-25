package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Blue_S1_HT1_Ig_W3_S1_IF1_S1_HT2_OB_P", group = "Autonomous")
public class Blue_S1_HT1_Ig_W3_S1_IF1_S1_HT2_OB_P extends PedroAutonomousBase {
    private final PoseFactory poses = PoseFactory.degrees();
    private final Pose start = poses.of(83.5265, 132.0965, 0);
    private final Pose path1End = poses.of(83.6547, 130.6821, 0);
    private final Pose path2End = poses.of(130.0465, 131.7011, 0);
    private final Pose path3End = poses.of(127.5841, 94.1825, 0);
    private final Pose path3Control = poses.of(120.5729, 100.2656, 0);
    private final Pose path4End = poses.of(117.1404, 102.1337, 0);
    private final Pose objectBankEnd = poses.of(84.2548, 106.9451, 0);
    private final Pose parkEnd = poses.of(131.2595, 47.9025, 0);
    private final Pose parkControl = poses.of(131.9087, 104.1584, 0);

    @Override protected Pose startPose() { return start; }
    @Override protected Command autoRoutine() {
        return sequential(follow(follower, path1()), follow(follower, path2()), follow(follower, path3()),
                follow(follower, path4()), follow(follower, objectBank()), follow(follower, park()));
    }
    private Path path1() { return Paths.line(start, path1End).linear(start, path1End); }
    private Path path2() { return Paths.line(path1End, path2End).linear(path1End, path2End); }
    private Path path3() { return Paths.curve(path2End, path3Control, path3End).linear(path2End, path3End); }
    private Path path4() { return Paths.line(path3End, path4End).linear(path3End, path4End); }
    private Path objectBank() { return Paths.line(path4End, objectBankEnd).linear(path4End, objectBankEnd); }
    private Path park() { return Paths.curve(objectBankEnd, parkControl, parkEnd).linear(objectBankEnd, parkEnd); }
}
