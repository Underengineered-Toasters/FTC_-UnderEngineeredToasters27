package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "Red Auton 2", group = "Autonomous")
public class Red_Auton_2 extends PedroAutonomousBase {
    private final PoseFactory poses = PoseFactory.degrees();
    private final Pose start = poses.of(58.6614, 10.8093, 270);
    private final Pose startToShootEnd = poses.of(59.0113, 10.7598, 270);
    private final Pose gardenEnd = poses.of(13.5686, 10.9092, 180);
    private final Pose pollenEnd = poses.of(15.2816, 46.7448, 180);
    private final Pose pollenControl = poses.of(20.811, 53.3767, 0);
    private final Pose returnEnd = poses.of(36.3087, 33.962, 180);
    private final Pose returnControl = poses.of(23.3985, 31.6416, 0);
    private final Pose parkEnd = poses.of(11.0715, 96.2541, 180);
    private final Pose parkControl = poses.of(10.3131, 65.4271, 0);

    @Override protected Pose startPose() { return start; }
    @Override protected Command autoRoutine() {
        return sequential(follow(follower, startToShoot()), follow(follower, shootToGarden()),
                follow(follower, shootToPollen()), follow(follower, pollenToShoot()), follow(follower, park()));
    }
    private Path startToShoot() { return Paths.line(start, startToShootEnd).linear(start, startToShootEnd); }
    private Path shootToGarden() { return Paths.line(startToShootEnd, gardenEnd).linear(startToShootEnd, gardenEnd); }
    private Path shootToPollen() { return Paths.curve(gardenEnd, pollenControl, pollenEnd).linear(gardenEnd, pollenEnd); }
    private Path pollenToShoot() { return Paths.curve(pollenEnd, returnControl, returnEnd).linear(pollenEnd, returnEnd); }
    private Path park() { return Paths.curve(returnEnd, parkControl, parkEnd).linear(returnEnd, parkEnd); }
}
