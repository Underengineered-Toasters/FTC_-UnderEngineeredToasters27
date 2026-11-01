package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red Alliance TeleOp", group = "Main")
public final class RedAllianceTeleOp extends RobotTeleOp {
    @Override protected String getAllianceTarget() { return AprilTagVision.RED_AUDIENCE; }
}
