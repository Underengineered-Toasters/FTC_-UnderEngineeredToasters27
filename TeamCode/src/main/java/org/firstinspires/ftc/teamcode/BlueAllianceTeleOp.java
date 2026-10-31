package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue Alliance TeleOp", group = "Main")
public final class BlueAllianceTeleOp extends RobotTeleOp {
    @Override protected String getAllianceTarget() { return AprilTagVision.BLUE_AUDIENCE; }
}
