package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Alliance - Shoot Now + Return", group = "Blue")
public class BlueShootNowAuto extends BioBuzzTimedAuto {
    @Override protected boolean isRed() { return false; }
    @Override protected Plan plan() { return Plan.SHOOT_RETURN_WAIT_SHOOT; }
}
