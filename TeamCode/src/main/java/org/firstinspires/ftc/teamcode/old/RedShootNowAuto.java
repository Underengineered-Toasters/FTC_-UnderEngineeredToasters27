package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Alliance - Shoot Now + Return", group = "Red")
public class RedShootNowAuto extends BioBuzzTimedAuto {
    @Override protected boolean isRed() { return true; }
    @Override protected Plan plan() { return Plan.SHOOT_RETURN_WAIT_SHOOT; }
}
