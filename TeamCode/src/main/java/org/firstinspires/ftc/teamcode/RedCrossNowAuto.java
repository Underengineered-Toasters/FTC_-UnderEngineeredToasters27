package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Alliance - Shoot Now + Cross + Park", group = "Red")
public class RedCrossNowAuto extends BioBuzzTimedAuto {
    @Override protected boolean isRed() { return true; }
    @Override protected Plan plan() { return Plan.SHOOT_CROSS_PARK; }
}
