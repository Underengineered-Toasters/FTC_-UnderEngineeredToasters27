package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Alliance - Shoot Now + Cross + Park", group = "Blue")
public class BlueCrossNowAuto extends BioBuzzTimedAuto {
    @Override protected boolean isRed() { return false; }
    @Override protected Plan plan() { return Plan.SHOOT_CROSS_PARK; }
}
