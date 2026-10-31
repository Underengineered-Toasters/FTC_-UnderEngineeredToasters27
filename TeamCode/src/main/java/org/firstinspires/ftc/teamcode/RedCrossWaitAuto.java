package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Alliance - Wait + Cross + Park", group = "Red")
public class RedCrossWaitAuto extends BioBuzzTimedAuto {
    @Override protected boolean isRed() { return true; }
    @Override protected Plan plan() { return Plan.WAIT_SHOOT_CROSS_PARK; }
}
