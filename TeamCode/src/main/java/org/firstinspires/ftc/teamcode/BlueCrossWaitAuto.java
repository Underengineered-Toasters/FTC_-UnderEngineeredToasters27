package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Alliance - Wait + Cross + Park", group = "Blue")
public class BlueCrossWaitAuto extends BioBuzzTimedAuto {
    @Override protected boolean isRed() { return false; }
    @Override protected Plan plan() { return Plan.WAIT_SHOOT_CROSS_PARK; }
}
