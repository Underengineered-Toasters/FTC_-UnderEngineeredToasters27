package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Red Alliance - Wait + Return Shoot", group = "Red")
public class RedAutonomous extends BioBuzzTimedAuto {
    @Override protected boolean isRed() { return true; }
    @Override protected Plan plan() { return Plan.WAIT_SHOOT_RETURN_SHOOT; }
}
