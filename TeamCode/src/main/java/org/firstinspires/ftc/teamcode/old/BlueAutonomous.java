package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Blue Alliance - Wait + Return Shoot", group = "Blue")
public class BlueAutonomous extends BioBuzzTimedAuto {
    @Override protected boolean isRed() { return false; }
    @Override protected Plan plan() { return Plan.WAIT_SHOOT_RETURN_SHOOT; }
}
