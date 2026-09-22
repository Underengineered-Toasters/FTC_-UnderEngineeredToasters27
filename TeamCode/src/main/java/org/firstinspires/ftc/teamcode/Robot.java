package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Container for every robot subsystem. OpModes call {@link #init} once and
 * {@link #update} each loop iteration; subsystems that need per-loop work
 * (like the flywheel velocity command) are driven from here.
 */
public class Robot {

    public final Drivetrain drivetrain = new Drivetrain();
    public final Shooter shooter = new Shooter();
    public final Intake intake = new Intake();
    /** Opt-in: call {@code vision.init(hardwareMap)} only in OpModes that use the camera. */
    public final AprilTagVision vision = new AprilTagVision();


    public void init(HardwareMap hardwareMap) {
        drivetrain.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
    }

    /** Call once per loop. */
    public void update() {
        shooter.update();
    }
}
