package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.robot.DualShooterKinematics;

public final class KinematicsModel {
    private final Vector2d botPose;
    private final Vector2d hivePose;

    private final double targetHiveHeightInInches;
    private final double shooterHeightInInches;
    private final double launchAngleInDegree;


    public KinematicsModel(Vector2d botPose, Vector2d hivePose) {
        this(botPose, hivePose, DualShooterKinematics.TARGET_HIVE_HEIGHT_INCHES,
                DualShooterKinematics.SHOOTER_HEIGHT_INCHES, DualShooterKinematics.LAUNCH_ANGLE_DEG);
    }

    public KinematicsModel(Vector2d botPose, Vector2d hivePose, double targetHiveHeightInInches,
                           double shooterHeightInInches, double launchAngleInDegree) {
        this.botPose = botPose;
        this.hivePose = hivePose;
        this.targetHiveHeightInInches = targetHiveHeightInInches;
        this.shooterHeightInInches = shooterHeightInInches;
        this.launchAngleInDegree = launchAngleInDegree;
    }

    public double getHorizontalDistanceInches() {
        return Math.hypot(hivePose.x - botPose.x, hivePose.y - botPose.y);
    }

    public double getTargetHiveHeightInInches() {
        return targetHiveHeightInInches;
    }

    public double getShooterHeightInInches() {
        return shooterHeightInInches;
    }

    public double getLaunchAngleInDegree() {
        return launchAngleInDegree;
    }

    public Vector2d getBotPose() {
        return botPose;
    }

    public Vector2d getHivePose() {
        return hivePose;
    }
}
