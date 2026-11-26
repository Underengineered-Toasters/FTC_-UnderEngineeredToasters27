package org.firstinspires.ftc.teamcode.messages;

public final class KinematicsModel {
    private final double horizontalDistanceInches;
    private final double targetHiveHeightInInches;
    private final double shooterHeightInInches;
    private final double launchAngleInDegree;

    public KinematicsModel(double horizontalDistanceInches, double targetHiveHeightInInches, double shooterHeightInInches, double launchAngleInDegree) {
        this.horizontalDistanceInches = horizontalDistanceInches;
        this.targetHiveHeightInInches = targetHiveHeightInInches;
        this.shooterHeightInInches = shooterHeightInInches;
        this.launchAngleInDegree = launchAngleInDegree;
    }

    public double getHorizontalDistanceInches() {
        return horizontalDistanceInches;
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
}
