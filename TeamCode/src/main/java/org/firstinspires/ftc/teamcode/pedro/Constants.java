package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/** Hardware and calibrated Pedro Pathing configuration for the competition robot. */
public final class Constants {
    private Constants() { }

    /** Safety switch for generated autonomous paths. Set true only after confirming localization. */
    public static final boolean AUTONOMOUS_ARMED = false;

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontleft");
        c.frontRightName.set("frontright");
        c.backLeftName.set("backleft");
        c.backRightName.set("backright");

        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.manualBrakeMode.set(true);
    });

    // Values measured with the team's Two Wheel AutoTune procedure.
    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
        // The dedicated odometry pods are connected to the encoder pins of these motor ports.
        c.xPodName.set("frontleft");
        c.yPodName.set("frontright");
        c.imuName.set("imu");
        c.xPodOffset.set(-3.793554066902618);
        c.yPodOffset.set(-6.191647312322701);
        c.forwardTicksToInches.set(0.0016335172015208453);
        c.strafeTicksToInches.set(0.0018013083543485444);
        c.xPodDirection.set(Encoder.FORWARD);
        c.yPodDirection.set(Encoder.FORWARD);
        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        )));
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryForward = Controller.proportional(0.3);
        Controller secondaryForward = Controller.proportional(0.1);
        Controller primaryStrafe = Controller.proportional(0.3);
        Controller secondaryStrafe = Controller.proportional(0.1);

        c.forwardTranslational.set(Controller.piecewise(secondaryForward).put(2.5, primaryForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryStrafe).put(2.5, primaryStrafe));
        c.coast.set(Controller.proportionalFeedforward(0.010978350889324107));
        c.brake.set(Controller.proportionalFeedforward(0.008731598255925491));
        c.headingFeedback.set(Controller.proportional(5.258721785960744));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642143125655298, 0.0063829525363003695));
        c.linearBrakeCoefficients.set(Matrix.diag(0.10605894992901523, 0.08719146175596092));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.0014663966976606565, 0.0013837064502458813));
        c.maxAchievableForwardVelocity.set(66.88);
        c.maxAchievableStrafeVelocity.set(66.88);
        c.naturalForwardDeceleration.set(85.01144677379789);
        c.naturalStrafeDeceleration.set(104.49787535782846);
        c.maxAccelerationConstraint.set(66.88);
        c.maxDecelerationConstraint.set(66.88);
        c.headingConstraint.set(3.4);
    });

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(
                new TwoWheelLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
