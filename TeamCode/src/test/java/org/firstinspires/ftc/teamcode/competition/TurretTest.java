package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.junit.Test;
import java.lang.reflect.Proxy;
import static org.junit.Assert.*;

public class TurretTest {
    private double power = 1;
    private DcMotor.RunMode mode;
    private DcMotor.ZeroPowerBehavior zeroPower;
    private DcMotor.Direction direction;

    private Turret initialize() {
        DcMotor motor = (DcMotor) Proxy.newProxyInstance(DcMotor.class.getClassLoader(),
                new Class<?>[]{DcMotor.class}, (proxy, method, args) -> {
                    if (method.getName().equals("setPower")) power = (Double) args[0];
                    if (method.getName().equals("getPower")) return power;
                    if (method.getName().equals("setMode")) mode = (DcMotor.RunMode) args[0];
                    if (method.getName().equals("setDirection")) direction = (DcMotor.Direction) args[0];
                    if (method.getName().equals("setZeroPowerBehavior")) zeroPower = (DcMotor.ZeroPowerBehavior) args[0];
                    return null;
                });
        Turret turret = new Turret(); turret.configure(motor); return turret;
    }

    @Test public void initializesStoppedWithBrakeAndNoEncoderReset() {
        initialize(); assertEquals(0, power, 0);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, mode);
        assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, zeroPower);
        assertEquals(DcMotor.Direction.REVERSE, direction);
    }

    @Test public void realAlignmentOutputDrivesTurretAndReleaseStopsIt() {
        Turret turret = initialize();
        HoldToAlignController controller = new HoldToAlignController(25);
        turret.applyAlignment(controller.update(true, 100, true, false, true,
                new ClusterAlignmentController.Sample(40, 72, 100)));
        assertEquals(0.15, power, 0);
        turret.applyAlignment(controller.update(false, 200, true, false, false, null));
        assertEquals(0, power, 0);
        turret.applyAlignment(controller.update(true, 300, true, false, true,
                new ClusterAlignmentController.Sample(10, 72, 300)));
        assertEquals(-0.15, power, 0);
        turret.applyAlignment(controller.update(true, 400, true, false, true, null));
        assertEquals(0.10, power, 0);
    }

    @Test public void cameraFailureAndTimeoutStopMotor() {
        Turret turret = initialize();
        HoldToAlignController controller = new HoldToAlignController();
        turret.applyAlignment(controller.update(true, 0, true, false, true,
                new ClusterAlignmentController.Sample(20, 72, 0)));
        turret.applyAlignment(controller.update(true, 100, false, true, false, null));
        assertEquals(0, power, 0);
        turret.applyAlignment(controller.update(true, 200, true, false, true,
                new ClusterAlignmentController.Sample(20, 72, 200)));
        turret.applyAlignment(controller.update(true, 6_000_000_000L, true, false, false, null));
        assertEquals(0, power, 0);
    }

    @Test public void invalidPowerAndStopCommandZero() {
        Turret turret = initialize();
        turret.setPower(10); assertEquals(0.15, power, 0);
        turret.setPower(Double.NaN); assertEquals(0, power, 0);
        turret.setPower(-10); assertEquals(-0.15, power, 0);
        turret.stop(); assertEquals(0, power, 0);
    }
}
