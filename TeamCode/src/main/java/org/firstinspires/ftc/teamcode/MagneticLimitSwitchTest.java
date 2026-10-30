package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * TEST OPMODE FOR TURRET MAGNETIC LIMIT SWITCH
 * 
 * Hardware Setup:
 * - Plugged into Digital Port 0-1 on the Control Hub.
 * - Driver Station Configuration Name: "mag" (Digital Port 0)
 */
@TeleOp(name = "Magnetic Limit Switch Test", group = "Test")
public class MagneticLimitSwitchTest extends LinearOpMode {

    private TouchSensor turretLimit;
    private DigitalChannel digitalChannelLimit;

    @Override
    public void runOpMode() {

        // Try initializing using config name "mag" (with fallback to "turretLimit")
        try {
            turretLimit = hardwareMap.get(TouchSensor.class, "mag");
        } catch (Exception e) {
            try {
                digitalChannelLimit = hardwareMap.get(DigitalChannel.class, "mag");
                if (digitalChannelLimit != null) {
                    digitalChannelLimit.setMode(DigitalChannel.Mode.INPUT);
                }
            } catch (Exception ex) {
                try {
                    turretLimit = hardwareMap.get(TouchSensor.class, "turretLimit");
                } catch (Exception ex2) {
                    turretLimit = null;
                    digitalChannelLimit = null;
                }
            }
        }

        telemetry.addLine("=================================");
        telemetry.addLine("  MAGNETIC LIMIT SWITCH TEST     ");
        telemetry.addLine("  Config Name: mag               ");
        telemetry.addLine("  Plugged into: Digital Port 0   ");
        telemetry.addLine("=================================");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean isMagnetFound = false;

            if (turretLimit != null) {
                isMagnetFound = turretLimit.isPressed();
            } else if (digitalChannelLimit != null) {
                // DigitalChannel active low: getState() is false when magnet is detected
                isMagnetFound = !digitalChannelLimit.getState();
            }

            telemetry.addLine("=================================");
            if (isMagnetFound) {
                telemetry.addLine("      STATUS: found!             ");
                telemetry.addLine("   >> MAGNET DETECTED! <<        ");
            } else {
                telemetry.addLine("   STATUS: Searching for magnet... ");
            }
            telemetry.addLine("=================================");
            telemetry.addData("Magnet Detected", isMagnetFound ? "YES (found!)" : "NO");
            telemetry.update();
        }
    }
}
