package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Ball Identifier")
public class ColorSensorTest extends LinearOpMode {

    private ColorSensor colorSensor;

    @Override
    public void runOpMode() {

        // Must match the name in your Robot Configuration
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        telemetry.addLine("Color Sensor Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            String ballType;

            // RED NECTAR
            // Red should be much stronger than green and blue
            if (red > green * 1.5 &&
                    red > blue * 1.5) {

                ballType = "RED NECTAR";
            }

            // POLLEN
            // Yellow should have strong red + green,
            // with green fairly close to red
            else if (red > blue * 1.4 &&
                    green > blue * 1.4 &&
                    green > red * 0.60) {

                ballType = "POLLEN";
            }

            // BLUE NECTAR
            // Blue should be much stronger than red and green
            else if (blue > red * 1.3 &&
                    blue > green * 1.3) {

                ballType = "BLUE NECTAR";
            }

            // If no clear color is detected
            else {

                ballType = "UNKNOWN";
            }

            // Show raw RGB values
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);

            // Show useful ratios for calibration
            if (red > 0) {
                telemetry.addData(
                        "Green / Red",
                        "%.2f",
                        (double) green / red
                );
            }

            if (blue > 0) {
                telemetry.addData(
                        "Red / Blue",
                        "%.2f",
                        (double) red / blue
                );

                telemetry.addData(
                        "Green / Blue",
                        "%.2f",
                        (double) green / blue
                );
            }

            telemetry.addLine("-------------------");

            // Show detected game piece
            telemetry.addData("Detected", ballType);

            telemetry.update();

            sleep(50);
        }
    }
}