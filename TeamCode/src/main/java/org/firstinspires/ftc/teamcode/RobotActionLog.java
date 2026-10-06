package org.firstinspires.ftc.teamcode;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/** CSV commands and hardware-reported status; does not measure feeder rotation. */
final class RobotActionLog implements AutoCloseable {
    private BufferedWriter writer;
    private String status;

    RobotActionLog(File directory) {
        try {
            if (!directory.isDirectory() && !directory.mkdirs()) {
                throw new IOException("Cannot create log directory");
            }
            File file = File.createTempFile("robot_actions_" + System.currentTimeMillis() + "_",
                    ".csv", directory);
            status = file.getAbsolutePath();
            writer = new BufferedWriter(new FileWriter(file));
            writer.write("time_ms,event,left_trigger,right_trigger,feeder_requested_power,"
                    + "feeder_commanded_power,feeder_pwm_enabled,feeder_port,feeder_connection,"
                    + "shooter_target_ticks_s,shooter_actual_ticks_s,intake_power,"
                    + "drive_left_power,drive_right_power\n");
            writer.flush();
        } catch (IOException e) {
            fail(e);
        }
    }

    void record(long timeMs, String event, double leftTrigger, double rightTrigger,
                double feederRequested, double feederCommanded, String pwmEnabled,
                int feederPort, String feederConnection, double shooterTarget,
                double shooterActual, double intakePower, double driveLeft, double driveRight) {
        if (writer == null) return;
        try {
            writer.write(String.format(Locale.US,
                    "%d,%s,%.3f,%.3f,%.3f,%.3f,%s,%d,%s,%.1f,%.1f,%.3f,%.3f,%.3f\n",
                    timeMs, quote(event), leftTrigger, rightTrigger, feederRequested,
                    feederCommanded, quote(pwmEnabled), feederPort, quote(feederConnection),
                    shooterTarget, shooterActual, intakePower, driveLeft, driveRight));
            writer.flush();
        } catch (IOException e) {
            fail(e);
        }
    }

    String getStatus() {
        return status;
    }

    private static String quote(String value) {
        return "\"" + value.replace("\"", "\"\"") + "\"";
    }

    private void fail(IOException error) {
        status = "CSV error: " + error.getMessage();
        if (writer != null) {
            try {
                writer.close();
            } catch (IOException ignored) {
                // Preserve the original error shown in telemetry.
            }
            writer = null;
        }
    }

    @Override
    public void close() {
        if (writer == null) return;
        try {
            writer.close();
            writer = null;
        } catch (IOException e) {
            fail(e);
        }
    }
}
