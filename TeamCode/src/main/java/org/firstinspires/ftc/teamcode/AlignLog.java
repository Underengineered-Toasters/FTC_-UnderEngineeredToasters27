package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * Per-run CSV logger for drive and AprilTag align tuning.
 *
 * Writes one row per OpMode loop to
 * /sdcard/FIRST/debug_logs/align_<milliseconds>.csv so every
 * teleop run (including align tests) is captured for later analysis.
 */
public class AlignLog {

    private PrintWriter writer;
    private final ElapsedTime elapsed = new ElapsedTime();
    private final StringBuilder row = new StringBuilder(256);

    /**
     * Opens a new CSV file and writes the header.
     * Call once at OpMode start; silently no-ops if the file
     * cannot be created so logging never breaks driving.
     */
    public void open() {

        File dir = new File(
                Environment.getExternalStorageDirectory(),
                "FIRST/debug_logs"
        );

        if (!dir.exists() && !dir.mkdirs()) {
            return;
        }

        File file = new File(dir, "align_" + System.currentTimeMillis() + ".csv");

        try {
            writer = new PrintWriter(new FileWriter(file));
            writer.println(
                    "time_ms,alliance,align_target,bearing_deg,turn_cmd,"
                            + "stick_ly,stick_rx,drive_cmd,"
                            + "left_cmd,right_cmd,left_vel_deg,right_vel_deg"
            );
            elapsed.reset();
        } catch (IOException e) {
            writer = null;
        }
    }

    /**
     * Writes one row. Any null align values are recorded as empty fields.
     */
    public void row(
            String alliance,
            String alignTarget,
            Double bearing,
            double turnCmd,
            double stickLy,
            double stickRx,
            double driveCmd,
            double leftCmd,
            double rightCmd,
            double leftVel,
            double rightVel
    ) {

        if (writer == null) {
            return;
        }

        row.setLength(0);
        row.append((long) (elapsed.milliseconds() + 0.5)).append(',');
        row.append(alliance).append(',');
        row.append(alignTarget == null ? "" : alignTarget).append(',');
        row.append(bearing == null ? "" : String.format("%.2f", bearing)).append(',');
        row.append(String.format("%.3f", turnCmd)).append(',');
        row.append(String.format("%.3f", stickLy)).append(',');
        row.append(String.format("%.3f", stickRx)).append(',');
        row.append(String.format("%.3f", driveCmd)).append(',');
        row.append(String.format("%.3f", leftCmd)).append(',');
        row.append(String.format("%.3f", rightCmd)).append(',');
        row.append(String.format("%.1f", leftVel)).append(',');
        row.append(String.format("%.1f", rightVel));

        writer.println(row);
    }

    /**
     * Flushes and closes the file. Call from OpMode cleanup.
     */
    public void close() {

        if (writer != null) {
            writer.flush();
            writer.close();
            writer = null;
        }
    }
}