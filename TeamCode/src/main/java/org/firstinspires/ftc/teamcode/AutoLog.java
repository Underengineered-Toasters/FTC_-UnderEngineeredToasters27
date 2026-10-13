package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class AutoLog {

    private PrintWriter writer;
    private final ElapsedTime timer = new ElapsedTime();
    private final StringBuilder sb = new StringBuilder(256);

    public void open(String opModeName) {
        File dir = new File(
                Environment.getExternalStorageDirectory(),
                "FIRST/debug_logs"
        );
        if (!dir.exists() && !dir.mkdirs()) {
            return;
        }

        File file = new File(dir, opModeName + "_" + System.currentTimeMillis() + ".csv");
        try {
            writer = new PrintWriter(new FileWriter(file));
            writer.println(
                    "time_ms,step,alliance,left_pwr,right_pwr,intake_pwr,shooter_pwr,"
                            + "align_target,target_found,aligned,bearing_deg,range_in,turn_cmd"
            );
            timer.reset();
        } catch (IOException e) {
            writer = null;
        }
    }

    public void log(
            String step,
            String alliance,
            double leftPwr,
            double rightPwr,
            double intakePwr,
            double shooterPwr,
            String target,
            boolean found,
            boolean aligned,
            Double bearing,
            Double range,
            double turnCmd
    ) {
        if (writer == null) {
            return;
        }

        sb.setLength(0);
        sb.append((long) (timer.milliseconds() + 0.5)).append(',');
        sb.append(step).append(',');
        sb.append(alliance).append(',');
        sb.append(String.format("%.3f", leftPwr)).append(',');
        sb.append(String.format("%.3f", rightPwr)).append(',');
        sb.append(String.format("%.3f", intakePwr)).append(',');
        sb.append(String.format("%.3f", shooterPwr)).append(',');
        sb.append(target == null ? "none" : target).append(',');
        sb.append(found).append(',');
        sb.append(aligned).append(',');
        sb.append(bearing != null ? String.format("%.2f", bearing) : "").append(',');
        sb.append(range != null ? String.format("%.2f", range) : "").append(',');
        sb.append(String.format("%.3f", turnCmd));
        writer.println(sb);
    }

    public void close() {
        if (writer != null) {
            writer.flush();
            writer.close();
            writer = null;
        }
    }
}
