package org.firstinspires.ftc.teamcode.ds;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.json.JSONException;
import org.json.JSONObject;

import java.util.Map;

/**
 * Telemetry wrapper installed on each opmode so plain telemetry.addData /
 * addLine calls also reach the driver station web UI. Mirrors values into
 * the DriverStation snapshot and pushes one "telemetryDump" broadcast per
 * update() call instead of a message per addData.
 */
class DsTelemetry implements Telemetry {

    private final Telemetry inner;

    DsTelemetry(Telemetry inner) {
        this.inner = inner;
    }

    // ---------- mirrored calls ----------

    @Override
    public Item addData(String caption, String format, Object... args) {
        DriverStation.putTelemetry(caption, format(format, args));
        return inner.addData(caption, format, args);
    }

    @Override
    public Item addData(String caption, Object value) {
        DriverStation.putTelemetry(caption, String.valueOf(value));
        return inner.addData(caption, value);
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        DriverStation.putTelemetry(caption, String.valueOf(eval(valueProducer)));
        return inner.addData(caption, valueProducer);
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        DriverStation.putTelemetry(caption, format(format, eval(valueProducer)));
        return inner.addData(caption, format, valueProducer);
    }

    @Override
    public boolean removeItem(Item item) {
        DriverStation.removeTelemetry(item.getCaption());
        return inner.removeItem(item);
    }

    @Override
    public Line addLine() {
        return inner.addLine();
    }

    @Override
    public Line addLine(String lineCaption) {
        DriverStation.putTelemetry(lineCaption, "");
        return inner.addLine(lineCaption);
    }

    @Override
    public boolean removeLine(Line line) {
        return inner.removeLine(line);
    }

    @Override
    public void clear() {
        DriverStation.clearTelemetry();
        inner.clear();
    }

    @Override
    public void clearAll() {
        DriverStation.clearTelemetry();
        inner.clearAll();
    }

    @Override
    public boolean update() {
        broadcastDump();
        return inner.update();
    }

    // ---------- pass-through ----------

    @Override
    public Object addAction(Runnable action) {
        return inner.addAction(action);
    }

    @Override
    public boolean removeAction(Object token) {
        return inner.removeAction(token);
    }

    @Override
    public void speak(String text) {
        inner.speak(text);
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        inner.speak(text, languageCode, countryCode);
    }

    @Override
    public boolean isAutoClear() {
        return inner.isAutoClear();
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        inner.setAutoClear(autoClear);
    }

    @Override
    public int getMsTransmissionInterval() {
        return inner.getMsTransmissionInterval();
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        inner.setMsTransmissionInterval(msTransmissionInterval);
    }

    @Override
    public String getItemSeparator() {
        return inner.getItemSeparator();
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        inner.setItemSeparator(itemSeparator);
    }

    @Override
    public String getCaptionValueSeparator() {
        return inner.getCaptionValueSeparator();
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        inner.setCaptionValueSeparator(captionValueSeparator);
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        inner.setDisplayFormat(displayFormat);
    }

    @Override
    public Log log() {
        return inner.log();
    }

    // ---------- helpers ----------

    private static String format(String format, Object... args) {
        try {
            return String.format(format, args);
        } catch (RuntimeException e) {
            return format;
        }
    }

    private static <T> Object eval(Func<T> f) {
        try {
            return f.value();
        } catch (RuntimeException e) {
            return null;
        }
    }

    private void broadcastDump() {
        JSONObject json = new JSONObject();
        try {
            json.put("type", "telemetryDump");
            JSONObject entries = new JSONObject();
            for (Map.Entry<String, String> e : DriverStation.telemetrySnapshot().entrySet()) {
                entries.put(e.getKey(), e.getValue());
            }
            json.put("entries", entries);
        } catch (JSONException e) {
            return;
        }
        DriverStation.broadcast(json);
    }
}
