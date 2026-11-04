package org.firstinspires.ftc.teamcode.ds;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Telemetry wrapper installed on each opmode so plain telemetry.addData /
 * addLine calls also reach the driver station web UI. The web UI mirrors the
 * SDK frame: rows set since the last update(), cleared whenever the SDK
 * auto-clears, so the web view cannot accumulate stale lines. Value producers
 * from addData(caption, Func) are re-evaluated at each update(), as the SDK does.
 * One "telemetryDump" is sent per update() while a browser is subscribed.
 */
class DsTelemetry implements Telemetry {

    private final Telemetry inner;
    /** Captions whose value comes from a Func, re-evaluated on every update(). */
    private final Map<String, Producer> producers = new LinkedHashMap<>();

    DsTelemetry(Telemetry inner) {
        this.inner = inner;
    }

    /** A live value: a Func plus the optional format it is rendered with. */
    private static final class Producer {
        final String format;
        final Func<?> value;

        Producer(String format, Func<?> value) {
            this.format = format;
            this.value = value;
        }
    }

    // ---------- mirrored calls ----------

    @Override
    public Item addData(String caption, String format, Object... args) {
        producers.remove(caption);
        DriverStation.putTelemetry(caption, format(format, args));
        return inner.addData(caption, format, args);
    }

    @Override
    public Item addData(String caption, Object value) {
        producers.remove(caption);
        DriverStation.putTelemetry(caption, String.valueOf(value));
        return inner.addData(caption, value);
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        producers.put(caption, new Producer(null, valueProducer));
        DriverStation.putTelemetry(caption, String.valueOf(eval(valueProducer)));
        return inner.addData(caption, valueProducer);
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        producers.put(caption, new Producer(format, valueProducer));
        DriverStation.putTelemetry(caption, format(format, eval(valueProducer)));
        return inner.addData(caption, format, valueProducer);
    }

    @Override
    public boolean removeItem(Item item) {
        producers.remove(item.getCaption());
        DriverStation.removeTelemetry(item.getCaption());
        return inner.removeItem(item);
    }

    @Override
    public Line addLine() {
        // Lines without a caption have no key to mirror; the web view shows the line's items only if added via addData.
        return inner.addLine();
    }

    @Override
    public Line addLine(String lineCaption) {
        producers.remove(lineCaption);
        DriverStation.putTelemetry(lineCaption, "");
        return inner.addLine(lineCaption);
    }

    @Override
    public boolean removeLine(Line line) {
        return inner.removeLine(line);
    }

    @Override
    public void clear() {
        producers.clear();
        DriverStation.clearFrameTelemetry();
        inner.clear();
    }

    @Override
    public void clearAll() {
        producers.clear();
        DriverStation.clearFrameTelemetry();
        inner.clearAll();
    }

    /**
     * Evaluates live values, publishes the frame, then forgets it when the SDK
     * auto-clears, so the next frame starts empty on both sides.
     */
    @Override
    public boolean update() {
        for (Map.Entry<String, Producer> e : producers.entrySet()) {
            Producer p = e.getValue();
            Object v = eval(p.value);
            DriverStation.putTelemetry(e.getKey(), p.format == null ? String.valueOf(v) : format(p.format, v));
        }
        DriverStation.publishTelemetryDump();
        boolean sent = inner.update();
        if (inner.isAutoClear()) {
            producers.clear();
            DriverStation.clearFrameTelemetry();
        }
        return sent;
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
}
