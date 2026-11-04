package org.firstinspires.ftc.teamcode.ds;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.Map;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Covers the DriverStation facade state that does not need a subscriber, so nothing
 * here reaches org.json or android Base64. Each test starts from a detached facade.
 */
public class DriverStationTest {

    @Before public void reset() {
        DriverStation.detach();
    }

    @After public void cleanup() {
        DriverStation.detach();
    }

    @Test public void pinnedTelemetrySurvivesFrameClears() {
        DriverStation.telemetry("Mode", "auto");
        DriverStation.putTelemetry("Heading", "90");
        DriverStation.clearFrameTelemetry();

        Map<String, String> snapshot = DriverStation.telemetrySnapshot();
        assertEquals("auto", snapshot.get("Mode"));
        assertFalse(snapshot.containsKey("Heading"));
    }

    @Test public void frameValueOverridesPinnedValueForTheSameKey() {
        DriverStation.telemetry("Pose", "pinned");
        DriverStation.putTelemetry("Pose", "frame");
        assertEquals("frame", DriverStation.telemetrySnapshot().get("Pose"));

        DriverStation.clearFrameTelemetry();
        assertEquals("pinned", DriverStation.telemetrySnapshot().get("Pose"));
    }

    @Test public void nullKeysAreIgnoredAndNullFrameValuesBecomeEmpty() {
        DriverStation.telemetry(null, "x");
        DriverStation.putTelemetry(null, "x");
        DriverStation.removeTelemetry(null);
        assertTrue(DriverStation.telemetrySnapshot().isEmpty());

        DriverStation.putTelemetry("Blank", null);
        assertEquals("", DriverStation.telemetrySnapshot().get("Blank"));
    }

    @Test public void nullValuesAreStoredAsText() {
        DriverStation.telemetry("Count", null);
        assertEquals("null", DriverStation.telemetrySnapshot().get("Count"));
    }

    @Test public void removeTelemetryDropsOnlyTheFrameRow() {
        DriverStation.telemetry("Keep", "1");
        DriverStation.putTelemetry("Drop", "2");
        DriverStation.removeTelemetry("Drop");

        Map<String, String> snapshot = DriverStation.telemetrySnapshot();
        assertEquals("1", snapshot.get("Keep"));
        assertFalse(snapshot.containsKey("Drop"));
    }

    @Test public void pinnedTelemetryIsBoundedAndEvictsOldestFirst() {
        int total = DriverStation.TELEMETRY_CAPACITY + 44;
        for (int i = 0; i < total; i++) {
            DriverStation.telemetry("k" + i, String.valueOf(i));
        }
        Map<String, String> snapshot = DriverStation.telemetrySnapshot();
        assertEquals(DriverStation.TELEMETRY_CAPACITY, snapshot.size());
        assertNull(snapshot.get("k0"));
        assertNull(snapshot.get("k43"));
        assertEquals(String.valueOf(total - 1), snapshot.get("k" + (total - 1)));
    }

    @Test public void detachForgetsPinnedAndFrameTelemetry() {
        DriverStation.telemetry("Pinned", "a");
        DriverStation.putTelemetry("Frame", "b");
        DriverStation.detach();
        assertTrue(DriverStation.telemetrySnapshot().isEmpty());
    }

    @Test public void detachForgetsTheCameraAndHardware() {
        DriverStation.setCameraSource(null);
        DriverStation.detach();
        assertNull(DriverStation.getCameraSource());
        assertNull(DriverStation.getHardwareMap());
    }

    @Test public void logLinesAreRingBuffered() {
        for (int i = 0; i < 205; i++) {
            DriverStation.log("line " + i);
        }
        java.util.List<String> lines = DriverStation.logSnapshot();
        assertEquals(200, lines.size());
        assertEquals("line 5", lines.get(0));
        assertEquals("line 204", lines.get(lines.size() - 1));
        DriverStation.log(null);
        assertEquals(200, DriverStation.logSnapshot().size());
    }

    @Test public void subscriberCountNeverGoesNegative() {
        assertFalse(DriverStation.hasSubscribers());
        DriverStation.subscriberLeft();
        assertFalse(DriverStation.hasSubscribers());

        DriverStation.subscriberJoined();
        DriverStation.subscriberJoined();
        assertTrue(DriverStation.hasSubscribers());
        DriverStation.subscriberLeft();
        assertTrue(DriverStation.hasSubscribers());
        DriverStation.subscriberLeft();
        assertFalse(DriverStation.hasSubscribers());
    }

    @Test public void publishWithoutSubscribersDoesNothing() {
        // With no subscriber, publish must return before touching JSON or the outbox.
        assertFalse(DriverStation.hasSubscribers());
        DriverStation.publishTelemetryDump();
        DriverStation.publish("log", null);
    }

    @Test public void outboxDropsOldestWhenFull() {
        DriverStation.BoundedOutbox<String> outbox = new DriverStation.BoundedOutbox<>(3);
        outbox.offer("a");
        outbox.offer("b");
        outbox.offer("c");
        outbox.offer("d");
        assertEquals(3, outbox.size());
        assertEquals(1L, outbox.dropped());
        assertEquals("b", take(outbox));
        assertEquals("c", take(outbox));
        assertEquals("d", take(outbox));
        assertEquals(0, outbox.size());
    }

    @Test public void outboxRejectsNullsAndBadCapacity() {
        try {
            new DriverStation.BoundedOutbox<String>(0);
            fail("capacity must be positive");
        } catch (IllegalArgumentException expected) {
            // expected
        }
        DriverStation.BoundedOutbox<String> outbox = new DriverStation.BoundedOutbox<>(1);
        try {
            outbox.offer(null);
            fail("null items must be rejected");
        } catch (NullPointerException expected) {
            // expected
        }
    }

    @Test public void takeOnEmptyOutboxReturnsWhenInterrupted() {
        DriverStation.BoundedOutbox<String> outbox = new DriverStation.BoundedOutbox<>(1);
        Thread.currentThread().interrupt();
        try {
            outbox.take();
            fail("take on an empty outbox must observe the interrupt");
        } catch (InterruptedException expected) {
            // The interrupt flag is consumed by the exception; the sender loop restores it.
        }
        assertFalse(Thread.currentThread().isInterrupted());
    }

    private static String take(DriverStation.BoundedOutbox<String> outbox) {
        try {
            return outbox.take();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new AssertionError(e);
        }
    }
}
