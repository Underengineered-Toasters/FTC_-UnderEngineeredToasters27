package org.firstinspires.ftc.teamcode.ds;

import org.junit.After;
import org.junit.Test;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.ds.DriverStationServer.FrameSlot;
import org.firstinspires.ftc.teamcode.ds.DriverStationServer.MjpegStream;

public class DriverStationServerTest {

    @After public void clearInterrupt() {
        Thread.interrupted();
    }

    // ---------- path sanitising ----------

    @Test public void servesOnlyPlainFileNamesBelowDs() {
        assertTrue(DriverStationServer.isSafeRequestPath("/ds"));
        assertTrue(DriverStationServer.isSafeRequestPath("/ds/"));
        assertTrue(DriverStationServer.isSafeRequestPath("/ds/index.html"));
        assertTrue(DriverStationServer.isSafeRequestPath("/ds/app.js"));
        assertTrue(DriverStationServer.isSafeRequestPath("/ds/a-b_c.v2.css"));
    }

    @Test public void rejectsTraversalAndEncodedOrOddSegments() {
        assertFalse(DriverStationServer.isSafeRequestPath("/ds/../secret"));
        assertFalse(DriverStationServer.isSafeRequestPath("/ds/a/../../secret"));
        assertFalse(DriverStationServer.isSafeRequestPath("/ds/./app.js"));
        assertFalse(DriverStationServer.isSafeRequestPath("/ds/%2e%2e/secret"));
        assertFalse(DriverStationServer.isSafeRequestPath("/ds/a%20b"));
        assertFalse(DriverStationServer.isSafeRequestPath("/ds/a b"));
        assertFalse(DriverStationServer.isSafeRequestPath("/ds/a\\b"));
    }

    @Test public void rejectsPathsOutsideDs() {
        assertFalse(DriverStationServer.isSafeRequestPath(null));
        assertFalse(DriverStationServer.isSafeRequestPath(""));
        assertFalse(DriverStationServer.isSafeRequestPath("/"));
        assertFalse(DriverStationServer.isSafeRequestPath("/dsx"));
        assertFalse(DriverStationServer.isSafeRequestPath("/other/index.html"));
    }

    // ---------- content types ----------

    @Test public void textTypesCarryUtf8Charset() {
        assertEquals("text/html; charset=utf-8", DriverStationServer.contentTypeFor("index.html"));
        assertEquals("text/css; charset=utf-8", DriverStationServer.contentTypeFor("app.css"));
        assertEquals("application/javascript; charset=utf-8", DriverStationServer.contentTypeFor("APP.JS"));
    }

    @Test public void binaryAndStructuredTypesHaveNoCharset() {
        assertEquals("application/json", DriverStationServer.contentTypeFor("data.json"));
        assertEquals("image/png", DriverStationServer.contentTypeFor("logo.PNG"));
        assertEquals("image/jpeg", DriverStationServer.contentTypeFor("photo.jpeg"));
    }

    @Test public void unknownOrMissingExtensionsAreOctetStream() {
        assertEquals("application/octet-stream", DriverStationServer.contentTypeFor("x.unknown"));
        assertEquals("application/octet-stream", DriverStationServer.contentTypeFor("README"));
        assertEquals("application/octet-stream", DriverStationServer.contentTypeFor("dir.d/file"));
    }

    // ---------- MJPEG framing ----------

    @Test public void mjpegMimeNamesTheBoundaryThePartsUse() {
        assertEquals("multipart/x-mixed-replace; boundary=frame", DriverStationServer.MJPEG_MIME);
        assertEquals("frame", DriverStationServer.MJPEG_BOUNDARY);
    }

    @Test public void mjpegPartFramesTheJpegWithItsLength() {
        byte[] jpeg = {1, 2, 3};
        byte[] expected = concat(
                "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: 3\r\n\r\n"
                        .getBytes(StandardCharsets.US_ASCII),
                jpeg,
                "\r\n".getBytes(StandardCharsets.US_ASCII));
        assertArrayEquals(expected, DriverStationServer.mjpegPart(jpeg));
    }

    // ---------- frame hand-off ----------

    @Test public void slotReturnsTheDeliveredFrame() {
        FrameSlot<String> slot = new FrameSlot<>();
        slot.deliver("frame");
        assertEquals("frame", slotAwait(slot, 1000));
    }

    @Test public void slotTimesOutWithNullAndDropsLateFrames() {
        FrameSlot<String> slot = new FrameSlot<>();
        assertNull(slotAwait(slot, 5));
        slot.deliver("late");
        assertNull(slotAwait(slot, 5));
    }

    @Test public void slotKeepsTheFirstFrame() {
        FrameSlot<String> slot = new FrameSlot<>();
        slot.deliver("first");
        slot.deliver("second");
        assertEquals("first", slotAwait(slot, 1000));
    }

    @Test public void slotIgnoresNullDeliveries() {
        FrameSlot<String> slot = new FrameSlot<>();
        slot.deliver(null);
        assertNull(slotAwait(slot, 5));
    }

    // ---------- MJPEG body ----------

    @Test public void streamWritesEachFrameAsAPartThenEnds() throws IOException {
        byte[] a = {10, 11};
        byte[] b = {20, 21, 22};
        FakeSource source = new FakeSource(a, b);
        MjpegStream stream = new MjpegStream(source, 0);
        byte[] expected = concat(DriverStationServer.mjpegPart(a), DriverStationServer.mjpegPart(b));
        assertArrayEquals(expected, readAll(stream));
        assertEquals(3, source.calls);
    }

    @Test public void emptyFramesAreSkipped() throws IOException {
        byte[] a = {7};
        FakeSource source = new FakeSource(new byte[0], new byte[0], a);
        MjpegStream stream = new MjpegStream(source, 0);
        assertArrayEquals(DriverStationServer.mjpegPart(a), readAll(stream));
    }

    @Test public void oneByteReadsMatchBulkReads() throws IOException {
        byte[] a = {1, 2, 3, 4, 5};
        MjpegStream stream = new MjpegStream(new FakeSource(a), 0);
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        int b;
        while ((b = stream.read()) != -1) out.write(b);
        assertArrayEquals(DriverStationServer.mjpegPart(a), out.toByteArray());
    }

    @Test public void runtimeFailureEndsTheStreamQuietly() throws IOException {
        MjpegStream stream = new MjpegStream(() -> {
            throw new IllegalStateException("camera gone");
        }, 0);
        assertEquals(-1, stream.read(new byte[8], 0, 8));
    }

    @Test public void interruptEndsTheStreamAndRestoresTheFlag() throws IOException {
        MjpegStream stream = new MjpegStream(() -> {
            throw new InterruptedException();
        }, 0);
        assertEquals(-1, stream.read(new byte[8], 0, 8));
        assertTrue(Thread.currentThread().isInterrupted());
    }

    @Test public void closeEndsTheStreamWithoutPullingMoreFrames() throws IOException {
        FakeSource source = new FakeSource(new byte[] {1}, new byte[] {2});
        MjpegStream stream = new MjpegStream(source, 0);
        stream.close();
        assertEquals(-1, stream.read(new byte[8], 0, 8));
        assertEquals(0, source.calls);
    }

    @Test public void zeroLengthReadsReturnZeroWithoutPulling() throws IOException {
        FakeSource source = new FakeSource(new byte[] {1});
        MjpegStream stream = new MjpegStream(source, 0);
        assertEquals(0, stream.read(new byte[8], 0, 0));
        assertEquals(0, source.calls);
    }

    // ---------- helpers ----------

    private static Object slotAwait(FrameSlot<String> slot, long timeoutMs) {
        try {
            return slot.await(timeoutMs);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            throw new AssertionError(e);
        }
    }

    private static byte[] readAll(InputStream in) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        byte[] buf = new byte[4];
        int n;
        while ((n = in.read(buf, 0, buf.length)) != -1) out.write(buf, 0, n);
        return out.toByteArray();
    }

    private static byte[] concat(byte[]... parts) {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        for (byte[] p : parts) out.write(p, 0, p.length);
        return out.toByteArray();
    }

    /** Serves the given frames in order, then ends the stream with null. */
    private static final class FakeSource implements MjpegStream.FrameSource {
        private final byte[][] frames;
        int calls;

        FakeSource(byte[]... frames) {
            this.frames = frames;
        }

        @Override public byte[] nextJpeg() {
            calls++;
            return calls <= frames.length ? frames[calls - 1] : null;
        }
    }
}
