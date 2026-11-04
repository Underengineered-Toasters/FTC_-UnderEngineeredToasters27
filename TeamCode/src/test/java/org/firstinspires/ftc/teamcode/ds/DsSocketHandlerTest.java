package org.firstinspires.ftc.teamcode.ds;

import org.junit.Test;

import java.nio.charset.StandardCharsets;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.firstinspires.ftc.teamcode.ds.DsSocketHandler.Action;
import org.firstinspires.ftc.teamcode.ds.DsSocketHandler.Verdict;

public class DsSocketHandlerTest {

    private static String repeat(char c, int n) {
        StringBuilder sb = new StringBuilder(n);
        for (int i = 0; i < n; i++) sb.append(c);
        return sb.toString();
    }

    @Test public void acceptsOrdinaryBase64Payloads() {
        assertNull(DsSocketHandler.payloadProblem(null));
        assertNull(DsSocketHandler.payloadProblem(""));
        assertNull(DsSocketHandler.payloadProblem("SGVsbG8="));
        assertNull(DsSocketHandler.payloadProblem("SGVsbG8"));
        assertNull(DsSocketHandler.payloadProblem(repeat('A', DsSocketHandler.MAX_PAYLOAD_CHARS)));
    }

    @Test public void rejectsOversizedPayloads() {
        assertEquals("payload is too large",
                DsSocketHandler.payloadProblem(repeat('A', DsSocketHandler.MAX_PAYLOAD_CHARS + 1)));
    }

    @Test public void rejectsPayloadsThatAreNotBase64() {
        assertEquals("payload is not base64", DsSocketHandler.payloadProblem("not base64!"));
        assertEquals("payload is not base64", DsSocketHandler.payloadProblem("AB=C"));
        assertEquals("payload is not base64", DsSocketHandler.payloadProblem("AB==A"));
        assertEquals("payload is not base64", DsSocketHandler.payloadProblem("A==="));
    }

    @Test public void opModeNamesMustBeUsable() {
        assertEquals("missing opmode name", DsSocketHandler.opModeNameProblem(""));
        assertEquals("missing opmode name", DsSocketHandler.opModeNameProblem(null));
        assertEquals("opmode name is too long",
                DsSocketHandler.opModeNameProblem(repeat('x', DsSocketHandler.MAX_NAME_CHARS + 1)));
        assertNull(DsSocketHandler.opModeNameProblem(repeat('x', DsSocketHandler.MAX_NAME_CHARS)));
        assertEquals("opmode name has control characters", DsSocketHandler.opModeNameProblem("Red\nAuto"));
        assertEquals("opmode name has control characters", DsSocketHandler.opModeNameProblem("Red\u007fAuto"));
        assertNull(DsSocketHandler.opModeNameProblem("Red Scoring (Test) \u00e9"));
    }

    @Test public void utf8IsDecodedStrictly() {
        assertEquals("Red Auto", DsSocketHandler.utf8Strict("Red Auto".getBytes(StandardCharsets.UTF_8)));
        assertEquals("\u00e9", DsSocketHandler.utf8Strict(new byte[] {(byte) 0xC3, (byte) 0xA9}));
        assertNull(DsSocketHandler.utf8Strict(new byte[] {(byte) 0xC3, (byte) 0x28}));
    }

    @Test public void nameFromPayloadFailsBeforeTouchingTheDecoderForBadInput() {
        assertFailsWith("payload is not base64", "!!");
        assertFailsWith("payload is too large", repeat('A', DsSocketHandler.MAX_PAYLOAD_CHARS + 1));
        // An absent payload decodes to empty bytes, which is an empty name.
        assertFailsWith("missing opmode name", null);
    }

    private static void assertFailsWith(String reason, String encoded) {
        try {
            DsSocketHandler.nameFromPayload(encoded);
            fail("expected IllegalArgumentException: " + reason);
        } catch (IllegalArgumentException e) {
            assertEquals(reason, e.getMessage());
        }
    }

    @Test public void initIsIdempotentForTheInitialisedOpMode() {
        assertEquals(Action.IGNORE, DsSocketHandler.checkInit("Auto", "Auto", "INIT").action);
    }

    @Test public void initIsRefusedForTheRunningOpMode() {
        Verdict verdict = DsSocketHandler.checkInit("Auto", "Auto", "LOOPING");
        assertEquals(Action.REJECT, verdict.action);
        assertTrue(verdict.message.contains("stop it"));
    }

    @Test public void initSwitchesToAnotherOpModeEvenWhileRunning() {
        assertEquals(Action.SEND, DsSocketHandler.checkInit("Teleop", "Auto", "LOOPING").action);
        assertEquals(Action.SEND, DsSocketHandler.checkInit("Teleop", "Auto", "INIT").action);
    }

    @Test public void initSendsWhenNothingIsActiveOrStoppedMidSwap() {
        assertEquals(Action.SEND, DsSocketHandler.checkInit("Auto", null, null).action);
        assertEquals(Action.SEND, DsSocketHandler.checkInit("Auto", "Auto", null).action);
    }

    @Test public void startNeedsAnInitialisedOpMode() {
        assertEquals(Action.SEND, DsSocketHandler.checkStart("Auto", "INIT").action);
        assertEquals(Action.REJECT, DsSocketHandler.checkStart(null, null).action);
        assertEquals(Action.REJECT, DsSocketHandler.checkStart(null, "INIT").action);
        assertEquals(Action.REJECT, DsSocketHandler.checkStart("Auto", null).action);
    }

    @Test public void startIsIdempotentWhileLooping() {
        assertEquals(Action.IGNORE, DsSocketHandler.checkStart("Auto", "LOOPING").action);
    }

    @Test public void stopIsIdempotentWhenNothingIsActive() {
        assertEquals(Action.IGNORE, DsSocketHandler.checkStop(null).action);
        assertEquals(Action.SEND, DsSocketHandler.checkStop("Auto").action);
    }
}
