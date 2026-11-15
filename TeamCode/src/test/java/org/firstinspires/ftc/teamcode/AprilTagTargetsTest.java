package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.AprilTagTargets;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.junit.Test;
import static org.junit.Assert.*;

public class AprilTagTargetsTest {
    @Test public void everyBioBuzzMemberMatchesTheSdkCluster() {
        AprilTagLibrary library = AprilTagGameDatabase.getCurrentGameTagLibrary();
        for (int id = 30; id <= 45; id++) {
            assertNull(library.lookupTag(id));
            assertEquals(library.lookupCluster(id).name, AprilTagTargets.bioBuzzClusterForId(id));
            AprilTagClusterDetection detection = new AprilTagClusterDetection(25,
                    library.lookupCluster(id), DistanceUnit.INCH, null, null, null, 100L);
            assertTrue(AprilTagTargets.matchesId(detection, id));
            assertEquals(library.lookupCluster(id).name, AprilTagTargets.nameOf(detection));
            assertNull(AprilTagTargets.centerOf(detection));
            int otherAlliance = id < 38 ? 38 : 30;
            assertFalse(AprilTagTargets.matchesId(detection, otherAlliance));
        }
    }

    @Test public void preservesIntendedAudienceAndScoringNames() {
        assertEquals("RED AUDIENCE", AprilTagTargets.bioBuzzClusterForId(34));
        assertEquals("RED SCORING", AprilTagTargets.bioBuzzClusterForId(30));
        assertEquals("BLUE AUDIENCE", AprilTagTargets.bioBuzzClusterForId(38));
        assertEquals("BLUE SCORING", AprilTagTargets.bioBuzzClusterForId(42));
    }

    @Test public void unknownInputsCannotSelectATarget() {
        assertNull(AprilTagTargets.bioBuzzClusterForId(29));
        assertNull(AprilTagTargets.bioBuzzClusterForId(46));
        assertFalse(AprilTagTargets.matchesId(null, 34));
        AprilTagClusterDetection unknown = new AprilTagClusterDetection(0,
                null, DistanceUnit.INCH, null, null, null, 100L);
        assertFalse(AprilTagTargets.matchesId(unknown, 34));
        assertEquals("Unknown cluster", AprilTagTargets.identityOf(unknown));
    }
}
