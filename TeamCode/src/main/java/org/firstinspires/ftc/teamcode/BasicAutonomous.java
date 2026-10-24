package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Timed out-and-back route with a tuned turnaround duration.
 * Distances and turns are estimates and require calibration on the robot.
 */
@Autonomous(name = "Basic Out and Back Auto", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {

    private final Robot robot = new Robot();
    private String shootingStatus = "Not started";
    private String targetCluster = AprilTagVision.RED_SCORING;
    private final ElapsedTime runtime = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();

    // Hardcoded four-pollen sequence; tune feeder timings on the robot.
    public static final int POLLEN_COUNT = 4;
    public static final double AUTONOMOUS_SHOOTER_POWER = 1.0;
    public static final long SHOOTER_SPINUP_MS = 2000;
    public static final long FEEDER_PUSH_MS = 400;
    public static final long FEEDER_RETURN_MS = 600;

    // Calibration: Estimated driving speed in inches per second at 0.5 power
    public static final double ESTIMATED_INCHES_PER_SECOND = 20.0;
    // Starting estimate from the previous right turn; calibrate both directions.
    public static final double TURN_POWER = 0.565;
    public static final double ESTIMATED_SECONDS_PER_90_DEGREES = 0.60;

    @Override
    public void runOpMode() {

        // Initialize drive motors and vision
        robot.init(hardwareMap);

        telemetry.addLine("Basic Out and Back Auto");
        telemetry.addLine("Forward 7 in, right turn, forward 8 in, left turn, align then shoot");
        telemetry.addLine("Shoot four pollen after movement four (timed feeder cycles).");
        telemetry.update();

        while (opModeInInit()) {
            if (gamepad1.x) targetCluster = AprilTagVision.BLUE_SCORING;
            if (gamepad1.b) targetCluster = AprilTagVision.RED_SCORING;
            telemetry.addLine("X = Blue, B = Red. Camera must align before shooting.");
            telemetry.addData("Scoring target", targetCluster);
            telemetry.addData("Camera", robot.vision.getCameraState());
            telemetry.update();
            sleep(20);
        }
        waitForStart();
        if (!opModeIsActive()) {
            robot.close();
            return;
        }
        try {
            runtime.reset();

            driveInches(7, 0.5);
            turnRight(TURN_POWER, ESTIMATED_SECONDS_PER_90_DEGREES);
            driveInches(8, 0.5);
            if (!opModeIsActive()) {
                return;
            }
            sleep(2000);
            turnLeft(TURN_POWER, ESTIMATED_SECONDS_PER_90_DEGREES);
            if (!alignForShooting()) return;
            if (!shootPollen()) return;
            if (!opModeIsActive()) {
                return;
            }

            // Continue the team-written return sequence after successful shooting.
            driveInches(46, 0.5);
            turnRight(TURN_POWER, ESTIMATED_SECONDS_PER_90_DEGREES);
            turnLeft(TURN_POWER, 2 * ESTIMATED_SECONDS_PER_90_DEGREES);
            driveInches(-7, 0.3);
            stopRobot();

            if (opModeIsActive()) {
                telemetry.addLine("Out and back route completed");
                telemetry.addData("Total Time Elapsed", "%.2f sec", runtime.seconds());
                telemetry.addData("Shooting status", shootingStatus);
                telemetry.update();
            }
        } finally {
            stopRobot();
            robot.shooter.spinDown();
            robot.intakeSystem.restFeeder();
            robot.intakeSystem.stopIntake();
            telemetry.addData("Shooting status", shootingStatus);
            telemetry.update();
            robot.close();
        }
    }

    private boolean alignForShooting() {
        boolean aligned = AutonomousCameraAligner.align(this, robot, targetCluster);
        if (!aligned) {
            shootingStatus = "Alignment failed; shooting skipped";
            telemetry.addData("Shooting status", shootingStatus);
            telemetry.update();
        }
        return aligned;
    }

    private boolean shootPollen() {
        stopRobot();
        if (!opModeIsActive()) {
            return false;
        }
        if (Double.isNaN(robot.intakeSystem.getFeederPosition())) {
            shootingStatus = "Skipped: feeder servo is missing from hardware configuration";
            return false;
        }
        robot.intakeSystem.stopIntake();
        robot.intakeSystem.restFeeder();
        try {
            robot.shooter.setTargetPower(AUTONOMOUS_SHOOTER_POWER);
            if (!waitForShootingPhase("Spinning up shooter", SHOOTER_SPINUP_MS, false)) {
                if (isStopRequested()) shootingStatus = "Stopped during spin-up";
                return false;
            }
            for (int pollen = 1; pollen <= POLLEN_COUNT; pollen++) {
                if (!opModeIsActive()) {
                    shootingStatus = "Stopped before feeder cycle " + pollen;
                    return false;
                }
                // Reacquire a stable lock after spin-up and before each feed.
                if (!alignForShooting()) return false;
                robot.intakeSystem.pushBall();
                if (!waitForShootingPhase("Pushing pollen " + pollen + "/" + POLLEN_COUNT,
                        FEEDER_PUSH_MS, true)) {
                    if (isStopRequested()) shootingStatus = "Stopped during feeder push";
                    return false;
                }
                robot.intakeSystem.restFeeder();
                if (!waitForShootingPhase("Returning feeder " + pollen + "/" + POLLEN_COUNT,
                        FEEDER_RETURN_MS, false)) {
                    if (isStopRequested()) shootingStatus = "Stopped during feeder return";
                    return false;
                }
            }
            shootingStatus = POLLEN_COUNT + " timed feeder cycles complete";
            return true;
        } finally {
            robot.shooter.spinDown();
            robot.intakeSystem.restFeeder();
            robot.intakeSystem.stopIntake();
            stopRobot();
        }
    }

    private boolean waitForShootingPhase(String phase, long milliseconds, boolean feeding) {
        shootingStatus = phase;
        ElapsedTime phaseTimer = new ElapsedTime();
        while (opModeIsActive() && phaseTimer.milliseconds() < milliseconds) {
            if (robot.vision.hasFailed() || !robot.vision.isReady()) {
                shootingStatus = "Camera lost during shooting";
                return false;
            }
            if (feeding && !AutonomousCameraAligner.hasFreshAim(robot, targetCluster)) {
                shootingStatus = "Target lost or aim changed; feeding aborted";
                return false;
            }
            telemetry.addData("Shooting status", phase);
            telemetry.addData("Shooter power", "%.2f", robot.shooter.getCurrentPower());
            telemetry.update();
            sleep(20);
        }
        return opModeIsActive();
    }

    // =========================================================================
    // TIME & POWER DRIVING HELPER METHODS
    // =========================================================================

    /**
     * Drives forward or backward based on target inches converted to seconds.
     */
    private void driveInches(double targetInches, double power) {
        double seconds = Math.abs(targetInches) / (ESTIMATED_INCHES_PER_SECOND * (Math.abs(power) / 0.5));
        double direction = targetInches >= 0 ? 1.0 : -1.0;
        double motorPower = Math.abs(power) * direction;

        driveForSeconds(motorPower, 0.0, seconds);
    }

    /**
     * Turns Left for a specified duration in seconds.
     */
    private void turnLeft(double power, double seconds) {
        driveForSeconds(0.0, -Math.abs(power), seconds);
    }

    /**
     * Turns Right for a specified duration in seconds.
     */
    private void turnRight(double power, double seconds) {
        driveForSeconds(0.0, Math.abs(power), seconds);
    }

    /**
     * Drives the robot with given forward and turn powers for exact duration in seconds.
     */
    private void driveForSeconds(double forward, double turn, double seconds) {
        stepTimer.reset();
        while (opModeIsActive() && stepTimer.seconds() < seconds) {
            // This chassis moves backward for positive drive power.
            // Reverse both axes so route distances and turn names stay physical.
            robot.drive(-forward, -turn);
            telemetry.addData("Executing Step", "Time: %.2fs / %.2fs", stepTimer.seconds(), seconds);
            telemetry.update();
        }
        stopRobot();
    }

    /**
     * Stop all drive motors.
     */
    private void stopRobot() {
        robot.drive(0.0, 0.0);
    }
}
