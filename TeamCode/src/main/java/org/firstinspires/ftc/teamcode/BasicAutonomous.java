package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Forward 12 inches, turn left 95 degrees, forward 12, turn left 95, forward 15. */
@Autonomous(name = "Basic Auto - Three Legs", group = "Autonomous")
public class BasicAutonomous extends LinearOpMode {
    // Stock BIOBUZZ 6WD StarterBot: goBILDA StarterBotAuto.java and assembly
    // instructions 3200-2627-0003 (30T:30T external drive gears).
    // https://www.gobilda.com/ftc-starter-bot-resource-guide-2026-2027-season/
    private static final double WHEEL_DIAMETER_INCHES = 96.0 / 25.4;
    private static final double TRACK_WIDTH_INCHES = 402.0 / 25.4;
    private static final double ENCODER_TICKS_PER_REV = 537.7;
    // Motor output shaft revolutions per wheel revolution (excludes motor gearbox).
    private static final double DRIVE_GEAR_RATIO = 1.0;
    private static final double LEG_INCHES = 12.0;
    private static final double FINAL_LEG_INCHES = 15.0;
    private static final double TURN_DEGREES = 95.0;
    private static final double MATCH_LIMIT_SECONDS = 29.0;
    private DcMotor left;
    private DcMotor right;
    private final ElapsedTime matchTimer = new ElapsedTime();
    private final ElapsedTime stepTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        left = hardwareMap.get(DcMotor.class, "left");
        right = hardwareMap.get(DcMotor.class, "right");
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stopDrive();
        try {
            if (!calibrationValid()) {
                report("Motion blocked: invalid StarterBot drivetrain calibration");
                waitForStart();
                return;
            }
            telemetry.addLine("Forward 12 in -> counterclockwise 95 -> forward 12 in");
            telemetry.addLine("-> counterclockwise 95 -> forward 15 in -> stop");
            telemetry.update();
            waitForStart();
            if (isStopRequested()) return;
            matchTimer.reset();
            if (!move("Step 1: forward 12 inches", LEG_INCHES, LEG_INCHES, 0.25, 5.0)) return;
            if (!turnCounterclockwise("Step 2: counterclockwise 95 degrees", TURN_DEGREES)) return;
            if (!move("Step 3: forward 12 inches", LEG_INCHES, LEG_INCHES, 0.25, 5.0)) return;
            if (!turnCounterclockwise("Step 4: counterclockwise 95 degrees", TURN_DEGREES)) return;
            if (!move("Step 5: forward 15 inches", FINAL_LEG_INCHES, FINAL_LEG_INCHES, 0.25, 5.0)) return;
            report("Sequence complete; stopped");
        } finally {
            stopDrive();
            left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private boolean calibrationValid() {
        return positive(WHEEL_DIAMETER_INCHES) && positive(TRACK_WIDTH_INCHES)
                && positive(DRIVE_GEAR_RATIO) && positive(ENCODER_TICKS_PER_REV);
    }

    private boolean positive(double value) {
        return value > 0.0 && !Double.isInfinite(value) && !Double.isNaN(value);
    }

    private boolean active() {
        return opModeIsActive() && matchTimer.seconds() < MATCH_LIMIT_SECONDS;
    }

    private double ticksPerInch() {
        return ENCODER_TICKS_PER_REV * DRIVE_GEAR_RATIO
                / (Math.PI * WHEEL_DIAMETER_INCHES);
    }

    private boolean turnCounterclockwise(String label, double degrees) {
        double arc = Math.PI * TRACK_WIDTH_INCHES * degrees / 360.0;
        return move(label, -arc, arc, 0.20, 5.0);
    }

    private boolean move(String label, double leftInches, double rightInches,
                         double power, double timeout) {
        if (!active()) return false;
        int leftTarget = left.getCurrentPosition() + (int) Math.round(leftInches * ticksPerInch());
        int rightTarget = right.getCurrentPosition() + (int) Math.round(rightInches * ticksPerInch());
        left.setTargetPosition(leftTarget);
        right.setTargetPosition(rightTarget);
        left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stepTimer.reset();
        boolean reached = false;
        while (active() && stepTimer.seconds() < timeout) {
            boolean leftReached = Math.abs(leftTarget - left.getCurrentPosition()) <= 0.25 * ticksPerInch();
            boolean rightReached = Math.abs(rightTarget - right.getCurrentPosition()) <= 0.25 * ticksPerInch();
            left.setPower(leftReached ? 0.0 : power);
            right.setPower(rightReached ? 0.0 : power);
            if (leftReached && rightReached) {
                reached = true;
                break;
            }
            telemetry.addData("Autonomous", label);
            telemetry.addData("Left encoder", "%d / %d", left.getCurrentPosition(), leftTarget);
            telemetry.addData("Right encoder", "%d / %d", right.getCurrentPosition(), rightTarget);
            telemetry.update();
            idle();
        }
        stopDrive();
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (!reached) report("Aborted: " + label + " timeout/STOP");
        return reached && active();
    }

    private void stopDrive() {
        left.setPower(0.0);
        right.setPower(0.0);
    }

    private void report(String message) {
        telemetry.addData("Autonomous", message);
        telemetry.addData("Elapsed", "%.1f s", matchTimer.seconds());
        telemetry.update();
    }
}
