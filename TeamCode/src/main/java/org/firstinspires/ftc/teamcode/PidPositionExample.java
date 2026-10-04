package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ds.DriverStation;

/**
 * Template: closed-loop position move using {@link PIDController}.
 *
 * <p>This is the calling pattern for anything that drives a mechanism to an encoder position
 * (arm, lift, slide). It is deliberately an example and not wired into {@link Robot}: the robot
 * configuration has no motor named {@code armMotor} today, so the OpMode reports that and exits
 * instead of crashing at INIT.
 *
 * <p>What it does that a bare PID loop does not:
 * <ul>
 *   <li>sets a tolerance, without which {@link PIDController#atSetpoint(double, double)} always
 *       returns false;</li>
 *   <li>stops when the move finishes, times out, or the mechanism stops responding to full
 *       power (unplugged encoder, binding arm) instead of pushing into a stall forever;</li>
 *   <li>gives the driver instant manual control plus a B-button stop;</li>
 *   <li>publishes target/actual/error/output to both the Driver Station screen and the web UI, so
 *       the gains can be tuned without redeploying.</li>
 * </ul>
 *
 * <p>To use it on real hardware: add the motor (default name {@code armMotor}) to the Control Hub
 * configuration, adjust the gains and {@link #TARGET_TICKS}, then tune on the robot.
 */
@TeleOp(name = "PID Position Example", group = "Examples")
public class PidPositionExample extends LinearOpMode {

    /** Must match the device name in the robot configuration. */
    private static final String MOTOR_NAME = "armMotor";

    private static final double TARGET_TICKS = 1000.0;
    private static final double TOLERANCE_TICKS = 10.0;

    /** Abort a move that has not settled within this many seconds. */
    private static final double MOVE_TIMEOUT_SECONDS = 5.0;

    /** Commanded power at or above this while the encoder does not move means something is wrong. */
    private static final double STALL_POWER_THRESHOLD = 0.9;
    private static final double STALL_TIMEOUT_SECONDS = 0.5;
    private static final double STALL_TICK_EPSILON = 1.0;

    private static final double MANUAL_DEADBAND = 0.05;

    private DcMotorEx motor;

    private final PIDController pid = new PIDController(0.005, 0.0001, 0.0002);

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime moveTimer = new ElapsedTime();
    private final ElapsedTime stallTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        DriverStation.attach(this);

        // tryGet instead of get: a missing device name must not crash this OpMode at INIT.
        motor = hardwareMap.tryGet(DcMotorEx.class, MOTOR_NAME);
        if (motor == null) {
            telemetry.addLine("Motor \"" + MOTOR_NAME + "\" is not in the robot configuration.");
            telemetry.addLine("Add it on the Control Hub, then run this OpMode again.");
            telemetry.update();
            DriverStation.log("PID position example: motor \"" + MOTOR_NAME + "\" is missing");
            waitForStart();
            return;
        }

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // The PID owns the power output, so the hub must not add its own velocity loop on top.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // BRAKE so the mechanism holds position (or holds against gravity) at zero power.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Required: atSetpoint() deliberately returns false until a tolerance is configured.
        pid.setTolerance(TOLERANCE_TICKS);

        telemetry.addLine("PID position example: ready");
        telemetry.addData("Target (ticks)", TARGET_TICKS);
        telemetry.addData("Left stick Y", "manual override");
        telemetry.addData("B", "stop");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        pid.reset();
        loopTimer.reset();
        moveTimer.reset();
        stallTimer.reset();

        boolean pidInControl = true;
        double lastPosition = motor.getCurrentPosition();
        String stopReason = "opmode stopped";

        while (opModeIsActive()) {

            double dt = loopTimer.seconds();
            loopTimer.reset();

            if (gamepad1.b) {
                stopReason = "stopped by driver (B)";
                break;
            }

            double position = motor.getCurrentPosition();
            double manual = -gamepad1.left_stick_y;
            double power;

            if (Math.abs(manual) > MANUAL_DEADBAND) {
                // Driver takes over immediately; the PID state is dropped, not carried over.
                power = manual;
                pidInControl = false;
                pid.reset();
                stallTimer.reset();
            } else {
                if (!pidInControl) {
                    // Handing control back to the PID: restart the move bookkeeping.
                    pidInControl = true;
                    pid.reset();
                    moveTimer.reset();
                    stallTimer.reset();
                }
                power = pid.calculate(TARGET_TICKS, position, dt);
            }

            motor.setPower(power);

            if (pidInControl && pid.atSetpoint(TARGET_TICKS, position)) {
                stopReason = String.format("settled at %.0f ticks", position);
                break;
            }

            if (pidInControl && moveTimer.seconds() > MOVE_TIMEOUT_SECONDS) {
                stopReason = String.format("timeout after %.1fs at %.0f ticks", MOVE_TIMEOUT_SECONDS, position);
                break;
            }

            if (pidInControl
                    && Math.abs(power) >= STALL_POWER_THRESHOLD
                    && Math.abs(position - lastPosition) <= STALL_TICK_EPSILON) {
                if (stallTimer.seconds() > STALL_TIMEOUT_SECONDS) {
                    stopReason = "mechanism is not moving under full power (encoder or binding)";
                    break;
                }
            } else {
                stallTimer.reset();
            }
            lastPosition = position;

            telemetry.addData("Target (ticks)", "%.0f", TARGET_TICKS);
            telemetry.addData("Actual (ticks)", "%.0f", position);
            telemetry.addData("Error (ticks)", "%.0f", TARGET_TICKS - position);
            telemetry.addData("Output (power)", "%.3f", power);
            telemetry.addData("Loop time (ms)", "%.1f", dt * 1000.0);
            telemetry.addData("Control", pidInControl ? "PID" : "manual");
            telemetry.addData("At setpoint", pid.atSetpoint(TARGET_TICKS, position));
            telemetry.update();

            DriverStation.telemetry("PID Target", String.format("%.0f", TARGET_TICKS));
            DriverStation.telemetry("PID Actual", String.format("%.0f", position));
            DriverStation.telemetry("PID Error", String.format("%.0f", TARGET_TICKS - position));
            DriverStation.telemetry("PID Output", String.format("%.3f", power));
            DriverStation.telemetry("PID Integral", String.format("%.4f", pid.getIntegralSum()));
            DriverStation.telemetry("PID Control", pidInControl ? "PID" : "manual");
        }

        // Always leave the mechanism at zero power, whatever the reason for stopping.
        motor.setPower(0.0);
        telemetry.addData("Stopped", stopReason);
        telemetry.update();
        DriverStation.log("PID position example stopped: " + stopReason);
    }
}