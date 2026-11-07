package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Timed autonomous template; tune on the actual field before competition use. */
public abstract class BioBuzzTimedAuto extends LinearOpMode {
    protected final Robot robot = new Robot();
    private final FlowerArm flowerArm = new FlowerArm();
    private final ElapsedTime match = new ElapsedTime();
    private static final double ARM_LOWER_SECONDS = 1.2;
    private static final double ARM_RAISE_TIMEOUT_SECONDS = 4.0;

    public static double INCHES_PER_SECOND_AT_HALF_POWER = 20.0;
    public static double TURN_POWER = 0.45;
    public static double SECONDS_PER_90_DEGREES = 0.60;
    public static double MOVE_POWER = 0.42;
    public static double SHOOT_SECONDS = 2.8;
    public static double INTAKE_SECONDS = 2.4;
    public static double LOADER_SETTLE_SECONDS = 0.35;
    public static double FLOWER_LEG_INCHES = 28.0;
    public static double PARK_LEG_INCHES = 26.0;
    public static double NOMINAL_BATTERY_VOLTS = 12.0;

    protected enum Plan {
        WAIT_SHOOT_RETURN_SHOOT,
        SHOOT_RETURN_WAIT_SHOOT,
        WAIT_SHOOT_CROSS_PARK,
        SHOOT_CROSS_PARK
    }

    protected abstract boolean isRed();
    protected abstract Plan plan();

    @Override public final void runOpMode() {
        flowerArm.init(hardwareMap);
        robot.init(hardwareMap);
        try {
            telemetry.addLine("BIOBUZZ timed auto — calibrate distance and turns before use");
            telemetry.addData("Alliance", isRed() ? "RED" : "BLUE");
            telemetry.addData("Route", plan());
            telemetry.addLine("Start aimed toward the alliance FLOWER");
            telemetry.update();
            waitForStart();
            if (!opModeIsActive()) return;

            if (!flowerArm.isUpperLimitPressed()) {
                telemetry.addLine("Flower arm must touch upper sensor at START; autonomous aborted");
                telemetry.update();
                return;
            }
            match.reset();
            Plan selected = plan();
            if (selected == Plan.WAIT_SHOOT_RETURN_SHOOT || selected == Plan.WAIT_SHOOT_CROSS_PARK) {
                waitSeconds(7.0);
            }

            shoot(SHOOT_SECONDS); // Up to four preloaded POLLEN
            if (!goToFlowerAndIntake()) return;

            if (selected == Plan.WAIT_SHOOT_RETURN_SHOOT || selected == Plan.SHOOT_RETURN_WAIT_SHOOT) {
                turn(180);
                driveInches(FLOWER_LEG_INCHES, MOVE_POWER);
                turn(180);
                if (selected == Plan.SHOOT_RETURN_WAIT_SHOOT) waitSeconds(2.0);
                if (match.seconds() < 27.0) shoot(Math.min(SHOOT_SECONDS, 29.0 - match.seconds()));
            } else {
                crossAndPark();
            }

            if (opModeIsActive()) {
                telemetry.addData("AUTO elapsed", "%.1fs", match.seconds());
                telemetry.update();
            }
        } finally {
            flowerArm.stop();
            stopAll();
            robot.close();
        }
    }

    private boolean goToFlowerAndIntake() {
        driveInches(FLOWER_LEG_INCHES, MOVE_POWER);
        if (!moveFlowerArm(false, ARM_LOWER_SECONDS)) return false;
        robot.setMatchLoaderDown(true);
        waitSeconds(LOADER_SETTLE_SECONDS);
        robot.intakeSystem.intakeIn();
        waitSeconds(INTAKE_SECONDS);
        robot.intakeSystem.stopIntake();
        robot.setMatchLoaderDown(false);
        waitSeconds(LOADER_SETTLE_SECONDS);
        return moveFlowerArm(true, ARM_RAISE_TIMEOUT_SECONDS);
    }

    private boolean moveFlowerArm(boolean up, double seconds) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && match.seconds() < 29.0 && timer.seconds() < seconds) {
            if (up && flowerArm.isUpperLimitPressed()) {
                flowerArm.stop();
                return true;
            }
            flowerArm.update(up, !up);
            telemetry.addData("Flower arm", up ? "RAISING" : "LOWERING");
            telemetry.addData("Flower upper limit", flowerArm.isUpperLimitPressed());
            telemetry.update();
            idle();
        }
        flowerArm.stop();
        boolean success = opModeIsActive() && match.seconds() < 29.0
                && (!up || flowerArm.isUpperLimitPressed());
        if (!success) {
            telemetry.addLine("Flower arm stopped: STOP, deadline, or upper sensor timeout; route aborted");
            telemetry.update();
        }
        return success;
    }

    private void crossAndPark() {
        turn(isRed() ? -90 : 90);
        driveInches(PARK_LEG_INCHES, MOVE_POWER * 0.80);
        turn(isRed() ? 90 : -90);
    }

    private void shoot(double seconds) {
        if (seconds <= 0 || !opModeIsActive()) return;
        robot.shooter1System.spinForwardMaxSpeed();
        robot.shooter2System.spinForwardMaxSpeed();
        waitSeconds(seconds);
        robot.shooter1System.spinDown();
        robot.shooter2System.spinDown();
    }

    private void driveInches(double inches, double power) {
        double scale = Math.max(0.10, Math.abs(power) / 0.50);
        double seconds = Math.abs(inches) / (INCHES_PER_SECOND_AT_HALF_POWER * scale);
        driveFor(inches >= 0 ? power : -power, 0, seconds);
    }

    private void turn(double degrees) {
        driveFor(0, Math.copySign(TURN_POWER, degrees),
                Math.abs(degrees) / 90.0 * SECONDS_PER_90_DEGREES);
    }

    private void driveFor(double forward, double turn, double seconds) {
        double batteryVolts = NOMINAL_BATTERY_VOLTS;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double reading = sensor.getVoltage();
            if (reading > 1.0 && reading < batteryVolts) batteryVolts = reading;
        }
        double compensatedSeconds = seconds * NOMINAL_BATTERY_VOLTS / batteryVolts;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < compensatedSeconds) {
            robot.drive(forward, turn);
            idle();
        }
        robot.drive(0, 0);
    }

    private void waitSeconds(double seconds) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (opModeIsActive() && timer.seconds() < seconds) idle();
    }

    private void stopAll() {
        robot.drive(0, 0);
        robot.intakeSystem.stopIntake();
        robot.shooter1System.spinDown();
        robot.shooter2System.spinDown();
        robot.setMatchLoaderDown(false);
    }
}
