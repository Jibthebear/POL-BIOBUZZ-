package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Duo shoot twice and park", group = "Autonomous")
public class DuoShoot2Park extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();
    private boolean isAutoSwaying = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        int delaySeconds = 10;
        boolean lastDpadUp = false;
        boolean lastDpadDown = false;

        boolean parkLeft = true;
        boolean lastX = false;

        // Loop during INIT so drivers can adjust the delay before pressing Play
        while (!isStarted() && !isStopRequested()) {
            boolean currentDpadUp = gamepad1.dpad_up || gamepad2.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down || gamepad2.dpad_down;

            if (currentDpadUp && !lastDpadUp) {
                delaySeconds++;
            }
            if (currentDpadDown && !lastDpadDown) {
                delaySeconds = Math.max(0, delaySeconds - 1); // Prevents negative delay
            }

            lastDpadUp = currentDpadUp;
            lastDpadDown = currentDpadDown;

            boolean currentX = gamepad1.x || gamepad2.x;

            if (currentX && !lastX) {
                parkLeft = !parkLeft;
            }
            lastX = currentX;

            telemetry.addLine("Odometry Auto Ready.");
            telemetry.addLine("Use D-Pad UP / DOWN to adjust start delay.");
            telemetry.addData("Initial Delay", "%d seconds", delaySeconds);
            telemetry.addLine("Press Square to toggle parking zone.");
            telemetry.addData("Parking Zone", parkLeft ? "LEFT Parking Spot" : "RIGHT Parking Spot");
            telemetry.update();
        }

        if (isStopRequested()) return;

        robot.odo.resetPosAndIMU();
        sleep(100);

        // Start shooter
        robot.setShooterVelocity(0.6 * RobotHardware.MAX_SHOOTER_VELOCITY);

        // =========================================================================
        // WRITE YOUR AUTO PATH HERE
        // X is Forward/Backward (Positive is Forward)
        // Y is Left/Right (Positive is Left, Negative is Right)
        // =========================================================================

        // Shooter Spin-up / Alliance Delay (converted from seconds to milliseconds)
        telemetry.addData("Status", "Waiting %d seconds...", delaySeconds);
        telemetry.update();
        sleep(delaySeconds * 1000L);

        //Start shooting
        setGateUp();
        setSwaying(true);

        //Intake "glitch" to catch stuck balls
        holdPosition(0, 0.0, 0.0, 0.4, 1.5);
        runIntake(-1);
        holdPosition(0, 0.0, 0.0, 0.4, 1);
        runIntake(1);
        holdPosition(0, 0.0, 0.0, 0.4, .1);
        runIntake(-1);
        holdPosition(0, 0.0, 0.0, 0.4, 1.9);

        //Done shooting. First tip
        setGateFlat();
        setSwaying(false);

        // Get in front of flower
        driveTo(-16.7, 0, 0, .3);
        setRampOut();
        driveTo(-16.7, 13.1, 0, .3);

        //Get Flower
        driveTo(-4.9, 13.1, 0, .3);
        holdPosition(-4.9,13.1, 0.0, 0.4, 1.5);

        driveTo(-27.6, 13.1, 0, .3);
        driveTo(-16.7, 13.1, 0, .3);
        setRampRetracted();
        driveTo(-9.5, -0.7, 0, .3);
        driveTo(-0.5, -0.5, 0, .3);

        //Shoot New Balls/ glitch
        setGateUp();
        setSwaying(true);
        holdPosition(0, 0.0, 0.0, 0.4, 1.5);
        runIntake(-1);
        holdPosition(0, 0.0, 0.0, 0.4, 1);
        runIntake(1);
        holdPosition(0, 0.0, 0.0, 0.4, .1);
        runIntake(-1);
        holdPosition(0, 0.0, 0.0, 0.4, 1.9);

        //Park
        robot.setShooterVelocity(0.0 * RobotHardware.MAX_SHOOTER_VELOCITY);
        setGateFlat();
        runIntake(0);
        driveTo(-10.7, -0.1, 0, .3);
        driveTo(parkLeft? -9.3: -39, 50.8, 0, .3);
        holdPosition(parkLeft? -9.3: -39, 50.8, 0.0, 0.4, 300000);
    }

    // =========================================================================
    // MECHANISM HELPER FUNCTIONS
    // =========================================================================

    public void runIntake(double power) { robot.setIntakePower(power); }
    public void setGateUp() { robot.wallUp(); }
    public void setGateFlat() { robot.wallFlat(); }
    public void setRampOut() { robot.rampOut(); }
    public void setRampRetracted() { robot.rampRetracted(); }

    public void setSwaying(boolean active) {
        isAutoSwaying = active;
        if (!active) {
            robot.setSwayPosition(0.5);
        }
    }

    // =========================================================================
    // ODOMETRY DRIVE FUNCTIONS
    // =========================================================================

    public void driveTo(double targetXInches, double targetYInches, double targetHeadingDeg, double maxSpeed) {
        while (opModeIsActive() && executePID(targetXInches, targetYInches, targetHeadingDeg, maxSpeed)) {
        }
        robot.driveMecanum(0, 0, 0, 0);
    }

    public void holdPosition(double targetXInches, double targetYInches, double targetHeadingDeg, double maxSpeed, double holdTimeSeconds) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        while (opModeIsActive() && holdTimer.seconds() < holdTimeSeconds) {
            executePID(targetXInches, targetYInches, targetHeadingDeg, maxSpeed);
        }
        robot.driveMecanum(0, 0, 0, 0);
    }

    // =========================================================================
    // CORE PID MATH
    // =========================================================================

    private boolean executePID(double targetXInches, double targetYInches, double targetHeadingDeg, double maxSpeed) {
        double targetX = targetXInches * 25.4;
        double targetY = targetYInches * 25.4;

        double Kp_Translate = 0.0025;
        double Kp_Turn      = 0.010;

        robot.odo.update();

        if (isAutoSwaying) {
            double swayPos = (Math.sin(getRuntime() * 3.0) + 1.0) / 2.0;
            robot.setSwayPosition(swayPos);
        }

        double currentX = robot.odo.getPosX(DistanceUnit.MM);
        double currentY = robot.odo.getPosY(DistanceUnit.MM);
        double currentHeading = robot.odo.getHeading(AngleUnit.RADIANS);
        double currentHeadingDeg = Math.toDegrees(currentHeading);

        double fieldXError = targetX - currentX;
        double fieldYError = targetY - currentY;

        double headingError = targetHeadingDeg - currentHeadingDeg;
        while (headingError > 180) headingError -= 360;
        while (headingError < -180) headingError += 360;

        double distanceError = Math.hypot(fieldXError, fieldYError);

        double robotXError = fieldXError * Math.cos(currentHeading) + fieldYError * Math.sin(currentHeading);
        double robotYError = -fieldXError * Math.sin(currentHeading) + fieldYError * Math.cos(currentHeading);

        double rawForward = robotXError * Kp_Translate;
        double rawStrafe  = -robotYError * Kp_Translate;
        double rawTurn    = -headingError * Kp_Turn;

        double translationSpeed = Math.hypot(rawForward, rawStrafe);

        double driveForward = rawForward;
        double driveStrafe = rawStrafe;

        if (translationSpeed > maxSpeed) {
            double scaleFactor = maxSpeed / translationSpeed;
            driveForward *= scaleFactor;
            driveStrafe  *= scaleFactor;
        }

        double driveTurn = Math.max(-maxSpeed, Math.min(maxSpeed, rawTurn));

        robot.driveMecanum(driveForward, driveStrafe, driveTurn, 1.0);

        telemetry.addLine("=== REAL-TIME PID ODOMETRY ===");
        telemetry.addData("Forward (X)", "Target: %.1f in | Actual: %.1f in", targetXInches, currentX / 25.4);
        telemetry.addData("Left    (Y)", "Target: %.1f in | Actual: %.1f in", targetYInches, currentY / 25.4);
        telemetry.addData("Angle (Turn)", "Target: %.1f°  | Actual: %.1f°", targetHeadingDeg, currentHeadingDeg);
        telemetry.addLine("==============================");
        telemetry.update();

        double XYLeeway = 25.4;
        double ZLeeway = 2.0;
        return (distanceError > XYLeeway || Math.abs(headingError) > ZLeeway);
    }
}