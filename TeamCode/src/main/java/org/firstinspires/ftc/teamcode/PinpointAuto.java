package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Pinpoint Auto Path", group = "Autonomous")
public class PinpointAuto extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();

    private boolean isAutoSwaying = false;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        telemetry.addLine("Odometry Auto Ready.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        robot.odo.resetPosAndIMU();
        sleep(100);

        robot.setShooterVelocity(0.61 * RobotHardware.MAX_SHOOTER_VELOCITY);

        // =========================================================================
        // WRITE YOUR AUTO PATH HERE
        // X is Forward/Backward (Positive is Forward)
        // Y is Left/Right (Positive is Left, Negative is Right)
        // =========================================================================

        // 1. Strafe right 2ft
        driveTo(48.0, 0.0, 0.0);

        // --- NEW: Example of shoving the ramp under the flowers ---
        // setRampOut();
        // sleep(1000); // Give pollen time to empty
        // setRampRetracted();

        // 2. Go forward 2ft
        driveTo(24.0, 0.0, 0.0);

        // 3. Strafe left 2ft
        driveTo(72.0, 0.0, 0.0);

        // 4. Go backwards 2ft
        driveTo(48.0, 0.0, 0.0);

        robot.setShooterVelocity(0);
        runIntake(0);

        telemetry.addLine("Auto Finished!");
        telemetry.update();
    }

    // =========================================================================
    // MECHANISM HELPER FUNCTIONS
    // =========================================================================

    public void runIntake(double power) {
        robot.setIntakePower(power);
    }

    public void setGateUp() {
        robot.wallUp();
    }

    public void setGateFlat() {
        robot.wallFlat();
    }

    public void setSwaying(boolean active) {
        isAutoSwaying = active;
        if (!active) {
            robot.setSwayPosition(0.5);
        }
    }

    // --- NEW: Flower Ramp Auto Commands ---
    public void setRampOut() {
        robot.rampOut();
    }

    public void setRampRetracted() {
        robot.rampRetracted();
    }

    // =========================================================================
    // ODOMETRY DRIVE FUNCTION
    // =========================================================================

    public void driveTo(double targetXInches, double targetYInches, double targetHeadingDeg) {

        double targetX = targetXInches * 25.4;
        double targetY = targetYInches * 25.4;

        double Kp_Translate = 0.0025;
        double Kp_Turn      = 0.010;

        double MAX_SPEED = 0.35;
        double MAX_TURN_SPEED = 0.3;

        double distanceError = 999;
        double headingError = 999;

        while (opModeIsActive() && (distanceError > 25.4 || Math.abs(headingError) > 2.0)) {

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

            headingError = targetHeadingDeg - currentHeadingDeg;
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;

            distanceError = Math.hypot(fieldXError, fieldYError);

            double robotXError = fieldXError * Math.cos(currentHeading) + fieldYError * Math.sin(currentHeading);
            double robotYError = -fieldXError * Math.sin(currentHeading) + fieldYError * Math.cos(currentHeading);

            double driveForward = robotXError * Kp_Translate;
            double driveStrafe  = -robotYError * Kp_Translate;
            double driveTurn    = -headingError * Kp_Turn;

            driveForward = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, driveForward));
            driveStrafe  = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, driveStrafe));
            driveTurn    = Math.max(-MAX_TURN_SPEED, Math.min(MAX_TURN_SPEED, driveTurn));

            robot.driveMecanum(driveForward, driveStrafe, driveTurn, 1.0);

            telemetry.addLine("=== REAL-TIME PID ODOMETRY ===");
            telemetry.addData("Forward (X)", "Target: %.1f in | Actual: %.1f in", targetXInches, currentX / 25.4);
            telemetry.addData("Left    (Y)", "Target: %.1f in | Actual: %.1f in", targetYInches, currentY / 25.4);
            telemetry.addData("Angle (Turn)", "Target: %.1f°  | Actual: %.1f°", targetHeadingDeg, currentHeadingDeg);
            telemetry.addLine("==============================");
            telemetry.update();
        }

        robot.driveMecanum(0, 0, 0, 0);
    }
}