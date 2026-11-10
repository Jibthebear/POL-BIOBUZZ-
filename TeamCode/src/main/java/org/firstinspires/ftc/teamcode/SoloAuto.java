package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Solo Auto", group = "Autonomous")
public class SoloAuto extends LinearOpMode {

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

        // Start shooter at exactly 61%
        robot.setShooterVelocity(0.61 * RobotHardware.MAX_SHOOTER_VELOCITY);

        // =========================================================================
        // WRITE YOUR AUTO PATH HERE
        // X is Forward/Backward (Positive is Forward)
        // Y is Left/Right (Positive is Left, Negative is Right)
        // =========================================================================

        // 1. Strafe right 2ft FAST (70% speed)
       // driveTo(48.0, 0.0, 0.0, 0.7);

        // 2. Open the gate and lock the drivetrain perfectly in place for 1.5 seconds while it shoots
        setGateUp();
        holdPosition(0, 0.0, 0.0, 0.4, 5);
        setGateFlat();

        // 3. Drive forward to the flowers at normal speed (40%)
        driveTo(24.0, 0.0, 0.0, 0.4);

        // 4. Shove the ramp out and lock position for 1 second to catch pollen
        setRampOut();
        holdPosition(24.0, 0.0, 0.0, 0.4, 1.0);
        setRampRetracted();

        // 5. Strafe left FAST
        driveTo(72.0, 0.0, 0.0, 0.7);

        // 6. Return to start
        driveTo(48.0, 0.0, 0.0, 0.5);

        // Clean up
        robot.setShooterVelocity(0);
        runIntake(0);

        telemetry.addLine("Auto Finished!");
        telemetry.update();
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

    // Drives to a point and exits the moment it gets within 1 inch and 2 degrees
    public void driveTo(double targetXInches, double targetYInches, double targetHeadingDeg, double maxSpeed) {
        // Loop continuously as long as executePID returns TRUE (meaning it hasn't reached the target yet)
        while (opModeIsActive() && executePID(targetXInches, targetYInches, targetHeadingDeg, maxSpeed)) {
            // The math and hardware updates are handled entirely inside executePID
        }
        robot.driveMecanum(0, 0, 0, 0);
    }
    //
    // Actively fights to hold a specific coordinate for a set amount of time
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

    // Returns TRUE if the robot is still outside the target zone, FALSE if it is on target
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

        double driveForward = Math.max(-maxSpeed, Math.min(maxSpeed, robotXError * Kp_Translate));
        double driveStrafe  = Math.max(-maxSpeed, Math.min(maxSpeed, -robotYError * Kp_Translate));
        double driveTurn    = Math.max(-maxSpeed, Math.min(maxSpeed, -headingError * Kp_Turn));

        robot.driveMecanum(driveForward, driveStrafe, driveTurn, 1.0);

        telemetry.addLine("=== REAL-TIME PID ODOMETRY ===");
        telemetry.addData("Forward (X)", "Target: %.1f in | Actual: %.1f in", targetXInches, currentX / 25.4);
        telemetry.addData("Left    (Y)", "Target: %.1f in | Actual: %.1f in", targetYInches, currentY / 25.4);
        telemetry.addData("Angle (Turn)", "Target: %.1f°  | Actual: %.1f°", targetHeadingDeg, currentHeadingDeg);
        telemetry.addLine("==============================");
        telemetry.update();

        // Check if the robot is outside the 1-inch (25.4mm) or 2-degree tolerance
        double XYLeeway = 25.4;
        double ZLeeway = 2;
        return (distanceError > XYLeeway || Math.abs(headingError) > ZLeeway);
    }
}