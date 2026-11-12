package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Solo shoot twice and park", group = "Autonomous")
public class SoloShoot2Park extends LinearOpMode {

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
        // Quite easy to run Tele mode with Odom readouts and physically move the robot and record positions
        // =========================================================================

        // Shooter Spin-up
        sleep(1500);

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

        //Approach Flower
        driveTo(-36.2, -33.9, -89.9, 0.3);
        setRampOut();
        runIntake(-1);
        holdPosition(-36.2, -33.9, -89.9, 0.3, .5);

        //Engage with flower
        driveTo(-37.2, -43.8, -89.9, 1);
        holdPosition(-37.2, -43.8, -89.9, 0.3, 1.5);

        //Back off from flower
        driveTo(-38.2, -24.8, -89.9, 0.3);

        //Drive forward a bit to intake remaining balls
        driveTo(-38.2, -33.4, -89.9, 0.3);
        setRampRetracted();

        //Face the next target
        driveTo(-38.2, -33.4, 180, 0.3);
        runIntake(0); // to not accidentally eat any other balls.

        //Go to the next target. takes a path that maneuvers around the hive structure and flower. Positions robot for shooting.
        driveTo(-83.2, -33.7, 180, .6);
        driveTo(-107.6, 2, 180, 1);
        driveTo(-118.5, 3.1, 180, 1);

        //Start shooting and run the "glitch" procedure
        setGateUp();
        setSwaying(true);
        holdPosition(-118.4, 3.1, 180, 0.3, 1.5);
        runIntake(-1);
        holdPosition(-118.4, 3.1, 180, 0.4, 1);
        runIntake(1);
        holdPosition(-118.4, 3.1, 180, 0.4, .3);
        runIntake(-1);
        holdPosition(-118.4, 3.1, 180, 0.4, 1.7); //Hold to let balls fall into shooter

        //Stop shooting
        setGateFlat();
        runIntake(0);
        robot.setShooterVelocity(0);

        //Go to parking zone and park. Turn off shooter.
        driveTo(-104.3, 5.5, 180, 0.6);
        telemetry.addLine("Auto Finished!");
        telemetry.update();
        holdPosition(-108.5, -46.6, 180, 0.4, 100);
        // Done
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

        // 1. Calculate raw PID outputs BEFORE speed limits
        double rawForward = robotXError * Kp_Translate;
        double rawStrafe  = -robotYError * Kp_Translate;
        double rawTurn    = -headingError * Kp_Turn;

        // 2. Vector Normalization (The Straight Line Fix)
        double translationSpeed = Math.hypot(rawForward, rawStrafe);

        double driveForward = rawForward;
        double driveStrafe = rawStrafe;

        // If combined speed is too fast, scale BOTH down by the same percentage to maintain the angle
        if (translationSpeed > maxSpeed) {
            double scaleFactor = maxSpeed / translationSpeed;
            driveForward *= scaleFactor;
            driveStrafe  *= scaleFactor;
        }

        // Turn can still be clamped independently since it doesn't affect straight-line geometry
        double driveTurn = Math.max(-maxSpeed, Math.min(maxSpeed, rawTurn));

        robot.driveMecanum(driveForward, driveStrafe, driveTurn, 1.0);

        telemetry.addLine("=== REAL-TIME PID ODOMETRY ===");
        telemetry.addData("Forward (X)", "Target: %.1f in | Actual: %.1f in", targetXInches, currentX / 25.4);
        telemetry.addData("Left    (Y)", "Target: %.1f in | Actual: %.1f in", targetYInches, currentY / 25.4);
        telemetry.addData("Angle (Turn)", "Target: %.1f°  | Actual: %.1f°", targetHeadingDeg, currentHeadingDeg);
        telemetry.addLine("==============================");
        telemetry.update();

        // Check if the robot is outside the 1-inch (25.4mm) or 2-degree tolerance
        double XYLeeway = 25.4;
        double ZLeeway = 2.0;
        return (distanceError > XYLeeway || Math.abs(headingError) > ZLeeway);
    }
}