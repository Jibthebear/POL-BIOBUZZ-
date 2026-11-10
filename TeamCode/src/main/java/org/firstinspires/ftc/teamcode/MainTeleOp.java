package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Main TeleOp", group = "TeleOp")
public class MainTeleOp extends LinearOpMode {

    private static final double DRIVE_NORMAL_MULTIPLIER = 0.65;
    private static final double DRIVE_PRECISION_MULTIPLIER = 0.35;
    private static final double SHOOTER_STEP = 0.05;
    private static final double SHOOTER_POWER = .61;

    private double shooterPower = 0.0;
    private double intakePower = 0.0;

    // Button state trackers
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastRightBumper2 = false;
    private boolean lastL2 = false;
    private boolean lastR2 = false;

    // --- NEW: Button Tracker for Flower Ramp ---
    private boolean lastX2 = false;

    // Hardware states
    private boolean isWallUp = false;
    private boolean isSwaying = false;

    // --- NEW: Hardware state for Ramp ---
    private boolean isRampOut = false;

    private final RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.odo.resetPosAndIMU();
        sleep(100);

        telemetry.addLine("Ready to start! (DcMotorEx PIDF Active)");
        telemetry.addLine("ODOMETRY: IMU Initialized securely.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            robot.odo.update();

            // =======================================================
            // 1. Drivetrain (Gamepad 1)
            // =======================================================
            double y  = -gamepad1.left_stick_y;
            double x  = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double speedMultiplier = gamepad1.right_bumper ? DRIVE_PRECISION_MULTIPLIER : DRIVE_NORMAL_MULTIPLIER;

            robot.driveMecanum(y, x, rx, speedMultiplier);

            // =======================================================
            // 2. Shooter Motor Velocity (Gamepad 2)
            // =======================================================
            if (gamepad2.left_bumper) {
                shooterPower = 0.0;
            } else {
                if (gamepad2.dpad_up && !lastDpadUp) {
                    shooterPower = SHOOTER_POWER;
                }
                if (gamepad2.dpad_down && !lastDpadDown) {
                    shooterPower = 0;
                }
            }
            lastDpadUp = gamepad2.dpad_up;
            lastDpadDown = gamepad2.dpad_down;

            shooterPower = Math.max(-1.0, Math.min(1.0, Math.round(shooterPower * 100.0) / 100.0));

            double targetVelocity = shooterPower * RobotHardware.MAX_SHOOTER_VELOCITY;
            robot.setShooterVelocity(targetVelocity);

            // =======================================================
            // 3. Toggleable Intake (Gamepad 1 Triggers & Cross/A button)
            // =======================================================
            boolean currentL2 = gamepad1.left_trigger > 0.5;
            boolean currentR2 = gamepad1.right_trigger > 0.5;

            if (currentL2 && !lastL2) {
                intakePower = 1.0;
            }
            else if (currentR2 && !lastR2) {
                intakePower = -1.0;
            }
            else if (gamepad1.cross) {
                intakePower = 0.0;
            }

            lastL2 = currentL2;
            lastR2 = currentR2;
            robot.setIntakePower(intakePower);

            // =======================================================
            // 4. Wall Servo, Sway & Flower Ramp (Gamepad 2)
            // =======================================================
            if (gamepad2.triangle) {
                robot.wallUp();
                isWallUp = false;
            } else {
                robot.wallFlat();
                isWallUp = true;
            }

            if (gamepad2.right_bumper && !lastRightBumper2) {
                isWallUp = !isWallUp;
                if (isWallUp) {
                    robot.wallUp();
                } else {
                    robot.wallFlat();
                }
            }
            lastRightBumper2 = gamepad2.right_bumper;

            if (gamepad2.dpad_left && !lastDpadLeft) {
                isSwaying = !isSwaying;
            }
            lastDpadLeft = gamepad2.dpad_left;

            if (isSwaying) {
                double swayPos = (Math.sin(getRuntime() * 3.0) + 1.0) / 2.0;
                robot.setSwayPosition(swayPos);
            } else {
                robot.setSwayPosition(.5);
            }

            // --- NEW: Toggle the Flower Ramp on Gamepad 2 Square/X ---
            if (gamepad2.x && !lastX2) {
                isRampOut = !isRampOut;
                if (isRampOut) {
                    robot.rampOut();
                } else {
                    robot.rampRetracted();
                }
            }
            lastX2 = gamepad2.x;

            // =======================================================
            // 5. Advanced Telemetry for PIDF Tuning
            // =======================================================
            double actualVelocity = robot.getShooterVelocity();
            double velocityError = targetVelocity - actualVelocity;

            telemetry.addData("Shooter Power", "%.0f%%", shooterPower * 100);

            telemetry.addLine("=== REAL-TIME ODOMETRY ===");
            telemetry.addData("X (Forward/Back)", "%.1f in", robot.odo.getPosX(DistanceUnit.INCH));
            telemetry.addData("Y (Left/Right)", "%.1f in", robot.odo.getPosY(DistanceUnit.INCH));
            telemetry.addData("Angle (CCW+)", "%.1f°", robot.odo.getHeading(AngleUnit.DEGREES));
            telemetry.addLine("==========================");

            telemetry.addLine("--- Hardware ---");
            telemetry.addData("Intake Status", intakePower > 0 ? "IN" : (intakePower < 0 ? "OUT" : "OFF"));
            telemetry.addData("WallUp", isWallUp);
            telemetry.addData("Ramp Out", isRampOut); // Show ramp state on driver hub
            telemetry.update();
        }
    }
}