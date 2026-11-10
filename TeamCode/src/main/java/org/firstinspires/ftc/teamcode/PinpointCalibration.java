package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Pinpoint Calibration Tool", group = "Tuning")
public class PinpointCalibration extends LinearOpMode {

    private final RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.odo.resetPosAndIMU();
        sleep(100); // FIX 3: Added delay to prevent I2C garbage data (800000+ bug)

        telemetry.addLine("=== Pinpoint Calibration ===");
        telemetry.addLine("1. Press Start.");
        telemetry.addLine("2. Hold Gamepad 1 Right Stick ALL THE WAY to spin.");
        telemetry.addLine("3. Spin at least 10 times for perfect math.");
        telemetry.update();

        waitForStart();

        // Variables to track continuous rotation past 180 degrees
        double accumulatedHeadingDegrees = 0;
        double lastHeadingDegrees = 0;

        // FIX 4: Snapshot variables to ignore the messy first spin
        boolean referenceSet = false;
        double refHeadingDeg = 0;
        double refXTravelMm = 0;
        double refYTravelMm = 0;

        while (opModeIsActive()) {

            // Allow the driver to spin the robot in place
            robot.driveMecanum(0, 0, gamepad1.right_stick_x, 0.65);

            robot.odo.update();

            double currentHeadingDegrees = robot.odo.getHeading(AngleUnit.DEGREES);

            // Track Accumulated Rotation to prevent snapping back to 0
            double deltaHeading = currentHeadingDegrees - lastHeadingDegrees;
            if (deltaHeading < -180) {
                deltaHeading += 360;
            } else if (deltaHeading > 180) {
                deltaHeading -= 360;
            }
            accumulatedHeadingDegrees += deltaHeading;
            lastHeadingDegrees = currentHeadingDegrees;

            // Convert raw ticks to true millimeters (goBILDA 4-bar pod = 13.2629 ticks/mm)
            double ticksPerMm = 13.2629;
            double currentXTravelMm = robot.odo.getEncoderX() / ticksPerMm;
            double currentYTravelMm = robot.odo.getEncoderY() / ticksPerMm;

            // Take a snapshot exactly after 1 full spin to eliminate startup slip
            if (Math.abs(accumulatedHeadingDegrees) >= 360 && !referenceSet) {
                referenceSet = true;
                refHeadingDeg = accumulatedHeadingDegrees;
                refXTravelMm = currentXTravelMm;
                refYTravelMm = currentYTravelMm;
            }

            if (!referenceSet) {
                telemetry.addLine("=== REAL-TIME CALIBRATION ===");
                telemetry.addLine("Keep spinning! Discarding the first spin to eliminate startup slip...");
                telemetry.addData("Spins Completed", "%.1f / 1.0", Math.abs(accumulatedHeadingDegrees) / 360.0);
            } else {
                // Calculate math relative ONLY to the smooth spins after the snapshot
                double effectiveHeadingDeg = accumulatedHeadingDegrees - refHeadingDeg;
                double effectiveXTravelMm = currentXTravelMm - refXTravelMm;
                double effectiveYTravelMm = currentYTravelMm - refYTravelMm;

                double effectiveRadians = Math.toRadians(effectiveHeadingDeg);

                double xOffsetMm = 0;
                double yOffsetMm = 0;

                // Avoid divide by zero if it hasn't moved past the snapshot yet
                if (Math.abs(effectiveRadians) > 0.1) {
                    xOffsetMm = effectiveXTravelMm / effectiveRadians;
                    yOffsetMm = effectiveYTravelMm / effectiveRadians;
                }

                telemetry.addData("Total Spins", "%.1f", Math.abs(accumulatedHeadingDegrees) / 360.0);
                telemetry.addLine("--------------------------------");
                telemetry.addData("Calculated X Offset (mm)", "%.2f", xOffsetMm);
                telemetry.addData("Calculated Y Offset (mm)", "%.2f", yOffsetMm);
                telemetry.addLine("--------------------------------");
                telemetry.addLine("Write down these offsets once you hit 10 spins.");
                telemetry.addLine("Put them in RobotHardware.java -> odo.setOffsets(X, Y, ...)");
            }

            telemetry.update();
        }
    }
}