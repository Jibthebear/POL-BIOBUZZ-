package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotorEx shooterMotor;
    private DcMotor intakeMotor;

    private Servo wallServo;
    private Servo swayServo;

    // Flower Ramp Servo
    private Servo flowerRampServo;

    public GoBildaPinpointDriver odo;

    public static final double MAX_SHOOTER_VELOCITY = 2450.0;
    public static final double STRAFE_CORRECTION = 1.1;

    public static final double WALL_FLAT = 0.5;
    public static final double WALL_UP   = 0.4;

    // Flower Ramp Servo Positions
    public static final double RAMP_RETRACTED = 1.0;
    public static final double RAMP_OUT = 0.0;

    private double wallPosition = WALL_FLAT;
    private double rampPosition = RAMP_RETRACTED;

    public void init(HardwareMap hardwareMap) {

        frontLeft  = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        backLeft   = hardwareMap.get(DcMotor.class, "back_left");
        backRight  = hardwareMap.get(DcMotor.class, "back_right");

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        intakeMotor  = hardwareMap.get(DcMotor.class, "intake_motor");

        wallServo       = hardwareMap.get(Servo.class, "wall_servo");
        swayServo       = hardwareMap.get(Servo.class, "sway_servo");

        // Map the flower ramp
        flowerRampServo = hardwareMap.get(Servo.class, "flower_ramp");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ==========================================================
        // SHOOTER PIDF TUNING
        // ==========================================================
        PIDFCoefficients shooterPIDF = new PIDFCoefficients(100.0, 0.0, 0.0, 13.0);
        shooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        wallPosition = WALL_FLAT;
        wallServo.setPosition(wallPosition);
        swayServo.setPosition(0.5);

        rampRetracted();

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        // Fixed offsets with DistanceUnit
        odo.setOffsets(210, -20, org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
    }

    public void driveMecanum(double y, double x, double rx, double speedMultiplier) {
        double correctedX = x * STRAFE_CORRECTION;
        double denominator = Math.max(Math.abs(y) + Math.abs(correctedX) + Math.abs(rx), 1.0);

        frontLeft.setPower(((y + correctedX + rx) / denominator) * speedMultiplier);
        backLeft.setPower(((y - correctedX + rx) / denominator) * speedMultiplier);
        frontRight.setPower(((y - correctedX - rx) / denominator) * speedMultiplier);
        backRight.setPower(((y + correctedX - rx) / denominator) * speedMultiplier);
    }

    public void setShooterVelocity(double ticksPerSecond) {
        shooterMotor.setVelocity(ticksPerSecond);
    }

    public double getShooterVelocity() {
        return shooterMotor.getVelocity();
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(clamp(power, -1.0, 1.0));
    }

    public void wallUp() {
        wallPosition = WALL_UP;
        wallServo.setPosition(wallPosition);
    }

    public void wallFlat() {
        wallPosition = WALL_FLAT;
        wallServo.setPosition(wallPosition);
    }

    public void setSwayPosition(double position) {
        swayServo.setPosition(position);
    }

    public void rampOut() {
        rampPosition = RAMP_OUT;
        flowerRampServo.setPosition(rampPosition);
    }

    public void rampRetracted() {
        rampPosition = RAMP_RETRACTED;
        flowerRampServo.setPosition(rampPosition);
    }

    public double getWallPosition() { return wallPosition; }
    public double getRampPosition() { return rampPosition; }

    public int getFrontLeftPosition()  { return frontLeft.getCurrentPosition(); }
    public int getFrontRightPosition() { return frontRight.getCurrentPosition(); }
    public int getBackLeftPosition()   { return backLeft.getCurrentPosition(); }
    public int getBackRightPosition()  { return backRight.getCurrentPosition(); }
    public int getIntakePosition()     { return intakeMotor.getCurrentPosition(); }
//
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}