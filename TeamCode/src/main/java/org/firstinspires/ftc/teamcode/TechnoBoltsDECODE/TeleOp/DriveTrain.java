package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TeleOp;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver; // Official Pinpoint Driver
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Drive Train with Pinpoint Test", group = "OpMode")
public class DriveTrain extends OpMode {

    // 1. HARDWARE DECLARATIONS
    // Drivetrain Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    // Intake Motor
    public DcMotorEx intake;

    // Pinpoint Odometry Computer Object
    public GoBildaPinpointDriver pinpoint;

    // ==========================================
    // 2. INITIALIZATION METHOD
    // ==========================================
    @Override
    public void init() {

        // Map motors to the names configured on the Driver Station
        frontLeft = hardwareMap.get(DcMotor.class, "front-left");
        frontRight = hardwareMap.get(DcMotor.class, "front-right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back-left");
        backRight = hardwareMap.get(DcMotorEx.class, "back-right");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        // Initialize Pinpoint Computer (Make sure "pinpoint" matches your Driver Station active config exactly)
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        /*
         * PINPOINT CALIBRATION & CONFIGURATION
         * 1. Set the physical offsets of your pods relative to your robot's center of rotation (in mm or inches).
         * X offset is how far forward/backward the tracking point is. Y offset is how far left/right it is.
         */
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM); // TODO: Input your team's specific physical offsets

        /*
         * 2. Define your encoder resolution.
         * If using goBILDA pods, use goBILDA_4_BAR_POD or goBILDA_SWINGARM_POD.
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * 3. Set Directions. X should increase when pushed forward. Y should increase when pushed left.
         * Change GoBildaPinpointDriver.EncoderDirection.REVERSED if your numbers read backward.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * 4. Reset position and recalibrate built-in IMU.
         * CRITICAL: The robot must remain perfectly still for roughly 0.25 seconds during initialization for this.
         */
        pinpoint.resetPosAndIMU();

        // Configure Motor Directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Set Zero Power Behaviors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set drivetrain motors to run using raw power percentages
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // ==========================================
    // 3. MAIN LOOP METHOD
    // ==========================================
    @Override
    public void loop() {
        // CRITICAL: You must call update() every loop cycle, or your odometry tracking will not refresh!
        pinpoint.update();

        // Handle driving and auxiliary motor controls
        handleControls(gamepad1);

        // Fetch and display live tracking data on the driver station
        Pose2D currentPose = pinpoint.getPosition();
        telemetry.addData("X Position (in)", currentPose.getX(DistanceUnit.INCH));
        telemetry.addData("Y Position (in)", currentPose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", currentPose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
        telemetry.update();
    }

    // ==========================================
    // 4. GAMEPAD CONTROL METHOD
    // ==========================================
    public void handleControls(Gamepad gamepad1) {
        double y   = -gamepad1.left_stick_y;
        double x   =  gamepad1.left_stick_x;
        double rx  =  gamepad1.right_stick_x;

        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;

        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

        if (max > 1.0) {
            frontLeftPower  /= max;
            backLeftPower   /= max;
            frontRightPower /= max;
            backRightPower  /= max;
        }

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        if (gamepad1.right_bumper) {
            intake.setPower(1.0);
        } else if (gamepad1.left_bumper) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }
    }
}