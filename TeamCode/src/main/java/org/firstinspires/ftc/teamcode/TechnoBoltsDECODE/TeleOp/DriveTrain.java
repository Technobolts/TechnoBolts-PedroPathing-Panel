package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    // 1. HARDWARE DECLARATIONS
    // Drivetrain Motors
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    // Intake Motor (Expansion Hub 2, Port 2)
    public DcMotorEx intake;

    public DcMotorEx encoderX; // Parallel wheel (tracks forward/backward)
    public DcMotorEx encoderY; // Perpendicular wheel (tracks strafe/sideways)

    // ==========================================
    // 2. INITIALIZATION METHOD
    // ==========================================
    public void init(HardwareMap hardwareMap) {

        // Map motors to the names configured on the Driver Station
        frontLeft = hardwareMap.get(DcMotor.class, "front-left");
        frontRight = hardwareMap.get(DcMotor.class, "front-right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back-left");
        backRight = hardwareMap.get(DcMotorEx.class, "back-right");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        /*
         * ODOMETRY ENCODER PORT MAPPING
         * TODO: change these ports to what it actually is
         */
        encoderX = hardwareMap.get(DcMotorEx.class, "encoder-x");
        encoderY = hardwareMap.get(DcMotorEx.class, "encoder-y");

        // Reset the odometry pods to 0 ticks at startup
        encoderX.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set them to run without internal velocity PID (required for dead wheels)
        encoderX.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderY.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Configure Motor Directions
        // Reverse left side so positive power drives the whole robot forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Intake direction (switch to REVERSE if it spits out instead of sucking in)
        intake.setDirection(DcMotor.Direction.FORWARD);

        // Set Zero Power Behaviors
        // BRAKE helps the drivetrain stop immediately when you let go of the sticks
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // FLOAT keeps the intake safe from snapping if a game element jams
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set drivetrain motors to run using raw power percentages
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // 3. GAMEPAD CONTROL METHOD
    public void handleControls(Gamepad gamepad1) {

        // Mecanum Drive Math
        double y   = -gamepad1.left_stick_y; // Inverted because joysticks are negative when pushed up
        double x   =  gamepad1.left_stick_x;
        double rx  =  gamepad1.right_stick_x; // Controls rotation

        // Calculate power for each wheel
        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;

        // Scale powers proportionally if any value exceeds 1.0 (100% motor speed)
        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

        if (max > 1.0) {
            frontLeftPower  /= max;
            backLeftPower   /= max;
            frontRightPower /= max;
            backRightPower  /= max;
        }

        // Apply calculated power values to motors
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        // Intake Button Mapping
        if (gamepad1.right_bumper) {
            intake.setPower(1.0);  // Intake fully inward
        } else if (gamepad1.left_bumper) {
            intake.setPower(-1.0); // Outtake/Spit out fully
        } else {
            intake.setPower(0.0);  // Stop spin when no button is held
        }
    }
}



    /*
     * CONTROL HUB CONFIG
     * Motors:
     * front-right: port 0
     * back-right: port 1
     *EXPANSION HUB
     * front-left: PORT 0
     * back-left port 1
     * Encoders:
     * encoder-right:
     * encoder-left:
     * */

