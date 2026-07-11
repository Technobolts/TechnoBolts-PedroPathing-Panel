package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RobotTeleOp extends OpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;
    public DcMotorEx TurretShooter;

    // Intake Motor (Expansion Hub 2, Port 2)

    public DcMotorEx encoderX; // Parallel wheel (tracks forward/backward)
    public DcMotorEx encoderY; // Perpendicular wheel (tracks strafe/sideways)

    private Servo spindexer;
    private Servo Kicker;
    private Servo TurretHood;

    private DcMotorEx intake1;

    private int slot = 0;

    double F = 18;

    double P = 27;

    double HoodAngle = 0;

    // YOUR TUNED POSITIONS
    private final double[] intakePos = {
            0.10,
            0.46,
            0.83
    };

    private final double[] shootPos = {
            0.25,
            0.62,
            0.99
    };

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastRB = false;


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
        spindexer = hardwareMap.get(Servo.class, "spindexerServo");
        TurretHood = hardwareMap.get(Servo.class, "hoodShooter");
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        Kicker = hardwareMap.get(Servo.class, "kickerServo");
        TurretShooter = hardwareMap.get(DcMotorEx.class, "turretShooter");
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0,0,F);
        TurretShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        telemetry.addLine("Init Complete");
        TurretHood.scaleRange(0.0, 0.9);

        // Configure Motor Directions
        // Reverse left side so positive power drives the whole robot forward
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        TurretShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        TurretHood.setDirection(Servo.Direction.REVERSE);

        // Set Zero Power Behaviors
        // BRAKE helps the drivetrain stop immediately when you let go of the sticks
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // FLOAT keeps the intake safe from snapping if a game element jams
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);



        // Set drivetrain motors to run using raw power percentages
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        slot = 0;
        spindexer.setPosition(intakePos[slot]);

        Kicker.setPosition(0.15);
    }


    @Override
    public void loop() {


        //----------Drivetrain-------------------

        double y   =  gamepad1.left_stick_y; // Inverted because joysticks are negative when pushed up
        double x   =  -gamepad1.left_stick_x; // Strafe
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


        //----------Intake------------------


        // Intake Button Mapping
        if (gamepad1.right_bumper) {
            intake1.setPower(0.6);  // Intake fully inward
        } else if (gamepad1.left_bumper) {
            intake1.setPower(-0.6); // Outtake/Spit out fully
        } else {
            intake1.setPower(0.0);  // Stop spin when no button is held
        }

        //----------Spindexer-------------------


        // =========================
        // INTAKE (instant snap)
        // =========================
        if (gamepad1.a && !lastA) {
            spindexer.setPosition(intakePos[slot]);
        }

        // =========================
        // SHOOT (instant snap)
        // =========================
        if (gamepad1.b && !lastB) {
            spindexer.setPosition(shootPos[slot]);
        }

        // =========================
        // CLOCKWISE ONLY INDEX
        // (important for stability)
        // =========================
        if (gamepad1.yWasPressed() && !lastRB) {

            slot = (slot + 1) % 3;

            // ALWAYS return to intake after indexing
            spindexer.setPosition(intakePos[slot]);
        }

        // save buttons
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastRB = gamepad1.yWasPressed();


        //----------Kicker-------------------

        if(gamepad1.dpad_up){
            Kicker.setPosition(0.5);
        }
        else {
            Kicker.setPosition(0.15);
        }

        //----------Turret-------------------

        double curTargetVelocity = 2000;

        if(gamepad1.x){
            TurretShooter.setVelocity(curTargetVelocity);
        }
        else{
            TurretShooter.setVelocity(0);
        }
        //        double curVelocity2 = ShooterLeft.getVelocity();
        double curVelocity1 = TurretShooter.getVelocity();

//        double errorLeft = curTargetVelocity - curVelocity2;
        double error = curTargetVelocity - curVelocity1;

        if(gamepad1.dpadRightWasPressed()){
            HoodAngleIncrease(0.1);
        }
        if(gamepad1.dpadLeftWasPressed()){
            HoodAngleDecrease(0.1);
        }

        TurretHood.setPosition(HoodAngle);


        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity1);
        telemetry.addData("Error Right",  "%.2f", error);
        telemetry.addData("Hood Angle",  "%.2f", HoodAngle);
        telemetry.addData("Slot", slot);
        telemetry.addData("Intake", intakePos[slot]);
        telemetry.addData("Shoot", shootPos[slot]);
        telemetry.update();

    }

    private void HoodAngleDecrease(double amount) {
        HoodAngle -= amount;
    }

    private void HoodAngleIncrease(double amount) {
        HoodAngle += amount;
    }
}

