package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    // SECTION 1: New Variables & Imports
    // Flywheel
    private final double TARGET_VELOCITY = 2000;
    private boolean shooterEnabled = false;
    private boolean lastX = false;

    // Ball Tracking
    private int ballsLoaded = 0;

    // Timer
    private ElapsedTime shootTimer = new ElapsedTime();

    // Hood Presets
    private final double CLOSE_HOOD = 0.22;
    private final double MID_HOOD = 0.34;
    private final double FAR_HOOD = 0.47;

    // Shooting State Machine
    private enum ShootState {
        IDLE,
        MOVE_TO_SHOOT,
        WAIT_FOR_SERVO,
        WAIT_FOR_FLYWHEEL,
        KICK_FORWARD,
        KICK_BACK,
        NEXT_BALL,
        DONE
    }

    private ShootState shootState = ShootState.IDLE;

    // Button Memory
    private boolean lastY = false;
    private boolean lastShoot = false;


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

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        TurretShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addLine("Init Complete");
        TurretHood.scaleRange(0.0, 0.9);

        // Configure Motor Directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        TurretShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        TurretHood.setDirection(Servo.Direction.REVERSE);

        // Set Zero Power Behaviors
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

        // SECTION 2: Adjusted Init Placements
        slot = 0;
        ballsLoaded = 0;

        spindexer.setPosition(intakePos[0]);
        Kicker.setPosition(0.15);

        HoodAngle = MID_HOOD;
        TurretHood.setPosition(HoodAngle);
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
        if (gamepad1.right_bumper) {
            intake1.setPower(0.6);  // Intake fully inward
        } else if (gamepad1.left_bumper) {
            intake1.setPower(-0.6); // Outtake/Spit out fully
        } else {
            intake1.setPower(0.0);  // Stop spin when no button is held
        }


        //----------SECTION 3: Intake & Flywheel Controls-------------------
        // Flywheel Toggle
        if (gamepad1.x && !lastX) {
            shooterEnabled = !shooterEnabled;
        }
        lastX = gamepad1.x;

        if (shooterEnabled) {
            TurretShooter.setVelocity(TARGET_VELOCITY);
        } else {
            TurretShooter.setVelocity(0);
        }

        // Intake Button (loads one ball sequentially)
        if (gamepad1.y && !lastY) {
            if (ballsLoaded < 3) {
                slot = ballsLoaded;
                spindexer.setPosition(intakePos[slot]);
                ballsLoaded++;
            }
        }
        lastY = gamepad1.y;


        //----------SECTION 4: Automatic Shooting State Machine-------------------
        // Start automatic shooting sequence
        if (gamepad1.b && !lastShoot) {
            if (ballsLoaded > 0 && shootState == ShootState.IDLE) {
                // Modified architecture: Sync slot pointer to track targeting positions safely
                slot = 3 - ballsLoaded;
                shootState = ShootState.MOVE_TO_SHOOT;
            }
        }
        lastShoot = gamepad1.b;

        // State Machine execution logic
        switch (shootState) {
            case MOVE_TO_SHOOT:
                spindexer.setPosition(shootPos[slot]);
                shootTimer.reset();
                shootState = ShootState.WAIT_FOR_SERVO;
                break;

            case WAIT_FOR_SERVO:
                if (shootTimer.milliseconds() > 200) {
                    shootState = ShootState.WAIT_FOR_FLYWHEEL;
                }
                break;

            case WAIT_FOR_FLYWHEEL:
                if (Math.abs(TurretShooter.getVelocity() - TARGET_VELOCITY) < 50) {
                    Kicker.setPosition(0.50);
                    shootTimer.reset();
                    shootState = ShootState.KICK_FORWARD;
                }
                break;

            case KICK_FORWARD:
                if (shootTimer.milliseconds() > 150) {
                    Kicker.setPosition(0.15);
                    shootTimer.reset();
                    shootState = ShootState.KICK_BACK;
                }
                break;

            case KICK_BACK:
                if (shootTimer.milliseconds() > 150) {
                    shootState = ShootState.NEXT_BALL;
                }
                break;

            case NEXT_BALL:
                ballsLoaded--;
                if (ballsLoaded <= 0) {
                    shootState = ShootState.DONE;
                } else {
                    // Update index based on actual chamber location progression
                    slot = (slot + 1) % 3;
                    shootState = ShootState.MOVE_TO_SHOOT;
                }
                break;

            case DONE:
                slot = 0;
                spindexer.setPosition(intakePos[0]);
                shootState = ShootState.IDLE;
                break;

            case IDLE:
            default:
                // Do nothing while waiting for automated button calls
                break;
        }


        //----------SECTION 5: Hood & Telemetry Controls-------------------
        if (gamepad1.dpad_down) {
            HoodAngle = CLOSE_HOOD;
        }
        if (gamepad1.dpad_left) {
            HoodAngle = MID_HOOD;
        }
        if (gamepad1.dpad_up) {
            HoodAngle = FAR_HOOD;
        }
        TurretHood.setPosition(HoodAngle);

        // Complete Telemetry Update block
        telemetry.addData("Balls Loaded", ballsLoaded);
        telemetry.addData("Current Slot", slot);
        telemetry.addData("Shoot State", shootState);
        telemetry.addData("Target Velocity", TARGET_VELOCITY);
        telemetry.addData("Current Velocity", TurretShooter.getVelocity());
        telemetry.addData("Hood Angle", HoodAngle);
        telemetry.update();
    }

    // Retained manual tracking math functions in case they are needed for custom configurations
    private void HoodAngleDecrease(double amount) {
        HoodAngle -= amount;
    }

    private void HoodAngleIncrease(double amount) {
        HoodAngle += amount;
    }
}