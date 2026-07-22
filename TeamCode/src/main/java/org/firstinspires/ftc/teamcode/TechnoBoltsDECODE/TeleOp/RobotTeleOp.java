package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TeleOp;

import static android.os.SystemClock.sleep;

import android.graphics.Color; // Added for color conversion
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms.TBWebcam.AlignToAprilTagTurret;


@TeleOp
public class RobotTeleOp extends OpMode {


    private Limelight3A limelight;

    private AlignToAprilTagTurret turret = new AlignToAprilTagTurret();
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    public DcMotorEx TurretShooter;

    double distance = 0;

    // -------- Color Sensor --------
    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    private int ballCount = 0;

    private boolean manualScanRequested = false;
    private int scannedBalls = 0;

    private enum IntakeState {
        WAIT_FOR_BALL,
        VERIFY_BALL,
        WAIT_FOR_CLEAR
    }

    private IntakeState intakeState = IntakeState.WAIT_FOR_BALL;

    private final ElapsedTime ballTimer = new ElapsedTime();

    private static final double BALL_DISTANCE = 35.0;
    private static final double VERIFY_TIME = 150.0;

    public double getDistance(double ty){
        double angleTarget = CAMERA_ANGLE + ty;
        double heightDifference = GOAL_HEIGHT - CAMERA_HEIGHT_IN;

        return heightDifference / Math.tan(Math.toRadians(angleTarget));
    }

    private double CAMERA_HEIGHT_IN = 31.11500;
    private double GOAL_HEIGHT = 74.95;
    private double CAMERA_ANGLE = 18;

    // Intake Motor (Expansion Hub 2, Port 2)

    private Servo spindexer;
    private Servo Kicker;
    private Servo TurretHood;

    private DcMotorEx intake1;

    private int slot = 0;

    double F = 1.2;

    double P = 350;

    double HoodAngle = 0;

    // YOUR TUNED POSITIONS
    private final double[] intakePos = {
            0.02,
            0.39,
            0.76
    };

    private final double[] shootPos = {
            0.19,
            0.57,
            0.95
    };

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastRB = false;

    // ---- NEW COLOR SENSOR VARIABLES ----
    private boolean autoIntakeEnabled = true;

    public enum DetectedColor {
        GREEN, PURPLE, BLUE, WHITE, BLACK, RED, ORANGE, YELLOW, UNKNOWN
    }
    // ------------------------------------

    // ---------------- Auto Shoot ----------------

    private boolean autoShoot = false;

    private boolean autoIndexMode = false;

    private int autoState = 0;

    private final ElapsedTime autoTimer = new ElapsedTime();

    boolean tracking = true;

    private final double KICKER_REST = 0.15;
    private final double KICKER_FIRE = 0.50;

    private int autoSlot = 0;

    private int LEFT_LIMIT = -1250;
    private int RIGHT_LIMIT = 1250;

    // Maximum manual speed
    private static final double MAX_POWER = 0.35;

    private double SHOOT_SPEED = turretSpeed(distance);

    //y=0.0969498x^{2}-4.97872x+1525.00813
    public double turretSpeed (double goalDist ){
//        return  MathFunctions.clamp(- 0.00000569338 * Math.pow(goalDist, 4)  +0.00246149 * Math.pow(goalDist, 3) -0.375414 * Math.pow(goalDist, 2) +24.62591 * (goalDist) +179.82739, 750, 900) - 40;
        return MathFunctions.clamp((0.0969498 * Math.pow(goalDist, 2)) - (4.97872 * goalDist) + 1550.00813, 1500, 3000);
    }
    //y=-\left(2.03775\times10^{-7}\right)x^{4}+0.0000739553x^{3}-0.00984175x^{2}+0.57947x-12.29583
    public double hoodAngle (double goalDist ){
        return  MathFunctions.clamp((- 0.000000203775 * Math.pow(goalDist, 4))  + (0.0000739553 * Math.pow(goalDist, 3)) - (0.00984175 * Math.pow(goalDist, 2)) + (0.57947 * (goalDist)) - 12.29583, 0, 1);
    }

    @Override
    public void init() {

        // Map motors to the names configured on the Driver Station
        frontLeft = hardwareMap.get(DcMotor.class, "front-left");
        frontRight = hardwareMap.get(DcMotor.class, "front-right");
        backLeft = hardwareMap.get(DcMotor.class, "back-left");
        backRight = hardwareMap.get(DcMotor.class, "back-right");
        spindexer = hardwareMap.get(Servo.class, "spindexerServo");
        TurretHood = hardwareMap.get(Servo.class, "hoodShooter");
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");
        Kicker = hardwareMap.get(Servo.class, "kickerServo");
        TurretShooter = hardwareMap.get(DcMotorEx.class, "turretShooter");
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0,0,F);
        TurretShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        telemetry.addLine("Init Complete");
        turret.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

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
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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

        // ---- NEW COLOR SENSOR INIT ----
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor_1");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color_sensor_1");
        if (colorSensor != null) {
            colorSensor.setGain(15.0f);
        }

        slot = 0;

        spindexer.setPosition(intakePos[0]);


        // -------------------------------
    }


    @Override
    public void loop() {


        //----------Drivetrain-------------------

        double y   =  -gamepad1.left_stick_y; // Inverted because joysticks are negative when pushed up
        double x   =  gamepad1.left_stick_x; // Strafe
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
        if (gamepad2.right_bumper) {
            intake1.setPower(0.6);  // Intake fully inward
        } else if (gamepad2.left_bumper) {
            intake1.setPower(-0.6); // Outtake/Spit out fully
        } else {
            intake1.setPower(0.0);  // Stop spin when no button is held
        }

        //----------Spindexer-------------------
        if (gamepad2.a && !lastA) {
            spindexer.setPosition(intakePos[slot]);
        }

        if (gamepad2.b && !lastB) {
            spindexer.setPosition(shootPos[slot]);
        }

        if (gamepad2.yWasPressed() && !lastRB) {

            slot = (slot + 1) % 3;

            // ALWAYS return to intake after indexing
            spindexer.setPosition(intakePos[slot]);
        }

        // save buttons
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastRB = gamepad1.yWasPressed();

        //----------Kicker-------------------
        if (!autoShoot) {
            if (gamepad2.dpad_up) {
                Kicker.setPosition(0.5);
            } else {
                Kicker.setPosition(0.15);
            }
        }

        //----------Turret-------------------


        //        double curVelocity2 = ShooterLeft.getVelocity();


        if(gamepad2.dpadRightWasPressed()){
            HoodAngleIncrease(0.1);
        }
        if(gamepad2.dpadLeftWasPressed()){
            HoodAngleDecrease(0.1);
        }

        TurretHood.setPosition(HoodAngle);




        //----------Tracking--------------

        if(gamepad2.aWasPressed()){
            tracking = false;
        }
        if(gamepad2.xWasPressed()){
            tracking = true;
        }



        if(tracking) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {


                // Horizontal angle from crosshair to tag
                double tx = result.getTx();

                turret.update(tx);

                telemetry.addData("tx", tx);
            } else {

                turret.stop();

                telemetry.addLine("No AprilTag");
            }
        }
        else if(!tracking){
            double turretRotate = gamepad2.right_stick_x;
            turret.getTurret().setPower(turretRotate);
        }

        //---------turret speed-------------
        if (!autoShoot) {
            if(distance != 0) {
                TurretShooter.setVelocity(turretSpeed(distance));
            }
            else{
                TurretShooter.setVelocity(1525);
            }
        }



        //-------hood angle-------
        HoodAngle = hoodAngle(distance);
        TurretHood.setPosition(HoodAngle);


        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()){
            distance = getDistance(llResult.getTy()) / 2.54;
            telemetry.addData("Distance", distance);
        }
        else {
            telemetry.addData("No Valid Target", "Found");
        }
        double curVelocity1 = TurretShooter.getVelocity();

//        double errorLeft = curTargetVelocity - curVelocity2;
        double error = turretSpeed(distance) - curVelocity1;

        // Manual recount button

        if (gamepad2.dpadDownWasPressed() && !autoShoot) {

            // First, count how many balls are actually loaded
            autoShoot = true;
            autoState = 0;
            SHOOT_SPEED = turretSpeed(distance);
            autoTimer.reset();

        }

//Manual override for positions 1 and 2
        //position 1


        telemetry.addLine("------------TurretShooter------");
        telemetry.addData("Target Velocity", turretSpeed(distance));
        telemetry.addData("Current Velocity", "%.2f", curVelocity1);
        telemetry.addData("Error Right",  "%.2f", error);
        telemetry.addLine("------------Hood---------------");
        telemetry.addData("Hood Angle",  "%.2f", HoodAngle);
        telemetry.addLine("------------Spindexer-----------");
        telemetry.addData("Slot", slot);
        telemetry.addData("Intake", intakePos[slot]);
        telemetry.addData("Shoot", shootPos[slot]);
        telemetry.addLine("-----------Autoshoot-----------");
        telemetry.addData("Auto Shoot", autoShoot);
        telemetry.addData("Auto State", autoState);
        telemetry.addLine("-----------Timer---------------");
        telemetry.addData("Timer", autoTimer.milliseconds());

        runAutoShoot(autoShoot);

        if(!autoShoot) {
            AutoIndexer();
        }

        telemetry.update();

    }

    public void HoodAngleDecrease(double amount) {
        HoodAngle -= amount;
    }

    public void HoodAngleIncrease(double amount) {
        HoodAngle += amount;
    }

    public void AutoIndexer() {

        if (autoShoot){
            ballCount = 0;
        }



        if (ballCount >= 4) {
            return;
        }

        double dist = distanceSensor.getDistance(DistanceUnit.MM);

        boolean ballDetected = dist < BALL_DISTANCE;

        switch (intakeState) {

            case WAIT_FOR_BALL:

                if (ballDetected) {
                    ballTimer.reset();
                    intakeState = IntakeState.VERIFY_BALL;
                }

                break;

            case VERIFY_BALL:

                if (!ballDetected) {

                    intakeState = IntakeState.WAIT_FOR_BALL;

                } else if (ballTimer.milliseconds() > VERIFY_TIME) {

                    ballCount++;

                    if (ballCount < 3) {
                        slot = Math.min(slot + 1, 2);
                        spindexer.setPosition(intakePos[slot]);
                    }

                    intakeState = IntakeState.WAIT_FOR_CLEAR;
                }

                break;

            case WAIT_FOR_CLEAR:

                if (!ballDetected) {
                    intakeState = IntakeState.WAIT_FOR_BALL;
                }

                break;
        }

        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Ball Distance", dist);
        telemetry.addData("Ball Detected", ballDetected);
        telemetry.addData("Intake State", intakeState);
    }

    public void runAutoShoot(boolean autoShootVar) {
//        int autoState =0;

        if (!autoShootVar) return;

        telemetry.addData("== Auto State ==", autoState);

        switch (autoState) {

                case 0:

                    TurretShooter.setVelocity(SHOOT_SPEED);

                    if (Math.abs(TurretShooter.getVelocity() - SHOOT_SPEED) < 100) {

                        spindexer.setPosition(shootPos[0]);
                        autoTimer.reset();
                        autoState = 1;
                    }

                    break;

                //==========================
                // Wait for spindexer
                //==========================
                case 1:

                    if (autoTimer.milliseconds() > 750) {

                        Kicker.setPosition(KICKER_FIRE);

                        autoTimer.reset();
                        autoState = 2;
                    }

                    break;

                //==========================
                // Hold kicker out
                //==========================
                case 2:

                    if (autoTimer.milliseconds() > 750) {

                        Kicker.setPosition(KICKER_REST);

                        autoTimer.reset();
                        autoState = 3;
                    }

                    break;

                //==========================
                // Wait for kicker to retract
                //==========================
                case 3:

                    if (autoTimer.milliseconds() > 750 && Math.abs(TurretShooter.getVelocity() - SHOOT_SPEED) < 100) {

                        spindexer.setPosition(shootPos[1]);

                        autoTimer.reset();
                        autoState = 4;
                    }

                    break;

                //==========================
                // Wait for second position
                //==========================
                case 4:

                    if (autoTimer.milliseconds() > 750) {

                        Kicker.setPosition(KICKER_FIRE);

                        autoTimer.reset();
                        autoState = 5;
                    }

                    break;

                //==========================
                // Hold kicker
                //==========================
                case 5:

                    if (autoTimer.milliseconds() > 750) {

                        Kicker.setPosition(KICKER_REST);

                        autoTimer.reset();
                        autoState = 6;
                    }

                    break;

                //==========================
                // Wait for kicker before moving
                //==========================
                case 6:

                    if (autoTimer.milliseconds() > 750 && Math.abs(TurretShooter.getVelocity() - SHOOT_SPEED) < 100) {

                        spindexer.setPosition(shootPos[2]);

                        autoTimer.reset();
                        autoState = 7;
                    }

                    break;

                //==========================
                // Wait for third position
                //==========================
                case 7:

                    if (autoTimer.milliseconds() > 750) {

                        Kicker.setPosition(KICKER_FIRE);

                        autoTimer.reset();
                        autoState = 8;
                    }

                    break;

                //==========================
                // Hold kicker
                //==========================
                case 8:

                    if (autoTimer.milliseconds() > 750) {

                        Kicker.setPosition(KICKER_REST);

                        autoTimer.reset();
                        autoState = 9;
                    }

                    break;

                //==========================
                // Wait for kicker to retract
                //==========================
                case 9:

                    if (autoTimer.milliseconds() > 750) {

                        spindexer.setPosition(0);

                        autoTimer.reset();
                        autoState = 10;
                    }

                    break;

                //==========================
                // Finish
                //==========================
                case 10:

                    if (autoTimer.milliseconds() > 750) {

                        TurretShooter.setVelocity(turretSpeed(distance));

                        slot = 0;
                        ballCount = 0;
                        intakeState = IntakeState.WAIT_FOR_BALL;

                        spindexer.setPosition(intakePos[0]);
                        autoState = 0;
                        autoShoot = false;

                        telemetry.addLine("---> Setting up autoState to 0 and autoShoot to false");
                        sleep(5000);
                    }

                    break;

                default:
                    break;
        }

    }



    @Override
    public void stop(){
        limelight.stop();
    }

}