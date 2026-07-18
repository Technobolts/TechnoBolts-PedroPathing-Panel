package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TeleOp;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
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

    public double getDistance(double ty){
        double angleTarget = CAMERA_ANGLE + ty;
        double heightDifference = GOAL_HEIGHT - CAMERA_HEIGHT_IN;

        return heightDifference / Math.tan(Math.toRadians(angleTarget));
    }

    private double CAMERA_HEIGHT_IN = 31.11500;
    private double GOAL_HEIGHT = 74.95;
    private double CAMERA_ANGLE = 18;

    // Intake Motor (Expansion Hub 2, Port 2)

    public DcMotorEx encoderX; // Parallel wheel (tracks forward/backward)
    public DcMotorEx encoderY; // Perpendicular wheel (tracks strafe/sideways)

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
            0,
            0.4,
            0.76
    };

    private final double[] shootPos = {
            0.22,
            0.59,
            0.96
    };

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastRB = false;

    // ---------------- Auto Shoot ----------------

    private boolean autoShoot = false;
    private int autoState = 0;

    private final ElapsedTime autoTimer = new ElapsedTime();

    private final double SHOOT_SPEED = 2100;

    private final double KICKER_REST = 0.15;
    private final double KICKER_FIRE = 0.50;

    private int autoSlot = 0;

    //y=0.0969498x^{2}-4.97872x+1525.00813
    public double turretSpeed (double goalDist ){
//        return  MathFunctions.clamp(- 0.00000569338 * Math.pow(goalDist, 4)  +0.00246149 * Math.pow(goalDist, 3) -0.375414 * Math.pow(goalDist, 2) +24.62591 * (goalDist) +179.82739, 750, 900) - 40;
        return MathFunctions.clamp((0.0969498 * Math.pow(goalDist, 2)) - (4.97872 * goalDist) + 1525.00813, 1500, 2200);
    }
//y=-\left(2.03775\times10^{-7}\right)x^{4}+0.0000739553x^{3}-0.00984175x^{2}+0.57947x-12.29583
    public double hoodAngle (double goalDist ){
        return  MathFunctions.clamp((- 0.000000203775 * Math.pow(goalDist, 4))  + (0.0000739553 * Math.pow(goalDist, 3)) - (0.00984175 * Math.pow(goalDist, 2)) + (0.57947 * (goalDist)) - 12.29583, 0, 1);
    }

    // ==========================================
    // 2. INITIALIZATION METHOD
    // ==========================================
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

        if(gamepad2.a && !lastA && !autoShoot){

            autoShoot = true;
            autoState = 0;
            autoSlot = 0;

            autoTimer.reset();

        }

        //--------TurretRotationLimit---------
        if (TurretShooter != null) {
            double TICKS_PER_DEGREE = 0.0778;
            double currentAngle = TurretShooter.getCurrentPosition() / TICKS_PER_DEGREE;

            double currentPower = TurretShooter.getPower();
            if (currentAngle >= 90.0 && currentPower > 0) {
                TurretShooter.setPower(0);
            } else if (currentAngle <= -90.0 && currentPower < 0) {
                TurretShooter.setPower(0);
            }
        }

        //----------Spindexer-------------------


        // =========================
        // INTAKE (instant snap)
        // =========================

        // =========================
        // SHOOT (instant snap)
        // =========================
        if (gamepad2.b && !lastB) {
            spindexer.setPosition(shootPos[slot]);
        }

        // =========================
        // CLOCKWISE ONLY INDEX
        // (important for stability)
        // =========================
        if (gamepad2.yWasPressed() && !lastRB) {

            slot = (slot + 1) % 3;

            // ALWAYS return to intake after indexing
            spindexer.setPosition(intakePos[slot]);
        }

        // save buttons
        lastA = gamepad2.a;
        lastB = gamepad2.b;
        lastRB = gamepad2.yWasPressed();

        //----------Kicker-------------------
        if (!autoShoot) {
            if (gamepad2.dpad_up) {
                Kicker.setPosition(0.5);
            } else {
                Kicker.setPosition(0.15);
            }
        }

        //----------Turret-------------------

        double curTargetVelocity = SHOOT_SPEED;

        if(!autoShoot){

            if(gamepad2.x){
                TurretShooter.setVelocity(curTargetVelocity);
            }
            else{
                TurretShooter.setVelocity(0);
            }

        }
        //        double curVelocity2 = ShooterLeft.getVelocity();
        double curVelocity1 = TurretShooter.getVelocity();

//        double errorLeft = curTargetVelocity - curVelocity2;
        double error = curTargetVelocity - curVelocity1;

        if(gamepad2.dpadRightWasPressed()){
            HoodAngleIncrease(0.1);
        }
        if(gamepad2.dpadLeftWasPressed()){
            HoodAngleDecrease(0.1);
        }

        TurretHood.setPosition(HoodAngle);

        runAutoShoot();


        //----------Tracking--------------

        LLResult result = limelight.getLatestResult();

        if(result != null && result.isValid()){

            // Horizontal angle from crosshair to tag
            double tx = result.getTx();

            turret.update(tx);

            telemetry.addData("tx", tx);
        }
        else{

            turret.stop();

            telemetry.addLine("No AprilTag");
        }

        //---------turret speed-------------
        TurretShooter.setVelocity(turretSpeed(distance));

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



        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity1);
        telemetry.addData("Error Right",  "%.2f", error);
        telemetry.addData("Hood Angle",  "%.2f", HoodAngle);
        telemetry.addData("Slot", slot);
        telemetry.addData("Intake", intakePos[slot]);
        telemetry.addData("Shoot", shootPos[slot]);
        telemetry.addData("Auto State", autoState);
        telemetry.addData("Timer", autoTimer.milliseconds());
        telemetry.update();

    }

    public void HoodAngleDecrease(double amount) {
        HoodAngle -= amount;
    }

    public void HoodAngleIncrease(double amount) {
        HoodAngle += amount;
    }

    public void runAutoShoot() {

        if (!autoShoot) return;

        switch (autoState) {

            //==========================
            // Spin up shooter & move to first ball
            //==========================
            case 0:

                TurretShooter.setVelocity(SHOOT_SPEED);

                if (Math.abs(TurretShooter.getVelocity() - SHOOT_SPEED) < 50) {

                    spindexer.setPosition(shootPos[0]);
                    autoTimer.reset();
                    autoState = 1;
                }

                break;

            //==========================
            // Wait for spindexer
            //==========================
            case 1:

                if (autoTimer.milliseconds() > 650) {

                    Kicker.setPosition(KICKER_FIRE);

                    autoTimer.reset();
                    autoState = 2;
                }

                break;

            //==========================
            // Hold kicker out
            //==========================
            case 2:

                if (autoTimer.milliseconds() > 380) {

                    Kicker.setPosition(KICKER_REST);

                    autoTimer.reset();
                    autoState = 3;
                }

                break;

            //==========================
            // Wait for kicker to retract
            //==========================
            case 3:

                if (autoTimer.milliseconds() > 650) {

                    spindexer.setPosition(shootPos[1]);

                    autoTimer.reset();
                    autoState = 4;
                }

                break;

            //==========================
            // Wait for second position
            //==========================
            case 4:

                if (autoTimer.milliseconds() > 650 && Math.abs(TurretShooter.getVelocity() - SHOOT_SPEED) < 50) {

                    Kicker.setPosition(KICKER_FIRE);

                    autoTimer.reset();
                    autoState = 5;
                }

                break;

            //==========================
            // Hold kicker
            //==========================
            case 5:

                if (autoTimer.milliseconds() > 380) {

                    Kicker.setPosition(KICKER_REST);

                    autoTimer.reset();
                    autoState = 6;
                }

                break;

            //==========================
            // Wait for kicker before moving
            //==========================
            case 6:

                if (autoTimer.milliseconds() > 450) {

                    spindexer.setPosition(shootPos[2]);

                    autoTimer.reset();
                    autoState = 7;
                }

                break;

            //==========================
            // Wait for third position
            //==========================
            case 7:

                if (autoTimer.milliseconds() > 650 && Math.abs(TurretShooter.getVelocity() - SHOOT_SPEED) < 50) {

                    Kicker.setPosition(KICKER_FIRE);

                    autoTimer.reset();
                    autoState = 8;
                }

                break;

            //==========================
            // Hold kicker
            //==========================
            case 8:

                if (autoTimer.milliseconds() > 380) {

                    Kicker.setPosition(KICKER_REST);

                    autoTimer.reset();
                    autoState = 9;
                }

                break;

            //==========================
            // Wait for kicker to retract
            //==========================
            case 9:

                if (autoTimer.milliseconds() > 450) {

                    spindexer.setPosition(intakePos[0]);

                    autoTimer.reset();
                    autoState = 10;
                }

                break;

            //==========================
            // Finish
            //==========================
            case 10:

                if (autoTimer.milliseconds() > 650) {

                    TurretShooter.setVelocity(0);

                    autoShoot = false;
                    autoState = 0;
                }

                break;
        }
    }

    @Override
    public void stop(){
        limelight.stop();
    }

}

