package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Auto;

import static android.os.SystemClock.sleep;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TeleOp.RobotTeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "Auto Selector", group = "Autonomous")
public class AutoSelector extends OpMode {


    // Enums for selection
    private Limelight3A limelight;
    enum Alliance { BLUE, RED }
    enum StartPos { A, B, C }

    // Constants for shooter PIDF
    double F = 1.2;
    double P = 350;

    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;
    double distance = 0;
    private int slot = 0;
    private int ballCount = 0;
    private double SHOOT_SPEED = turretSpeed(distance);
    private final double KICKER_REST = 0.15;
    private final double KICKER_FIRE = 0.50;
    private final ElapsedTime autoTimer = new ElapsedTime();

    private double CAMERA_HEIGHT_IN = 31.11500;
    private double GOAL_HEIGHT = 74.95;
    private double CAMERA_ANGLE = 18;

    private static final double BALL_DISTANCE = 35.0;
    private final ElapsedTime ballTimer = new ElapsedTime();
    private static final double VERIFY_TIME = 150.0;

    private enum IntakeState {
        WAIT_FOR_BALL,
        VERIFY_BALL,
        WAIT_FOR_CLEAR
    }

    private IntakeState intakeState = IntakeState.WAIT_FOR_BALL;
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


    private int autoState =0;

    //y=0.0969498x^{2}-4.97872x+1525.00813
    public double turretSpeed (double goalDist ){
//        return  MathFunctions.clamp(- 0.00000569338 * Math.pow(goalDist, 4)  +0.00246149 * Math.pow(goalDist, 3) -0.375414 * Math.pow(goalDist, 2) +24.62591 * (goalDist) +179.82739, 750, 900) - 40;
        return MathFunctions.clamp((0.0969498 * Math.pow(goalDist, 2)) - (4.97872 * goalDist) + 1625.00813, 1500, 3000);
    }

    // Selection variables with default values
    Alliance alliance = Alliance.RED;
    StartPos startPos = StartPos.A;

    // Button state variable for debouncing (prevents crazy fast toggling)
    boolean lastGamepad2X = false;

    // Followers and hardware
    Follower follower;
    public DcMotor intake;
    public DcMotorEx turretShooter, turret;
    public Servo spindexer, kicker, turretHood;

    // Sub-auto instances for all 6 paths
    AutoRedA redAutoA;
    AutoRedB redAutoB;
    AutoRedC redAutoC;

    AutoBlueA blueAutoA;
    AutoBlueB blueAutoB;
    AutoBlueC blueAutoC;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Initialize motors
        intake = hardwareMap.get(DcMotor.class, "intake1");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        turretShooter = hardwareMap.get(DcMotorEx.class, "turretShooter");
        turretShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // AprilTag pipeline
        limelight.start();

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor_1");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "color_sensor_1");
        if (colorSensor != null) {
            colorSensor.setGain(15.0f);
        }

        turret = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        if(turret == null)
            telemetry.addLine("Turret object is null");


        PIDFCoefficients pidfCoefficientsRight = new PIDFCoefficients(P, 0, 0, F);
        turretShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficientsRight);

        // Initialize servos
        kicker = hardwareMap.get(Servo.class, "kickerServo");
        spindexer = hardwareMap.get(Servo.class, "spindexerServo");
        turretHood = hardwareMap.get(Servo.class, "hoodShooter");

        // Instantiate all 6 auto classes with the follower and hardware dependencies
        redAutoA = new AutoRedA(this, follower, telemetry, intake, turretShooter, kicker, spindexer, turretHood, limelight, colorSensor, distanceSensor, turret);
        redAutoB = new AutoRedB(follower, telemetry, intake, turretShooter, kicker, spindexer, turretHood, limelight, colorSensor, distanceSensor, turret);
        redAutoC = new AutoRedC(follower, telemetry, intake, turretShooter, kicker, spindexer, turretHood, limelight, colorSensor, distanceSensor, turret);

        blueAutoA = new AutoBlueA(follower, telemetry, intake, turretShooter, kicker, spindexer, turretHood, limelight, colorSensor, distanceSensor, turret);
        blueAutoB = new AutoBlueB(follower, telemetry, intake, turretShooter, kicker, spindexer, turretHood, limelight, colorSensor, distanceSensor, turret);
        blueAutoC = new AutoBlueC(follower, telemetry, intake, turretShooter, kicker, spindexer, turretHood, limelight, colorSensor, distanceSensor, turret);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("=== AUTO SELECTION ===");
        telemetry.addLine("Press X to TOGGLE Alliance (Red/Blue)");
        telemetry.addLine("Press A, B, or Y to choose Position (A, B, C)");
        telemetry.addLine("---------------------------------------------");

        // Alliance Toggle Logic (Registers only once per complete press)
        if (gamepad2.x && !lastGamepad2X) {
            if (alliance == Alliance.RED) {
                alliance = Alliance.BLUE;
            } else {
                alliance = Alliance.RED;
            }
        }
        lastGamepad2X = gamepad2.x; // Update last button state

        // Position Selection Input
        if (gamepad2.a) startPos = StartPos.A;
        if (gamepad2.b) startPos = StartPos.B;
        if (gamepad2.y) startPos = StartPos.C;

        // Display selection telemetry
        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Position", startPos);
        telemetry.addLine("---------------------------------------------");
        telemetry.update();
    }

    @Override
    public void start() {
        // Starts the path state machine inside the selected auto class
        if (alliance == Alliance.RED) {
            if (startPos == StartPos.A) {redAutoA.start();}
            else if (startPos == StartPos.B) {redAutoB.start();}
            else if (startPos == StartPos.C) {redAutoC.start();}
        } else { // BLUE
            if (startPos == StartPos.A) {blueAutoA.start();}
            else if (startPos == StartPos.B) {blueAutoB.start();}
            else if (startPos == StartPos.C) {blueAutoC.start();}
        }
    }

    @Override
    public void loop() {

        double curVelocity1 = turretShooter.getVelocity();
        double error = turretSpeed(distance) - curVelocity1;

        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            distance = getDistance(llResult.getTy()) / 2.54;
            telemetry.addData("Distance", distance);
        }
        else {
            telemetry.addData("No Valid Target", "Found");
        }

        // Continuously runs the update method for the active path
        if (alliance == Alliance.RED) {
            if (startPos == StartPos.A) {
                redAutoA.update();
                telemetry.addData("Path Timer",redAutoA.pathTimer.getElapsedTimeSeconds());
                telemetry.addData("== Auto State ==", autoState);
                telemetry.addData("Target Velocity", turretSpeed(distance));
                telemetry.addData("Current Velocity", "%.2f", curVelocity1);
                telemetry.addData("Error",  "%.2f", error);
            }
            else if (startPos == StartPos.B) {
                redAutoB.update();
                telemetry.addData("Path Timer",redAutoB.pathTimer.getElapsedTimeSeconds());
                telemetry.addData("== Auto State ==", autoState);
                telemetry.addData("Target Velocity", turretSpeed(distance));
                telemetry.addData("Current Velocity", "%.2f", curVelocity1);
                telemetry.addData("Error",  "%.2f", error);
            }
            else if (startPos == StartPos.C) {
                redAutoC.update();
                telemetry.addData("Path Timer",redAutoC.pathTimer.getElapsedTimeSeconds());
                telemetry.addData("== Auto State ==", autoState);
                telemetry.addData("Target Velocity", turretSpeed(distance));
                telemetry.addData("Current Velocity", "%.2f", curVelocity1);
                telemetry.addData("Error",  "%.2f", error);
            }
        } else { // BLUE
            if (startPos == StartPos.A) {
                blueAutoA.update();
                telemetry.addData("Path Timer",blueAutoA.pathTimer.getElapsedTimeSeconds());
                telemetry.addData("== Auto State ==", autoState);
                telemetry.addData("Target Velocity", turretSpeed(distance));
                telemetry.addData("Current Velocity", "%.2f", curVelocity1);
                telemetry.addData("Error",  "%.2f", error);
            }
            else if (startPos == StartPos.B) {
                blueAutoB.update();
                telemetry.addData("Path Timer",blueAutoB.pathTimer.getElapsedTimeSeconds());
                telemetry.addData("== Auto State ==", autoState);
                telemetry.addData("Target Velocity", turretSpeed(distance));
                telemetry.addData("Current Velocity", "%.2f", curVelocity1);
                telemetry.addData("Error",  "%.2f", error);
            }
            else if (startPos == StartPos.C) {
                blueAutoC.update();
                telemetry.addData("Path Timer",blueAutoC.pathTimer.getElapsedTimeSeconds());
                telemetry.addData("== Auto State ==", autoState);
                telemetry.addData("Target Velocity", turretSpeed(distance));
                telemetry.addData("Current Velocity", "%.2f", curVelocity1);
                telemetry.addData("Error",  "%.2f", error);
            }
        }

        // Live coordinate feedback on the Driver Station
        telemetry.addData("X Position", follower.getPose().getX());
        telemetry.addData("Y Position", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }

    public boolean runAutoShoot(boolean autoShoot, DcMotorEx turretShooter, Servo spindexer, Servo kicker) {
        //int autoState =0;
        if (!autoShoot) return false;

        SHOOT_SPEED = turretSpeed(distance);

        switch (autoState) {

            case 0:
                turretShooter.setVelocity(SHOOT_SPEED);
                sleep(750);
                if (Math.abs(turretShooter.getVelocity() - SHOOT_SPEED) < 100) {
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

                    kicker.setPosition(KICKER_FIRE);

                    autoTimer.reset();
                    autoState = 2;
                }

                break;

            //==========================
            // Hold kicker out
            //==========================
            case 2:

                if (autoTimer.milliseconds() > 750) {

                    kicker.setPosition(KICKER_REST);

                    autoTimer.reset();
                    autoState = 3;
                }

                break;

            //==========================
            // Wait for kicker to retract
            //==========================
            case 3:

                if (autoTimer.milliseconds() > 750 && Math.abs(turretShooter.getVelocity() - SHOOT_SPEED) < 100) {

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

                    kicker.setPosition(KICKER_FIRE);

                    autoTimer.reset();
                    autoState = 5;
                }

                break;

            //==========================
            // Hold kicker
            //==========================
            case 5:

                if (autoTimer.milliseconds() > 750) {

                    kicker.setPosition(KICKER_REST);

                    autoTimer.reset();
                    autoState = 6;
                }

                break;

            //==========================
            // Wait for kicker before moving
            //==========================
            case 6:

                if (autoTimer.milliseconds() > 750 && Math.abs(turretShooter.getVelocity() - SHOOT_SPEED) < 100) {

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

                    kicker.setPosition(KICKER_FIRE);

                    autoTimer.reset();
                    autoState = 8;
                }

                break;

            //==========================
            // Hold kicker
            //==========================
            case 8:

                if (autoTimer.milliseconds() > 750) {

                    kicker.setPosition(KICKER_REST);

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

                    turretShooter.setVelocity(0);

                    slot = 0;
                    ballCount = 0;
                    intakeState = IntakeState.WAIT_FOR_BALL;

                    spindexer.setPosition(intakePos[0]);
                    autoState = 0;
                    return true;
                }

                break;

            default:
                break;
        }
        return false;

    }
    public void AutoIndexer(boolean autoShoot) {

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

    public double getDistance(double ty){
        double angleTarget = CAMERA_ANGLE + ty;
        double heightDifference = GOAL_HEIGHT - CAMERA_HEIGHT_IN;

        return heightDifference / Math.tan(Math.toRadians(angleTarget));
    }
}