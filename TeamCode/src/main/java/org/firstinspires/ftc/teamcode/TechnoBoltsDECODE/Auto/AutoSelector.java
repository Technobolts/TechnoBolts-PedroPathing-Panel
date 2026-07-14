package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "Auto Selector")
public class AutoSelector extends OpMode {


    enum Alliance {BLUE, RED}

    enum StartPos {TOP, BOTTOM}

    enum Preset {ZERO, ONE, TWO, THREE}

    double FR = 12.22;
    double PR = 100.5;

    double FL = 12.62;
    double PL = 100.85;

    Alliance alliance = Alliance.BLUE;
    StartPos startPos = StartPos.BOTTOM;

    Preset preset = Preset.ZERO;

    Follower follower;

    public DcMotor intake;
    public DcMotorEx turretShooter;
    public Servo Spindexer, Kicker, turretHood;


    AutoTopRedA topRedAuto;   // the top red auto
    AutoTopBlue topBlueAuto;   // the top blue auto
    AutoBottomBlue bottomBlueAuto; // the bottom blue auto
    AutoBottomRed bottomRedAuto; // the bottom red auto


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake1");     // Hardware map names
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        turretShooter = hardwareMap.get(DcMotorEx.class, "turretShooter");
        turretShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficientsRight = new PIDFCoefficients(PR, 0,0,FR);
        PIDFCoefficients pidfCoefficientsLeft = new PIDFCoefficients(PL, 0,0,FL);
        turretShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsRight);
        Kicker = hardwareMap.get(Servo.class, "kickerServo");
        Spindexer = hardwareMap.get(Servo.class, "spindexerServo");
        turretHood = hardwareMap.get(Servo.class, "hoodShooter");


//        bottomBlueAuto = new AutoBottomBlue(follower, flip1, intake, launcher1, launcher2);
//        bottomRedAuto = new AutoBottomRed(follower, flip1, intake, launcher1, launcher2);
        topRedAuto = new AutoTopRedA(follower, telemetry, intake, turretShooter , Kicker, Spindexer, turretHood);
        topBlueAuto = new AutoTopBlue(follower, telemetry, intake, turretShooter , Kicker, Spindexer, turretHood);
        bottomBlueAuto = new AutoBottomBlue(follower, telemetry, intake, turretShooter, Kicker, Spindexer,turretHood);
        bottomRedAuto = new AutoBottomRed(follower, telemetry, intake, turretShooter, Kicker, Spindexer, turretHood);
    }

    @Override
    public void init_loop() {

        telemetry.addLine("gamepad2X = BLUE");
        telemetry.addLine("gamepad2B = RED");
        telemetry.addLine("gamepad2A = BOTTOM");
        telemetry.addLine("gamepad2Y = TOP");
        telemetry.addLine("-------------------");

        if (gamepad2.x) alliance = Alliance.BLUE;   // alliance and color selections
        if (gamepad2.b) alliance = Alliance.RED;

        if (gamepad2.a) startPos = StartPos.BOTTOM;
        if (gamepad2.y) startPos = StartPos.TOP;

        if(gamepad2.dpadUpWasPressed()) preset = Preset.ZERO;
        if(gamepad2.dpadLeftWasPressed()) preset = Preset.ONE;
        if(gamepad2.dpadRightWasPressed()) preset = Preset.TWO;
        if(gamepad2.dpadDownWasPressed()) preset = Preset.THREE;


        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start Position", startPos);
        telemetry.addData("Number of Preset", preset);
        telemetry.addLine("-------------------");
        telemetry.update();
    }

    @Override
    public void start() {
        if (alliance == Alliance.BLUE && startPos == StartPos.BOTTOM) {   // starts the selected auto
            bottomBlueAuto.start();
        }
        if (alliance == Alliance.RED && startPos == StartPos.BOTTOM) {
            bottomRedAuto.start();
        }
        if (alliance == Alliance.RED && startPos == StartPos.TOP) {
            topRedAuto.start();
            }
        if (alliance == Alliance.BLUE && startPos == StartPos.TOP) {
            topBlueAuto.start();
            }
        }

        @Override
        public void loop() {


        if (alliance == Alliance.BLUE && startPos == StartPos.BOTTOM) {  // updates the selected auto
                    bottomBlueAuto.update();
        }
        if (alliance == Alliance.RED && startPos == StartPos.BOTTOM) {
                    bottomRedAuto.update();
        }
        if (alliance == Alliance.RED && startPos == StartPos.TOP) {
                    topRedAuto.update();
        }
        if (alliance == Alliance.BLUE && startPos == StartPos.TOP) {
                    topBlueAuto.update();
        }

            telemetry.addData("Y", follower.getPose().getY());

            telemetry.addData("X", follower.getPose().getX());

            telemetry.addData("Heading", follower.getPose().getHeading());

            }
        }