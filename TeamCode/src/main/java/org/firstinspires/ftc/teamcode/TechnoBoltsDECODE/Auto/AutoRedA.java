package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class AutoRedA {

    private Follower follower;
    private Timer pathTimer, opModeTimer;


    public final DcMotor intake;
    public final DcMotorEx turretShooter;
    public final Servo Kicker;
    public final Servo Spindexer;
    public final Servo turretHood;


    private final double ShooterOn = 760;
    private final double leftPowerOff = 0;
    private final double rightPowerOff = 0;
    private final double lowerRampOn = -0.5;
    private final double middleRampOn = 0.5;
    private final double lowerRampSlow = 0.2;
    private final double middleRampSlow = 0.2;
    private final double intakePowerOn = -0.6;
    private final double intakeHalfPower = -0.4;
    private final double intakePowerOff = 0;
    private final double kickerStopPower = -0.7;
    private final double kickerLaunchPower = 1;
    private final double kickerHalfLaunchPower = 0.3;
    // Define the motor at the top of your OpMode class




    Telemetry telemetry;

    public AutoRedA(Follower follower, Telemetry telemetry, DcMotor intake, DcMotorEx turretShooter, Servo Kicker, Servo Spindexer, Servo turretHood) {

        this.follower = follower;
        this.telemetry = telemetry;
        this.intake = intake;
        this.turretShooter = turretShooter;
        this.Kicker = Kicker;
        this.Spindexer = Spindexer;
        this.turretHood = turretHood;


        pathTimer = new Timer();
    }

    public void start () {
//        opModeTimer.resetTimer();

        follower.setPose(startPose);
        buildPaths();

        pathState = PathState.STARTPOS_SHOOTPOS;
        setPathState(pathState);

        pathTimer = new Timer();
        opModeTimer = new Timer();
    }

    public void update() {
        follower.update();
        autonomousPathUpdate();
    }

    public void setPathState (PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    public void doIntakePowerOn() {
        intake.setPower(intakePowerOn);
    }

    public void doIntakePowerOff() {
        intake.setPower(intakePowerOff);
    }



    public enum PathState {
        // START POSITION --> END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE
        STARTPOS_SHOOTPOS, //From start to shoot position
        SHOOTPOS_INTAKE1, //From shoot position to intake preset 1
        INTAKE1_SHOOT1POS, //From intake preset 1 to shoot preset 1
        SHOOT1POS_ALIGN3, //From shoot preset 1 to align preset 3
        ALIGN3_INTAKE3, //From align preset 3 to intake preset 3
        INTAKE3_SHOOT3, //From intake preset 3 to shoot preset 3
        SHOOT3_LEAVE, //From shoot preset 3 to leave zone
        IDLE //Done with code
    }

   PathState pathState;

   private final Pose startPose = new Pose(124.7999988888889, 124.25887777777776, Math.toRadians(225));
   private final Pose shootPose = new Pose(92.8, 84.219999999999999, Math.toRadians(360));
   private final Pose IntakePreset1Pose = new Pose(122.95555555555556, 84.219999999999999, Math.toRadians(360));
   private final Pose ShootPreset1Pose = new Pose(92.8, 84.219999999999999, Math.toRadians(290));
   private final Pose AlignToPreset3Pose = new Pose(104.46666666666667, 35.6888888888889, Math.toRadians(360));
        private final Pose AlignToPreset3PoseControl = new Pose(95.41111111111111, 38.33333333333323, Math.toRadians(360));
   private final Pose IntakePreset3Pose = new Pose(123, 35.46666666666667, Math.toRadians(360));
   private final Pose ShootPreset3Pose = new Pose(92.8, 84.219999999999999, Math.toRadians(290));
        private final Pose ShootPreset3PoseControl = new Pose(87.39524838012959, 55.04967602591793, Math.toRadians(290));
   private final Pose LeaveZonePose = new Pose(102.06493506493507, 76.22077922077922, Math.toRadians(290));



    private PathChain StartToShoot, ShootToIntake1, Intake1ToShoot1, Shoot1ToAlignPreset3, AlignPreset3ToIntake3, Intake3ToShoot3, Shoot3ToLeave;


        public void buildPaths () {
            // put in coordinates for starting pose > ending pose
            StartToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                    .build();

            ShootToIntake1 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, IntakePreset1Pose))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), IntakePreset1Pose.getHeading())
                    .build();

            Intake1ToShoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(IntakePreset1Pose, ShootPreset1Pose))
                    .setLinearHeadingInterpolation(IntakePreset1Pose.getHeading(), ShootPreset1Pose.getHeading())
                    .build();

            Shoot1ToAlignPreset3 = follower.pathBuilder()
                    .addPath(new BezierCurve(ShootPreset1Pose, AlignToPreset3PoseControl, AlignToPreset3Pose))
                    .setLinearHeadingInterpolation(ShootPreset1Pose.getHeading(), AlignToPreset3Pose.getHeading())
                    .build();

            AlignPreset3ToIntake3 = follower.pathBuilder()
                    .addPath(new BezierLine(AlignToPreset3Pose, IntakePreset3Pose))
                    .setLinearHeadingInterpolation(AlignToPreset3Pose.getHeading(), IntakePreset3Pose.getHeading())
                    .build();

            Intake3ToShoot3 = follower.pathBuilder()
                    .addPath(new BezierCurve(IntakePreset3Pose, ShootPreset3PoseControl, ShootPreset3Pose))
                    .setLinearHeadingInterpolation(IntakePreset3Pose.getHeading(), ShootPreset3Pose.getHeading())
                    .build();

            Shoot3ToLeave = follower.pathBuilder()
                    .addPath(new BezierLine(ShootPreset3Pose, LeaveZonePose))
                    .setLinearHeadingInterpolation(ShootPreset3Pose.getHeading(), LeaveZonePose.getHeading())
                    .build();
        }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case STARTPOS_SHOOTPOS:
                if (!follower.isBusy()) {
                    follower.followPath(StartToShoot, true);
                }
                // 2. ONLY transition once we have arrived and the follower is done
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(PathState.SHOOTPOS_INTAKE1); // Resets pathTimer and advances
                }
                break;

            case SHOOTPOS_INTAKE1:
                if (!follower.isBusy()) {
                    telemetry.addLine("Intake Preset 1");
                    telemetry.update();
                    follower.followPath(ShootToIntake1, 0.5, true);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    telemetry.addLine("Intake Preset 1 set path");
                    telemetry.update();
                    setPathState(PathState.INTAKE1_SHOOT1POS);
                }
                break;

            case INTAKE1_SHOOT1POS:
                if (!follower.isBusy()) {
                    follower.followPath(Intake1ToShoot1, true);
                    telemetry.addLine("Shoot Preset");
                    telemetry.update();
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(PathState.SHOOT1POS_ALIGN3);
                    telemetry.addLine("Shoot Preset 1 set path");
                    telemetry.update();
                }
                break;

            case SHOOT1POS_ALIGN3:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot1ToAlignPreset3, true);
                    telemetry.addLine("Aligning to Preset 3");
                    telemetry.update();
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(PathState.ALIGN3_INTAKE3);
                    telemetry.addLine("Aligning to Preset 3 Path state");
                    telemetry.update();
                }
                break;

            case ALIGN3_INTAKE3:
                if (!follower.isBusy()) {
                    follower.followPath(AlignPreset3ToIntake3, 0.5, true);
                    telemetry.addLine("Intake Preset 3");
                    telemetry.update();
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(PathState.INTAKE3_SHOOT3);
                }
                break;

            case INTAKE3_SHOOT3:
                if (!follower.isBusy()) {
                    follower.followPath(Intake3ToShoot3, true);
                    telemetry.addLine("Shoot Preset 3");
                    telemetry.update();
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(PathState.SHOOT3_LEAVE);
                }
                break;

            case SHOOT3_LEAVE:
                if (!follower.isBusy()) {
                    follower.followPath(Shoot3ToLeave, true);
                    telemetry.addLine("Leaving Zone");
                    telemetry.update();
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                telemetry.addLine("Autonomous Complete!");
                telemetry.update();
                break;

            default:
                telemetry.addLine("No State Commanded");
                telemetry.update();
                break;
        }
    }


//        @Override
//        public void init () {
//            pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
//            pathTimer = new Timer();
//            opModeTimer = new Timer();
//            follower = Constants.createFollower(hardwareMap);
//            //TODO add in any other init mechanisms
//
//            buildPaths();
//            follower.setPose(startPose);
//        }



//        @Override
//        public void loop () {
//
//            follower.update();
//            statePathUpdate();
//
//            telemetry.addData("path state", pathState.toString());
//            telemetry.addData("x", follower.getPose().getX());
//            telemetry.addData("y", follower.getPose().getY());
//            telemetry.addData("heading", follower.getPose().getHeading());
//            telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
//        }


}
