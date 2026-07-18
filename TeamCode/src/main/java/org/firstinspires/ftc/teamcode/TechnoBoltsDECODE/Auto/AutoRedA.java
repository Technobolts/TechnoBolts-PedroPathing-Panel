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
    public Timer pathTimer, opModeTimer;


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

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
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
        this.telemetry.addData("Path timer:", pathTimer.getElapsedTime());
       // this.telemetry.update();
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
        DRIVE_STARTPOS_SHOOT_POS, //From start to shoot position
        SHOOT_PRELOAD_INTAKE1, //Shoot preload
        SHOOT_INTAKE_PRESET1, //aligned preset to fully intake preset
        PRESET1_PRESET3 , //From shooting to preset 3
        INTAKE_PRESET3 , //Intaking preset 3
        SHOOT_PRESET3 , // Shooting preset 3
        SHOOT_PRESET3_PRESET2 , //From shooting preset 3 to preset 2
        DONE
    }

    PathState pathState;

    private final Pose startPose = new Pose(124.799, 124.258, Math.toRadians(225));
    private final Pose shootPose = new Pose(92.8, 84.219, Math.toRadians(360));
    private final Pose IntakePreset1Pose = new Pose(122.956, 84.2199, Math.toRadians(360));
    private final Pose ShootPreset1Pose = new Pose(92.8, 84.219, Math.toRadians(290));
    private final Pose AlignToPreset3Pose = new Pose(104.47, 35.69, Math.toRadians(360));
    private final Pose AlignToPreset3PoseControl = new Pose(95.411, 38.33, Math.toRadians(360));
    private final Pose IntakePreset3Pose = new Pose(123, 35.467, Math.toRadians(360));
    private final Pose ShootPreset3Pose = new Pose(92.8, 84.219, Math.toRadians(290));
    private final Pose ShootPreset3PoseControl = new Pose(87.39, 55.0493, Math.toRadians(290));
    private final Pose LeaveZonePose = new Pose(102.067, 76.22, Math.toRadians(290));





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

            case DRIVE_STARTPOS_SHOOT_POS:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 3) {
                    follower.followPath(StartToShoot, true);
                    setPathState(PathState.SHOOT_PRELOAD_INTAKE1);
                }
                break;
                // at the first line
            case SHOOT_PRELOAD_INTAKE1:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 3) {
                    follower.followPath(ShootToIntake1,  0.35,true);
                    setPathState(PathState.SHOOT_INTAKE_PRESET1);
                }
                break;
                // leave first line and shoot
            case SHOOT_INTAKE_PRESET1:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 3) {
                    follower.followPath(Intake1ToShoot1, true);
                    setPathState(PathState.PRESET1_PRESET3);
                }
                break;

            case PRESET1_PRESET3:
                if (!follower.isBusy() || pathTimer.getElapsedTime() > 3) {
                    follower.followPath(Shoot1ToAlignPreset3, true);
                    setPathState(PathState.INTAKE_PRESET3);
                }
                break;
                // ready to get intake 3
            case INTAKE_PRESET3:
                if (!follower.isBusy()|| pathTimer.getElapsedTime() > 3) {
                    follower.followPath(AlignPreset3ToIntake3, true);
                    setPathState(PathState.SHOOT_PRESET3);
                }
                break;

            case SHOOT_PRESET3:

                if (!follower.isBusy() || pathTimer.getElapsedTime() > 3) {
                    follower.followPath(Intake3ToShoot3,  true);
                    setPathState(PathState.SHOOT_PRESET3_PRESET2);
                }
                break;

            case SHOOT_PRESET3_PRESET2:

                if (!follower.isBusy() || pathTimer.getElapsedTime() > 3) {

                    follower.followPath(Shoot3ToLeave,  true);
                    setPathState(PathState.SHOOT_PRESET3);
                }
                break;

            default:
                telemetry.addLine("No State Commanded");
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