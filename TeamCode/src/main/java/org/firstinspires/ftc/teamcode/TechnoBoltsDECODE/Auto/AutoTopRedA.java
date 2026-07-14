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


public class AutoTopRedA {

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

    public AutoTopRedA(Follower follower, Telemetry telemetry, DcMotor intake, DcMotorEx turretShooter, Servo Kicker, Servo Spindexer, Servo turretHood) {

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

   private final Pose startPose = new Pose(124.7999988888889, 124.25887777777776, Math.toRadians(225));

   private final Pose shootPose = new Pose(92.8, 84.219999999999999, Math.toRadians(360));
   private final Pose IntakePreset1 = new Pose(122.95555555555556, 84.219999999999999, Math.toRadians(360));
   private final Pose shootPoseAlignPreset3 = new Pose(92.8, 84.219999999999999, Math.toRadians(290));
   private final Pose shootPoseIntake3Align = new Pose(104.46666666666667, 35.6888888888889, Math.toRadians(360));
        private final Pose shootPoseIntake3AlignCtrl = new Pose(95.41111111111111, 38.33333333333323, Math.toRadians(360));
   private final Pose shootPoseIntake3 = new Pose(123, 35.46666666666667, Math.toRadians(360));




    private PathChain driveStartPosShootPosAlign, driveShootPosAlignIntake1, driveIntake1ShootPos, driveShootPosIntake3Align, driveIntake3AlignIntake3;


        public void buildPaths () {
            // put in coordinates for starting pose > ending pose
            driveStartPosShootPosAlign = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                    .build();

            driveShootPosAlignIntake1 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, IntakePreset1))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), IntakePreset1.getHeading())
                    .build();

            driveIntake1ShootPos = follower.pathBuilder()
                    .addPath(new BezierLine(IntakePreset1, shootPoseAlignPreset3))
                    .setLinearHeadingInterpolation(IntakePreset1.getHeading(), shootPoseAlignPreset3.getHeading())
                    .build();

            driveShootPosIntake3Align = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPoseAlignPreset3, shootPoseIntake3AlignCtrl, shootPoseIntake3Align))
                    .setLinearHeadingInterpolation(shootPoseAlignPreset3.getHeading(), shootPoseIntake3Align.getHeading())
                    .build();

            driveIntake3AlignIntake3 = follower.pathBuilder()
                    .addPath(new BezierLine(shootPoseIntake3Align, shootPoseIntake3))
                    .setLinearHeadingInterpolation(shootPoseIntake3Align.getHeading(), shootPoseIntake3.getHeading())
                    .build();

        }

        public void autonomousPathUpdate() {
            switch (pathState) {

                case DRIVE_STARTPOS_SHOOT_POS:
                    follower.followPath(driveStartPosShootPosAlign, true);

                    setPathState(PathState.SHOOT_PRELOAD_INTAKE1); //reset the timer & make new state
                    break;

                case SHOOT_PRELOAD_INTAKE1:
                    if (!follower.isBusy() && pathTimer.getElapsedTime() > 1500 ) {

                        telemetry.addLine("Intake Preset 1");
                        follower.followPath(driveShootPosAlignIntake1,0.5,true);
                        setPathState(PathState.SHOOT_INTAKE_PRESET1);
                    }
                    break;

                case SHOOT_INTAKE_PRESET1:
                    if (!follower.isBusy()) {

                        telemetry.addLine("Shoot Preset");
                        follower.followPath(driveIntake1ShootPos, true);
                        setPathState(PathState.PRESET1_PRESET3);
                    }
                    break;

                case PRESET1_PRESET3:
                    if  (!follower.isBusy() && pathTimer.getElapsedTime() > 1500) {
                        telemetry.addLine("Aligning to Preset 3");
                        follower.followPath(driveShootPosIntake3Align,true);
                        setPathState(PathState.INTAKE_PRESET3);
                    }

                case INTAKE_PRESET3:
                    if (!follower.isBusy()){
                        telemetry.addLine("Intake Preset 3");
                        follower.followPath(driveIntake3AlignIntake3, 0.5, true);
                        setPathState(PathState.SHOOT_PRESET3);
                    }

                default:
                    telemetry.addLine("No State Commanded");
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
