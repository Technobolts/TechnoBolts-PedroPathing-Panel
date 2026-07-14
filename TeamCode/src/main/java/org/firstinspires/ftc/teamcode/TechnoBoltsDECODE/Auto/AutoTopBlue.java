package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Auto;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class AutoTopBlue {

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

    public AutoTopBlue(Follower follower, Telemetry telemetry, DcMotor intake, DcMotorEx turretShooter, Servo Kicker, Servo Spindexer, Servo turretHood) {

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



//    public void shoot() {
//        KickerStop();
//        sleep(1700);
//        KickerLaunch();
//    }

    public enum PathState {
        // START POSITION --> END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE
        DRIVE_STARTPOS_SHOOT_POS, //From start to shoot position
        SHOOT_PRELOAD, //Shoot preload
        SHOOT_PRELOAD_PRESET1, //Shoot to start intake preset
        INTAKE_PRESET1, //aligned preset to fully intake preset
        SHOOT_PRESET1_PRESET3 , //From shooting to preset 3
        INTAKE_PRESET3 , //Intaking preset 3
        SHOOT_PRESET3 , // Shooting preset 3
        SHOOT_PRESET3_PRESET2 , //From shooting preset 3 to preset 2
        LEAVE_LAUNCH_ZONE,
        INTAKE_PRESET2,
        STRAFE_OUT, //Intake preset 2
        PRESET2_EMPTY_RAMP,

        EMPTY_RAMP_SHOOT,
        DONE
    }

    PathState pathState;


    private final Pose startPose = new Pose(25.066666666666666, 122.66666666666667, Math.toRadians(320));

    private final Pose shootPose = new Pose(58.15555555555557, 78.11111111111111, Math.toRadians(307));

    private final Pose presetPose = new Pose(47.644444444444446, 72.3111111111111, Math.toRadians(180));

    private final Pose Preset1PosIntakePose = new Pose(15.822222222222223, 72.60000000000001, Math.toRadians(180));

    private final Pose IntakePoseShootPosePreset1 = new Pose(58.15555555555557, 78.11111111111111, Math.toRadians(307));
    private final Pose ShootPosPreset3Pos = new Pose(48, 24.133333333333326, Math.toRadians(180));
    private final Pose Preset3PosIntakePose = new Pose(23.822222222222223, 24.955555555555552, Math.toRadians(180));
    private final Pose IntakePoseShootPosePreset3 = new Pose(58.15555555555557, 78.11111111111111, Math.toRadians(307));
    //    private final Pose leaveLaunchZone = new Pose(, Math.toRadians(235));
    private final Pose Preset2PosIntakePose = new Pose(26.31111111111111, 90.31111111111109, Math.toRadians(0));

    private final Pose shootPosePreset2Pose = new Pose(76.42222222222226, 101.19999999999999);
//    private final Pose Preset2PosIntakePose = new Pose(108.4, 84.08888888888887, Math.toRadians(0));

//    private final Pose shootPosePreset2Pose = new Pose(76.244, 83.24444444444444);
//    private final Pose IntakePoseShootPosePreset2 = new Pose(112.35555555555555, 72, Math.toRadians(0));
//    private final Pose Preset2IntakeEmptyRamp = new Pose(121.06666666666668, 83.73333333333333, Math.toRadians(0));
//
//    private final Pose EmptyRampShootingPos = new Pose(74.84444444444443, 78.11111111111111, Math.toRadians(225));
    private final Pose StrafeOut = new Pose(55.46666666666667, 75.19999999999999, Math.toRadians(320));


    private PathChain driveStartPosShootPos, driveShootPosPreset1Pos, drivePreset1PosIntakePose, driveIntakePoseShootPosePreset1 , driveShootPosPreset3Pos, drivePreset3PosIntakePose, driveIntakePoseShootPosePreset3, driveLeaveLaunchZone,  drivePreset2PosIntakePose, driveStrafeOut, driveIntakePoseShootPosePreset2, drivePreset2IntakeEmptyRamp, driveEmptyRampShootPos;


    public void buildPaths () {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosPreset1Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose,presetPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), presetPose.getHeading())
                .build();
        drivePreset1PosIntakePose = follower.pathBuilder()
                .addPath(new BezierLine(presetPose, Preset1PosIntakePose))
                .setLinearHeadingInterpolation(presetPose.getHeading(), Preset1PosIntakePose.getHeading())
                .build();
        driveIntakePoseShootPosePreset1 = follower.pathBuilder()
                .addPath(new BezierLine(Preset1PosIntakePose, IntakePoseShootPosePreset1))
                .setLinearHeadingInterpolation(Preset1PosIntakePose.getHeading(), IntakePoseShootPosePreset1.getHeading())
                .build();
        driveShootPosPreset3Pos = follower.pathBuilder()
                .addPath(new BezierLine(IntakePoseShootPosePreset1, ShootPosPreset3Pos))
                .setLinearHeadingInterpolation(IntakePoseShootPosePreset1.getHeading(), ShootPosPreset3Pos.getHeading())
                .build();
        drivePreset3PosIntakePose = follower.pathBuilder()
                .addPath(new BezierLine(ShootPosPreset3Pos, Preset3PosIntakePose))
                .setLinearHeadingInterpolation(ShootPosPreset3Pos.getHeading(), Preset3PosIntakePose.getHeading())
                .build();
        driveIntakePoseShootPosePreset3 = follower.pathBuilder()
                .addPath(new BezierLine(Preset3PosIntakePose, IntakePoseShootPosePreset3))
                .setLinearHeadingInterpolation(Preset3PosIntakePose.getHeading(), IntakePoseShootPosePreset3.getHeading())
                .build();
        driveLeaveLaunchZone = follower.pathBuilder()
                .addPath(new BezierLine(IntakePoseShootPosePreset3, StrafeOut))
                .setLinearHeadingInterpolation(IntakePoseShootPosePreset3.getHeading(), StrafeOut.getHeading())
                .build();
//            driveStrafeOut = follower.pathBuilder()
//                    .addPath(new BezierLine(IntakePoseShootPosePreset3, StrafeOut))
//                    .setLinearHeadingInterpolation(IntakePoseShootPosePreset3.getHeading(), StrafeOut.getHeading())
//                    .build();
        drivePreset2PosIntakePose = follower.pathBuilder()
                .addPath(new BezierCurve(IntakePoseShootPosePreset3, shootPosePreset2Pose,Preset2PosIntakePose))
                .setLinearHeadingInterpolation(IntakePoseShootPosePreset3.getHeading(), Preset2PosIntakePose.getHeading())
                .build();
//            driveStrafeOut = follower.pathBuilder()
//                    .addPath(new BezierLine(IntakePoseShootPosePreset3, StrafeOut))
//                    .setLinearHeadingInterpolation(IntakePoseShootPosePreset3.getHeading(), StrafeOut.getHeading())
//                    .build();
//            drivePreset2PosIntakePose = follower.pathBuilder()
//                    .addPath(new BezierCurve(IntakePoseShootPosePreset3, shootPosePreset2Pose,Preset2PosIntakePose))
//                    .setLinearHeadingInterpolation(IntakePoseShootPosePreset3.getHeading(), Preset2PosIntakePose.getHeading())
//                    .build();
//            driveIntakePoseShootPosePreset2 = follower.pathBuilder()
//                    .addPath(new BezierLine(Preset2PosIntakePose, IntakePoseShootPosePreset2))
//                    .setLinearHeadingInterpolation(Preset2PosIntakePose.getHeading(), IntakePoseShootPosePreset2.getHeading())
//                    .build();
//            drivePreset2IntakeEmptyRamp = follower.pathBuilder()
//                    .addPath(new BezierLine(IntakePoseShootPosePreset2, Preset2IntakeEmptyRamp))
//                    .setLinearHeadingInterpolation(IntakePoseShootPosePreset2.getHeading(), Preset2IntakeEmptyRamp.getHeading())
//                    .build();
//            driveEmptyRampShootPos = follower.pathBuilder()
//                    .addPath(new BezierLine(Preset2IntakeEmptyRamp, EmptyRampShootingPos))
//                    .setLinearHeadingInterpolation(Preset2IntakeEmptyRamp.getHeading(), EmptyRampShootingPos.getHeading())
//                    .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);

                setPathState(PathState.SHOOT_PRELOAD); //reset the timer & make new state
                break;

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
