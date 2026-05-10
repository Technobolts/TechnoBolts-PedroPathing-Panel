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
    public final DcMotorEx leftDeposit;
    public final DcMotorEx rightDeposit;
    public final CRServo kickerServo;
    public final CRServo lowerTServo;
    public final CRServo middleTServo;
    public final Servo ledDepo;


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

    public AutoTopBlue(Follower follower, Telemetry telemetry, DcMotor intake, DcMotorEx leftDeposit, DcMotorEx rightDeposit, CRServo Kicker, CRServo lowerTServo, CRServo middleTServo, Servo ledDepo) {

        this.follower = follower;
        this.telemetry = telemetry;
        this.intake = intake;
        this.leftDeposit = leftDeposit;
        this.rightDeposit = rightDeposit;
        this.kickerServo = Kicker;
        this.lowerTServo = lowerTServo;
        this.middleTServo = middleTServo;
        this.ledDepo = ledDepo;


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

    public void doRampOn() {
        lowerTServo.setPower(lowerRampOn);
        middleTServo.setPower(middleRampOn);
    }

    public void doRampSlow() {
        lowerTServo.setPower(lowerRampSlow);
        middleTServo.setPower(middleRampSlow);
    }

    public void kickerStop() {
        kickerServo.setPower(kickerStopPower);
    }

    public void kickerHalfLaunch(){
        kickerServo.setPower(kickerHalfLaunchPower);
    }

    public void doIntakeHalfPower() {
        intake.setPower(intakeHalfPower);
    }

    public void halfPowerAll() {
        doIntakeHalfPower();
        doRampSlow();
        kickerStop();

    }

    public void kickerLaunch() {
        kickerServo.setPower(kickerLaunchPower);
    }

    public void doDepositOn() {
        leftDeposit.setVelocity(ShooterOn);
        rightDeposit.setVelocity(ShooterOn);
    }

    public void doDepositOff() {
        leftDeposit.setPower(leftPowerOff);
        rightDeposit.setPower(rightPowerOff);
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
                doDepositOn();
                kickerHalfLaunch();
                setPathState(PathState.SHOOT_PRELOAD); //reset the timer & make new state
                break;

            case SHOOT_PRELOAD:
                //check is follower done its path?
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1500 ) {
                    kickerLaunch();
                    doRampOn();
                    doIntakePowerOn();
                    sleep(4700);
                    kickerStop();
                    telemetry.addLine("Shooting Preload");
                    follower.followPath(driveShootPosPreset1Pos,true);
                    setPathState(PathState.SHOOT_PRELOAD_PRESET1);
                }
                break;

            case SHOOT_PRELOAD_PRESET1:
                if (!follower.isBusy()) {
                    halfPowerAll();
                    telemetry.addLine("Aligning to preset1");
                    follower.followPath(drivePreset1PosIntakePose, 0.5,true);
                    setPathState(PathState.INTAKE_PRESET1);
                }
                break;

            case INTAKE_PRESET1:
                if (!follower.isBusy()) {
                    doRampSlow();
                    telemetry.addLine("Intaking Artifacts of Preset 1");
                    follower.followPath(driveIntakePoseShootPosePreset1,true);
                    setPathState(PathState.SHOOT_PRESET1_PRESET3);

                }
                break;


            case SHOOT_PRESET1_PRESET3:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 2500 ) {
                    telemetry.addLine("Shooting Preset 1");
                    doRampOn();
                    doIntakePowerOn();
                    kickerLaunch();
                    sleep(5000);
                    halfPowerAll();
                    follower.followPath(driveShootPosPreset3Pos,  true);
                    setPathState(PathState.INTAKE_PRESET3);
                }
                break;
            case INTAKE_PRESET3:
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 1500) {
                    halfPowerAll();
                    telemetry.addLine("Aligning Artifacts");
                    follower.followPath(drivePreset3PosIntakePose,0.5,true);
                    setPathState(PathState.SHOOT_PRESET3);

                }
                break;
            case SHOOT_PRESET3:
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 1500) {
                    halfPowerAll();
                    telemetry.addLine("Intaking Artifacts");
                    follower.followPath(driveIntakePoseShootPosePreset3, true);
                    setPathState(PathState.SHOOT_PRESET3_PRESET2);
                }
                break;
            case SHOOT_PRESET3_PRESET2:
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 1500) {
                    doRampOn();
                    doIntakePowerOn();
                    kickerLaunch();
                    sleep(5000);
                    halfPowerAll();
                    telemetry.addLine("Shooting Preset 3");
                    follower.followPath(driveLeaveLaunchZone, true);
                    setPathState(PathState.LEAVE_LAUNCH_ZONE);

//                        shoot();
//                        doDepositOff();
                }
                break;
            case LEAVE_LAUNCH_ZONE:
                if(!follower.isBusy() ) {
                    telemetry.addLine("Leaving Launch Zone");
//                        follower.followPath(driveLeaveLaunchZone, true);
                    setPathState(PathState.LEAVE_LAUNCH_ZONE);
                }
//                    break;

//                case STRAFE_OUT:
//                    if (!follower.isBusy()) {
//                        telemetry.addLine("Strafing out");
//                        follower.followPath(driveStrafeOut, true);
//                        setPathState(PathState.STRAFE_OUT);
//                    }i
//                    break;
//                case INTAKE_PRESET2:
//                    if(!follower.isBusy()&& pathTimer.getElapsedTime() > 1500) {
//                        telemetry.addLine("Aligning to artifacts");
//                        follower.followPath(driveIntakePoseShootPosePreset2, true);
//                            setPathState(PathState.PRESET2_EMPTY_RAMP);
//                    }
//                    break;
//                case PRESET2_EMPTY_RAMP:
//                    if(!follower.isBusy()2&& pathTimer.getElapsedTime() > 2500) {
//                        telemetry.addLine("Emptying Ramp");
//                        follower.followPath(drivePreset2IntakeEmptyRamp, true);
//                            setPathState(PathState.EMPTY_RAMP_SHOOT);
//                    }
//                    break;
//                case EMPTY_RAMP_SHOOT:
//                    if(!follower.isBusy() && pathTimer.getElapsedTime() > 4500) {
//                        telemetry.addLine("Shooting");
//                        follower.followPath(driveEmptyRampShootPos, true);
//                        setPathState(PathState.EMPTY_RAMP_SHOOT);
//                    }
//                    break;
//                case DONE:
//                    if(!follower.isBusy()){
//                        telemetry.addLine("Auto Done");
//                    }
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
