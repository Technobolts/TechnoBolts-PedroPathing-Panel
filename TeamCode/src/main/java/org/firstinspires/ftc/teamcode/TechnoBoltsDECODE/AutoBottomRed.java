package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

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


public class AutoBottomRed {

    private Follower follower;
    private Timer pathTimer, opModeTimer;


    public final DcMotor intake;
    public final DcMotorEx leftDeposit;
    public final DcMotorEx rightDeposit;
    public final CRServo kickerServo;
    public final CRServo lowerTServo;
    public final CRServo middleTServo;
    public final Servo ledDepo;


    private final double ShooterOn = 820;
    private final double leftPowerOff = 0;
    private final double rightPowerOff = 0;
    private final double lowerRampOn = -0.5;
    private final double middleRampOn = 0.5;
    private final double lowerRampSlow = 0.2;
    private final double middleRampSlow = 0.2;
    private final double intakePowerOn = -0.6;
    private final double intakePowerOff = 0;
    private final double kickerStopPower = -0.7;
    private final double kickerLaunchPower = 1;

    private final double kickerHalfLaunchPower = 0.3;
    // Define the motor at the top of your OpMode class




    Telemetry telemetry;

    public AutoBottomRed(Follower follower, Telemetry telemetry, DcMotor intake, DcMotorEx leftDeposit, DcMotorEx rightDeposit, CRServo Kicker, CRServo lowerTServo, CRServo middleTServo, Servo ledDepo) {

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

        pathState = PathState.SHOOT_PRELOAD;
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
        SHOOT_PRELOAD,
        SHOOT_PRELOAD_HUMAN //Shoot to start intake preset
//        INTAKE_PRESET1, //aligned preset to fully intake preset
//        SHOOT_PRESET1_PRESET3 , //From shooting to preset 3
//        INTAKE_PRESET3 , //Intaking preset 3
//        SHOOT_PRESET3 , // Shooting preset 3
//        SHOOT_PRESET3_PRESET2 , //From shooting preset 3 to preset 2
//        LEAVE_LAUNCH_ZONE,
//        INTAKE_PRESET2,
//        STRAFE_OUT, //Intake preset 2
//        PRESET2_EMPTY_RAMP,
//
//        EMPTY_RAMP_SHOOT,
//        DONE
    }

    PathState pathState;

    private final Pose startPose = new Pose(92, 9.689999999999999999, Math.toRadians(270));
    private final Pose startShootPose = new Pose(95, 9.689999999999999999, Math.toRadians(240));

    private final Pose humanZone = new Pose(129.6, 11.7333333333333, 0);

    private PathChain driveStartPosHumanPos, driveStartPosShootPos,driveShootPosPreset1Pos, drivePreset1PosIntakePose, driveIntakePoseShootPosePreset1 , driveShootPosPreset3Pos, drivePreset3PosIntakePose, driveIntakePoseShootPosePreset3, driveleaveLaunchZone,  drivePreset2PosIntakePose, driveStrafeOut, driveIntakePoseShootPosePreset2, drivePreset2IntakeEmptyRamp, driveEmptyRampShootPos;


    public void buildPaths () {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), startShootPose.getHeading())
                .build();
        driveStartPosHumanPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose,humanZone))
                .setLinearHeadingInterpolation(startPose.getHeading(), humanZone.getHeading())
                .build();
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
                    doDepositOn();
                    sleep(1500);
                    kickerLaunch();
                    doRampOn();
                    doIntakePowerOn();
                    sleep(6000);
                    kickerStop();
                    telemetry.addLine("Shooting Preload");
                    follower.followPath(driveStartPosHumanPos);
                    setPathState(PathState.SHOOT_PRELOAD_HUMAN);
                }
                break;

            case SHOOT_PRELOAD_HUMAN:
                if (!follower.isBusy()) {
                    doRampSlow();
                    kickerStop();
                    telemetry.addLine("Aligning to preset1");
//                    follower.followPath(drivePreset1PosIntakePose, 0.5,true);
                    setPathState(PathState.SHOOT_PRELOAD_HUMAN);
                }
//                break;
//
//            case INTAKE_PRESET1:
//                if (!follower.isBusy()) {
//                    doRampOn();
//                    telemetry.addLine("Intaking Artifacts of Preset 1");
//                    follower.followPath(driveIntakePoseShootPosePreset1,true);
//                    setPathState(PathState.SHOOT_PRESET1_PRESET3);
//
//                }
//                break;
//
//
//            case SHOOT_PRESET1_PRESET3:
//                if (!follower.isBusy() && pathTimer.getElapsedTime() > 2500 ) {
//                    telemetry.addLine("Shooting Preset 1");
//                    kickerLaunch();
//                    sleep(5200);
//                        kickerStop();
//                        sleep(1000);
//                    //shoot();
//                    kickerStop();
//
//                        sleep (1000);
//                        shoot();
//
//                    follower.followPath(driveShootPosPreset3Pos,  true);
//                    setPathState(PathState.INTAKE_PRESET3);
//                }
//                break;
//            case INTAKE_PRESET3:
//                if(!follower.isBusy() && pathTimer.getElapsedTime() > 1500) {
//                    doRampSlow();
//                    kickerStop();
//                    telemetry.addLine("Aligning Artifacts");
//                    follower.followPath(drivePreset3PosIntakePose,0.5,true);
//                    setPathState(PathState.SHOOT_PRESET3);
//
//                }
//                break;
//            case SHOOT_PRESET3:
//                if(!follower.isBusy() && pathTimer.getElapsedTime() > 1500) {
//                    doRampOn();
//                    telemetry.addLine("Intaking Artifacts");
//                    follower.followPath(driveIntakePoseShootPosePreset3, true);
//                    setPathState(PathState.SHOOT_PRESET3_PRESET2);
//                }
//                break;
//            case SHOOT_PRESET3_PRESET2:
//                if(!follower.isBusy() && pathTimer.getElapsedTime() > 1500) {
//                        sleep (1500);
//                        shoot();
//                    sleep(50);
//                    kickerLaunch();
//                    sleep(5200);
//
//                    telemetry.addLine("Shooting Preset 3");
//                    follower.followPath(drivePreset2PosIntakePose, true);
//                    setPathState(PathState.LEAVE_LAUNCH_ZONE);
//
//                        shoot();
//                        doDepositOff();
//                }
//                break;
//            case LEAVE_LAUNCH_ZONE:
//                if(!follower.isBusy() ) {
//                    telemetry.addLine("Leaving Launch Zone");
//                        follower.followPath(drivePreset2PosIntakePose, true);
//                    setPathState(PathState.LEAVE_LAUNCH_ZONE);
//                }
//                break;

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
