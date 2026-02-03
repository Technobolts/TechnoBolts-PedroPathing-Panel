package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class AutoBottomBlue {
    private Follower follower;
    private Timer pathTimer, opModeTimer;


    public final CRServo intake;
    public final DcMotor leftDeposit;
    public final DcMotor rightDeposit;
    public final Servo kickerServo;
    public final CRServo lowerTServo;
    public final CRServo middleTServo;
    public final Servo ledDepo;


    private final double leftPowerOn = -0.36;
    private final double rightPowerOn = 0.36;
    private final double leftPowerOff = 0;
    private final double rightPowerOff = 0;
    private final double lowerRampOn = -1;
    private final double middleRampOn = 1;

    private final double lowerRampOff = 0;
    private final double middleRampOff = 0;
    private final double intakePowerOn = -1;
    private final double intakePowerOff = 0;
    private final double kickerAngleUp = 0;
    private final double kickerAngleDown = 0.8;
    // Define the motor at the top of your OpMode class




    Telemetry telemetry;

    public AutoBottomBlue(Follower follower, Telemetry telemetry, CRServo intake, DcMotor leftDeposit, DcMotor rightDeposit, Servo upperTServo, CRServo lowerTServo, CRServo middleTServo, Servo ledDepo) {

        this.follower = follower;
        this.telemetry = telemetry;
        this.intake = intake;
        this.leftDeposit = leftDeposit;
        this.rightDeposit = rightDeposit;
        this.kickerServo = upperTServo;
        this.lowerTServo = lowerTServo;
        this.middleTServo = middleTServo;
        this.ledDepo = ledDepo;


        pathTimer = new Timer();
    }

    public void start () {
//        opModeTimer.resetTimer();

        telemetry.addLine("Starting BOTTOM BLUE Auto");
        telemetry.update();

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

    public void doRampOff() {
        lowerTServo.setPower(lowerRampOff);
        middleTServo.setPower(middleRampOff);
    }

    public void doKickerOn() {
        kickerServo.setPosition(kickerAngleDown);
    }

    public void doKickerOff() {
        kickerServo.setPosition(kickerAngleUp);
    }

    public void doDepositOn() {
        leftDeposit.setPower(leftPowerOn);
        rightDeposit.setPower(rightPowerOn);
    }

    public void doDepositOff() {
        leftDeposit.setPower(leftPowerOff);
        rightDeposit.setPower(rightPowerOff);
    }

    public void shoot() {
        doKickerOn();
        sleep(1700);
        doKickerOff();
    }

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
        INTAKE_PRESET2, //Intake preset 2
        PRESET2_EMPTY_RAMP,

        EMPTY_RAMP_SHOOT,
        DONE
    }

    PathState pathState;

    private final Pose startPose = new Pose(25.066666666666666, 122.66666666666667, Math.toRadians(320));

    private final Pose shootPose = new Pose(69.15555555555557, 78.11111111111111, Math.toRadians(320));

    private final Pose presetPose = new Pose(47.644444444444446, 66.3111111111111, Math.toRadians(180));

    private final Pose Preset1PosIntakePose = new Pose(23.822222222222223, 65.60000000000001, Math.toRadians(180));

    private final Pose IntakePoseShootPosePreset1 = new Pose(71.82222222222222, 73.77777777777779, Math.toRadians(320));
    private final Pose ShootPosPreset3Pos = new Pose(48, 18.133333333333326, Math.toRadians(180));
    private final Pose Preset3PosIntakePose = new Pose(23.822222222222223, 17.955555555555552, Math.toRadians(180));
    private final Pose IntakePoseShootPosePreset3 = new Pose(71.82222222222222, 73.77777777777779, Math.toRadians(320));
//    private final Pose Preset2PosIntakePose = new Pose(41.6, 71.82222222222222, Math.toRadians(360));
//    private final Pose IntakePoseShootPosePreset2 = new Pose(31.64444444444445, 72, Math.toRadians(360));
//    private final Pose Preset2IntakeEmptyRamp = new Pose(22.93333333333332, 83.73333333333333, Math.toRadians(360));
//    private final Pose EmptyRampShootingPos = new Pose(69.15555555555557, 78.11111111111111, Math.toRadians(135));

    private PathChain driveStartPosShootPos, driveShootPosPreset1Pos, drivePreset1PosIntakePose, driveIntakePoseShootPosePreset1 , driveShootPosPreset3Pos, drivePreset3PosIntakePose, driveIntakePoseShootPosePreset3, drivePreset2PosIntakePose, driveIntakePoseShootPosePreset2, drivePreset2IntakeEmptyRamp, driveEmptyRampShootPos;


    public void buildPaths () {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosPreset1Pos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, presetPose))
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
//        drivePreset2PosIntakePose = follower.pathBuilder()
//                .addPath(new BezierLine(IntakePoseShootPosePreset3, Preset2PosIntakePose))
//                .setLinearHeadingInterpolation(IntakePoseShootPosePreset3.getHeading(), Preset2PosIntakePose.getHeading())
//                .build();
//        driveIntakePoseShootPosePreset2 = follower.pathBuilder()
//                .addPath(new BezierLine(Preset2PosIntakePose, IntakePoseShootPosePreset2))
//                .setLinearHeadingInterpolation(Preset2PosIntakePose.getHeading(), IntakePoseShootPosePreset2.getHeading())
//                .build();
//        drivePreset2IntakeEmptyRamp = follower.pathBuilder()
//                .addPath(new BezierLine(IntakePoseShootPosePreset2, Preset2IntakeEmptyRamp))
//                .setLinearHeadingInterpolation(IntakePoseShootPosePreset2.getHeading(), Preset2IntakeEmptyRamp.getHeading())
//                .build();
//        driveEmptyRampShootPos = follower.pathBuilder()
//                .addPath(new BezierLine(Preset2IntakeEmptyRamp, EmptyRampShootingPos))
//                .setLinearHeadingInterpolation(Preset2IntakeEmptyRamp.getHeading(), EmptyRampShootingPos.getHeading())
//                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                doDepositOn();
                telemetry.addLine("Going to the shooting position");
                telemetry.update();
                setPathState(PathState.SHOOT_PRELOAD); //reset the timer & make new state
                break;

            case SHOOT_PRELOAD:
                //check is follower done its path?
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1500 ) {
                    sleep(500);
//                        shoot();
//                        sleep (1000);

                    doRampOn();

                    doIntakePowerOn();

                    sleep(5500);

                    shoot();

                    doDepositOff();

                    telemetry.addLine("Shooting Preload");
                    follower.followPath(driveShootPosPreset1Pos, true);
                    setPathState(PathState.SHOOT_PRELOAD_PRESET1);

                }
                break;

            case SHOOT_PRELOAD_PRESET1:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 3) {
                    doDepositOff();
                    telemetry.addLine("Aligning to preset1");
                    follower.followPath(drivePreset1PosIntakePose, true);
                    setPathState(PathState.INTAKE_PRESET1);
                }
                break;

            case INTAKE_PRESET1:
                if (!follower.isBusy()) {
                    doDepositOn();
                    telemetry.addLine("Intaking Artifacts of Preset 1");
                    follower.followPath(driveIntakePoseShootPosePreset1, true);
                    setPathState(PathState.SHOOT_PRESET1_PRESET3);
                }
                break;

            case SHOOT_PRESET1_PRESET3:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 2500 ) {
                    telemetry.addLine("Shooting Preset 1");
                    sleep (1000);
                    shoot();

                    doDepositOff();

                    follower.followPath(driveShootPosPreset3Pos , true);
                    setPathState(PathState.INTAKE_PRESET3);
                }
                break;
            case INTAKE_PRESET3:
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 1500) {
                    doDepositOff();
                    telemetry.addLine("Aligning Artifacts");
                    follower.followPath(drivePreset3PosIntakePose, true);
                    setPathState(PathState.SHOOT_PRESET3);
                }
                break;
            case SHOOT_PRESET3:
                if(!follower.isBusy() && pathTimer.getElapsedTime() > 1500) {
                    doDepositOff();
                    telemetry.addLine("Intaking Artifacts");
                    follower.followPath(driveIntakePoseShootPosePreset3, true);
                    setPathState(PathState.SHOOT_PRESET3);
                }
//                break;
//            case SHOOT_PRESET3_PRESET2:
//                if(!follower.isBusy() && pathTimer.getElapsedTime() > 1500) {
//                    doDepositOn();
//                    sleep(2000);
//                    shoot();
//                    sleep (2500);
//                    shoot();
//
//                    telemetry.addLine("Shooting Preset 3");
//                    follower.followPath(drivePreset2PosIntakePose, true);
//                    setPathState(PathState.INTAKE_PRESET2);
//                }
//                break;
//            case INTAKE_PRESET2:
//                if(!follower.isBusy()&& pathTimer.getElapsedTime() > 1500) {
//                    telemetry.addLine("Aligning to artifacts");
//                    follower.followPath(driveIntakePoseShootPosePreset2, true);
//                    setPathState(PathState.PRESET2_EMPTY_RAMP);
//                }
//                break;
//            case PRESET2_EMPTY_RAMP:
//                if(!follower.isBusy()&& pathTimer.getElapsedTime() > 2500) {
//                    telemetry.addLine("Emptying Ramp");
//                    follower.followPath(drivePreset2IntakeEmptyRamp, true);
//                    setPathState(PathState.EMPTY_RAMP_SHOOT);
//                }
//                break;
//            case EMPTY_RAMP_SHOOT:
//                if(!follower.isBusy() && pathTimer.getElapsedTime() > 4500) {
//                    telemetry.addLine("Shooting");
//                    follower.followPath(driveEmptyRampShootPos, true);
//                    setPathState(PathState.EMPTY_RAMP_SHOOT);
//                }
//                break;
//            case DONE:
//                if(!follower.isBusy()){
//                    telemetry.addLine("Auto Done");
//                }
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
