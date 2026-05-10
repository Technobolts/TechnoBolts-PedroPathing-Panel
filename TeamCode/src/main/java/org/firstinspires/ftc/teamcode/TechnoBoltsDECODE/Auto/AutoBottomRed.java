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
        SHOOT_PRELOAD_HUMAN,
        TURN_OFF
        //Shoot to start intake preset
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
    private final Pose startShootPose = new Pose(92, 12.689999999999999999, Math.toRadians(245));

    private final Pose ShootPosehumanPos = new Pose(85.5111111111111, 23.999999999999993);
    private final Pose humanZone = new Pose(100.26666666666667, 24.53333333333334, 0);
    private PathChain driveStartPosShootPos, drivehumanZone;


    public void buildPaths () {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), startShootPose.getHeading())
                .build();

        drivehumanZone = follower.pathBuilder()
                .addPath(new BezierCurve(startShootPose,ShootPosehumanPos,humanZone))
                .setLinearHeadingInterpolation(startShootPose.getHeading(), ShootPosehumanPos.getHeading(),humanZone.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case DRIVE_STARTPOS_SHOOT_POS:
                if (!follower.isBusy()) {
                    doDepositOn();
                    kickerStop();
                    doIntakePowerOn();
                    follower.followPath(driveStartPosShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }//reset the timer & make new state
                break;

            case SHOOT_PRELOAD:
                //check is follower done its path?
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 1500 ) {
                    doDepositOn();
                    sleep(1500);
                    kickerLaunch();
                    doRampOn();
                    //doIntakePowerOn();
                    sleep(6000);
                    kickerStop();
                    telemetry.addLine("Shooting Preload");
                    follower.followPath(drivehumanZone);
                    setPathState(PathState.SHOOT_PRELOAD_HUMAN);
                }
                break;

            case SHOOT_PRELOAD_HUMAN:
                if (!follower.isBusy()) {
                    doRampSlow();
                    kickerStop();
                    telemetry.addLine("Aligning to human position");
                    follower.followPath(drivehumanZone, 0.5,true);
                    setPathState(PathState.TURN_OFF);
                }
                break;
            case TURN_OFF:
                if (!follower.isBusy()) {
                    doRampSlow();
                    kickerStop();
                    doDepositOff();
                    setPathState(PathState.TURN_OFF);
                    telemetry.addLine("Turning-Off");
                }
            default:
                telemetry.addLine("No State Commanded");
        }

    }
}
