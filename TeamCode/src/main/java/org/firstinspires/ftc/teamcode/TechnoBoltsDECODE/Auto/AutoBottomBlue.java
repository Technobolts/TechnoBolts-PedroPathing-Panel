package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Auto;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class AutoBottomBlue {

    private Follower follower;
    private Timer pathTimer, opModeTimer;


    public final DcMotor intake;
    public final DcMotorEx turretShooter;
    public final Servo Kicker;
    public final Servo Spindexer;
    public final Servo turretHood;


    private final double ShooterOn = 800;
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

    public AutoBottomBlue(Follower follower, Telemetry telemetry, DcMotor intake, DcMotorEx turretShooter, Servo Kicker, Servo Spindexer, Servo turretHood) {

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

    private final Pose startPose = new Pose(50.93333333333336, 9.689999999999999999, Math.toRadians(270));
//    private final Pose startShootPose = new Pose(38.06666666666667, 105.24444444444444, Math.toRadians(225));

    private final Pose startShootPose = new Pose(60, 24, Math.toRadians(295));
//    private final Pose startHumanPose = new Pose(55.7777777777778,25.466666666666647, Math.toRadians(180));
    private final Pose humanZone = new Pose(40.82222222222222,24.711111111111112, 0);
    private PathChain driveStartPosShootPos, drivehumanZone ;


    public void buildPaths () {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, startShootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), startShootPose.getHeading())
                .build();

        drivehumanZone = follower.pathBuilder()
                .addPath(new BezierLine(startShootPose,humanZone))
                .setLinearHeadingInterpolation(startShootPose.getHeading(), humanZone.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case DRIVE_STARTPOS_SHOOT_POS:
                if (!follower.isBusy()) {

                    doIntakePowerOn();
                    follower.followPath(driveStartPosShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }//reset the timer & make new state
                break;

            default:
                telemetry.addLine("No State Commanded");
        }

    }
}
