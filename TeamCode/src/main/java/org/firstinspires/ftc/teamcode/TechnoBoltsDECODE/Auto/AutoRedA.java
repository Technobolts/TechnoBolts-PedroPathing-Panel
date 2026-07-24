package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms.TBWebcam.AlignToAprilTagTurret;


public class AutoRedA {

    private Follower follower;

    //RobotTeleOp robot = new RobotTeleOp();
    AutoSelector autoSelector;
    public Timer pathTimer, opModeTimer;
    private ElapsedTime autoTimer = new ElapsedTime();

    public final DcMotor intake;
    public final DcMotorEx turretShooter, turretMotor;
    public final Servo kicker;
    public final Servo spindexer;
    public final Servo turretHood;
    public final Limelight3A limelight;
    public final NormalizedColorSensor colorSensor;
    public final DistanceSensor distanceSensor;
    private AlignToAprilTagTurret turret = new AlignToAprilTagTurret();

    //Spindexer positions

    //Everything else
    private final double intakePowerOn = -0.6;
    private final double intakePowerOff = 0;
    private final double kickerRest = 0.15;
    private final double kickerFire = 0.5;
    private final double outtakeOn  =  0.22;




    // Define the motor at the top of your OpMode class




    Telemetry telemetry;

    public AutoRedA(AutoSelector autoSelector, Follower follower, Telemetry telemetry, DcMotor intake1, DcMotorEx TurretShooter, Servo Kicker, Servo spindexer, Servo turretHood, Limelight3A limelight, NormalizedColorSensor colorSensor, DistanceSensor distanceSensor, DcMotorEx turret) {

        this.autoSelector = autoSelector;
        this.follower = follower;
        this.telemetry = telemetry;
        this.intake = intake1;
        this.turretShooter = TurretShooter;
        this.kicker = Kicker;
        this.spindexer = spindexer;
        this.turretHood = turretHood;
        this.limelight = limelight;
        this.colorSensor = colorSensor;
        this.distanceSensor = distanceSensor;
        this.turretMotor = turret;

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

    public void update(Telemetry telemetry) {
        follower.update();
        autonomousPathUpdate(telemetry);
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
        SHOOT_PRELOAD_INTAKE1,//Shoot preload
        ALIGN_PRESET1,

        INTAKE_PRESET_1,
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
    private final Pose shootPoseAlignPreset1 = new Pose(104.46666666666667, 60.219999999999999, Math.toRadians(360));
    private final Pose IntakePreset1 = new Pose(122.95555555555556, 60.219999999999999, Math.toRadians(360));
    private final Pose Preset1shootPose = new Pose(92.8, 84.219999999999999, Math.toRadians(360));
    private final Pose shootPoseIntake3Align = new Pose(104.46666666666667, 11.6888888888889, Math.toRadians(360));
    private final Pose shootPoseIntake3AlignCtrl = new Pose(95.41111111111111, 14.33333333333323, Math.toRadians(360));
    private final Pose shootPoseIntake3 = new Pose(123, 11.46666666666667, Math.toRadians(360));
    private final Pose ShootPreset3Pose = new Pose(92.8, 84.219, Math.toRadians(360));
    private final Pose ShootPreset3PoseControl = new Pose(87.39, 31.0493, Math.toRadians(290));
    private final Pose LeaveZonePose = new Pose(102.067, 76.22, Math.toRadians(290));







    private PathChain driveStartPosShootPosAlign, driveShootPosAlignIntake1, driveAlignPresetToIntake1,driveIntake1ShootPos, driveShootPosIntake3Align, driveIntake3AlignIntake3, Intake3ToShoot3, Shoot3ToLeave;


    public void buildPaths () {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPosAlign = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosAlignIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, shootPoseAlignPreset1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), shootPoseAlignPreset1.getHeading())
                .build();

        driveAlignPresetToIntake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseAlignPreset1, IntakePreset1))
                .setLinearHeadingInterpolation(shootPoseAlignPreset1.getHeading(), IntakePreset1.getHeading())
                .build();

        driveIntake1ShootPos = follower.pathBuilder()
                .addPath(new BezierLine(IntakePreset1, Preset1shootPose))
                .setLinearHeadingInterpolation(IntakePreset1.getHeading(), Preset1shootPose.getHeading())
                .build();

        driveShootPosIntake3Align = follower.pathBuilder()
                .addPath(new BezierCurve(Preset1shootPose, shootPoseIntake3AlignCtrl, shootPoseIntake3Align))
                .setLinearHeadingInterpolation(Preset1shootPose.getHeading(), shootPoseIntake3Align.getHeading())
                .build();

        driveIntake3AlignIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPoseIntake3Align, shootPoseIntake3))
                .setLinearHeadingInterpolation(shootPoseIntake3Align.getHeading(), shootPoseIntake3.getHeading())
                .build();

        Intake3ToShoot3 = follower.pathBuilder()
                .addPath(new BezierCurve(shootPoseIntake3, ShootPreset3PoseControl, ShootPreset3Pose))
                .setLinearHeadingInterpolation(shootPoseIntake3.getHeading(), ShootPreset3Pose.getHeading())
                .build();

        Shoot3ToLeave = follower.pathBuilder()
                .addPath(new BezierLine(ShootPreset3Pose, LeaveZonePose))
                .setLinearHeadingInterpolation(ShootPreset3Pose.getHeading(), LeaveZonePose.getHeading())
                .build();

    }
    public void TurretAprilTagTracking(){


        LLResult result = this.limelight.getLatestResult();

        if (result != null && result.isValid()) {

            // Horizontal angle from crosshair to tag
            double tx = result.getTx();

            turret.update(tx, turretMotor);

            telemetry.addData("TX foudn -> tx : ", tx);
            telemetry.update();
        } else {

            turret.stop(turretMotor);

            telemetry.addLine("No AprilTag");
            telemetry.update();
        }
    }

    public void StopTurretAprilTagTracking(){
        turret.stop(turretMotor);
    }

    public void autonomousPathUpdate(Telemetry telemetry) {
        switch (pathState) {

            case DRIVE_STARTPOS_SHOOT_POS:

                if (!follower.isBusy()) {
                    follower.followPath(driveStartPosShootPosAlign, true);
                    setPathState(PathState.ALIGN_PRESET1);
                }

                break;

            case ALIGN_PRESET1:
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    doIntakePowerOn();
//                    TurretAprilTagTracking();
                    if (autoSelector.runAutoShoot(true)) {

                    }

                    telemetry.addLine("Align Preset 1");
                    follower.followPath(driveShootPosAlignIntake1,0.5,true);
                    setPathState(PathState.INTAKE_PRESET_1);
                }
                break;

            case INTAKE_PRESET_1:
                if (!follower.isBusy()|| pathTimer.getElapsedTimeSeconds() > 3) {

                    autoSelector.AutoIndexer(false);
                    telemetry.addLine("Intaking Preset 1");
                    follower.followPath(driveAlignPresetToIntake1, 0.35,true);
                    setPathState(PathState.SHOOT_INTAKE_PRESET1);
                }
                break;

            case SHOOT_INTAKE_PRESET1:
                if  (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    telemetry.addLine("Shooting Preset 1");
                    if (autoSelector.runAutoShoot(true)) {

                    }
                    follower.followPath(driveIntake1ShootPos,true);

                    setPathState(PathState.PRESET1_PRESET3);
                }
                break;

            case PRESET1_PRESET3:
                if (!follower.isBusy()|| pathTimer.getElapsedTimeSeconds() > 3){

                    telemetry.addLine("Aligning to Preset 3");
                    follower.followPath(driveShootPosIntake3Align, 0.5, true);
                    setPathState(PathState.INTAKE_PRESET3);
                }
                break;

            case INTAKE_PRESET3:

                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                        telemetry.addLine("Intaking Preset 3");
                        follower.followPath(driveIntake3AlignIntake3,  true);
                        setPathState(PathState.SHOOT_PRESET3);
                    }
                    break;


            case SHOOT_PRESET3:

                    if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                        telemetry.addLine("Shooting Preset 3");
                        follower.followPath(Intake3ToShoot3,  true);
                        if (autoSelector.runAutoShoot(true)) {

                        }
                        StopTurretAprilTagTracking();
                        setPathState(PathState.SHOOT_PRESET3_PRESET2);
                    }
                    break;

            case SHOOT_PRESET3_PRESET2:

                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3) {
                    telemetry.addLine("Shooting Preset 3");
                    follower.followPath(Shoot3ToLeave,  true);

                    setPathState(PathState.DONE);
                }
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
