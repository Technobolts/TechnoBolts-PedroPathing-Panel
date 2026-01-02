package TechnoBolts;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class TechnoBoltsAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

//    private final CRServo intake;
//    private final DcMotor leftDeposit;
//    private final DcMotor rightDeposit;


    private final double leftPower = 0.4;
    private final double rightPower = -0.4;
    private final double kickerAngleUp = 0;
    private final double kickerAngleDown = 0.8;

//    public TechnoBoltsAuto(Follower follower, CRServo intake, DcMotor leftDeposit, DcMotor rightDeposit) {
//
//        this.follower = follower;
//        this.intake = intake;
//        this.leftDeposit = leftDeposit;
//        this.rightDeposit = rightDeposit;
//
//    }

    public enum PathState {
        // START POSITION --> END POSITION
        // DRIVE > MOVEMENT STATE
        // SHOOT > ATTEMPT TO SCORE
        DRIVE_STARTPOS_SHOOT_POS, //From start to shoot position
        SHOOT_PRELOAD, //Shoot preload
        SHOOT_PRELOAD_PRESET1, //Shoot to start intake preset
        INTAKE_PRESET1, //aligned preset to fully intake preset
        PRESET_SHOOTING,
        INTAKE_PRESET3 ,
        SHOOT_PRELOAD_PRESET3 ,


    }

        PathState pathState;

        private final Pose startPose = new Pose(122.31111111111112, 122.4888888888889, Math.toRadians(225));

        private final Pose shootPose = new Pose(74.84444444444443, 79.11111111111111, Math.toRadians(225));

        private final Pose shootPosePresetPose = new Pose(79.644,95.111);

        private final Pose presetPose = new Pose(100.26666666666667, 96.35555555555555, Math.toRadians(0));

        private final Pose presetPoseIntakePose = new Pose(111.28888888888889, 96.17777777777778, Math.toRadians(0));
        private final Pose IntakePoseShootPosePreset1 = new Pose(74.84444444444443, 79.11111111111111, Math.toRadians(225));
       private final Pose Preset3PoseIntakePose = new Pose(101.86666666666666, 48.53333333333333, Math.toRadians(0));
       private final Pose IntakePoseShootPosePreset3 = new Pose(112.71111111111111, 48.53333333333333, Math.toRadians(0));

        private PathChain driveStartPosShootPos, driveShootPosPresetPos, drivePresetPosIntakePose, driveIntakePoseShootPosePreset1 , drivePreset3PoseIntakePose, driveIntakePoseShootPosePreset3;

        public void buildPaths () {
            // put in coordinates for starting pose > ending pose
            driveStartPosShootPos = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                    .build();
            driveShootPosPresetPos = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPose, shootPosePresetPose,presetPose))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), presetPose.getHeading())
                    .build();
            drivePresetPosIntakePose = follower.pathBuilder()
                    .addPath(new BezierLine(presetPose, presetPoseIntakePose))
                    .setLinearHeadingInterpolation(presetPose.getHeading(), presetPoseIntakePose.getHeading())
                    .build();
            driveIntakePoseShootPosePreset1 = follower.pathBuilder()
                    .addPath(new BezierLine(presetPoseIntakePose, IntakePoseShootPosePreset1))
                    .setLinearHeadingInterpolation(presetPoseIntakePose.getHeading(), IntakePoseShootPosePreset1.getHeading())
                    .build();
            drivePreset3PoseIntakePose = follower.pathBuilder()
                    .addPath(new BezierLine(IntakePoseShootPosePreset1, Preset3PoseIntakePose))
                    .setLinearHeadingInterpolation(IntakePoseShootPosePreset1.getHeading(), Preset3PoseIntakePose.getHeading())
                    .build();
            driveIntakePoseShootPosePreset3 = follower.pathBuilder()
                    .addPath(new BezierLine(Preset3PoseIntakePose, IntakePoseShootPosePreset3))
                    .setLinearHeadingInterpolation(Preset3PoseIntakePose.getHeading(), IntakePoseShootPosePreset3.getHeading())
                    .build();
        }

        public void statePathUpdate () {
            switch (pathState) {

                case DRIVE_STARTPOS_SHOOT_POS:
                    follower.followPath(driveStartPosShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD); //reset the timer & make new state
                    break;

                case SHOOT_PRELOAD:
                    //check is follower done its path?
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                        // TODO add logic to flywheel shooter
//                        leftDeposit.setPower(leftPower);
//                        rightDeposit.setPower(rightPower);
                        telemetry.addLine("Shooting");
                        follower.followPath(driveShootPosPresetPos, true);
                        setPathState(PathState.SHOOT_PRELOAD_PRESET1);
                    }
                    break;

                case SHOOT_PRELOAD_PRESET1:
                    if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5 ) {
                        telemetry.addLine("Aligning to preset");
                        follower.followPath(drivePresetPosIntakePose, true);
                        setPathState(PathState.INTAKE_PRESET1);
                    }
                    break;

                case INTAKE_PRESET1:
                    if (!follower.isBusy()) {
                        telemetry.addLine("Intaking Artifacts");
                        follower.followPath(driveIntakePoseShootPosePreset1, true);
                        setPathState(PathState.PRESET_SHOOTING);
                    }
                    break;

                case PRESET_SHOOTING:
                    if(!follower.isBusy()) {
                        telemetry.addLine("Shooting");
                    }
                    break;
                case INTAKE_PRESET3:
                    if (!follower.isBusy()) {
                        telemetry.addLine("Intaking Artifacts");
                        follower.followPath(drivePreset3PoseIntakePose , true);
                        setPathState(PathState.INTAKE_PRESET3);
                    }
                    break;
                case SHOOT_PRELOAD_PRESET3:
                    if(!follower.isBusy()) {
                        telemetry.addLine("Shooting");
                        follower.followPath(driveIntakePoseShootPosePreset3, true);
                    }
                    break;
                default:
                    telemetry.addLine("No State Commanded");
            }

        }

        public void setPathState (PathState newState){
            pathState = newState;
            pathTimer.resetTimer();
        }


        @Override
        public void init () {
            pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
            pathTimer = new Timer();
            opModeTimer = new Timer();
            follower = Constants.createFollower(hardwareMap);
            //TODO add in any other init mechanisms

            buildPaths();
            follower.setPose(startPose);
        }

        public void start () {
            opModeTimer.resetTimer();
            setPathState(pathState);
        }

        @Override
        public void loop () {

            follower.update();
            statePathUpdate();

            telemetry.addData("path state", pathState.toString());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        }


}
