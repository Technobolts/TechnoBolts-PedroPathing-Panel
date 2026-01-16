package TechnoBolts;

import static android.os.SystemClock.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.function.Supplier;


@TeleOp
@Configurable
public class TeleOpRed extends OpMode {

    private static final Logger log = LoggerFactory.getLogger(TechnoBolts.class);
    private Follower follower;
    public static Pose startingPose;    //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> Center;

    private Supplier<PathChain> FarRed;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    boolean wasReady = false;

    final double COLOR_YELLOW = 0.25;
    final double COLOR_GREEN = 0.45;
    private double slowModeMultiplier = 0.5;  // we don't use this

    private CRServo Intake, middleTServo, lowerTServo;
    private Servo upperTServo, ledDepo;  // servos
    private DcMotorEx rightDeposit, leftDeposit;  // DcMotors

    int intakeflag = 0;   // these are the flags
    int launchflag = 0;
    int parkflag = 0;
    int limeflag = 0;

    private double leftOn = 0.4;
    private double rightOn = -0.4;
    private int launcherOff = 0;
    private int intakeOn = 1;
    private int intakeOff = 0;
    private int intakeReverse = -1;
    private double flickDown = 0.8;
    private double flickUp = 0;
    private double lightGreen = 0.5;
    private double lightPurple = 0.722;
    private int lightOff = 0;
    private int rampOn = 1;
    private double rampOff = 0;

    double endGameStart;
    boolean isEndGame = false;
    double trackTimer;



    // --- PID constants ---
    public static double P = 0.02;    // these are the PID controls for the turret and limelight
    public static double I = 0.0;
    public static double D = 0.0;

    private double integral = 0;
    private double lastError = 0;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose(74.84444444444443, 78.11111111111111, Math.toRadians(225)) : startingPose);   // set where the robot starts in TeleOp
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Center = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(74.84444444444443, 78.11111111111111))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(225), 0.8))
                .build();
        FarRed = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(74.84444444444443, 78.11111111111111))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(225), 0.8))
                .build();





        Intake = hardwareMap.get(CRServo.class, "intake");     // Hardware map names
        rightDeposit = hardwareMap.get(DcMotorEx.class, "rightDeposit");
        rightDeposit.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftDeposit = hardwareMap.get(DcMotorEx.class, "leftDeposit");
        leftDeposit.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        upperTServo = hardwareMap.get(Servo.class, "upperTServo");
        lowerTServo = hardwareMap.get(CRServo.class, "lowerTServo");
        middleTServo = hardwareMap.get(CRServo.class, "middleTServo");
        ledDepo = hardwareMap.get(Servo.class, "ledDepo");


        telemetry.addLine("Initialized");




    }

    @Override
    public void start() {
        follower.startTeleopDrive();  // starts the driving
        endGameStart = getRuntime() + 103;
        trackTimer = getRuntime() + 15;
    }

    @Override
    public void loop() {


        follower.update();
        telemetryM.update();
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    gamepad1.left_stick_y * slowModeMultiplier,
                    gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        double leftVelocity = leftDeposit.getVelocity();
        double rightVelocity = rightDeposit.getVelocity();
        telemetry.update();

        // 1. Determine the current state
        boolean isReady = (leftVelocity >= 820 && leftVelocity <= 1000 &&
                rightVelocity <= -820 && rightVelocity >= -1000);

        // 2. Only update the LED if the state has CHANGED
        if (isReady != wasReady) {
            if (isReady) {
                ledDepo.setPosition(COLOR_GREEN);
            } else {
                ledDepo.setPosition(COLOR_YELLOW);
            }
            // 3. Update the tracker so we don't send the command again next loop
            wasReady = isReady;
        }
        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(Center.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }
        //Optional way to change slow mode strength
        //      if (gamepad1.xWasPressed()) {
        //         slowModeMultiplier += 0.25;
        //     }
        //Optional way to change slow mode strength
        //     if (gamepad2.yWasPressed()) {
        //         slowModeMultiplier -= 0.25;
        //     }

        upperTServo.setDirection(Servo.Direction.FORWARD);
        if (gamepad2.right_bumper) {
            upperTServo.setPosition(0.8); //Originally 0.5
        } else {
            upperTServo.setPosition(0);
        }



        if (gamepad2.aWasPressed()){
            if (intakeflag == 0){
                Intake.setPower(intakeOn);
                intakeflag = 1;
            }
            else if (intakeflag == 1) {
                Intake.setPower(intakeOff);
                intakeflag = 0;
            }
            else if (intakeflag == -1){
                Intake.setPower(intakeOff);
                intakeflag = 0;
            }
        }

        if (gamepad2.bWasPressed()) {
            if (intakeflag == 0){
                Intake.setPower(intakeReverse);
                intakeflag = -1;
            }
            else if(intakeflag == -1){
                Intake.setPower(intakeOff);
                intakeflag = 0;
            }
            else if(intakeflag == 1){
                Intake.setPower(intakeOff);
                intakeflag = 0;
            }
        }


        if (gamepad2.dpadUpWasPressed()) {
            if (launchflag == 0) {
                rightDeposit.setPower(rightOn);
                leftDeposit.setPower(leftOn);
                launchflag = 1;
                limeflag = 1;
            }
            else if (launchflag == 1) {
                rightDeposit.setPower(launcherOff);
                leftDeposit.setPower(launcherOff);
                launchflag = 0;
                limeflag = 0;
            }
        }
        if (gamepad2.dpadDownWasPressed()) {
            if (launchflag == 0) {
                rightDeposit.setPower(rightOn);
                leftDeposit.setPower(leftOn);
                launchflag = 1;
                limeflag = 1;
            } else if (launchflag == 1) {
                rightDeposit.setPower(launcherOff);
                leftDeposit.setPower(launcherOff);
                launchflag = 0;
                limeflag = 0;
            }
        }

        if(gamepad2.dpad_right){
            middleTServo.setPower(1);
            lowerTServo.setPower(-1);
        }
        if(gamepad2.dpad_down){
            middleTServo.setPower(0);
            lowerTServo.setPower(0);
        }
        if(gamepad2.dpad_left){
            middleTServo.setPower(-1);
            lowerTServo.setPower(1);
        }

        telemetry.addData("Y", follower.getPose().getY());
//        if (trackTimer <= getRuntime()) {
//            gamepad2.rumbleBlips(1);
//            trackTimer = getRuntime() + 15;
//        }
        telemetry.addData("X", follower.getPose().getX());

        telemetry.addData("Velocity left/Right", "%4.2f, %4.2f", leftVelocity, rightVelocity);

        telemetry.addData("Runtime", getRuntime());
        if (endGameStart <= getRuntime() && !isEndGame) {
//            gamepad1.rumble(5000);
//            gamepad2.rumble(5000);
            isEndGame = true;
        }

    }

}