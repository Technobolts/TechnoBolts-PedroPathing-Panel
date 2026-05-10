package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TeleOp;

//import static org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TechnoBoltsAprilTagWebcam.flywheelSpeed;

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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TechnoBolts;
import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TechnoBoltsAprilTagWebcam;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import java.util.function.Supplier;
@TeleOp
@Configurable
public class TeleOpRedFar extends OpMode {

    private final TechnoBoltsAprilTagWebcam aprilTagWebcam = new TechnoBoltsAprilTagWebcam();

    private static final Logger log = LoggerFactory.getLogger(TechnoBolts.class);
    private Follower follower;
    public static Pose startingPose;    //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> Center, CloseRed, FarRed, InTri;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;

    boolean wasReady = false;

    final double COLOR_YELLOW = 0.25;
    final double COLOR_GREEN = 0.45;
    private double slowModeMultiplier = 0.5;  // we don't use this

    private CRServo upperTServo, middleTServo, lowerTServo;
    private DcMotor Intake;
    private Servo ledDepo;  // servos
    private DcMotorEx rightDeposit, leftDeposit;  // DcMotors
    int intakeflag = 0;   // these are the flags
    int launchflag = 0;
    int parkflag = 0;
    int limeflag = 0;
    private int launcherOff = 0;
    private double intakeOn = -0.6;
    private int intakeOff = 0;
    private double intakeReverse = 0.1;
    private double flickDown = 0.8;
    private double flickUp = 0;
    private double lightGreen = 0.5;
    private double lightPurple = 0.722;
    private int lightOff = 0;

    double endGameStart;
    boolean isEndGame = false;
    double trackTimer;
    // --- PID constants ---
    double FR = 12.22;
    double PR = 100.5;

    double FL = 12.62;
    double PL = 100.85;

    double kP = 0.03;
    double error = 0;
    double lastError = 0;
    double goalX = 0; //offset here
    double angleTolerance = 0.1;
    double kD = 0.0004;
    double curTime = 0;
    double lastTime = 0;

    double forward, strafe, rotate;

    public DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;

    @Override
    public void init() {


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose(100.26666666666667, 24.53333333333334, 0): startingPose);   // set where the robot starts in TeleOp
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Center = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(74.84444444444443, 78.11111111111111))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(225), 0.8))
                .build();
        CloseRed = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(85.13333333333333, 84.91111111111108))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(225), 0.8))
                .build();
        FarRed = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(90.8655666, 24.48555))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(255), 0.8))
                .build();
        InTri = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(50.2232412, 117.251333334))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(200), 0.8))
                .build();


        aprilTagWebcam.init(hardwareMap,telemetry);
        Intake = hardwareMap.get(DcMotor.class, "intake");// Hardware map names
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDeposit = hardwareMap.get(DcMotorEx.class, "rightDeposit");
        rightDeposit.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightDeposit.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDeposit = hardwareMap.get(DcMotorEx.class, "leftDeposit");
        leftDeposit.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        upperTServo = hardwareMap.get(CRServo.class, "Kicker");
        lowerTServo = hardwareMap.get(CRServo.class, "lowerTServo");
        middleTServo = hardwareMap.get(CRServo.class, "middleTServo");
        ledDepo = hardwareMap.get(Servo.class, "ledDepo");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDFCoefficients pidfCoefficientsRight = new PIDFCoefficients(PR, 0,0,FR);
        PIDFCoefficients pidfCoefficientsLeft = new PIDFCoefficients(PL, 0,0,FL);
        leftDeposit.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsLeft);
        rightDeposit.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsRight);


        telemetry.addLine("Initialized");

    }

    @Override
    public void start() {
        resetRuntime();
        curTime = getRuntime();
        follower.startTeleopDrive();  // starts the driving
        endGameStart = getRuntime() + 103;
        trackTimer = getRuntime() + 15;
    }
    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;


        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);

        if (gamepad2.left_bumper == true) {
            if (id24 != null) {
                error = goalX - id24.ftcPose.bearing; // tx

                if (Math.abs(error) < angleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = error * kP;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((error - lastError) / dT) * kD;

                    rotate = -(Range.clip(pTerm + dTerm, -0.4, 0.4));

                    lastError = error;
                    lastTime = curTime;
                }
            } else {
                lastTime = getRuntime();
                lastTime = 0;

            }
        } else {
            lastError = 0;
            lastTime = getRuntime();
        }
        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors
            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(forward, strafe, rotate,
                    true // Robot Centric
            );
                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    forward * slowModeMultiplier,
                    strafe * slowModeMultiplier,
                    rotate * slowModeMultiplier,
                    true // Robot Centric
            );
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

        if (gamepad1.xWasPressed()){
            follower.followPath(CloseRed.get());
            automatedDrive = true;
        }

        if(gamepad1.yWasPressed()) {
            follower.followPath(FarRed.get());
            automatedDrive = true;
        }

        if(gamepad1.leftBumperWasPressed()) {
            follower.followPath(InTri.get());
            automatedDrive = true;
        }
        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

// __________________________________________________





        if(gamepad2.right_bumper) {
            upperTServo.setPower(1);
        }
        else {
            upperTServo.setPower(-0.3);
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
        if (gamepad2.dpadUpWasPressed()) {
            if (launchflag == 0  && id24 != null) {
                rightDeposit.setVelocity(aprilTagWebcam.flywheelSpeed(id24.ftcPose.y));
                leftDeposit.setVelocity(aprilTagWebcam.flywheelSpeed(id24.ftcPose.y));
                launchflag = 1;
            }
            else if (launchflag == 0 && id24 == null){
                rightDeposit.setVelocity(760);
                leftDeposit.setVelocity(760);
                launchflag = 1;
            }
            else if (launchflag == 1) {
                rightDeposit.setPower(launcherOff);
                leftDeposit.setPower(launcherOff);
                launchflag = 0;
            }
        }

        double curVelocity2 = leftDeposit.getVelocity();
        double curVelocity1 = rightDeposit.getVelocity();


        if (id24 != null) {
            double errorLeft = aprilTagWebcam.flywheelSpeed(id24.ftcPose.y) - curVelocity2;
            double errorRight = aprilTagWebcam.flywheelSpeed(id24.ftcPose.y) - curVelocity1;

            leftDeposit.setVelocity(leftDeposit.getVelocity() + errorLeft);
            rightDeposit.setVelocity(rightDeposit.getVelocity() + errorRight);

            telemetry.update();

            // 1. Determine the current state
            boolean isReady = (errorLeft >= -20 && errorLeft <= 40 &&
                    errorRight >= -20 && errorRight <= 40);

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
            telemetry.addData("Error Right", "%6.1f", errorRight);
            telemetry.addData("Error Left", "%6.1f", errorLeft);

        }
        else if (id24 == null){
            double errorLeft = 840 - curVelocity2;
            double errorRight = 840 - curVelocity1;

            leftDeposit.setVelocity(leftDeposit.getVelocity() + errorLeft);
            rightDeposit.setVelocity(rightDeposit.getVelocity() + errorRight);

            telemetry.update();

            boolean isReady = (errorLeft >= -20 && errorLeft <= 40 &&
                    errorRight >= -20 && errorRight <= 40);

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

            telemetry.addData("Error Right", "%6.1f", errorRight);
            telemetry.addData("Error Left", "%6.1f", errorLeft);
        }

        else {
            double errorLeft = 0;
            double errorRight = 0;
            telemetry.addData("Error Right", "%6.1f", errorRight);
            telemetry.addData("Error Left", "%6.1f", errorLeft);
        }


        double leftVelocity = leftDeposit.getVelocity();
        double rightVelocity = rightDeposit.getVelocity();

        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Velocity left/Right", "%6.1f, %6.1f", leftVelocity, rightVelocity);
        telemetry.addData("Runtime", getRuntime());
        telemetry.addLine("-------------------------------------------");
        if (id24 != null) {
            if (gamepad2.left_trigger > 0.3) {
                telemetry.addLine("AUTO ALIGN");
            }
            aprilTagWebcam.displayDetectionTelemetry(id24);
            telemetry.addData("Error", error);
        } else {
            telemetry.addLine("MANUAL Rotate Mode");
        }

        if (endGameStart <= getRuntime() && !isEndGame) {
//            gamepad1.rumble(5000);
//            gamepad2.rumble(5000);
            isEndGame = true;
        }

    }

}