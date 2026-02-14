package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.function.Supplier;
@Configurable


@TeleOp(name="Chassis - TechnoBolts Manual Run - Final", group="TechnoBolts - OpMode")
public class TechnoBolts extends OpMode {
    private static final Logger log = LoggerFactory.getLogger(TechnoBolts.class);
    //private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

    public DcMotor Intake = null;

    public Servo ledDepo;
    boolean wasReady = false;
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    final double COLOR_RED = 0.05;
    final double COLOR_ORANGE = 0.13;
    final double COLOR_YELLOW = 0.25;
    final double COLOR_GREEN = 0.45;
    final double COLOR_BLUE = 0.65;
    final double COLOR_PURPLE = 0.85;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder()
                .addPath(new BezierLine(new Pose(89.244, 4.622), new Pose(72.178, 87.822)))
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
                .build();
    }


    @Override
    public final void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {

        follower.update();
        telemetryM.update();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        Intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotorEx myMotorLeft = hardwareMap.get(DcMotorEx.class, "leftDeposit");
        myMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        DcMotorEx myMotorRight = hardwareMap.get(DcMotorEx.class, "rightDeposit");
        myMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //RevBlinkinLedDriver ledDepo = hardwareMap.get(RevBlinkinLedDriver.class, "ledDepo");
        Servo ledDepo = hardwareMap.get(Servo.class, "ledDepo");
        Servo upperTServo = hardwareMap.get(Servo.class, "upperTServo");
        CRServo lowerTServo = hardwareMap.get(CRServo.class, "lowerTServo");
        CRServo middleTServo = hardwareMap.get(CRServo.class, "middleTServo");
        ledDepo = hardwareMap.get(Servo.class, "ledDepo");

        // ########################################################################################
        // !!!!            IMPORTANT Drive Information. Test your motor directions.            !!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);




        // runtime.reset();

//        while (gamepad1.right_bumper){
//            upperTServo.setPosition(1);
//        }
        double max;

        double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftBackPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightBackPower = (y + x - rx) / denominator;


        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));


        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }


        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);


        // Show the elapsed game time and wheel power.



        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Auto-TeleOP Code

        if (gamepad1.xWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }
        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }


        //Outtake Code


//                    while (power >= 0.25) {
//                        ledDepo.setPower(1);
//                        myMotorLeft.setPower(-0.25); // Set the motor power
//                        myMotorRight.setPower(0.25);
//                        power = 0.25;
//                    }
        if(gamepad2.left_bumper){
            myMotorRight.setVelocity(-900);
            myMotorLeft.setVelocity(900);
        }
        else {
            myMotorLeft.setPower(0); // Set the motor power
            myMotorRight.setPower(0);
        }
            double leftVelocity = myMotorLeft.getVelocity();
            double rightVelocity = myMotorRight.getVelocity();
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


//                    else {
//                    myMotorLeft.setPower(0);
//                    myMotorRight.setPower(0);
//                }
//                telemetry.addData("Motor Power");
//                telemetry.update();
        //Intake Code

//        if (gamepad2.dpad_up)
//            Intake.setPower(-1);
//        if (gamepad2.dpad_down)
//            Intake.setPower(0);

        if(gamepad2.dpad_up) {
            Intake.setPower(-1);
        }
        if(gamepad2.dpad_down) {
            Intake.setPower(0);
        }

        //Transfer system code



        if(gamepad2.b){
            middleTServo.setPower(1);
            lowerTServo.setPower(-1);
        }
        if(gamepad2.a){
            middleTServo.setPower(0);
            lowerTServo.setPower(0);
        }
        if(gamepad2.x){
            middleTServo.setPower(-1);
            lowerTServo.setPower(1);
        }

        upperTServo.setDirection(Servo.Direction.FORWARD);
        if (gamepad2.right_bumper) {
            upperTServo.setPosition(0.8); //Originally 0.5
        } else {
            upperTServo.setPosition(0);
        }


        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
       // telemetry.addData("Outtake left/Right", "%4.2f, %4.2f", myMotorLeft, myMotorRight);
        telemetry.addData("Velocity left/Right", "%4.2f, %4.2f", leftVelocity, rightVelocity);
        telemetry.update();

    }

}
