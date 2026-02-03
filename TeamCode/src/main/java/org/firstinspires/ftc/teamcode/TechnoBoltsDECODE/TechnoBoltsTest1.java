//package TechnoBolts;
//
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import java.util.function.Supplier;
//
//@Configurable
//@Autonomous(name="TechnoBoltsTEST1", group="TechnoBolts - OpMode")
//public class TechnoBoltsTest1 extends OpMode {
//    // PUBLIC HARDWARE DECLARATIONS-
//    public DcMotor leftFrontDrive = null;
//    public DcMotor leftBackDrive = null;
//    public DcMotor rightFrontDrive = null;
//    public DcMotor rightBackDrive = null;
//    public CRServo Intake = null;
//    public DcMotor myMotorLeft = null;  // Matches your requested name
//    public DcMotor myMotorRight = null; // Matches your requested name
//    public CRServo ledDepo = null;
//    public Servo upperTServo = null;
//    public CRServo lowerTServo = null;
//    public CRServo middleTServo = null;
//
//    // PUBLIC RPM LOGIC VARIABLES
//    public ElapsedTime rpmTimer = new ElapsedTime();
//    public int lastLeftPos = 0;
//    public int lastRightPos = 0;
//    public double lastTime = 0;
//    public final double TICKS_PER_REV = 28.0;
//    public final double TARGET_RPM = 4500.0;
//    public final double TOLERANCE = 200.0;
//
//    // PEDRO PATHING & TELEMETRY
//    public Follower follower;
//    public static Pose startingPose;
//    public boolean automatedDrive;
//    public Supplier<PathChain> pathChain;
//    public TelemetryManager telemetryM;
//    public boolean slowMode = false;
//    public double slowModeMultiplier = 0.5;
//
//    @Override
//    public void init() {
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
//        follower.update();
//        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
//
//        // HARDWARE MAPPING (Matches your configuration names)
//        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
//        Intake = hardwareMap.get(CRServo.class, "intake");
//        myMotorLeft = hardwareMap.get(DcMotor.class, "leftDeposit");
//        myMotorRight = hardwareMap.get(DcMotor.class, "rightDeposit");
//        ledDepo = hardwareMap.get(CRServo.class, "ledDepo");
//      // upperTServo = hardwareMap.get(Servo.class, "upperTServo");
//        lowerTServo = hardwareMap.get(CRServo.class, "lowerTServo");
//        middleTServo = hardwareMap.get(CRServo.class, "middleTServo");
//
//
//        // MOTOR DIRECTIONS
//        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
//
//        pathChain = () -> follower.pathBuilder()
//                .addPath(
//                        new BezierLine(new Pose(89.244, 4.622), new Pose(72.178, 87.822)))
//                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(45))
//                .build();
//
//        rpmTimer.reset();
//    }
//
//    @Override
//    public final void start() {
//        follower.startTeleopDrive();
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        telemetryM.update();
//
//        // --- RPM DETECTION AND LED FEEDBACK LOGIC ---
//        double currentTime = rpmTimer.seconds();
//        double deltaTime = currentTime - lastTime;
//
//
//
//        // --- CHASSIS DRIVE LOGIC ---
//        double max;
//        double y = gamepad1.left_stick_y;
//        double x = -gamepad1.left_stick_x;
//        double rx = gamepad1.right_stick_x;
//
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
//        double leftFrontPower = (y + x + rx) / denominator;
//        double leftBackPower = (y - x + rx) / denominator;
//        double rightFrontPower = (y - x - rx) / denominator;
//        double rightBackPower = (y + x - rx) / denominator;
//
//        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
//        max = Math.max(max, Math.abs(leftBackPower));
//        max = Math.max(max, Math.abs(rightBackPower));
//
//        if (max > 1.0) {
//            leftFrontPower /= max;
//            rightFrontPower /= max;
//            leftBackPower /= max;
//            rightBackPower /= max;
//        }
//
//        leftFrontDrive.setPower(leftFrontPower);
//        rightFrontDrive.setPower(rightFrontPower);
//        leftBackDrive.setPower(leftBackPower);
//        rightBackDrive.setPower(rightBackPower);
//
//
//        //Outtake Code
//
//
////                    while (power >= 0.25) {
////                        ledDepo.setPower(1);
////                        myMotorLeft.setPower(-0.25); // Set the motor power
////                        myMotorRight.setPower(0.25);
////                        power = 0.25;
////                    }
//        if(gamepad2.dpad_up){
//            myMotorRight.setPower(-0.75);
//            myMotorLeft.setPower(0.75);
//        }
//        ledDepo.setPower(0);
//        myMotorLeft.setPower(0); // Set the motor power
//        myMotorRight.setPower(0);
//
////                    else {
////                    myMotorLeft.setPower(0);
////                    myMotorRight.setPower(0);
////                }
////                telemetry.addData("Motor Power");
////                telemetry.update();
//
//        if (gamepad2.dpad_down) {
//            myMotorLeft.setPower(0);
//            myMotorRight.setPower(0);// Stop the motor when the OpMode ends
//        }
//        //Intake Code
//
//        if (gamepad2.a)
//            Intake.setPower(-1);
//        if (gamepad2.b)
//            Intake.setPower(0);
//
//        //Transfer system code
//
//
//        if(gamepad2.x){
//            middleTServo.setPower(1);
//            lowerTServo.setPower(-1);
//        }
//        if(gamepad2.y){
//            middleTServo.setPower(0);
//            lowerTServo.setPower(0);
//        }
//
//        upperTServo.setDirection(Servo.Direction.REVERSE);
//        if(gamepad2.right_bumper ==true){
//            upperTServo.setPosition(0.65); //Originally 0.5
//        }
//        else{
//            upperTServo.setPosition(0);
//        }
//        // BRAKE BEHAVIOR
//        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//        telemetry.update();
//
//        if (deltaTime >= 0.05) { // Calculate every 50ms
//            int currentLeftPos = myMotorLeft.getCurrentPosition();
//            int currentRightPos = myMotorRight.getCurrentPosition();
//
//            // Formula: ((Change in Ticks) / TicksPerRev) * (60s / TimeInSeconds)
//            double leftRPM = ((currentLeftPos - lastLeftPos) / TICKS_PER_REV) * (60.0 / deltaTime);
//            double rightRPM = ((currentRightPos - lastRightPos) / TICKS_PER_REV) * (60.0 / deltaTime);
//
//            boolean isReady = (Math.abs(leftRPM) >= TARGET_RPM - TOLERANCE) &&
//                    (Math.abs(rightRPM) >= TARGET_RPM - TOLERANCE);
//
//            // Set Headlight power based on target
//            ledDepo.setPower(isReady ? 1.0 : 0.0);
//
//            // Update for next loop
//            lastLeftPos = currentLeftPos;
//            lastRightPos = currentRightPos;
//            lastTime = currentTime;
//
//            telemetry.addData("Outtake Status", isReady ? "READY" : "SPOOLING");
//            telemetry.addData("Left RPM", Math.round(leftRPM));
//            telemetry.addData("Right RPM", Math.round(rightRPM));
//        }
//    }
//}
