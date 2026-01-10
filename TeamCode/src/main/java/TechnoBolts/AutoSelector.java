package TechnoBolts;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "Auto Selector")
public class AutoSelector extends OpMode {

    enum Alliance { BLUE, RED }
    enum StartPos { TOP, BOTTOM }

    Alliance alliance = Alliance.BLUE;
    StartPos startPos = StartPos.BOTTOM;

    Follower follower;

    public CRServo intake, lowerTServo, middleTServo;
    public DcMotorEx rightDeposit, leftDeposit;
    public Servo upperTServo, ledDepo;


    AutoTopRed topRedAuto;   // the top red auto
    AutoTopBlue topBlueAuto;   // the top blue auto


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        intake = hardwareMap.get(CRServo.class, "intake");     // Hardware map names
        rightDeposit = hardwareMap.get(DcMotorEx.class, "rightDeposit");
        leftDeposit = hardwareMap.get(DcMotorEx.class, "leftDeposit");
        upperTServo = hardwareMap.get(Servo.class, "upperTServo");
        lowerTServo = hardwareMap.get(CRServo.class, "lowerTServo");
        middleTServo = hardwareMap.get(CRServo.class, "middleTServo");
        ledDepo = hardwareMap.get(Servo.class, "ledDepo");


//        bottomBlueAuto = new AutoBottomBlue(follower, flip1, intake, launcher1, launcher2);
//        bottomRedAuto = new AutoBottomRed(follower, flip1, intake, launcher1, launcher2);
        topRedAuto = new AutoTopRed(follower, telemetry, intake, rightDeposit, leftDeposit, upperTServo, lowerTServo, middleTServo, ledDepo);
        topBlueAuto = new AutoTopBlue(follower, telemetry, intake, rightDeposit, leftDeposit, upperTServo, lowerTServo, middleTServo, ledDepo);
    }

    @Override
    public void init_loop() {

        telemetry.addLine("gamepad2X = BLUE");
        telemetry.addLine("gamepad2B = RED");
        telemetry.addLine("gamepad2A = BOTTOM");
        telemetry.addLine("gamepad2Y = TOP");
        telemetry.addLine("-------------------");

        if (gamepad2.x) alliance = Alliance.BLUE;   // alliance and color selections
        if (gamepad2.b) alliance = Alliance.RED;

        if (gamepad2.a) startPos = StartPos.BOTTOM;
        if (gamepad2.y) startPos = StartPos.TOP;


        telemetry.addData("Alliance", alliance);
        telemetry.addData("Start Position", startPos);
        telemetry.update();
    }

    @Override
    public void start() {
//        if (alliance == Alliance.BLUE && startPos == StartPos.BOTTOM) {   // starts the selected auto
//            bottomBlueAuto.start();
//        }
//        if (alliance == Alliance.RED && startPos == StartPos.BOTTOM) {
//            bottomRedAuto.start();
//        }
        if (alliance == Alliance.RED && startPos == StartPos.TOP) {
            topRedAuto.start();
        }
        if (alliance == Alliance.BLUE && startPos == StartPos.TOP) {
            topBlueAuto.start();
        }
    }

    @Override
    public void loop() {
//        if (alliance == Alliance.BLUE && startPos == StartPos.BOTTOM) {  // updates the selected auto
//            bottomBlueAuto.update();
//        }
//        if (alliance == Alliance.RED && startPos == StartPos.BOTTOM) {
//            bottomRedAuto.update();
//        }
        if (alliance == Alliance.RED && startPos == StartPos.TOP) {
            topRedAuto.update();
        }
        if (alliance == Alliance.BLUE && startPos == StartPos.TOP) {
            topBlueAuto.update();
        }
    }
}