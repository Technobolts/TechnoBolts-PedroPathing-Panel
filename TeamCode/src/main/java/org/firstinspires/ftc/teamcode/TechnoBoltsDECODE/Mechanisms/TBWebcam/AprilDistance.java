package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms.TBWebcam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TechnoBoltsAprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class AprilDistance extends OpMode {

    private final TechnoBoltsAprilTagWebcam aprilTagWebcam = new TechnoBoltsAprilTagWebcam();

    public DcMotorEx ShooterRight, ShooterLeft;
    public DcMotor Intake;
    public Servo upperTServo, DepoAngle;
    public CRServo lowerTServo, middleTServo;

    public double intakeOn = 1;
    public double intakeOff = 0;
    public double intakeflag = 0;


    private double curVelocity = 1;

    private double curAngle = 0;

    double FR = 12.22;
    double PR = 100.5;

    double FL = 12.62;
    double PL = 100.85;


    double [] stepSizes = {100.0, 10.0, 1.0, 0.1, 0.01, 0.001};

    int stepIndex = 1;


    @Override
    public void init() {

        aprilTagWebcam.init(hardwareMap, telemetry);
        upperTServo = hardwareMap.get(Servo.class, "upperTServo");
        lowerTServo = hardwareMap.get(CRServo.class, "lowerTServo");
        middleTServo = hardwareMap.get(CRServo.class, "middleTServo");
        Intake = hardwareMap.get(DcMotor.class, "intake");
        DepoAngle = hardwareMap.get(Servo.class,"depoAngle");
        ShooterLeft = hardwareMap.get(DcMotorEx.class, "leftDeposit");
        ShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterRight = hardwareMap.get(DcMotorEx.class, "rightDeposit");
        ShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficientsRight = new PIDFCoefficients(PR, 0,0,FR);
        PIDFCoefficients pidfCoefficientsLeft = new PIDFCoefficients(PL, 0,0,FL);
        ShooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsLeft);
        ShooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsRight);
        telemetry.addLine("Init Complete");

    }

    @Override
    public void loop() {

        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadUpWasPressed()){
            curVelocity += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            curVelocity -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()){
            curAngle += stepSizes[stepIndex];
        }
        if(gamepad1.dpadLeftWasPressed()){
            curAngle -= stepSizes[stepIndex];
        }



        PIDFCoefficients pidfCoefficientsRight = new PIDFCoefficients(PR, 0,0,FR);
        PIDFCoefficients pidfCoefficientsLeft = new PIDFCoefficients(PL, 0,0,FL);
        ShooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsLeft);
        ShooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsRight);

        ShooterLeft.setVelocity(curVelocity);
        ShooterRight.setVelocity(curVelocity);

        DepoAngle.setPosition(curAngle);

        double curVelocity2 = ShooterLeft.getVelocity();
        double curVelocity1 = ShooterRight.getVelocity();
        double curAngle1 = DepoAngle.getPosition();


        telemetry.addData( "Current Velocity Right Wheel", "%.2f", curVelocity1);
        telemetry.addData( "Current Velocity Left Wheel", "%.2f", curVelocity2);
        telemetry.addData("Current Angle", curAngle1);
//        telemetry.addData( "Error Right",  "%.2f", errorRight);
//        telemetry.addData( "Error Left",  "%.2f", errorLeft);
//        telemetry.addLine("------------------------------------");
//        telemetry.addData( "Tuning P",  "%.4f (D-Pad U/D)", P);
        telemetry.addData( "SPEED", "%.4f (D-Pad U/D)", curVelocity);
        telemetry.addData( "Step Size",  "%.4f (B Button)", stepSizes[stepIndex]);

        if (id24 != null){
            aprilTagWebcam.displayDetectionTelemetry(id24);
        }
        else {
            telemetry.addLine("NOT Detected");
        }

    }
}

