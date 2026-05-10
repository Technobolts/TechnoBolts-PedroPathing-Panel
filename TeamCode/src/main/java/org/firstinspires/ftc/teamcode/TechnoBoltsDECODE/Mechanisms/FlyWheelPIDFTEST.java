package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FlyWheelPIDFTEST extends OpMode {
    public DcMotorEx ShooterRight, ShooterLeft;
    public Servo upperTServo;
    public CRServo lowerTServo, middleTServo;
    public DcMotor Intake;

    public double intakeOn = 1;
    public double intakeOff = 0;
    public double intakeflag = 0;

    public double highVelocity = 1500;

    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double FR = 11.873;
    double PR = 36.131;

    double FL = 12.62;
    double PL = 100.85;


    double [] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;


    @Override
    public void init() {

        upperTServo = hardwareMap.get(Servo.class, "upperTServo");
        lowerTServo = hardwareMap.get(CRServo.class, "lowerTServo");
        middleTServo = hardwareMap.get(CRServo.class, "middleTServo");
        Intake = hardwareMap.get(DcMotor.class, "intake");

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

        if (gamepad1.yWasPressed()){
            if (curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else{ curTargetVelocity = highVelocity;}
        }

        if (gamepad2.right_bumper) {
            upperTServo.setPosition(0.8); //Originally 0.5
        } else {
            upperTServo.setPosition(0);
        }

        if(gamepad2.dpad_right){
            middleTServo.setPower(1);
            lowerTServo.setPower(-1);
        }
        if(gamepad2.dpad_down){
            middleTServo.setPower(0);
            lowerTServo.setPower(0);
        }
        if(gamepad2.dpad_left) {
            middleTServo.setPower(-1);
            lowerTServo.setPower(1);
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
            if (intakeflag == 0) {
                Intake.setPower(-1);
                intakeflag = -1;
            } else if (intakeflag == -1) {
                Intake.setPower(intakeOff);
                intakeflag = 0;
            } else if (intakeflag == 1) {
                Intake.setPower(intakeOff);
                intakeflag = 0;
            }
        }

        PIDFCoefficients pidfCoefficientsRight = new PIDFCoefficients(PR, 0,0,FR);
        PIDFCoefficients pidfCoefficientsLeft = new PIDFCoefficients(PL, 0,0,FL);
        ShooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsLeft);
        ShooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficientsRight);

        ShooterLeft.setVelocity(curTargetVelocity);
        ShooterRight.setVelocity(curTargetVelocity);

        double curVelocity2 = ShooterLeft.getVelocity();
        double curVelocity1 = ShooterRight.getVelocity();

        double errorLeft = curTargetVelocity - curVelocity2;
        double errorRight = curTargetVelocity - curVelocity1;


        telemetry.addData( "Target Velocity", curTargetVelocity);
        telemetry.addData( "Current Velocity Right Wheel", "%.2f", curVelocity1);
        telemetry.addData( "Current Velocity Left Wheel", "%.2f", curVelocity2);
        telemetry.addData( "Error Right",  "%.2f", errorRight);
        telemetry.addData( "Error Left",  "%.2f", errorLeft);
//        telemetry.addLine("------------------------------------");
//        telemetry.addData( "Tuning P",  "%.4f (D-Pad U/D)", P);
//        telemetry.addData( "Tuning F", "%.4f (D-Pad L/R)", F);
//        telemetry.addData( "Step Size",  "%.4f (B Button)", stepSizes[stepIndex]);



    }
}

