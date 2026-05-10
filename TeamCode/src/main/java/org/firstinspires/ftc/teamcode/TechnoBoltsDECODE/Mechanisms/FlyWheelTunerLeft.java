package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp

public class FlyWheelTunerLeft extends OpMode {
    public DcMotorEx ShooterRight, ShooterLeft;

    public double highVelocity = 1500;

    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;

    double P = 0;

    double [] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;


    @Override
    public void init() {
        ShooterLeft = hardwareMap.get(DcMotorEx.class, "leftDeposit");
        ShooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        ShooterRight = hardwareMap.get(DcMotorEx.class, "rightDeposit");
//        ShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        ShooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0,0,F);
        ShooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
//        ShooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        telemetry.addLine("Init Complete");

    }

    @Override
    public void loop() {

        if (gamepad1.yWasPressed()){
            if (curTargetVelocity == highVelocity){
                curTargetVelocity = lowVelocity;
            } else{ curTargetVelocity = highVelocity;}
        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }
        if(gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0,0,F);
        ShooterLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
//        ShooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

        ShooterLeft.setVelocity(curTargetVelocity);
//        ShooterRight.setVelocity(curTargetVelocity);

        double curVelocity2 = ShooterLeft.getVelocity();
//        double curVelocity1 = ShooterRight.getVelocity();

        double errorLeft = curTargetVelocity - curVelocity2;
//        double errorRight = curTargetVelocity - curVelocity1;


        telemetry.addData( "Target Velocity", curTargetVelocity);
//        telemetry.addData( "Current Velocity Right Wheel", "%.2f", curVelocity1);
        telemetry.addData( "Current Velocity Left Wheel", "%.2f", curVelocity2);
//        telemetry.addData( "Error Right",  "%.2f", errorRight);
        telemetry.addData( "Error Left",  "%.2f", errorLeft);
        telemetry.addLine("------------------------------------");
        telemetry.addData( "Tuning P",  "%.4f (D-Pad U/D)", P);
        telemetry.addData( "Tuning F", "%.4f (D-Pad L/R)", F);
        telemetry.addData( "Step Size",  "%.4f (B Button)", stepSizes[stepIndex]);

    }
}

// P = 100.8500
// F = 12.6300