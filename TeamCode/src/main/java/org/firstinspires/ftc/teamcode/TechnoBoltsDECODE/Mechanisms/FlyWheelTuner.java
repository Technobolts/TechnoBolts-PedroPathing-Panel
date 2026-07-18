package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms.TBWebcam.AlignToAprilTagTurret;

@TeleOp
public class
FlyWheelTuner extends OpMode {
    public DcMotorEx ShooterRight, ShooterLeft;
    private double CAMERA_HEIGHT_IN = 31.11500;
    private double GOAL_HEIGHT = 74.95;
    private double CAMERA_ANGLE = 18;
    private double distance = 0;
    public double TarVelocity = 1500;

    double HoodAngle = 0;

    public double getDistance(double ty){
        double angleTarget = CAMERA_ANGLE + ty;
        double heightDifference = GOAL_HEIGHT - CAMERA_HEIGHT_IN;

        return heightDifference / Math.tan(Math.toRadians(angleTarget));
    }

    double F = 1.2;

    double P = 350;

    double [] stepSizes = {1000.0, 100.0, 10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    private Servo TurretHood, Kicker;
    private Limelight3A limelight;

    private AlignToAprilTagTurret turret = new AlignToAprilTagTurret();
    @Override
    public void init() {

        ShooterRight = hardwareMap.get(DcMotorEx.class, "turretShooter");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        TurretHood = hardwareMap.get(Servo.class, "hoodShooter");
        Kicker = hardwareMap.get(Servo.class, "kickerServo");
        TurretHood.scaleRange(0.0, 0.9);
        TurretHood.setDirection(Servo.Direction.REVERSE);
        limelight.pipelineSwitch(0); // AprilTag pipeline
        turret.init(hardwareMap);
        limelight.start();

        ShooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0,0,F);

        ShooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        telemetry.addLine("Init Complete");

    }

    @Override
    public void loop() {

        TurretHood.setPosition(HoodAngle);



        if (gamepad1.a) {
            Kicker.setPosition(0.5);
        } else {
            Kicker.setPosition(0.15);
        }


        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadRightWasPressed()){
            TarVelocity += stepSizes[stepIndex];
        }
        if(gamepad1.dpadLeftWasPressed()){
            TarVelocity -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()){
            HoodAngle += 0.1;
        }
        if(gamepad1.dpadDownWasPressed()){
            HoodAngle -= 0.1;
        }


        LLResult result = limelight.getLatestResult();

        if(result != null && result.isValid()){

            // Horizontal angle from crosshair to tag
            double tx = result.getTx();

            turret.update(tx);

            telemetry.addData("tx", tx);
        }
        else{

            turret.stop();

            telemetry.addLine("No AprilTag");
        }

        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()){
            distance = getDistance(llResult.getTy()) / 2.54;
            telemetry.addData("Distance", distance);
        }
        else {
            telemetry.addData("No Valid Target", "Found");
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0,0,F);
        ShooterRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        ShooterRight.setVelocity(TarVelocity);
        double curVelocity1 = ShooterRight.getVelocity();
        double errorShooter = TarVelocity - curVelocity1;

        telemetry.addData( "Target Velocity", TarVelocity);
        telemetry.addData( "Current Velocity Right Wheel", "%.2f", curVelocity1);
        telemetry.addData( "Error Right",  "%.2f", errorShooter);
        telemetry.addLine("------------------------------------");
        telemetry.addData( "Change Target Velocity",  "%.4f (D-Pad L/R)", TarVelocity);
        telemetry.addData( "Change Hood Angle", "%.4f (D-Pad U/D)", HoodAngle);

        telemetry.addData( "Step Size",  "%.4f (B Button)", stepSizes[stepIndex]);

    }
}

// P = 350
// F = 1.2