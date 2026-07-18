package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms.TBWebcam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Limelight Turret Test")
public class AlignToAprilTagTurretTest extends OpMode {
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotor frontLeft;
    public DcMotor frontRight;
    private Limelight3A limelight;

    private AlignToAprilTagTurret turret = new AlignToAprilTagTurret();


    double[] stepSizes = {10,1,0.1,0.01,0.001,0.0001};

    int stepIndex = 1;

    @Override
    public void init() {

        turret.init(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        frontLeft = hardwareMap.get(DcMotor.class, "front-left");
        frontRight = hardwareMap.get(DcMotor.class, "front-right");
        backLeft = hardwareMap.get(DcMotor.class, "back-left");
        backRight = hardwareMap.get(DcMotor.class, "back-right");
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);


        limelight.pipelineSwitch(0); // AprilTag pipeline

        limelight.start();

        telemetry.addLine("Initialized");

    }

    @Override
    public void start(){
        turret.resetTimer();

    }

    @Override
    public void loop(){

        //----- Drivetrain -----

        double y   =  -gamepad1.left_stick_y; // Inverted because joysticks are negative when pushed up
        double x   =  gamepad1.left_stick_x; // Strafe
        double rx  =  gamepad1.right_stick_x; // Controls rotation

        // Calculate power for each wheel
        double frontLeftPower  = y + x + rx;
        double backLeftPower   = y - x + rx;
        double frontRightPower = y - x - rx;
        double backRightPower  = y + x - rx;

        // Scale powers proportionally if any value exceeds 1.0 (100% motor speed)
        double max = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

        if (max > 1.0) {
            frontLeftPower  /= max;
            backLeftPower   /= max;
            frontRightPower /= max;
            backRightPower  /= max;
        }

        // Apply calculated power values to motors
        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);


        //----- Tracking -----

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

        // Tune gains
        if(gamepad1.bWasPressed()){

            stepIndex++;

            if(stepIndex >= stepSizes.length)
                stepIndex = 0;
        }

        if(gamepad1.dpadLeftWasPressed()){
            turret.setkP(turret.getkP() - stepSizes[stepIndex]);
        }

        if(gamepad1.dpadRightWasPressed()){
            turret.setkP(turret.getkP() + stepSizes[stepIndex]);
        }

        if(gamepad1.dpadUpWasPressed()){
            turret.setkD(turret.getkD() + stepSizes[stepIndex]);
        }

        if(gamepad1.dpadDownWasPressed()){
            turret.setkD(turret.getkD() - stepSizes[stepIndex]);
        }



        telemetry.addLine("-----------------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad L/R)", turret.getkP());
        telemetry.addData("Tuning D", "%.4f (D-Pad U/D)", turret.getkD());
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

    }

    @Override
    public void stop(){
        limelight.stop();
    }
}