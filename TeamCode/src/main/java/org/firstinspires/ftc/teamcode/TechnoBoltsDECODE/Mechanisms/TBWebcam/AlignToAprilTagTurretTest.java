package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms.TBWebcam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.AlignToAprilTagTurret;
import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TechnoBoltsAprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp
public class AlignToAprilTagTurretTest extends OpMode {
    private TechnoBoltsAprilTagWebcam aprilTagWebcam = new TechnoBoltsAprilTagWebcam();
    private AlignToAprilTagTurret turret = new AlignToAprilTagTurret();


    double[] stepSizes = {10, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap, telemetry);
        turret.init(hardwareMap);

        telemetry.addLine("Initialized");
    }

    public void start(){
        turret.resetTimer();
    }

    @Override
    public void loop() {
        // vision logic
        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);

        turret.update(id24);



        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length; //Modulo wraps the index back to 0
        }

        // D-pad left/right adjusts the P gain
        if(gamepad1.dpadLeftWasPressed()){
            turret.setkP(turret.getkP() - stepSizes[stepIndex]);
        }
        if(gamepad1.dpadRightWasPressed()){
            turret.setkP(turret.getkP() + stepSizes[stepIndex]);
        }

        // D-pad up/down adjusts the P gain
        if(gamepad1.dpadUpWasPressed()){
            turret.setkD(turret.getkD() + stepSizes[stepIndex]);
        }
        if(gamepad1.dpadDownWasPressed()){
            turret.setkD(turret.getkD() - stepSizes[stepIndex]);
        }



        if(id24 != null){
            telemetry.addData("Cur ID", aprilTagWebcam);
        }
        else{
            telemetry.addLine("No Tag Detected, Stopping turret");
        }

        telemetry.addLine("-----------------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad L/R)", turret.getkP());
        telemetry.addData("Tuning D", "%.4f (D-Pad U/D)", turret.getkD());
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
    }
}
