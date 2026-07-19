package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
@TeleOp
        (name = "LimelightDistanceTest")
public class AprilTagLimelightTest extends OpMode {
    private Limelight3A limelight;

    private double CAMERA_HEIGHT_IN = 31.11500;
    private double GOAL_HEIGHT = 74.95;
    private double CAMERA_ANGLE = 18;
    private double distance = 0;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); //april tag

    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();

        if (llResult != null && llResult.isValid()){
            distance = getDistance(llResult.getTy()) / 2.54;
            telemetry.addData("Distance", distance);
        }
        else {
            telemetry.addData("No Valid Target", "Found");
        }
    }

    public double getDistance(double ty){
        double angleTarget = CAMERA_ANGLE + ty;
        double heightDifference = GOAL_HEIGHT - CAMERA_HEIGHT_IN;

        return heightDifference / Math.tan(Math.toRadians(angleTarget));
    }
}
