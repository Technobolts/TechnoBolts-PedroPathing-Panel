package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class AlignToAprilTagTurret {

    private DcMotor turret;
    private double kP = 0.0001;
    private double kD = 0.0000;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.2;
    private final double MAX_POWER = 0.6;
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap){
        turret = hwMap.get(DcMotor.class, "turret");
    }

    public void setkP(double newkP){
        kP = newkP;
    }

    public double getkP(){
        return kP;
    }
    public void setkD(double newkD){
        kD = newkD;
    }
    public double getkD(){
        return kD;
    }

    public void resetTimer(){
        timer.reset();
    }

    public void update(AprilTagDetection curID){
        double deltaTime = timer.seconds();
        timer.reset();

        if(curID == null){
            turret.setPower(0);
            lastError = 0;
            return;
        }

        // -------------- PD Controller ---------------

        double error = goalX - curID.ftcPose.bearing;
        double pTerm = error * kP;

        double dTerm = 0;
        if(deltaTime > 0){
            dTerm = ((error - lastError) / deltaTime) * kD;
        }

        if(Math.abs(error) < angleTolerance){
            power = 0;
        }
        else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        turret.setPower(power);
        lastError = error;

    }

}
