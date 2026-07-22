package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms.TBWebcam;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class AlignToAprilTagTurret {


    private DcMotorEx turret;

    // ---------------- PD Controller ----------------
    private double kP = 0.1201; //0.1101
    private double kD = 0.0002;

    private double goalAngle = 0;
    private double lastError = 0;

    private final double ANGLE_TOLERANCE = 0.2;
    private final double MAX_POWER = 0.6;

    private final ElapsedTime timer = new ElapsedTime();



    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turretMotor");
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
// This is used in the auto
    public void init(DcMotorEx turret)
    {
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
    public void resetTimer() {
        timer.reset();
    }

    // ---------------- PD Update ----------------

    public double update(double currentBearing) {

        double dt = timer.seconds();
        timer.reset();

        double error = goalAngle - currentBearing;

        double pTerm = error * kP;

        double dTerm = 0;

        if (dt > 0) {
            dTerm = ((error - lastError) / dt) * kD;
        }

        double power;

        if (Math.abs(error) < ANGLE_TOLERANCE) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm,
                    -MAX_POWER,
                    MAX_POWER);
        }

        turret.setPower(power);

        lastError = error;
        return dt;
    }

    //USED IN AUTO
public double update(double currentBearing,DcMotorEx turret) {

        double dt = timer.seconds();
        timer.reset();

        double error = goalAngle - currentBearing;

        double pTerm = error * kP;

        double dTerm = 0;

        if (dt > 0) {
            dTerm = ((error - lastError) / dt) * kD;
        }

        double power;

        if (Math.abs(error) < ANGLE_TOLERANCE) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm,
                    -MAX_POWER,
                    MAX_POWER);
        }

        turret.setPower(power);

        lastError = error;
        return dt;
    }

    public void stop() {
        turret.setPower(0);
        lastError = 0;
    }

    public void stop(DcMotorEx turret) {
        turret.setPower(0);
        lastError = 0;
    }

    // ---------------- PD Getters/Setters ----------------

    public void setkP(double p) {
        kP = p;
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double d) {
        kD = d;
    }

    public double getkD() {
        return kD;
    }

    public DcMotorEx getTurret() {return turret;}
}

// kp = 0.1101
// kd = 0.003