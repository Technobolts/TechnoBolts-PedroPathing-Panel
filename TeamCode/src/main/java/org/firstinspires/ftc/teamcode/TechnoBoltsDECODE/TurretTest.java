package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;


@TeleOp
@Configurable
public class TurretTest extends OpMode {

    int limeflag = 0;

    double endGameStart;
    boolean isEndGame = false;
    double trackTimer;


    private DcMotorEx turret;    // turret
    private Limelight3A limelight;  // limelight

    // --- PID constants ---
    public static double P = 0.04;    // these are the PID controls for the turret and limelight
    public static double I = 0.000000001;
    public static double D = 0.05;

    private double integral = 0;
    private double lastError = 0;

    @Override
    public void init() {


        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");


        telemetry.addLine("Initialized");



    }

    @Override
    public void start() {
        limelight.start();   // starts the limelight
        limelight.pipelineSwitch(0);  // pipeline 1 is for blue tracking
        endGameStart = getRuntime() + 103;
        trackTimer = getRuntime() + 15;

    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            if (limeflag == 1) {
                // Error is just tx straight from Limelight
                double error = result.getTx();

                // Basic PID
                integral += error;
                double derivative = error - lastError;

                double power = P * error + I * integral + D * derivative;


                turret.setPower(power);

                lastError = error;

                telemetry.addData("tx", error);
                telemetry.addData("power", power);
            }
        } else {
            // No target -> stop motor
            turret.setPower(0);
            telemetry.addLine("No Target");
        }

        telemetry.update();


    }
}