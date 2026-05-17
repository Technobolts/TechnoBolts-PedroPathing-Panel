package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
@Configurable
public class TurretTest extends OpMode {
    private DcMotorEx turret;
    private Limelight3A limelight;

    // --- TUNING (Lowered P to stop oscillations) ---
    public static double P = -0.0026;
    public  static double I = 0.00001;
    public  static double F = 0.0;
    public static double D = 0.0058;
    public static double TICKS_PER_DEGREE = 15.5;

    public static double SEARCH_POWER = 0.12;
    public static double HOLD_DEADZONE = 8; // Ticks of error to ignore

    private double targetTicks = 0;
    private double lastError = 0;
    private boolean isLocked = false;

    double [] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad1.dpadRightWasPressed()){
            D += stepSizes[stepIndex];
        }
        if(gamepad1.dpadLeftWasPressed()){
            D -= stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }
        if(gamepad1.yWasPressed()){
            F += stepSizes[stepIndex];
        }
        if(gamepad1.aWasPressed()){
            F -= stepSizes[stepIndex];
        }

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, I,D,F);
        turret.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);


        LLResult result = limelight.getLatestResult();
        int currentTicks = turret.getCurrentPosition();

        // 1. DETECTION LOGIC
        if (result != null && result.isValid()) {
            // We see a target: Update the target coordinate
            double errorDegrees = result.getTx();
            targetTicks = currentTicks + (errorDegrees * TICKS_PER_DEGREE);
            isLocked = true;
        } else if (isLocked && !result.isValid()) {
            // We HAD a target, but lost it.
            // We stay isLocked = true to "HOLD" the last known position.
            // If the error stays near 0 for too long without seeing a tag,
            // you could set isLocked = false to resume searching.
        }

        // 2. MOVEMENT LOGIC
        if (isLocked) {
            double tickError = targetTicks - currentTicks;

            // If we are close enough, just stop (Deadzone stops oscillation)
            if (Math.abs(tickError) < HOLD_DEADZONE) {
                turret.setPower(0);
                lastError = 0;
                telemetry.addData("Status", "HOLDING POSITION");
            } else {
                // PID to move to and stay at targetTicks
                double derivative = tickError - lastError;
                double power = (P * tickError) + (D * derivative);

                // Limit power to prevent overshooting
                power = Math.max(-0.47, Math.min(0.47, power));
                turret.setPower(power);
                lastError = tickError;
                telemetry.addData("Status", "LOCKING...");
            }
        } else {
            // SEARCHING: Only moves if we have never seen a tag or explicitly lost lock
            turret.setPower(SEARCH_POWER);
            telemetry.addData("Status", "SEARCHING");
        }

        telemetry.addData("Target Ticks", (int)targetTicks);
        telemetry.addData("Current Ticks", currentTicks);
        telemetry.addLine("------------------------------------");
        telemetry.addData( "Tuning P",  "%.4f (D-Pad U/D)", P);
        telemetry.addData( "Tuning D", "%.4f (D-Pad L/R)", D);
        telemetry.addData( "Tuning F", "%.4f (Buttons Y/A)", F);
        telemetry.addData( "Step Size",  "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.update();
    }
}
// P = -0.0026
// D = 0.0058