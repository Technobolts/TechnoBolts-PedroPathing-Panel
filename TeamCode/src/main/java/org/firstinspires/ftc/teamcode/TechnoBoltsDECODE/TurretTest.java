package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

@TeleOp
@Configurable
public class TurretTest extends OpMode {
    private DcMotorEx turret;
    private Limelight3A limelight;

    // --- TUNING (Lowered P to stop oscillations) ---
    public static double P = 0.0006;
    public static double D = 0.0003;
    public static double TICKS_PER_DEGREE = 15.5;

    public static double SEARCH_POWER = 0.12;
    public static double HOLD_DEADZONE = 8; // Ticks of error to ignore

    private double targetTicks = 0;
    private double lastError = 0;
    private boolean isLocked = false;

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
                power = Math.max(-0.25, Math.min(0.25, power));
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
        telemetry.update();
    }
}
