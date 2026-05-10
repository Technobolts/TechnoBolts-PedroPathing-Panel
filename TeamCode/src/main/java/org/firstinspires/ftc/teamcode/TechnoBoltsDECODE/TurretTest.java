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

    public static double P = 0.0006;
    public static double D = 0.0003;
    public static double TICKS_PER_DEGREE = 15.5;

    public static double SEARCH_POWER = 0.12;
    public static double HOLD_DEADZONE = 8;
    // How far the turret can drift from target before we assume manual override
    public static double MANUAL_OVERRIDE_THRESHOLD = 80;

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
        double tickError = targetTicks - currentTicks;

        // 1. MANUAL OVERRIDE CHECK
        // If we're locked but the turret was physically pushed far off target,
        // assume manual intervention and resume searching.
        if (isLocked && Math.abs(tickError) > MANUAL_OVERRIDE_THRESHOLD) {
            isLocked = false;
            lastError = 0;
            telemetry.addData("Event", "Manual override detected — resuming search");
        }

        // 2. DETECTION LOGIC
        if (result != null && result.isValid()) {
            if (!isLocked) {
                // Acquire target ONCE — don't keep recalculating while locked
                double errorDegrees = result.getTx();
                targetTicks = currentTicks + (errorDegrees * TICKS_PER_DEGREE);
                isLocked = true;
                lastError = 0;
            }
            // If already locked, ignore new Limelight data to avoid drift
        }
        // If result is null or invalid and isLocked, hold the last known targetTicks

        // 3. MOVEMENT LOGIC
        // Recalculate tickError after possible targetTicks update
        tickError = targetTicks - currentTicks;

        if (isLocked) {
            if (Math.abs(tickError) < HOLD_DEADZONE) {
                turret.setPower(0);
                lastError = 0;
                telemetry.addData("Status", "HOLDING POSITION");
            } else {
                double derivative = tickError - lastError;
                double power = (P * tickError) + (D * derivative);
                power = Math.max(-0.25, Math.min(0.25, power));
                turret.setPower(power);
                lastError = tickError;
                telemetry.addData("Status", "LOCKING...");
            }
        } else {
            turret.setPower(SEARCH_POWER);
            telemetry.addData("Status", "SEARCHING");
        }

        telemetry.addData("Locked", isLocked);
        telemetry.addData("Target Ticks", (int) targetTicks);
        telemetry.addData("Current Ticks", currentTicks);
        telemetry.addData("Tick Error", (int) tickError);
        telemetry.update();
    }
}