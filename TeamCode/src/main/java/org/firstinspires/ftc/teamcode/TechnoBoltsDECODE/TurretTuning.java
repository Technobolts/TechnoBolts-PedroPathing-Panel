package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TurretTuning extends OpMode {

    private DcMotorEx turret;
    private Limelight3A limelight;

    // ===== TRACKING =====
    public static double P = -0.0042;
    public static double MAX_POWER = 0.45;
    public static double DEADZONE = 5;

    // ===== SEARCH =====
    private double searchDir = 1;
    private static final double SEARCH_POWER = 0.28;
    private static final long LOST_TIME = 250;

    private long lastSeen = 0;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        boolean hasTarget = result != null && result.isValid();

        int pos = turret.getCurrentPosition();

        // =========================
        // TRACK MODE
        // =========================
        if (hasTarget) {

            lastSeen = System.currentTimeMillis();

            double tx = result.getTx();

            double error = tx;

            if (Math.abs(error) < DEADZONE) {
                turret.setPower(0);
                telemetry.addData("Mode", "LOCKED");
            } else {

                double power = P * error;

                power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

                turret.setPower(power);

                telemetry.addData("Mode", "TRACKING");
            }
        }

        // =========================
        // SEARCH MODE
        // =========================
        else {

            long lostFor = System.currentTimeMillis() - lastSeen;

            if (lostFor < LOST_TIME) {
                turret.setPower(0);
                telemetry.addData("Mode", "HOLD");
            } else {

                // FORCE MOTION (fixes your "stops moving" issue)
                double power = SEARCH_POWER * searchDir;

                turret.setPower(power);

                // bounce off limits using encoder
                if (pos > 1400) searchDir = -1;
                if (pos < -1400) searchDir = 1;

                telemetry.addData("Mode", "SEARCHING");
            }
        }

        telemetry.addData("Tx", hasTarget ? result.getTx() : "NONE");
        telemetry.addData("Pos", pos);
        telemetry.update();
    }
}