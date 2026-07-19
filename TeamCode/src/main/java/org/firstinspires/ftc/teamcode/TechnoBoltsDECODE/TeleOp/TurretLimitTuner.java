package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Turret Limit Tuner")
public class TurretLimitTuner extends OpMode {

    private DcMotorEx turret;

    // ===== Tune these values =====
    private int LEFT_LIMIT = -1250;
    private int RIGHT_LIMIT = 1250;

    // Maximum manual speed
    private static final double MAX_POWER = 0.35;

    @Override
    public void init() {

        turret = hardwareMap.get(DcMotorEx.class, "turretMotor");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Turret Limit Tuner Ready");
        telemetry.addLine("Move with Right Stick X");
        telemetry.addLine("Read encoder values at each limit");
    }

    @Override
    public void loop() {

        double power = gamepad1.right_stick_x * MAX_POWER;

        int pos = turret.getCurrentPosition();

        // Left limit
        if (pos <= LEFT_LIMIT && power < 0) {
            power = 0;
        }

        // Right limit
        if (pos >= RIGHT_LIMIT && power > 0) {
            power = 0;
        }

        turret.setPower(power);

        telemetry.addData("Encoder", pos);
        telemetry.addData("Motor Power", power);
        telemetry.addData("Left Limit", LEFT_LIMIT);
        telemetry.addData("Right Limit", RIGHT_LIMIT);

        telemetry.addLine("------------------------");
        telemetry.addLine("Drive to LEFT stop and record encoder.");
        telemetry.addLine("Drive to RIGHT stop and record encoder.");

        telemetry.update();
    }

    @Override
    public void stop() {
        turret.setPower(0);
    }
}