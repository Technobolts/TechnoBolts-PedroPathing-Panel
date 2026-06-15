package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp(name = "Outtake Test (Velocity Controlled)", group = "Linear Opmode")
public class OutakeTest extends LinearOpMode {

    // ===== SHOOTER CONSTANTS =====
    private static final double TARGET_RPM = 3000;
    private static final double RPM_TOLERANCE = 75;
    private static final double TICKS_PER_REV = 28;   // Change if needed
    private static final double TRIGGER_DEADZONE = 0.1;

    @Override
    public void runOpMode() {

        // ===== MOTORS =====
        DcMotorEx leftMotor = hardwareMap.get(DcMotorEx.class, "leftDeposit");
        DcMotorEx rightMotor = hardwareMap.get(DcMotorEx.class, "rightDeposit");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ===== LED =====
        RevBlinkinLedDriver blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        RevBlinkinLedDriver.BlinkinPattern lastPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;

        telemetry.addLine("Velocity Shooter Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double trigger = gamepad1.right_trigger;
            boolean shooterEnabled = trigger > TRIGGER_DEADZONE;

            // Convert target RPM → ticks per second
            double targetVelocityTicks = (TARGET_RPM / 60.0) * TICKS_PER_REV;

            if (shooterEnabled) {
                leftMotor.setVelocity(targetVelocityTicks);
                rightMotor.setVelocity(targetVelocityTicks);
            } else {
                leftMotor.setPower(0);
                rightMotor.setPower(0);
            }

            // ===== READ ACTUAL RPM =====
            double currentVelocityTicks = leftMotor.getVelocity();
            double currentRPM = (currentVelocityTicks / TICKS_PER_REV) * 60.0;

            double error = TARGET_RPM - currentRPM;

            // ===== LED LOGIC =====
            RevBlinkinLedDriver.BlinkinPattern newPattern;

            if (!shooterEnabled) {
                newPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            } else if (Math.abs(error) <= RPM_TOLERANCE) {
                newPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN; // Ready to shoot
            } else {
                newPattern = RevBlinkinLedDriver.BlinkinPattern.RED;   // Not ready
            }

            if (newPattern != lastPattern) {
                blinkin.setPattern(newPattern);
                lastPattern = newPattern;
            }

            // ===== TELEMETRY =====
            telemetry.addData("Shooter Enabled", shooterEnabled);
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Current RPM", currentRPM);
            telemetry.addData("Error", error);
            telemetry.addData("LED Pattern", newPattern.toString());
            telemetry.update();
        }
    }
}
