package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake Test Run", group = "Linear OpMode")
public class IntakeTestRun extends LinearOpMode {
    private DcMotor intake1;

    @Override
    public void runOpMode() {
        intake1 = hardwareMap.get(DcMotor.class, "intake1");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                runIntake();
            } else if (gamepad1.b) {
                stopIntake();
            }

            // Optional: Show power status on the Driver Station
            telemetry.addData("Intake Power", intake1.getPower());
            telemetry.update();
        }
    }
    public void runIntake() {
        intake1.setPower(-0.6);
    }

    public void stopIntake() {
        intake1.setPower(0.0);
    }
}