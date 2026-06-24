package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Spindexer;

@TeleOp(name = "Decode Spindexer Test")
public class DecodeSpindexerTest
        extends LinearOpMode {

    private DcMotor intake1;

    @Override
    public void runOpMode() {

        intake1 = hardwareMap.get(DcMotor.class, "intake1");

        Spindexer spindexer =
                new Spindexer(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // AUTOMATIC SPINDEXER
            spindexer.update();

            telemetry.addData(
                    "Current Slot",
                    spindexer.getCurrentSlot()
            );

            telemetry.addData(
                    "Detected Color",
                    spindexer.getDetectedColor()
            );

            telemetry.addData(
                    "Stored Artifacts",
                    spindexer.getStoredArtifacts()
            );

            telemetry.addData(
                    "RED",
                    spindexer.getRed()
            );

            telemetry.addData(
                    "GREEN",
                    spindexer.getGreen()
            );

            telemetry.addData(
                    "BLUE",
                    spindexer.getBlue()
            );

            telemetry.update();


            if (gamepad1.a) {
                intake1.setPower(-0.6);
            } else if (gamepad1.b) {
                intake1.setPower(0);
            }
        }
    }
}