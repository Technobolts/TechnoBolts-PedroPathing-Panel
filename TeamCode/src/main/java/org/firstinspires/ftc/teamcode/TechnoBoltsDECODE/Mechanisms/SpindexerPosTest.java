package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class SpindexerPosTest extends OpMode {

    double spindexerPos = 0;
    public Servo spindexer;

    @Override
    public void init() {
        spindexer = hardwareMap.get(Servo.class, "spindexerServo");
        spindexer.setPosition(spindexerPos);
    }

    @Override
    public void loop() {

        if (gamepad1.aWasPressed()){
            spindexerPos += 0.1;
        }
        if (gamepad1.bWasPressed()){
            spindexerPos -= 0.1;
        }
        if (gamepad1.xWasPressed()){
            spindexerPos += 0.01;
        }
        if (gamepad1.yWasPressed()){
            spindexerPos -= 0.01;
        }


        spindexer.setPosition(spindexerPos);

        telemetry.addData("current angle", spindexerPos);
    }
}
