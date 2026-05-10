package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "KickerTest")
public class KickerTest extends OpMode {

    private CRServo kicker;

    @Override
    public void init() {
        kicker = hardwareMap.get(CRServo.class, "Kicker");
        kicker.setPower(0);
    }

    @Override
    public void loop() {

        if(gamepad2.right_bumper) {
            kicker.setPower(1);
        }
        else {
            kicker.setPower(-0.3);
        }


    }
}
