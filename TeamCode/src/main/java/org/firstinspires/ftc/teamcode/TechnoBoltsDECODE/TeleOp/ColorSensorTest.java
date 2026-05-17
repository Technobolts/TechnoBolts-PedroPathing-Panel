package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms.Sensors;

public class ColorSensorTest extends OpMode {

    Sensors sensor = new Sensors();

    @Override
    public void init(){
        sensor.init(hardwareMap);
    }
    @Override
    public void loop(){
        sensor.getDetectedColor(telemetry);
    }
}

