package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms.colorSensor;
@TeleOp
public class ColorSensor extends OpMode {

    colorSensor sensor = new colorSensor();

    @Override
    public void init(){
        sensor.init(hardwareMap);
    }
    @Override
    public void loop(){
        sensor.getDetectedColor(telemetry);
    }
}

