package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class Sensors {

    NormalizedColorSensor colorSensor;

    public enum detectedColor{
        GREEN,
        PURPLE,
        UNKNOWN
    }
    public  void init(HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class, "color_sensor_1");
    }
    public DetectedColor getDetectedColor(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normPurple, normGreen;
        normGreen = colors.green / colors.alpha;

    }



}
