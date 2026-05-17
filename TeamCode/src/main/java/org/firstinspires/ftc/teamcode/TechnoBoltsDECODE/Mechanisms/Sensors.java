package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sensors {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public void init(HardwareMap hwMap){
        colorSensor = hwMap.get(NormalizedColorSensor.class, "color_sensor_1");
    }

    public DetectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float[] hsv = new float[3];

        // 2. Convert the normalized colors (scaled 0-255) to HSV
        Color.RGBToHSV(
                (int)(colors.red * 255),
                (int)(colors.green * 255),
                (int)(colors.blue * 255),
                hsv
        );

        // 3. Extract the Hue and Saturation using proper array indices
        float hue = hsv[0];
        float saturation = hsv[1];

        // Telemetry for easy calibration debugging
        telemetry.addData("Current Hue", hue);
        telemetry.addData("Current Saturation", saturation);
        String color = "";

        // 4. Threshold Logic (Adjust these numbers based on your field tests)
        if (hue >= 100 && hue <= 140 && saturation > 0.4) {
            color = "Green";
            telemetry.addData("COLOR:", color);
            return DetectedColor.GREEN;
         } else if (hue >= 165 && hue <= 240 && saturation > 0.4) {
            color = "Purple";
            telemetry.addData("COLOR:", color);
            return DetectedColor.PURPLE;
        }

        return DetectedColor.UNKNOWN;
    }


}
