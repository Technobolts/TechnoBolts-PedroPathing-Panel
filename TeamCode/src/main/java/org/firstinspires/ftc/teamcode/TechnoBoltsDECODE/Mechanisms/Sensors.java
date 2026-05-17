package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Sensors {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        GREEN, PURPLE, BLUE, WHITE, YELLOW, BLACK, RED, UNKNOWN
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

        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];
        String color = "";
// 1. BLACK CHECK
        if (value < 0.15) {
            color = "Black";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE",hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE",value);
            return DetectedColor.BLACK;
        }
// 2. WHITE CHECK
        else if (saturation < 0.20 && value > 0.70) {
            color = "White";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE",hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE",value);
            return DetectedColor.WHITE;
        }
// 3. YELLOW CHECK
        else if (hue >= 45 && hue < 95 && saturation > 0.40) {
            color = "Yellow";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE",hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE",value);
            return DetectedColor.YELLOW;

        }
// 4. GREEN CHECK
        else if (hue >= 95 && hue <= 140 && saturation > 0.40) {
            color = "Green";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE",hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE",value);
            return DetectedColor.GREEN;
        }
// 5. BLUE CHECK
        else if (hue >= 190 && hue < 225 && saturation > 0.40) {
            color = "Blue";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE",hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE",value);
            return DetectedColor.BLUE;
        }
// 6. PURPLE CHECK
        else if (hue >= 225 && hue <= 270 && saturation > 0.40) {
            color = "Purple";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE",hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE",value);
            return DetectedColor.PURPLE;
        }
// 7. RED CHECK (Handles the 360 to 0 wrap-around)
        else if ((hue >= 340 || hue <= 20) && saturation > 0.40) {
            color = "Red";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE",hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE",value);
            return DetectedColor.RED;
        }

// Default Fallback
        telemetry.addData("COLOR:", "Unknown");
        telemetry.addData("HUE",hue);
        telemetry.addData("SATURATION", saturation);
        telemetry.addData("VALUE",value);
        return DetectedColor.UNKNOWN;

    }


}
