package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name="Run Color Sensor", group="Test")
public class colorSensor extends OpMode {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        GREEN, PURPLE, BLUE, WHITE, BLACK, RED, ORANGE, YELLOW, UNKNOWN
    }
// hue 150 saturation .74 value .819
    @Override
    public void init() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor_1");
        colorSensor.setGain(15.0f); // Keeps the signal amplified
    }

    @Override
    public void loop() {
        getDetectedColor();
    }

    public DetectedColor getDetectedColor(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float[] hsv = new float[3];
        int colorInt = colors.toColor();
        Color.colorToHSV(colorInt, hsv);

        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];
        String color = "";

        // ==========================================
        // 1. ENVIRONMENTAL FILTERS (White remains specific)
        // ==========================================
        if (saturation < 0.20 && value > 0.60) {
            color = "White";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.WHITE;
        }

        // ==========================================
        // 2. NARROW SPECTRIC HUES (Highly Specific)
        // ==========================================
        else if (hue > 30  && hue < 60 && saturation > 0.40) {
            color = "Orange";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.ORANGE;
        }
        else if (hue >= 190 && hue < 225 && saturation > 0.40) {
            color = "Blue";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.BLUE;
        }
        else if (hue >= 225 && hue <= 270 && saturation > 0.40) {
            color = "Purple";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.PURPLE;
        }

        // ==========================================
        // 3. BROAD SPECTRIC HUES (Least Specific)
        // ==========================================
        else if (hue >= 100 && hue <= 160 && saturation > 0.40) {
            color = "Green";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.GREEN;
        }
        else if (hue >= 50 && hue < 95 && saturation > 0.40) {
            color = "Yellow";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.YELLOW;
        }
        else if ((hue >= 340 || hue <= 27) && saturation > 0.40) {
            color = "Red";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.RED;
        }

        // ==========================================
        // 4. LOW LIGHT/FALLBACK FILTERS (Broadest)
        // ==========================================

        // BLACK: Checked LAST so it only triggers if no actual color hue was matched
        else if (value < 0.15) {
            color = "Black";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.BLACK;
        }

        // Ultimate Fallback
        telemetry.addData("COLOR:", "Unknown");
        telemetry.addData("HUE", hue);
        telemetry.addData("SATURATION", saturation);
        telemetry.addData("VALUE", value);
        return DetectedColor.UNKNOWN;
    }
}