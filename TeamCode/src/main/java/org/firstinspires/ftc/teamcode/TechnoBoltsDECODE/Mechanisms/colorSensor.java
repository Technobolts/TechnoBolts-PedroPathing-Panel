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

    // This handles the setup on your Driver Hub when you hit INIT
    @Override
    public void init() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor_1");
    }

    // This runs continuously on your Driver Hub after you press PLAY
    @Override
    public void loop() {
        getDetectedColor();
    }

    public DetectedColor getDetectedColor(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float[] hsv = new float[3];

        // Convert the normalized colors (scaled 0-255) to HSV
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

        // ==========================================
        // 1. ENVIRONMENTAL FILTERS (Most Specific)
        // ==========================================

        // BLACK: Filter out absolute lack of light first
        if (value < 0.15) {
            color = "Black";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.BLACK;
        }
        // WHITE: Filter out completely desaturated light next
        else if (saturation < 0.20 && value > 0.70) {
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

        // ORANGE: Very tight hue window (21-44). Must be checked before broader Red/Yellow.
        else if (hue > 20 && hue < 45 && saturation > 0.40) {
            color = "Orange";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.ORANGE;
        }
        // BLUE: Moderately tight target window (190-224)
        else if (hue >= 190 && hue < 225 && saturation > 0.40) {
            color = "Blue";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.BLUE;
        }
        // PURPLE: Specific upper-end window (225-270)
        else if (hue >= 225 && hue <= 270 && saturation > 0.40) {
            color = "Purple";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.PURPLE;
        }

        // ==========================================
        // 3. BROAD SPECTRIC HUES (Least Specific / Large Windows)
        // ==========================================

        // GREEN: Huge window (95-140)
        else if (hue >= 95 && hue <= 140 && saturation > 0.40) {
            color = "Green";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.GREEN;
        }
        // YELLOW: Broad middle window (45-94)
        else if (hue >= 45 && hue < 95 && saturation > 0.40) {
            color = "Yellow";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.YELLOW;
        }
        // RED: Massive split window (340-360 AND 0-20) acts as a wide net
        else if ((hue >= 340 || hue <= 20) && saturation > 0.40) {
            color = "Red";
            telemetry.addData("COLOR:", color);
            telemetry.addData("HUE", hue);
            telemetry.addData("SATURATION", saturation);
            telemetry.addData("VALUE", value);
            return DetectedColor.RED;
        }

        // ==========================================
        // 4. THE ULTIMATE FALLBACK
        // ==========================================
        telemetry.addData("COLOR:", "Unknown");
        telemetry.addData("HUE", hue);
        telemetry.addData("SATURATION", saturation);
        telemetry.addData("VALUE", value);
        return DetectedColor.UNKNOWN;
    }
}