package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SensorTouch {

    private DigitalChannel ftc_touch_Sensor;

    public void init(HardwareMap hwMap) {
        ftc_touch_Sensor = hwMap.get(DigitalChannel.class, "ftc_touch_sensor");
        ftc_touch_Sensor.setMode(DigitalChannel.Mode.INPUT);
    }

    // Returns true when the button is PRESSED
    public boolean isPressed() {
        return !ftc_touch_Sensor.getState();  // REV touch sensor is active‑low
    }

    // Returns true when the button is NOT pressed
    public boolean isReleased() {
        return ftc_touch_Sensor.getState();
    }
}
