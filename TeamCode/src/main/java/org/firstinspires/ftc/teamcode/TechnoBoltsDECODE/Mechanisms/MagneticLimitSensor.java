package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MagneticLimitSensor {

    private DigitalChannel magneticLimitSensor;

    public void init(HardwareMap hwMap) {
        magneticLimitSensor = hwMap.get(DigitalChannel.class, "magnet_limit_sensor");
        magneticLimitSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isPressed() {
        // Returns true if the magnet is detected, false if not
        return !magneticLimitSensor.getState();
    }
}