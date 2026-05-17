package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class magnetic_limit_sensor {

    private DigitalChannel magneticLimitSensor;

    public void init(@NonNull HardwareMap hwMap) {
        magneticLimitSensor = hwMap.get(DigitalChannel.class, "magnet_limit_sensor");
        magneticLimitSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getMagneticLimitSensor() {
        // REV magnetic switch is ACTIVE-LOW:
        // getState() == true  → OPEN (not triggered)
        // getState() == false → CLOSED (triggered)
        return !magneticLimitSensor.getState();
    }
}



