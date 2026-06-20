package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class magnetic_limit_sensor {

    private DigitalChannel magneticLimitSensor;

    public void init(HardwareMap hwMap) {
        magneticLimitSensor = hwMap.get(DigitalChannel.class, "magnet_limit_sensor");
        magneticLimitSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getMagneticLimitSensor() {
        return !magneticLimitSensor.getState();
    }
}



