package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TestBenchDistance {
    private DistanceSensor distance;

    // Initialize the hardware map
    public void init(HardwareMap hardwareMap) {
        // "sensorDistance" must match the name configured in your driver hub
        distance = hardwareMap.get(DistanceSensor.class, "sensorDistance");
    }

    // Getter method to retrieve distance in centimeters
    public double getDistance() {
        return distance.getDistance(DistanceUnit.CM);
    }
}
