package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorMechanism {
    // Declare the DistanceSensor object
    private DistanceSensor distanceSensor;

    // Constructor: Maps the sensor using the hardwareMap passed from the OpMode
    public DistanceSensorMechanism(HardwareMap hardwareMap) {
        // "distance_sensor" must exactly match the name in your Driver Station configuration
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
    }

    /**
     * Getter method to return the current distance reading in inches.
     * You can swap DistanceUnit.INCH for DistanceUnit.CM if you prefer centimeters.
     */
    public double getDistanceInches() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }
}