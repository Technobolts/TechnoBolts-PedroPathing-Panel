package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.DistanceSensorMechanism;

@TeleOp(name = "Distance Sensor Test", group = "Tutorials")
public class DistanceSensorOpMode extends LinearOpMode {

    // Declare a reference to our mechanism class
    private DistanceSensorMechanism sensorSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the mechanism by passing this OpMode's hardwareMap
        sensorSubsystem = new DistanceSensorMechanism(hardwareMap);

        // Wait for the driver to press the PLAY button
        waitForStart();

        // Loop continuously while the OpMode is running
        while (opModeIsActive()) {
            // Get the live distance from our mechanism
            double currentDistance = sensorSubsystem.getDistanceInches();

            // Display the data on the Driver Station screen
            telemetry.addData("Distance (Inches)", "%.2f", currentDistance);
            telemetry.update();

            // Small optional sleep to keep the loop from running overly fast
            sleep(20);
        }
    }
}