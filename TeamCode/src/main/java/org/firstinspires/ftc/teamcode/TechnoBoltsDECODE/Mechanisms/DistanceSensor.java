package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DistanceSensor extends OpMode {

    // Create an instance of the mechanism class
    TestBenchDistance bench = new TestBenchDistance();

    @Override
    public void init() {
        // Initialize the hardware bench
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        // Sense: grab the current distance
        double currentDistance = bench.getDistance();

        if (currentDistance <= 0.5) {
            telemetry.addData("Object Detected at",currentDistance);
        } else {
            telemetry.addData("No Object in 0.5 in distance: ", currentDistance);
        }

        telemetry.update();
    }
}
