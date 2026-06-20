package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class DistanceTest extends OpMode {

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

        // Think & Act: logic to display a message based on object distance
        if (currentDistance < 10.0) {
            telemetry.addLine("Too Close!");
        } else {
            telemetry.addData("Distance (cm)", currentDistance);
        }

        telemetry.update();
    }
}
