package TechnoBolts;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HuskyLens Distance Test", group = "Test")
public class TechnoBoltsHuskyLensDistance extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize HuskyLens (make sure the name matches your config!)
        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "HuskyLens");
        huskyLens.initialize();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);


        telemetry.addLine("HuskyLens Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();

            if (blocks != null && blocks.length > 0) {
                HuskyLens.Block obj = blocks[0]; // first detected object

                int width = obj.width;
                int height = obj.height;

                double distanceEstimate = 1200.0 / width; // calibrate later

                telemetry.addData("Object Width", width);
                telemetry.addData("Object Height", height);
                telemetry.addData("Estimated Distance (inches)", "%.2f", distanceEstimate);
            } else {
                telemetry.addLine("No object detected");
            }

            telemetry.update();
        }
    }
}
