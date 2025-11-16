package TechnoBolts;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;




@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "HuskyLens Distance Test", group = "Test")
public class TechnoBoltsHuskyLensDistance extends LinearOpMode {


    private HuskyLens huskyLens;


    @Override
    public void runOpMode() {
        // Initialize HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.initialize();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);

        telemetry.addLine("HuskyLens Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            HuskyLensObject obj = huskyLens
            if (obj != null) {
                int width = obj.width;
                int height = obj.height;

                // Estimate distance (you'll calibrate this later)
                double distanceEstimate = 1200.0 / width;

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
