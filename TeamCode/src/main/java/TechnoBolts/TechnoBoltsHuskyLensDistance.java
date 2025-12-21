package TechnoBolts;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HuskyLens Distance & Angle Test")
public class TechnoBoltsHuskyLensDistance extends LinearOpMode {

    // ====== CAMERA RESOLUTION ======
    private static final double IMAGE_WIDTH  = 320.0;
    private static final double IMAGE_HEIGHT = 240.0;

    // HuskyLens typical horizontal field of view degrees
    private static final double HFOV_DEG = 52.0;

    // ====== DISTANCE CALIBRATION ======
    // Update these after measuring!
    private static final double CALIB_KNOWN_DISTANCE_CM = 60.96;
    private static final double CALIB_KNOWN_HEIGHT_PX   = 35.56;

    private static final double DIST_K = CALIB_KNOWN_DISTANCE_CM * CALIB_KNOWN_HEIGHT_PX;

    @Override
    public void runOpMode() {

        // Make sure your configuration name matches "HuskyLens"
        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "HuskyLens");

        huskyLens.initialize();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);

        telemetry.addLine("HuskyLens Ready – waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            HuskyLens.Block[] blocks = huskyLens.blocks();

            if (blocks != null && blocks.length > 0) {
                HuskyLens.Block obj = blocks[0];

                int x = obj.x;          // center X
                int y = obj.y;          // center Y
                int width = obj.width;
                int height = obj.height;
                int id = obj.id;

                double angleDeg = computeAngleX(x);
                double distanceCm = estimateDistanceCm(height);
                double distanceInches = distanceCm / 2.54;

                telemetry.addLine("=== Object Detected ===");
                telemetry.addData("ID", id);
                telemetry.addData("Center", "x=%d, y=%d", x, y);
                telemetry.addData("Width", width);
                telemetry.addData("Height", height);
                telemetry.addData("Angle (deg)", "%.2f", angleDeg);
                telemetry.addData("Distance (in)", "%.1f", distanceCm);
             //   telemetry.addData("Distance (in)", "%.2f", distanceInches);

            } else {
                telemetry.addLine("No object detected");
            }

            telemetry.update();
        }
    }

    // Computes horizontal angle based on image center
    private double computeAngleX(int xCenterPx) {
        double dx = xCenterPx - (IMAGE_WIDTH / 2.0);
        double degreesPerPixel = HFOV_DEG / IMAGE_WIDTH;
        return dx * degreesPerPixel;
    }

    // Simple inverse model: distance ≈ K / boxHeight
    private double estimateDistanceCm(int boxHeightPx) {
        if (boxHeightPx <= 1) return 0;
        return DIST_K / boxHeightPx;
    }
}