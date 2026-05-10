package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms.TBWebcam;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HuskyLens AprilTag Distance & Angle")
public class TechnoBoltsHuskyLensDistance extends LinearOpMode {

    // ====== CAMERA RESOLUTION (HuskyLens default) ======
    private static final double IMAGE_WIDTH  = 320.0;
    private static final double IMAGE_HEIGHT = 240.0;

    // HuskyLens horizontal field of view
    private static final double HFOV_DEG = 52.0;

    // ====== DISTANCE CALIBRATION ======
    // Measure these with YOUR AprilTag
    private static final double CALIB_KNOWN_DISTANCE_CM = 203.2; // example: 80 in
    private static final double CALIB_KNOWN_HEIGHT_PX   = 38.0;  // tag height in px

    private static final double DIST_K =
            CALIB_KNOWN_DISTANCE_CM * CALIB_KNOWN_HEIGHT_PX;

    // OPTIONAL: lock onto one tag ID
    private static final int TARGET_TAG_ID = 1;

    @Override
    public void runOpMode() {

        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "HuskyLens");

        huskyLens.initialize();

        // 🔹 SWITCH TO TAG RECOGNITION
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addLine("AprilTag mode ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            HuskyLens.Block[] blocks = huskyLens.blocks();

            if (blocks != null && blocks.length > 0) {

                HuskyLens.Block tag = null;

                // Find the correct tag ID
                for (HuskyLens.Block b : blocks) {
                    if (b.id == TARGET_TAG_ID) {
                        tag = b;
                        break;
                    }
                }

                if (tag != null) {

                    int x = tag.x;
                    int y = tag.y;
                    int width = tag.width;
                    int height = tag.height;
                    int id = tag.id;

                    double angleDeg = computeAngleX(x);
                    double distanceCm = estimateDistanceCm(height);
                    double distanceIn = distanceCm / 2.54;

                    telemetry.addLine("=== AprilTag Detected ===");
                    telemetry.addData("Tag ID", id);
                    telemetry.addData("Center", "x=%d y=%d", x, y);
                    telemetry.addData("Height(px)", height);
                    telemetry.addData("Angle (deg)", "%.2f", angleDeg);
                    telemetry.addData("Distance (in)", "%.1f", distanceIn);

                } else {
                    telemetry.addLine("Target tag not found");
                }

            } else {
                telemetry.addLine("No tags detected");
            }

            telemetry.update();
        }
    }

    // ====== ANGLE CALC ======
    private double computeAngleX(int xCenterPx) {
        double dx = xCenterPx - (IMAGE_WIDTH / 2.0);
        double degreesPerPixel = HFOV_DEG / IMAGE_WIDTH;
        return dx * degreesPerPixel;
    }

    // ====== DISTANCE MODEL ======
    private double estimateDistanceCm(int boxHeightPx) {
        if (boxHeightPx <= 1) return 0;
        return DIST_K / boxHeightPx;
    }
}
