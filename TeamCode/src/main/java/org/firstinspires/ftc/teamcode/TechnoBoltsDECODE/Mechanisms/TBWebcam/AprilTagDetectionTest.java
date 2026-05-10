package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms.TBWebcam;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TechnoBoltsAprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
@Autonomous
public class AprilTagDetectionTest extends OpMode {
    TechnoBoltsAprilTagWebcam aprilTagWebcam = new TechnoBoltsAprilTagWebcam();

    @Override
    public void init() {
        aprilTagWebcam.init(hardwareMap,telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);
        aprilTagWebcam.displayDetectionTelemetry(id24);
    }
}
