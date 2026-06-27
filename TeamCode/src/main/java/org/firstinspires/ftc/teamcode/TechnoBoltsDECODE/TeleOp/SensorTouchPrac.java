package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.DistanceSensorMechanism;

@TeleOp(name = "Sensor Touch Practice")
public class SensorTouchPrac extends OpMode {

    // Create an instance of your SensorTouch class
    private DistanceSensorMechanism.SensorTouch touch = new DistanceSensorMechanism.SensorTouch();

    @Override
    public void init() {
        touch.init(hardwareMap);
        telemetry.addLine("Touch sensor initialized");
    }

    @Override
    public void loop() {
        if (touch.isPressed()) {
            telemetry.addLine("Touch Sensor: PRESSED");
        } else {
            telemetry.addLine("Touch Sensor: RELEASED");
        }
        telemetry.update();
    }
}
