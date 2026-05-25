package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sensor Touch Practice")
public class SensorTouchPrac extends OpMode {

    // Create an instance of your SensorTouch class
    private SensorTouch touch = new SensorTouch();

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
