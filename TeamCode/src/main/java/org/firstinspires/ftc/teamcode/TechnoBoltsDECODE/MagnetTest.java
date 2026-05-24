package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Magnet Switch Test")
public class MagnetTest extends OpMode {
    private DigitalChannel magnetSwitch;

    @Override
    public void init() {
        // Hardware name must match your configuration EXACTLY
        magnetSwitch = hardwareMap.get(DigitalChannel.class, "magnet_switch_sensor");

        // REV Magnetic Switch is a digital input
        magnetSwitch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("Magnet Switch Initialized");
    }

    @Override
    public void loop() {
        boolean isPressed = !magnetSwitch.getState();
        // REV magnetic switch returns:
        // getState() == true  → NOT pressed (open)
        // getState() == false → PRESSED (closed)

        telemetry.addData("Magnet State", isPressed ? "CLOSED" : "OPEN");
        telemetry.update();
    }
}




