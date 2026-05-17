package org.firstinspires.ftc.robotcontroller;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class MagnetTest extends OpMode {
    MagnetTest bench = new MagnetTest();

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.addData("Magnet state", bench.isMagneticlimitsensorCLosed());

    }
}
