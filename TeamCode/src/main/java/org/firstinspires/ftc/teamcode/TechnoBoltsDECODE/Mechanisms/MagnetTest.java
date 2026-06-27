package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms.MagneticLimitSensor;

@TeleOp(name = "Mechanism Class Test")
public class MagnetTest extends OpMode {

    // Instantiate your custom mechanism class
    MagneticLimitSensor limitSensor = new MagneticLimitSensor();

    @Override
    public void init() {
        limitSensor.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Is Magnet Close?", limitSensor.isPressed());
        telemetry.update();
    }
}