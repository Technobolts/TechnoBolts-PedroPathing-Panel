package TechnoBolts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="HuskyTest")
public class HuskyTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Servo huskyServo = hardwareMap.get(Servo.class, "huskyServo");


    }
}



