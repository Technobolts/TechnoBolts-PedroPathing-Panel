package TechnoBolts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="HuskyTest")
public class HuskyTest extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        Servo huskyServo = hardwareMap.get(Servo.class, "huskyServo");
        TechnoBoltsHuskyLensDistance distance = new TechnoBoltsHuskyLensDistance();
        CalculateShooterAngle calculator = new CalculateShooterAngle();
        while (opModeIsActive()) {

            while(gamepad2.left_bumper) {
                //while(){
                //  servo spins around
                // }
                //use distance(var) and get a distance from april tag
                //put that distance in calculateShooterAngle
                //set rpm of the flywheels to calculated rpm
                //green light once everything finishes
            }
        }
    }
}



