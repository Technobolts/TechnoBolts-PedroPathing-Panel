package TechnoBolts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
    @TeleOp(name="DepositAngleTest")
    public class DepositAngleTest extends OpMode {
        // Declare hardware here
        Servo depositServo;

        @Override
        public void init() {
            // Map hardware
            depositServo = hardwareMap.get(Servo.class, "depositServo");
        }

        @Override
        public void loop() {
            if (gamepad1.a) {
                depositServo.setPosition(0.0);   // 0 degrees
            }
            if (gamepad1.b) {
                depositServo.setPosition(0.-5);   // 90 degrees
            }
            //if (gamepad1.x) {
            //   depositServo.setPosition(-1.0);   // 180 degrees
            //}
        }
    }

