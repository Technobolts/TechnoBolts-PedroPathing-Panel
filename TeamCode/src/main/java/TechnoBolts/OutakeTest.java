package TechnoBolts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LED;

@TeleOp(name="OutakeTest", group="Linear Opmode")
    public class OutakeTest extends LinearOpMode {

        @Override
        public void runOpMode() {

            // Initialize the motor, matching the name in the Robot Controller configuration

            DcMotor myMotorLeft = hardwareMap.get(DcMotor.class, "leftDeposit");
            DcMotor myMotorRight = hardwareMap.get(DcMotor.class, "rightDeposit");
            LED ledDepo = hardwareMap.get(LED.class,"ledDepo" );
            // Set the motor direction if needed (e.g., if it spins backward)
             myMotorLeft.setDirection(DcMotor.Direction.FORWARD);
            myMotorRight.setDirection(DcMotor.Direction.FORWARD);

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart(); // Wait for the start button to be pressed

            while (opModeIsActive()) { // Loop while the OpMode is active
                double motorPower = 0;
                // Example: Control motor with gamepad input
                while(gamepad1.left_bumper) {
                    motorPower += 10; // Use left bumper for power
                    ledDepo.enableLight(motorPower <= 1000);
                }
                myMotorLeft.setPower(motorPower); // Set the motor power
                myMotorRight.setPower(motorPower);

                telemetry.addData("Motor Power", motorPower);
                telemetry.update();
            }

            myMotorLeft.setPower(0);
            myMotorRight.setPower(0);// Stop the motor when the OpMode ends
        }
    }
