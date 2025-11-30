package TechnoBolts;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="OutakeTest", group="Linear Opmode")
    public class OutakeTest extends LinearOpMode {

        @Override
        public void runOpMode() {

            // Initialize the motor, matching the name in the Robot Controller configuration

            DcMotor myMotorLeft = hardwareMap.get(DcMotor.class, "leftDeposit");
            DcMotor myMotorRight = hardwareMap.get(DcMotor.class, "rightDeposit");
            RevBlinkinLedDriver LEDDepo = hardwareMap.get(RevBlinkinLedDriver.class, "ledDeposit");

            // Set the motor direction if needed (e.g., if it spins backward)
             myMotorLeft.setDirection(DcMotor.Direction.FORWARD);
            myMotorRight.setDirection(DcMotor.Direction.REVERSE);

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart(); // Wait for the start button to be pressed

            while (opModeIsActive()) { // Loop while the OpMode is active

                // Example: Control motor with gamepad input


                double motorPower = (gamepad1.left_stick_y/2); // Use left stick Y for power

            //    if {
            //        motorPower = (gamepad1.left_stick_y/2);
            //    }

                myMotorLeft.setPower(motorPower); // Set the motor power
                myMotorRight.setPower(motorPower);

                telemetry.addData("Motor Power", motorPower);
                telemetry.update();
            }

            myMotorLeft.setPower(0);
            myMotorRight.setPower(0);// Stop the motor when the OpMode ends
        }
    }
