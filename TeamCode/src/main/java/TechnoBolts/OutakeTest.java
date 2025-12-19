package TechnoBolts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx; // Use Ex for velocity control
import com.qualcomm.robotcore.hardware.LED;

@TeleOp(name="OutakeTest", group = "Linear Opmode")
public class OutakeTest extends LinearOpMode {

    // Target Settings
    final double TARGET_RPM = 3000; // Set RPM here
    final double TICKS_PER_REV = 28; // Adjust based on motor

    // Stating Variables
    boolean motorRunning = false;
    boolean lastBumperState = false;

    @Override
    public void runOpMode() {
        // Cast to DcMotorEx to access setVelocity()
        DcMotorEx myMotorLeft = hardwareMap.get(DcMotorEx.class, "leftDeposit");
        DcMotorEx myMotorRight = hardwareMap.get(DcMotorEx.class, "rightDeposit");
        LED ledDepo = hardwareMap.get(LED.class, "ledDepo");

        myMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        myMotorRight.setDirection(DcMotor.Direction.FORWARD);

        // Required for precise RPM control
        myMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        myMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Conversion: Ticks per second = (RPM / 60) * Ticks per Rev
        double targetVelocity = (TARGET_RPM / 60) * TICKS_PER_REV;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Rising Edge Detector: Checks if the bumper was JUST pressed
            if (gamepad1.left_bumper && !lastBumperState) {
                motorRunning = !motorRunning; // Toggles the motor on/off
            }
            lastBumperState = gamepad1.left_bumper; // Saves state for next loop

            if (motorRunning) {
                myMotorLeft.setVelocity(targetVelocity);
                myMotorRight.setVelocity(targetVelocity);
                ledDepo.enableLight(true);
            } else {
                myMotorLeft.setVelocity(0);
                myMotorRight.setVelocity(0);
                ledDepo.enableLight(false);
            }

            // Feedback to driver
            telemetry.addData("Motor State", motorRunning ? "RUNNING" : "STOPPED");
            telemetry.addData("Target RPM", TARGET_RPM);
            telemetry.addData("Actual Left RPM", (myMotorLeft.getVelocity() / TICKS_PER_REV) * 60);
            telemetry.update();
        }

        myMotorLeft.setPower(0);
        myMotorRight.setPower(0);
    }
}
