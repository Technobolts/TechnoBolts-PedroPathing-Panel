package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo; // Changed from CRServo
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="RPM and LED Testing")
public class RPMAndLEDTesting extends LinearOpMode {
    public DcMotorEx leftDeposit, rightDeposit;
    public Servo ledDepo; // Changed to Servo for precise color control

    final double TICKS_PER_REV = 28.0;
    final double TARGET_RPM = 2400.0;
    final double TOLERANCE = 50.0;


    // Red is at the lower end (~0.0 - 0.1)
    // Green is near the middle or specific set point (~0.4 - 0.5)
    final double COLOR_RED = 0.05;
    final double COLOR_GREEN = 0.45;

    public ElapsedTime timer = new ElapsedTime();
    public long lastLeftPos = 0;
    public long lastRightPos = 0;
    public double lastTime = 0;

    @Override
    public void runOpMode() {
        leftDeposit = hardwareMap.get(DcMotorEx.class, "leftDeposit");
        leftDeposit.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightDeposit = hardwareMap.get(DcMotorEx.class, "rightDeposit");
        rightDeposit.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // Ensure "ledDepo" is configured as a Servo in the Robot Configuration
        ledDepo = hardwareMap.get(Servo.class, "ledDepo");


        waitForStart();
        timer.reset();

//        int i = 0;
        boolean wasReady = false;
        while (opModeIsActive()) {
//          ledDepo.setPosition(COLOR_RED);

            if(gamepad2.left_bumper){
                rightDeposit.setPower(-0.4);
                leftDeposit.setPower(0.4);
            }
            else {
                leftDeposit.setPower(0); // Set the motor power
                rightDeposit.setPower(0);
            }
            double leftVelocity = leftDeposit.getVelocity();
            double rightVelocity = rightDeposit.getVelocity();
            telemetry.addData("Velocity left/Right", "%4.2f, %4.2f", leftVelocity, rightVelocity);
            telemetry.update();

            // 1. Determine the current state
            boolean isReady = (leftVelocity >= 820 && leftVelocity <= 1000 &&
                    rightVelocity <= -820 && rightVelocity >= -1000);

            // 2. Only update the LED if the state has CHANGED
            if (isReady != wasReady) {
                if (isReady) {
                    ledDepo.setPosition(COLOR_GREEN);
                } else {
                    ledDepo.setPosition(COLOR_RED);
                }
                // 3. Update the tracker so we don't send the command again next loop
                wasReady = isReady;
            }

//            if(leftVelocity<0.4 && rightVelocity>-0.4){
//              ledDepo.setPosition(COLOR_RED);
//            }else{
//               ledDepo.setPosition(COLOR_GREEN);
//            }

//            long leftCurrentPosition = leftDeposit.getCurrentPosition();
//            // Calculate difference since last loop
//            long leftDeltaTicks = leftCurrentPosition - lastLeftPos;
//            double leftElapsedTime = (System.nanoTime() - lastTime) / 1_000_000_000.0; // seconds
//            // Ticks per second
//            double leftTicksPerSecond = leftDeltaTicks / leftElapsedTime;
//            lastLeftPos = leftCurrentPosition;
//
//            double leftRpm = leftTicksPerSecond / leftDeposit.getMotorType().getTicksPerRev() * 60; //to turn into true RPM, must multiply by 60
//
//            long rightCurrentPosition = rightDeposit.getCurrentPosition();
//            // Calculate difference since last loop
//            long rightDeltaTicks = rightCurrentPosition - lastRightPos;
//            double rightElapsedTime = (System.nanoTime() - lastTime) / 1_000_000_000.0; // seconds
//            // Ticks per second
//            double rightTicksPerSecond = rightDeltaTicks / rightElapsedTime;
//            lastRightPos = rightCurrentPosition;
//
//            double rightRpm = rightTicksPerSecond / rightDeposit.getMotorType().getTicksPerRev();

            lastTime = System.nanoTime();


//            double currentTime = timer.seconds();
//            double deltaTime = currentTime - lastTime;

//            if (deltaTime >= 0.05) {
//                int currentLeftPos = leftDeposit.getCurrentPosition();
//                int currentRightPos = rightDeposit.getCurrentPosition();

//                double leftTicksPerSec = (currentLeftPos - lastLeftPos) / deltaTime;
//                double rightTicksPerSec = (currentRightPos - lastRightPos) / deltaTime;

            double leftTicksPerRev = leftDeposit.getMotorType().getTicksPerRev();
            double rightTicksPerRev = rightDeposit.getMotorType().getTicksPerRev();

//                double leftRPM = (leftTicksPerSec / TICKS_PER_REV) * 60.0;
//                double rightRPM = (rightTicksPerSec / TICKS_PER_REV) * 60.0;
//
//                // Check if BOTH motors meet the target
//                boolean isReady = (Math.abs(leftRPM) >= TARGET_RPM - TOLERANCE) &&
//                        (Math.abs(rightRPM) >= TARGET_RPM - TOLERANCE);
//
//                // LED Logic: Use setPosition for colors
//                if (isReady) {
//                    ledDepo.setPosition(COLOR_GREEN);
//                } else {
//                    ledDepo.setPosition(COLOR_RED);
//                }
//
//                lastLeftPos = currentLeftPos;
//                lastRightPos = currentRightPos;
//                lastTime = currentTime;
//
//                telemetry.addData("Left RPM", Math.round(leftRPM));
//                telemetry.addData("Right RPM", Math.round(rightRPM));
//                telemetry.addData("Status", isReady ? "READY" : "WAITING (RED)");
//                telemetry.update();
//            }
        }
    }
}
