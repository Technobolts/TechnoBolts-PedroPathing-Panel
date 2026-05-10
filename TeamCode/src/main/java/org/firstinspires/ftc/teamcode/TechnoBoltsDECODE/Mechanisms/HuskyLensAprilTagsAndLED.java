package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp(name = "LED + HuskyLens")
public class HuskyLensAprilTagsAndLED extends LinearOpMode {

    // ===================== LED =====================
    public RevBlinkinLedDriver blinkin;

    // ===================== SHOOTER MOTORS =====================
    public DcMotorEx leftDeposit;
    private DcMotorEx rightDeposit;

    // ===================== HUSKYLENS =====================
    private HuskyLens huskyLens;

    // ===================== RPM SETTINGS =====================
    private static final double TARGET_RPM = 3000; // TUNE
    private static final double RPM_TOLERANCE = 100;      // ± target window
    private static final double RPM_MATCH_TOLERANCE = 75; // motors must match each other

    @Override
    public void runOpMode() {

        // ===================== HARDWARE MAP =====================
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        leftDeposit  = hardwareMap.get(DcMotorEx.class, "leftDeposit");   // CHANGE NAME
        rightDeposit = hardwareMap.get(DcMotorEx.class, "rightDeposit");  // CHANGE NAME

        huskyLens = new HuskyLens(hardwareMap);

        leftDeposit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDeposit.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.APRILTAG);

        telemetry.addLine("LED + Dual RPM + HuskyLens Test");
        telemetry.addLine("Both flywheels must match RPM");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ===================== MAIN LOOP =====================
        while (opModeIsActive()) {

            // ===== APRILTAG DETECTION =====
            boolean tagDetected = false;
            if (huskyLens.requestBlocks() && huskyLens.getBlocksCount() > 0) {
                tagDetected = true;
            }

            // ===== RPM CALCULATION =====
            double leftRPM  = getRPM(leftDeposit);
            double rightRPM = getRPM(rightDeposit);

            boolean leftInRange  = Math.abs(leftRPM  - TARGET_RPM) <= RPM_TOLERANCE;
            boolean rightInRange = Math.abs(rightRPM - TARGET_RPM) <= RPM_TOLERANCE;

            boolean motorsMatch =
                    Math.abs(leftRPM - rightRPM) <= RPM_MATCH_TOLERANCE;

            // RPM is met ONLY if all conditions are true
            boolean rpmMet = leftInRange && rightInRange && motorsMatch;

            // ===== LED LOGIC =====
            RevBlinkinLedDriver.BlinkinPattern ledPattern;

            if (!rpmMet) {
                ledPattern = RevBlinkinLedDriver.BlinkinPattern.RED; //If RPM and April Tag not detected
            }
            else if (!tagDetected) {
                ledPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW; //If RPM met and April Tag not detected
            }
            else {
                ledPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN; //If RPM met and April Tag is detected
            }

            blinkin.setPattern(ledPattern);

            // ===== TELEMETRY =====
            telemetry.addData("Left RPM", "%.0f", leftRPM);
            telemetry.addData("Right RPM", "%.0f", rightRPM);
            telemetry.addData("Motors Match", motorsMatch);
            telemetry.addData("RPM Ready", rpmMet);
            telemetry.addData("AprilTag", tagDetected);
            telemetry.addData("LED", ledPattern);
            telemetry.update();
        }
    }

    // ===================== HELPERS =====================
    private double getRPM(DcMotorEx motor) {
        return motor.getVelocity() * 60
                / motor.getMotorType().getTicksPerRev();
    }

    // ===================== HUSKYLENS STUB =====================
    private static class HuskyLens {

        public enum Algorithm { APRILTAG }

        public static class Block {
            public int x, y, width, height, id;
        }

        public HuskyLens(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {}

        public void selectAlgorithm(Algorithm algo) {}

        public boolean requestBlocks() { return false; }

        public int getBlocksCount() { return 0; }

        public Block getBlock(int index) { return new Block(); }
    }
}
