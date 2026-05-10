package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms.TBWebcam;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "HuskyLens Shooter")
public class TechnoBoltsHuskyLens extends LinearOpMode {

    int intakeflag = 0;
    private double intakeOn = 1;
    private double intakeOff = 0;

    // ====== HUSKYLENS ======
    private static final double IMAGE_WIDTH = 320.0;
    private static final double HFOV_DEG = 52.0;

    // ====== DISTANCE CALIBRATION ======
    private static final double CALIB_KNOWN_DISTANCE_CM = 144.78; // 57 in
    private static final double CALIB_KNOWN_HEIGHT_PX = 38.0;
    private static final double DIST_K = CALIB_KNOWN_DISTANCE_CM * CALIB_KNOWN_HEIGHT_PX;

    // ====== SHOOTER RPM TUNING ======
    private static final double MAX_RPM = 960;
    private static final double MIN_RPM = 800;
    static final double RPM_START_DISTANCE_IN = 77.0;
    private static final double RPM_MIN_DISTANCE_IN = 30.0;

    // ====== ANGLE COMPENSATION ======
    private static final double ANGLE_DEADBAND_DEG = 1.5;
    private static final double RPM_PER_DEGREE = 35.0;
    private static final double MAX_RPM_CORRECTION = 200;

    // ====== RPM SMOOTHING ======
    private static final double RPM_ALPHA = 0.1;

    // ====== HARDWARE ======
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private Servo upperTServo;
    private CRServo lowerTServo;
    private CRServo middleTServo;
    private DcMotor Intake;
    // ====== STATE ======
    private double smoothLeftRPM = 0;
    private double smoothRightRPM = 0;
    private boolean shooterOn = false;
    private boolean lastDpadUp = false; // for edge detection

    @Override
    public void runOpMode() {

        // Shooter motors
        shooterLeft = hardwareMap.get(DcMotorEx.class, "leftDeposit");
        shooterRight = hardwareMap.get(DcMotorEx.class, "rightDeposit");
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        upperTServo = hardwareMap.get(Servo.class, "upperTServo");
        lowerTServo = hardwareMap.get(CRServo.class, "lowerTServo");
        middleTServo = hardwareMap.get(CRServo.class, "middleTServo");
        Intake = hardwareMap.get(DcMotor.class, "intake");
        // HuskyLens
        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "HuskyLens");
        huskyLens.initialize();
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addLine("Shooter Ready");
        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {

            // ====== DPAD TOGGLE ======
            boolean dpadUp = gamepad2.dpad_up;
            if (dpadUp && !lastDpadUp) {
                shooterOn = !shooterOn; // toggle shooter
            }
            lastDpadUp = dpadUp;

            // ====== APRILTAG + SHOOTER ======
            HuskyLens.Block[] blocks = huskyLens.blocks();
            if (blocks != null && blocks.length > 0 && shooterOn) {
                HuskyLens.Block tag = blocks[0];

                double angleDeg = computeAngleX(tag.x);
                double distanceIn = estimateDistanceCm(tag.height) / 2.54;

                double baseRPM = calculateBaseRPM(distanceIn);
                double correction = calculateAngleCorrection(angleDeg);

                double targetLeftRPM = baseRPM + correction;
                double targetRightRPM = baseRPM - correction;

                // Smooth RPM
                smoothLeftRPM += RPM_ALPHA * (targetLeftRPM - smoothLeftRPM);
                smoothRightRPM += RPM_ALPHA * (targetRightRPM - smoothRightRPM);

                shooterLeft.setVelocity(smoothLeftRPM);
                shooterRight.setVelocity(smoothRightRPM);

                telemetry.addLine("=== SHOOTER ACTIVE ===");
                telemetry.addData("Distance (in)", "%.1f", distanceIn);
                telemetry.addData("Angle (deg)", "%.2f", angleDeg);
                telemetry.addData("Left RPM", "%.0f", smoothLeftRPM);
                telemetry.addData("Right RPM", "%.0f", smoothRightRPM);
                telemetry.addData("Shooter", shooterOn ? "ON" : "OFF");

            } else {
                // Shooter off or no tag
                shooterLeft.setVelocity(0);
                shooterRight.setVelocity(0);
                telemetry.addLine("Shooter OFF or No Tag Detected");
            }

            telemetry.update();

            if (gamepad2.right_bumper) {
                upperTServo.setPosition(0.8); //Originally 0.5
            } else {
                upperTServo.setPosition(0);
            }

            if(gamepad2.dpad_right){
                middleTServo.setPower(1);
                lowerTServo.setPower(-1);
            }
            if(gamepad2.dpad_down){
                middleTServo.setPower(0);
                lowerTServo.setPower(0);
            }
            if(gamepad2.dpad_left) {
                middleTServo.setPower(-1);
                lowerTServo.setPower(1);
            }

            if (gamepad2.aWasPressed()){
                if (intakeflag == 0){
                    Intake.setPower(intakeOn);
                    intakeflag = 1;
                }
                else if (intakeflag == 1) {
                    Intake.setPower(intakeOff);
                    intakeflag = 0;
                }
                else if (intakeflag == -1){
                    Intake.setPower(intakeOff);
                    intakeflag = 0;
                }
            }

            if (gamepad2.bWasPressed()) {
                if (intakeflag == 0){
                    Intake.setPower(-1);
                    intakeflag = -1;
                }
                else if(intakeflag == -1){
                    Intake.setPower(intakeOff);
                    intakeflag = 0;
                }
                else if(intakeflag == 1){
                    Intake.setPower(intakeOff);
                    intakeflag = 0;
                }
            }
        }
    }

    // ================= HELPER METHODS =================
    private double computeAngleX(int xCenterPx) {
        double dx = xCenterPx - (IMAGE_WIDTH / 2.0);
        return (HFOV_DEG / IMAGE_WIDTH) * dx;
    }

    private double estimateDistanceCm(int boxHeightPx) {
        if (boxHeightPx <= 1) return 0;
        return DIST_K / boxHeightPx;
    }

    private double calculateBaseRPM(double distanceIn) {
        if (distanceIn >= RPM_START_DISTANCE_IN) return MAX_RPM;
        if (distanceIn <= RPM_MIN_DISTANCE_IN) return MIN_RPM;

        double ratio = (distanceIn - RPM_MIN_DISTANCE_IN) / (RPM_START_DISTANCE_IN - RPM_MIN_DISTANCE_IN);
        return MIN_RPM + ratio * (MAX_RPM - MIN_RPM);
    }

    private double calculateAngleCorrection(double angleDeg) {
        if (Math.abs(angleDeg) < ANGLE_DEADBAND_DEG) return 0;

        double correction = angleDeg * RPM_PER_DEGREE;
        return Math.max(-MAX_RPM_CORRECTION, Math.min(MAX_RPM_CORRECTION, correction));
    }
}
