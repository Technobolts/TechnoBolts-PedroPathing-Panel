package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms.TBWebcam;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.TechnoBoltsAprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name = "Align To AprilTag", group = "Test")
public class AlignToAprilTag extends OpMode {
    private final TechnoBoltsAprilTagWebcam aprilTagWebcam = new TechnoBoltsAprilTagWebcam();

    public DcMotor leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;

    // ------------------------ PD Controller -------------------------
    double kP = 0.03;
    double error = 0;
    double lastError = 0;
    double goalX = 0; //offset here
    double angleTolerance = 0.4;
    double kD = 0.0004;
    double curTime = 0;
    double lastTime = 0;

    // -------------------- Driving Setup ---------------------
    double forward, strafe, rotate;

    // --------------------- Controller based PD tuning -----------------------

    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        aprilTagWebcam.init(hardwareMap, telemetry);
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("Initialized");

    }

    public void start(){
        resetRuntime();
        curTime = getRuntime();
    }

    @Override
    public void loop() {
        //----------------------- get mecanum drive inputs -------------------------
        forward = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // ---------------------- get apriltag info -----------------------------
        aprilTagWebcam.update();
        AprilTagDetection id24 = aprilTagWebcam.getTagBySpecificId(24);

        // ---------------------- auto align rotation trigger --------------------------

        if (gamepad1.left_trigger > 0.3) {
            if (id24 != null) {
                error = goalX - id24.ftcPose.bearing; // tx

                if (Math.abs(error) < angleTolerance){
                    rotate = 0;
                } else {
                    double pTerm = error * kP;

                    curTime = getRuntime();
                    double dT = curTime - lastTime;
                    double dTerm = ((error - lastError) / dT) *kD;

                    rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                    lastError = error;
                    lastTime = curTime;
                }
            }
            else {
                lastTime = getRuntime();
                lastTime = 0;

            }
        }
        else {
            lastError = 0;
            lastTime = getRuntime();
        }




        // drive our motors
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        double max;
        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);
        double leftFrontPower = (forward + strafe + rotate) / denominator;
        double leftBackPower = (forward - strafe + rotate) / denominator;
        double rightFrontPower = (forward - strafe - rotate) / denominator;
        double rightBackPower = (forward + strafe - rotate) / denominator;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);



        //Adjust kP, kD on the fly

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length; //Modulo wraps the index back to 0
        }

        // D-pad left/right adjusts the P gain
        if(gamepad1.dpadLeftWasPressed()){
            kP -= stepSizes[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed()){
            kP += stepSizes[stepIndex];
        }

        // D-pad up/down adjusts the P gain
        if(gamepad1.dpadUpWasPressed()){
            kD += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            kD -= stepSizes[stepIndex];
        }

        // ------------------------- Telemetry --------------------------
        if (id24 != null){
            if(gamepad1.left_trigger > 0.3){
                telemetry.addLine("AUTO ALIGN");
            }
            aprilTagWebcam.displayDetectionTelemetry(id24);
            telemetry.addData("Error", error);
        }
        else {
            telemetry.addLine("MANUAL Rotate Mode");
        }
        telemetry.addLine("-----------------------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad L/R)", kP);
        telemetry.addData("Tuning D", "%.4f (D-Pad U/D)", kD);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);

    }
}



// kP = 0.03
// kD = 0.0002