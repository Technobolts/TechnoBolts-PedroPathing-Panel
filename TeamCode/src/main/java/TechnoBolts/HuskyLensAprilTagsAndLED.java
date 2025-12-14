package TechnoBolts;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
public class HuskyLensAprilTagsAndLED {


// NOTE: Replace these with your actual HuskyLens imports
// import org.firstinspires.ftc.teamcode.vision.HuskyLens;

    @TeleOp(name = "HuskyLens AprilTag Lock + LED")
    public static class HuskyLensAprilTagsLED extends LinearOpMode {

        // Drivetrain
        private DcMotorEx leftFront, leftRear, rightFront, rightRear;

        // LED driver (Blinkin)
        public RevBlinkinLedDriver blinkin;

        // HuskyLens camera (you must implement this wrapper)
        public HuskyLens huskyLens;

        // Constants for camera and control
        private static final double CAMERA_WIDTH_PIXELS = 320.0;   // Adjust if different
        private static final double CENTER_X = CAMERA_WIDTH_PIXELS / 2.0;

        // How “centered” the tag must be to count as locked (pixels)
        private static final double CENTER_TOLERANCE_PIXELS = 10.0;

        // Proportional gain for steering toward tag
        private static final double KP_TURN = 0.01;    // tune this on the field

        // Drive power limits
        private static final double MAX_DRIVE_POWER = 0.6;
        private static final double MAX_TURN_POWER  = 0.4;

        @Override
        public void runOpMode() {

            // Map hardware
            leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
            leftRear   = hardwareMap.get(DcMotorEx.class, "leftRear");
            rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
            rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

            blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

            // You must replace this with however you construct your HuskyLens object
            huskyLens = new HuskyLens(hardwareMap);

            // Standard drivetrain directions (adjust if needed)
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            leftRear.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightRear.setDirection(DcMotor.Direction.REVERSE);

            // Optional braking behavior
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Ensure encoders are being used (if desired)
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Put HuskyLens in AprilTag mode (adjust to your API)
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.APRILTAG);

            telemetry.addLine("HuskyLens AprilTag Lock + LED");
            telemetry.addLine("Hold RIGHT BUMPER to auto-aim at AprilTag");
            telemetry.update();

            waitForStart();

            if (isStopRequested()) return;

            while (opModeIsActive()) {

                // Basic arcade drive input
              //  double drive = -gamepad1.left_stick_y;   // forward/back
                // double turn  = gamepad1.right_stick_x;   // manual turn
                double drive = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
                double turn = gamepad1.left_stick_x; // Counteract imperfect strafing
                double strafe = gamepad1.right_stick_x;

                boolean aimAssist = gamepad1.right_bumper;  // enable auto-aim

                // By default, LED shows “manual mode”
                RevBlinkinLedDriver.BlinkinPattern ledPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;

                double tagErrorX = 0;
                boolean tagVisible = false;
                boolean tagLocked  = false;

                if (aimAssist) {
                    // Request latest AprilTag blocks from HuskyLens
                    boolean gotData = huskyLens.requestBlocks();

                    if (gotData && huskyLens.getBlocksCount() > 0) {
                        // For simplicity, use the first detected tag
                        HuskyLens.Block block = huskyLens.getBlock(0);
                        double tagX = block.x;

                        tagErrorX = tagX - CENTER_X;
                        tagVisible = true;

                        // Compute a turn command from tag offset
                        double autoTurn = KP_TURN * tagErrorX;
                        autoTurn = clip(autoTurn, -MAX_TURN_POWER, MAX_TURN_POWER);

                        // Combine: manual forward/back, auto turning
                        turn = autoTurn;

                        // Evaluate lock status
                        if (Math.abs(tagErrorX) <= CENTER_TOLERANCE_PIXELS) {
                            tagLocked = true;
                        }

                        // LED: Green when locked+rpm, Yellow when rpm
                        if (tagLocked) {
                            ledPattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                        } else {
                            ledPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                        }

                    } else {
                        // No tag detected
                        tagVisible = false;
                        tagLocked = false;
                        ledPattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                    }

                   }
//                else {
//                    // Aim-assist not active (manual control)
//                    ledPattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
//                }

                // Scale drive power
                drive = clip(drive, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
                turn  = clip(turn, -MAX_TURN_POWER, MAX_TURN_POWER);

                // Differential drive
                double leftPower  = drive + turn;
                double rightPower = drive - turn;

                leftPower  = clip(leftPower, -1.0, 1.0);
                rightPower = clip(rightPower, -1.0, 1.0);

                leftFront.setPower(leftPower);
                leftRear.setPower(leftPower);
                rightFront.setPower(rightPower);
                rightRear.setPower(rightPower);

                // Apply LED pattern
                blinkin.setPattern(ledPattern);

                // Telemetry
                telemetry.addData("Aim Assist", aimAssist ? "ON (RB)" : "OFF");
                telemetry.addData("Tag Visible", tagVisible);
                telemetry.addData("Tag Locked", tagLocked);
                telemetry.addData("Tag Error X (px)", tagErrorX);
                telemetry.addData("Drive", drive);
                telemetry.addData("Turn", turn);
                telemetry.addData("LED", ledPattern.name());
                telemetry.update();
            }
        }

        /**
         * Simple local clip function so we don’t rely on Range.clip.
         */
        private double clip(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }

        /**
         * Stub HuskyLens class.
         * Replace this with your actual HuskyLens API implementation.
         */
        private static class HuskyLens {

            public enum Algorithm {
                APRILTAG
            }

            public static class Block {
                public int x;
                public int y;
                public int width;
                public int height;
                public int id;
            }

            public HuskyLens(com.qualcomm.robotcore.hardware.HardwareMap hardwareMap) {
                // Initialize I2C/UART here for your actual HuskyLens driver
            }

            public void selectAlgorithm(Algorithm algo) {
                // Send command to HuskyLens to switch to AprilTag mode
            }

            public boolean requestBlocks() {
                // Query HuskyLens for the latest detection result
                // Return true if data was successfully updated
                return false;
            }

            public int getBlocksCount() {
                // Return number of detected blocks
                return 0;
            }

            public Block getBlock(int index) {
                // Return block at index
                return new Block();
            }
        }
    }
}
