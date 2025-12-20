package TechnoBolts;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="AutoScrimmage")
public class AutoScrimmage extends LinearOpMode {

        // Drive Motors
        private DcMotor leftFront, rightFront, leftBack, rightBack;
    private DcMotor rightDeposit;
        // Ramp/Transfer Servos
        private CRServo middleTServo, lowerTServo; // CRServos use setPower
        private Servo upperTServo;      // Standard Servo uses setPosition

        // Adjust these values for your robot
        static final double DRIVE_POWER = 0.5;
        static final long MOVE_TO_LINE_TIME = 2400; // Time to reach 6/8 of the line
        static final long CLEAR_LINE_TIME = 1500;   // Time to fully exit the line
        static final double SHOOT_POWER = 0.9;      // RPM for the flywheels

        @Override
        public void runOpMode() {
            // Hardware Mapping
            leftFront   = hardwareMap.get(DcMotor.class, "left_front");
            rightFront  = hardwareMap.get(DcMotor.class, "right_front");
            leftBack    = hardwareMap.get(DcMotor.class, "left_back");
            rightBack   = hardwareMap.get(DcMotor.class, "right_back");
            // Outtake Motors
            DcMotor leftDeposit = hardwareMap.get(DcMotor.class, "outtake_left");
            rightDeposit= hardwareMap.get(DcMotor.class, "outtake_right");

            // Servo Mapping
            middleTServo = hardwareMap.get(CRServo.class, "middleTServo");
            lowerTServo  = hardwareMap.get(CRServo.class, "lowerTServo");
            upperTServo  = hardwareMap.get(Servo.class, "upperTServo");

            // Direction Setup
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
            rightDeposit.setDirection(DcMotor.Direction.REVERSE);
            upperTServo.setDirection(Servo.Direction.REVERSE);

            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();

            if (opModeIsActive()) {
                // STEP 1: Move to 6/8 (75%) of the Launch Line
                driveForward(DRIVE_POWER, MOVE_TO_LINE_TIME);
                stopDrive();

                // STEP 2: Shoot 3 Artifacts with 5s RPM recovery
                leftDeposit.setPower(SHOOT_POWER);
                rightDeposit.setPower(SHOOT_POWER);

                // Initial spin-up
                sleep(1000);

                for (int i = 1; i <= 3; i++) {
                    telemetry.addData("Shooting", "Artifact " + i);
                    telemetry.update();

                    // Activate internal ramp/transfer mechanism to feed ball
                    middleTServo.setPower(1.0);  // Use setPower for CRServo
                    lowerTServo.setPower(-1.0);
                    upperTServo.setPosition(0.65); // Use setPosition for standard Servo

                    sleep(800); // Time to feed one ball

                    // Stop feeder between shots
                    middleTServo.setPower(0);
                    lowerTServo.setPower(0);
                    upperTServo.setPosition(0);

                    if (i < 3) {
                        telemetry.addData("Action", "Waiting 5s for RPM recovery...");
                        telemetry.update();
                        sleep(5000); // Wait 5 seconds for outtake to reach full speed again
                    }
                }

                // Turn off outtake after all shots
                leftDeposit.setPower(0);
                rightDeposit.setPower(0);

                // STEP 3: Clear the white line based on alliance
                // UNCOMMENT ONLY THE LINE FOR OUR ALLIANCE BEFORE THE MATCH:

               // strafeLeft(DRIVE_POWER, CLEAR_LINE_TIME);  // For BLUE alliance
                 strafeRight(DRIVE_POWER, CLEAR_LINE_TIME); // For RED alliance

                stopDrive();
            }
        }

        // Movement Helper Methods
        public void driveForward(double p, long t) { setDrive(p, p, p, p); sleep(t); }
        public void strafeLeft(double p, long t) { setDrive(-p, p, p, -p); sleep(t); }
        public void strafeRight(double p, long t) { setDrive(p, -p, -p, p); sleep(t); }

        private void setDrive(double lf, double rf, double lb, double rb) {
            leftFront.setPower(lf); rightFront.setPower(rf);
            leftBack.setPower(lb); rightBack.setPower(rb);
        }

        public void stopDrive() { setDrive(0,0,0,0); }
    }



