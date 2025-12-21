package TechnoBolts;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Headlight RPM Indicator")
public class RPMAndLEDTesting extends LinearOpMode {
    public DcMotorEx leftDeposit, rightDeposit;
    public CRServo ledDepo;

    final double TICKS_PER_REV = 28.0;
    final double TARGET_RPM = 4500.0;
    final double TOLERANCE = 200.0;

    @Override
    public void runOpMode() {
        leftDeposit = hardwareMap.get(DcMotorEx.class, "leftDeposit");
        rightDeposit = hardwareMap.get(DcMotorEx.class, "rightDeposit");

        // Map the headlight as a standard Servo
        ledDepo = hardwareMap.get(CRServo.class, "ledDepo");

        waitForStart();

        while (opModeIsActive()) {
            // RPM Calculation
            double leftRPM = (leftDeposit.getVelocity() * 60.0) / TICKS_PER_REV;
            double rightRPM = (rightDeposit.getVelocity() * 60.0) / TICKS_PER_REV;

            boolean isReady = (leftRPM >= TARGET_RPM - TOLERANCE) &&
                    (rightRPM >= TARGET_RPM - TOLERANCE);

            // Control Headlight based on RPM state
            if (isReady) {
                ledDepo.setPower(1.0); // Full brightness (Target met)
            } else {
                ledDepo.setPower(0.0); // Off (Target not met)
            }

            telemetry.addData("Status", isReady ? "READY" : "WAITING");
            telemetry.update();
        }
    }
}


