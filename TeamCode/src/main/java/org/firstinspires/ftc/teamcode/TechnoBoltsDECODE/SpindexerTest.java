package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Spindexer Test")
public class SpindexerTest extends OpMode {

    public DcMotorEx intake1;
    private Servo spindexer;

    // Current slot (0, 1, or 2)
    private int currentSlot = 0;

    // Servo positions (TUNE THESE)
    private final double[] intakePos = {
            0.10,
            0.55,
            1.0
    };

    private final double[] shootPos = {
            0.0,
            0.35,
            0.80
    };

    // Button edge detection
    private boolean lastRB = false;
    private boolean lastLB = false;
    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void init() {

        spindexer = hardwareMap.get(Servo.class, "spindexerServo");
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");

        // Start at slot 0 in intake position
        spindexer.setPosition(intakePos[currentSlot]);

        telemetry.addLine("Spindexer Ready");
    }

    @Override
    public void loop() {

        // Next slot
        if (gamepad2.right_bumper && !lastRB) {
            currentSlot = (currentSlot + 1) % 3;
            spindexer.setPosition(intakePos[currentSlot]);
        }

        // Previous slot
        if (gamepad2.left_bumper && !lastLB) {
            currentSlot = (currentSlot + 2) % 3;
            spindexer.setPosition(intakePos[currentSlot]);
        }

        // Shoot position
        if (gamepad2.b && !lastB) {
            spindexer.setPosition(shootPos[currentSlot]);
        }

        // Return to intake position
        if (gamepad2.a && !lastA) {
            spindexer.setPosition(intakePos[currentSlot]);
        }

        // Save button states
        lastRB = gamepad2.right_bumper;
        lastLB = gamepad2.left_bumper;
        lastA = gamepad2.a;
        lastB = gamepad2.b;

        if (gamepad2.x) {
            intake1.setPower(0.6);  // Intake fully inward
        } else if (gamepad2.y) {
            intake1.setPower(-0.6); // Outtake/Spit out fully
        } else {
            intake1.setPower(0.0);  // Stop spin when no button is held
        }

        telemetry.addData("Current Slot", currentSlot);
        telemetry.addData("Intake Position", intakePos[currentSlot]);
        telemetry.addData("Shoot Position", shootPos[currentSlot]);
    }
}