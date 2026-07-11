package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Spindexer Test")
public class SpindexerTest extends OpMode {

    private Servo spindexer;

    private DcMotorEx intake1;

    private int slot = 0;

    // YOUR TUNED POSITIONS
    private final double[] intakePos = {
            0.10,
            0.46,
            0.83
    };

    private final double[] shootPos = {
            0.25,
            0.62,
            0.99
    };

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastRB = false;

    @Override
    public void init() {
        spindexer = hardwareMap.get(Servo.class, "spindexerServo");
        intake1 = hardwareMap.get(DcMotorEx.class, "intake1");

        slot = 0;
        spindexer.setPosition(intakePos[slot]);
    }

    @Override
    public void loop() {

        // =========================
        // INTAKE (instant snap)
        // =========================
        if (gamepad2.a && !lastA) {
            spindexer.setPosition(intakePos[slot]);
        }

        // =========================
        // SHOOT (instant snap)
        // =========================
        if (gamepad2.b && !lastB) {
            spindexer.setPosition(shootPos[slot]);
        }

        // =========================
        // CLOCKWISE ONLY INDEX
        // (important for stability)
        // =========================
        if (gamepad2.right_bumper && !lastRB) {

            slot = (slot + 1) % 3;

            // ALWAYS return to intake after indexing
            spindexer.setPosition(intakePos[slot]);
        }

        // save buttons
        lastA = gamepad2.a;
        lastB = gamepad2.b;
        lastRB = gamepad2.right_bumper;

        if (gamepad2.x) {
            intake1.setPower(1.0);  // Intake fully inward
        } else if (gamepad2.y) {
            intake1.setPower(-1.0); // Outtake/Spit out fully
        } else {
            intake1.setPower(0.0);  // Stop spin when no button is held
        }

        telemetry.addData("Slot", slot);
        telemetry.addData("Intake", intakePos[slot]);
        telemetry.addData("Shoot", shootPos[slot]);
        telemetry.update();
    }
}