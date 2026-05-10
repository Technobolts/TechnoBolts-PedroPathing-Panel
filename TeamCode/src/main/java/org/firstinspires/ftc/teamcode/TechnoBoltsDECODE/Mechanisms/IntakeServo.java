package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Example OpMode to control a Servo to specific positions.
 * The positions are defined as constants between 0.0 and 1.0.
 */
@Disabled
@TeleOp(name = "IntakeServo")
public class IntakeServo extends OpMode {

    // --- HARDWARE DECLARATION ---
    // Declare the servo object
    private Servo testServo = null;

    // --- SERVO POSITION CONSTANTS ---
    // Define position constants (0.0 to 1.0) for your desired angles.
    // NOTE: You will need to physically tune these values on your robot.
    final double POSITION_DOWN = 0.2; // Example: Corresponds to a lower angle
    final double POSITION_UP   = 0.8; // Example: Corresponds to a higher angle
    final double POSITION_START = 0.5; // Example: Center or starting position

    // --- INITIALIZATION METHOD ---
    @Override
    public void init() {

        // 1. Get the servo from the hardware map
        // The string "claw_servo" must match the name in your
        // Robot Controller's hardware configuration file.
        try {
            testServo = hardwareMap.get(Servo.class, "claw_servo");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find servo 'claw_servo'. Check config.");
        }

        // 2. Set the servo to its starting position
        if (testServo != null) {
            testServo.setPosition(POSITION_START);
            telemetry.addData("Status", "Servo Initialized to: " + POSITION_START);
        }

        telemetry.update();
    }

    // --- LOOP METHOD (The main control loop) ---
    @Override
    public void loop() {

        // Check if the servo was initialized successfully
        if (testServo != null) {

            // 1. CONTROL LOGIC
            // Move the servo to POSITION_UP when the right bumper is pressed
            if (gamepad1.right_bumper) {
                testServo.setPosition(POSITION_UP);
                telemetry.addData("Servo State", "Moving UP");
            }
            // Move the servo to POSITION_DOWN when the left bumper is pressed
            else if (gamepad1.left_bumper) {
                testServo.setPosition(POSITION_DOWN);
                telemetry.addData("Servo State", "Moving DOWN");
            }
            // You can add more positions or controls here (e.g., using A/B/X/Y buttons)

            // 2. TELEMETRY OUTPUT
            // Display the current position setpoint
            telemetry.addData("Current Setpoint", "%.2f", testServo.getPosition());
            telemetry.update();
        }
    }

    // --- STOP METHOD ---
    @Override
    public void stop() {
        // Optional: Reset the servo to a safe position when the OpMode stops
        // if (testServo != null) {
        //     testServo.setPosition(POSITION_START);
        // }
    }
}
