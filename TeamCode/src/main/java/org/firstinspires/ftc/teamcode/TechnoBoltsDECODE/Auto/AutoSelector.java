package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "Auto Selector", group = "Autonomous")
public class AutoSelector extends OpMode {

    // Enums for selection
    enum Alliance { BLUE, RED }
    enum StartPos { A, B, C }

    // Constants for shooter PIDF
    double FR = 12.22;
    double PR = 100.5;

    double FL = 12.62;
    double PL = 100.85;

    // Selection variables with default values
    Alliance alliance = Alliance.RED;
    StartPos startPos = StartPos.A;

    // Button state variable for debouncing (prevents crazy fast toggling)
    boolean lastGamepad2X = false;

    // Followers and hardware
    Follower follower;
    public DcMotor intake;
    public DcMotorEx turretShooter;
    public Servo Spindexer, Kicker, turretHood;

    // Sub-auto instances for all 6 paths
    AutoRedA redAutoA;
    AutoRedB redAutoB;
    AutoRedC redAutoC;

    AutoBlueA blueAutoA;
    AutoBlueB blueAutoB;
    AutoBlueC blueAutoC;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Initialize motors
        intake = hardwareMap.get(DcMotor.class, "intake1");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        turretShooter = hardwareMap.get(DcMotorEx.class, "turretShooter");
        turretShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pidfCoefficientsRight = new PIDFCoefficients(PR, 0, 0, FR);
        turretShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficientsRight);

        // Initialize servos
        Kicker = hardwareMap.get(Servo.class, "kickerServo");
        Spindexer = hardwareMap.get(Servo.class, "spindexerServo");
        turretHood = hardwareMap.get(Servo.class, "hoodShooter");

        // Instantiate all 6 auto classes with the follower and hardware dependencies
        redAutoA = new AutoRedA(follower, telemetry, intake, turretShooter, Kicker, Spindexer, turretHood);
        redAutoB = new AutoRedB(follower, telemetry, intake, turretShooter, Kicker, Spindexer, turretHood);
        redAutoC = new AutoRedC(follower, telemetry, intake, turretShooter, Kicker, Spindexer, turretHood);

        blueAutoA = new AutoBlueA(follower, telemetry, intake, turretShooter, Kicker, Spindexer, turretHood);
        blueAutoB = new AutoBlueB(follower, telemetry, intake, turretShooter, Kicker, Spindexer, turretHood);
        blueAutoC = new AutoBlueC(follower, telemetry, intake, turretShooter, Kicker, Spindexer, turretHood);
    }

    @Override
    public void init_loop() {
        telemetry.addLine("=== AUTO SELECTION ===");
        telemetry.addLine("Press X to TOGGLE Alliance (Red/Blue)");
        telemetry.addLine("Press A, B, or Y to choose Position (A, B, C)");
        telemetry.addLine("---------------------------------------------");

        // Alliance Toggle Logic (Registers only once per complete press)
        if (gamepad2.x && !lastGamepad2X) {
            if (alliance == Alliance.RED) {
                alliance = Alliance.BLUE;
            } else {
                alliance = Alliance.RED;
            }
        }
        lastGamepad2X = gamepad2.x; // Update last button state

        // Position Selection Input
        if (gamepad2.a) startPos = StartPos.A;
        if (gamepad2.b) startPos = StartPos.B;
        if (gamepad2.y) startPos = StartPos.C;

        // Display selection telemetry
        telemetry.addData("Selected Alliance", alliance);
        telemetry.addData("Selected Position", startPos);
        telemetry.addLine("---------------------------------------------");
        telemetry.update();
    }

    @Override
    public void start() {
        // Starts the path state machine inside the selected auto class
        if (alliance == Alliance.RED) {
            if (startPos == StartPos.A) redAutoA.start();
            else if (startPos == StartPos.B) redAutoB.start();
            else if (startPos == StartPos.C) redAutoC.start();
        } else { // BLUE
            if (startPos == StartPos.A) blueAutoA.start();
            else if (startPos == StartPos.B) blueAutoB.start();
            else if (startPos == StartPos.C) blueAutoC.start();
        }
    }

    @Override
    public void loop() {
        // Continuously runs the update method for the active path
        if (alliance == Alliance.RED) {
            if (startPos == StartPos.A) redAutoA.update();
            else if (startPos == StartPos.B) redAutoB.update();
            else if (startPos == StartPos.C) redAutoC.update();
        } else { // BLUE
            if (startPos == StartPos.A) blueAutoA.update();
            else if (startPos == StartPos.B) blueAutoB.update();
            else if (startPos == StartPos.C) blueAutoC.update();
        }

        // Live coordinate feedback on the Driver Station
        telemetry.addData("X Position", follower.getPose().getX());
        telemetry.addData("Y Position", follower.getPose().getY());
        telemetry.addData("Heading (Deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}