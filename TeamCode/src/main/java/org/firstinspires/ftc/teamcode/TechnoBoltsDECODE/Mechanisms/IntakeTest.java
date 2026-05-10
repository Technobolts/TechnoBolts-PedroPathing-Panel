package org.firstinspires.ftc.teamcode.TechnoBoltsDECODE.Mechanisms;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name="DecodeIntakeTest", group="TechnoBolts - OpMode")
public class IntakeTest extends LinearOpMode {
    //private ElapsedTime runtime = new ElapsedTime();
    public DcMotor Intake = null;


    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        Intake = hardwareMap.get(DcMotor.class, "intake");

        waitForStart();
        // runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.a)
                Intake.setPower(-1);
            if (gamepad1.b)
                Intake.setPower(0);
        }
    }
}

