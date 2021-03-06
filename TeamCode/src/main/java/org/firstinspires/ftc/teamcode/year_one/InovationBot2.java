package org.firstinspires.ftc.teamcode.year_one;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Conveyor Belt Triggers2", group = "mortonElements")
@Disabled
public class InovationBot2 extends LinearOpMode
{
    private DcMotor motorLeftDrive;
    private DcMotor motorRightDrive;

    private DcMotor conveyorMotor;
    @Override
    public void runOpMode() throws InterruptedException
    {
        motorLeftDrive = hardwareMap.dcMotor.get("motorLeftDrive");
        motorRightDrive = hardwareMap.dcMotor.get("motorRightDrive");

        conveyorMotor = hardwareMap.dcMotor.get("conveyorMotor");
        motorLeftDrive.setDirection(DcMotor.Direction.REVERSE);


        waitForStart();
        while(opModeIsActive())
        {
            motorLeftDrive.setPower(-gamepad1.left_stick_y);
            motorRightDrive.setPower(-gamepad1.right_stick_y);

            conveyorMotor.setPower(-gamepad2.right_stick_y);
            idle();
        }
    }
}

