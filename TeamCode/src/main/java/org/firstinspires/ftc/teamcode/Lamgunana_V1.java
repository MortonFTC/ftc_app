package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Lamgunana_V1",group = "mortonElements")
public class Lamgunana_V1 extends LinearOpMode
{
    private DcMotor motorLeft;
    private DcMotor motorRight;

    private Servo armServo1;
    private Servo armServo2;
    private Servo gripperServo1;
    private Servo gripperServo2;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        armServo1 = hardwareMap.servo.get("armServo1");
        armServo2 = hardwareMap.servo.get("armServo2");
        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        gripperServo2 = hardwareMap.servo.get("gripperServo2");

        waitForStart();

        while(opModeIsActive())
        {
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            if(gamepad2.a)
            {
                armServo1.setPosition(0.5);
                armServo2.setPosition(-0.5);
            }

            idle();
        }
    }
}

