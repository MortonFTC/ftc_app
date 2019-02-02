package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "AProgram",group = "mortonElements")
@Disabled
public class AProgram extends LinearOpMode {
    private DcMotor motorLeft;
    private DcMotor motorRight;

    private Servo armServo1;
    private Servo armServo2;
    private Servo gripperServo1;
    private Servo gripperServo2;

    //@Override
    public void runOpMode() throws InterruptedException {

        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");



        waitForStart();

        while (opModeIsActive()) {
        motorLeft.setPower(-gamepad1.left_stick_y * 0.05);
        motorRight.setPower(-gamepad1.right_stick_y * 0.05);
                idle();
            }
        }
    }


