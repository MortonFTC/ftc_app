package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Lamgunana_V3", group = "mortonElements")
public class Lamguana_V3 extends LinearOpMode {

//@Disabled

    //TODO Work in progress.
    private DcMotor motorLeft;
    private DcMotor motorRight;

    private Servo armServo1;
    private Servo armServo2;
    private Servo gripperServo1;
    private Servo gripperServo2;

    //@Override
    public void runOpMode() throws InterruptedException {
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        armServo1 = hardwareMap.servo.get("armServo1");
        armServo2 = hardwareMap.servo.get("armServo2");
        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        gripperServo2 = hardwareMap.servo.get("gripperServo2");

        final double POSITION_CHANGE_RATE = 0.005;                 // sets rate to move servo

        double clawOffset = 0.0;
        final double MID_SERVO_claw = 0.5; //Default servo position


        double armOffset = 0.0;
        final double MID_SERVO_arm = 0.5; //Default servo position

        waitForStart();

        while (opModeIsActive()) {
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            //Moves the arms
            if (gamepad1.right_bumper)
                armOffset += Range.clip(POSITION_CHANGE_RATE, -.5, .5);
            else if (gamepad1.left_bumper)
                armOffset -= Range.clip(POSITION_CHANGE_RATE, -.5, .5);

            if (gamepad1.right_trigger > 0)
                clawOffset += Range.clip(POSITION_CHANGE_RATE, -.5, .5);
            else if (gamepad1.left_trigger > 0)
                clawOffset -= Range.clip(POSITION_CHANGE_RATE, -.5, .5);
            idle();


        }
    }
}



