package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


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

        final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

        double          clawOffset  = 0.0 ;                  // Servo mid position
        final double    MID_SERVO_claw       =  0.5 ;
        double          armOffset  = 0.0 ;                  // Servo mid position
        final double    MID_SERVO_arm       =  0.5 ;

        waitForStart();

        while(opModeIsActive())
        {
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad2.right_bumper)
                armOffset += CLAW_SPEED;
            else if (gamepad2.left_bumper)
                armOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            armServo1.setPosition(MID_SERVO_arm + clawOffset);
            armServo2.setPosition(MID_SERVO_arm - clawOffset);


            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad2.right_bumper)
                clawOffset += CLAW_SPEED;
            else if (gamepad2.left_bumper)
                clawOffset -= CLAW_SPEED;

            // Move both servos to new position.  Assume servos are mirror image of each other.
            clawOffset = Range.clip(clawOffset, -0.5, 0.5);
            gripperServo1.setPosition(MID_SERVO_claw + clawOffset);
            gripperServo2.setPosition(MID_SERVO_claw - clawOffset);

            idle();
        }
    }
}

