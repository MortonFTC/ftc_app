package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Lamgunana_V2",group = "mortonElements")
//@Disabled
public class Lamgunana_V2 extends LinearOpMode {
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

        final double CLAW_SPEED = 0.005;                 // sets rate to move servo

        double clawOffset = 0.0;                  // Servo mid position
        final double MID_SERVO_claw = 0.5;
        double armOffset = 0.0;                  // Servo mid position
        final double MID_SERVO_arm = 0.5;

        waitForStart();

        while (opModeIsActive()) {
            motorLeft.setPower(-gamepad1.left_stick_y);
            motorRight.setPower(-gamepad1.right_stick_y);

            // Use left & right triggers to send claw arm to preset positions
            // Left Trigger = top position; Right Trigger = bottom position

            //if (gamepad2.left_trigger) {
            if (gamepad1.x || gamepad1.b) {
                if (gamepad1.x) {
                    armServo1.setPosition(0.36);
                    armServo2.setPosition(0.64);
                    gripperServo1.setPosition(0.36);
                    gripperServo2.setPosition(0.64);
                }
                //if (gamepad2.right_trigger) {
                if (gamepad1.b) {
                    armServo1.setPosition(0.61);
                    armServo2.setPosition(0.39);
                    gripperServo1.setPosition(0.60);
                    gripperServo2.setPosition(0.40);
                }
            }
                else {
                    // Use gamepad left & right Bumpers to open and close the claw
                    //if (gamepad2.right_bumper)
                    if (gamepad1.right_bumper)
                        armOffset += CLAW_SPEED;
                        //else if (gamepad2.left_bumper)
                    else if (gamepad1.left_bumper)
                        armOffset -= CLAW_SPEED;

                    // Move both servos to new position.  Assume servos are mirror image of each other.
                    clawOffset = Range.clip(clawOffset, -0.5, 0.5);
                    armServo1.setPosition(MID_SERVO_arm + clawOffset);
                    armServo2.setPosition(MID_SERVO_arm - clawOffset);


                    // Use gamepad left & right Bumpers to open and close the claw
                    //if (gamepad2.right_bumper)
                    if (gamepad1.right_bumper)
                        clawOffset += CLAW_SPEED;
                        //else if (gamepad2.left_bumper)
                    else if (gamepad1.left_bumper)
                        clawOffset -= CLAW_SPEED;

                    // Move both servos to new position.  Assume servos are mirror image of each other.
                    clawOffset = Range.clip(clawOffset, -0.5, 0.5);
                    gripperServo1.setPosition(MID_SERVO_claw + clawOffset);
                    gripperServo2.setPosition(MID_SERVO_claw - clawOffset);

                    telemetry.addData("Claw Speed...", CLAW_SPEED);
                    telemetry.addData("Claw Offset...", clawOffset);
                    telemetry.addData("Arm 1 Position...", armServo1.getPosition());
                    telemetry.addData("Arm 2 Position...", armServo2.getPosition());
                    telemetry.addData("Gripper 1 Position...", gripperServo1.getPosition());
                    telemetry.addData("Gripper 2 Position...", gripperServo2.getPosition());
                    telemetry.update();

                    idle();
                }
            }
        }
    }

