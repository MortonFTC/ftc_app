package org.firstinspires.ftc.teamcode.year_two;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ErrorCorrectionTest", group = "mortonElements")
//@Disabled
public class ErrorCorrectionTest extends LinearOpMode {
    private Servo leftOut;
    private Servo leftIn;
    private Servo rightIn;
    private Servo rightOut;

    //@Override
    public void runOpMode() throws InterruptedException {
        leftOut = hardwareMap.get(Servo.class, "leftOut");
        leftIn = hardwareMap.get(Servo.class,"leftIn");
        rightIn = hardwareMap.get(Servo.class,"rightIn");
        rightOut = hardwareMap.get(Servo.class,"rightOut");

        boolean run = false;

        Double QUARTER_DEGREE = 2/280D;

        waitForStart();

        int i = 0;
        while (opModeIsActive()) {

            if (!run) {
                leftOut.setPosition(1 - QUARTER_DEGREE);
                leftIn.setPosition(0 + QUARTER_DEGREE);
                rightIn.setPosition(1 - QUARTER_DEGREE);
                rightOut.setPosition(0 + QUARTER_DEGREE);
                run =true;
            }
                telemetry.addData("leftOut", leftOut.getPosition());
                telemetry.addData("leftIn", leftIn.getPosition());
                telemetry.addData("rightIn", rightIn.getPosition());
                telemetry.addData("rightOut", rightOut.getPosition());
                i++;
                telemetry.addData("loops", i);
                telemetry.update();
            }
        }
    }



