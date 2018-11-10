package org.firstinspires.ftc.teamcode.year_two;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ErrorCorrectFinal", group = "mortonElements")
//@Disabled
public class ErrorCorrectFinal extends LinearOpMode {
    private Servo leftOut;
    private Servo leftIn;
    private Servo rightIn;
    private Servo rightOut;
    private Servo upperRight;
    private Servo upperLeft;

    //@Override
    public void runOpMode() throws InterruptedException {
        //leftOut = hardwareMap.get(Servo.class, "leftOut");
        //leftIn = hardwareMap.get(Servo.class,"leftIn");
        //rightIn = hardwareMap.get(Servo.class,"rightIn");
        //rightOut = hardwareMap.get(Servo.class,"rightOut");
        upperLeft = hardwareMap.get(Servo.class,"upperLeft");
        upperRight = hardwareMap.get(Servo.class, "upperRight");

        waitForStart();

        while (opModeIsActive()) {
            /*final Double LeftOutSTART = 0.931;  //FINAL START POS
            final Double LeftInSTART = 0.082;   //FINAL START POS
            final Double RightOutSTART = 0.047;
            final Double RightInSTART = 0.966;
            */
            final Double uppperLeftSTART = 0.9742;
            final Double upperRightSTART = 0.0517;

            /*eftOut.setPosition(LeftOutSTART);
            leftIn.setPosition(LeftInSTART);
            rightIn.setPosition(RightInSTART);
            rightOut.setPosition(RightOutSTART);
            */
            upperRight.setPosition(upperRightSTART);
            upperLeft.setPosition(uppperLeftSTART);
            sleep(1000);     // pause for 1 second

            //telemetry.addData("leftOut", leftOut.getPosition());
            //telemetry.addData("leftIn", leftIn.getPosition());
            //telemetry.addData("rightIn", rightIn.getPosition());
            //telemetry.addData("rightOut", rightOut.getPosition());
            telemetry.addData("uppperLeft", upperLeft.getPosition());
            telemetry.addData("upperRight", upperRight.getPosition());
            telemetry.update();
            //DECREMENT i BY 1 WHICH REDUCES START POSITION BY # OF DEGREE FOR NEXT ITERATION
            //i--;
        }
    }

    }



