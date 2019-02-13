package org.firstinspires.ftc.teamcode.year_two.test;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ErrorCorrectLoop", group = "mortonElements")
@Disabled
public class ErrorCorrectLoop extends LinearOpMode {
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


        //THE FOLLOWING CODE STARTS THE SERVOS AT APPROXIMATELY 15 DEGREES AWAY FROM START
        //SERVOS WILL MOVE APPROXIMATELY 1/4 DEGREE TOWARD START POSITION EVERY SECOND
        //NOTE THE SERVO POSITIONS WHEN IN DESIRED POSITION AND THEN UNCOMMENT THE METHOD
        //INIT_Final_Pos TO ADJUST MANUALLY

        //INITIALIZE TO 10 DEGRESS FROM START (I.E. 10 * DEGREE_DIFF)
        Double DEGREE_DIFF = 1/280D;  //RESULT IS THE DECIMAL VALUE OF 1 DEGREE AS # BETWEEN 0 AND 1
        int i = 15;

        waitForStart();

        while (opModeIsActive()) {
            //leftOut.setPosition(1 - (i * DEGREE_DIFF));
            //leftIn.setPosition(0 + (i * DEGREE_DIFF));
            //rightIn.setPosition(1 - DEGREE_DIFF);
            //rightOut.setPosition(0 + DEGREE_DIFF);

            upperRight.setPosition(0 + (i * DEGREE_DIFF));
            upperLeft.setPosition(1 - (i * DEGREE_DIFF));

            sleep(1000);     // pause for 1 second

            //telemetry.addData("leftOut", leftOut.getPosition());
            //telemetry.addData("leftIn", leftIn.getPosition());
            //telemetry.addData("rightIn", rightIn.getPosition());
            //telemetry.addData("rightOut", rightOut.getPosition());
            telemetry.addData("uppperLeft", upperLeft.getPosition());
            telemetry.addData("upperRight", upperRight.getPosition());
            telemetry.update();
            //DECREMENT i BY 1 WHICH REDUCES START POSITION BY # OF DEGREE FOR NEXT ITERATION
            i--;
        }
    }

    }



