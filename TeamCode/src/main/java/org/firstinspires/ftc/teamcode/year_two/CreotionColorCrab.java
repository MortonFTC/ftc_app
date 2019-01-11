package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Thread.sleep;

public class CreotionColorCrab extends LinearOpMode {

    public DcMotor  leftFrontDrive   = null; // Port 0: REV Robotics HD Hex motor
    public DcMotor  rightFrontDrive  = null; // Port 1: REV Robotics HD Hex motor
    public DcMotor  leftRearDrive    = null; // Port 2: REV Robotics HD Hex motor
    public DcMotor  rightRearDrive   = null; // Port 3: REV Robotics HD Hex motor

    public ColorSensor colorSensor  = null; // Port 1: REV Color/Range Sensor

    private static final String VUFORIA_KEY = "AeWrHWf/////AAAAmZG3057lc0JXoVs+HjtHkjZyYL2/IQH4DPGcMKxDXU12F688beSRkSeE6Oz1nH1imNIbBvdwCWFtpqBTu9aqKnlQ9XE3cDLcuUa6/iv0yK3oKy/4p+C1KqltmtvLTda0rgoW8mVcohX38181Apke+iCMjogFT0FHT+3o36MrhYRT03H7Al4Ynqd09uLIGiCXwffq0Ws+YJvWbgbw3Upvjn+Rpbh/xUckxiqFFfU/5j5uCdjMFvUn3YLrLelYAKsaKLKTfMy+OeMbv8wd9By4EjM+A9RB7HKVv3pNZX8fOD9MuSh8y9zV+ZZi+EzcAzJehi9M4mLq7qAmjUgs4qOvtafr6L2dav8Vfw8TarFoD1mk";

    public HardwareMecanum robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new HardwareMecanum();
        robot.init(hardwareMap);


        //colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


    }

    public void encoderIMUCrabsteer(int direction, double inches, double power) {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double powerFinal = Math.abs(power);

        double countsToMove = inches * robot.COUNTS_PER_INCH;

        if (direction == 0) {
            robot.leftFrontDrive.setTargetPosition((int) Math.round(robot.leftFrontDrive.getCurrentPosition() + countsToMove));
            robot.leftRearDrive.setTargetPosition((int) Math.round(robot.leftFrontDrive.getCurrentPosition() - countsToMove));
            robot.rightFrontDrive.setTargetPosition((int) Math.round(robot.leftFrontDrive.getCurrentPosition() + countsToMove));
            robot.rightRearDrive.setTargetPosition((int) Math.round(robot.leftFrontDrive.getCurrentPosition() - countsToMove));

            robot.leftFrontDrive.setPower(powerFinal);
            robot.leftRearDrive.setPower(-powerFinal);
            robot.rightFrontDrive.setPower(powerFinal);
            robot.rightRearDrive.setPower(-powerFinal);
        } else if (direction == 1) {
            robot.leftFrontDrive.setTargetPosition((int) Math.round(robot.leftFrontDrive.getCurrentPosition() - countsToMove));
            robot.leftRearDrive.setTargetPosition((int) Math.round(robot.leftFrontDrive.getCurrentPosition() + countsToMove));
            robot.rightFrontDrive.setTargetPosition((int) Math.round(robot.leftFrontDrive.getCurrentPosition() - countsToMove));
            robot.rightRearDrive.setTargetPosition((int) Math.round(robot.leftFrontDrive.getCurrentPosition() + countsToMove));

            robot.leftFrontDrive.setPower(-powerFinal);
            robot.leftRearDrive.setPower(powerFinal);
            robot.rightFrontDrive.setPower(-powerFinal);
            robot.rightRearDrive.setPower(powerFinal);
        }
        sleep(1000);
    }


    public void crabSteer(int direction, double miliseconds, double power) { //left = 0, right = 1
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long time = System.currentTimeMillis();
        double powerFinal = Math.abs(power);

        while (System.currentTimeMillis() < time + miliseconds)
        {
            if (direction == 0)
            {
                leftFrontDrive.setPower(powerFinal);
                leftRearDrive.setPower(-powerFinal);
                rightFrontDrive.setPower(powerFinal);
                rightRearDrive.setPower(-powerFinal);
            }
            else if (direction == 1)
            {
                leftFrontDrive.setPower(-powerFinal);
                leftRearDrive.setPower(powerFinal);
                rightFrontDrive.setPower(-powerFinal);
                rightRearDrive.setPower(powerFinal);
            }
            telemetry.addData("is it running?", "YEAH!");
        }
        telemetry.addData("EXITING", "YEAH!");
    }
}
