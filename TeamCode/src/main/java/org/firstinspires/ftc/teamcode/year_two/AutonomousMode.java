package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static java.lang.Thread.sleep;

public class AutonomousMode {

    LinearOpMode autonomousClass;
    HardwareMecanum robot;

    public int position; //Either a 0 or a 1.

    HardwareMap hwMap;

    Orientation lastAngles = new Orientation();
    Orientation originalAngle = new Orientation();
    double globalAngle, power = .30, correction;

    private static final String VUFORIA_KEY = "AeWrHWf/////AAAAmZG3057lc0JXoVs+HjtHkjZyYL2/IQH4DPGcMKxDXU12F688beSRkSeE6Oz1nH1imNIbBvdwCWFtpqBTu9aqKnlQ9XE3cDLcuUa6/iv0yK3oKy/4p+C1KqltmtvLTda0rgoW8mVcohX38181Apke+iCMjogFT0FHT+3o36MrhYRT03H7Al4Ynqd09uLIGiCXwffq0Ws+YJvWbgbw3Upvjn+Rpbh/xUckxiqFFfU/5j5uCdjMFvUn3YLrLelYAKsaKLKTfMy+OeMbv8wd9By4EjM+A9RB7HKVv3pNZX8fOD9MuSh8y9zV+ZZi+EzcAzJehi9M4mLq7qAmjUgs4qOvtafr6L2dav8Vfw8TarFoD1mk";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public final double DOOR_START_POS = 0.45D;
    public final double DOOR_OPEN_POS = DOOR_START_POS + 90/280.0;

    public boolean positionDecided;


    public AutonomousMode(LinearOpMode autonomousClass, int position) {
        robot = new HardwareMecanum();
        this.hwMap = autonomousClass.hardwareMap;

        //AutonomousMode class is passed an integer position value based on starting position of robot where...
        //     2 = Crater side
        //     3 = Depot side
        this.position = position;
        this.autonomousClass = autonomousClass;
    }

    public void startAutonomousMode() throws InterruptedException {
        robot.init(hwMap);

        initVuforia();
        initTfod();

        tfod.activate();

        //robot.leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //robot.leftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        robot.imu.initialize(parameters);

        autonomousClass.telemetry.addData("Mode", "calibrating...");
        autonomousClass.telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!autonomousClass.isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            autonomousClass.idle();
        }

        autonomousClass.telemetry.addData("Mode", "waiting for start");
        autonomousClass.telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        autonomousClass.telemetry.update();

        autonomousClass.waitForStart();

        robot.flipperServo.setPosition(robot.FLIPPER_UP_POSITION);

        int goldMineralPosition = 0;

        //This starts autonomous mode where robot begins on CRATER side
        if (position == 2) {
            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() + 6000);

            robot.armLower.setPower(.5);
            Thread.sleep(3500);

            encoderCrabsteer(0, 3.5, .5);
            sleep(500);

            encoderDrive(.3, 14, 14, 10, false);
            sleep(500);

            encoderDrive(.3, -19, 19, 10, false);
            sleep(500);

            robot.armUpper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armUpper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armUpper.setTargetPosition(robot.armUpper.getCurrentPosition() - 10750);
            robot.armUpper.setPower(.3);

            robot.armLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() - 1250);
            robot.armLower.setPower(.3);

            encoderDrive(.3, -24, -24, 10, false);
            sleep(500);

            autonomousClass.telemetry.addData("Starting Drive Forward 40 Inches", null);
            autonomousClass.telemetry.update();

            int targetPos = encoderDrive(.5, 68, 68, 10, true);

            autonomousClass.telemetry.addData("Start Checking for Gold Mineral", null);
            autonomousClass.telemetry.update();

            encoderDrive(.3, -7.0,7.0,10, false);
            sleep(500);

            robot.armLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() - 5950);
            robot.armLower.setPower(1);

            encoderDrive(.7, 36,36, 10, false);
            sleep(500);

            robot.door.setPosition(DOOR_OPEN_POS);
            sleep(700);

            encoderDrive(.3, -3, 3, 10, false); //TODO Was 1.5
            sleep(250);

            robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            encoderDrive(.5, -68,-68, 10, false);
        }

        //This begins autonomous mode where robot begins on DEPOT side
        if (position == 3) {
            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() + 6000);

            robot.armLower.setPower(.5);
            Thread.sleep(3500);

            encoderCrabsteer(0, 3.5, .5);
            sleep(500);

            encoderDrive(.3, 14, 14, 10, false);
            sleep(500);

            encoderDrive(.3, -19, 19, 10, false);
            sleep(500);

            robot.armUpper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armUpper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armUpper.setTargetPosition(robot.armUpper.getCurrentPosition() - 10750);
            robot.armUpper.setPower(.3);

            robot.armLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() - 1250);
            robot.armLower.setPower(.3);

            encoderDrive(.3, -24, -24, 10, false);
            sleep(500);

            autonomousClass.telemetry.addData("Starting Drive Forward 40 Inches", null);
            autonomousClass.telemetry.update();

            int targetPos = encoderDrive(.5, 68, 68, 10, true);

            autonomousClass.telemetry.addData("Start Checking for Gold Mineral", null);
            autonomousClass.telemetry.update();

            encoderDrive(.3, 25.5, -25.5, 10, false);
            sleep(250);

            robot.armLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() - 5950);
            robot.armLower.setPower(1);

            encoderDrive(.7, 47,47, 10, false);
            sleep(500);

            robot.door.setPosition(DOOR_OPEN_POS);
            sleep(700);

            encoderDrive(.3, 2.5, -2.5, 10, false); //TODO Was 1.5
            sleep(250);

            robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            encoderDrive(.5, -62,-62,10, false);

        }

        if (position == 1) {

            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            autonomousClass.telemetry.addData("currentPos", robot.armLower.getCurrentPosition());
            autonomousClass.telemetry.update();

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() + 6300);

            robot.armLower.setPower(.5);
            Thread.sleep(3500);

            autonomousClass.telemetry.addData("currentPos", robot.armLower.getCurrentPosition());
            autonomousClass.telemetry.update();

            encoderCrabsteer(0, 3.5, .5);

            //TODO Reset lower arm encoders


            encoderDrive(.3, 15, 15, 10, false);
            sleep(500);

            robot.armLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armUpper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armUpper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() - 5800);
            robot.armUpper.setTargetPosition(robot.armUpper.getCurrentPosition() - 20000);

            robot.armLower.setPower(.2);
            robot.armUpper.setPower(1);

            encoderCrabsteer(1, 20, .3);
            sleep(2000);
            if (goldMineralIsPresent() && !positionDecided) {
                goldMineralPosition = 1;
                autonomousClass.telemetry.addData("mineralPosition", goldMineralPosition);
                autonomousClass.telemetry.update();
                pushMineral(1);
            }

            encoderCrabsteer(0, 20, .3);
            sleep(2000);
            if (goldMineralIsPresent() && !positionDecided) {
                goldMineralPosition = 2;
                autonomousClass.telemetry.addData("mineralPosition", goldMineralPosition);
                autonomousClass.telemetry.update();

                pushMineral(2);
            }

            encoderCrabsteer(0, 20, .3);
            sleep(2000);
            if (goldMineralIsPresent() && !positionDecided) {
                goldMineralPosition = 3;
                autonomousClass.telemetry.addData("mineralPosition", goldMineralPosition);
                autonomousClass.telemetry.update();

                pushMineral(3);
            }

            if (positionDecided == false) {

            }

            encoderCrabsteer(0, 34, .3);
            sleep(2500);
            encoderDrive(.3, 8.5, -8.5, 10, false); //Turn parallel to wall.
            sleep(250);

            encoderDrive(.6, 48.5, 48.5, 20, false);
            sleep(2000);

            robot.door.setPosition(.45D + 90/280.0);
            sleep(1500);
            robot.door.setPosition(.45D);

            //TODO Place marker

            encoderDrive(.6, -82, -82, 20, false);
            sleep(2500);
        }

        if (position == 0) {

            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            autonomousClass.telemetry.addData("currentPos", robot.armLower.getCurrentPosition());
            autonomousClass.telemetry.update();

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() + 6300);

            robot.armLower.setPower(.5);
            Thread.sleep(3500);

            encoderCrabsteer(0, 3.5, .5);

            sleep(2000);

            autonomousClass.telemetry.addData("currentPos", robot.armLower.getCurrentPosition());
            autonomousClass.telemetry.update();

            //TODO Reset lower arm encoders

            encoderDrive(.4, 45, 45, 10, false);

            /*
            encoderDrive(.3, 15, 15, 10);
            sleep(500);

            robot.armLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armUpper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armUpper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() - 5800);
            robot.armUpper.setTargetPosition(robot.armUpper.getCurrentPosition() - 20000);

            robot.armLower.setPower(.2);
            robot.armUpper.setPower(1);

            encoderCrabsteer(1, 20, .3);
            sleep(1500);
            if (goldMineralIsPresent()) {
                goldMineralPosition = 1;
                autonomousClass.telemetry.addData("mineralPosition", goldMineralPosition);
                autonomousClass.telemetry.update();
                pushMineral(1);
            }

            encoderCrabsteer(0, 20, .3);
            sleep(1500);
            if (goldMineralIsPresent()) {
                goldMineralPosition = 2;
                autonomousClass.telemetry.addData("mineralPosition", goldMineralPosition);
                autonomousClass.telemetry.update();

                pushMineral(2);
            }

            encoderCrabsteer(0, 20, .3);
            sleep(1500);
            if (goldMineralIsPresent()) {
                goldMineralPosition = 3;
                autonomousClass.telemetry.addData("mineralPosition", goldMineralPosition);
                autonomousClass.telemetry.update();

                pushMineral(3);
            }

            if (positionDecided == false) {

            }


            encoderCrabsteer(0, 37, .3);
            sleep(2500);
            encoderDrive(.3, -8.5, 8.5, 10); //Turn parallel to wall.
            sleep(250);

            encoderDrive(.3, -40, -40, 20);
            sleep(2500);


            robot.armLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armUpper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armUpper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() + 6500);
            robot.armUpper.setTargetPosition(robot.armUpper.getCurrentPosition() - 20000);

            robot.armLower.setPower(.3);
            robot.armUpper.setPower(.3);

            sleep(15000);

            encoderDrive(.3, 72, 72, 20);
            sleep(2500);
            */
        }

        if (position == 10) //Depot
        {
            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() + 5500);

            robot.armLower.setPower(.5);
            Thread.sleep(3500);

            encoderCrabsteer(0, 3.5, .5);
            sleep(500);

            encoderDrive(.3, 15, 15, 10, false);
            sleep(500);

            //rotate((Math.round((originalAngle.firstAngle + 90) - robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)), .5);
            rotate(90, .4);

            robot.armUpper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armUpper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armUpper.setTargetPosition(robot.armUpper.getCurrentPosition() - 10750);
            robot.armUpper.setPower(.3);

            robot.armLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() - 1250);
            robot.armLower.setPower(.3);

            encoderDrive(.3, -25, -25, 10, false);
            sleep(500);

            autonomousClass.telemetry.addData("Starting Drive Forward 40 Inches", null);
            autonomousClass.telemetry.update();

            int targetPos = encoderDrive(.5, 68, 68, 10, true);

            autonomousClass.telemetry.addData("Start Checking for Gold Mineral", null);
            autonomousClass.telemetry.update();

            rotate(-100, .4);

            robot.armLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() - 5950);
            robot.armLower.setPower(1);

            encoderDrive(.8, 47,47, 10, false);
            sleep(500);

            robot.door.setPosition(DOOR_OPEN_POS);
            sleep(500);

            rotate(10, .4);

            robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            encoderDrive(.8, -62,-62,10, false);
        }


        if (position == 11)
        {
            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() + 5500);

            robot.armLower.setPower(.5);
            Thread.sleep(3500);

            encoderCrabsteer(0, 3.5, .5);
            sleep(500);

            encoderDrive(.3, 15, 15, 10, false);
            sleep(500);

            rotate(90, .4);

            robot.armUpper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armUpper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armUpper.setTargetPosition(robot.armUpper.getCurrentPosition() - 10750);
            robot.armUpper.setPower(.3);

            robot.armLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() - 1250);
            robot.armLower.setPower(.3);

            encoderDrive(.3, -25, -25, 10, false);
            sleep(500);

            autonomousClass.telemetry.addData("Starting Drive Forward 40 Inches", null);
            autonomousClass.telemetry.update();

            int targetPos = encoderDrive(.5, 68, 68, 10, true);

            autonomousClass.telemetry.addData("Start Checking for Gold Mineral", null);
            autonomousClass.telemetry.update();

            rotate(25, .4);

            robot.armLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() - 5950);
            robot.armLower.setPower(1);

            encoderDrive(.8, 36,36, 10, false);
            sleep(500);

            robot.door.setPosition(DOOR_OPEN_POS);
            sleep(500);

            rotate(20, .4);

            robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            encoderDrive(.8, -68,-68, 10, false);

        }
    }

    public void rampSpeedDrive() {

    }

    public int encoderDrive (double speed,
                             double leftInches, double rightInches,
                             double timeoutS,
                             boolean checkMinerals) throws InterruptedException { //All mineral detection stuff here
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // negate left-side motors due position on robot
        leftInches = -leftInches;

        // Ensure that the opmode is still active
        if (autonomousClass.opModeIsActive()) {

            int startingPos = robot.rightFrontDrive.getCurrentPosition();

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int) (leftInches * robot.COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int) (rightInches * robot.COUNTS_PER_INCH);
            newLeftRearTarget = robot.leftRearDrive.getCurrentPosition() + (int) (leftInches * robot.COUNTS_PER_INCH);
            newRightRearTarget = robot.rightRearDrive.getCurrentPosition() + (int) (rightInches * robot.COUNTS_PER_INCH);
            robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            robot.leftRearDrive.setTargetPosition(newLeftRearTarget);
            robot.rightRearDrive.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            if (checkMinerals) {
                robot.leftFrontDrive.setPower(0.07);
                robot.rightFrontDrive.setPower(0.07);
                robot.leftRearDrive.setPower(0.07);
                robot.rightRearDrive.setPower(0.07);
            }
            else {
                robot.leftFrontDrive.setPower(speed);
                robot.rightFrontDrive.setPower(speed);
                robot.leftRearDrive.setPower(speed);
                robot.rightRearDrive.setPower(speed);
            }

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when ANY motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.

            boolean goldMineralFound = false;
            double firstInches = startingPos + (4 * robot.COUNTS_PER_INCH);
            double dropArmByPos = startingPos + (34 * robot.COUNTS_PER_INCH);

            while (autonomousClass.opModeIsActive() &&
                    (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy() &&
                            robot.leftRearDrive.isBusy() && robot.rightRearDrive.isBusy())) {

                // Display it for the driver.
                autonomousClass.telemetry.addData("Target:  ", "Running to %7d :%7d :%7d :%7d",
                        newLeftFrontTarget, newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
                autonomousClass.telemetry.addData("Current: ", "Running at %7d :%7d :%7d :%7d",
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftRearDrive.getCurrentPosition(),
                        robot.rightRearDrive.getCurrentPosition());
                autonomousClass.telemetry.update();

                if ((checkMinerals && goldMineralIsPresent() && !goldMineralFound) ||
                        (checkMinerals && !goldMineralFound && (robot.rightFrontDrive.getCurrentPosition() > dropArmByPos)))
                {
                    goldMineralFound = true;
                    robot.flipperServo.setPosition(robot.FLIPPER_DOWN_POSITION);
                    int time = 2750;
                    if (robot.rightFrontDrive.getCurrentPosition() < firstInches)
                    {
                        time = 3000;
                    }
                    sleep(time);
                    robot.flipperServo.setPosition(robot.FLIPPER_UP_POSITION);
                    //break;
                    robot.leftFrontDrive.setPower(speed);
                    robot.rightFrontDrive.setPower(speed);
                    robot.leftRearDrive.setPower(speed);
                    robot.rightRearDrive.setPower(speed);
                }

            }

            // Stop all motion;
            robot.leftFrontDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightRearDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            return newRightFrontTarget;
            //  sleep(250);   // optional pause after each move
        }
        return 0;
    }

    public void encoderCrabsteer(int direction, double inches, double power) throws InterruptedException //left = 0, right = 1
    {
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
        autonomousClass.telemetry.addData("is it running?", "YEAH!");

        autonomousClass.telemetry.addData("EXITING", "YEAH!");
        sleep(1000);
    }

    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double getRelativeAngle()
    {
        return robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - originalAngle.firstAngle;
    }

    private void rotate(int degrees, double power)
    {
        robot.leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double  actualDegrees;

        if (degrees > 0)
        {
            actualDegrees = degrees -15;
        }
        else
        {
            actualDegrees = degrees + 15;
        }

        double  leftPower, rightPower;
        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (actualDegrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = -power;
        }
        else if (actualDegrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        robot.leftFrontDrive.setPower(leftPower);
        robot.leftRearDrive.setPower(leftPower);

        robot.rightFrontDrive.setPower(rightPower);
        robot.rightRearDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (actualDegrees < 0)
        {
            // On right turn we have to get off zero first.
            while (autonomousClass.opModeIsActive() && getAngle() == 0) {}

            while (autonomousClass.opModeIsActive() && getAngle() > actualDegrees) {}
        }
        else    // left turn.
            while (autonomousClass.opModeIsActive() && getAngle() < actualDegrees) {}

        // turn the motors off.
        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);

        // wait for rotation to stop.
        autonomousClass.sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public boolean goldMineralIsPresent() throws InterruptedException {
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            autonomousClass.telemetry.addData("This part is executing", "yes");
            boolean goldPresent = false;
            boolean silverPresent = false;
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    goldPresent = true;
                }
                else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                    silverPresent = true;
                }
            }
            if (goldPresent && !silverPresent) {
                return true;
            }
        }
        return false;
    }

    public void pushMineral(int position) throws InterruptedException {
        positionDecided = true;

        encoderDrive(.15, 13, 13, 10, false);

        sleep(500);

        encoderDrive(.15, -13, -13, 10, false);

        sleep(500);

        robot.flipperServo.setPosition(0);
    }

     /*
    public void drive(double leftInches, double rightInches, double power)
    {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontDrive.setTargetPosition((int) Math.round(-(robot.leftFrontDrive.getCurrentPosition() - (robot.WHEELS_COUNTS_PER_SHAFT_REV * robot.WHEEL_CIRCUMFERENCE * leftInches))));
        robot.leftRearDrive.setTargetPosition((int) Math.round(-(robot.leftRearDrive.getCurrentPosition() - (robot.WHEELS_COUNTS_PER_SHAFT_REV * robot.WHEEL_CIRCUMFERENCE * leftInches))));

        robot.rightFrontDrive.setTargetPosition((int) Math.round(robot.rightFrontDrive.getCurrentPosition() + (robot.WHEELS_COUNTS_PER_SHAFT_REV * robot.WHEEL_CIRCUMFERENCE * rightInches)));
        robot.rightRearDrive.setTargetPosition((int) Math.round(robot.rightRearDrive.getCurrentPosition() + (robot.WHEELS_COUNTS_PER_SHAFT_REV * robot.WHEEL_CIRCUMFERENCE * rightInches)));

        robot.leftFrontDrive.setPower(power);
        robot.leftRearDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
        robot.rightRearDrive.setPower(power);
    }*/

    /*public void crabSteer(int direction, double inches, double power) //0 = left, 1 = right
    {
        double modifiedInches = inches * 3;

        int leftFrontModifier = 0; //If direction is not either a 1 or a 0, the robot will not crabsteer.
        int leftRearModifier = 0;
        int rightFrontModifier = 0;
        int rightRearModifier = 0;

        if (direction == 0){
            leftFrontModifier = -1;
            leftRearModifier = 1;
            rightFrontModifier= -1;
            rightRearModifier = 1;
        }
        else if (direction == 1)
        {
            leftFrontModifier = 1;
            leftRearModifier = -1;
            rightFrontModifier = 1;
            rightRearModifier = -1;
        }

        while (true) {

            robot.leftFrontDrive.setTargetPosition((int) Math.round(-(robot.leftFrontDrive.getCurrentPosition() - (robot.WHEELS_COUNTS_PER_SHAFT_REV * WHEEL_CIRCUMFERENCE * inches))));
            robot.leftRearDrive.setTargetPosition((int) Math.round(-(robot.leftRearDrive.getCurrentPosition() - (robot.WHEELS_COUNTS_PER_SHAFT_REV * WHEEL_CIRCUMFERENCE * inches))));

            robot.rightFrontDrive.setTargetPosition((int) Math.round(robot.rightFrontDrive.getCurrentPosition() + (robot.WHEELS_COUNTS_PER_SHAFT_REV * WHEEL_CIRCUMFERENCE * inches)));
            robot.rightRearDrive.setTargetPosition((int) Math.round(robot.rightRearDrive.getCurrentPosition() + (robot.WHEELS_COUNTS_PER_SHAFT_REV * WHEEL_CIRCUMFERENCE * inches)));

            robot.leftFrontDrive.setPower(power);
            robot.leftRearDrive.setPower(power);
            robot.rightFrontDrive.setPower(power);
            robot.rightRearDrive.setPower(power);
        }
    }*/

    public void crabSteer(int direction, double miliseconds, double power) throws
            InterruptedException //left = 0, right = 1
    {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long time = System.currentTimeMillis();
        double powerFinal = Math.abs(power);

        while (System.currentTimeMillis() < time + miliseconds) {
            if (direction == 0) {
                robot.leftFrontDrive.setPower(powerFinal);
                robot.leftRearDrive.setPower(-powerFinal);
                robot.rightFrontDrive.setPower(powerFinal);
                robot.rightRearDrive.setPower(-powerFinal);
            } else if (direction == 1) {
                robot.leftFrontDrive.setPower(-powerFinal);
                robot.leftRearDrive.setPower(powerFinal);
                robot.rightFrontDrive.setPower(-powerFinal);
                robot.rightRearDrive.setPower(powerFinal);
            }
            autonomousClass.telemetry.addData("is it running?", "YEAH!");
        }
        autonomousClass.telemetry.addData("EXITING", "YEAH!");
        sleep(1000);
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = autonomousClass.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", autonomousClass.hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
