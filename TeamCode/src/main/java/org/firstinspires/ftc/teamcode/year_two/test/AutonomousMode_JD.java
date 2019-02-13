package org.firstinspires.ftc.teamcode.year_two.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.year_two.HardwareMecanum;

import java.util.List;

import static java.lang.Math.abs;
import static java.lang.Thread.sleep;

public class AutonomousMode_JD {

    LinearOpMode autonomousClass;
    HardwareMecanum robot;

    public int position; //Either a 0 or a 1.

    HardwareMap hwMap;

    // The IMU sensor object
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    double targetAngle;
    double globalAngle;

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.3;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.3;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1;      // As tight as we can make it with an integer gyro

    // COEFF - used in the getSteer function to determine speed based upon how large current error is.
    // The steer is clipped between -1 and 1 and then +, - from speed.
    // Example:  a COEFF of 0.1 implies that turns/corrections occur at full power until the error
    // becomes <= 10 degrees.
    // Test runs indicate IMU overturns by 15 degrees so the COEFF must be set to reduce speed before that.
    // Example:  20 degree error ==> 0.05 COEFF
    static final double     P_TURN_COEFF            = 0.05D;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.05D;     // Larger is more responsive, but also less stable

    private static final String VUFORIA_KEY = "AeWrHWf/////AAAAmZG3057lc0JXoVs+HjtHkjZyYL2/IQH4DPGcMKxDXU12F688beSRkSeE6Oz1nH1imNIbBvdwCWFtpqBTu9aqKnlQ9XE3cDLcuUa6/iv0yK3oKy/4p+C1KqltmtvLTda0rgoW8mVcohX38181Apke+iCMjogFT0FHT+3o36MrhYRT03H7Al4Ynqd09uLIGiCXwffq0Ws+YJvWbgbw3Upvjn+Rpbh/xUckxiqFFfU/5j5uCdjMFvUn3YLrLelYAKsaKLKTfMy+OeMbv8wd9By4EjM+A9RB7HKVv3pNZX8fOD9MuSh8y9zV+ZZi+EzcAzJehi9M4mLq7qAmjUgs4qOvtafr6L2dav8Vfw8TarFoD1mk";
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public final double DOOR_START_POS = 0.45D;
    public final double DOOR_OPEN_POS = DOOR_START_POS + 90/280.0;

    public boolean positionDecided;


    public AutonomousMode_JD(LinearOpMode autonomousClass, int position) {
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

        autonomousClass.waitForStart();

        robot.flipperServo.setPosition(robot.FLIPPER_UP_POSITION);

        int goldMineralPosition = 0;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Send telemetry message to alert driver that we are calibrating;
        autonomousClass.telemetry.addData(">", "Calibrating IMU");    //
        autonomousClass.telemetry.update();

        // make sure the gyro is calibrated before continuing
        while (!autonomousClass.isStopRequested() && !imu.isGyroCalibrated())  {
            sleep(50);
            autonomousClass.idle();
        }

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        targetAngle = angles.firstAngle;

        autonomousClass.telemetry.addData(">", "Robot Ready.");    //
        autonomousClass.telemetry.update();

        int i = 0;
        while (i < 10) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            autonomousClass.telemetry.addData("Angle = ", angles.firstAngle);
            autonomousClass.telemetry.update();
            sleep(500);
            i+=1;
        }
        targetAngle = angles.firstAngle;

        if (position == 9) {
            targetAngle = getRelativeAngle(90.0);
            autonomousClass.telemetry.addData("Turn current...", angles.firstAngle);
            autonomousClass.telemetry.addData("     new.......", targetAngle);
            autonomousClass.telemetry.addData("Speed = ", TURN_SPEED);
            autonomousClass.telemetry.addData("COEFF = ", P_TURN_COEFF);
            autonomousClass.telemetry.update();
            gyroTurn( TURN_SPEED, targetAngle);
        }
        if (position == 10) {
            targetAngle = getRelativeAngle(0.0);
            gyroDrive(DRIVE_SPEED, 48.0, 0.0);
        }
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
    }

    /**
     * The gyroDrive, gryoTurn, and gyroHold methods receive an absolute angle value.
     * For most movements, we will want to move the robot +/- a number of degress from
     * current location.  This method takes 2 values:  (1) the currentAngle; (2) the desired # of degrees
     * to move and will return the correct absolute position to pass to the gyro functions.
     */
    public double getRelativeAngle (double moveAngle) {
        double newAngle;
        newAngle = targetAngle + moveAngle;
        if (newAngle > 180)  newAngle -= 360;
        if (newAngle <= -180) newAngle += 360;
        return newAngle;
    }

    public int encoderDrive (double speed,
                             double leftInches, double rightInches,
                             double timeoutS,
                             boolean checkMinerals) throws InterruptedException {
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
            double dropArmByPos = startingPos + (31.5 * robot.COUNTS_PER_INCH);

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
                    int time = 2250;
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

        double powerFinal = abs(power);

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

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - angles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        //angles.getClass() = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) throws InterruptedException {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            leftPower = -power;
            rightPower = power;
        } else if (degrees > 0) {   // turn left.
            leftPower = power;
            rightPower = -power;
        } else return;

        // set power to rotate.

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.leftFrontDrive.setPower(leftPower);
        robot.leftFrontDrive.setPower(leftPower);

        robot.rightFrontDrive.setPower(rightPower);
        robot.rightRearDrive.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (autonomousClass.opModeIsActive() && getAngle() == 0) {
            }

            while (autonomousClass.opModeIsActive() && getAngle() > degrees) {
            }
        } else    // left turn.
            while (autonomousClass.opModeIsActive() && getAngle() < degrees) {
            }

        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);


        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    private void resetAngle() {
        //lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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

    public void unhook() {
        autonomousClass.telemetry.addData("Still running", 1);
        autonomousClass.telemetry.update();
        robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() + 300);
        robot.armLower.setPower(-1);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        autonomousClass.telemetry.addData("Still running", 2);
        autonomousClass.telemetry.update();
        //robot.hookServo.setPosition(0.2);

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        autonomousClass.telemetry.addData("Still running", 3);
        autonomousClass.telemetry.update();
        robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() + 9000);
        robot.armLower.setPower(1);
        autonomousClass.telemetry.addData("Still running", 4);
        autonomousClass.telemetry.update();
        //crabSteer(1, 500, .5);
    }

    public void pushMineral(int position) throws InterruptedException {
        positionDecided = true;

        encoderDrive(.15, 13, 13, 10, false);

        sleep(500);

        encoderDrive(.15, -13, -13, 10, false);

        sleep(500);

        robot.flipperServo.setPosition(0);
    }

    public void crabSteer(int direction, double miliseconds, double power) throws
            InterruptedException //left = 0, right = 1
    {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long time = System.currentTimeMillis();
        double powerFinal = abs(power);

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
    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    /**
     *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
     * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        final double CRCTN_FACTOR = 0.05;

        // Ensure that the opmode is still active
        if (autonomousClass.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * robot.COUNTS_PER_INCH);

            newLeftTarget = robot.leftFrontDrive.getCurrentPosition() + moveCounts;
            robot.leftFrontDrive.setTargetPosition(newLeftTarget);
            newLeftTarget = robot.leftRearDrive.getCurrentPosition() + moveCounts;
            robot.leftRearDrive.setTargetPosition(newLeftTarget);
            newRightTarget = robot.rightFrontDrive.getCurrentPosition() + moveCounts;
            robot.rightFrontDrive.setTargetPosition(newRightTarget);
            newRightTarget = robot.rightRearDrive.getCurrentPosition() + moveCounts;
            robot.rightRearDrive.setTargetPosition(newRightTarget);

            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            //speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            //telemetry.addData("Drive Speed = ", speed);
            //telemetry.update();
            robot.leftFrontDrive.setPower(speed);
            robot.leftRearDrive.setPower(speed);
            robot.rightFrontDrive.setPower(speed);
            robot.rightRearDrive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (autonomousClass.opModeIsActive() &&
                    (robot.leftRearDrive.isBusy() && robot.leftFrontDrive.isBusy() &&
                     robot.rightRearDrive.isBusy() && robot.rightFrontDrive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                //steer = getSteer(error, P_DRIVE_COEFF);
                /*
                  steer calculation based on the following table of desired speeds based on error:
                       ERROR    SPEED-LEFT  SPEED-RIGHT
                       1        0.3         0.3         >> no correction needed
                       2        0.285       0.3
                       3        0.27        0.3
                       4        0.255       0.3
                       5        0.24        0.3
                       10       0.165       0.3
                       15       0.09        0.3
                   ASSUMPITONS:  desired drive speed = 0.03 and CRCTN_FACTOR = 0.05
                 */
                steer = 1 - (CRCTN_FACTOR * (abs(error) - 1));

                // error > 0 ==> turn <LEFT>  when moving forward
                // error < 0 ==> turn <RIGHT> when moving forward

                // if driving in reverse, the motor correction also needs to be reversed.
                // by reversing error, we switch which wheel has reduced speed
                if (distance < 0)
                    //steer *= -1.0;
                    error = error * -1;

                if (error >= 0) {
                    leftSpeed = speed * steer;
                    rightSpeed = speed;
                }
                else {
                    leftSpeed = speed;
                    rightSpeed = speed * steer;
                }
                autonomousClass.telemetry.addData("Left Speed = ", leftSpeed);
                autonomousClass.telemetry.addData("Right Speed = ", rightSpeed);
                autonomousClass.telemetry.update();

                // Normalize speeds if either one exceeds +/- 1.0;
                //max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                //if (max > 1.0)
                //{
                //    leftSpeed /= max;
                //    rightSpeed /= max;
                //}

                robot.leftFrontDrive.setPower(leftSpeed);
                robot.leftRearDrive.setPower(leftSpeed);
                robot.rightFrontDrive.setPower(rightSpeed);
                robot.rightRearDrive.setPower(rightSpeed);
            }

            // Stop all motion;
            robot.leftFrontDrive.setPower(0);
            robot.leftRearDrive.setPower(0);
            robot.rightFrontDrive.setPower(0);
            robot.rightRearDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (autonomousClass.opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            autonomousClass.telemetry.update();
        }
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *
     * @param speed      Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (autonomousClass.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            //telemetry.update();
        }

        // Stop all motion;
        robot.leftRearDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
    }

    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;
        final double ERROR_THRESHOLD = 20; //angle at which we begin to reduce turning speed

        // determine turn power based on +/- error
        error = getError(angle);
        // error > 0 ==> turn <LEFT>  when moving forward
        // error < 0 ==> turn <RIGHT> when moving forward

        //determine our 'safe' zone.  If error within threshold, no corrections are made
        if (abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            //steer = Range.clip(getSteer(error, PCoeff), TURN_SPEED * -1, TURN_SPEED);
            if (error < 0)
                speed = speed * -1;

            // divide the ERROR_THRESHOLD into quartiles to reduce speed as we near our heading
            // for each quartile, we will reduce speed by power of 2

            if (error <= ERROR_THRESHOLD * 0.25)
                //speed = speed / Math.pow(2, 4);
                speed = speed / 8;
            else if (error <= ERROR_THRESHOLD * 0.50)
                //speed = speed / Math.pow(2, 3);
                speed = speed / 6;
            else  if (error <= ERROR_THRESHOLD * 0.75)
                //speed = speed / Math.pow(2, 2);
                speed = speed / 4;
            else if (error <= ERROR_THRESHOLD)
                //speed = speed / Math.pow(2, 3);
                speed = speed / 2;
            else
                speed = speed;

            // set LEFT and RIGHT speed as negation of each other to ensure that robot
            // turns in place
            rightSpeed  = speed;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.leftFrontDrive.setPower(leftSpeed);
        robot.leftRearDrive.setPower(leftSpeed);
        robot.rightFrontDrive.setPower(rightSpeed);
        robot.rightRearDrive.setPower(rightSpeed);

        // Display it for the driver.
        autonomousClass.telemetry.addData("Angle/Error", "%5.2f/%5.2f", angles.firstAngle, error);
        autonomousClass.telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        //robotError = targetAngle - gyro.getIntegratedZValue();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
