package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;

public class AutonomousMode {

    LinearOpMode autonomousClass;
    HardwareMecanum robot;

    int position; //Either a 0 or a 1.

    HardwareMap hwMap;

    Orientation lastAngles = new Orientation();
    double globalAngle;


    public AutonomousMode(LinearOpMode autonomousClass, int position)
    {
        robot = new HardwareMecanum();
        this.hwMap = autonomousClass.hardwareMap;

        this.position = position;
        this.autonomousClass = autonomousClass;
    }

    public void startAutonomousMode() throws InterruptedException {
        robot.init(hwMap);

        //robot.leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        //robot.leftRearDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        autonomousClass.waitForStart();

        encoderCrabsteer(0, 5, .3);

        encoderDrive(.3, 120, 120, 20);
    }

    public void unhook()
    {
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
        robot.hookServo.setPosition(0.2);

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

    public void crabSteer(int direction, double miliseconds, double power) throws InterruptedException //left = 0, right = 1
    {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        long time = System.currentTimeMillis();
        double powerFinal = Math.abs(power);

        while (System.currentTimeMillis() < time + miliseconds)
        {
            if (direction == 0)
            {
                robot.leftFrontDrive.setPower(powerFinal);
                robot.leftRearDrive.setPower(-powerFinal);
                robot.rightFrontDrive.setPower(powerFinal);
                robot.rightRearDrive.setPower(-powerFinal);
            }
            else if (direction == 1)
            {
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

        if (direction == 0)
        {
            robot.leftFrontDrive.setTargetPosition((int) Math.round(robot.leftFrontDrive.getCurrentPosition() + countsToMove));
            robot.leftRearDrive.setTargetPosition((int) Math.round(robot.leftFrontDrive.getCurrentPosition() - countsToMove));
            robot.rightFrontDrive.setTargetPosition((int) Math.round(robot.leftFrontDrive.getCurrentPosition() + countsToMove));
            robot.rightRearDrive.setTargetPosition((int) Math.round(robot.leftFrontDrive.getCurrentPosition() - countsToMove));

            robot.leftFrontDrive.setPower(powerFinal);
            robot.leftRearDrive.setPower(-powerFinal);
            robot.rightFrontDrive.setPower(powerFinal);
            robot.rightRearDrive.setPower(-powerFinal);
        }
        else if (direction == 1)
        {
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

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
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
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) throws InterruptedException {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else return;

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
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (autonomousClass.opModeIsActive() && getAngle() == 0) {}

            while (autonomousClass.opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (autonomousClass.opModeIsActive() && getAngle() < degrees) {}

        robot.rightFrontDrive.setPower(0);
        robot.rightRearDrive.setPower(0);
        robot.leftFrontDrive.setPower(0);
        robot.leftRearDrive.setPower(0);


        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
    private void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // negate left-side motors due position on robot
        leftInches = -leftInches;

        // Ensure that the opmode is still active
        if (autonomousClass.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)(leftInches * robot.COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)(rightInches * robot.COUNTS_PER_INCH);
            newLeftRearTarget = robot.leftRearDrive.getCurrentPosition() + (int)(leftInches * robot.COUNTS_PER_INCH);
            newRightRearTarget = robot.rightRearDrive.getCurrentPosition() + (int)(rightInches * robot.COUNTS_PER_INCH);
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
            robot.leftFrontDrive.setPower(speed);
            robot.rightFrontDrive.setPower(speed);
            robot.leftRearDrive.setPower(speed);
            robot.rightRearDrive.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when ANY motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (autonomousClass.opModeIsActive() &&
                    (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy() &&
                            robot.leftRearDrive.isBusy() && robot.rightRearDrive.isBusy())) {

                // Display it for the driver.
                autonomousClass.telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d",
                        newLeftFrontTarget,  newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
                autonomousClass.telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftRearDrive.getCurrentPosition(),
                        robot.rightRearDrive.getCurrentPosition());
                autonomousClass.telemetry.update();
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

            //  sleep(250);   // optional pause after each move
        }
    }
}
