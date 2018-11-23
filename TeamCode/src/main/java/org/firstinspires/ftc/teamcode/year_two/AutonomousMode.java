package org.firstinspires.ftc.teamcode.year_two;

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

    final int WHEEL_DIAMETER_INCHES = 6;
    final double WHEEL_CIRCUMFERENCE = 6 * Math.PI;

    HardwareMap hwMap;

    Orientation lastAngles = new Orientation();
    double globalAngle;


    public AutonomousMode(LinearOpMode autonomousClass, int position)
    {
        robot = new HardwareMecanum();
        this.hwMap = autonomousClass.hardwareMap;

        this.position = position;
    }

    public void init()
    {
        robot.init(hwMap);
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        unhook();
    }

    public void unhook()
    {
        int rightAngleCounts = 9100;
        //robot.armLowerRight.setTargetPosition(robot.armLowerRight.getCurrentPosition() + rightAngleCounts);
        crabSteer(0, 1, 1);
    }

    public void drive(double leftInches, double rightInches, double power)
    {
        robot.leftFrontDrive.setTargetPosition((int) Math.round(-(robot.leftFrontDrive.getCurrentPosition() - (robot.WHEELS_COUNTS_PER_SHAFT_REV * WHEEL_CIRCUMFERENCE * leftInches))));
        robot.leftRearDrive.setTargetPosition((int) Math.round(-(robot.leftRearDrive.getCurrentPosition() - (robot.WHEELS_COUNTS_PER_SHAFT_REV * WHEEL_CIRCUMFERENCE * leftInches))));

        robot.rightFrontDrive.setTargetPosition((int) Math.round(robot.rightFrontDrive.getCurrentPosition() + (robot.WHEELS_COUNTS_PER_SHAFT_REV * WHEEL_CIRCUMFERENCE * rightInches)));
        robot.rightRearDrive.setTargetPosition((int) Math.round(robot.rightRearDrive.getCurrentPosition() + (robot.WHEELS_COUNTS_PER_SHAFT_REV * WHEEL_CIRCUMFERENCE * rightInches)));

        robot.leftFrontDrive.setPower(power);
        robot.leftRearDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
        robot.rightRearDrive.setPower(power);
    }

    public void crabSteer(int direction, double inches, double power) //0 = left, 1 = right
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
}
