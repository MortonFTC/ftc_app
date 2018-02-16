/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//@Disabled
@Autonomous(name="Mecanum: Auto Drive By Encoder (Red Short)", group="Auto")
public class MecanumAutoDriveByEncoder_Linear_RedShort extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareMecanum         robot   = new HardwareMecanum();   // Use a Mecanum's hardware
    AutonomousMecanum       auto = new AutonomousMecanum();
    private ElapsedTime     runtime = new ElapsedTime();

    OpenGLMatrix lastLocation = null;

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

       parameters.vuforiaLicenseKey = auto.vuforiaLicenseKey_MortonFTC;

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armTilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armTilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armExtender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d: %7d :%7d",
            robot.leftFrontDrive.getCurrentPosition(),
            robot.rightFrontDrive.getCurrentPosition(),
            robot.leftRearDrive.getCurrentPosition(),
            robot.rightRearDrive.getCurrentPosition(),
            robot.armTilt.getCurrentPosition(),
            robot.armExtender.getCurrentPosition(),
            robot.armLift.getCurrentPosition());
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();
        runtime.reset();


        relicTrackables.activate();

        while (opModeIsActive()) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN || (runtime.seconds() >= AutonomousMecanum.vuMarkTimeout)) {

                /* Found an instance of the template or 15 seconds has elapsed */
                telemetry.addData("VuMark", "%s visible", vuMark);


                /** BALL KNOCK OFF SEQUENCE **/
                robot.swingServo.setPosition(0.85);
                robot.ballArmServo.setPosition(0.2);
                sleep(500);
                robot.ballArmServo.setPosition(0.08);
                sleep(500);

                // determine ball color
                int redValue = robot.colorSensor.red();
                int blueValue = robot.colorSensor.blue();

                boolean isRed = (redValue > blueValue) ? true : false;
                String color = (isRed) ? "red" : "blue";

                telemetry.addData("color", color);
                telemetry.addData("red", redValue);
                telemetry.addData("blue", blueValue);
                telemetry.update();

                // knock ball off
                if (isRed) {
                    robot.swingServo.setPosition(1.2);
                } else {
                    robot.swingServo.setPosition(0.6);
                }

                // reset ball knock off servos
                sleep(250);
                robot.ballArmServo.setPosition(0.4);
                sleep(250);
                robot.swingServo.setPosition(0.5);
                sleep(250);
                robot.ballArmServo.setPosition(0.5);


                /** TRAVEL SEQUENCE **/
                if (vuMark == RelicRecoveryVuMark.UNKNOWN)           {
                    vuMark = RelicRecoveryVuMark.CENTER;
                }
                double vuTravel = 0;
                switch (vuMark) {
                    case LEFT:
                        vuTravel = -42.30;
                        break;
                    case CENTER:
                        vuTravel = -34.67;
                        break;
                    case RIGHT:
                        vuTravel = -27.04;
                        break;
                }

                // Step through each leg of the path,
                // Note: Reverse movement is obtained by setting a negative distance (not speed)
                encoderDrive(auto.DRIVE_SPEED,  vuTravel,  vuTravel, 15.0);  // S1: Forward 47 Inches with 15 Sec timeout
                sleep(500);
                encoderDrive(auto.TURN_SPEED,   -21.7, 21.7, 15.0);  // S2: Turn Right 12 Inches with 15 Sec timeout
                sleep(250);
                encoderTilt(0.5, 40, 15.0);
                sleep(250);
                encoderDrive(auto.DRIVE_SPEED, 6.05, 6.05, 15.0);  // S3: Reverse 24 Inches with 15 Sec timeout
                sleep(250);

                // Move both servos to new position.  Assume servos are mirror image of each other.
                //clawOffset_glyph = Range.clip(clawOffset_glyph, -0.0625, .125);//-0.2,-0.05
                robot.leftGripper.setPosition(auto.leftGripperPosition);//(robot.MID_SERVO +.02 )
                robot.rightGripper.setPosition(auto.rightGripperPosition);//(robot.MID_SERVO - .02)
                sleep(500);
                encoderDrive(auto.DRIVE_SPEED, -4, -4, 15.0);  // S3: Reverse 24 Inches with 15 Sec timeout

        //        robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        //        robot.rightClaw.setPosition(0.0);
                sleep(30000);     // pause for servos to move

                telemetry.addData("Path", "Complete");
                telemetry.update();

            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }


    }


    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
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
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = robot.leftFrontDrive.getCurrentPosition() + (int)(leftInches * auto.COUNTS_PER_INCH);
            newRightFrontTarget = robot.rightFrontDrive.getCurrentPosition() + (int)(rightInches * auto.COUNTS_PER_INCH);
            newLeftRearTarget = robot.leftRearDrive.getCurrentPosition() + (int)(leftInches * auto.COUNTS_PER_INCH);
            newRightRearTarget = robot.rightRearDrive.getCurrentPosition() + (int)(rightInches * auto.COUNTS_PER_INCH);
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
            runtime.reset();
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
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy() &&
                    robot.leftRearDrive.isBusy() && robot.rightRearDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d",
                        newLeftFrontTarget,  newRightFrontTarget, newLeftRearTarget, newRightRearTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                        robot.leftFrontDrive.getCurrentPosition(),
                        robot.rightFrontDrive.getCurrentPosition(),
                        robot.leftRearDrive.getCurrentPosition(),
                        robot.rightRearDrive.getCurrentPosition());
                telemetry.update();
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

    public void encoderTilt(double speed,
                             double rotation,
                             double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = robot.armTilt.getCurrentPosition() + (int) (rotation * 110);
            robot.armTilt.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            robot.armTilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.armTilt.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when ANY motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.armTilt.isBusy())) {
            }

            // Stop all motion;
            robot.armTilt.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.armTilt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

}
