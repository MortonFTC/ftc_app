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

package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file provides basic Telop driving for a Mecanum robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Mecanum hardware class to define the devices on the robot.
 * All device access is managed through the HardwareMecanum class.
 *
 * This particular OpMode executes a basic Crab Steer Teleop for a Mecanum
 * TODO: add comments about steering
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
//@Disabled
@TeleOp(name="Teleop 2018", group="Manual")

public class Teleop2018 extends OpMode {

    HardwareMecanum robot = new HardwareMecanum();

    public final double SERVO_RATE_OF_CHANGE = 1/280.0;
    public final int ARM_LOWER_RATE_OF_CHANGE = 28*40;
    public final double BRUSH_SPEED = 1; //TODO
    public final double DOOR_START_POS = 0.45D;
    public final double DOOR_OPEN_POS = DOOR_START_POS + 90/280.0;

    public int armLowerOffset = 0; //This is the offset from the starting position of the motor, in units of encoder counts.
    //public boolean doorClosed = true;

    //TODO Set one armLower motor to go in reverse.


    //public final double ARM_MID_MAX_POS = 1; //TODO
    //public final double ARM_MID_MIN_POS = 1;

    //public final double ARM_UPPER_MAX_POS = 1; //TODO
    //public final double ARM_UPPER_MIN_POS = 1;

    @Override
    public void init() {
        robot.init(hardwareMap);
        /*
        //robot.armMidLeftOut.setPosition(0.931);
        //robot.armMidLeftIn.setPosition(0.082);
        robot.armMidLeftOut.setPosition(0);
        robot.armMidLeftIn.setPosition(1);
        //robot.armMidRightOut.setPosition(0.047);
        //robot.armMidRightIn.setPosition(0.966);
        robot.armMidRightOut.setPosition(1);
        robot.armMidRightIn.setPosition(0);
        //robot.hookServo.setPosition(0.0517);
        //robot.armUpperLeft.setPosition(0.9742);
        */
        //robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       // robot.door.setPosition(DOOR_START_POS);
    }

    @Override
    public void loop() {
        // pilot controller variables
        double p_left_y; // forward/reverse (left side)
        double p_right_y; // forward/reverse (right side)
        double p_left_x; // left/right shift
        boolean p_left_bumper;
        boolean p_right_bumper;

        // gunner controller variables
        double g_left_y; // armTilt
        double g_right_y; // armExtender
        boolean g_button_y; // armLift (up)
        boolean g_button_b; // armLift (down)boolean g_button_y; // armLift (up)
        boolean g_button_a; // armLift (down)
        boolean g_button_x;
        boolean g_bumper_left; // leftGripper/rightGripper (open)
        boolean g_bumper_right; // leftGripper/rightGripper (close)
        double g_trigger_left; // relicGripper (open)
        double g_trigger_right; // relicGripper (close)
        boolean g_dpad_up; // relicPivot (turn CC)
        boolean g_dpad_down; // relicPivot (turn C)


        // ASSIGN CONTROLLER INPUTS TO VARIABLES

        // pilot
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        p_left_y = -gamepad1.left_stick_y * .75; // use multiple to adjust speed
        p_right_y = -gamepad1.right_stick_y * .75; // use multiple to adjust speed
        p_left_x = gamepad1.left_stick_x; // full speed
        p_left_bumper = gamepad1.left_bumper;
        p_right_bumper = gamepad1.right_bumper;

        // gunner
        g_left_y = gamepad2.left_stick_y;
        //g_left_y = -gamepad2.left_stick_y;
        g_right_y = gamepad2.right_stick_y;
        g_button_y = gamepad2.y;
        g_button_a = gamepad2.a;
        g_button_x = gamepad2.x;
        g_button_b = gamepad2.b;
        g_bumper_left = gamepad2.left_bumper;
        g_bumper_right = gamepad2.right_bumper;
        g_trigger_left = gamepad2.left_trigger;
        g_trigger_right = gamepad2.right_trigger;
        g_dpad_up = gamepad2.dpad_up;
        g_dpad_down = gamepad2.dpad_down;

        // DECLARE MOTOR VARIABLES

        double lFrontDrive = 0;
        double rFrontDrive = 0;
        double lRearDrive = 0;
        double rRearDrive = 0;

        if (Math.abs(p_left_y) > 0 || Math.abs(p_right_y) > 0) { // forward/reverse
            lFrontDrive = -p_left_y;
            lRearDrive = -p_left_y;
            rFrontDrive = p_right_y;
            rRearDrive = p_right_y;
        } else if (Math.abs(p_left_x) > 0) { // left/right shift
            lFrontDrive = -p_left_x;
            lRearDrive = p_left_x;
            rFrontDrive = -p_left_x;
            rRearDrive = p_left_x;
        }

        robot.leftFrontDrive.setPower(lFrontDrive);
        robot.rightFrontDrive.setPower(rFrontDrive);
        robot.leftRearDrive.setPower(lRearDrive);
        robot.rightRearDrive.setPower(rRearDrive);

        //Preset positions open, mid, and closed.
        if (g_button_x) {
            setPresetPosition(PresetLocation.CLOSED);
        }
       // else if (g_button_y) {
       //    //setPresetPosition(PresetLocation.OPEN);
        //}
        //else if (g_button_b) {
        //    //setPresetPosition(PresetLocation.MID);
        //}
        //Pick up and drop preset positions.
        //if (p_left_bumper)
            //pickUpPosition(); TODO Uncomment
       // else if (p_right_bumper)
            //dropPosition();

        //Controlling the arms.
        if (g_dpad_up) {
            /*telemetry.addData("armMidLeftOut", robot.armMidLeftOut.getPosition());
            telemetry.addData("armMidLeftIn", robot.armMidLeftIn.getPosition());
            telemetry.addData("armMidRightIn", robot.armMidRightIn.getPosition());
            telemetry.addData("armMidRightOut", robot.armMidRightOut.getPosition());
            telemetry.update();

            if (robot.armMidLeftOut.getPosition() - SERVO_RATE_OF_CHANGE < 0 || robot.armMidLeftIn.getPosition() + SERVO_RATE_OF_CHANGE > 1 || robot.armMidRightOut.getPosition() + SERVO_RATE_OF_CHANGE > 1 || robot.armMidRightIn.getPosition() - SERVO_RATE_OF_CHANGE < 0) return;

            robot.armMidLeftOut.setPosition(robot.armMidLeftOut.getPosition() - SERVO_RATE_OF_CHANGE);
            robot.armMidLeftIn.setPosition(robot.armMidLeftIn.getPosition() + SERVO_RATE_OF_CHANGE);
            robot.armMidRightOut.setPosition(robot.armMidRightOut.getPosition() + SERVO_RATE_OF_CHANGE);
            robot.armMidRightIn.setPosition(robot.armMidRightIn.getPosition() - SERVO_RATE_OF_CHANGE);
            */

        }
        else if (g_dpad_down) {
            /*telemetry.addData("armMidLeftOut", robot.armMidLeftOut.getPosition());
            telemetry.addData("armMidLeftIn", robot.armMidLeftIn.getPosition());
            telemetry.addData("armMidRightIn", robot.armMidRightIn.getPosition());
            telemetry.addData("armMidRightOut", robot.armMidRightOut.getPosition());
            telemetry.update();

            if (robot.armMidLeftOut.getPosition() + SERVO_RATE_OF_CHANGE > .991 || robot.armMidLeftIn.getPosition() - SERVO_RATE_OF_CHANGE < .009 || robot.armMidRightOut.getPosition() - SERVO_RATE_OF_CHANGE < .047 || robot.armMidRightIn.getPosition() + SERVO_RATE_OF_CHANGE > .966) return;

            robot.armMidLeftOut.setPosition(robot.armMidLeftOut.getPosition() + SERVO_RATE_OF_CHANGE);
            robot.armMidLeftIn.setPosition(robot.armMidLeftIn.getPosition() - SERVO_RATE_OF_CHANGE);
            robot.armMidRightOut.setPosition(robot.armMidRightOut.getPosition() - SERVO_RATE_OF_CHANGE);
            robot.armMidRightIn.setPosition(robot.armMidRightIn.getPosition() + SERVO_RATE_OF_CHANGE);
            */
        }

        if (g_left_y != 0 & Math.abs(g_left_y) >= .3) { //Deadzone for joystick
            if (-g_left_y > .3 ) { //If joystick is up.
                //robot.armLowerLeft.setTargetPosition(robot.armLowerLeft.getCurrentPosition() - ARM_LOWER_RATE_OF_CHANGE);
                robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() - ARM_LOWER_RATE_OF_CHANGE);
                armLowerOffset -= ARM_LOWER_RATE_OF_CHANGE;
            }
            else if (-g_left_y < -.3) { //If joystick is down.
                //robot.armLowerLeft.setTargetPosition(robot.armLowerLeft.getCurrentPosition() + ARM_LOWER_RATE_OF_CHANGE); //(1800 * 28) / 150 loops per second / 12 = 12 seconds for full arm rotation.
                robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() + ARM_LOWER_RATE_OF_CHANGE);
                armLowerOffset += ARM_LOWER_RATE_OF_CHANGE;
            }

            telemetry.addData("armLowerOffset", armLowerOffset);

            //robot.armLowerLeft.setPower(.05); //TODO If arm keeps moving after joystick released, increase this value.
            robot.armLower.setPower(1);
        }

        if (g_button_y) {
            robot.hookServo.setPosition(robot.hookServo.getPosition() + SERVO_RATE_OF_CHANGE);
            //robot.armUpperLeft.setPosition(robot.armUpperLeft.getPosition() - SERVO_RATE_OF_CHANGE);
        }
        if (g_button_a) {
            robot.hookServo.setPosition(robot.hookServo.getPosition() - SERVO_RATE_OF_CHANGE);
            //robot.armUpperLeft.setPosition(robot.armUpperLeft.getPosition() + SERVO_RATE_OF_CHANGE);
        }

        //Activating the brushes.
        /*if (g_trigger_right > 0)
            robot.brush.setPower(BRUSH_SPEED);
        else if (g_trigger_left > 0) //Reverse
            robot.brush.setPower(-BRUSH_SPEED);
        else robot.brush.setPower(0);

        if (g_bumper_right) {
            robot.door.setPosition(DOOR_OPEN_POS);
        }
        else {
            robot.door.setPosition(DOOR_START_POS);
        }
        */
    }

    public enum PresetLocation {
        CLOSED, OPEN, MID, PICK_UP, DROP
    }

    public void setPresetPosition(PresetLocation location) {
        //TODO Add support for threads.
        Integer armLowerDesiredPosition = null; //In units of encoder counts from starting position

        Double armMidLeftOut = null; //In units of 0 to 1
        Double armMidLeftIn = null;
        Double armMidRightOut = null;
        Double armMidRightIn = null;
        Double armUpperLeft = null;
        Double armUpperRight = null;

        switch (location) {
            case CLOSED:
                armMidLeftOut = 0.931D;
                armMidLeftIn = 0.082D;
                armMidRightOut = 0.966D;
                armMidRightIn = 0.047D;
                armUpperLeft = 0.9742D;
                armUpperRight = 0.0517D;
                break;
            case OPEN:
                armMidLeftOut = .3571D;
                armMidLeftIn = .6429D;
                armMidRightOut = .6429D;
                armMidRightIn = .3571D;
                armUpperLeft = 0.9742D;
                armUpperRight = 0.0517D;
                break;
            case MID:
                armMidLeftOut = .0714D;
                armMidLeftIn = .9286D;
                armMidRightOut = .9286D;
                armMidRightIn = .0714D;
                armUpperLeft = 0.9742D;
                armUpperRight = 0.0517D;
                break;
            case PICK_UP:
                //TODO
                break;
            case DROP:
                //TODO
                break;
        }

        if (armLowerDesiredPosition != null) {
            robot.armLower.setTargetPosition(-(armLowerDesiredPosition - armLowerOffset));
            robot.armLower.setPower(.05);
        }

        /*
        if (armMidLeftOut != null)
            robot.armMidLeftOut.setPosition(armMidLeftOut);
        if (armMidLeftIn != null)
            robot.armMidLeftIn.setPosition(armMidLeftIn);
        if (armMidRightOut != null)
            robot.armMidRightOut.setPosition(armMidRightOut);
        if (armMidRightIn != null)
            robot.armMidRightIn.setPosition(armMidRightIn);
        if (armUpperRight != null)
            robot.hookServo.setPosition(armUpperRight);
        //if (armUpperLeft != null)
           // robot.armUpperLeft.setPosition(armUpperLeft);
           */
    }
}


