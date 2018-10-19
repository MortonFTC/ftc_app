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

package org.firstinspires.ftc.teamcode.year_one;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="Mecanum: Teleop Creotion", group="Manual")

public class MecanumTeleop_Creotion extends OpMode{

    /* Declare OpMode members. */
    HardwareMecanum robot       = new HardwareMecanum(); // use the class created to define a robot's hardware
    // could also use HardwarePushbotMatrix class.

    double          clawOffset_glyph  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED_glyph  = 0.02 ;                 // sets rate to move servo

    double          clawOffset_relic  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED_relic  = 0.01 ;                 // sets rate to move servo


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        robot.swingServo.setPosition(0.5);
        robot.ballArmServo.setPosition(.5);

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // DECLARE CONTROLLER VARIABLES

        // pilot controller variables
        double p_left_y; // forward/reverse (left side)
        double p_right_y; // forward/reverse (right side)
        double p_left_x; // left/right shift

        // gunner controller variables
        double g_left_y; // armTilt
        double g_right_y; // armExtender
        boolean g_button_y; // armLift (up)
        boolean g_button_a; // armLift (down)
        boolean g_bumper_left; // leftGripper/rightGripper (open)
        boolean g_bumper_right; // leftGripper/rightGripper (close)
        double g_trigger_left; // relicGripper (open)
        double g_trigger_right; // relicGripper (close)
        boolean g_dpad_left; // relicPivot (turn CC)
        boolean g_dpad_right; // relicPivot (turn C)


        // ASSIGN CONTROLLER INPUTS TO VARIABLES

        // pilot
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        p_left_y = -gamepad1.left_stick_y * .75; // use multiple to adjust speed
        p_right_y = -gamepad1.right_stick_y * .75; // use multiple to adjust speed
        p_left_x = gamepad1.left_stick_x; // full speed

        // gunner
        g_left_y = -gamepad2.left_stick_y;
        g_right_y = gamepad2.right_stick_y;
        g_button_y = gamepad2.y;
        g_button_a = gamepad2.a;
        g_bumper_left = gamepad2.left_bumper;
        g_bumper_right = gamepad2.right_bumper;
        g_trigger_left = gamepad2.left_trigger;
        g_trigger_right = gamepad2.right_trigger;
        g_dpad_left = gamepad2.dpad_left;
        g_dpad_right = gamepad2.dpad_right;


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

        robot.armTilt.setPower(g_left_y);

        telemetry.addData("maxVertUpDD", robot.maxVertUpDD.getState());
        telemetry.addData("maxVertDownDD", robot.maxVertDownDD.getState());
        telemetry.addData("maxTiltDD", robot.maxTiltDD.getState());

        if (!robot.maxTiltDD.getState() && g_right_y > 0) {
            robot.armExtender.setPower(0);
        } else {
            robot.armExtender.setPower(g_right_y);
        }

        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (g_button_y && robot.maxVertUpDD.getState())
            robot.armLift.setPower(robot.ARM_UP_POWER);
        else if (g_button_a && robot.maxVertDownDD.getState())
            robot.armLift.setPower(robot.ARM_DOWN_POWER);
        else
            robot.armLift.setPower(0.0);

        // GLYPH GRIPPER
        // Use gamepad left & right Bumpers to open and close the claw
        if (g_bumper_right)
            clawOffset_glyph += CLAW_SPEED_glyph;
        else if (g_bumper_left)
            clawOffset_glyph -= CLAW_SPEED_glyph;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        if (g_bumper_right || g_bumper_left) {
            clawOffset_glyph = Range.clip(clawOffset_glyph, -0.1, .2);
            robot.leftGripper.setPosition(robot.MID_SERVO + clawOffset_glyph + 0.1);
            robot.rightGripper.setPosition(robot.MID_SERVO - clawOffset_glyph);
        }

        // RELIC PIVOT
        if (g_dpad_left)
            clawOffset_relic += CLAW_SPEED_relic;
        else if (g_dpad_right)
            clawOffset_relic -= CLAW_SPEED_relic;

        if (g_dpad_left || g_dpad_right) {
            clawOffset_relic = Range.clip(clawOffset_relic, -0.5, 0.5);
            robot.relicPivot.setPosition(robot.MID_SERVO + clawOffset_relic);
        }

        // RELIC GRIPPER
        if (g_trigger_left != 0)
            clawOffset_relic += CLAW_SPEED_relic;
        else if (g_trigger_right != 0)
            clawOffset_relic -= CLAW_SPEED_relic;

        if (g_trigger_left != 0 || g_trigger_right != 0) {
            clawOffset_relic = Range.clip(clawOffset_relic, -0.1, 0.5);
            robot.relicGripper.setPosition(robot.MID_SERVO - clawOffset_relic);
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", clawOffset_glyph);
        telemetry.addData("p_left_y",  "%.2f", p_left_y);
        telemetry.addData("p_right_y", "%.2f", p_right_y);
        telemetry.addData("p_left_x", "%.2f", p_left_x);
        telemetry.addData("g_left_y", "%.2f", g_left_y);
        telemetry.addData("g_right_y", "%.2f", g_right_y);
        telemetry.addData("g_button_y", "", g_button_y);
        telemetry.addData("g_button_a", "", g_button_a);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
