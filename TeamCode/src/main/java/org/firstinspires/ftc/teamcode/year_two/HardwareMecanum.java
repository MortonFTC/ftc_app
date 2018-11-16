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

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Creotion (Mecanum drive).
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 */
    //brush SERVO = HUB LEFT - POS = 0
    //armUpperLeft SERVO = HUB LEFT - POS = 1
    //armMidLeftIn SERVO = HUB LEFT - POS = 2
    //armMidLeftOut SERVO = HUB LEFT - POS = 3
    //door SERVO = HUB RIGHT - POS = 0
    //armUpperRight SERVO = HUB RIGHT - POS = 1
    //armMidRightIn SERVO = HUB RIGHT - POS = 2
    //armMidRightOut SERVO = HUB RIGHT - POS = 3

public class HardwareMecanum
{

    // Motors
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  leftRearDrive    = null;
    public DcMotor  rightRearDrive   = null;
    public DcMotor armLowerLeft = null;
    public DcMotor armLowerRight = null;

    // Servos
    public Servo armMidLeftOut = null;
    public Servo armMidLeftIn = null;
    public Servo armMidRightOut = null;
    public Servo armMidRightIn = null;
    public Servo armUpperRight = null;
    public Servo armUpperLeft = null;

    public Servo singleMastDrive = null;
    public Servo hook = null;
    public Servo door = null;
    public Servo doubleMastDrive1 = null;
    public Servo doubleMastDrive2 = null;

    public CRServo brush = null;

    public BNO055IMU imu = null;

    //Misc
    public ColorSensor colorSensor  = null; // Port 1: REV Color/Range Sensor


    /* constants
    public static final double MID_SERVO       =  0.5;
    public static final double ARM_UP_POWER    = -1.0;
    public static final double ARM_DOWN_POWER  = 1.0;
*/
    static final double ARM_COUNTS_PER_MOTOR_REV    = 1300;    // eg: TETRIX Motor Encoder
    static final double ARM_MOTOR_REVS_PER_SHAFT_REV = 28;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hwMap) {
        this.hwMap = hwMap;

        // Define and Initialize Motors
        leftFrontDrive  = hwMap.get(DcMotor.class, "leftFrontDrive"); // counter-clockwise = forward
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive"); // clockwise = forward
        leftRearDrive  = hwMap.get(DcMotor.class, "leftRearDrive"); // counter-clockwise = forward
        rightRearDrive = hwMap.get(DcMotor.class, "rightRearDrive"); // clockwise = forward

        //armLowerLeft = hwMap.get(DcMotor.class, "armLowerLeft");
        armLowerRight = hwMap.get(DcMotor.class, "armLowerRight");

        armMidLeftOut = hwMap.get(Servo.class, "armMidLeftOut");
        armMidLeftIn = hwMap.get(Servo.class, "armMidLeftIn");
        armMidRightOut = hwMap.get(Servo.class, "armMidRightOut");
        armMidRightIn = hwMap.get(Servo.class, "armMidRightIn");

        armUpperRight = hwMap.get(Servo.class, "armUpperRight");
        armUpperLeft = hwMap.get(Servo.class, "armUpperLeft");

        //singleMastDrive = hwMap.get(Servo.class, "singleMastDrive");
        //doubleMastDrive1 = hwMap.get(Servo.class, "doubleMastDrive1");
        //doubleMastDrive2 = hwMap.get(Servo.class, "doubleMastDrive2");
        //hook = hwMap.get(Servo.class, "hook");
        brush = hwMap.get(CRServo.class, "brush");
        door = hwMap.get(Servo.class, "door");

        //colorSensor = hwMap.get(ColorSensor.class, "colorSensor");

        imu = hwMap.get(BNO055IMU.class, "imu");

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
    }
}