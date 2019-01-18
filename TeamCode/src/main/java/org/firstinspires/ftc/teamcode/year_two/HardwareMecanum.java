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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
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
    //hookServo SERVO = HUB RIGHT - POS = 1
    //armMidRightIn SERVO = HUB RIGHT - POS = 2
    //armMidRightOut SERVO = HUB RIGHT - POS = 3

public class HardwareMecanum
{

    // Motors
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  leftRearDrive    = null;
    public DcMotor  rightRearDrive   = null;
    public DcMotor armLower = null;
    public DcMotor armUpper = null;

    // Servos
    public Servo door = null;
    public CRServo brush = null;
    public Servo flipperServo = null;

    //Sensors
    public BNO055IMU imu = null;
    public TouchSensor upperArmLimitSwitch = null;
    public ColorSensor colorSensor  = null; // Port 1: REV Color/Range Sensor

    static final double ARM_LOWER_COUNTS_PER_MOTOR_REV = 1300;    // eg: TETRIX Motor Encoder
    static final double ARM_LOWER_MOTOR_REVS_PER_SHAFT_REV = 28;
    static final double WHEELS_COUNTS_PER_SHAFT_REV = 1400;

    final int WHEEL_DIAMETER_INCHES = 6;
    final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCHES * Math.PI;
    final double COUNTS_PER_INCH = WHEELS_COUNTS_PER_SHAFT_REV / WHEEL_CIRCUMFERENCE;

    final double FLIPPER_UP_POSITION = .5;
    final double FLIPPER_DOWN_POSITION = 1;

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

        armLower = hwMap.get(DcMotor.class, "armLower");
        armUpper = hwMap.get(DcMotor.class, "armUpper");

        brush = hwMap.get(CRServo.class, "brush");
        door = hwMap.get(Servo.class, "door");

        flipperServo = hwMap.get(Servo.class, "flipperServo");

        imu = hwMap.get(BNO055IMU.class, "imu");
        //upperArmLimitSwitch = hwMap.get(TouchSensor.class, "upperArmLimitSwitch");


        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
    }
}