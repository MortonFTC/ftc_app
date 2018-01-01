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

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;

import static org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot.MID_SERVO;

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
public class HardwareMecanum
{

    /* Expansion Hub 2 */

    // Motors
    public DcMotor  leftFrontDrive   = null; // Port 0: REV Robotics HD Hex motor
    public DcMotor  rightFrontDrive  = null; // Port 1: REV Robotics HD Hex motor
    public DcMotor  leftRearDrive    = null; // Port 2: REV Robotics HD Hex motor
    public DcMotor  rightRearDrive   = null; // Port 3: REV Robotics HD Hex motor

    // Servos
    public Servo leftGripper    = null; // Port 0: Servo
    public Servo rightGripper   = null; // Port 1: Servo
    public Servo relicPivot     = null; // Port 2: Servo
    public Servo relicGripper   = null; // Port 3: Servo

    // I2C Bus 0
    public Gyroscope imu    = null; // Port 0: REV Expansion Hub IMU


    /* Expansion Hub 3 */

    // Motors
    public DcMotor  armTilt     = null; // Port 0: REV Robotics HD Hex motor
    public DcMotor  armExtender = null; // Port 1: REV Robotics HD Hex motor
    public DcMotor  armLift     = null; // Port 2: REV Robotics Core Hex motor

    // Servos
    public Servo ballArmServo   = null; // Port 0: Servo
    public Servo swingServo     = null; // Port 1: Servo

    // Digital Devices
    public DigitalChannel maxVertUpDD   = null; // Port 1: Digital Device
    public DigitalChannel maxVertDownDD = null; // Port 3: Digital Device
    public DigitalChannel maxTiltDD     = null; // Port 5: Digital Device

    // I2C Bus 0
    public Gyroscope imu1           = null; // Port 0: REV Expansion Hub IMU
    public ColorSensor colorSensor  = null; // Port 1: REV Color/Range Sensor


    /* constants */
    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    = -1.0 ;
    public static final double ARM_DOWN_POWER  = 1.0 ;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareMecanum(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        // Define and Initialize Motors

        // Expansion Hub (Address: 2)
        leftFrontDrive  = hwMap.get(DcMotor.class, "leftFrontDrive"); // counter-clockwise = forward
        rightFrontDrive = hwMap.get(DcMotor.class, "rightFrontDrive"); // clockwise = forward
        leftRearDrive  = hwMap.get(DcMotor.class, "leftRearDrive"); // counter-clockwise = forward
        rightRearDrive = hwMap.get(DcMotor.class, "rightRearDrive"); // clockwise = forward

        // Expansion Hub (Address: 3)
        armTilt = hwMap.get(DcMotor.class, "armTilt"); //
        armExtender  = hwMap.get(DcMotor.class, "armExtender"); // clockwise = up/out
        armLift = hwMap.get(DcMotor.class, "armLift"); // counter-clockwise = up

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);
        armTilt.setPower(0);
        armExtender.setPower(0);
        armLift.setPower(0);


        // Define and initialize ALL installed servos.

        // Expansion Hub (Address: 2)
        leftGripper = hwMap.get(Servo.class, "leftGripper"); // port 0
        rightGripper = hwMap.get(Servo.class, "rightGripper"); // port 1
        relicPivot  = hwMap.get(Servo.class, "relicPivot"); // port 2
        relicGripper = hwMap.get(Servo.class, "relicGripper"); // port 3

        // Expansion Hub (Address: 3)
        ballArmServo = hwMap.get(Servo.class, "ballArmServo"); // port 0
        swingServo = hwMap.get(Servo.class, "swingServo"); // port 1

        leftGripper.setPosition(MID_SERVO);
        rightGripper.setPosition(MID_SERVO);
        relicPivot.setPosition(MID_SERVO);
        relicGripper.setPosition(MID_SERVO);
        ballArmServo.setPosition(MID_SERVO);
        swingServo.setPosition(MID_SERVO);


        // Define and initialize ALL installed digital devices.

        // Expansion Hub (Address: 3)
        maxVertUpDD = hwMap.get(DigitalChannel.class, "maxVertUpDD");
        maxVertDownDD = hwMap.get(DigitalChannel.class, "maxVertDownDD");
        maxTiltDD = hwMap.get(DigitalChannel.class, "maxTiltDD");


        // Define and initialize ALL installed sensors.

        // Expansion Hub (Address: 2)
        imu = hwMap.get(Gyroscope.class, "imu");

        // Expansion Hub (Address: 3)
        imu1 = hwMap.get(Gyroscope.class, "imu 1");
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor"); // port 1


    }
}