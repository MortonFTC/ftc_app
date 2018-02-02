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

/**
 * This class defines methods and properties for Autonomous Op modes
 */
public class AutonomousMecanum
{

    /* constants */
    public static final double     COUNTS_PER_MOTOR_REV    = 1120;    // eg: TETRIX Motor Encoder
    public static final double     DRIVE_GEAR_REDUCTION    = 1.4;     // This is < 1.0 if geared UP
    public static final double     WHEEL_DIAMETER_INCHES   = 6.0;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double     DRIVE_SPEED             = 0.15;
    public static final double     TURN_SPEED              = 0.15;

    public static double          clawOffset_glyph  = 0.0;                  // Servo mid position
    public static final double    CLAW_SPEED_glyph  = 0.02;                 // sets rate to move servo

    public static double          leftGripperPosition  = 0.75;                  // Servo mid position
    public static final double    rightGripperPosition  = 0.375;                 // sets rate to move servo

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code onthe next line, between the double quotes.
     */

    // license key added on 01/28/18 (registered to mortonftc for mecanumRobot) https://developer.vuforia.com/targetmanager/licenseManager/licenseListingDetails
    public String vuforiaLicenseKey_MortonFTC = "AeWrHWf/////AAAAmZG3057lc0JXoVs+HjtHkjZyYL2/IQH4DPGcMKxDXU12F688beSRkSeE6Oz1nH1imNIbBvdwCWFtpqBTu9aqKnlQ9XE3cDLcuUa6/iv0yK3oKy/4p+C1KqltmtvLTda0rgoW8mVcohX38181Apke+iCMjogFT0FHT+3o36MrhYRT03H7Al4Ynqd09uLIGiCXwffq0Ws+YJvWbgbw3Upvjn+Rpbh/xUckxiqFFfU/5j5uCdjMFvUn3YLrLelYAKsaKLKTfMy+OeMbv8wd9By4EjM+A9RB7HKVv3pNZX8fOD9MuSh8y9zV+ZZi+EzcAzJehi9M4mLq7qAmjUgs4qOvtafr6L2dav8Vfw8TarFoD1mk";


    /* Constructor */
    public AutonomousMecanum(){

    }

}