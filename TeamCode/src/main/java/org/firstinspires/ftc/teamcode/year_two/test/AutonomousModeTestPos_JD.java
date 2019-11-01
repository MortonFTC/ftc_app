package org.firstinspires.ftc.teamcode.year_two.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomous Test Pos_JD", group="Auto")
//@Disabled
public class AutonomousModeTestPos_JD extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
                AutonomousMode_JD am = new AutonomousMode_JD(this, 99);
        am.startAutonomousMode();
    }
}

