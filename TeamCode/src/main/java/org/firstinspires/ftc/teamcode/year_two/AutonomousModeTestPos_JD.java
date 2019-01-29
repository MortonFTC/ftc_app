package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomous Test Pos_JD", group="Auto")
public class AutonomousModeTestPos_JD extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousMode_JD am = new AutonomousMode_JD(this, 2);
        am.startAutonomousMode();
    }
}

