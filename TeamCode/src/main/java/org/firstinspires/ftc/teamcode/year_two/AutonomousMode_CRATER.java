package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomous Test Pos", group="Auto")
public class AutonomousMode_CRATER extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousMode am = new AutonomousMode(this, 3);
        am.startAutonomousMode();
    }
}

