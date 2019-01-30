package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomous Test Pos", group="Auto")
@Disabled
public class AutonomousModeTestPos extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousMode am = new AutonomousMode(this, 2);
        am.startAutonomousMode();
    }
}

