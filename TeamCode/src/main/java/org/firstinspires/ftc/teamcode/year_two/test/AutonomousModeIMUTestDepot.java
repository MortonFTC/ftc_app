package org.firstinspires.ftc.teamcode.year_two.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.year_two.AutonomousMode;

@Autonomous(name="Autonomous Pos 10", group="Auto")
public class AutonomousModeIMUTestDepot extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousMode am = new AutonomousMode(this, 10);
        am.startAutonomousMode();
    }
}
