package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.year_two.AutonomousMode;

@Autonomous(name="Autonomous IMU Depot", group="Auto")
@Disabled
public class AutonomousModeIMUDepot extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousMode am = new AutonomousMode(this, 10);
        am.startAutonomousMode();
    }
}
