package org.firstinspires.ftc.teamcode.year_two.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.year_two.AutonomousMode;

@Disabled
@Autonomous(name="AutonomousMode_CRATER", group="Auto")
public class AutonomousMode_CRATER extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousMode am = new AutonomousMode(this, 2);
        am.startAutonomousMode();
    }
}

