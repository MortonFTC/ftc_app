package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomous Pos 0", group="Auto")
@Disabled
public class AutonomousModePos0 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousMode am = new AutonomousMode(this, 0);
        am.startAutonomousMode();
    }
}
