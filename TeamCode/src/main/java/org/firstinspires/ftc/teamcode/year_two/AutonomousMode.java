package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutonomousMode {
    HardwareMap hwMap;
    int position;

    public AutonomousMode(HardwareMap hwMap, int position)
    {
        this.hwMap = hwMap;
        this.position = position;
    }

    public void init()
    {

    }
}
