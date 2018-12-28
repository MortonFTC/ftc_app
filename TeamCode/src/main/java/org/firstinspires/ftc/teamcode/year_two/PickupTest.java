package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="PickupTest", group="Auto")
public class PickupTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareMecanum robot = new HardwareMecanum();

        robot.init(hardwareMap);

        robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("currentPos", robot.armLower.getCurrentPosition());
        telemetry.update();

        robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() + 6000);

        robot.armLower.setPower(.1);
        Thread.sleep(20000);

        telemetry.addData("currentPos", robot.armLower.getCurrentPosition());
        telemetry.update();
    }
}
