package org.firstinspires.ftc.teamcode.year_two.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.year_two.HardwareMecanum;

import static java.lang.Thread.sleep;

@Autonomous(name="ArmEncoderTest", group="Auto")
@Disabled
public class ArmEncoderTest extends LinearOpMode {

    HardwareMecanum robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareMecanum();

        robot.init(hardwareMap);

        waitForStart();

        robot.armLower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armUpper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armLower.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armUpper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Reach backwards
        //robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() + 6500);

        robot.armLower.setTargetPosition(robot.armLower.getCurrentPosition() - 6000);
        robot.armUpper.setTargetPosition(robot.armUpper.getCurrentPosition() - 20000);


        robot.armLower.setPower(.2);
        robot.armUpper.setPower(1);

        sleep(15000);
    }
}
