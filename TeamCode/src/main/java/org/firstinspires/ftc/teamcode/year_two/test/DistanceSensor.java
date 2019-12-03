package org.firstinspires.ftc.teamcode.year_two.test;

import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor: Distance", group = "Distance")
//@Disabled

public class DistanceSensor  extends LinearOpMode {
    private Rev2mDistanceSensor sensorDistance;

    public void runOpMode() throws InterruptedException {

        sensorDistance = hardwareMap.get(Rev2mDistanceSensor.class, "sensorDistance");
        sensorDistance.initialize();

        try {
            getDistance(); // actually execute the sample
        } finally {
            telemetry.addData("Error: ",sensorDistance);
            telemetry.update();
        }
    }

    private void getDistance() {
        double distance = sensorDistance.getDistance(DistanceUnit.INCH);

        telemetry.addData("Distance (INCH)", distance);
        telemetry.update();

        /*if (distance < 10)
        {
            robot.leftCollector.setPower(1);
        }
        else
        {
            robot.leftCollector.setPower(0);
        }
         */
    }
}
