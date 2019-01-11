package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static java.lang.Thread.sleep;

//@Disabled
@TeleOp(name = "Lamgunana_V3", group = "Lamguana")
public class Lamguana_V3 extends OpMode {

    //TODO Work in progress.
    private DcMotor motorLeft;
    private DcMotor motorRight;

    private Servo armServo1;
    private Servo armServo2;
    private Servo gripperServo1;
    private Servo gripperServo2;

    private BNO055IMU imu = null;
    private Integer elapsed_time = 0;
    private String strElapsed_Time;

    final double POSITION_CHANGE_RATE = 0.005;                 // sets rate to move servo

    double clawOffset = 0.0;
    final double MID_SERVO_claw = 0.5; //Default servo position


    double armOffset = 0.0;
    final double MID_SERVO_arm = 0.5; //Default servo position

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    @Override
    public void init() {

        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        armServo1 = hardwareMap.servo.get("armServo1");
        armServo2 = hardwareMap.servo.get("armServo2");
        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        gripperServo2 = hardwareMap.servo.get("gripperServo2");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        telemetry.addData(">", "Initializing IMU parameters");    //
        telemetry.update();
        imu.initialize(parameters);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating IMU");    //
        telemetry.update();

        // make sure the gyro is calibrated before continuing
        /*while (!imu.isGyroCalibrated())  {
            try {
                sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            elapsed_time += 100;
        }

        telemetry.addData(">", "Calibration Time: " + elapsed_time.toString());    //
        telemetry.update();
        */
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        /*try {
            sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }*/

        // Set up our telemetry dashboard
        composeTelemetry();
        telemetry.update();
    }

    @Override
    public void loop() {
        motorLeft.setPower(-gamepad1.left_stick_y);
        motorRight.setPower(-gamepad1.right_stick_y);

        //Moves the arms
        if (gamepad1.right_bumper)
            armOffset += Range.clip(POSITION_CHANGE_RATE, -.5, .5);
        else if (gamepad1.left_bumper)
            armOffset -= Range.clip(POSITION_CHANGE_RATE, -.5, .5);

        if (gamepad1.right_trigger > 0)
            clawOffset += Range.clip(POSITION_CHANGE_RATE, -.5, .5);
        else if (gamepad1.left_trigger > 0)
            clawOffset -= Range.clip(POSITION_CHANGE_RATE, -.5, .5);

        composeTelemetry();
        telemetry.update();
    }

    //@Override
    public void runOpMode() throws InterruptedException {

    }

    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}



