package org.firstinspires.ftc.teamcode.year_two;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
//@Disabled
@TeleOp(name = "Lamgunana_V3", group = "mortonElements")
public class Lamguana_V3 extends OpMode {

    //TODO Work in progress..
    private DcMotor motorLeft;
    private DcMotor motorRight;

    private Servo armServo1;
    private Servo armServo2;
    private Servo gripperServo1;
    private Servo gripperServo2;

    private BNO055IMU imu = null;

    final double POSITION_CHANGE_RATE = 0.005;                 // sets rate to move servo

    double clawOffset = 0.0;
    final double MID_SERVO_claw = 0.5; //Default servo position


    double armOffset = 0.0;
    final double MID_SERVO_arm = 0.5; //Default servo position

    @Override
    public void init() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".

        imu.initialize(parameters);

        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        armServo1 = hardwareMap.servo.get("armServo1");
        armServo2 = hardwareMap.servo.get("armServo2");
        gripperServo1 = hardwareMap.servo.get("gripperServo1");
        gripperServo2 = hardwareMap.servo.get("gripperServo2");

        telemetry.addData("Mode", "calibrating...");
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
    }

    //@Override
    public void runOpMode() throws InterruptedException {

    }
}



