package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;




public class HardwareAuto7288 {

    public DcMotor FrontLeft;
    public DcMotor BackLeft;
    public DcMotor FrontRight;
    public DcMotor BackRight;

    public DcMotor Intake;
    public DcMotor Shooter;

    public Servo RedServo;
    public Servo BlueServo;

    public ModernRoboticsI2cRangeSensor Range;
    public ModernRoboticsI2cGyro Gyro;
    public ColorSensor Color;

    private ElapsedTime period = new ElapsedTime();

    HardwareMap hwmap;

    public void init(HardwareMap ahwMap) {
        hwmap = ahwMap;


        FrontLeft = hwmap.dcMotor.get("FL");
        BackLeft = hwmap.dcMotor.get("BL");
        FrontRight = hwmap.dcMotor.get("FR");
        BackRight = hwmap.dcMotor.get("BR");

        Intake = hwmap.dcMotor.get("Intake");
        Shooter = hwmap.dcMotor.get("Shooter");

        Gyro =  (ModernRoboticsI2cGyro) hwmap.get("Gyro");
        Range = (ModernRoboticsI2cRangeSensor)hwmap.get("Range");
        Color = hwmap.colorSensor.get("Color");

        RedServo = hwmap.servo.get("RedServo");
        BlueServo = hwmap.servo.get("BlueServo");


        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
        Intake.setPower(0);

        RedServo.setPosition(0);
        BlueServo.setPosition(0);

    }



    public void waitForTick(long periodMs) throws InterruptedException {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
