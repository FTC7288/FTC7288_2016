package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Tomy on 12/7/2016.
 */

public class Hardware7288 {
    public DcMotor FrontLeft;
    public DcMotor BackLeft;
    public DcMotor FrontRight;
    public DcMotor BackRight;

    public DcMotor Intake;
    public DcMotor Shooter;

    public Servo Red;
    public Servo Blue;

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

        Red = hwmap.servo.get("Red");
        Blue = hwmap.servo.get("Blue");

        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.REVERSE);

        FrontLeft.setPower(0);
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        BackRight.setPower(0);
        Intake.setPower(0);

        Red.setPosition(0);
        Blue.setPosition(0);


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


