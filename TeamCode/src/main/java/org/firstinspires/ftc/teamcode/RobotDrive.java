package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.Target;


@Autonomous(name = "Auto", group = "Auto")

public class RobotDrive extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor RearLMotor;
    DcMotor RearRMotor;
    DcMotor FrontLMotor;
    DcMotor FrontRMotor;


    GyroSensor gyro;



    @Override
    public void runOpMode() throws InterruptedException {
        FrontLMotor = hardwareMap.dcMotor.get("FL");
        FrontRMotor = hardwareMap.dcMotor.get("FR");
        RearLMotor = hardwareMap.dcMotor.get("RL");
        RearRMotor = hardwareMap.dcMotor.get("RR");
        gyro = hardwareMap.gyroSensor.get("gyro");


        FrontLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontRMotor.setDirection(DcMotor.Direction.REVERSE);
        RearRMotor.setDirection(DcMotor.Direction.REVERSE);

        gyro.calibrate();

        sleep(5000);


        waitForStart();


        // speed, front left, front right, rear left,rear right, time out in seconds
        encoderDrive(0.25, 5, 5, 5, 5, 10);

        // angle , speed
        gyroTurnRight(200, 0.25);

        // angle, speed
        gyroTurnLeft(180, 0.25);


    }

    public void gyroTurnRight(int TargetAngle, double speed) throws InterruptedException {
        boolean done;
        done = false;

        while (!done && opModeIsActive()) {
            if ((TargetAngle + 1) >= gyro.getHeading() && (TargetAngle - 1) <= gyro.getHeading()) {
                telemetry.addData("Gyro", gyro.getHeading());
                FrontLMotor.setPower(0);
                FrontRMotor.setPower(0);
                RearLMotor.setPower(0);
                RearRMotor.setPower(0);
                telemetry.update();
            } else {
                telemetry.addData("Gyro", gyro.getHeading());
                FrontLMotor.setPower(speed);
                FrontRMotor.setPower(-speed);
                RearLMotor.setPower(speed);
                RearRMotor.setPower(-speed);
                telemetry.update();
                done = true;
            }
            idle();
        }
    }

    public void gyroTurnLeft(int TargetAngle, double speed) throws InterruptedException {
        boolean done;
        done = false;

        while (!done&& opModeIsActive()) {
            if ((TargetAngle + 1) >= gyro.getHeading() && (TargetAngle - 1) <= gyro.getHeading()) {
                telemetry.addData("Gyro", gyro.getHeading());
                FrontLMotor.setPower(0);
                FrontRMotor.setPower(0);
                RearLMotor.setPower(0);
                RearRMotor.setPower(0);
                telemetry.update();
                done = true;
            } else {
                telemetry.addData("Gyro", gyro.getHeading());
                FrontLMotor.setPower(-speed);
                FrontRMotor.setPower(speed);
                RearLMotor.setPower(-speed);
                RearRMotor.setPower(speed);
                telemetry.update();
            }
            idle();
        }
    }


    public void encoderDrive(double speed, double fl, double fr, double rl, double rr, double timeoutS) throws InterruptedException {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newRearLeftTarget;
        int newRearRightTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = (int) FrontLMotor.getCurrentPosition() + (int) (fl * COUNTS_PER_INCH);
            newFrontRightTarget = (int) FrontRMotor.getCurrentPosition() + (int) (fr * COUNTS_PER_INCH);
            newRearLeftTarget = (int) RearLMotor.getCurrentPosition() + (int) (rl * COUNTS_PER_INCH);
            newRearRightTarget = (int) RearRMotor.getCurrentPosition() + (int) (rr * COUNTS_PER_INCH);

            FrontLMotor.setTargetPosition(newFrontLeftTarget);
            FrontRMotor.setTargetPosition(newFrontRightTarget);
            RearLMotor.setTargetPosition(newRearLeftTarget);
            RearRMotor.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            FrontLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RearRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            FrontLMotor.setPower(Math.abs(speed));
            FrontRMotor.setPower(Math.abs(speed));
            RearLMotor.setPower(Math.abs(speed));
            RearRMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (FrontLMotor.isBusy() && FrontRMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d", FrontLMotor.getCurrentPosition(), FrontRMotor.getCurrentPosition(), RearLMotor.getCurrentPosition(), RearRMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            FrontLMotor.setPower(0);
            FrontRMotor.setPower(0);
            RearLMotor.setPower(0);
            RearRMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            FrontLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RearLMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RearRMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}








