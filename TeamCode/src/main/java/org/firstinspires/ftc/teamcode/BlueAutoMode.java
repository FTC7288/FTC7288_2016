package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "Blue Auto", group = "Auto")

public class BlueAutoMode extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) / (WHEEL_DIAMETER_INCHES * 3.1415);

    HardwareAuto7288 robot   = new HardwareAuto7288();
    private ElapsedTime runtime = new ElapsedTime();

    int rangeError;
    int angleError;
    int servoStartingAngle = 0;
    int servoScoringAngle = 90;






    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.Gyro.calibrate();

        sleep(5000);


        waitForStart();


        // speed, front left, front right, rear left,rear right, time out in seconds
        encoderDrive(0.25, 5, 5, 5, 5, 10);



        rangeError = rangeCheck(200);

        if(rangeError != 0){
            //drive the error distatance from the wall

            encoderDrive(0.25,rangeError,rangeError,rangeError,rangeError, 10);
        }


        // angle , speed
        gyroTurnRight(200, 0.25);

        // angle, speed
        gyroTurnLeft(180, 0.25);


        angleError = angleCheck(180);

        if(angleError != 0){
            if(angleError > 1){
                gyroTurnRight((angleError + robot.Gyro.getHeading()), 0.25);
            }else if(angleError < -1){
                gyroTurnLeft((angleError + robot.Gyro.getHeading()), 0.25);
            }
        }


        colorCheck();

        encoderDrive(0.25,5,5,5,10,5);

        servoReset();


        /*Steps for Auto Mode

        Drive X Inches
        Turn X Angle
        Drive X Inches
        Turn X Angle
        Drive X Inches
        Check Distance from wall adjust if needed
        Check Angle Adjust if needed
        Check Color and Activate correct servo
        Drive Forward then back
        Put Severos Up
        Strafe to other beacon
        Check Distance from wall and adjust if needed
        Check Angle adjust if needed
        Check Color and Activate Correct Servo

         */


    }

    public int angleCheck(int Angle){
        int errorAngle;
        if((Angle +1) >= robot.Gyro.getHeading() && (Angle - 1) <= robot.Gyro.getHeading()){
            // Do Nothing
            return 0;
        }else{
            errorAngle = Angle - robot.Gyro.getHeading();
            return errorAngle;
        }
    }

    public void colorCheck(){
        robot.Color.enableLed(true);

        if(robot.Color.red() > 30 && robot.Color.blue() < 10){
            robot.RedServo.setPosition(servoScoringAngle);
        }else if(robot.Color.blue() > 30 && robot.Color.red() < 10){
            robot.BlueServo.setPosition(servoScoringAngle);
        }
    }

    public void servoReset(){
        robot.RedServo.setPosition(servoStartingAngle);
        robot.BlueServo.setPosition(servoStartingAngle);
    }

    public int rangeCheck(int distance){
        int errorDistance;
        if((distance + 1) >= robot.Range.getDistance(DistanceUnit.INCH) && (distance - 1) <= robot.Range.getDistance(DistanceUnit.INCH)){
            // Do Nothing
            return 0;
        }else{
            errorDistance = (int)robot.Range.getDistance(DistanceUnit.INCH) - distance;
            return errorDistance;

        }

    }

    public void gyroTurnRight(int TargetAngle, double speed) throws InterruptedException {
        boolean done;
        done = false;

        while (!done && opModeIsActive()) {
            if ((TargetAngle + 1) >= robot.Gyro.getHeading() && (TargetAngle - 1) <= robot.Gyro.getHeading()) {
                telemetry.addData("Gyro", robot.Gyro.getHeading());
                robot.FrontLeft.setPower(0);
                robot.FrontRight.setPower(0);
                robot.BackLeft.setPower(0);
                robot.BackRight.setPower(0);
                telemetry.update();
            } else {
                telemetry.addData("Gyro", robot.Gyro.getHeading());
                robot.FrontLeft.setPower(speed);
                robot.FrontRight.setPower(-speed);
                robot.BackLeft.setPower(speed);
                robot.BackRight.setPower(-speed);
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
            if ((TargetAngle + 1) >= robot.Gyro.getHeading() && (TargetAngle - 1) <= robot.Gyro.getHeading()) {
                telemetry.addData("Gyro", robot.Gyro.getHeading());
                robot.FrontLeft.setPower(0);
                robot.FrontRight.setPower(0);
                robot.BackLeft.setPower(0);
                robot.BackRight.setPower(0);
                telemetry.update();
                done = true;
            } else {
                telemetry.addData("Gyro", robot.Gyro.getHeading());
                robot.FrontLeft.setPower(-speed);
                robot.FrontRight.setPower(speed);
                robot.BackLeft.setPower(-speed);
                robot.BackRight.setPower(speed);
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
            newFrontLeftTarget = (int) robot.FrontLeft.getCurrentPosition() + (int) (fl * COUNTS_PER_INCH);
            newFrontRightTarget = (int) robot.FrontRight.getCurrentPosition() + (int) (fr * COUNTS_PER_INCH);
            newRearLeftTarget = (int) robot.BackLeft.getCurrentPosition() + (int) (rl * COUNTS_PER_INCH);
            newRearRightTarget = (int) robot.BackRight.getCurrentPosition() + (int) (rr * COUNTS_PER_INCH);

            robot.FrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.FrontRight.setTargetPosition(newFrontRightTarget);
            robot.BackLeft.setTargetPosition(newRearLeftTarget);
            robot.BackRight.setTargetPosition(newRearRightTarget);

            // Turn On RUN_TO_POSITION
            robot.FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.FrontLeft.setPower(Math.abs(speed));
            robot.FrontRight.setPower(Math.abs(speed));
            robot.BackLeft.setPower(Math.abs(speed));
            robot.BackRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.FrontLeft.isBusy() && robot.FrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d :%7d :%7d", newFrontLeftTarget, newFrontRightTarget, newRearLeftTarget, newRearRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d", robot.FrontLeft.getCurrentPosition(), robot.FrontRight.getCurrentPosition(), robot.BackLeft.getCurrentPosition(), robot.BackRight.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            robot.FrontLeft.setPower(0);
            robot.FrontRight.setPower(0);
            robot.BackLeft.setPower(0);
            robot.BackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}








