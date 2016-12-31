package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;




@TeleOp(name = "Robot Drive", group = "Drive")

public class RobotDrive extends LinearOpMode {

    Hardware7288 robot           = new Hardware7288();


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);


        waitForStart();





        while (opModeIsActive()) {

            robot.FrontRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x  - gamepad1.right_stick_x);
            robot.FrontLeft .setPower(gamepad1.left_stick_y + gamepad1.left_stick_x  + gamepad1.right_stick_x);
            robot.BackRight.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x  - gamepad1.right_stick_x);
            robot.BackLeft.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x  + gamepad1.right_stick_x);
            //Launcher.setPower(-gamepad1.right_trigger);

            if(gamepad1.left_bumper){
                robot.Intake.setPower(1);
            }else if(gamepad1.right_bumper){
                robot.Intake.setPower(1);
            }else{
                robot.Intake.setPower(0);
            }



        }
    }
}

